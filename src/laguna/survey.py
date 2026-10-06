"""Multi-pass surveys: cover an area, or repeat a line.

Everything in laguna is single-pass. ``TopographicProfiler.scan()`` runs one
transect, ``GocatorScanner.scan_with_gantry()`` runs one traverse. Anything
larger than the sensor's field of view, or repeated over the course of an
experiment, has been a hand-written loop in a script.

Two shapes cover almost all of it:

:class:`Tile` — tile a region with parallel passes, because the Gocator's
laser fan is ~2 m across but a flume bed is wider, and one pass only
captures a strip.

:class:`Traverse` — re-run the same line, optionally with a different
instrument. This is what turns "scan the bed hourly" or "compare the OD2000
and the WTT12L over the same ground" into a plan rather than a script.

Both are expressed in **experiment-frame coordinates** and resolved through
:class:`~laguna.frames.FrameRegistry`, so a survey is specified in terms of
the flume rather than the robot — and the same region description works for
whichever instrument is doing the measuring, since the registry compensates
for where each one is mounted.

Both are also **pure planners**. They emit a list of passes and hold no
hardware; execution is a separate step that can be checkpointed, resumed,
or simply printed and inspected before anything moves. A survey is often the
longest single operation an experiment performs, so being able to see the
plan first matters.

Two speeds, one shorthand
--------------------------
Every plan distinguishes ``scan_speed`` (the measuring traverse — often
constrained by the sensor) from ``travel_speed`` (repositioning to a pass's
start, which has no such constraint and can usually go faster). Pass
``speed=`` instead to set both to the same value.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np

from .robot.macron.commands import ramp_distance_mm, ramp_time_s
from .robot.motion_arbiter import DEFAULT_ARBITER

logger = logging.getLogger(__name__)

#: Checkpoint metadata key holding the survey's geometry fingerprint.
_FINGERPRINT_KEY = "survey_fingerprint"


class SurveyCheckpointMismatch(RuntimeError):
    """A checkpoint on disk describes a different survey than the one being run.

    Resuming anyway would skip passes the *new* geometry never ran, because a
    checkpoint only records pass indices. Nothing has been deleted.
    """


#: Cartesian axis name -> index into a 3-component [x, y, z] vector. Shared
#: by Tile and SurveyRunner rather than each keeping its own copy.
_AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


@dataclass
class Pass:
    """One traverse: measure from `measure_start` to `end`, experiment-frame mm.

    The gantry may be commanded from further back than the measuring
    starts (`start` vs. `cruise_start`) — see those attributes below.

    Attributes:
        index: Position in the plan, 0-based. Used as the checkpoint id, so
            an interrupted survey resumes at the right pass.
        start: [x, y, z] where the gantry is *commanded* to begin moving —
            the ramp start. Ordinarily identical to `cruise_start`; a
            :class:`Tile` with `accel_mm_s2` set moves this back along the
            travel axis so the axis is already at `scan_speed` by the time
            it reaches `cruise_start`, instead of a step change.
        end: [x, y, z] where it should finish.
        cruise_start: [x, y, z] where the measuring point should actually
            begin — the swath boundary a plan asked for. None means it's
            identical to `start` (the common case: no ramp lead-in was
            computed). This, not `start`, is what a scan's dead-reckoned
            surface must be anchored to; see `SurveyRunner._run_pass()`
            and `GocatorScanner`'s `cruise_start_mm` parameter.
        instrument: Which instrument measures this pass.
        axis: Experiment-frame axis the traverse runs along, resolved from
            the geometry. ``SurveyRunner`` maps it to the gantry axis that
            actually moves through the experiment frame's rotation (the two
            differ whenever that rotation is 90 or 270 degrees).
        scan_speed: Measuring traverse speed, if the plan fixed one.
        travel_speed: Speed for the pre-scan repositioning move to `start`,
            if it should differ from `scan_speed`. Defaults to `scan_speed`
            when omitted — repositioning at scan speed is slower than
            necessary but always safe.
        label: Human-readable description, for logs and progress.
        edge_align: If True, `start`/`end`'s `step_axis` coordinate names
            the swath's near edge rather than the instrument's usual
            (centerline) measurement point — see :class:`Tile`. Set by
            :class:`Tile`, left False by :class:`Traverse` (a single line
            has no swath edge to align).
        step_axis: Axis the swath is offset across, required when
            `edge_align` is True — see :attr:`Tile.step_axis`.
        overrun_end: [x, y, z] where the gantry is *commanded* to stop, past
            `end` along the travel axis — the mirror image of the lead-in.
            The axis decelerates over its last ``v^2 / 2a`` before this
            point, so stopping exactly at `end` would put the end of the
            swath in the slowdown, where a sensor writing rows at a fixed
            spacing misplaces them. Moving the stop past `end` by that
            distance keeps the whole measured stretch (`measure_start` to
            `end`) in the constant-speed slew. None means no lead-out.
    """

    index: int
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    cruise_start: Optional[Tuple[float, float, float]] = None
    instrument: str = "gocator"
    axis: str = "X"
    scan_speed: Optional[float] = None
    travel_speed: Optional[float] = None
    label: str = ""
    edge_align: bool = False
    step_axis: Optional[str] = None
    overrun_end: Optional[Tuple[float, float, float]] = None

    @property
    def commanded_end(self) -> Tuple[float, float, float]:
        """`overrun_end` if set, else `end` — where the gantry is told to stop.

        Returns:
            The point the move is commanded to, which lies past `end` when a
            lead-out was planned so the axis is still at speed at `end`.
        """
        return self.overrun_end if self.overrun_end is not None else self.end

    @property
    def measure_start(self) -> Tuple[float, float, float]:
        """`cruise_start` if set, else `start` — where measuring truly begins.

        Returns:
            The swath boundary a plan asked for, regardless of whether a
            ramp lead-in moved `start` behind it.
        """
        return self.cruise_start if self.cruise_start is not None else self.start

    @property
    def length_mm(self) -> float:
        """Get the measuring traverse distance in mm.

        Returns:
            Distance from `measure_start` to `end` — the swath itself, not
            including any ramp lead-in before `measure_start`.
        """
        return math.dist(self.measure_start, self.end)

    def duration_s(self, scan_speed: Optional[float] = None) -> Optional[float]:
        """Get the traverse duration at a given scan speed.

        Args:
            scan_speed: Speed in mm/s. If not provided, uses the pass's
                configured scan_speed.

        Returns:
            Duration in seconds, or None if no speed is available.
        """
        rate = scan_speed or self.scan_speed
        if not rate:
            return None
        return self.length_mm / rate

    def to_dict(self) -> Dict[str, Any]:
        """Export pass to a dictionary.

        Returns:
            Dict with pass index, start/end, instrument, axis, speeds, and label.
        """
        return {
            "index": self.index,
            "start": list(self.start),
            "end": list(self.end),
            "cruise_start": list(self.cruise_start) if self.cruise_start is not None else None,
            "instrument": self.instrument,
            "axis": self.axis,
            "scan_speed": self.scan_speed,
            "travel_speed": self.travel_speed,
            "label": self.label,
            "overrun_end": list(self.overrun_end) if self.overrun_end is not None else None,
            "edge_align": self.edge_align,
            "step_axis": self.step_axis,
        }


class Survey:
    """Base class for survey planners that produce a list of passes."""

    def passes(self) -> List[Pass]:  # pragma: no cover - interface
        """Get the planned passes.

        Returns:
            List of Pass objects.
        """
        raise NotImplementedError

    def fingerprint(self) -> str:
        """Hash of the geometry this survey measures, for checkpoint safety.

        A checkpoint records only pass *indices*; this is what lets a resume
        notice that index 3 now means a different strip of bed. It covers what
        each pass measures — instrument, travel axis, measuring start and end,
        and swath alignment — and deliberately **not**:

        - speeds, labels: they don't change which ground a completed pass
          covered;
        - the commanded ramp start (``Pass.start`` when a lead-in was added):
          it follows the axis's accel, which can be re-read between runs, yet
          a pass already measured from a different run-up still covered the
          same swath.

        Returns:
            A short hex digest; equal exactly when the measured geometry is.
        """
        def _pt(v: Sequence[float]) -> List[float]:
            return [round(float(c), 6) for c in v]

        rows = [
            {
                "instrument": p.instrument,
                "axis": p.axis,
                "start": _pt(p.measure_start),
                "end": _pt(p.end),
                "edge_align": bool(p.edge_align),
                "step_axis": p.step_axis,
            }
            for p in self.passes()
        ]
        blob = json.dumps(rows, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(blob).hexdigest()[:16]

    def __iter__(self) -> Iterator[Pass]:
        """Iterate over the passes."""
        return iter(self.passes())

    def __len__(self) -> int:
        """Get the number of passes.

        Returns:
            Number of passes in this survey.
        """
        return len(self.passes())

    def duration_s(self, scan_speed: Optional[float] = None) -> Optional[float]:
        """Get total traverse time, ignoring repositioning between passes.

        A lower bound, as the gantry repositioning and sensor settle time are
        not included.

        Args:
            scan_speed: Speed in mm/s. If not provided, uses each pass's
                configured scan speed.

        Returns:
            Total duration in seconds, or None if any pass lacks a speed.
        """
        totals = [p.duration_s(scan_speed) for p in self.passes()]
        if any(t is None for t in totals):
            return None
        return sum(totals)

    def describe(self) -> str:
        """Get a multi-line human-readable summary.

        Returns:
            Formatted string describing all passes in the survey.
        """
        rows = [f"{len(self)} passes"]
        for p in self.passes():
            ramp_note = (
                f" (ramp from {tuple(round(v, 1) for v in p.start)})"
                if p.cruise_start is not None and p.cruise_start != p.start
                else ""
            )
            if p.overrun_end is not None:
                ramp_note += f" (stops at {tuple(round(v, 1) for v in p.overrun_end)})"
            rows.append(
                f"  [{p.index}] {p.label or p.instrument}: "
                f"{tuple(round(v, 1) for v in p.measure_start)} -> "
                f"{tuple(round(v, 1) for v in p.end)} "
                f"along {p.axis} ({p.length_mm:.0f} mm){ramp_note}"
            )
        return "\n".join(rows)


@dataclass
class Tile(Survey):
    """Cover a rectangular region with parallel passes.

    The Gocator images a strip as wide as its active area, so a bed wider
    than that needs several passes side by side. This lays them out.

    Args:
        origin: [x, y, z] corner of the region, experiment frame.
        length_mm: Extent along the traverse axis.
        width_mm: Extent across it — the direction passes step along.
        swath_mm: How wide one pass actually images. For the Gocator this is
            the active area's width, **not** the sensor's full field of view;
            read it from ``get_active_area()["width_mm"]`` rather than the
            datasheet, since restricting the area is the main speed lever.
        overlap: Fraction of each swath to re-image on the next pass.
            Non-zero by default because butt-joined strips leave a seam
            wherever the travel speed wandered, and stitching needs something
            to match on.
        axis: Gantry axis the traverses run along.
        step_axis: Axis the passes step along. Defaults to the other
            horizontal axis.
        serpentine: Alternate direction each pass, so the gantry does not
            drive back to the same side every time. Halves the repositioning
            travel; set False if the instrument is direction-sensitive.
        instrument: Which instrument measures.
        scan_speed: Measuring traverse speed for every pass.
        travel_speed: Repositioning speed to each pass's start, if it should
            differ from scan_speed — see Pass.travel_speed.
        speed: Convenience for setting scan_speed and travel_speed to the
            same value. Ignored for whichever of the two is set explicitly.
        accel_mm_s2: The travel axis's configured acceleration, mm/s^2. When
            set, each pass's commanded start (``Pass.start``) is moved back
            along the travel axis by the accel-ramp distance
            (``v^2 / 2a`` at ``scan_speed``), on whichever side matches that
            pass's serpentine direction, so the axis is already at
            ``scan_speed`` — not still accelerating — by the time it
            reaches the swath boundary (``Pass.cruise_start``). Left unset
            (the default), passes are commanded exactly at the swath
            boundary as before: no ramp adjustment, and the classic
            travel-direction seam between swaths (see issue #58) can
            reappear. ``SurveyRunner.run()`` fills this in automatically
            from the live axis's ``get_accel()`` when a ``Tile`` leaves it
            unset — set it explicitly only for planning/dry-run use before
            a gantry is connected.

            The same distance is added past the end of each pass
            (``Pass.overrun_end``): the axis is commanded to stop one ramp
            *beyond* the survey edge, assuming it decelerates at the rate it
            accelerates, so the swath is covered entirely at constant speed
            and the slowdown happens outside the region of interest.

    Each pass's ``step_axis`` coordinate names the swath's **near edge**,
    not its centerline — ``SurveyRunner`` reads the instrument's live
    active area (and the frame's mount rotation) to place that edge
    correctly rather than assuming the instrument's configured reference
    point is already edge-aligned (it normally isn't: a Gocator's own
    coordinate origin is its centerline). See ``Pass.edge_align``.
    """

    origin: Sequence[float]
    length_mm: float
    width_mm: float
    swath_mm: float
    overlap: float = 0.1
    axis: str = "X"
    step_axis: Optional[str] = None
    serpentine: bool = True
    instrument: str = "gocator"
    scan_speed: Optional[float] = None
    travel_speed: Optional[float] = None
    speed: Optional[float] = None
    accel_mm_s2: Optional[float] = None

    def __post_init__(self) -> None:
        """Validate tile survey parameters."""
        if len(self.origin) != 3:
            raise ValueError(f"origin must have 3 components [x, y, z], got {self.origin!r}")
        if self.swath_mm <= 0:
            raise ValueError(f"swath_mm must be positive, got {self.swath_mm}")
        if not 0.0 <= self.overlap < 1.0:
            raise ValueError(
                f"overlap must be in [0, 1), got {self.overlap} — 1.0 would "
                "mean each pass re-images the previous one entirely and the "
                "survey would never advance"
            )
        if self.length_mm <= 0 or self.width_mm <= 0:
            raise ValueError("length_mm and width_mm must be positive")
        if self.axis not in _AXIS_INDEX:
            raise ValueError(f"axis must be one of {list(_AXIS_INDEX)}")
        if self.step_axis is None:
            self.step_axis = "Y" if self.axis == "X" else "X"
        if self.step_axis == self.axis:
            raise ValueError("step_axis must differ from axis")
        if self.speed is not None:
            if self.scan_speed is None:
                self.scan_speed = self.speed
            if self.travel_speed is None:
                self.travel_speed = self.speed

    @classmethod
    def from_roi(
        cls,
        roi_x_mm: Sequence[float],
        roi_y_mm: Sequence[float],
        z_mm: float,
        *,
        swath_mm: Optional[float] = None,
        lab: Optional[Any] = None,
        instrument: str = "gocator",
        min_overlap: float = 0.1,
        axis: Optional[str] = None,
        gantry_axis: Optional[str] = None,
        frames: Optional[Any] = None,
        center_single_pass: bool = True,
        **kwargs: Any,
    ) -> "Tile":
        """Plan the passes needed to cover a region of interest.

        Give the region as experiment-frame bounds and the planner works out
        how many swaths it needs and how much they overlap, rather than
        taking the overlap as a given:

        - **Wider than one swath:** the fewest passes that keep at least
          `min_overlap` between neighbours, spread *evenly* so every seam
          has the same overlap and the passes cover the region exactly
          (first swath starts on the near edge, last ends on the far edge).
        - **No wider than one swath** (including exactly one): a single
          pass, not an error. With `center_single_pass` the swath is
          centred on the region so the sensor sees margin on both sides;
          otherwise it starts on the near edge.

        The traverse always runs the region's full extent along `axis`.

        Args:
            roi_x_mm: ``(min, max)`` of the region along experiment X.
            roi_y_mm: ``(min, max)`` of the region along experiment Y.
            z_mm: Experiment-frame Z the passes are planned at.
            swath_mm: How wide one pass images. Omit to read it from `lab`.
            lab: Connected FlumeLab whose `instrument` reports its live
                ``get_active_area()["width_mm"]`` — the swath actually
                imaged, not the datasheet field of view. Only read from.
            instrument: Instrument whose active area sets the swath.
            min_overlap: Smallest acceptable fraction of a swath re-imaged
                by the next pass, in ``[0, 1)``.
            axis: *Experiment* axis the traverses run along (default ``"X"``);
                passes step along the other horizontal axis.
            gantry_axis: Alternatively, the *gantry* axis the scan must run
                along, whatever the experiment frame's rotation: the matching
                experiment axis is worked out from `frames` (or `lab.frames`)
                and used as `axis`. At 0/180 degrees that is the same letter,
                at ±90 it is the other one. Give this or `axis`, not both.
            frames: A FrameRegistry, needed with `gantry_axis` when no `lab`
                is given.
            center_single_pass: Centre a one-pass swath on the region.
            **kwargs: Passed to :class:`Tile` (speeds, `accel_mm_s2`,
                `serpentine`, `instrument`, ...).

        Returns:
            A Tile whose passes cover the region.

        Raises:
            ValueError: If neither `swath_mm` nor a `lab` with an active
                area is given, a range is empty or reversed,
                `min_overlap` is outside ``[0, 1)``, both `axis` and
                `gantry_axis` are given, `gantry_axis` has no `frames` to
                resolve it with, or the experiment frame is not rotated by a
                multiple of 90 degrees.
        """
        if gantry_axis is not None:
            if axis is not None:
                raise ValueError("give axis (experiment) or gantry_axis, not both")
            resolver = frames if frames is not None else getattr(lab, "frames", None)
            if resolver is None:
                raise ValueError("gantry_axis needs `frames` or a `lab` to resolve it with")
            axis = resolver.experiment_axis_for(gantry_axis)
        elif axis is None:
            axis = "X"
        if axis not in ("X", "Y"):
            raise ValueError(f"axis must be 'X' or 'Y' for a horizontal ROI, got {axis!r}")
        if not 0.0 <= min_overlap < 1.0:
            raise ValueError(f"min_overlap must be in [0, 1), got {min_overlap}")
        if swath_mm is None:
            scanner = getattr(lab, instrument, None) if lab is not None else None
            if scanner is None or not hasattr(scanner, "get_active_area"):
                raise ValueError(
                    "Tile.from_roi() needs swath_mm, or a lab whose "
                    f"{instrument!r} reports get_active_area()"
                )
            swath_mm = float(scanner.get_active_area()["width_mm"])
        if swath_mm <= 0:
            raise ValueError(f"swath_mm must be positive, got {swath_mm}")

        ranges = {"X": tuple(float(v) for v in roi_x_mm), "Y": tuple(float(v) for v in roi_y_mm)}
        for name, (lo, hi) in ranges.items():
            if not hi > lo:
                raise ValueError(f"roi_{name.lower()}_mm must be (min, max) with max > min, got {(lo, hi)}")
        step = "Y" if axis == "X" else "X"
        t_lo, t_hi = ranges[axis]
        s_lo, s_hi = ranges[step]
        extent = s_hi - s_lo

        if extent <= swath_mm:
            width = swath_mm if center_single_pass else extent
            step_origin = (s_lo + s_hi) / 2 - swath_mm / 2 if center_single_pass else s_lo
            overlap = min_overlap
        else:
            count = math.ceil((extent - swath_mm) / (swath_mm * (1.0 - min_overlap)) - 1e-9) + 1
            pitch = (extent - swath_mm) / (count - 1)
            # A hair under the exact value, so floating-point noise can't make
            # passes() count one pass too many; the last swath is clamped onto
            # the far edge regardless.
            overlap = max(0.0, 1.0 - pitch * (1.0 + 1e-9) / swath_mm)
            width, step_origin = extent, s_lo

        origin = [0.0, 0.0, float(z_mm)]
        origin[_AXIS_INDEX[axis]] = t_lo
        origin[_AXIS_INDEX[step]] = step_origin
        return cls(
            origin=origin, length_mm=t_hi - t_lo, width_mm=width, swath_mm=swath_mm,
            overlap=overlap, axis=axis, instrument=instrument, **kwargs,
        )

    @property
    def pitch_mm(self) -> float:
        """Get the distance between adjacent pass centres.

        Returns:
            Pitch distance in mm.
        """
        return self.swath_mm * (1.0 - self.overlap)

    def passes(self) -> List[Pass]:
        """Generate the planned passes for this tile survey.

        Returns:
            List of Pass objects tiling the region.
        """
        travel_i = _AXIS_INDEX[self.axis]
        step_i = _AXIS_INDEX[self.step_axis]

        # A region no wider than one swath needs exactly one pass — anything
        # else is duplicate, identical coverage (both the first and second
        # pass's offset clamp to 0 via the max(0, width - swath) below, so
        # the naive ceil(width / pitch) counted the same pass twice). Beyond
        # that, round up: the first pass covers one swath, and each
        # additional pass advances by one pitch — a region 2.5 swaths wide
        # needs 3 passes, and the last one overlapping more than asked is
        # fine, a gap is not.
        if self.width_mm <= self.swath_mm:
            count = 1
        else:
            count = math.ceil((self.width_mm - self.swath_mm) / self.pitch_mm) + 1

        ramp_mm = ramp_distance_mm(self.scan_speed, self.accel_mm_s2)

        out: List[Pass] = []
        for i in range(count):
            offset = min(i * self.pitch_mm, max(0.0, self.width_mm - self.swath_mm))
            start = list(self.origin)
            end = list(self.origin)
            start[step_i] += offset
            end[step_i] += offset
            forward = (i % 2 == 0) or not self.serpentine
            if forward:
                end[travel_i] += self.length_mm
            else:
                start[travel_i] += self.length_mm
            cruise_start = tuple(float(v) for v in start)
            overrun_end = None
            if ramp_mm > 0:
                # Mirror of the lead-in below: stop one ramp past the swath
                # end, so the axis is still at scan_speed when the measured
                # stretch ends and decelerates outside it.
                stop = list(end)
                stop[travel_i] += (1.0 if forward else -1.0) * ramp_mm
                overrun_end = tuple(float(v) for v in stop)
                # Extend the commanded start opposite the travel direction,
                # so the axis is already at scan_speed — not still
                # ramping — when it reaches the true swath boundary
                # (cruise_start). Direction alternates with serpentine
                # travel direction, same as the `forward` branch above.
                direction = 1.0 if forward else -1.0
                start[travel_i] -= direction * ramp_mm
            out.append(
                Pass(
                    index=i,
                    start=tuple(float(v) for v in start),
                    end=tuple(float(v) for v in end),
                    cruise_start=cruise_start if ramp_mm > 0 else None,
                    overrun_end=overrun_end,
                    instrument=self.instrument,
                    axis=self.axis,
                    scan_speed=self.scan_speed,
                    travel_speed=self.travel_speed,
                    label=f"tile {i + 1}/{count}",
                    edge_align=True,
                    step_axis=self.step_axis,
                )
            )
        return out

    def coverage_mm(self) -> float:
        """Get the total width actually imaged.

        Returns:
            Total width coverage, which may exceed requested width_mm due to overlap.
        """
        passes = self.passes()
        step_i = _AXIS_INDEX[self.step_axis]
        first = passes[0].start[step_i]
        last = passes[-1].start[step_i]
        return (last - first) + self.swath_mm

    def stitch(
        self,
        passes: Sequence[Pass],
        results: Sequence[Any],
        frames: Any,
    ) -> Any:
        """Naively merge this tile's per-pass scans into one SurfaceScan.

        "Naive": each pass is placed into shared experiment coordinates via
        :func:`~laguna.frames.orient_scan` (mount rotation + that pass's
        actual commanded gantry position — unaffected by whichever
        ``reference_point`` ``SurveyRunner`` used to get the gantry there,
        since ``orient_scan`` works from where the gantry *actually ended
        up*, not how it was targeted), then every pass's points are simply
        concatenated. It does **not** use the overlap between adjacent
        swaths to register or correct passes against each other — no ICP,
        no cross-correlation, nothing that would catch travel-speed wander
        or a small position error between passes. It trusts the calibrated
        frames outright. ``overlap`` exists for a future registration step
        to consume (see this class's own docstring); this doesn't consume
        it yet.

        Args:
            passes: The passes that were actually run, e.g. the list
                ``SurveyRunner.run()`` returns. Same length and order as
                `results`.
            results: Each pass's raw (un-oriented) SurfaceScan, e.g.
                ``SurveyRunner.results`` from a ``run(keep_results=True)``
                call.
            frames: The lab's FrameRegistry, for orienting each pass into
                shared experiment coordinates.

        Returns:
            One :class:`~laguna.scanner.pointcloud.SurfaceScan` covering
            the whole tile, in experiment coordinates, as a flat ``(1, N)``
            non-uniform grid of every valid point from every pass —
            exportable exactly like any other SurfaceScan
            (``save_csv``/``save_las``/``save_ply``/``save_npz``).

        Raises:
            ValueError: If `passes` and `results` differ in length, or
                either is empty.
        """
        from .frames import orient_scan
        from .scanner.pointcloud import SensorMounting, SurfaceScan

        if len(passes) != len(results):
            raise ValueError(
                f"stitch() needs one result per pass — got {len(passes)} "
                f"passes and {len(results)} results"
            )
        if not passes:
            raise ValueError("stitch() needs at least one pass/result to merge")

        oriented = [
            orient_scan(scan, instrument=p.instrument, frames=frames)
            for p, scan in zip(passes, results)
        ]
        points = np.concatenate([o.to_points(drop_invalid=True) for o in oriented], axis=0)

        metadata = {
            "stitched": True,
            "stitch_method": "naive-concatenate",
            "stitch_pass_count": len(passes),
            "stitch_pass_labels": [p.label or p.instrument for p in passes],
            "stitch_pass_point_counts": [o.valid_count for o in oriented],
            "stitch_instruments": sorted({p.instrument for p in passes}),
        }
        return SurfaceScan(
            x_mm=points[:, 0].reshape(1, -1),
            y_mm=points[:, 1].reshape(1, -1),
            z_mm=points[:, 2].reshape(1, -1),
            metadata=metadata,
            is_uniform=False,
            mounting=SensorMounting(),
        )


@dataclass
class Traverse(Survey):
    """Re-run one line, optionally with several instruments.

    Two uses. Repeating a transect over an experiment gives a time series of
    the same ground — the usual way to watch a bed evolve. Running the same
    line with more than one instrument gives a comparison over identical
    ground, which is only meaningful because
    :class:`~laguna.frames.FrameRegistry` compensates for their different
    mountings.

    Args:
        start: [x, y, z] start of the line, experiment frame.
        end: [x, y, z] end of the line.
        instruments: One pass per instrument, in this order.
        repeats: How many times to run the whole set.
        axis: Gantry axis the line runs along.
        scan_speed: Measuring traverse speed.
        travel_speed: Repositioning speed to the line's start, if it should
            differ from scan_speed — see Pass.travel_speed.
        speed: Convenience for setting scan_speed and travel_speed to the
            same value. Ignored for whichever of the two is set explicitly.
    """

    start: Sequence[float]
    end: Sequence[float]
    instruments: Sequence[str] = ("gocator",)
    repeats: int = 1
    axis: str = "X"
    scan_speed: Optional[float] = None
    travel_speed: Optional[float] = None
    speed: Optional[float] = None

    def __post_init__(self) -> None:
        """Validate traverse parameters."""
        if isinstance(self.instruments, str):
            # A bare "wtt12l" is one instrument, not six — Sequence[str]
            # would otherwise iterate it character by character.
            self.instruments = (self.instruments,)
        if len(self.start) != 3:
            raise ValueError(f"start must have 3 components [x, y, z], got {self.start!r}")
        if len(self.end) != 3:
            raise ValueError(f"end must have 3 components [x, y, z], got {self.end!r}")
        if self.repeats < 1:
            raise ValueError(f"repeats must be >= 1, got {self.repeats}")
        if not self.instruments:
            raise ValueError("at least one instrument is required")
        if self.speed is not None:
            if self.scan_speed is None:
                self.scan_speed = self.speed
            if self.travel_speed is None:
                self.travel_speed = self.speed

    def passes(self) -> List[Pass]:
        """Generate the planned passes for this traverse.

        Returns:
            List of Pass objects repeating the line with each instrument.
        """
        out: List[Pass] = []
        for r in range(self.repeats):
            for instrument in self.instruments:
                out.append(
                    Pass(
                        index=len(out),
                        start=tuple(float(v) for v in self.start),
                        end=tuple(float(v) for v in self.end),
                        instrument=instrument,
                        axis=self.axis,
                        scan_speed=self.scan_speed,
                        travel_speed=self.travel_speed,
                        label=(
                            f"{instrument} repeat {r + 1}/{self.repeats}"
                            if self.repeats > 1
                            else instrument
                        ),
                    )
                )
        return out


class SurveyRunner:
    """Executes a plan, one pass at a time, resumably.

    Kept separate from the planners so a survey can be inspected, costed and
    checkpointed before anything moves.

    Each pass positions the instrument's *measuring point* via
    ``lab.place()`` — not the gantry's commanded point — so the plan means
    the same thing whichever instrument runs it.

    Args:
        lab: A connected FlumeLab.
        survey: The plan.
        checkpoint: Optional CheckpointStore. Pass indices are marked
            complete as they finish, so a survey interrupted by a pause
            resumes where it left off rather than starting over. A tile of
            a wide bed can be the longest thing an experiment does. The
            survey's geometry fingerprint (:meth:`Survey.fingerprint`) is
            stored in it, and a checkpoint with completed passes made for a
            *different* geometry — or by a version that recorded none — is
            refused rather than trusted (see ``restart``).
        restart: Start over against `checkpoint` instead of refusing a
            mismatch. The old checkpoint is moved aside, never deleted
            (``CheckpointStore.clear()``), so the record of what the earlier
            plan completed survives.

    Raises:
        SurveyCheckpointMismatch: At construction, if `checkpoint` has
            completed passes for a different (or unrecorded) geometry and
            `restart` is False.
    """

    def __init__(
        self, lab: Any, survey: Survey, checkpoint: Optional[Any] = None, restart: bool = False
    ) -> None:
        """Initialize a survey runner.

        Args:
            lab: Connected FlumeLab instance.
            survey: The survey plan to execute.
            checkpoint: Optional CheckpointStore for resumable execution.
            restart: See the class docstring.
        """
        self.lab = lab
        self.survey = survey
        self.checkpoint = checkpoint
        if checkpoint is not None:
            self._bind_checkpoint(restart)
        self.completed: List[int] = []
        #: Acquired SurfaceScan/ProfileResult objects, one per pass that
        #: actually ran — only populated when run(keep_results=True). Empty
        #: otherwise, since holding every scan in memory for a long survey
        #: (a full-resolution Gocator surface is hundreds of MB) is real
        #: cost most callers don't want by default.
        self.results: List[Any] = []

    def _bind_checkpoint(self, restart: bool) -> None:
        """Tie the checkpoint to this survey's geometry, or refuse a mismatch.

        Checked at construction, before anything moves. A checkpoint with no
        completed passes carries nothing to protect and is simply stamped.

        Args:
            restart: Move any existing progress aside and start over.

        Raises:
            SurveyCheckpointMismatch: See the class docstring.
        """
        wanted = self.survey.fingerprint()
        stored = self.checkpoint.meta.get(_FINGERPRINT_KEY)
        has_progress = self.checkpoint.last_completed() is not None

        if restart:
            if has_progress or stored is not None:
                self.checkpoint.clear()
            self.checkpoint.set_meta(_FINGERPRINT_KEY, wanted)
            return
        if stored == wanted:
            return
        if stored is None and not has_progress:
            self.checkpoint.set_meta(_FINGERPRINT_KEY, wanted)
            return
        done = sorted(
            i for i in range(len(self.survey)) if self.checkpoint.is_complete(i)
        )
        if stored is None:
            why = "it records no survey geometry (written before fingerprints existed)"
        else:
            why = f"it was made for a different geometry (fingerprint {stored}, now {wanted})"
        raise SurveyCheckpointMismatch(
            f"Refusing to resume: checkpoint has passes {done} complete, but {why}. "
            "Resuming would skip passes this survey never ran. Nothing was deleted. "
            "Pass restart=True to move the old checkpoint aside and start over, or, "
            "if you know the geometry is unchanged, adopt it with "
            f"checkpoint.set_meta({_FINGERPRINT_KEY!r}, survey.fingerprint())."
        )

    def pending(self) -> List[Pass]:
        """Get the list of passes not yet completed.

        Returns:
            Passes not yet done, according to the checkpoint (if present).
        """
        if self.checkpoint is None:
            return list(self.survey.passes())
        return [p for p in self.survey.passes() if not self.checkpoint.is_complete(p.index)]

    def run(self, dry_run: bool = False, keep_results: bool = False) -> List[Pass]:
        """Execute all pending passes in order.

        Args:
            dry_run: If True, log the plan without moving or measuring.
            keep_results: If True, also collect each pass's acquired
                SurfaceScan/ProfileResult into ``self.results`` (in the same
                order as the returned passes — ``self.results[i]`` is what
                ``done[i]`` measured). Off by default: a long survey holding
                every scan in memory (a full-resolution Gocator surface is
                hundreds of MB) is real cost most callers don't want.

        Returns:
            The passes that were executed.
        """
        self._fill_tile_accel()
        for p in self.survey.passes():
            self._gantry_axis_for(p.axis)      # fail before any motion, not mid-survey
        done: List[Pass] = []
        for p in self.pending():
            reference_point = self._resolve_reference_point(p)
            gantry_start = self.lab.frames.gantry_target_for(
                p.instrument, list(p.start), reference_point=reference_point
            )
            gantry_end = self.lab.frames.gantry_target_for(
                p.instrument, list(p.end), reference_point=reference_point
            )
            ramp_note = ""
            if p.cruise_start is not None and p.cruise_start != p.start:
                ramp_note = f" (ramp start {tuple(round(v, 1) for v in p.start)})"
            if p.overrun_end is not None:
                ramp_note += f" (stops at {tuple(round(v, 1) for v in p.overrun_end)})"
            logger.info(
                "Survey pass %d/%d — %s | experiment %s -> %s | gantry %s -> %s%s",
                p.index + 1, len(self.survey), p.label or p.instrument,
                tuple(round(v, 1) for v in p.measure_start), tuple(round(v, 1) for v in p.end),
                tuple(round(float(v), 1) for v in gantry_start),
                tuple(round(float(v), 1) for v in gantry_end),
                ramp_note,
            )
            if dry_run:
                done.append(p)
                continue
            result = self._run_pass(p, reference_point)
            if keep_results:
                self.results.append(result)
            done.append(p)
            self.completed.append(p.index)
            if self.checkpoint is not None:
                self.checkpoint.mark_complete(
                    p.index, runtime_s=self.lab.clock.elapsed(),
                    wall_time=self.lab.clock.wall_time(), name=p.label,
                )
        return done

    def _gantry_axis_for(self, experiment_axis: str) -> str:
        """Get the gantry axis that drives a straight pass along an experiment axis.

        Passes are planned along an experiment axis but the gantry moves along
        its own; see :meth:`FrameRegistry.gantry_axis_for`.

        Args:
            experiment_axis: ``"X"``, ``"Y"`` or ``"Z"``.

        Returns:
            The gantry axis name.

        Raises:
            ValueError: If the experiment axis is diagonal in gantry axes.
        """
        frames = getattr(self.lab, "frames", None)
        if not hasattr(frames, "gantry_axis_for"):
            return experiment_axis
        return frames.gantry_axis_for(experiment_axis)

    def _fill_tile_accel(self) -> None:
        """Read the travel axis's accel into a Tile survey that didn't set one.

        Lets ``Tile`` stay hardware-free for planning/dry-run use
        (``describe()``, ``coverage_mm()``) while ``run()`` — which already
        needs a connected gantry — supplies the real number so passes get
        a ramp-derived lead-in without every caller reading
        ``get_accel()`` itself. Mutating ``self.survey`` here is safe:
        ``Tile.passes()`` isn't cached, so `run()`'s subsequent calls pick
        up the filled-in value immediately.

        A tile with an explicit ``accel_mm_s2`` is left untouched. Any
        other survey type (e.g. ``Traverse``, which has no such attribute)
        or a lab with no ``gantry`` is a no-op.
        """
        if not isinstance(self.survey, Tile) or self.survey.accel_mm_s2 is not None:
            return
        gantry = getattr(self.lab, "gantry", None)
        if gantry is None:
            return
        try:
            self.survey.accel_mm_s2 = gantry.axis(self._gantry_axis_for(self.survey.axis)).get_accel()
        except Exception as e:
            logger.warning(
                "Could not read %s accel for tile-scan ramp lead-in — "
                "passes will run without one, and the travel-direction "
                "seam between swaths (issue #58) may reappear: %s",
                self.survey.axis, e,
            )

    def _run_pass(self, p: Pass, reference_point: Optional[List[float]] = None) -> Any:
        """Position and measure one pass.

        Logs success or failure to the event log, then re-raises on error so
        a failed pass escalates (not silently continues).

        Args:
            p: Pass to execute.
            reference_point: Pre-resolved via ``_resolve_reference_point()``
                — taken as a parameter rather than resolved again here so
                the actual placement matches what ``run()`` already logged
                (and to avoid a second live ``get_active_area()`` call).

        Returns:
            The acquired SurfaceScan (Gocator) or ProfileResult (rangefinder).

        Raises:
            ValueError: If the pass is missing required parameters.
            Any exception from scanner.acquire() or the profiler path.
        """
        # place() puts the INSTRUMENT's measuring point on the target, which
        # is what makes one plan valid for several instruments. Reposition
        # at travel_speed if the pass sets one, falling back to scan speed —
        # repositioning at scan speed is slower than necessary but always
        # safe.
        travel_speed = p.travel_speed if p.travel_speed is not None else p.scan_speed
        scanner = getattr(self.lab, p.instrument, None)
        gantry = getattr(self.lab, "gantry", None)
        arbiter = getattr(gantry, "arbiter", DEFAULT_ARBITER)

        # Held across the reposition *and* the scan, so nothing scheduled can
        # move the gantry in between. Holding it also makes place() run
        # inline (blocking) rather than returning a still-moving handle.
        # Inside the try: a pass that fails while repositioning (a fence, a
        # halt) is logged against the pass like any other failure.
        try:
            with arbiter.hold(f"survey pass {p.index} ({p.instrument})"):
                return self._run_pass_held(p, reference_point, travel_speed, scanner, gantry)
        except Exception as exc:
            self.lab.event_log.log(
                self.lab.clock.elapsed(), p.instrument, "survey_pass",
                result=f"error: {exc}", notes=p.label or "",
            )
            raise

    def _run_pass_held(
        self,
        p: Pass,
        reference_point: Optional[List[float]],
        travel_speed: Optional[float],
        scanner: Any,
        gantry: Any,
    ) -> Any:
        """Body of _run_pass(), with the gantry held."""
        self.lab.place(
            p.instrument, list(p.start), speed=travel_speed, reference_point=reference_point
        ).wait()

        if scanner is not None and hasattr(scanner, "acquire"):
            # The same reference point as the start: an edge-aligned
            # pass is positioned by its swath edge, and its end must be
            # too, or a rotated mount shifts the end along the travel axis.
            end_gantry = self.lab.frames.gantry_target_for(
                p.instrument, list(p.end), reference_point=reference_point
            )
            gantry_axis = self._gantry_axis_for(p.axis)
            axis_index = _AXIS_INDEX[gantry_axis]
            overrides: Dict[str, Any] = {}
            end_mm = float(end_gantry[axis_index])
            if p.overrun_end is not None:
                # Lead-out: send the axis one ramp past the swath end so it is
                # still at speed there, but end the capture at the swath end.
                stop_gantry = self.lab.frames.gantry_target_for(
                    p.instrument, list(p.overrun_end), reference_point=reference_point
                )
                overrides["capture_end_mm"] = end_mm
                end_mm = float(stop_gantry[axis_index])
            if p.scan_speed:
                overrides["feed_rate_mm_s"] = p.scan_speed
            if p.cruise_start is not None:
                # A ramp lead-in was planned: tell the scanner where the
                # true swath boundary is (for gantry_start_mm/dead
                # reckoning) rather than letting it anchor to wherever
                # p.start's live pre-motion read happens to be, and give
                # it the matching kinematic settle time — same accel and
                # scan_speed the lead-in distance was computed from — so
                # the trigger fires once the axis has actually covered
                # that lead-in, not after an unrelated guess. Same
                # reference point as start/end, or a rotated mount would
                # shift the anchor along the travel axis.
                cruise_gantry = self.lab.frames.gantry_target_for(
                    p.instrument, list(p.cruise_start), reference_point=reference_point
                )
                overrides["cruise_start_mm"] = float(cruise_gantry[axis_index])
                overrides["settle_s"] = ramp_time_s(p.scan_speed, self.survey.accel_mm_s2)
            scan = scanner.acquire(
                gantry=gantry,
                axis=gantry_axis,
                end_mm=end_mm,
                **overrides,
            )
            result_note = f"points={scan.valid_count}" if scan is not None else "no data"
        else:
            # No acquire() — a rangefinder transect goes through the
            # profiler path instead, which has no per-instrument
            # fallback rate the way GocatorScanner.acquire() does, so a
            # survey/pass without one would otherwise fail deep inside
            # FlumeLab.acquire_scan() with a message that doesn't name
            # the pass.
            if p.scan_speed is None:
                raise ValueError(
                    f"pass {p.index} ({p.instrument}) has no scan_speed — "
                    "set it on the Survey or override this Pass; unlike "
                    "GocatorScanner.acquire(), the rangefinder profiler path "
                    "has no configured-spec fallback to fall back to."
                )
            result = self.lab.acquire_scan(
                p.instrument,
                start=None,
                end=self._acquire_scan_end(p, reference_point),
                feed_rate_mm_s=p.scan_speed,
                axis=self._gantry_axis_for(p.axis),
            )
            result_note = f"file={result.path}"
        self.lab.event_log.log(
            self.lab.clock.elapsed(), p.instrument, "survey_pass",
            result=result_note, notes=p.label or "",
        )
        return scan if scanner is not None and hasattr(scanner, "acquire") else result

    def _resolve_reference_point(self, p: Pass) -> Optional[List[float]]:
        """Get the reference point `place()` should use for `p`.

        None means the instrument's usual (configured) one.

        Shared by ``run()``'s per-pass log line and ``_run_pass()``'s actual
        placement, so the logged gantry target — dry-run or real — is always
        the one that would actually be commanded, not a stand-in.

        Args:
            p: The pass to resolve a reference point for.

        Returns:
            ``[x, y, z]`` — see ``_edge_reference_point()`` for the frame
            it's in — or None if `p` isn't edge-aligned or the instrument
            has no live active area (e.g. not yet connected, or a
            rangefinder with no such concept).
        """
        if not (p.edge_align and p.step_axis is not None):
            return None
        scanner = getattr(self.lab, p.instrument, None)
        if scanner is None or not hasattr(scanner, "get_active_area"):
            return None
        return self._edge_reference_point(p, scanner)

    def _edge_reference_point(self, p: Pass, scanner: Any) -> List[float]:
        """Reference point sitting at the swath's near edge, for `place()`.

        A Tile pass's ``step_axis`` coordinate names the swath's near
        edge (see ``Tile``'s docstring), but an instrument's *configured*
        reference point is its own coordinate origin — for the Gocator,
        the centerline of whatever active area is set, not either edge.
        Reading the sensor's own X (cross-laser) span live off
        ``get_active_area()`` and substituting it as the reference point
        (via ``place(reference_point=...)``) lands the actual edge where
        Tile intended, instead of straddling it by half a swath.

        Which physical boundary (the active area's min or max X) counts
        as "near" depends on the mount's rotation between sensor and
        gantry axes — read live off the scanner's own ``mounting`` (see
        ``laguna.scanner.mounting``) rather than re-deriving the sign here
        — **and** on the experiment frame's rotation relative to the
        gantry, since the pass's ``step_axis`` is an experiment axis.

        Rotated here, not left to ``place()``: ``frames.instruments.gocator``
        is deliberately translation-only (``orient_scan()`` refuses to run
        if it also carries a rotation, since the scan's own ``mounting``
        already provides one — see that config block's comment). So the
        value returned is already rotated into gantry directions, not the
        sensor's own frame — safe here specifically because
        ``frames.instruments.gocator``'s mount has no rotation of its own
        to compose with, so passing an already-rotated delta through it
        via ``place(reference_point=...)`` is a plain translation add, not
        a double transform.

        Args:
            p: The edge-aligned pass (``p.edge_align`` and ``p.step_axis``
                must both be set).
            scanner: The instrument, with a working ``get_active_area()``
                and ``mounting``.

        Returns:
            ``[x, y, z]``, gantry-oriented — see above.
        """
        area = scanner.get_active_area()
        mounting = scanner.mounting
        step_i = _AXIS_INDEX[p.step_axis]
        # Column 0 of the mounting matrix is where sensor +X (across the
        # laser) points in *gantry* axes, but step_axis is an *experiment*
        # axis. With a rotated experiment frame (e.g. rotation_deg: 180)
        # the two disagree in sign, so carry the direction through the
        # experiment-from-gantry rotation before asking which way
        # increasing sensor X moves along step_axis: toward or away from
        # the pass's near (offset) edge. Skipping this picked the far edge
        # and offset a whole tile by one swath.
        across_in_gantry = mounting.matrix[:3, 0]
        frames = getattr(self.lab, "frames", None)
        experiment_from_gantry = getattr(frames, "experiment_from_gantry", None)
        rotation = (
            experiment_from_gantry.matrix[:3, :3]
            if experiment_from_gantry is not None
            else np.eye(3)
        )
        sign_along_step = (rotation @ across_in_gantry)[step_i]
        if sign_along_step >= 0:
            edge_x = area["x_mm"]
        else:
            edge_x = area["x_mm"] + area["width_mm"]
        delta_gantry = mounting.apply_to_points(np.array([[edge_x, 0.0, 0.0]]))[0]
        return [float(v) for v in delta_gantry]

    def _acquire_scan_end(
        self, p: Pass, reference_point: Optional[List[float]] = None
    ) -> List[float]:
        """Build acquire_scan()'s full per-axis `end` from an experiment-frame target.

        gantry_target_for() only returns X/Y/Z — Theta is outside the
        Cartesian frame model (see FrameRegistry) — but acquire_scan()
        requires one value per *configured* gantry axis, same as
        move_to()'s vector form. Any axis outside X/Y/Z (Theta) is
        backfilled with its live current position, same as move_to()'s
        keyword form leaves unspecified axes untouched.
        """
        xyz = self.lab.frames.gantry_target_for(
            p.instrument, list(p.end), reference_point=reference_point
        )
        by_name = {"X": float(xyz[0]), "Y": float(xyz[1]), "Z": float(xyz[2])}
        gantry = self.lab.gantry
        return [
            by_name[axis.name] if axis.name in by_name
            else gantry.cmd.get_actual_position(axis)
            for axis in gantry._axes
        ]


__all__ = ["Pass", "Survey", "SurveyCheckpointMismatch", "Tile", "Traverse", "SurveyRunner"]
