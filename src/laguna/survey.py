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

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)

#: Cartesian axis name -> index into a 3-component [x, y, z] vector. Shared
#: by Tile and SurveyRunner rather than each keeping its own copy.
_AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


@dataclass
class Pass:
    """One traverse: measure from `start` to `end`, both experiment-frame mm.

    Attributes:
        index: Position in the plan, 0-based. Used as the checkpoint id, so
            an interrupted survey resumes at the right pass.
        start: [x, y, z] where the measuring point should begin.
        end: [x, y, z] where it should finish.
        instrument: Which instrument measures this pass.
        axis: Gantry axis the traverse runs along, resolved from the
            geometry.
        scan_speed: Measuring traverse speed, if the plan fixed one.
        travel_speed: Speed for the pre-scan repositioning move to `start`,
            if it should differ from `scan_speed`. Defaults to `scan_speed`
            when omitted — repositioning at scan speed is slower than
            necessary but always safe.
        label: Human-readable description, for logs and progress.
    """

    index: int
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    instrument: str = "gocator"
    axis: str = "X"
    scan_speed: Optional[float] = None
    travel_speed: Optional[float] = None
    label: str = ""

    @property
    def length_mm(self) -> float:
        """Get the traverse distance in mm.

        Returns:
            Distance from start to end.
        """
        return math.dist(self.start, self.end)

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
            "instrument": self.instrument,
            "axis": self.axis,
            "scan_speed": self.scan_speed,
            "travel_speed": self.travel_speed,
            "label": self.label,
        }


class Survey:
    """Base class for survey planners that produce a list of passes."""

    def passes(self) -> List[Pass]:  # pragma: no cover - interface
        """Get the planned passes.

        Returns:
            List of Pass objects.
        """
        raise NotImplementedError

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
            rows.append(
                f"  [{p.index}] {p.label or p.instrument}: "
                f"{tuple(round(v, 1) for v in p.start)} -> "
                f"{tuple(round(v, 1) for v in p.end)} "
                f"along {p.axis} ({p.length_mm:.0f} mm)"
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
            out.append(
                Pass(
                    index=i,
                    start=tuple(float(v) for v in start),
                    end=tuple(float(v) for v in end),
                    instrument=self.instrument,
                    axis=self.axis,
                    scan_speed=self.scan_speed,
                    travel_speed=self.travel_speed,
                    label=f"tile {i + 1}/{count}",
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
            a wide bed can be the longest thing an experiment does.
    """

    def __init__(self, lab: Any, survey: Survey, checkpoint: Optional[Any] = None) -> None:
        """Initialize a survey runner.

        Args:
            lab: Connected FlumeLab instance.
            survey: The survey plan to execute.
            checkpoint: Optional CheckpointStore for resumable execution.
        """
        self.lab = lab
        self.survey = survey
        self.checkpoint = checkpoint
        self.completed: List[int] = []
        #: Acquired SurfaceScan/ProfileResult objects, one per pass that
        #: actually ran — only populated when run(keep_results=True). Empty
        #: otherwise, since holding every scan in memory for a long survey
        #: (a full-resolution Gocator surface is hundreds of MB) is real
        #: cost most callers don't want by default.
        self.results: List[Any] = []

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
        done: List[Pass] = []
        for p in self.pending():
            logger.info(
                "Survey pass %d/%d — %s", p.index + 1, len(self.survey), p.label or p.instrument
            )
            if dry_run:
                done.append(p)
                continue
            result = self._run_pass(p)
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

    def _run_pass(self, p: Pass) -> Any:
        """Position and measure one pass.

        Logs success or failure to the event log, then re-raises on error so
        a failed pass escalates (not silently continues).

        Args:
            p: Pass to execute.

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
        self.lab.place(p.instrument, list(p.start), speed=travel_speed)

        scanner = getattr(self.lab, p.instrument, None)
        try:
            if scanner is not None and hasattr(scanner, "acquire"):
                gantry = getattr(self.lab, "gantry", None)
                end_gantry = self.lab.frames.gantry_target_for(p.instrument, list(p.end))
                axis_index = _AXIS_INDEX[p.axis]
                scan = scanner.acquire(
                    gantry=gantry,
                    axis=p.axis,
                    end_mm=float(end_gantry[axis_index]),
                    **({"feed_rate_mm_s": p.scan_speed} if p.scan_speed else {}),
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
                    end=self._acquire_scan_end(p),
                    feed_rate_mm_s=p.scan_speed,
                    axis=p.axis,
                )
                result_note = f"file={result.path}"
        except Exception as exc:
            self.lab.event_log.log(
                self.lab.clock.elapsed(), p.instrument, "survey_pass",
                result=f"error: {exc}", notes=p.label or "",
            )
            raise
        self.lab.event_log.log(
            self.lab.clock.elapsed(), p.instrument, "survey_pass",
            result=result_note, notes=p.label or "",
        )
        return scan if scanner is not None and hasattr(scanner, "acquire") else result

    def _acquire_scan_end(self, p: Pass) -> List[float]:
        """Build acquire_scan()'s full per-axis `end` from an experiment-frame target.

        gantry_target_for() only returns X/Y/Z — Theta is outside the
        Cartesian frame model (see FrameRegistry) — but acquire_scan()
        requires one value per *configured* gantry axis, same as
        move_to()'s vector form. Any axis outside X/Y/Z (Theta) is
        backfilled with its live current position, same as move_to()'s
        keyword form leaves unspecified axes untouched.
        """
        xyz = self.lab.frames.gantry_target_for(p.instrument, list(p.end))
        by_name = {"X": float(xyz[0]), "Y": float(xyz[1]), "Z": float(xyz[2])}
        gantry = self.lab.gantry
        return [
            by_name[axis.name] if axis.name in by_name
            else gantry.cmd.get_actual_position(axis)
            for axis in gantry._axes
        ]


__all__ = ["Pass", "Survey", "Tile", "Traverse", "SurveyRunner"]
