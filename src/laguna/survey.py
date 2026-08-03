"""Multi-pass surveys: cover an area, or repeat a line.

Everything in laguna is single-pass. ``TopographicProfiler.scan()`` runs one
transect, ``GocatorScanner.scan_with_gantry()`` runs one traverse. Anything
larger than the sensor's field of view, or repeated over the course of an
experiment, has been a hand-written loop in a script.

Two shapes cover almost all of it:

:class:`RasterSurvey` — tile a region with parallel passes, because the
Gocator's laser fan is ~2 m across but a flume bed is wider, and one pass only
captures a strip.

:class:`RepeatTransect` — re-run the same line, optionally with a different
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
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)


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
        feed_rate_mm_s: Traverse speed, if the plan fixed one.
        label: Human-readable description, for logs and progress.
    """

    index: int
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    instrument: str = "gocator"
    axis: str = "X"
    feed_rate_mm_s: Optional[float] = None
    label: str = ""

    @property
    def length_mm(self) -> float:
        return math.dist(self.start, self.end)

    def duration_s(self, feed_rate_mm_s: Optional[float] = None) -> Optional[float]:
        """How long this pass takes to traverse, if a feed rate is known."""
        rate = feed_rate_mm_s or self.feed_rate_mm_s
        if not rate:
            return None
        return self.length_mm / rate

    def to_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index,
            "start": list(self.start),
            "end": list(self.end),
            "instrument": self.instrument,
            "axis": self.axis,
            "feed_rate_mm_s": self.feed_rate_mm_s,
            "label": self.label,
        }


class Survey:
    """Base for anything that plans a list of passes."""

    def passes(self) -> List[Pass]:  # pragma: no cover - interface
        raise NotImplementedError

    def __iter__(self) -> Iterator[Pass]:
        return iter(self.passes())

    def __len__(self) -> int:
        return len(self.passes())

    def duration_s(self, feed_rate_mm_s: Optional[float] = None) -> Optional[float]:
        """Total traverse time, ignoring repositioning between passes.

        A lower bound rather than an estimate — the gantry still has to get
        from the end of one pass to the start of the next, and the sensor
        needs its settle time. Useful for "will this fit in the gap between
        scheduled events", which is the question that usually matters.
        """
        totals = [p.duration_s(feed_rate_mm_s) for p in self.passes()]
        if any(t is None for t in totals):
            return None
        return sum(totals)

    def describe(self) -> str:
        """Multi-line summary, for printing a plan before running it."""
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
class RasterSurvey(Survey):
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
        feed_rate_mm_s: Traverse speed for every pass.
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
    feed_rate_mm_s: Optional[float] = None

    _AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}

    def __post_init__(self) -> None:
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
        if self.axis not in self._AXIS_INDEX:
            raise ValueError(f"axis must be one of {list(self._AXIS_INDEX)}")
        if self.step_axis is None:
            self.step_axis = "Y" if self.axis == "X" else "X"
        if self.step_axis == self.axis:
            raise ValueError("step_axis must differ from axis")

    @property
    def pitch_mm(self) -> float:
        """Distance between adjacent pass centres."""
        return self.swath_mm * (1.0 - self.overlap)

    def passes(self) -> List[Pass]:
        travel_i = self._AXIS_INDEX[self.axis]
        step_i = self._AXIS_INDEX[self.step_axis]

        # Round up: a region 2.5 swaths wide needs 3 passes, and the last one
        # overlapping more than asked is fine — a gap is not.
        count = max(1, math.ceil(self.width_mm / self.pitch_mm))

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
                    feed_rate_mm_s=self.feed_rate_mm_s,
                    label=f"raster {i + 1}/{count}",
                )
            )
        return out

    def coverage_mm(self) -> float:
        """Total width actually imaged, which may exceed `width_mm`."""
        passes = self.passes()
        step_i = self._AXIS_INDEX[self.step_axis]
        first = passes[0].start[step_i]
        last = passes[-1].start[step_i]
        return (last - first) + self.swath_mm


@dataclass
class RepeatTransect(Survey):
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
        feed_rate_mm_s: Traverse speed.
    """

    start: Sequence[float]
    end: Sequence[float]
    instruments: Sequence[str] = ("gocator",)
    repeats: int = 1
    axis: str = "X"
    feed_rate_mm_s: Optional[float] = None

    def __post_init__(self) -> None:
        if self.repeats < 1:
            raise ValueError(f"repeats must be >= 1, got {self.repeats}")
        if not self.instruments:
            raise ValueError("at least one instrument is required")

    def passes(self) -> List[Pass]:
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
                        feed_rate_mm_s=self.feed_rate_mm_s,
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
            resumes where it left off rather than starting over. A raster of
            a wide bed can be the longest thing an experiment does.
    """

    def __init__(self, lab: Any, survey: Survey, checkpoint: Optional[Any] = None) -> None:
        self.lab = lab
        self.survey = survey
        self.checkpoint = checkpoint
        self.completed: List[int] = []

    def pending(self) -> List[Pass]:
        """Passes not yet done, according to the checkpoint."""
        if self.checkpoint is None:
            return list(self.survey.passes())
        return [p for p in self.survey.passes() if not self.checkpoint.is_complete(p.index)]

    def run(self, dry_run: bool = False) -> List[Pass]:
        """Execute the pending passes in order.

        Args:
            dry_run: Log the plan without moving or measuring.

        Returns:
            The passes actually executed.
        """
        done: List[Pass] = []
        for p in self.pending():
            logger.info(
                "Survey pass %d/%d — %s", p.index + 1, len(self.survey), p.label or p.instrument
            )
            if dry_run:
                done.append(p)
                continue
            self._run_pass(p)
            done.append(p)
            self.completed.append(p.index)
            if self.checkpoint is not None:
                self.checkpoint.mark_complete(
                    p.index, runtime_s=self.lab.clock.elapsed(),
                    wall_time=self.lab.clock.wall_time(), name=p.label,
                )
        return done

    def _run_pass(self, p: Pass) -> None:
        """Position for one pass and measure it."""
        # place() puts the INSTRUMENT's measuring point on the target, which
        # is what makes one plan valid for several instruments.
        self.lab.place(p.instrument, list(p.start), speed=p.feed_rate_mm_s)

        scanner = getattr(self.lab, p.instrument, None)
        if scanner is not None and hasattr(scanner, "acquire"):
            gantry = getattr(self.lab, "gantry", None)
            end_gantry = self.lab.frames.gantry_target_for(p.instrument, list(p.end))
            axis_index = {"X": 0, "Y": 1, "Z": 2}[p.axis]
            scanner.acquire(
                gantry=gantry,
                axis=p.axis,
                end_mm=float(end_gantry[axis_index]),
                **({"feed_rate_mm_s": p.feed_rate_mm_s} if p.feed_rate_mm_s else {}),
            )
        else:
            # No acquire() — a rangefinder transect goes through the
            # profiler path instead.
            self.lab.acquire_scan(
                p.instrument,
                start=None,
                end=list(self.lab.frames.gantry_target_for(p.instrument, list(p.end))),
                feed_rate_mm_s=p.feed_rate_mm_s,
            )


__all__ = ["Pass", "Survey", "RasterSurvey", "RepeatTransect", "SurveyRunner"]
