"""Tie one experiment run's outputs together.

Every subsystem timestamps in UTC wall clock and writes wherever its own
config section says. The Gocator drops surfaces in ``gocator.output_dir``,
the profiler pulls CSVs into whatever the caller passed, cameras land under
``pi_cameras.output_dir``, and the event log sits in the working directory
appending across runs with no separator. There is no run identifier anywhere.

So a scan in ``data/scans`` and a capture in ``captures/pi`` cannot be
attributed to the same run except by eyeballing wall-clock windows against
``experiment_events.csv`` — and finding the right ``experiment_start`` /
``experiment_stop`` pair by hand first.

Worse, **the runtime-to-wall mapping does not survive the process.**
``ExperimentClock`` keeps ``_start_wall`` and ``_pause_offset`` privately and
they die on exit. Pauses make the mapping piecewise, so it cannot even be
reconstructed from a single start time: 600 s of runtime is a different wall
instant depending on how long the run spent paused before it. Since
:meth:`FlumeLab.pause` now stops the clock, this is not a corner case.

:class:`RunContext` fixes all three: it mints a run id, gives the run a
directory, and writes ``run.json`` with the pause intervals needed to convert
between the two timelines afterwards.

The manifest looks like::

    {
      "run_id": "20260803T101530Z-a4f1",
      "started_wall": 1785701730.5,
      "ended_wall": 1785705330.9,
      "pauses": [[1785702000.0, 1785702300.0]],
      "outputs": [{"subsystem": "gocator", "path": "...", "runtime_s": 612.3}]
    }
"""

from __future__ import annotations

import json
import logging
import os
import secrets
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

MANIFEST_NAME = "run.json"


def new_run_id() -> str:
    """A sortable, collision-resistant run identifier.

    UTC timestamp so runs sort chronologically by name, plus four random hex
    characters because two runs started in the same second — a rerun after an
    immediate failure, say — must not share an id.
    """
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{stamp}-{secrets.token_hex(2)}"


class RunContext:
    """One experiment run: an id, a directory, and a recoverable timeline.

    Args:
        root: Directory to create the run directory beneath. None disables
            the run directory entirely, leaving each subsystem's configured
            ``output_dir`` untouched — which is the default, so nothing moves
            for a rig that has not opted in.
        run_id: Override the generated id.
    """

    def __init__(
        self,
        root: Optional[str] = None,
        run_id: Optional[str] = None,
        speed_factor: float = 1.0,
    ) -> None:
        self.run_id = run_id or new_run_id()
        #: Experiment seconds per real second, so runtime_at()/wall_at() stay
        #: correct for an accelerated rehearsal. Recorded in the manifest
        #: because a file written during one is otherwise indistinguishable
        #: from a real run's, and its timestamps would convert wrongly.
        self.speed_factor = float(speed_factor)
        self.root = Path(root) if root else None
        self.started_wall: Optional[float] = None
        self.ended_wall: Optional[float] = None
        #: (paused_at_wall, resumed_at_wall); the tail may be open.
        self.pauses: List[List[Optional[float]]] = []
        self.outputs: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------

    @property
    def directory(self) -> Optional[Path]:
        """This run's directory, created on first use. None if not enabled."""
        if self.root is None:
            return None
        path = self.root / self.run_id
        path.mkdir(parents=True, exist_ok=True)
        return path

    def path_for(self, subsystem: str, default_dir: str) -> Path:
        """Where `subsystem` should write.

        Returns a subdirectory of the run directory when one is configured,
        and otherwise the subsystem's own ``default_dir`` unchanged — so
        enabling run directories is opt-in and reversible.
        """
        directory = self.directory
        if directory is None:
            return Path(default_dir)
        path = directory / subsystem
        path.mkdir(parents=True, exist_ok=True)
        return path

    # ------------------------------------------------------------------
    # Timeline
    # ------------------------------------------------------------------

    def started(self) -> None:
        self.started_wall = time.time()
        self.write()

    def paused(self) -> None:
        # Guard against a double pause leaving two open intervals, which
        # would make the mapping ambiguous rather than merely wrong.
        if self.pauses and self.pauses[-1][1] is None:
            return
        self.pauses.append([time.time(), None])
        self.write()

    def resumed(self) -> None:
        if self.pauses and self.pauses[-1][1] is None:
            self.pauses[-1][1] = time.time()
            self.write()

    def ended(self) -> None:
        self.ended_wall = time.time()
        if self.pauses and self.pauses[-1][1] is None:
            self.pauses[-1][1] = self.ended_wall
        self.write()

    def paused_before(self, wall: float) -> float:
        """Total seconds spent paused before a wall-clock instant."""
        total = 0.0
        for start, end in self.pauses:
            if start >= wall:
                break
            total += min(end if end is not None else wall, wall) - start
        return total

    def runtime_at(self, wall: float) -> float:
        """Experiment runtime corresponding to a wall-clock instant.

        The conversion the manifest exists for: given a scan file's UTC
        timestamp, what was the experiment runtime? Piecewise, because every
        pause shifts the two timelines apart.

        Raises:
            RuntimeError: If the run never recorded a start.
        """
        if self.started_wall is None:
            raise RuntimeError("this run has no recorded start time")
        return (wall - self.started_wall - self.paused_before(wall)) * self.speed_factor

    def wall_at(self, runtime_s: float) -> float:
        """Wall-clock instant corresponding to an experiment runtime.

        The inverse of :meth:`runtime_at`. Walks the pause intervals forward
        rather than inverting analytically, since the mapping is a step
        function and a closed form would have to special-case landing inside
        a pause.
        """
        if self.started_wall is None:
            raise RuntimeError("this run has no recorded start time")
        wall = self.started_wall + runtime_s / self.speed_factor
        for start, end in self.pauses:
            if start >= wall:
                break
            wall += (end if end is not None else start) - start
        return wall

    # ------------------------------------------------------------------

    def record_output(
        self,
        subsystem: str,
        path: Any,
        runtime_s: Optional[float] = None,
        **extra: Any,
    ) -> None:
        """Note a file this run produced, for the manifest."""
        self.outputs.append(
            {
                "subsystem": subsystem,
                "path": str(path),
                "runtime_s": runtime_s,
                "wall_time": time.time(),
                **extra,
            }
        )
        self.write()

    def stamp(self, runtime_s: Optional[float] = None) -> Dict[str, Any]:
        """Fields to embed in a subsystem's own metadata.

        Both timelines, so a file is self-describing even if the manifest is
        ever separated from it.
        """
        return {
            "run_id": self.run_id,
            "runtime_s": runtime_s,
            "wall_time": time.time(),
        }

    def to_dict(self) -> Dict[str, Any]:
        return {
            "run_id": self.run_id,
            "speed_factor": self.speed_factor,
            "started_wall": self.started_wall,
            "ended_wall": self.ended_wall,
            "pauses": [list(p) for p in self.pauses],
            "outputs": self.outputs,
        }

    def write(self) -> Optional[Path]:
        """Persist the manifest. Atomic, and never raises.

        Written on every change rather than at the end, so a run killed
        mid-flight still leaves a usable timeline — which is exactly the case
        where reconstructing one by hand is hardest.
        """
        try:
            directory = self.directory
            if directory is None:
                return None
            path = directory / MANIFEST_NAME
            tmp = path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.to_dict(), indent=2))
            os.replace(tmp, path)
            return path
        except Exception as exc:
            # Broad on purpose, and the mkdir is inside the try: a bad root
            # raises ValueError rather than OSError. Bookkeeping trouble must
            # never take down a running experiment.
            logger.warning("Could not write the run manifest: %s", exc)
            return None

    @classmethod
    def load(cls, path: str) -> "RunContext":
        """Read a manifest back, for after-the-fact analysis.

        Accepts either the ``run.json`` itself or the directory holding it.
        """
        p = Path(path)
        if p.is_dir():
            p = p / MANIFEST_NAME
        data = json.loads(p.read_text())
        ctx = cls(root=str(p.parent.parent), run_id=data["run_id"],
                  speed_factor=data.get("speed_factor", 1.0))
        ctx.started_wall = data.get("started_wall")
        ctx.ended_wall = data.get("ended_wall")
        ctx.pauses = [list(x) for x in data.get("pauses", [])]
        ctx.outputs = list(data.get("outputs", []))
        return ctx

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"RunContext({self.run_id!r}, outputs={len(self.outputs)})"


__all__ = ["RunContext", "new_run_id", "MANIFEST_NAME"]
