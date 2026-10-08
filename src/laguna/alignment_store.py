"""Save and reload the intermediate results of a Gocator alignment run.

An alignment run (``calibration/gocator_alignment_and_seam.ipynb``)
produces a few expensive-to-get results in sequence: the block's position in
gantry mm from a slow WTT12L scan, three Gocator passes over it, the corners
picked on those passes, and finally the solved mounting and translation.
Each is perishable in the sense that re-getting it means driving the rig
again. This module writes each one to the run's folder as soon as it exists
and reads it back later, so a rerun can start from a known block position, or
from saved passes, instead of re-scanning. Loading never commands anything.

Results live under ``calibration/results/`` (:data:`CALIBRATION_RESULTS_DIR`),
deliberately apart from ``data/``: they describe how the rig is set up, not
what an experiment measured, and are re-made whenever the setup changes.

A saved block position is only valid while the block hasn't moved: the files
record when they were written, and the caller owns the judgement about
whether that's still true.
"""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Union

import numpy as np

from .scanner.pointcloud import SurfaceScan

logger = logging.getLogger(__name__)

#: Where alignment/calibration runs are written, kept separate from experiment
#: data (``data/``) on purpose. Relative to the working directory, like ``data/``.
CALIBRATION_RESULTS_DIR = Path("calibration/results")

#: Names of the three alignment passes, in acquisition order.
PASS_NAMES = ("P1", "P2", "P3")

_BLOCK_FILE = "block_location.json"
_CORNERS_FILE = "block_corners.json"
_SOLUTION_FILE = "alignment.json"
_SOURCE_FILE = "loaded_from.json"


def _plain(value: Any) -> Any:
    """Make numpy scalars/arrays JSON-serialisable."""
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


class AlignmentStore:
    """One alignment run's folder: write results as they appear, read them back later.

    Args:
        directory: The run folder (e.g. ``calibration/results/seam_test_<time>``).
            Created on the first write, not on construction, so a store
            pointed at an old run for loading never creates anything.
    """

    def __init__(self, directory: Union[str, Path]) -> None:
        """Remember the folder; touch nothing."""
        self.directory = Path(directory)

    @classmethod
    def new_run(
        cls, root: Union[str, Path] = CALIBRATION_RESULTS_DIR, prefix: str = "seam_test"
    ) -> "AlignmentStore":
        """A store for a fresh, timestamped run folder under `root`.

        Nothing is created until the first write.

        Args:
            root: Folder holding run folders; defaults to
                :data:`CALIBRATION_RESULTS_DIR`.
            prefix: Run folder name prefix, matching ``latest()``'s default pattern.

        Returns:
            A store for ``<root>/<prefix>_<YYYYmmdd_HHMMSS>``.
        """
        return cls(Path(root) / time.strftime(f"{prefix}_%Y%m%d_%H%M%S"))

    # -- helpers -----------------------------------------------------------

    def _write_json(self, name: str, payload: Dict[str, Any]) -> Path:
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / name
        path.write_text(json.dumps(_plain(payload), indent=2))
        return path

    def _read_json(self, name: str) -> Optional[Dict[str, Any]]:
        path = self.directory / name
        if not path.exists():
            return None
        return json.loads(path.read_text())

    # -- the block's position (WTT12L scan) ----------------------------------

    def save_block(
        self,
        block_center_gantry: Sequence[float],
        transects: Optional[Dict[str, Dict[str, float]]] = None,
        block_size_mm: Optional[Sequence[float]] = None,
    ) -> Path:
        """Record the block's centre in gantry X/Y, as found by the WTT12L scan.

        Args:
            block_center_gantry: ``[x, y]`` block centre, gantry mm.
            transects: Per-axis ``{"X": {"center", "width", "lag"}, ...}``
                results, kept for the record.
            block_size_mm: Block ``(x, y)`` size the scan was run for.

        Returns:
            The written file's path.
        """
        return self._write_json(_BLOCK_FILE, {
            "block_center_gantry_mm": [float(v) for v in block_center_gantry][:2],
            "reverse_positions_fixed": True,
            "transects": transects or {},
            "block_size_mm": list(block_size_mm) if block_size_mm is not None else None,
            "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
        })

    def load_block(self) -> Optional[np.ndarray]:
        """The saved block centre, or None if this folder has none.

        Returns:
            ``(2,)`` array of gantry X/Y mm.
        """
        data = self._read_json(_BLOCK_FILE)
        if data is None:
            # Older runs only wrote it inside the solution file.
            data = self._read_json(_SOLUTION_FILE)
            if not data or "B_W" not in data:
                return None
            self._warn_if_unfixed(None)
            return np.asarray(data["B_W"], dtype=float)
        self._warn_if_unfixed(data)
        return np.asarray(data["block_center_gantry_mm"], dtype=float)

    def _warn_if_unfixed(self, block_file: Optional[Dict[str, Any]]) -> None:
        """Warn about block positions that predate the reverse-scan position fix.

        The WTT12L agent labelled every reverse-direction scan's positions as if
        the axis had moved forward (fixed 2026-10-08), so the out-and-back
        average of such a run lands near the reverse scan's start point instead
        of the block: wrong by roughly one scan half-length in each axis.
        """
        if block_file is None or not block_file.get("reverse_positions_fixed"):
            logger.warning(
                "block position in %s was saved before the reverse-scan position fix "
                "(2026-10-08) and is probably off by about half a scan length in each axis; "
                "re-scan, or give the block centre explicitly (B_W_KNOWN)", self.directory,
            )

    # -- the three Gocator passes --------------------------------------------

    def save_pass(self, name: str, scan: SurfaceScan) -> Path:
        """Write one alignment pass as ``align_<name>.npz``.

        Args:
            name: One of :data:`PASS_NAMES`.
            scan: The acquired pass.

        Returns:
            The written file's path.
        """
        if name not in PASS_NAMES:
            raise ValueError(f"pass name must be one of {PASS_NAMES}, got {name!r}")
        return scan.save_npz(self.directory / f"align_{name}.npz")

    def load_passes(self) -> Dict[str, SurfaceScan]:
        """The alignment passes saved here, by name. Missing ones are left out.

        Returns:
            ``{"P1": scan, ...}`` for every ``align_P*.npz`` present.
        """
        out: Dict[str, SurfaceScan] = {}
        for name in PASS_NAMES:
            path = self.directory / f"align_{name}.npz"
            if path.exists():
                out[name] = SurfaceScan.from_npz(path)
        return out

    # -- picked block corners ------------------------------------------------

    def save_corners(self, corners: Dict[str, Sequence[float]]) -> Path:
        """Record the corner-pick boxes ``{"P1": (x0, y0, x1, y1), ...}``."""
        return self._write_json(_CORNERS_FILE, {k: [float(v) for v in c] for k, c in corners.items()})

    def load_corners(self) -> Dict[str, tuple]:
        """The saved corner picks, or ``{}``.

        Picks are in the oriented frame of the passes they were made on, so
        they are only meaningful with those same passes loaded.
        """
        data = self._read_json(_CORNERS_FILE) or {}
        return {k: tuple(float(v) for v in c) for k, c in data.items()}

    # -- the solved alignment -------------------------------------------------

    def save_solution(
        self,
        block_center_gantry: Sequence[float],
        mounting: Dict[str, str],
        translation: Sequence[float],
        transects: Optional[Dict[str, Dict[str, float]]] = None,
    ) -> Path:
        """Record the solved mounting and translation for the run."""
        return self._write_json(_SOLUTION_FILE, {
            "B_W": [float(v) for v in block_center_gantry][:2],
            "mounting": dict(mounting),
            "translation": [float(v) for v in translation],
            "wtt12l": transects or {},
        })

    def load_solution(self) -> Optional[Dict[str, Any]]:
        """The saved solution dict, or None."""
        return self._read_json(_SOLUTION_FILE)

    # -- provenance ---------------------------------------------------------------

    def note_loaded_from(self, source: "AlignmentStore", what: Sequence[str]) -> Path:
        """Record in this run's folder which earlier run its inputs came from.

        Args:
            source: The store that was loaded from.
            what: Which results were reused (e.g. ``["block", "passes"]``).

        Returns:
            The written file's path.
        """
        return self._write_json(_SOURCE_FILE, {
            "source": str(source.directory),
            "reused": list(what),
            "when": time.strftime("%Y-%m-%d %H:%M:%S"),
        })

    @staticmethod
    def latest(
        root: Union[str, Path] = CALIBRATION_RESULTS_DIR,
        having: str = "block",
        pattern: str = "seam_test_*",
    ) -> Optional["AlignmentStore"]:
        """The most recent run folder under `root` that has a given result.

        Args:
            root: Folder holding run folders; defaults to :data:`CALIBRATION_RESULTS_DIR`.
            having: ``"block"``, ``"passes"`` or ``"solution"``.
            pattern: Glob for run folder names.

        Returns:
            A store for it, or None if nothing matches.
        """
        checks = {
            "block": lambda s: s.load_block() is not None,
            "passes": lambda s: len(s.load_passes()) == len(PASS_NAMES),
            "solution": lambda s: s.load_solution() is not None,
        }
        if having not in checks:
            raise ValueError(f"having must be one of {sorted(checks)}, got {having!r}")
        for folder in sorted(Path(root).glob(pattern), reverse=True):
            if folder.is_dir():
                store = AlignmentStore(folder)
                if checks[having](store):
                    return store
        return None
