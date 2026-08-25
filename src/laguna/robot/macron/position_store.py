"""Persists the gantry's last-known axis positions across power cycles.

The OEM-2T's position registers (ACP) live only in the PLC's volatile state —
a power cycle wipes them. This gives a faster path back to a known position
than re-running home() every time.

This is a last-known-value cache, not a substitute for homing: it is exactly
as accurate as "nothing moved an axis between the last write and the power
cycle." See GantryController.restore_last_position()'s docstring for why
applying it is a deliberate, explicit call rather than automatic.

Same atomic-write idiom as laguna.timing.checkpoint.CheckpointStore: write to
a .tmp file, then os.replace() so a crash mid-write can't corrupt it.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional


class GantryPositionStore:
    """Reads/writes {axis_name: position_mm} to a JSON file."""

    def __init__(self, path: str) -> None:
        """Initialize position store at the given file path.

        Args:
            path: Filesystem path to the JSON checkpoint file.
        """
        self._path = Path(path)

    def save(self, positions: Dict[str, float]) -> None:
        """Atomically persist `positions`, tagged with the current wall time.

        Args:
            positions: Dictionary mapping axis names to position values in mm.

        Raises:
            OSError: If file write fails.
        """
        tmp = self._path.with_suffix(".tmp")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps({"positions": positions, "wall_time": time.time()}, indent=2))
        os.replace(tmp, self._path)

    def load(self) -> Optional[Dict[str, Any]]:
        """Return the saved snapshot, or None if there isn't a usable one.

        Yields ``{"positions": {...}, "wall_time": ...}``. Missing,
        unreadable and malformed files all return None — callers treat them
        identically, since each means "no checkpoint to restore".
        """
        if not self._path.exists():
            return None
        try:
            data = json.loads(self._path.read_text())
        except (json.JSONDecodeError, OSError):
            return None
        if not isinstance(data, dict) or "positions" not in data:
            return None
        return data
