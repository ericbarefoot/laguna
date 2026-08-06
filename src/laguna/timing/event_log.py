"""Event log — append-only CSV record of all experiment events."""

import csv
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


_HEADER = [
    "event_id", "wall_time_iso", "wall_time_unix", "runtime_s",
    "subsystem", "event_type", "result", "notes", "refers_to",
]


def _last_event_id(path: Path) -> int:
    """Read the last well-formed event_id from an existing event log.

    Scans from the end to find the most recent valid row, so resumed
    experiments continue ID numbering past existing rows (never restart at 1
    and collide) even if the final row was left malformed by a crash.

    Args:
        path: Path to the event log file.

    Returns:
        The highest event_id in the file, or 0 if none found or no valid rows.
    """
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    for row in reversed(rows):
        if row:
            try:
                return int(row[0])
            except ValueError:
                continue
    return 0


class EventLog:
    """Thread-safe append-only CSV log of experiment events.

    Each call to log() writes one row immediately and flushes to disk.
    The file is created (with a header) on first open if it does not exist;
    if it already exists the header is not re-written, so logs from resumed
    experiments accumulate in the same file, and event_id continues from
    the last row already there rather than restarting at 1.

    This is the archival record — the narrative that ships alongside a
    published experiment's data, so it stays terse and human-readable by
    design: state-changing actions and milestones only ("weir moved to
    300mm", "scan completed"), not passive polling/status reads or
    low-level operational detail (connections, individual motion segments)
    — see laguna.subsystem_logging's module docstring for that split.

    Columns:
        event_id        — 1-based, monotonic per file (not per process —
                           see _last_event_id)
        wall_time_iso    — ISO-8601 UTC timestamp
        wall_time_unix   — Unix epoch float
        runtime_s        — experiment runtime at time of event
        subsystem        — which subsystem generated the event
        event_type       — name/type of the event
        result           — "ok", "failed", or custom string
        notes            — freeform additional context
        refers_to        — event_id of a prior row this one explains/
                           annotates (e.g. an operator note added after a
                           pause), or empty
    """

    def __init__(self, path: str) -> None:
        """Initialize the event log.

        Creates the file with a header row on first open; if the file already
        exists, appends to it without re-writing the header so resumed
        experiments accumulate in the same log.

        Args:
            path: File path for the CSV event log.
        """
        self._path = Path(path)
        self._lock = threading.Lock()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self._path.exists()
        self._next_id = 1 if write_header else _last_event_id(self._path) + 1
        self._fh = open(self._path, "a", newline="")
        self._writer = csv.writer(self._fh)
        if write_header:
            self._writer.writerow(_HEADER)
            self._fh.flush()

    def log(
        self,
        runtime_s: float,
        subsystem: str,
        event_type: str,
        result: str = "ok",
        notes: str = "",
        refers_to: Optional[int] = None,
    ) -> int:
        """Append one event row to the log and flush to disk.

        Args:
            runtime_s: Experiment runtime in seconds when the event occurred.
            subsystem: Name of the subsystem generating this event.
            event_type: Type/name of the event.
            result: Status string, typically "ok" or "failed" or an error.
            notes: Optional freeform context about the event.
            refers_to: Optional event_id of a prior row this event explains
                or annotates (e.g., an operator note added after a pause).

        Returns:
            This row's event_id; capture it to let later notes refer back
            to this specific event via the refers_to parameter.
        """
        now = time.time()
        iso = datetime.fromtimestamp(now, tz=timezone.utc).isoformat()
        with self._lock:
            event_id = self._next_id
            self._next_id += 1
            self._writer.writerow([
                event_id, iso, f"{now:.6f}", f"{runtime_s:.3f}", subsystem,
                event_type, result, notes, "" if refers_to is None else refers_to,
            ])
            self._fh.flush()
        return event_id

    def close(self) -> None:
        """Flush and close the event log file."""
        with self._lock:
            self._fh.flush()
            self._fh.close()
