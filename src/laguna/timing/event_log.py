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
    """Read the last well-formed row's event_id from an existing event log.

    Scans from the end so a resumed experiment's IDs continue past what's
    already in the file (never restart at 1 and collide with earlier rows)
    even if the very last row was left malformed by a crash mid-write.
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
        """Append one event row and flush to disk.

        Returns:
            This row's event_id — capture it to let a later note refer
            back to this specific event via log()'s refers_to.
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
        """Flush and close the underlying file."""
        with self._lock:
            self._fh.flush()
            self._fh.close()
