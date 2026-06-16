"""Event log — append-only CSV record of all experiment events."""

import csv
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


_HEADER = ["wall_time_iso", "wall_time_unix", "runtime_s", "subsystem", "event_type", "result", "notes"]


class EventLog:
    """Thread-safe append-only CSV log of experiment events.

    Each call to log() writes one row immediately and flushes to disk.
    The file is created (with a header) on first open if it does not exist;
    if it already exists the header is not re-written, so logs from resumed
    experiments accumulate in the same file.

    Columns:
        wall_time_iso   — ISO-8601 UTC timestamp
        wall_time_unix  — Unix epoch float
        runtime_s       — experiment runtime at time of event
        subsystem       — which subsystem generated the event
        event_type      — name/type of the event
        result          — "ok", "failed", or custom string
        notes           — freeform additional context
    """

    def __init__(self, path: str) -> None:
        self._path = Path(path)
        self._lock = threading.Lock()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self._path.exists()
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
    ) -> None:
        """Append one event row and flush to disk."""
        now = time.time()
        iso = datetime.fromtimestamp(now, tz=timezone.utc).isoformat()
        with self._lock:
            self._writer.writerow([iso, f"{now:.6f}", f"{runtime_s:.3f}", subsystem, event_type, result, notes])
            self._fh.flush()

    def close(self) -> None:
        """Flush and close the underlying file."""
        with self._lock:
            self._fh.flush()
            self._fh.close()
