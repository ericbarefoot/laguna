"""Checkpoint store — persists completed experiment events to disk for crash recovery."""

import json
import os
from pathlib import Path
from typing import List, Optional


class CheckpointStore:
    """Records completed experiment events to a JSON file.

    On restart with resume=True, previously completed events are reloaded so
    the experiment loop can skip them with is_complete().

    Writes are atomic (write to .tmp then os.replace) so a crash mid-write
    cannot corrupt the checkpoint file.

    Event IDs are arbitrary integers assigned by the caller — typically the
    index of the event in a sequence.  An optional name field is available
    for human-readable labels and is the hook point for future phase-level
    checkpointing.

    Example:
        store = CheckpointStore("experiment.json", resume=True)
        for i, t in enumerate(capture_times):
            if store.is_complete(i):
                continue
            clock.wait_until(t)
            cameras.trigger_capture()
            store.mark_complete(i, runtime_s=clock.elapsed())
    """

    def __init__(self, path: str, resume: bool = False) -> None:
        self._path = Path(path)
        self._events: List[dict] = []

        if resume and self._path.exists():
            try:
                data = json.loads(self._path.read_text())
                self._events = data.get("events", [])
            except (json.JSONDecodeError, KeyError):
                self._events = []
        elif not resume:
            # Fresh start — remove any stale file
            if self._path.exists():
                self._path.unlink()

    def mark_complete(
        self,
        event_id: int,
        runtime_s: float,
        wall_time: float,
        name: str = "",
    ) -> None:
        """Record that event_id completed and persist to disk."""
        self._events.append(
            {"id": event_id, "runtime_s": runtime_s, "wall_time": wall_time, "name": name}
        )
        self._write()

    def is_complete(self, event_id: int) -> bool:
        """Return True if event_id has already been marked complete."""
        return any(e["id"] == event_id for e in self._events)

    def last_completed(self) -> Optional[int]:
        """Return the highest completed event_id, or None if none yet."""
        if not self._events:
            return None
        return max(e["id"] for e in self._events)

    def clear(self) -> None:
        """Wipe all state and delete the checkpoint file."""
        self._events = []
        if self._path.exists():
            self._path.unlink()

    def _write(self) -> None:
        tmp = self._path.with_suffix(".tmp")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps({"events": self._events}, indent=2))
        os.replace(tmp, self._path)
