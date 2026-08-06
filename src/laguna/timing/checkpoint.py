"""Checkpoint store — persists completed experiment events to disk for crash recovery."""

import json
import os
from pathlib import Path
from typing import List, Optional


class CheckpointStore:
    """Records completed experiment events to a JSON file for crash recovery.

    On restart with resume=True, previously completed events are reloaded so
    the experiment loop can skip them with is_complete(). Writes are atomic
    (write to .tmp then os.replace) so a crash mid-write cannot corrupt the
    checkpoint file. Event IDs are arbitrary integers assigned by the caller —
    typically the index of the event in a sequence. An optional name field is
    available for human-readable labels and is the hook point for future
    phase-level checkpointing.
    """

    def __init__(self, path: str, resume: bool = False) -> None:
        """Initialize the checkpoint store.

        Args:
            path: File path for the JSON checkpoint file.
            resume: If True, load existing events from path if it exists;
                if False, clear any existing file and start fresh.
        """
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
        """Record that event_id completed and persist to disk.

        Args:
            event_id: Arbitrary integer identifying this event (typically a sequence index).
            runtime_s: Experiment runtime in seconds when the event completed.
            wall_time: Unix wall time when the event completed.
            name: Optional human-readable label for this event.
        """
        self._events.append(
            {"id": event_id, "runtime_s": runtime_s, "wall_time": wall_time, "name": name}
        )
        self._write()

    def is_complete(self, event_id: int) -> bool:
        """Check if event_id has already been marked complete.

        Args:
            event_id: Event identifier to check.

        Returns:
            True if the event has been marked complete, False otherwise.
        """
        return any(e["id"] == event_id for e in self._events)

    def last_completed(self) -> Optional[int]:
        """Return the highest event_id marked complete, or None if none exist.

        Returns:
            The maximum event_id that has been marked complete, or None if
            no events have been completed.
        """
        if not self._events:
            return None
        return max(e["id"] for e in self._events)

    def clear(self) -> None:
        """Wipe all checkpoint state and delete the file."""
        self._events = []
        if self._path.exists():
            self._path.unlink()

    def _write(self) -> None:
        """Write checkpoint state to disk atomically via tmp+rename."""
        tmp = self._path.with_suffix(".tmp")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps({"events": self._events}, indent=2))
        os.replace(tmp, self._path)
