"""Checkpoint store — persists completed experiment events to disk for crash recovery."""

import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any, List, Optional

logger = logging.getLogger(__name__)


class CheckpointCorruptError(RuntimeError):
    """A checkpoint file exists but can't be read. It has been set aside, not deleted."""


class CheckpointStore:
    """Records completed experiment events to a JSON file for crash recovery.

    On restart with resume=True, previously completed events are reloaded so
    the experiment loop can skip them with is_complete(). Writes are atomic
    and durable (write to a temp file, fsync, then os.replace) so neither a
    crash nor a power cut mid-write can leave a truncated checkpoint. Event
    IDs are arbitrary integers assigned by the caller — typically the index
    of the event in a sequence. An optional name field is available for
    human-readable labels and is the hook point for future phase-level
    checkpointing.

    A checkpoint is the only record of which passes of a crashed run already
    completed, so this class never deletes one. Starting fresh (resume=False)
    or clear() moves an existing file aside to
    ``<name>.<YYYYmmdd-HHMMSS>.bak``; an unreadable one is moved to
    ``<name>.<YYYYmmdd-HHMMSS>.corrupt`` and reported, rather than silently
    treated as empty (which used to re-run every completed pass and then
    overwrite the evidence).
    """

    def __init__(self, path: str, resume: bool = False) -> None:
        """Initialize the checkpoint store.

        Args:
            path: File path for the JSON checkpoint file.
            resume: If True, load existing events from path if it exists;
                if False, set any existing file aside (see the class
                docstring) and start fresh.

        Raises:
            CheckpointCorruptError: resume=True and the existing file can't
                be parsed. It has been moved aside; inspect it before
                deciding which passes to re-run.
        """
        self._path = Path(path)
        self._events: List[dict] = []
        self._meta: dict = {}
        self._lock = threading.Lock()

        if resume and self._path.exists():
            try:
                data = json.loads(self._path.read_text())
                events = data["events"]
                if not isinstance(events, list):
                    raise TypeError(f"'events' is a {type(events).__name__}, not a list")
                self._events = events
                meta = data.get("meta", {})
                if not isinstance(meta, dict):
                    raise TypeError(f"'meta' is a {type(meta).__name__}, not a dict")
                self._meta = meta
            except (json.JSONDecodeError, KeyError, TypeError, AttributeError) as exc:
                kept = self._set_aside("corrupt")
                raise CheckpointCorruptError(
                    f"Checkpoint {self._path} is unreadable ({exc}); moved to {kept}. "
                    "Nothing was deleted — check it to see which passes completed."
                ) from exc
        elif not resume and self._path.exists():
            kept = self._set_aside("bak")
            logger.warning(
                "Starting a fresh checkpoint; the previous one was moved to %s "
                "(pass resume=True to continue from it instead)", kept,
            )

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
        with self._lock:
            self._events.append(
                {"id": event_id, "runtime_s": runtime_s, "wall_time": wall_time, "name": name}
            )
            self._write()

    @property
    def meta(self) -> dict:
        """A copy of the free-form metadata stored alongside the events.

        Used to record what the events *mean* — e.g. ``SurveyRunner`` stores
        a fingerprint of the survey geometry so a resume against a changed
        plan is caught instead of trusting stale pass indices.
        """
        with self._lock:
            return dict(self._meta)

    def set_meta(self, key: str, value: Any) -> None:
        """Store a JSON-serializable metadata value and persist to disk.

        Args:
            key: Metadata key.
            value: JSON-serializable value.
        """
        with self._lock:
            self._meta[key] = value
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
        """Forget all checkpoint state, moving the file aside rather than deleting it."""
        with self._lock:
            self._events = []
            self._meta = {}
            if self._path.exists():
                kept = self._set_aside("bak")
                logger.warning("Checkpoint cleared; the previous one was moved to %s", kept)

    def _set_aside(self, kind: str) -> Path:
        """Rename the checkpoint file to a timestamped sibling; return its new path."""
        stamp = time.strftime("%Y%m%d-%H%M%S")
        target = self._path.with_name(f"{self._path.name}.{stamp}.{kind}")
        n = 1
        while target.exists():
            target = self._path.with_name(f"{self._path.name}.{stamp}-{n}.{kind}")
            n += 1
        os.replace(self._path, target)
        return target

    def _write(self) -> None:
        """Write checkpoint state to disk atomically and durably via tmp + fsync + rename."""
        tmp = self._path.with_name(self._path.name + ".tmp")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        with open(tmp, "w") as f:
            f.write(json.dumps({"events": self._events, "meta": self._meta}, indent=2))
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self._path)
