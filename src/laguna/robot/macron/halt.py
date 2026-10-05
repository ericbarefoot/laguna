"""The gantry's halt latch: once pause/stop/estop fires, motion stays refused.

Two defects motivated this, both of the "motion commandable from a stop
condition" kind CLAUDE.md puts first:

- Nothing at the controller level remembered a halt. ``estop()`` aborted
  every axis, engaged the brakes and cut the motors, and the very next
  ``move_to()`` sent ``BMT`` straight into them — a stall against an engaged
  brake is what corrupted Y's encoder on 2026-07-30.
- A halt didn't cancel a move already in flight. ``pause()`` sent ``BST``
  from the caller's thread, but the thread executing a multi-leg move just
  saw "move finished" and issued its next leg.

:class:`HaltLatch` fixes both. Tripping it bumps a generation counter and
records the most severe tier seen; it stays latched until explicitly
cleared (``resume()`` for a pause, ``rearm()`` for stop/estop). A move
snapshots the generation into a :class:`MotionGuard` when it starts and
checks it before every command that starts motion and between every poll —
if a halt fired since, it raises :class:`MotionHalted` instead.

Why a lock, not just a counter: "check, then send ``BMT``" is two steps. A
halt landing between them would bump the counter after the check passed,
send ``BST`` before the ``BMT`` reached the wire, and the ``BMT`` would then
start a fresh move that nothing stops. :meth:`MotionGuard.issuing` holds the
latch's lock across both steps, and :meth:`HaltLatch.trip` takes the same
lock to bump the counter — so a ``BMT`` either goes out before the trip (and
the halt's own ``BST``/``ABT``, sent afterwards, stops it) or is refused.
The lock is only ever held for one command's round trip, and ``trip()``
gives up waiting after a bounded timeout rather than delay a halt behind a
hung transport.
"""

from __future__ import annotations

import enum
import logging
import threading
from contextlib import contextmanager
from typing import Iterator, Optional

logger = logging.getLogger(__name__)

#: Longest trip() waits for an in-flight motion command to finish being
#: issued. Past this, it bumps the generation anyway — a halt must never
#: queue behind a hung transport. The halt's own stop commands follow, so a
#: BMT that slips through in that window is still stopped.
TRIP_LOCK_TIMEOUT_S = 1.0


class MotionHalted(RuntimeError):
    """Motion was refused or cancelled because the gantry is halted."""


class HaltLevel(enum.IntEnum):
    """Halt severity, ordered so a lesser trip never downgrades a greater one."""

    PAUSE = 1
    STOP = 2
    ESTOP = 3


class HaltLatch:
    """Remembers the most severe halt since the last clear, and invalidates in-flight moves."""

    def __init__(self) -> None:
        """Create an untripped latch."""
        self._lock = threading.Lock()
        self._generation = 0
        self._level: Optional[HaltLevel] = None
        self._reason: Optional[str] = None

    @property
    def level(self) -> Optional[HaltLevel]:
        """The latched halt tier, or None if motion is allowed."""
        return self._level

    @property
    def reason(self) -> Optional[str]:
        """What tripped the latch, for error messages."""
        return self._reason

    def trip(self, level: HaltLevel, reason: str) -> None:
        """Latch `level` (or keep a more severe one) and cancel every in-flight move.

        Args:
            level: The halt tier being applied.
            reason: Human-readable cause, surfaced by :class:`MotionHalted`.
        """
        acquired = self._lock.acquire(timeout=TRIP_LOCK_TIMEOUT_S)
        if not acquired:
            logger.warning(
                "Halt latch lock still held after %.1fs — tripping without it",
                TRIP_LOCK_TIMEOUT_S,
            )
        try:
            self._generation += 1
            if self._level is None or level > self._level:
                self._level = level
                self._reason = reason
        finally:
            if acquired:
                self._lock.release()

    def clear(self, up_to: HaltLevel) -> bool:
        """Clear the latch if it is no more severe than `up_to`.

        Args:
            up_to: The most severe tier this caller is allowed to clear —
                ``resume()`` passes PAUSE, ``rearm()`` passes ESTOP.

        Returns:
            True if the latch is now clear, False if a more severe halt is
            still latched.
        """
        with self._lock:
            if self._level is not None and self._level > up_to:
                return False
            self._level = None
            self._reason = None
            return True

    def require_clear(self, description: str) -> None:
        """Raise :class:`MotionHalted` if the latch is tripped.

        Args:
            description: The motion being attempted, for the error message.
        """
        if self._level is not None:
            raise MotionHalted(
                f"{description} refused: gantry is halted ({self._level.name.lower()}: "
                f"{self._reason}). "
                + (
                    "Call resume() to continue after a pause."
                    if self._level is HaltLevel.PAUSE
                    else "Call rearm() once the cause is cleared."
                )
            )

    def guard(self, description: str) -> "MotionGuard":
        """Snapshot the current generation for a move that is about to start.

        Args:
            description: The motion being started, for error messages.

        Raises:
            MotionHalted: If the latch is already tripped.
        """
        self.require_clear(description)
        return MotionGuard(self, self._generation, description)


class MotionGuard:
    """One move's view of the latch: raises once any halt has fired since it started."""

    def __init__(self, latch: HaltLatch, generation: int, description: str) -> None:
        """Bind to `latch` at `generation`. Use :meth:`HaltLatch.guard` instead."""
        self._latch = latch
        self._generation = generation
        self._description = description

    def check(self) -> None:
        """Raise :class:`MotionHalted` if a halt has fired since this move started."""
        if self._latch._generation != self._generation:
            raise MotionHalted(
                f"{self._description} cancelled: gantry was halted mid-move "
                f"({self._latch.reason})"
            )

    @contextmanager
    def issuing(self) -> Iterator[None]:
        """Hold the latch across "check, then send a motion-starting command".

        See the module docstring for the race this closes.
        """
        with self._latch._lock:
            self.check()
            yield


__all__ = [
    "HaltLatch",
    "HaltLevel",
    "MotionGuard",
    "MotionHalted",
    "TRIP_LOCK_TIMEOUT_S",
]
