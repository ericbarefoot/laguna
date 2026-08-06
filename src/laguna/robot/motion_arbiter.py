"""One lock over the gantry, so two things can't drive it at once.

``Scheduler._fire()`` dispatches every scheduled action in its own daemon
thread with no coordination between them. That is harmless only while nothing
schedulable moves the gantry — the moment a Gocator scan or a rangefinder
transect becomes a scheduled action (see
``laguna.experiment.runner.setup_run``), two threads can be commanding motion
at the same time.

The transports already serialise individual commands: ``PiGantryConnection``
holds a lock across each request/response pair, so bytes never interleave on
the wire. But that is per *command*, not per *operation*. A scan is hundreds
of commands with a physical traverse in between, and nothing stopped a
``move_to()`` from another thread landing in the middle of one. Both would
succeed on the wire and the gantry would go somewhere neither caller intended.

This module adds the missing layer: a re-entrant lock held for the duration of
a whole motion operation.

Re-entrant on purpose — ``scan_with_gantry()`` acquires it and then calls
``AxisHandle.begin_move_to()``, which acquires it again on the same thread.
A plain :class:`threading.Lock` would deadlock on that.

Timeouts, not blocking forever. A scheduled action that cannot get the gantry
should fail loudly and let the scheduler log it, rather than piling up threads
that all wake later and move the machine in some unpredictable order.
"""

from __future__ import annotations

import logging
import threading
from contextlib import contextmanager
from typing import Iterator, Optional

logger = logging.getLogger(__name__)

#: How long to wait for the gantry by default, seconds. Generous enough to
#: outlast a normal move, short enough that a wedged holder surfaces quickly.
DEFAULT_TIMEOUT_S = 120.0


class MotionBusyError(RuntimeError):
    """The gantry was already in use and did not free up in time.

    Names the current holder, because "resource busy" with no attribution is
    close to useless when the contender is a background scheduler thread.
    """


class MotionArbiter:
    """Serialises whole motion operations on one gantry.

    Args:
        timeout_s: Default seconds to wait before raising
            :class:`MotionBusyError`.
    """

    def __init__(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        """Create an arbiter with no current holder.

        Args:
            timeout_s: Default seconds to wait for the gantry before raising
                :class:`MotionBusyError`.
        """
        self._lock = threading.RLock()
        self._timeout_s = timeout_s
        self._holder: Optional[str] = None
        self._depth = 0

    @property
    def holder(self) -> Optional[str]:
        """Description of whatever currently holds the gantry, or None."""
        return self._holder

    @property
    def is_held(self) -> bool:
        """True if any thread currently holds the gantry."""
        return self._holder is not None

    @contextmanager
    def hold(
        self,
        description: str,
        timeout_s: Optional[float] = None,
    ) -> Iterator[None]:
        """Hold the gantry for the duration of the block.

        Re-entrant on the same thread, so nesting (``scan_with_gantry`` ->
        ``begin_move_to``) is fine and only the outermost description is
        reported as the holder.

        Args:
            description: What is being done, e.g. ``"scan X 0->200mm"``. This
                is what a blocked caller sees, so make it identifiable.
            timeout_s: Override the default wait.

        Raises:
            MotionBusyError: If another thread holds it beyond the timeout.
        """
        timeout = self._timeout_s if timeout_s is None else timeout_s
        if not self._lock.acquire(timeout=timeout):
            raise MotionBusyError(
                f"Could not start {description!r}: the gantry is already in use "
                f"by {self._holder!r} and did not free up within {timeout:.0f}s. "
                "Scheduled motion actions serialise through a single arbiter — "
                "either the holder is stuck, or two actions are scheduled too "
                "close together for both to run."
            )
        outermost = self._depth == 0
        self._depth += 1
        if outermost:
            self._holder = description
            logger.debug("Gantry acquired for %s", description)
        try:
            yield
        finally:
            self._depth -= 1
            if self._depth == 0:
                self._holder = None
                logger.debug("Gantry released after %s", description)
            self._lock.release()


#: Process-wide arbiter. There is one physical gantry, and the subsystems that
#: drive it (GantryController, GocatorScanner, TopographicProfiler) are
#: constructed independently and have no reference to each other — so a shared
#: default is what actually makes the guarantee hold. Pass an explicit arbiter
#: instead if a process ever drives two machines.
DEFAULT_ARBITER = MotionArbiter()


__all__ = [
    "MotionArbiter",
    "MotionBusyError",
    "DEFAULT_ARBITER",
    "DEFAULT_TIMEOUT_S",
]
