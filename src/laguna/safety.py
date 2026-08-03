"""Three tiers of stopping, and the triggers that fire the hardest one.

Before this module, ``stop()`` meant four different severities depending on
which subsystem you called it on: a zero-decel abort with brakes engaged on
the gantry, a halted move on the weir, **the pump switched off** on the flow
controller, and merely closing an SDK data channel on the Gocator. Gauge,
rangefinders and the Pi cameras had no ``stop()`` at all. So
``FlumeLab.emergency_stop()``, which called ``stop()`` on everything, produced
an inconsistent result — and stopped the pump, which for a flume is not
obviously the safe state.

Three verbs, from gentlest to hardest:

**pause()** — temporary and resumable. The gantry decelerates on its own ramp
(``soft_stop()``), the pump is stopped and its setpoint remembered, sensors
stop acquiring, and **the experiment clock stops too** — so runtime means
"time under experimental conditions" and a schedule row at t=600 fires 600 s
of real experiment time in, however long the pause lasted. Everything stays
connected. ``resume()`` restores the flow setpoint and restarts the clock.

**stop()** — end the run cleanly. Quiesce as for pause, then disconnect.

**estop()** — screeching halt. Gantry hard abort (zero-decel, brakes engaged,
motors disabled), pump off, both valves closed, acquisition stopped. Reaching
``ESTOPPED`` requires an explicit re-arm; see ``FlumeLab.rearm()``.

Existing ``stop()`` methods keep their current meaning and become internal
details these verbs call — ``GantryController.stop()`` is still the hard abort
it is documented and tested as, reached via ``estop()``, while ``pause()``
routes to ``soft_stop()``. Nothing silently changed behaviour.

Triggers
--------
Estop can be fired from outside the process by a pluggable list of trigger
sources, polled on a background thread. A background poll rather than a signal
handler is deliberate: Python delivers signals only in the main thread between
bytecodes, so a signal cannot interrupt a blocking serial read or SDK call —
the halt would be deferred exactly when it matters most.

:class:`SentinelFileTrigger` watches for a file (``./ESTOP`` by default). A
physical button wired to create that file therefore needs no additional
software support at all.
"""

from __future__ import annotations

import enum
import logging
import threading
from pathlib import Path
from typing import Callable, List, Optional, Protocol, runtime_checkable

logger = logging.getLogger(__name__)

#: How often trigger sources are polled, seconds.
TRIGGER_POLL_S = 0.1

#: Default sentinel path. Creating this file trips an estop.
DEFAULT_SENTINEL = "./ESTOP"


class SafetyState(enum.Enum):
    """Where the rig is in the pause/stop/estop lifecycle."""

    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"
    ESTOPPED = "estopped"


@runtime_checkable
class Quiescible(Protocol):
    """What a subsystem may implement to take part in the safety verbs.

    Every method is optional — :class:`~laguna.core.FlumeLab` probes with
    ``getattr`` and skips what isn't there, so a subsystem that has nothing
    meaningful to do (a passive sensor, say) needs no changes at all.
    """

    def pause(self) -> None:
        """Cease activity, hold state, stay connected. Must be resumable."""

    def resume(self) -> None:
        """Undo pause(), restoring whatever setpoint it captured."""

    def estop(self) -> None:
        """Reach the safest reachable state as fast as possible.

        **Must not raise**, and in particular must not require being
        connected — an estop that throws part-way leaves the rest of the rig
        running.
        """


# ----------------------------------------------------------------------
# Triggers
# ----------------------------------------------------------------------


class EstopTrigger(Protocol):
    """A source that can demand an emergency stop."""

    name: str

    def is_tripped(self) -> bool:
        """True if this source is currently demanding an estop."""


class SentinelFileTrigger:
    """Trips while a sentinel file exists.

    Works no matter which thread is blocked, needs no ports or dependencies,
    and can be fired from any terminal with ``touch ESTOP`` — or by a physical
    button wired to create the file, which is why no separate button trigger
    is needed.

    The file must be **removed** before :meth:`~laguna.core.FlumeLab.rearm`
    will re-arm, so the rig cannot be brought back up into a live emergency.
    """

    def __init__(self, path: str = DEFAULT_SENTINEL, name: str = "sentinel_file") -> None:
        self.path = Path(path)
        self.name = name

    def is_tripped(self) -> bool:
        return self.path.exists()

    def clear(self) -> bool:
        """Remove the sentinel. Returns True if a file was actually removed."""
        try:
            self.path.unlink()
            return True
        except FileNotFoundError:
            return False

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"SentinelFileTrigger({str(self.path)!r})"


class CallableTrigger:
    """Adapts any zero-arg predicate into a trigger.

    The extension point for sources that need no dedicated class — polling
    the VFD's own hardware ``e_stop`` flag, a GPIO read, an IO-Link input::

        CallableTrigger(lambda: lab.flow.get_status().get("vfd_estop", False),
                        name="vfd_hardware_estop")

    A predicate that raises is treated as **not** tripped and logged at debug
    level: a monitoring source that has itself failed must not be able to
    spuriously halt an experiment.
    """

    def __init__(self, predicate: Callable[[], bool], name: str = "callable") -> None:
        self._predicate = predicate
        self.name = name

    def is_tripped(self) -> bool:
        try:
            return bool(self._predicate())
        except Exception as exc:
            logger.debug("Estop trigger %r could not be read: %s", self.name, exc)
            return False


class EstopMonitor:
    """Polls trigger sources on a background thread and fires a callback once.

    Args:
        triggers: Sources to poll.
        on_trip: Called with the tripping trigger's name. Invoked **once** per
            armed period — a sentinel file that stays on disk must not fire an
            estop on every poll.
        poll_s: Seconds between polls.
    """

    def __init__(
        self,
        triggers: Optional[List[EstopTrigger]] = None,
        on_trip: Optional[Callable[[str], None]] = None,
        poll_s: float = TRIGGER_POLL_S,
    ) -> None:
        self.triggers: List[EstopTrigger] = list(triggers or [])
        self._on_trip = on_trip
        self._poll_s = poll_s
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._fired = False

    def add(self, trigger: EstopTrigger) -> "EstopMonitor":
        self.triggers.append(trigger)
        return self

    def tripped_by(self) -> Optional[str]:
        """Name of the first currently-tripped trigger, or None."""
        for trigger in self.triggers:
            try:
                if trigger.is_tripped():
                    return trigger.name
            except Exception as exc:  # pragma: no cover - defensive
                logger.debug("Estop trigger %r failed: %s", trigger.name, exc)
        return None

    def start(self) -> None:
        """Begin polling. No-op if already running or there are no triggers."""
        if self._thread is not None or not self.triggers:
            return
        self._stop.clear()
        self._fired = False
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="estop-monitor"
        )
        self._thread.start()
        logger.info(
            "Estop monitor watching: %s", ", ".join(t.name for t in self.triggers)
        )

    def stop(self) -> None:
        """Stop polling and join the thread."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self._poll_s * 20)
            self._thread = None

    def rearm(self) -> None:
        """Allow the callback to fire again once triggers have cleared."""
        self._fired = False

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self._fired:
                name = self.tripped_by()
                if name is not None:
                    self._fired = True
                    logger.critical("ESTOP demanded by trigger %r", name)
                    if self._on_trip is not None:
                        try:
                            self._on_trip(name)
                        except Exception:  # pragma: no cover - defensive
                            logger.exception("Estop handler itself failed")
            self._stop.wait(self._poll_s)


__all__ = [
    "SafetyState",
    "Quiescible",
    "EstopTrigger",
    "SentinelFileTrigger",
    "CallableTrigger",
    "EstopMonitor",
    "DEFAULT_SENTINEL",
]
