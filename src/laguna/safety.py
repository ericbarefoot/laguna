"""One safety vocabulary, used identically by every subsystem.

Before this, ``stop()`` meant four different severities depending on which
subsystem you called it on — a zero-decel abort with brakes engaged on the
gantry, a halted move on the weir, **the pump switched off** on the flow
controller, and merely closing an SDK data channel on the Gocator. Gauge,
rangefinders and the Pi cameras had no ``stop()`` at all.

Every subsystem now implements the same four verbs with the same meanings, so
there is nothing to remember about which one you happen to be holding:

**pause()** — cease activity, hold state, stay connected, be resumable. The
gantry decelerates on its own ramp, the pump stops with its setpoint
remembered, acquisition aborts. Recoverable with ``resume()``.

**resume()** — undo ``pause()``, restoring whatever setpoint it captured.

**stop()** — end the run cleanly. Quiesce into a state that is safe to
disconnect from. Not resumable; start a new run instead.

**estop()** — the hardest halt the hardware can reach. Gantry zero-decel abort
with brakes engaged and motors disabled, pump off, both valves closed.
Requires an explicit re-arm.

This deliberately **changes what some existing methods do**, most notably
``GantryController.stop()``, which used to be the hard abort and is now the
clean stop — the hard abort moved to ``estop()``, where the vocabulary says it
belongs. A unified API was judged more valuable than a stable one.

Verbs may return a short note describing anything a human needs to know —
above all, **data that was discarded**. ``FlumeLab`` writes those notes to the
experiment event log, because silently missing scan data can invalidate an
experiment just as thoroughly as bad data can.

Triggers
--------
All three tiers can be demanded from outside the process, not just estop. A
health check that notices the scanner has stopped returning surfaces, or a
camera has stopped producing frames, can write a ``PAUSE`` file and bring the
whole experiment to a graceful halt before more perishable data is lost — no
code changes and no new infrastructure.

Polling on a background thread rather than handling signals is deliberate:
Python delivers signals only in the main thread between bytecodes, so a signal
cannot interrupt a blocking serial read or SDK call — the halt would be
deferred exactly when it matters most.
"""

from __future__ import annotations

import enum
import logging
import threading
from pathlib import Path
from typing import Callable, List, Optional, Protocol, Tuple, runtime_checkable

logger = logging.getLogger(__name__)

#: How often trigger sources are polled, seconds.
TRIGGER_POLL_S = 0.1

#: Default sentinel filenames, one per tier. Creating one demands that action.
DEFAULT_SENTINELS = {
    "pause": "./PAUSE",
    "stop": "./STOP",
    "estop": "./ESTOP",
}


class SafetyTier(enum.Enum):
    """What a trigger is demanding, gentlest first."""

    PAUSE = "pause"
    STOP = "stop"
    ESTOP = "estop"

    @property
    def severity(self) -> int:
        return {"pause": 0, "stop": 1, "estop": 2}[self.value]


class SafetyState(enum.Enum):
    """Where the rig is in the pause/stop/estop lifecycle."""

    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"
    ESTOPPED = "estopped"


@runtime_checkable
class Quiescible(Protocol):
    """The safety verbs every subsystem implements.

    Each may return a short note — typically about discarded data — which
    ``FlumeLab`` writes to the event log. ``None`` means "nothing a human
    needs to know".

    ``estop()`` **must not raise and must not require being connected**: an
    emergency stop that throws part-way leaves the rest of the rig running.
    """

    def pause(self) -> Optional[str]:
        """Cease activity, hold state, stay connected. Must be resumable."""

    def resume(self) -> Optional[str]:
        """Undo pause(), restoring whatever setpoint it captured."""

    def stop(self) -> Optional[str]:
        """End cleanly — quiesce into a state safe to disconnect from."""

    def estop(self) -> Optional[str]:
        """Reach the safest state the hardware can, as fast as possible."""


# ----------------------------------------------------------------------
# Triggers
# ----------------------------------------------------------------------


class SentinelFileTrigger:
    """Demands a tier while a sentinel file exists.

    Works no matter which thread is blocked, needs no ports or dependencies,
    and fires from any terminal with ``touch PAUSE``. A physical button wired
    to create the file needs no further software support — and neither does a
    health check, which just writes the file.

    The file must be **removed** before the rig will resume or re-arm, so it
    cannot be brought back up into the condition that tripped it.
    """

    def __init__(
        self,
        path: str,
        tier: SafetyTier = SafetyTier.ESTOP,
        name: Optional[str] = None,
    ) -> None:
        self.path = Path(path)
        self.tier = tier
        self.name = name or f"{tier.value}_file:{self.path.name}"

    @property
    def hint(self) -> str:
        """How to clear this trigger, for use in refusal messages."""
        return f"remove {self.path}"

    def is_tripped(self) -> bool:
        return self.path.exists()

    def clear(self) -> bool:
        """Remove the sentinel. True if a file was actually removed."""
        try:
            self.path.unlink()
            return True
        except FileNotFoundError:
            return False

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"SentinelFileTrigger({str(self.path)!r}, {self.tier.name})"


class CallableTrigger:
    """Adapts any zero-arg predicate into a trigger.

    The extension point for health checks and hardware flags — a scanner that
    has stopped returning surfaces, a camera whose frame count has stalled,
    the VFD's own e-stop circuit::

        CallableTrigger(lambda: lab.flow.get_status().get("vfd_estop", False),
                        tier=SafetyTier.ESTOP, name="vfd_hardware_estop")

    A predicate that raises is treated as **not** tripped and logged at debug
    level: a monitoring source that has itself failed must not be able to halt
    an experiment on its own malfunction.
    """

    def __init__(
        self,
        predicate: Callable[[], bool],
        tier: SafetyTier = SafetyTier.ESTOP,
        name: str = "callable",
        hint: Optional[str] = None,
    ) -> None:
        self._predicate = predicate
        self.tier = tier
        self.name = name
        self._hint = hint

    @property
    def hint(self) -> str:
        """How to clear this trigger. Generic unless the caller supplied one —
        a health-check predicate's remediation is caller-specific (e.g. a
        VFD e-stop needs the physical button released, not a file removed)."""
        return self._hint or f"resolve whatever '{self.name}' is checking"

    def is_tripped(self) -> bool:
        try:
            return bool(self._predicate())
        except Exception as exc:
            logger.debug("Safety trigger %r could not be read: %s", self.name, exc)
            return False


class SafetyMonitor:
    """Polls triggers on a background thread and fires the matching verb.

    Watches all three tiers, not just estop. The motivating case is a health
    check: if the scanner stops returning surfaces, or a camera stops producing
    frames, something writes ``PAUSE`` and the whole experiment halts
    gracefully before more perishable data is lost.

    Args:
        triggers: Sources to poll.
        on_trip: Called with ``(tier, trigger_name)``. Fires **once** per armed
            period per severity — a sentinel that stays on disk must not
            re-fire every poll — and only **escalates**: once ESTOP has fired,
            a lingering PAUSE file cannot downgrade the rig.
        poll_s: Seconds between polls.
    """

    def __init__(
        self,
        triggers: Optional[List] = None,
        on_trip: Optional[Callable[[SafetyTier, str], None]] = None,
        poll_s: float = TRIGGER_POLL_S,
    ) -> None:
        self.triggers: List = list(triggers or [])
        self._on_trip = on_trip
        self._poll_s = poll_s
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._fired_at: Optional[int] = None    # highest severity already fired

    def add(self, trigger) -> "SafetyMonitor":
        self.triggers.append(trigger)
        return self

    def tripped(self) -> Optional[Tuple[SafetyTier, str]]:
        """The most severe currently-tripped ``(tier, name)``, or None."""
        worst: Optional[Tuple[SafetyTier, str]] = None
        for trigger in self.triggers:
            try:
                if not trigger.is_tripped():
                    continue
            except Exception as exc:  # pragma: no cover - defensive
                logger.debug("Safety trigger %r failed: %s", trigger.name, exc)
                continue
            if worst is None or trigger.tier.severity > worst[0].severity:
                worst = (trigger.tier, trigger.name)
        return worst

    def tripped_by(self, tier: Optional[SafetyTier] = None) -> Optional[str]:
        """Name of a currently-tripped trigger, optionally filtered by tier.

        Used by ``resume``/``rearm`` to refuse while the cause is still there.
        """
        for trigger in self.triggers:
            try:
                if trigger.is_tripped() and (tier is None or trigger.tier is tier):
                    return trigger.name
            except Exception:  # pragma: no cover - defensive
                continue
        return None

    def hint_for(self, name: str) -> str:
        """Remediation text for a tripped trigger by name, for refusal messages.

        Falls back to generic advice if the trigger can't be found (e.g. it
        was removed from the monitor between the trip and this lookup).
        """
        for trigger in self.triggers:
            if trigger.name == name:
                return getattr(trigger, "hint", "clear whatever tripped it")
        return "clear whatever tripped it"

    def start(self) -> None:
        """Begin polling. No-op if already running or there are no triggers."""
        if self._thread is not None or not self.triggers:
            return
        self._stop.clear()
        self._fired_at = None
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="safety-monitor"
        )
        self._thread.start()
        logger.info(
            "Safety monitor watching: %s",
            ", ".join(f"{t.name}({t.tier.value})" for t in self.triggers),
        )

    def stop(self) -> None:
        """Stop polling and join the thread."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self._poll_s * 20)
            self._thread = None

    def rearm(self) -> None:
        """Allow triggers to fire again, once their causes have cleared."""
        self._fired_at = None

    def _run(self) -> None:
        while not self._stop.is_set():
            found = self.tripped()
            if found is not None:
                tier, name = found
                # Escalate only. A lingering PAUSE file must never undo an
                # ESTOP that has already fired.
                if self._fired_at is None or tier.severity > self._fired_at:
                    self._fired_at = tier.severity
                    logger.critical("%s demanded by trigger %r", tier.name, name)
                    if self._on_trip is not None:
                        try:
                            self._on_trip(tier, name)
                        except Exception:  # pragma: no cover - defensive
                            logger.exception("Safety handler itself failed")
            self._stop.wait(self._poll_s)


__all__ = [
    "SafetyTier",
    "SafetyState",
    "Quiescible",
    "SentinelFileTrigger",
    "CallableTrigger",
    "SafetyMonitor",
    "DEFAULT_SENTINELS",
    "TRIGGER_POLL_S",
]
