"""Two-tier logging for subsystems: event log vs. operational log.

**The event log** (laguna.timing.EventLog, written via this mixin's
log_event()) is the terse, archival narrative — it ships alongside
published experiment data as metadata, so it stays to state-changing
actions and milestones only: "weir moved to 300mm", "scan completed",
"inflow on". It is emphatically not a record of passive reads or status
polls (a gauge reading, a periodic get_status() snapshot) — those measure
the experiment's state without changing it, so by default they don't
belong here (see laguna.experiment.runner's log_as_event opt-in for the
cases where a human decides an infrequent poll IS a milestone worth
archiving).

**The operational log** is everything else worth knowing about but not
worth archiving: connections firing, broad motion commands (one line per
lab.gantry.move_to() call, not one per tessellated G-code segment), and —
at DEBUG — the low-level detail underneath. It is deliberately just
standard Python `logging`, not a second structured file format: every
laguna module already has its own `logging.getLogger(__name__)`, so this
tier costs nothing new to adopt beyond calling it. See
FlumeLab._configure_operational_log() for how it's leveled and (optionally)
persisted to a file under the run directory.

FlumeLab.add() duck-types on attach_event_log() (see core.py) and wires any
subsystem that inherits this mixin to the run's shared event_log/clock, so
scripts and laguna.experiment.runner no longer need to hand-roll
``lab.event_log.log(...)`` calls after every subsystem action — the
subsystem logs its own key actions via log_event(). log_level and
event_log_verbosity are deliberately independent: an experiment often wants
a quiet terminal but a complete event_log audit trail, or the reverse
during hardware bring-up. FlumeLab(debug=True) (see set_global_debug())
overrides every subsystem's own log_level with DEBUG for troubleshooting,
without needing to edit each one's config section individually.
"""

import logging
from typing import Any, Optional

_LEVEL_VALUES = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}

#: Set by FlumeLab(debug=True) — see set_global_debug(). When true,
#: attach_event_log() leaves each subsystem's terminal logger alone rather
#: than applying its own configured log_level, so it inherits DEBUG from
#: the "laguna" root logger FlumeLab sets instead of being pinned quieter.
_GLOBAL_DEBUG = False


def set_global_debug(enabled: bool) -> None:
    """Set global DEBUG mode for all laguna loggers.

    Called by FlumeLab(debug=True) to override all subsystem log_level configs.

    Args:
        enabled: If True, set all laguna loggers to DEBUG; if False, leave
            subsystem log_level configs in control.
    """
    global _GLOBAL_DEBUG
    _GLOBAL_DEBUG = enabled


class SubsystemLogging:
    """Mixin giving a subsystem self.log_event(), wired by FlumeLab.add().

    Concrete subsystems read log_level / event_log_verbosity from their own
    config section in __init__ and assign them to self — the class
    attributes below are just the "key absent" default.
    """

    log_level: str = "INFO"
    event_log_verbosity: str = "INFO"

    _event_log: Optional[Any] = None
    _clock: Optional[Any] = None

    def attach_event_log(self, event_log: Any, clock: Any) -> None:
        """Wire this subsystem's log_event() calls to the run's event log.

        Called automatically by FlumeLab.add() for subsystems with this method.

        Args:
            event_log: EventLog instance to write to.
            clock: ExperimentClock for elapsed time.
        """
        self._event_log = event_log
        self._clock = clock
        if not _GLOBAL_DEBUG:
            logging.getLogger(type(self).__module__).setLevel(
                _LEVEL_VALUES.get(self.log_level, logging.INFO)
            )

    def log_event(self, action: str, level: str = "INFO", result: str = "ok", **fields: Any) -> None:
        """Record a key action to the event log.

        No-op before attach_event_log() is called, or if `level` is below
        the subsystem's event_log_verbosity threshold.

        Args:
            action: Action type string.
            level: Log level (DEBUG, INFO, WARNING, etc.).
            result: Result string (e.g., 'ok', 'error: ...')
            **fields: Additional key=value pairs for the log record.
        """
        if self._event_log is None or self._clock is None:
            return
        if _LEVEL_VALUES.get(level, 20) < _LEVEL_VALUES.get(self.event_log_verbosity, 20):
            return
        notes = " ".join(f"{k}={v}" for k, v in fields.items())
        self._event_log.log(self._clock.elapsed(), self.subsystem_name, action, result, notes)
