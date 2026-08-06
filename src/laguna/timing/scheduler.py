"""Scheduler — fires registered actions in daemon threads on experiment time."""

import logging
import threading
import time
from typing import Callable, List, Optional, Any

from .clock import ExperimentClock
from .event_log import EventLog

logger = logging.getLogger(__name__)


class Scheduler:
    """Fires registered actions concurrently based on experiment runtime.

    Background actions (repeat / at) are dispatched in daemon threads so
    they never block the foreground experiment loop.

    Typical usage:
        scheduler = Scheduler(clock, event_log)
        scheduler.repeat(every=5, action=hydraulics.get_status, subsystem="hydraulics")
        scheduler.at(runtime_s=60, action=cameras.trigger_capture, subsystem="camera")
        scheduler.run(duration=300)   # blocks for 5 experiment-minutes
    """

    _POLL = 0.05        # real seconds between schedule checks at speed 1.0
    _MIN_POLL = 0.001   # floor, so a fast clock doesn't spin the CPU
    _MAX_CATCHUP = 100  # firings per poll before declaring the backlog lost

    def __init__(
        self,
        clock: ExperimentClock,
        event_log: Optional[EventLog] = None,
    ) -> None:
        """Initialize the scheduler.

        Args:
            clock: The ExperimentClock that drives scheduling.
            event_log: Optional EventLog to record scheduler events (e.g., failures).
        """
        self._clock = clock
        self._event_log = event_log
        self._recurring: List[dict] = []
        self._oneshot: List[dict] = []
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def repeat(
        self,
        every: float,
        action: Callable,
        subsystem: str = "scheduler",
        name: str = "",
    ) -> None:
        """Register action to fire every `every` runtime seconds.

        Args:
            every: Interval in experiment runtime seconds between firings.
            action: Callable to invoke at each interval.
            subsystem: Name of the subsystem (for logging and identification).
            name: Optional human-readable name for this action; defaults to action.__name__.

        Note:
            The first firing happens at runtime = every (not at 0).
        """
        self._recurring.append(
            {"every": every, "action": action, "subsystem": subsystem,
             "name": name or action.__name__, "_next": every}
        )

    def at(
        self,
        runtime_s: float,
        action: Callable,
        subsystem: str = "scheduler",
        name: str = "",
    ) -> None:
        """Register action to fire once at a specific runtime.

        Args:
            runtime_s: Experiment runtime in seconds when the action should fire.
            action: Callable to invoke at the specified time.
            subsystem: Name of the subsystem (for logging and identification).
            name: Optional human-readable name for this action; defaults to action.__name__.
        """
        self._oneshot.append(
            {"runtime_s": runtime_s, "action": action, "subsystem": subsystem,
             "name": name or action.__name__, "_fired": False}
        )

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def run(self, duration: float, on_complete: Optional[Callable[[], Any]] = None) -> None:
        """Block for `duration` runtime seconds, firing registered actions as due.

        Resumes the clock if paused (e.g. after a previous stop() call).

        Args:
            duration: Duration in experiment runtime seconds to run.
            on_complete: Optional callback invoked only when the duration expires
                naturally (not when stop() is called externally).
        """
        self._stop_event.clear()
        if self._clock.is_running and self._clock.is_paused:
            self._clock.resume()
        end_runtime = self._clock.elapsed() + duration

        # Reset next-fire times relative to current runtime
        current = self._clock.elapsed()
        for entry in self._recurring:
            entry["_next"] = current + entry["every"]

        # Keep the *effective* time resolution constant however fast the
        # clock runs, so an accelerated rehearsal schedules like the real run
        # rather than in coarse jumps. Floored so a very high speed factor
        # doesn't turn this into a spin loop.
        poll = max(self._MIN_POLL, self._POLL / self._clock.speed_factor)

        while not self._stop_event.is_set():
            now = self._clock.elapsed()

            if now >= end_runtime:
                break

            for entry in self._recurring:
                # Advance by `every` rather than rebasing on `now`. Rebasing
                # made every firing drift late by however long the poll
                # happened to overshoot, and the error accumulated over a
                # run. The while-loop catches up when more than one interval
                # elapsed between polls — otherwise a fast clock silently
                # fires fewer times than the real run would, which would make
                # a rehearsal under-report.
                fired = 0
                while now >= entry["_next"] and fired < self._MAX_CATCHUP:
                    self._fire(entry["action"], entry["subsystem"], entry["name"], now)
                    entry["_next"] += entry["every"]
                    fired += 1
                if fired >= self._MAX_CATCHUP:
                    logger.warning(
                        "%s/%s fell more than %d intervals behind; skipping the "
                        "backlog. The schedule is denser than this machine can "
                        "dispatch — lower speed_factor or lengthen the interval.",
                        entry["subsystem"], entry["name"], self._MAX_CATCHUP,
                    )
                    entry["_next"] = now + entry["every"]

            for entry in self._oneshot:
                if not entry["_fired"] and now >= entry["runtime_s"]:
                    self._fire(entry["action"], entry["subsystem"], entry["name"], now)
                    entry["_fired"] = True

            time.sleep(poll)

        _natural = not self._stop_event.is_set()
        self.stop()
        if _natural and on_complete is not None:
            on_complete()

    def stop(self) -> None:
        """Interrupt run() and pause the experiment clock.

        Pauses (does not stop) the clock to preserve elapsed runtime and allow
        resumption with a later run() call.
        """
        self._stop_event.set()
        if self._clock.is_running and not self._clock.is_paused:
            if self._event_log:
                self._event_log.log(self._clock.elapsed(), "scheduler", "stop", "ok")
            self._clock.pause()

    def run_async(self, duration: float) -> threading.Thread:
        """Run the scheduler in a background daemon thread.

        Args:
            duration: Duration in experiment runtime seconds to run.

        Returns:
            The daemon thread so the caller can join() it if needed.
            Call stop() from another thread to interrupt the run.
        """
        t = threading.Thread(
            target=self.run, args=(duration,), daemon=True, name="scheduler-main"
        )
        t.start()
        return t

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _fire(self, action: Callable, subsystem: str, name: str, runtime_s: float) -> None:
        """Dispatch action in a daemon thread; log exceptions only.

        Args:
            action: Callable to invoke in a background thread.
            subsystem: Subsystem name for logging context.
            name: Action name for logging context.
            runtime_s: Runtime when this action fires (for event logging).
        """
        def _run():
            try:
                action()
            except Exception as exc:
                logger.warning("Scheduled action %s/%s failed: %s", subsystem, name, exc)
                if self._event_log:
                    self._event_log.log(runtime_s, subsystem, name, f"error: {exc}")

        threading.Thread(target=_run, daemon=True, name=f"sched-{subsystem}-{name}").start()
