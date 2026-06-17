"""Scheduler — fires registered actions in daemon threads on experiment time."""

import logging
import threading
import time
from typing import Callable, List, Optional

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

    _POLL = 0.05  # seconds between schedule checks

    def __init__(
        self,
        clock: ExperimentClock,
        event_log: Optional[EventLog] = None,
    ) -> None:
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
        """Register action to fire once at `runtime_s`."""
        self._oneshot.append(
            {"runtime_s": runtime_s, "action": action, "subsystem": subsystem,
             "name": name or action.__name__, "_fired": False}
        )

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def run(self, duration: float) -> None:
        """Block for `duration` runtime seconds, firing registered actions as due.

        Resumes the clock if it was paused (e.g. after a previous stop() call).
        Returns when duration expires or stop() is called.
        """
        self._stop_event.clear()
        if self._clock.is_running and self._clock.is_paused:
            self._clock.resume()
        end_runtime = self._clock.elapsed() + duration

        # Reset next-fire times relative to current runtime
        current = self._clock.elapsed()
        for entry in self._recurring:
            entry["_next"] = current + entry["every"]

        while not self._stop_event.is_set():
            now = self._clock.elapsed()

            if now >= end_runtime:
                break

            for entry in self._recurring:
                if now >= entry["_next"]:
                    self._fire(entry["action"], entry["subsystem"], entry["name"], now)
                    entry["_next"] = now + entry["every"]

            for entry in self._oneshot:
                if not entry["_fired"] and now >= entry["runtime_s"]:
                    self._fire(entry["action"], entry["subsystem"], entry["name"], now)
                    entry["_fired"] = True

            time.sleep(self._POLL)

    def stop(self) -> None:
        """Interrupt run() and pause the experiment clock.

        Pauses rather than stops the clock so elapsed runtime is preserved
        and the experiment can be resumed with run() later.
        """
        self._stop_event.set()
        if self._clock.is_running and not self._clock.is_paused:
            self._clock.pause()

    def run_async(self, duration: float) -> threading.Thread:
        """Run the scheduler in a background daemon thread.

        Returns the thread so the caller can join() it if needed.
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
        """Dispatch action in a daemon thread and log the firing."""
        def _run():
            try:
                action()
                result = "ok"
            except Exception as exc:
                result = f"error: {exc}"
                logger.warning("Scheduled action %s/%s failed: %s", subsystem, name, exc)
            if self._event_log:
                self._event_log.log(runtime_s, subsystem, name, result)

        threading.Thread(target=_run, daemon=True, name=f"sched-{subsystem}-{name}").start()
