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
        # One Event per run() call, not one shared Event that run() clears:
        # with a shared one, a quick pause-then-resume cleared it before the
        # old loop's next poll saw it set, and two loops ran at once,
        # double-firing every action.
        self._stop_event = threading.Event()
        self._run_lock = threading.Lock()
        self._running = False
        #: Asked immediately before every firing; a non-None return is the
        #: reason to skip it. FlumeLab sets this to refuse firings unless the
        #: rig is RUNNING — closing the window where an action dispatched
        #: just before a pause/estop ran just after it.
        self.gate: Optional[Callable[[], Optional[str]]] = None

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
            The first firing happens `every` seconds after the first run()
            that sees this registration — at runtime = every if registered
            before the run starts. Pausing does not reset the interval.
        """
        self._recurring.append(
            {"every": every, "action": action, "subsystem": subsystem,
             "name": name or action.__name__, "_next": None}
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

    @property
    def is_running(self) -> bool:
        """True while a run() loop is active (on any thread)."""
        return self._running

    def run(self, duration: float, on_complete: Optional[Callable[[], Any]] = None) -> None:
        """Block for `duration` runtime seconds, firing registered actions as due.

        Never resumes a paused clock itself. It used to, which let a bare
        ``lab.resume()`` restart the schedule straight out of an estop — the
        caller (``FlumeLab.resume()``) decides whether resuming is safe and
        resumes the clock first.

        Args:
            duration: Duration in experiment runtime seconds to run.
            on_complete: Optional callback invoked only when the duration expires
                naturally (not when stop() is called externally).

        Raises:
            RuntimeError: If the clock is not running, is paused, or another
                run() loop is already active.
        """
        with self._run_lock:
            if self._running:
                raise RuntimeError("Scheduler is already running — stop() it first")
            if not self._clock.is_running:
                raise RuntimeError("Scheduler.run() needs a started clock")
            if self._clock.is_paused:
                raise RuntimeError(
                    "Clock is paused — resume it (lab.resume()) before running the schedule"
                )
            self._running = True
            stop_event = threading.Event()
            self._stop_event = stop_event

        natural = False
        try:
            natural = self._loop(duration, stop_event)
        except Exception as exc:
            # A crashed loop used to vanish silently with the clock still
            # running, so the run looked alive while nothing was collected.
            logger.exception("Scheduler loop crashed")
            if self._event_log:
                self._event_log.log(
                    self._clock.elapsed(), "scheduler", "run", f"error: {exc}"
                )
        finally:
            # This run's own token, not self._stop_event — so ending this run
            # can never stop a newer one.
            stop_event.set()
            self._pause_clock()
            self._running = False
        if natural and on_complete is not None:
            on_complete()

    def _loop(self, duration: float, stop_event: threading.Event) -> bool:
        """The body of run(). Returns True if the duration expired naturally."""
        current = self._clock.elapsed()
        end_runtime = current + duration

        # Each repeat keeps its phase across pause/resume — runtime doesn't
        # advance while paused, so nothing piles up. Resetting every interval
        # on each run() (the old behaviour) meant pause/resume cycles shorter
        # than the interval could postpone a capture forever.
        for entry in self._recurring:
            if entry["_next"] is None:
                entry["_next"] = current + entry["every"]

        # Keep the *effective* time resolution constant however fast the
        # clock runs, so an accelerated rehearsal schedules like the real run
        # rather than in coarse jumps. Floored so a very high speed factor
        # doesn't turn this into a spin loop.
        poll = max(self._MIN_POLL, self._POLL / self._clock.speed_factor)

        while not stop_event.is_set():
            now = self._clock.elapsed()
            # Fire everything due up to the end of the run *before* deciding
            # the run is over: an action scheduled at exactly `duration`
            # (an end-of-run capture) used to be dropped.
            due_by = min(now, end_runtime)

            for entry in self._recurring:
                # Advance by `every` rather than rebasing on `now`. Rebasing
                # made every firing drift late by however long the poll
                # happened to overshoot, and the error accumulated over a
                # run. The while-loop catches up when more than one interval
                # elapsed between polls — otherwise a fast clock silently
                # fires fewer times than the real run would, which would make
                # a rehearsal under-report.
                fired = 0
                while (
                    due_by >= entry["_next"] and fired < self._MAX_CATCHUP
                    and not stop_event.is_set()
                ):
                    self._fire(entry["action"], entry["subsystem"], entry["name"], now, stop_event)
                    entry["_next"] += entry["every"]
                    fired += 1
                if fired >= self._MAX_CATCHUP and due_by >= entry["_next"]:
                    skipped = int((due_by - entry["_next"]) // entry["every"]) + 1
                    logger.warning(
                        "%s/%s fell more than %d intervals behind; skipping %d firing(s). "
                        "The schedule is denser than this machine can dispatch — lower "
                        "speed_factor or lengthen the interval.",
                        entry["subsystem"], entry["name"], self._MAX_CATCHUP, skipped,
                    )
                    if self._event_log:
                        self._event_log.log(
                            now, entry["subsystem"], entry["name"],
                            result=f"skipped: {skipped} firing(s) from runtime "
                                   f"{entry['_next']:.3f}s — scheduler fell behind",
                        )
                    entry["_next"] += skipped * entry["every"]

            for entry in self._oneshot:
                if stop_event.is_set():
                    break
                if not entry["_fired"] and due_by >= entry["runtime_s"]:
                    self._fire(entry["action"], entry["subsystem"], entry["name"], now, stop_event)
                    entry["_fired"] = True

            if now >= end_runtime:
                return not stop_event.is_set()
            time.sleep(poll)
        return False

    def stop(self) -> None:
        """Interrupt run() and pause the experiment clock.

        Pauses (does not stop) the clock to preserve elapsed runtime and allow
        resumption with a later run() call. Firings already dispatched but
        not yet started are skipped (see _fire).
        """
        self._stop_event.set()
        self._pause_clock()

    def _pause_clock(self) -> None:
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

        Raises:
            RuntimeError: Anything run() would refuse with, raised here on
                the calling thread rather than lost in the background one.
        """
        if self._running:
            raise RuntimeError("Scheduler is already running — stop() it first")
        if not self._clock.is_running or self._clock.is_paused:
            raise RuntimeError(
                "Scheduler.run_async() needs a started, unpaused clock — resume it first"
            )
        t = threading.Thread(
            target=self.run, args=(duration,), daemon=True, name="scheduler-main"
        )
        t.start()
        return t

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _fire(
        self,
        action: Callable,
        subsystem: str,
        name: str,
        runtime_s: float,
        stop_event: Optional[threading.Event] = None,
    ) -> None:
        """Dispatch action in a daemon thread; log exceptions only.

        The thread re-checks, immediately before calling the action, that
        the run hasn't been stopped and the gate (see ``gate``) still allows
        it. A firing dispatched a moment before a pause or estop otherwise
        ran a moment *after* it — turning the pump back on after an estop,
        say. A skip is written to the event log, so the missed firing is on
        record and can be re-acquired.

        Args:
            action: Callable to invoke in a background thread.
            subsystem: Subsystem name for logging context.
            name: Action name for logging context.
            runtime_s: Runtime when this action fires (for event logging).
            stop_event: The dispatching run's stop token.
        """
        def _run():
            reason = None
            if stop_event is not None and stop_event.is_set():
                reason = "scheduler stopped"
            elif self.gate is not None:
                try:
                    reason = self.gate()
                except Exception as exc:  # fail closed
                    reason = f"gate check failed: {exc}"
            if reason is not None:
                logger.warning("Skipped scheduled %s/%s: %s", subsystem, name, reason)
                if self._event_log:
                    self._event_log.log(runtime_s, subsystem, name, f"skipped: {reason}")
                return
            try:
                action()
            except Exception as exc:
                logger.warning("Scheduled action %s/%s failed: %s", subsystem, name, exc)
                if self._event_log:
                    try:
                        self._event_log.log(runtime_s, subsystem, name, f"error: {exc}")
                    except Exception:
                        logger.exception("Could not record %s/%s's failure", subsystem, name)

        threading.Thread(target=_run, daemon=True, name=f"sched-{subsystem}-{name}").start()
