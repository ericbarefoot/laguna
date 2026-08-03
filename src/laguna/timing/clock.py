"""Experiment clock — tracks wall time and experiment runtime simultaneously."""

import logging
import time
from typing import Callable, Optional, Tuple

logger = logging.getLogger(__name__)


class ExperimentClock:
    """Tracks two timelines: wall clock and experiment runtime.

    Runtime advances only while the clock is running; it freezes during pause()
    and resumes from where it left off after resume().  This lets you distinguish
    "time elapsed in the real world" from "time spent collecting data."

    Typical use via FlumeLab.experiment() context manager, but can also be used
    standalone:
        clock = ExperimentClock()
        clock.start()
        clock.wait_until(30)   # blocks until 30s of runtime have elapsed
        clock.pause()
        ...
        clock.resume()
    """

    def __init__(self, speed_factor: float = 1.0) -> None:
        """
        Args:
            speed_factor: Experiment seconds per real second. 1.0 is real
                time. Above 1.0 the clock runs fast, so an offline rehearsal
                of a two-hour run finishes in minutes — see
                laguna.simulation. Only sensible when nothing is waiting on
                real hardware, since physical motion cannot be sped up.
        """
        if speed_factor <= 0:
            raise ValueError(
                f"speed_factor must be positive, got {speed_factor}"
            )
        self._speed = float(speed_factor)
        #: Called on every pause/resume transition, whoever caused it.
        #: RunContext subscribes so the piecewise runtime<->wall mapping is
        #: recorded no matter which code path paused the clock — Scheduler.
        #: stop() pauses it directly, so a FlumeLab-only hook silently missed
        #: those intervals and left the saved timeline wrong.
        self.on_pause: Optional[Callable[[], None]] = None
        self.on_resume: Optional[Callable[[], None]] = None
        self._start_wall: Optional[float] = None
        self._pause_offset: float = 0.0      # total seconds spent paused
        self._pause_start: Optional[float] = None
        self._running: bool = False
        self._paused: bool = False

    @property
    def speed_factor(self) -> float:
        """Experiment seconds per real second."""
        return self._speed

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Mark T=0 and begin counting runtime."""
        if self._running:
            raise RuntimeError("Clock is already running. Call stop() first.")
        self._start_wall = time.time()
        self._pause_offset = 0.0
        self._pause_start = None
        self._running = True
        self._paused = False

    def stop(self) -> None:
        """Freeze the clock. elapsed() will return the final runtime value."""
        if self._paused:
            self._pause_offset += time.time() - self._pause_start
            self._pause_start = None
        self._running = False
        self._paused = False

    def pause(self) -> None:
        """Freeze the runtime counter (wall clock keeps ticking)."""
        if not self._running:
            raise RuntimeError("Clock is not running.")
        if self._paused:
            return
        self._pause_start = time.time()
        self._paused = True
        self._notify(self.on_pause)

    def resume(self) -> None:
        """Unfreeze the runtime counter after a pause()."""
        if not self._running:
            raise RuntimeError("Clock is not running.")
        if not self._paused:
            return
        self._pause_offset += time.time() - self._pause_start
        self._pause_start = None
        self._paused = False
        self._notify(self.on_resume)

    @staticmethod
    def _notify(hook: Optional[Callable[[], None]]) -> None:
        """Fire an observer, never letting it break the clock."""
        if hook is None:
            return
        try:
            hook()
        except Exception:  # pragma: no cover - defensive
            logger.exception("Clock observer failed")

    # ------------------------------------------------------------------
    # Reading time
    # ------------------------------------------------------------------

    def elapsed(self) -> float:
        """Return experiment runtime in seconds (excludes paused time)."""
        if not self._running:
            if self._start_wall is None:
                return 0.0
            # clock was stopped — return final value
            return (time.time() - self._start_wall - self._pause_offset) * self._speed
        if self._paused:
            return (self._pause_start - self._start_wall - self._pause_offset) * self._speed
        return (time.time() - self._start_wall - self._pause_offset) * self._speed

    def wall_time(self) -> float:
        """Return current Unix wall time."""
        return time.time()

    def now(self) -> Tuple[float, float]:
        """Return (wall_time, runtime_s) simultaneously."""
        w = time.time()
        if not self._running or self._start_wall is None:
            return w, 0.0
        if self._paused:
            rt = (self._pause_start - self._start_wall - self._pause_offset) * self._speed
        else:
            rt = (w - self._start_wall - self._pause_offset) * self._speed
        return w, rt

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_paused(self) -> bool:
        return self._paused

    # ------------------------------------------------------------------
    # Blocking wait
    # ------------------------------------------------------------------

    def wait_until(self, runtime_s: float, poll_interval: float = 0.01) -> None:
        """Block until elapsed() >= runtime_s.

        Uses short polling rather than a single long sleep so pause/resume
        is respected and the wait can be interrupted cleanly.

        Args:
            runtime_s: Target experiment runtime in seconds.
            poll_interval: How often to check the clock (seconds).
        """
        if not self._running:
            raise RuntimeError("Clock is not running. Call start() first.")
        while self.elapsed() < runtime_s:
            time.sleep(poll_interval)
