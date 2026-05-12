"""Experiment clock — tracks wall time and experiment runtime simultaneously."""

import time
from typing import Optional, Tuple


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

    def __init__(self) -> None:
        self._start_wall: Optional[float] = None
        self._pause_offset: float = 0.0      # total seconds spent paused
        self._pause_start: Optional[float] = None
        self._running: bool = False
        self._paused: bool = False

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

    def resume(self) -> None:
        """Unfreeze the runtime counter after a pause()."""
        if not self._running:
            raise RuntimeError("Clock is not running.")
        if not self._paused:
            return
        self._pause_offset += time.time() - self._pause_start
        self._pause_start = None
        self._paused = False

    # ------------------------------------------------------------------
    # Reading time
    # ------------------------------------------------------------------

    def elapsed(self) -> float:
        """Return experiment runtime in seconds (excludes paused time)."""
        if not self._running:
            if self._start_wall is None:
                return 0.0
            # clock was stopped — return final value
            return time.time() - self._start_wall - self._pause_offset
        if self._paused:
            return self._pause_start - self._start_wall - self._pause_offset
        return time.time() - self._start_wall - self._pause_offset

    def wall_time(self) -> float:
        """Return current Unix wall time."""
        return time.time()

    def now(self) -> Tuple[float, float]:
        """Return (wall_time, runtime_s) simultaneously."""
        w = time.time()
        if not self._running or self._start_wall is None:
            return w, 0.0
        if self._paused:
            rt = self._pause_start - self._start_wall - self._pause_offset
        else:
            rt = w - self._start_wall - self._pause_offset
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
