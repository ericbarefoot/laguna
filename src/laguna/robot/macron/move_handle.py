"""Non-blocking gantry moves: ``move_to()`` returns a handle instead of blocking.

A blocking ``move_to()`` held the REPL — and in a notebook, the kernel — for
the whole traverse, so the operator's ``lab.pause()`` cell couldn't run
until the move had already finished. Ctrl-C first, then ``pause()``, is two
fumble-prone steps exactly when speed matters. Now ``move_to()`` fence-checks
on the calling thread (so a ``FenceViolation`` still raises immediately, with
nothing sent), then executes on a background thread and returns a
:class:`MoveHandle`. Scripts that need sequencing call ``.wait()``.

An error on the background thread is not lost if nobody waits: it is logged
at ERROR as soon as it happens, and :meth:`MoveHandle.wait` re-raises it.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)


class MoveHandle:
    """A gantry move in progress (or already finished).

    Attributes:
        description: What the move is, e.g. ``"move_to({'X': 100})"``.
        result: What the operation produced, once it succeeds — e.g. the
            standoff position from ``home_axis()``. None for a plain move.
    """

    def __init__(self, description: str) -> None:
        """Create a handle for a move that has not finished yet.

        Args:
            description: Human-readable summary, used in logs and errors.
        """
        self.description = description
        self.result: Any = None
        self._done = threading.Event()
        self._error: Optional[BaseException] = None
        self._started_at = time.monotonic()
        self._finished_at: Optional[float] = None

    @property
    def done(self) -> bool:
        """True once the move has finished, failed, or been cancelled."""
        return self._done.is_set()

    @property
    def error(self) -> Optional[BaseException]:
        """The exception that ended the move, or None if it succeeded (or is still running)."""
        return self._error

    @property
    def succeeded(self) -> bool:
        """True once the move has finished without error."""
        return self.done and self._error is None

    def wait(self, timeout: Optional[float] = None) -> "MoveHandle":
        """Block until the move finishes, re-raising whatever ended it early.

        Args:
            timeout: Seconds to wait; None waits as long as the move takes.

        Returns:
            self, so ``lab.move_to(...).wait()`` reads naturally.

        Raises:
            TimeoutError: If `timeout` elapses first. The move keeps
                running — call ``pause()`` to halt it.
            Exception: Whatever the move raised (e.g. ``MotionHalted`` if a
                pause cancelled it).
        """
        if not self._done.wait(timeout):
            raise TimeoutError(
                f"{self.description} still running after {timeout:.0f}s — it was not "
                "stopped; call pause() to halt it"
            )
        if self._error is not None:
            raise self._error
        return self

    def _finish(self, error: Optional[BaseException] = None) -> None:
        self._error = error
        self._finished_at = time.monotonic()
        self._done.set()

    def __bool__(self) -> bool:
        """Refuse to be used as a truth value.

        ``move_to()``/``home()`` used to return a bool. A handle is always
        truthy, so ``if not gantry.home(): abort()`` would silently never
        abort — refusing outright turns that into an immediate error.
        """
        raise TypeError(
            f"{self.description}: a MoveHandle has no truth value — call .wait() "
            "(and read .result) to find out how the move ended"
        )

    def __repr__(self) -> str:
        """Summarise state for the REPL."""
        if not self.done:
            state = f"running {time.monotonic() - self._started_at:.1f}s"
        elif self._error is not None:
            state = f"failed: {type(self._error).__name__}: {self._error}"
        else:
            state = f"done in {self._finished_at - self._started_at:.1f}s"
        return f"<MoveHandle {self.description} — {state}>"

    @classmethod
    def run_inline(cls, description: str, body: Callable[[], Any]) -> "MoveHandle":
        """Run `body` on the calling thread and return an already-finished handle.

        Used when the caller already holds the motion arbiter (library code
        sequencing several moves inside one operation): a background thread
        would block on the arbiter the caller holds. Errors propagate
        directly, exactly as a blocking call's would.
        """
        handle = cls(description)
        try:
            handle.result = body()
        except BaseException as exc:
            handle._finish(exc)
            raise
        handle._finish()
        return handle

    @classmethod
    def run_in_background(
        cls,
        description: str,
        prepare: Callable[[], Callable[[], Any]],
        hold: Callable[[], "object"],
    ) -> "MoveHandle":
        """Run a move on a background thread, surfacing setup errors synchronously.

        The thread enters `hold()` (the motion arbiter's context manager)
        first, then calls `prepare()` — which validates, reads live position
        and fence-checks — and only then hands control back to the caller.
        So a ``FenceViolation``, ``MotionHalted`` or ``MotionBusyError``
        still raises from ``move_to()`` itself, with nothing sent, while the
        traverse returned by `prepare()` runs after the caller has its
        handle back.

        Args:
            description: Human-readable summary of the move.
            prepare: Returns the callable that executes the move.
            hold: Returns the context manager serialising gantry operations.

        Returns:
            The handle, once `prepare()` has succeeded.

        Raises:
            Exception: Anything `hold()` or `prepare()` raised.
        """
        handle = cls(description)
        prepared = threading.Event()
        setup_error: list = []

        def _worker() -> None:
            try:
                with hold():
                    try:
                        execute = prepare()
                    except BaseException as exc:
                        setup_error.append(exc)
                        prepared.set()
                        return
                    prepared.set()
                    handle.result = execute()
            except BaseException as exc:
                if not prepared.is_set():
                    setup_error.append(exc)
                    prepared.set()
                    return
                logger.error("%s failed: %s", description, exc)
                handle._finish(exc)
                return
            handle._finish()

        threading.Thread(target=_worker, daemon=True, name=f"gantry-{description}").start()
        prepared.wait()
        if setup_error:
            handle._finish(setup_error[0])
            raise setup_error[0]
        return handle


__all__ = ["MoveHandle"]
