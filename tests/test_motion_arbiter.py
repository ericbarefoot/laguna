"""Tests for the gantry motion arbiter.

The scheduler fires every action in its own daemon thread with no
coordination. These pin the behaviour that stops two of them driving the
gantry at once.
"""

import threading
import time

import pytest

from laguna.robot.motion_arbiter import (
    DEFAULT_ARBITER,
    MotionArbiter,
    MotionBusyError,
)


class TestMotionArbiter:
    def test_uncontended_hold_is_transparent(self):
        arbiter = MotionArbiter()
        assert arbiter.is_held is False
        with arbiter.hold("a move"):
            assert arbiter.is_held is True
            assert arbiter.holder == "a move"
        assert arbiter.is_held is False
        assert arbiter.holder is None

    def test_reentrant_on_the_same_thread(self):
        """scan_with_gantry() holds it, then calls begin_move_to() which holds
        it again. A plain Lock would deadlock here."""
        arbiter = MotionArbiter()
        with arbiter.hold("scan"):
            with arbiter.hold("inner move"):
                # The outermost description stays the reported holder.
                assert arbiter.holder == "scan"
            assert arbiter.is_held is True
        assert arbiter.is_held is False

    def test_second_thread_is_blocked_out(self):
        arbiter = MotionArbiter()
        started = threading.Event()
        release = threading.Event()
        result = {}

        def holder():
            with arbiter.hold("long scan"):
                started.set()
                release.wait(timeout=2)

        t = threading.Thread(target=holder, daemon=True)
        t.start()
        started.wait(timeout=2)
        try:
            with pytest.raises(MotionBusyError) as exc:
                with arbiter.hold("competing move", timeout_s=0.05):
                    result["ran"] = True
            assert "ran" not in result
            # The error must name the holder — "resource busy" alone is
            # useless when the contender is a background scheduler thread.
            assert "long scan" in str(exc.value)
        finally:
            release.set()
            t.join(timeout=2)

    def test_released_after_the_holder_finishes(self):
        arbiter = MotionArbiter()
        done = threading.Event()

        def holder():
            with arbiter.hold("brief"):
                time.sleep(0.02)
            done.set()

        t = threading.Thread(target=holder, daemon=True)
        t.start()
        done.wait(timeout=2)
        t.join(timeout=2)
        with arbiter.hold("now free", timeout_s=0.5):
            assert arbiter.holder == "now free"

    def test_released_even_if_the_body_raises(self):
        arbiter = MotionArbiter()
        with pytest.raises(ValueError):
            with arbiter.hold("doomed"):
                raise ValueError("boom")
        assert arbiter.is_held is False
        with arbiter.hold("after", timeout_s=0.5):
            pass

    def test_timeout_message_explains_the_situation(self):
        arbiter = MotionArbiter()
        started, release = threading.Event(), threading.Event()

        def holder():
            with arbiter.hold("scheduled scan"):
                started.set()
                release.wait(timeout=2)

        t = threading.Thread(target=holder, daemon=True)
        t.start()
        started.wait(timeout=2)
        try:
            with pytest.raises(MotionBusyError, match="scheduled too"):
                with arbiter.hold("other", timeout_s=0.02):
                    pass
        finally:
            release.set()
            t.join(timeout=2)

    def test_default_arbiter_is_shared(self):
        """The subsystems that drive the gantry are constructed independently
        and hold no reference to each other, so a shared default is what makes
        the guarantee actually hold."""
        from laguna.robot.motion_arbiter import DEFAULT_ARBITER as again

        assert again is DEFAULT_ARBITER


class TestGantryUsesTheArbiter:
    def _controller(self, responses=None, arbiter=None):
        from tests.macron_fixtures import FakeSnapConnection
        from laguna.robot.macron.controller import GantryController

        return GantryController(
            connection=FakeSnapConnection(responses or {}),
            mm_per_unit=15.0,
            arbiter=arbiter,
        )

    def test_defaults_to_the_shared_arbiter(self):
        assert self._controller().arbiter is DEFAULT_ARBITER

    def test_accepts_an_explicit_arbiter(self):
        mine = MotionArbiter()
        assert self._controller(arbiter=mine).arbiter is mine

    def test_move_to_holds_the_gantry(self):
        arbiter = MotionArbiter()
        seen = []
        responses = {
            "A6 ACP": lambda cmd: seen.append(arbiter.holder) or "0",
        }
        controller = self._controller(responses, arbiter=arbiter)
        controller.move_to(Theta=0.0)
        assert seen and seen[0] is not None, "move_to ran without holding the gantry"
        assert "move_to" in seen[0]
        assert arbiter.is_held is False, "arbiter not released after the move"

    def test_a_blocked_move_raises_rather_than_queueing(self):
        """A scheduled action that cannot get the gantry should fail loudly
        and let the scheduler log it, rather than piling up threads that all
        wake later and move the machine in some unpredictable order."""
        arbiter = MotionArbiter()
        controller = self._controller({"A6 ACP": "0"}, arbiter=arbiter)
        started, release = threading.Event(), threading.Event()

        def holder():
            with arbiter.hold("scan in progress"):
                started.set()
                release.wait(timeout=2)

        t = threading.Thread(target=holder, daemon=True)
        t.start()
        started.wait(timeout=2)
        try:
            arbiter._timeout_s = 0.02
            with pytest.raises(MotionBusyError, match="scan in progress"):
                controller.move_to(Theta=0.0)
        finally:
            release.set()
            t.join(timeout=2)
