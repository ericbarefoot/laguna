"""Halt latch, non-blocking moves, and the gates every motion path goes through.

Covers the controller-level guarantees CLAUDE.md puts first: motion is never
commandable from a stop condition, a halt cancels a move in flight rather
than letting its next leg start, and every motion entry point — fenced or
deliberately unfenced — refuses under safe_mode. All offline, against
FakeSnapConnection.
"""

from __future__ import annotations

import threading
import time

import pytest

from laguna.robot.macron.connection import SnapMotionError
from laguna.robot.macron.controller import GantryController
from laguna.robot.macron.fences import BoxFence, FenceViolation
from laguna.robot.macron.halt import HaltLatch, HaltLevel, MotionHalted
from laguna.robot.macron.move_handle import MoveHandle
from laguna.robot.motion_arbiter import MotionArbiter, MotionBusyError
from tests.macron_fixtures import FakeSnapConnection


def _responses(**overrides):
    """Every command the stop/brake/read paths send, answered."""
    responses = {}
    for i in (1, 2, 5, 6):
        for verb in ("ACP", "ABT", "BST", "MTR 0", "MTR 1", "MIF", "SPD"):
            responses[f"A{i} {verb}"] = "1" if verb == "MIF" else "0"
    responses.update({
        "SOB 4 0": "0", "SOB 5 0": "0", "SOB 4 1": "0", "SOB 5 1": "0",
        "C1 INI 1 2": "0", "C1 MIF": "1", "C1 SPD": "20",
    })
    responses.update(overrides)
    return responses


def _gantry(responses=None, fences=(), safe_mode=False, arbiter=None):
    conn = FakeSnapConnection(_responses(**(responses or {})))
    gantry = GantryController(
        connection=conn, mm_per_unit=1.0, fences=list(fences),
        arbiter=arbiter or MotionArbiter(),
    )
    gantry._is_connected = True
    gantry._safe_mode = safe_mode
    return gantry, conn


def _bmt_sent(conn):
    return [c for c in conn.sent if " BMT " in c or " JOG " in c]


# ---------------------------------------------------------------------------
# The latch itself
# ---------------------------------------------------------------------------


class TestHaltLatch:
    def test_a_lesser_halt_never_downgrades_a_greater_one(self):
        latch = HaltLatch()
        latch.trip(HaltLevel.ESTOP, "estop()")
        latch.trip(HaltLevel.PAUSE, "pause()")
        assert latch.level is HaltLevel.ESTOP

    def test_resume_cannot_clear_a_stop_or_estop(self):
        latch = HaltLatch()
        latch.trip(HaltLevel.STOP, "stop()")
        assert latch.clear(up_to=HaltLevel.PAUSE) is False
        assert latch.level is HaltLevel.STOP
        assert latch.clear(up_to=HaltLevel.ESTOP) is True
        assert latch.level is None

    def test_a_guard_taken_before_a_trip_raises_after_it(self):
        latch = HaltLatch()
        guard = latch.guard("move")
        guard.check()
        latch.trip(HaltLevel.PAUSE, "pause()")
        with pytest.raises(MotionHalted):
            guard.check()

    def test_clearing_does_not_revive_a_cancelled_move(self):
        """resume() allows *new* motion; the move a pause cancelled stays cancelled."""
        latch = HaltLatch()
        guard = latch.guard("move")
        latch.trip(HaltLevel.PAUSE, "pause()")
        latch.clear(up_to=HaltLevel.PAUSE)
        with pytest.raises(MotionHalted):
            guard.check()

    def test_no_guard_while_tripped(self):
        latch = HaltLatch()
        latch.trip(HaltLevel.PAUSE, "pause()")
        with pytest.raises(MotionHalted, match="resume"):
            latch.guard("move")

    def test_a_trip_waits_for_a_command_being_issued_then_cancels_after_it(self):
        """The check-then-send race: a trip can't land between the check and
        the BMT — it waits for the send, then the halt's own stop follows."""
        latch = HaltLatch()
        guard = latch.guard("move")
        order = []
        in_send = threading.Event()

        def issue():
            with guard.issuing():
                in_send.set()
                time.sleep(0.05)
                order.append("BMT")

        t = threading.Thread(target=issue)
        t.start()
        in_send.wait()
        latch.trip(HaltLevel.PAUSE, "pause()")
        order.append("trip")
        t.join()
        assert order == ["BMT", "trip"]
        with pytest.raises(MotionHalted):
            with guard.issuing():
                pytest.fail("issued a command after the halt")


# ---------------------------------------------------------------------------
# No motion from a stop condition
# ---------------------------------------------------------------------------


class TestMotionRefusedWhileHalted:
    @pytest.mark.parametrize("verb", ["pause", "stop", "estop"])
    def test_every_motion_path_refuses_after_a_halt(self, verb):
        gantry, conn = _gantry()
        getattr(gantry, verb)()
        conn.sent.clear()
        for attempt in (
            lambda: gantry.move_to(X=10.0),
            lambda: gantry.move_to_unfenced("X", 10.0),
            lambda: gantry.jog_unfenced("X", 5.0),
            lambda: gantry.home(),
            lambda: gantry.home_axis("X"),
            lambda: gantry.locate_limit_switch("X"),
        ):
            with pytest.raises(MotionHalted):
                attempt()
        assert _bmt_sent(conn) == []

    def test_estop_then_move_sends_nothing(self):
        """Regression: estop() cut the motors and engaged the brakes, and the
        next move_to() drove BMT straight into them."""
        gantry, conn = _gantry()
        gantry.estop()
        conn.sent.clear()
        with pytest.raises(MotionHalted, match="rearm"):
            gantry.move_to(X=80.0)
        assert conn.sent == []

    def test_resume_allows_motion_after_a_pause(self):
        gantry, conn = _gantry({"C1 BMT 10 0": "0"})
        gantry.pause()
        assert gantry.resume() is None
        gantry.move_to(X=10.0).wait()
        assert "C1 BMT 10 0" in conn.sent

    def test_resume_does_not_undo_an_estop(self):
        gantry, _ = _gantry()
        gantry.estop()
        assert "rearm" in gantry.resume()
        assert gantry.halted == "estop"

    def test_rearm_clears_the_latch_but_leaves_safe_mode_on(self):
        gantry, conn = _gantry()
        gantry.estop()
        conn.sent.clear()
        assert gantry.rearm() is True
        assert gantry.halted is None
        assert gantry._safe_mode is True
        assert not any(c.endswith("MTR 1") for c in conn.sent)
        with pytest.raises(SnapMotionError, match="safe_mode"):
            gantry.move_to(X=10.0)

    def test_a_fresh_connect_clears_a_stop_but_not_an_estop(self):
        gantry, conn = _gantry()
        gantry.stop()
        gantry._is_connected = False
        gantry.connect()
        assert gantry.halted is None

        gantry.estop()
        gantry._is_connected = False
        gantry.connect()
        assert gantry.halted == "estop"

    def test_status_reports_the_halt(self):
        gantry, _ = _gantry()
        gantry.pause()
        assert gantry.get_status()["halted"] == "pause"


# ---------------------------------------------------------------------------
# A halt cancels the move in flight
# ---------------------------------------------------------------------------


class TestHaltCancelsMoveInFlight:
    def test_pause_mid_program_stops_the_next_leg_being_issued(self):
        """Regression: pause() sent BST, but the executing thread saw "move
        finished" and issued the program's next leg anyway."""
        gantry, conn = _gantry()
        conn.responses["C1 BMT 10 0"] = "0"
        conn.responses["C1 BMT 20 0"] = "0"

        def mif(_cmd):
            gantry.pause()  # a halt arrives while the first leg is in flight
            return "1"

        conn.responses["C1 MIF"] = mif
        gantry.gcode.sync_position_from_hardware()
        trajectory = gantry.gcode.plan("G90\nG1 X10 Y0\nG1 X20 Y0")
        guard = gantry._halt.guard("program")
        with pytest.raises(MotionHalted):
            gantry.gcode.execute(trajectory, guard=guard)
        assert "C1 BMT 10 0" in conn.sent
        assert "C1 BMT 20 0" not in conn.sent

    def test_pause_from_another_thread_ends_a_running_move_to(self):
        """The notebook case: move_to() returns at once, and a pause() cell
        run straight after it cancels the move."""
        gantry, conn = _gantry({"C1 BMT 10 0": "0"})
        moving = threading.Event()

        def mif(_cmd):
            moving.set()
            return "0"  # never finishes on its own

        conn.responses["C1 MIF"] = mif
        handle = gantry.move_to(X=10.0)
        assert moving.wait(timeout=2), "move never started"
        assert not handle.done
        gantry.pause()
        with pytest.raises(MotionHalted):
            handle.wait(timeout=2)
        assert "A1 BST" in conn.sent

    def test_the_arbiter_is_released_once_a_halted_move_ends(self):
        arbiter = MotionArbiter()
        gantry, conn = _gantry({"C1 BMT 10 0": "0"}, arbiter=arbiter)
        conn.responses["C1 MIF"] = lambda _c: (gantry.pause(), "0")[1]
        with pytest.raises(MotionHalted):
            gantry.move_to(X=10.0).wait(timeout=2)
        assert arbiter.is_held is False


# ---------------------------------------------------------------------------
# Non-blocking move_to()
# ---------------------------------------------------------------------------


class TestNonBlockingMoveTo:
    def test_returns_before_the_move_finishes(self):
        gantry, conn = _gantry({"C1 BMT 10 0": "0"})
        release = threading.Event()
        conn.responses["C1 MIF"] = lambda _c: "1" if release.is_set() else "0"
        handle = gantry.move_to(X=10.0)
        assert isinstance(handle, MoveHandle)
        assert not handle.done
        release.set()
        assert handle.wait(timeout=2).succeeded

    def test_a_fence_violation_raises_from_the_call_itself_with_nothing_sent(self):
        gantry, conn = _gantry(fences=[BoxFence("post", 4, 6, -1, 1, -1, 1)])
        with pytest.raises(FenceViolation):
            gantry.move_to(X=10.0)
        assert _bmt_sent(conn) == []

    def test_a_second_move_while_one_runs_is_refused_immediately(self):
        gantry, conn = _gantry({"C1 BMT 10 0": "0"})
        release = threading.Event()
        conn.responses["C1 MIF"] = lambda _c: "1" if release.is_set() else "0"
        first = gantry.move_to(X=10.0)
        start = time.monotonic()
        with pytest.raises(MotionBusyError):
            gantry.move_to(X=20.0)
        assert time.monotonic() - start < 1.0, "queued instead of refusing"
        release.set()
        first.wait(timeout=2)

    def test_inside_an_arbiter_hold_it_runs_inline(self):
        """Library code holding the gantry (acquire_scan, a survey pass)
        sequences moves; a background thread would deadlock on the hold."""
        arbiter = MotionArbiter()
        gantry, conn = _gantry({"C1 BMT 10 0": "0"}, arbiter=arbiter)
        with arbiter.hold("survey pass"):
            handle = gantry.move_to(X=10.0)
            assert handle.done and handle.succeeded

    def test_a_failure_during_the_traverse_is_kept_on_the_handle(self):
        gantry, conn = _gantry({"C1 BMT 10 0": SnapMotionError(7, "wire fault")})
        handle = gantry.move_to(X=10.0)
        with pytest.raises(SnapMotionError, match="wire fault"):
            handle.wait(timeout=2)
        assert isinstance(handle.error, SnapMotionError)

    def test_a_handle_refuses_to_be_a_truth_value(self):
        """move_to()/home() used to return a bool; `if not gantry.home():`
        must not silently pass now that they return a handle."""
        handle = MoveHandle.run_inline("x", lambda: None)
        with pytest.raises(TypeError, match="wait"):
            bool(handle)

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_targets_are_refused(self, bad):
        """A NaN target formatted as "nan" and silently backfilled — no move."""
        gantry, conn = _gantry()
        with pytest.raises(ValueError, match="finite"):
            gantry.move_to(X=bad)
        assert conn.sent == []

    @pytest.mark.parametrize("bad", [0.0, -5.0, float("nan")])
    def test_non_positive_speeds_are_refused(self, bad):
        gantry, conn = _gantry()
        with pytest.raises(ValueError, match="speed"):
            gantry.move_to(X=10.0, speed=bad)


# ---------------------------------------------------------------------------
# safe_mode is checked client-side on every path, whatever the transport
# ---------------------------------------------------------------------------


class TestSafeModeOnEveryPath:
    def test_move_to_under_safe_mode_sends_nothing_on_a_plain_transport(self):
        """Regression: on rs232/ethernet (no transport gate) a fenced
        move_to() under safe_mode=True sent C1 BMT to the wire."""
        gantry, conn = _gantry(safe_mode=True)
        with pytest.raises(SnapMotionError, match="safe_mode"):
            gantry.move_to(X=50.0)
        assert conn.sent == []

    @pytest.mark.parametrize("call", [
        lambda g: g.move_to_unfenced("X", 10.0),
        lambda g: g.jog_unfenced("X", 5.0),
        lambda g: g.home(),
        lambda g: g.home_axis("X"),
        lambda g: g.locate_limit_switch("X"),
    ])
    def test_every_other_motion_path_refuses_too(self, call):
        gantry, conn = _gantry(safe_mode=True)
        with pytest.raises(SnapMotionError, match="safe_mode"):
            call(gantry)
        assert _bmt_sent(conn) == []

    def test_motion_refused_while_disconnected(self):
        gantry, conn = _gantry()
        gantry._is_connected = False
        with pytest.raises(SnapMotionError, match="not connected"):
            gantry.move_to(X=10.0)

    def test_disabling_safe_mode_applies_soft_limits_on_a_plain_transport(self):
        """Regression: limits were only written by connect(), so enabling
        motion later left the configured travel limits unset."""
        conn = FakeSnapConnection(_responses(**{
            "A1 NLT -5": "-5", "A1 PLT 100": "100", "A1 NLT": "-5", "A1 PLT": "100",
        }))
        gantry = GantryController(
            connection=conn, mm_per_unit=1.0, soft_limits={"X": (-5.0, 100.0)},
        )
        gantry._is_connected = True
        gantry.set_safe_mode(False)
        assert "A1 NLT -5" in conn.sent and "A1 PLT 100" in conn.sent


# ---------------------------------------------------------------------------
# Deliberately unfenced motion
# ---------------------------------------------------------------------------


class TestUnfencedMotion:
    def test_move_to_unfenced_ignores_fences_and_resyncs(self):
        gantry, conn = _gantry(
            {"A1 BMT 10": "0"}, fences=[BoxFence("post", 4, 6, -1, 1, -1, 1)],
        )
        with pytest.raises(FenceViolation):
            gantry.move_to(X=10.0)
        conn.sent.clear()
        gantry.move_to_unfenced("X", 10.0).wait(timeout=2)
        assert "A1 BMT 10" in conn.sent
        assert conn.sent[-4:] == ["A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP"]

    def test_move_to_unfenced_is_logged_as_unfenced(self, caplog):
        gantry, _ = _gantry({"A1 BMT 10": "0"})
        with caplog.at_level("WARNING"):
            gantry.move_to_unfenced("X", 10.0).wait(timeout=2)
        assert "NO FENCE CHECK" in caplog.text

    def test_jog_unfenced_zero_always_stops_even_when_halted(self):
        gantry, conn = _gantry({"A1 JOG 0": "0"})
        gantry.estop()
        gantry.jog_unfenced("X", 0)
        assert "A1 JOG 0" in conn.sent

    def test_jog_unfenced_refuses_while_another_motion_holds_the_gantry(self):
        arbiter = MotionArbiter()
        gantry, conn = _gantry({"A1 JOG 5": "0"}, arbiter=arbiter)
        holding, release = threading.Event(), threading.Event()

        def holder():
            with arbiter.hold("scan"):
                holding.set()
                release.wait(2)

        t = threading.Thread(target=holder)
        t.start()
        holding.wait(2)
        try:
            with pytest.raises(MotionBusyError):
                gantry.jog_unfenced("X", 5.0)
        finally:
            release.set()
            t.join()
        assert "A1 JOG 5" not in conn.sent


# ---------------------------------------------------------------------------
# Scan passes
# ---------------------------------------------------------------------------


class TestScanMoves:
    def test_begin_scan_move_requires_the_arbiter(self):
        gantry, _ = _gantry()
        with pytest.raises(RuntimeError, match="arbiter"):
            gantry.begin_scan_move("X", 10.0, 5.0)

    def test_begin_scan_move_is_fence_checked(self):
        arbiter = MotionArbiter()
        gantry, conn = _gantry(fences=[BoxFence("post", 4, 6, -1, 1, -1, 1)], arbiter=arbiter)
        with arbiter.hold("scan"):
            with pytest.raises(FenceViolation):
                gantry.begin_scan_move("X", 10.0, 5.0)
        assert _bmt_sent(conn) == []

    def test_begin_scan_move_starts_the_pass(self):
        arbiter = MotionArbiter()
        gantry, conn = _gantry({"A1 SPD 5": "5", "A1 BMT 10": "0"}, arbiter=arbiter)
        with arbiter.hold("scan"):
            guard = gantry.begin_scan_move("X", 10.0, 5.0)
        assert conn.sent[-2:] == ["A1 SPD 5", "A1 BMT 10"]
        guard.check()

    def test_plan_scan_move_accepts_a_blc_token(self):
        arbiter = MotionArbiter()
        gantry, conn = _gantry(fences=[BoxFence("post", 4, 6, -1, 1, -1, 1)], arbiter=arbiter)
        with arbiter.hold("scan"):
            with pytest.raises(FenceViolation):
                gantry.plan_scan_move("A1", 10.0)

    def test_halting_a_pi_agent_scan_cancels_it_and_reports_partial_data(self):
        gantry, conn = _gantry()
        conn.is_scan_running = True
        stops = []
        conn.stop_scan = lambda: stops.append(True)
        note = gantry.estop()
        assert stops == [True]
        assert "partial" in note


# ---------------------------------------------------------------------------
# estop speed
# ---------------------------------------------------------------------------


class TestEstop:
    def test_does_not_read_positions_before_returning(self):
        """FlumeLab estops the gantry first; position reads here delayed the
        pump and valves. FlumeLab resyncs afterwards instead."""
        gantry, conn = _gantry()
        gantry.estop()
        assert not any(c.endswith("ACP") for c in conn.sent)

    def test_one_unexpected_failure_does_not_abandon_the_rest(self):
        gantry, conn = _gantry({"A1 ABT": RuntimeError("transport hiccup")})
        gantry.estop()
        assert "A2 ABT" in conn.sent
        assert "SOB 4 0" in conn.sent
        assert "A6 MTR 0" in conn.sent
