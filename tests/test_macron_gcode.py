"""Tests for G-code parsing and fence-checked execution.

All offline: FakeSnapConnection for the wire, and a HomingProcedure built
on the same fake for the G28 path.
"""

import math

import pytest

from laguna.robot.macron.commands import MMCCommands, X_AXIS, Y_AXIS, Z_AXIS
from laguna.robot.macron.fences import BoxFence, FenceRegistry, FenceViolation, TrajectoryChecker
from laguna.robot.macron.gcode import (
    GCodeError,
    GCodeExecutionAborted,
    GCodeExecutor,
    GCodeParser,
)
from laguna.robot.macron.commands import IOMap
from laguna.robot.macron.homing import HomingConfig, HomingProcedure
from tests.macron_fixtures import FakeSnapConnection


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

class TestParserModalState:
    def test_g1_absolute_move(self):
        program = GCodeParser().parse("G1 X10 Y20 Z5 F1200")
        assert len(program.moves) == 1
        move = program.moves[0]
        assert move.kind == "LINEAR"
        assert move.target == (10.0, 20.0, 5.0)
        assert move.feed_mm_s == 20.0  # 1200 mm/min -> 20 mm/s

    def test_g0_and_g1_treated_the_same(self):
        program = GCodeParser().parse("G0 X10\nG1 X20")
        assert [m.target for m in program.moves] == [(10.0, 0.0, 0.0), (20.0, 0.0, 0.0)]

    def test_relative_mode_accumulates(self):
        program = GCodeParser().parse("G91\nG1 X10\nG1 X10")
        assert program.moves[0].target == (10.0, 0.0, 0.0)
        assert program.moves[1].target == (20.0, 0.0, 0.0)

    def test_absolute_mode_is_default_and_can_be_restored(self):
        program = GCodeParser().parse("G91\nG1 X10\nG90\nG1 X5")
        assert program.moves[0].target == (10.0, 0.0, 0.0)
        assert program.moves[1].target == (5.0, 0.0, 0.0)  # absolute, not relative to 10

    def test_feed_persists_across_lines_until_changed(self):
        program = GCodeParser().parse("G1 X10 F600\nG1 X20")
        assert program.moves[0].feed_mm_s == 10.0
        assert program.moves[1].feed_mm_s == 10.0

    def test_partial_axis_words_keep_other_axes_unchanged(self):
        program = GCodeParser().parse("G1 X10 Y10 Z10\nG1 X20")
        assert program.moves[1].target == (20.0, 10.0, 10.0)

    def test_g21_is_a_no_op(self):
        program = GCodeParser().parse("G21\nG1 X1")
        assert len(program.moves) == 1

    def test_g20_inches_raises(self):
        with pytest.raises(GCodeError, match="G20"):
            GCodeParser().parse("G20")

    def test_comments_stripped(self):
        program = GCodeParser().parse("G1 X10 ; move to 10\n(this is a comment)\nG1 X20")
        assert len(program.moves) == 2

    def test_unsupported_gcode_raises_naming_it(self):
        with pytest.raises(GCodeError, match="G17"):
            GCodeParser().parse("G17")

    def test_unsupported_mcode_raises_naming_it(self):
        with pytest.raises(GCodeError, match="M104"):
            GCodeParser().parse("M104 S200")

    def test_error_includes_line_number(self):
        with pytest.raises(GCodeError, match="line 2"):
            GCodeParser().parse("G1 X1\nG17\nG1 X2")

    def test_extruder_and_spindle_words_ignored(self):
        program = GCodeParser().parse("G1 X10 E5.2 S12000")
        assert program.moves[0].target == (10.0, 0.0, 0.0)


class TestParserControlCodes:
    def test_g28_produces_home_move(self):
        program = GCodeParser().parse("G28")
        assert program.moves[0].kind == "HOME"

    def test_g4_dwell_p_is_milliseconds(self):
        program = GCodeParser().parse("G4 P500")
        assert program.moves[0].kind == "DWELL"
        assert program.moves[0].dwell_s == 0.5

    def test_g4_dwell_s_is_seconds(self):
        program = GCodeParser().parse("G4 S2")
        assert program.moves[0].dwell_s == 2.0

    def test_m0_and_m1_produce_pause(self):
        program = GCodeParser().parse("M0\nM1")
        assert [m.kind for m in program.moves] == ["PAUSE", "PAUSE"]

    def test_m114_is_a_no_op(self):
        program = GCodeParser().parse("M114")
        assert program.moves == []


class TestParserArcs:
    def test_g2_quarter_circle_ij(self):
        # Quarter circle, center at (10, 0), from (0,0) to (10,10), clockwise... actually
        # for a CCW-consistent quarter arc let's use G3 for simplicity of expectation below.
        program = GCodeParser().parse("G3 X10 Y10 I10 J0")
        move_targets = [m.target for m in program.moves]
        assert len(move_targets) > 1  # tessellated into multiple segments
        # Every tessellated point should lie ~10mm from the arc center (10, 0).
        for x, y, _ in move_targets:
            dist = math.hypot(x - 10.0, y - 0.0)
            assert dist == pytest.approx(10.0, abs=0.06)
        # Final point must be the exact commanded endpoint.
        assert move_targets[-1] == pytest.approx((10.0, 10.0, 0.0))

    def test_g2_r_form_equivalent_to_ij(self):
        program_ij = GCodeParser().parse("G3 X10 Y10 I10 J0")
        program_r = GCodeParser().parse("G3 X10 Y10 R10")
        assert program_ij.moves[-1].target == pytest.approx(program_r.moves[-1].target)

    def test_arc_requires_ij_or_r(self):
        with pytest.raises(GCodeError):
            GCodeParser().parse("G2 X10 Y10")

    def test_arc_zero_radius_raises(self):
        with pytest.raises(GCodeError):
            GCodeParser().parse("G2 X10 Y10 I0 J0")

    def test_helical_arc_interpolates_z(self):
        program = GCodeParser().parse("G3 X10 Y10 Z10 I10 J0")
        assert program.moves[-1].target[2] == pytest.approx(10.0)
        # Z should increase monotonically across the tessellated segments.
        z_values = [m.target[2] for m in program.moves]
        assert z_values == sorted(z_values)

    def test_arc_moves_are_marked_linear_for_downstream_execution(self):
        program = GCodeParser().parse("G3 X10 Y10 I10 J0")
        assert all(m.kind == "LINEAR" for m in program.moves)


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------

def _make_executor(responses, dry_run=False, confirm_cb=None, fences=None):
    conn = FakeSnapConnection(responses)
    cmd = MMCCommands(conn)
    registry = FenceRegistry()
    for fence in fences or []:
        registry.add(fence)
    checker = TrajectoryChecker(registry)
    homing_config = HomingConfig(poll_interval_s=0.001, timeout_s=1.0, backoff_timeout_s=1.0)
    # z_brake_status_input defaults to None (unreachable via ASCII on real
    # hardware — it lives on the responder's own input bank). Stand in a
    # test-only channel here so homing tests can exercise the full
    # brake-confirm flow; this is not a claim about real reachability.
    io_map = IOMap(
        y_brake_output=4, z_brake_output=5, y_brake_status_input=8, z_brake_status_input=1
    )
    homing = HomingProcedure(cmd, homing_config, io_map=io_map)
    executor = GCodeExecutor(
        cmd, checker, homing=homing, axes=(X_AXIS, Y_AXIS, Z_AXIS),
        dry_run=dry_run, confirm_cb=confirm_cb,
    )
    return executor, conn


class TestExecutorTypeGuard:
    def test_execute_rejects_non_checked_trajectory(self):
        executor, _ = _make_executor({})
        with pytest.raises(TypeError):
            executor.execute([(0, 0, 0), (10, 10, 10)])  # a raw list, not a CheckedTrajectory

    def test_execute_rejects_trajectory_not_from_plan(self):
        executor, _ = _make_executor({})
        registry = FenceRegistry()
        other_checker = TrajectoryChecker(registry)
        foreign_trajectory = other_checker.check_and_wrap([(0, 0, 0), (1, 1, 1)])
        with pytest.raises(RuntimeError):
            executor.execute(foreign_trajectory)

    def test_plan_raises_fence_violation_before_any_execution(self):
        executor, conn = _make_executor(
            {}, fences=[BoxFence("post", 5, 15, 5, 15, 0, 10)]
        )
        with pytest.raises(FenceViolation):
            executor.plan("G1 X10 Y10")
        assert conn.sent == []


class TestExecutorLinearMoves:
    def test_executes_group_init_then_moves(self):
        responses = {
            "C1 INI 1 2 5": "0",
            "C1 SPD 20": "20",
            "C1 BMT 10 0 0": "0",
            "C1 MIF": "1",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 F1200")
        executor.execute(trajectory)
        assert conn.sent == ["C1 INI 1 2 5", "C1 SPD 20", "C1 BMT 10 0 0", "C1 MIF"]

    def test_polls_until_move_finished(self):
        calls = {"n": 0}

        def mif_response(cmd):
            calls["n"] += 1
            return "1" if calls["n"] >= 3 else "0"

        responses = {
            "C1 INI 1 2 5": "0",
            "C1 BMT 5 0 0": "0",
            "C1 MIF": mif_response,
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X5")
        executor.execute(trajectory)
        assert conn.sent.count("C1 MIF") == 3

    def test_dry_run_sends_nothing(self):
        executor, conn = _make_executor({}, dry_run=True)
        trajectory = executor.plan("G1 X10 Y10 F600")
        executor.execute(trajectory)
        assert conn.sent == []

    def test_confirm_cb_can_abort_before_sending(self):
        executor, conn = _make_executor({"C1 INI 1 2 5": "0"}, confirm_cb=lambda move: False)
        trajectory = executor.plan("G1 X10")
        with pytest.raises(GCodeExecutionAborted):
            executor.execute(trajectory)
        assert "C1 BMT 10 0 0" not in conn.sent

    def test_confirm_cb_receives_the_move(self):
        seen = []
        executor, conn = _make_executor(
            {"C1 INI 1 2 5": "0", "C1 BMT 10 0 0": "0", "C1 MIF": "1"},
            confirm_cb=lambda move: seen.append(move) or True,
        )
        trajectory = executor.plan("G1 X10")
        executor.execute(trajectory)
        assert len(seen) == 1
        assert seen[0].target == (10.0, 0.0, 0.0)


class TestExecutorHomeDwellPause:
    def test_g28_calls_home_all(self):
        responses = {
            "SOB 5 1": "0", "INB 1": "1",  # Z brake disengage + status confirm
            "SOB 4 1": "0", "INB 8": "1",  # Y brake disengage + status confirm
            "A5 AIC": "0", "A5 CAB": "0", "A5 JOG -10": "-10", "A5 CAT": "1",
            "A5 BST": "0", "A5 MIF": "1", "A5 CAP": "0", "A5 ACP": "0", "A5 ACP 0": "0",
            "A5 MVT 5": "5",
            "A1 AIC": "0", "A1 CAB": "0", "A1 JOG -10": "-10", "A1 CAT": "1",
            "A1 BST": "0", "A1 MIF": "1", "A1 CAP": "0", "A1 ACP": "0", "A1 ACP 0": "0",
            "A1 MVT 5": "5",
            "A2 AIC": "0", "A2 CAB": "0", "A2 JOG -10": "-10", "A2 CAT": "1",
            "A2 BST": "0", "A2 MIF": "1", "A2 CAP": "0", "A2 ACP": "0", "A2 ACP 0": "0",
            "A2 MVT 5": "5",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G28")
        executor.execute(trajectory)
        assert "A5 JOG -10" in conn.sent  # Z (home_order default starts with Z)

    def test_g4_dwell_sleeps(self, monkeypatch):
        slept = []
        monkeypatch.setattr("laguna.robot.macron.gcode.time.sleep", lambda s: slept.append(s))
        executor, conn = _make_executor({})
        trajectory = executor.plan("G4 P250")
        executor.execute(trajectory)
        assert slept == [0.25]

    def test_dry_run_dwell_does_not_sleep(self, monkeypatch):
        slept = []
        monkeypatch.setattr("laguna.robot.macron.gcode.time.sleep", lambda s: slept.append(s))
        executor, conn = _make_executor({}, dry_run=True)
        trajectory = executor.plan("G4 S1")
        executor.execute(trajectory)
        assert slept == []

    def test_m0_pause_calls_confirm_cb(self):
        seen = []
        executor, conn = _make_executor(
            {}, confirm_cb=lambda move: seen.append(move.kind) or True
        )
        trajectory = executor.plan("M0")
        executor.execute(trajectory)
        assert seen == ["PAUSE"]

    def test_m0_pause_with_no_confirm_cb_proceeds(self):
        executor, conn = _make_executor({})
        trajectory = executor.plan("M0")
        executor.execute(trajectory)  # should not raise
