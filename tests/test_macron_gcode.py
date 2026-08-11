"""Tests for G-code parsing and fence-checked execution.

All offline: FakeSnapConnection for the wire, and a HomingProcedure built
on the same fake for the G28 path.
"""

import math

import pytest

from laguna.robot.macron.commands import MMCCommands, THETA_AXIS, X_AXIS, Y_AXIS, Z_AXIS
from laguna.robot.macron.connection import SnapMotionError
from laguna.robot.macron.fences import BoxFence, FenceRegistry, FenceViolation, TrajectoryChecker
from laguna.robot.macron import gcode as gcode_module
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

    def test_a_word_resolves_theta_absolute(self):
        program = GCodeParser().parse("G1 X10 A90")
        assert program.moves[0].target == (10.0, 0.0, 0.0)
        assert program.moves[0].theta == 90.0

    def test_a_word_resolves_theta_relative(self):
        program = GCodeParser().parse("G91\nG1 A10\nG1 A10")
        assert program.moves[0].theta == 10.0
        assert program.moves[1].theta == 20.0

    def test_no_a_word_means_theta_is_none_not_unchanged(self):
        """Unlike X/Y/Z (which always resolve, backfilling the current
        value), a line with no A word must produce theta=None — "don't
        move Theta at all", not "hold Theta's last-seen position".
        """
        program = GCodeParser().parse("G1 A90\nG1 X10")
        assert program.moves[0].theta == 90.0
        assert program.moves[1].theta is None

    def test_start_theta_seeds_relative_moves(self):
        program = GCodeParser().parse("G91\nG1 A10", start_theta=45.0)
        assert program.moves[0].theta == 55.0

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

def _make_executor(responses, dry_run=False, confirm_cb=None, fences=None, with_theta_group=True):
    conn = FakeSnapConnection(responses)
    cmd = MMCCommands(conn)
    # A second MMCCommands for the Z/Theta responder-node group (C2),
    # sharing the same wire — mirrors GantryController.__init__. Building
    # it costs nothing (INI is lazy), so most tests get it for free; pass
    # with_theta_group=False for the few that specifically want to exercise
    # the "no theta_cmd configured" error path.
    theta_cmd = MMCCommands(conn, group_index=2, group_axes=(Z_AXIS, THETA_AXIS)) if with_theta_group else None
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
        cmd, checker, homing=homing, axes=(X_AXIS, Y_AXIS), z_axis=Z_AXIS, theta_axis=THETA_AXIS,
        theta_cmd=theta_cmd, dry_run=dry_run, confirm_cb=confirm_cb,
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
        # group init
        # no longer spans Z (the responder node) — see GCodeExecutor's class
        # docstring. X10 with Z unchanged means no Z leg is sent at all.
        responses = {
            "C1 INI 1 2": "0",
            "C1 SPD 20": "20",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "A1 ACP": "10",  # post-move resync of X/Y from hardware
            "A2 ACP": "0",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 F1200")
        executor.execute(trajectory)
        assert conn.sent == [
            "C1 INI 1 2", "C1 SPD 20", "C1 BMT 10 0", "C1 MIF", "A1 ACP", "A2 ACP",
        ]

    def test_group_init_is_sent_once_not_per_execute(self):
        """INI is remembered across execute() calls — a 40-move
        run was re-sending an identical `C1 INI 1 2` 40 times.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 SPD 20": "20",
            "C1 BMT 10 0": "0",
            "C1 BMT 20 0": "0",
            "C1 MIF": "1",
            "A1 ACP": "10",  # post-move resync of X/Y from hardware
            "A2 ACP": "0",
        }
        executor, conn = _make_executor(responses)
        for target in ("G1 X10 F1200", "G1 X20 F1200"):
            executor.execute(executor.plan(target))
        assert conn.sent.count("C1 INI 1 2") == 1
        assert conn.sent.count("C1 BMT 10 0") == 1
        assert conn.sent.count("C1 BMT 20 0") == 1

    def test_reset_group_init_forces_ini_to_be_resent(self):
        """A power-cycle/reflash can clear the controller's group state —
        GantryController.connect() calls this so the next move re-inits.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 SPD 20": "20",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "A1 ACP": "10",  # post-move resync of X/Y from hardware
            "A2 ACP": "0",
        }
        executor, conn = _make_executor(responses)
        executor.execute(executor.plan("G1 X10 F1200"))
        executor.reset_group_init()
        executor._current_pos = (0.0, 0.0, 0.0)  # pretend we're back at the start
        executor.execute(executor.plan("G1 X10 F1200"))
        assert conn.sent.count("C1 INI 1 2") == 2

    def test_polls_until_move_finished(self):
        calls = {"n": 0}

        def mif_response(cmd):
            calls["n"] += 1
            return "1" if calls["n"] >= 3 else "0"

        responses = {
            "C1 INI 1 2": "0",
            "C1 BMT 5 0": "0",
            "C1 SPD": "20",  # no F word — read X/Y's own speed to predict duration
            "C1 MIF": mif_response,
            "A1 ACP": "5",  # post-move resync of X/Y from hardware
            "A2 ACP": "0",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X5")
        executor.execute(trajectory)
        assert conn.sent.count("C1 MIF") == 3

    def test_no_f_word_predicts_off_the_axis_own_slow_speed_not_zero(self, monkeypatch):
        """A move with no F word used to predict a 0.0s duration (speed
        unknown), which floors the timeout at MIN_TIMEOUT_S regardless of
        how slow the axis is actually configured — a legitimately slow
        move (e.g. X/Y left at 1 mm/s from an earlier command) would then
        time out on a move that was still correctly in progress. Reading
        X/Y's own currently-configured SPD fixes that: predicted_move_s is
        now called with the live-read speed (1, from "C1 SPD"), not None.
        """
        seen_speeds = []
        real_predicted_move_s = gcode_module.predicted_move_s
        monkeypatch.setattr(
            gcode_module,
            "predicted_move_s",
            lambda distance, speed: (seen_speeds.append(speed), real_predicted_move_s(distance, speed))[1],
        )
        responses = {
            "C1 INI 1 2": "0",
            "C1 BMT 200 0": "0",
            "C1 SPD": "1",  # slow — X/Y left at 1 mm/s from some earlier move
            "C1 MIF": "1",
            "A1 ACP": "200",
            "A2 ACP": "0",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X200")  # no F word
        executor.execute(trajectory)
        assert conn.sent.count("C1 SPD") == 1
        assert seen_speeds == [1.0]  # not None — the old speed=None/predicted_s=0.0 bug

    def test_moves_z_concurrently_with_the_xy_group_scaled_to_match_duration(self):
        """A move touching both Z and X/Y fires the Z leg and the XY group
        leg as two independent non-blocking BMTs, back-to-back — both BMTs
        land on the wire before either MIF poll starts. Z travels less than
        X/Y here (3mm vs 10mm), so it's the "short" leg: before firing
        either BMT, the executor reads X/Y's currently-configured ramp
        (C1 ACL/DCL, the scaling reference) *and* Z's own currently-
        configured ramp and speed (A5 ACL/DCL/SPD, so they can be restored
        afterward), and scales Z's speed *and* accel/decel down by
        k=3/10=0.3, so both legs' trapezoids take the same time — not just
        the same nominal F. X/Y (the "long" leg) is left at the plain
        nominal feed rate and whatever ramp it already had. Once both legs
        finish, Z's ramp *and* speed are restored to their own original
        (unscaled) values — this driver otherwise never touches
        ACL/DCL/SPD, so a scaled-down value must not outlive this one move
        (see _execute_concurrent_pair's docstring). It still never sends a
        3-axis group command (`C1 INI 1 2 5`, confirmed on the bench to
        return error 1010): Z stays on its own single-axis command since
        Theta isn't moving with it.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 ACL": "50",
            "C1 DCL": "40",
            "A5 ACL": "25",  # Z's own ramp before this move — restored after
            "A5 DCL": "20",
            "A5 SPD": "8",  # Z's own speed before this move — restored after
            "A5 ACL 15": "15",
            "A5 DCL 12": "12",
            "A5 SPD 3": "3",
            "A5 BMT 3": "0",
            "A5 MIF": "1",
            "C1 SPD 10": "10",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "A5 ACL 25": "25",  # restore
            "A5 DCL 20": "20",
            "A5 SPD 8": "8",  # restore
            "A1 ACP": "10",  # post-move resync of X/Y/Z from hardware
            "A2 ACP": "0",
            "A5 ACP": "3",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 Z3 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C1 INI 1 2", "C1 ACL", "C1 DCL",  # read X/Y's ramp (scaling reference)
            "A5 ACL", "A5 DCL", "A5 SPD",  # read Z's own ramp/speed (to restore later)
            "A5 ACL 15", "A5 DCL 12", "A5 SPD 3", "A5 BMT 3",  # Z, scaled by k=0.3
            "C1 SPD 10", "C1 BMT 10 0",  # X/Y, unscaled
            "A5 MIF", "C1 MIF",
            "A5 ACL 25", "A5 DCL 20", "A5 SPD 8",  # Z's ramp/speed restored
            "A1 ACP", "A2 ACP", "A5 ACP",  # resync _current_pos from hardware
        ]
        assert "C1 INI 1 2 5" not in conn.sent

    def test_skips_scaling_when_reference_ramp_is_zero_or_negative(self):
        """If the scaling-reference leg's ACL/DCL reads back as 0 or
        negative — e.g. the X/Y group was just C1 INI'd this session and
        nothing has ever set its ACL/DCL, per _read_xy_ramp's "querying
        ACL/DCL before INI is untested" note — scaling Z's ramp by k would
        send a 0-or-negative ACL/DCL to the controller, which the ASCII
        API rejects outright (escape codes 16/17, "User Accels/Decels 0 Or
        Negative"). The executor must detect this and skip scaling for
        this move (both legs unscaled) rather than sending it.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 ACL": "0",  # never explicitly set on this group yet
            "C1 DCL": "0",
            "A5 SPD 10": "10",  # nominal feed rate (F600 -> 10mm/s), not scaled
            "A5 BMT 3": "0",
            "A5 MIF": "1",
            "C1 SPD 10": "10",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "A1 ACP": "10",  # post-move resync of X/Y/Z from hardware
            "A2 ACP": "0",
            "A5 ACP": "3",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 Z3 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C1 INI 1 2", "C1 ACL", "C1 DCL",  # read X/Y's ramp (scaling reference) — 0/0
            "A5 SPD 10", "A5 BMT 3",  # Z, unscaled — nominal feed rate, ramp untouched
            "C1 SPD 10", "C1 BMT 10 0",  # X/Y, unscaled
            "A5 MIF", "C1 MIF",
            "A1 ACP", "A2 ACP", "A5 ACP",  # resync _current_pos from hardware
        ]
        assert "A5 ACL" not in conn.sent
        assert "A5 DCL" not in conn.sent

    def test_scales_the_short_leg_even_with_no_f_word(self):
        """A move with no F word at all used to skip scaling entirely,
        leaving each leg at whatever SPD it already independently had —
        which could differ arbitrarily and desync completion by seconds
        even though nothing about this move asked for that. Now the
        "nominal" speed the long leg (X/Y here) keeps is read live from
        its own currently-configured SPD (C1 SPD, no argument) instead of
        requiring an F word to supply it, and the short leg (Z) is scaled
        from that reading exactly as it would be from an explicit F.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 ACL": "50",
            "C1 DCL": "40",
            "C1 SPD": "20",  # X/Y's own current speed — the implied nominal feed
            "A5 ACL": "25",  # Z's own ramp before this move — restored after
            "A5 DCL": "20",
            "A5 SPD": "8",  # Z's own speed before this move — restored after
            "A5 ACL 15": "15",
            "A5 DCL 12": "12",
            "A5 SPD 6": "6",  # 0.3 * 20 (nominal read from C1 SPD, not an F word)
            "A5 BMT 3": "0",
            "A5 MIF": "1",
            "C1 BMT 10 0": "0",  # no C1 SPD sent — X/Y stays at the 20 it already had
            "C1 MIF": "1",
            "A5 ACL 25": "25",  # restore
            "A5 DCL 20": "20",
            "A5 SPD 8": "8",  # restore
            "A1 ACP": "10",  # post-move resync of X/Y/Z from hardware
            "A2 ACP": "0",
            "A5 ACP": "3",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 Z3")  # no F word
        executor.execute(trajectory)
        assert conn.sent == [
            "C1 INI 1 2", "C1 ACL", "C1 DCL",  # read X/Y's ramp (scaling reference)
            "C1 SPD",  # read X/Y's own speed — the implied nominal feed
            "A5 ACL", "A5 DCL", "A5 SPD",  # read Z's own ramp/speed (to restore later)
            "A5 ACL 15", "A5 DCL 12", "A5 SPD 6", "A5 BMT 3",  # Z, scaled by k=0.3
            "C1 BMT 10 0",  # X/Y, unscaled — no SPD sent, already at the nominal
            "A5 MIF",
            # X/Y's own SPD was never assigned above (it's already the value
            # just read) — _poll_xy_leg's duration prediction reads it again.
            "C1 SPD", "C1 MIF",
            "A5 ACL 25", "A5 DCL 20", "A5 SPD 8",  # Z's ramp/speed restored
            "A1 ACP", "A2 ACP", "A5 ACP",  # resync _current_pos from hardware
        ]

    def test_second_legs_poll_wait_is_reduced_by_the_first_legs_elapsed_time(self, monkeypatch):
        """Both BMTs are already on the wire and physically running
        concurrently before either poll starts — polling them is
        necessarily sequential in Python, but the second poll's
        predicted-duration sleep must not restart from zero. A well-
        synced pair finishes at nearly the same real time; without
        subtracting however long the first poll's own wait already
        consumed, the second poll would blindly sleep through its full
        predicted duration *again* on top of that — this is what made the
        REPL return ~10-15s after the gantry had visibly already stopped.
        """
        fake_clock = {"t": 100.0}
        monkeypatch.setattr("laguna.robot.macron.gcode.time.monotonic", lambda: fake_clock["t"])

        responses = {
            "C1 INI 1 2": "0", "C1 ACL": "50", "C1 DCL": "40",
            "A5 ACL": "25", "A5 DCL": "20", "A5 SPD": "8",
            "A5 ACL 15": "15", "A5 DCL 12": "12", "A5 SPD 3": "3", "A5 BMT 3": "0",
            "C1 SPD 10": "10", "C1 BMT 10 0": "0",
            "A5 ACL 25": "25", "A5 DCL 20": "20", "A5 SPD 8": "8",
            "A1 ACP": "10", "A2 ACP": "0", "A5 ACP": "3",
        }
        executor, conn = _make_executor(responses)

        def fake_poll_z_leg(*args, **kwargs):
            fake_clock["t"] += 5.0  # simulate this poll's own wait taking 5s

        monkeypatch.setattr(executor, "_poll_z_leg", fake_poll_z_leg)

        captured = {}

        def fake_poll_xy_leg(*args, **kwargs):
            captured["already_elapsed_s"] = kwargs.get("already_elapsed_s")

        monkeypatch.setattr(executor, "_poll_xy_leg", fake_poll_xy_leg)

        trajectory = executor.plan("G1 X10 Z3 F600")
        executor.execute(trajectory)

        assert captured["already_elapsed_s"] == pytest.approx(5.0)

    def test_moves_z_and_theta_concurrently_with_the_xy_group_scaled_to_match_duration(self):
        """A move touching X/Y, Z, and Theta (A) together fires the Z/Theta
        group's BMT (C2) and the XY group's BMT (C1) back-to-back,
        non-blocking, then polls both — the full emulated 4-axis case from
        issue #24. Z/Theta is the "short" leg here: its "distance" is
        max(|delta Z|, |delta Theta|) = max(3, 2) = 3 (Z dominates), still
        less than X/Y's 10mm, so its speed and ramp are scaled by k=0.3
        from X/Y's ramp, same as the Z-only case above — and its own prior
        ramp *and speed* are restored afterward the same way.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 ACL": "50",
            "C1 DCL": "40",
            "C2 INI 5 6": "0",
            "C2 ACL": "25",  # Z/Theta's own ramp before this move — restored after
            "C2 DCL": "20",
            "C2 SPD": "8",  # Z/Theta's own speed before this move — restored after
            "C2 ACL 15": "15",
            "C2 DCL 12": "12",
            "C2 SPD 3": "3",
            "C2 BMT 3 2": "0",
            "C2 MIF": "1",
            "C1 SPD 10": "10",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "C2 ACL 25": "25",  # restore
            "C2 DCL 20": "20",
            "C2 SPD 8": "8",  # restore
            "A1 ACP": "10",  # post-move resync of X/Y/Z/Theta from hardware
            "A2 ACP": "0",
            "A5 ACP": "3",
            "A6 ACP": "2",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 Z3 A2 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C1 INI 1 2", "C1 ACL", "C1 DCL",  # read X/Y's ramp (scaling reference)
            "C2 INI 5 6", "C2 ACL", "C2 DCL", "C2 SPD",  # read Z/Theta's own ramp/speed (to restore later)
            "C2 ACL 15", "C2 DCL 12", "C2 SPD 3", "C2 BMT 3 2",  # Z/Theta, scaled
            "C1 SPD 10", "C1 BMT 10 0",  # X/Y, unscaled
            "C2 MIF", "C1 MIF",
            "C2 ACL 25", "C2 DCL 20", "C2 SPD 8",  # Z/Theta's ramp/speed restored
            "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP",  # resync _current_pos/_current_theta from hardware
        ]

    def test_moves_z_concurrently_with_z_as_the_long_leg(self):
        """Opposite branch from the two tests above: Z travels further than
        X/Y (20 vs 5), so Z is the "long" leg (nominal feed rate, ramp
        untouched) and X/Y is scaled down by k=5/20=0.25 from Z's
        currently-configured ramp, read via single-axis A5 ACL/DCL
        queries (no group involved for a plain Z leg). X/Y's own prior
        ramp and speed are read too (C1 ACL/DCL/SPD) and restored once
        both legs finish — otherwise a later X/Y move issued with no F
        word (e.g. a direct AxisHandle.move_to()) would silently run at
        this leftover scaled-down speed.
        """
        responses = {
            "A5 ACL": "20",
            "A5 DCL": "16",
            "C1 INI 1 2": "0",
            "C1 ACL": "12",  # X/Y's own ramp before this move — restored after
            "C1 DCL": "10",
            "C1 SPD": "8",  # X/Y's own speed before this move — restored after
            "A5 SPD 10": "10",
            "A5 BMT 20": "0",
            "A5 MIF": "1",
            "C1 ACL 5": "5",
            "C1 DCL 4": "4",
            "C1 SPD 2.5": "2.5",
            "C1 BMT 5 0": "0",
            "C1 MIF": "1",
            "C1 ACL 12": "12",  # restore
            "C1 DCL 10": "10",
            "C1 SPD 8": "8",  # restore
            "A1 ACP": "5",  # post-move resync of X/Y/Z from hardware
            "A2 ACP": "0",
            "A5 ACP": "20",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X5 Z20 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "A5 ACL", "A5 DCL",  # read Z's ramp (scaling reference)
            "C1 INI 1 2", "C1 ACL", "C1 DCL", "C1 SPD",  # read X/Y's own ramp/speed (to restore later)
            "A5 SPD 10", "A5 BMT 20",  # Z, unscaled
            "C1 ACL 5", "C1 DCL 4", "C1 SPD 2.5", "C1 BMT 5 0",  # X/Y, scaled by k=0.25
            "A5 MIF", "C1 MIF",
            "C1 ACL 12", "C1 DCL 10", "C1 SPD 8",  # X/Y's ramp/speed restored
            "A1 ACP", "A2 ACP", "A5 ACP",  # resync _current_pos from hardware
        ]

    def test_moves_z_and_theta_concurrently_with_z_theta_as_the_long_leg(self):
        """Same opposite-branch coverage as above, but for the Z/Theta
        group (C2) rather than a plain Z leg. Its "distance" is
        max(|delta Z|, |delta Theta|) = max(20, 15) = 20 (Z dominates),
        still further than X/Y's 5mm, so Z/Theta stays "long" and X/Y is
        scaled down by k=5/20=0.25 from the group's currently-configured
        ramp (C2 ACL/DCL); X/Y's own prior ramp *and speed* are restored
        afterward.
        """
        responses = {
            "C2 INI 5 6": "0",
            "C2 ACL": "20",
            "C2 DCL": "16",
            "C1 INI 1 2": "0",
            "C1 ACL": "12",  # X/Y's own ramp before this move — restored after
            "C1 DCL": "10",
            "C1 SPD": "8",  # X/Y's own speed before this move — restored after
            "C2 SPD 10": "10",
            "C2 BMT 20 15": "0",
            "C2 MIF": "1",
            "C1 ACL 5": "5",
            "C1 DCL 4": "4",
            "C1 SPD 2.5": "2.5",
            "C1 BMT 5 0": "0",
            "C1 MIF": "1",
            "C1 ACL 12": "12",  # restore
            "C1 DCL 10": "10",
            "C1 SPD 8": "8",  # restore
            "A1 ACP": "5",  # post-move resync of X/Y/Z/Theta from hardware
            "A2 ACP": "0",
            "A5 ACP": "20",
            "A6 ACP": "15",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X5 Z20 A15 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C2 INI 5 6", "C2 ACL", "C2 DCL",  # read Z/Theta's ramp (scaling reference)
            "C1 INI 1 2", "C1 ACL", "C1 DCL", "C1 SPD",  # read X/Y's own ramp/speed (to restore later)
            "C2 SPD 10", "C2 BMT 20 15",  # Z/Theta, unscaled
            "C1 ACL 5", "C1 DCL 4", "C1 SPD 2.5", "C1 BMT 5 0",  # X/Y, scaled by k=0.25
            "C2 MIF", "C1 MIF",
            "C1 ACL 12", "C1 DCL 10", "C1 SPD 8",  # X/Y's ramp/speed restored
            "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP",  # resync _current_pos/_current_theta from hardware
        ]

    def test_zt_distance_is_dominated_by_theta_not_just_z(self):
        """Z alone moves only 2mm (less than X/Y's 5mm), which would make
        Z/Theta look like the "short" leg if distance were Z-only — but
        Theta rotates 50 units, so max(|delta Z|, |delta Theta|)=50 makes
        Z/Theta the "long" leg instead (unscaled) and X/Y the "short" one
        (k=5/50=0.1). This is exactly the case _zt_distance exists for:
        a Theta-dominated move must not get scaled down as if it were
        short just because Z's own contribution is small.
        """
        responses = {
            "C2 INI 5 6": "0",
            "C2 ACL": "20",
            "C2 DCL": "16",
            "C1 INI 1 2": "0",
            "C1 ACL": "12",  # X/Y's own ramp before this move — restored after
            "C1 DCL": "10",
            "C1 SPD": "8",  # X/Y's own speed before this move — restored after
            "C2 SPD 10": "10",
            "C2 BMT 2 50": "0",
            "C2 MIF": "1",
            "C1 ACL 2": "2",
            "C1 DCL 1.6": "1.6",
            "C1 SPD 1": "1",
            "C1 BMT 5 0": "0",
            "C1 MIF": "1",
            "C1 ACL 12": "12",  # restore
            "C1 DCL 10": "10",
            "C1 SPD 8": "8",  # restore
            "A1 ACP": "5",  # post-move resync of X/Y/Z/Theta from hardware
            "A2 ACP": "0",
            "A5 ACP": "2",
            "A6 ACP": "50",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X5 Z2 A50 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C2 INI 5 6", "C2 ACL", "C2 DCL",  # read Z/Theta's ramp (scaling reference)
            "C1 INI 1 2", "C1 ACL", "C1 DCL", "C1 SPD",  # read X/Y's own ramp/speed (to restore later)
            "C2 SPD 10", "C2 BMT 2 50",  # Z/Theta, unscaled — Theta dominates, so it's the "long" leg
            "C1 ACL 2", "C1 DCL 1.6", "C1 SPD 1", "C1 BMT 5 0",  # X/Y, scaled by k=0.1
            "C2 MIF", "C1 MIF",
            "C1 ACL 12", "C1 DCL 10", "C1 SPD 8",  # X/Y's ramp/speed restored
            "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP",  # resync _current_pos/_current_theta from hardware
        ]

    def test_ramp_is_restored_even_if_a_leg_times_out(self, monkeypatch):
        """If either leg's poll times out (poll_until_move_finished
        returns False, the move is aborted, SnapMotionError raised), the
        scaled leg's ramp must still be restored — the whole point of
        wrapping the fire+poll sequence in try/finally. Forces the
        "not finished" outcome directly rather than waiting out a real
        30s timeout.
        """
        monkeypatch.setattr(
            "laguna.robot.macron.gcode.poll_until_move_finished", lambda *a, **kw: False
        )
        responses = {
            "C1 INI 1 2": "0",
            "C1 ACL": "50",
            "C1 DCL": "40",
            "A5 ACL": "25",  # Z's own ramp before this move
            "A5 DCL": "20",
            "A5 SPD": "8",  # Z's own speed before this move
            "A5 ACL 15": "15",
            "A5 DCL 12": "12",
            "A5 SPD 3": "3",
            "A5 BMT 3": "0",
            "A5 ABT": "0",  # abort(), issued by _poll_axis_move_finished on timeout
            "C1 SPD 10": "10",
            "C1 BMT 10 0": "0",
            "A5 ACL 25": "25",  # restore, even though the move timed out
            "A5 DCL 20": "20",
            "A5 SPD 8": "8",  # restore, even though the move timed out
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X10 Z3 F600")
        with pytest.raises(SnapMotionError):
            executor.execute(trajectory)
        assert "A5 ABT" in conn.sent
        assert conn.sent[-3:] == ["A5 ACL 25", "A5 DCL 20", "A5 SPD 8"]

    def test_z_and_theta_without_xy_uses_the_theta_group_alone(self):
        """A move touching only Z and Theta (no X/Y) needs no concurrency —
        it's a single leg via the Z/Theta group, same shape as an XY-only
        move today.
        """
        responses = {
            "C2 INI 5 6": "0",
            "C2 SPD 10": "10",
            "C2 BMT 3 90": "0",
            "C2 MIF": "1",
            "A5 ACP": "3",  # post-move resync of Z/Theta from hardware
            "A6 ACP": "90",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 Z3 A90 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C2 INI 5 6", "C2 SPD 10", "C2 BMT 3 90", "C2 MIF", "A5 ACP", "A6 ACP",
        ]

    def test_theta_only_move_uses_a_single_axis_command(self):
        """A move touching only Theta (no X/Y/Z) never touches either
        coordinated group.
        """
        responses = {"A6 SPD 10": "10", "A6 BMT 90": "0", "A6 MIF": "1", "A6 ACP": "90"}
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 A90 F600")
        executor.execute(trajectory)
        assert conn.sent == ["A6 SPD 10", "A6 BMT 90", "A6 MIF", "A6 ACP"]

    def test_z_theta_group_requires_theta_cmd(self):
        """A move needing the Z/Theta group without a configured theta_cmd
        raises clearly instead of silently doing the wrong thing.
        """
        executor, conn = _make_executor({}, with_theta_group=False)
        trajectory = executor.plan("G1 Z3 A90")
        with pytest.raises(GCodeError, match="theta_cmd"):
            executor.execute(trajectory)

    def test_dual_elbow_fence_check_catches_either_ordering(self):
        """Since the XY leg and the Z leg now run concurrently rather than
        as a fixed sequence, the true swept path is the bounding box
        between start and end, not one line. plan() checks both possible
        elbow orderings (Z-first and XY-first) — a fence only the
        Z-first elbow enters must still be caught.
        """
        from laguna.robot.macron.fences import BoxFence
        # Straight diagonal (0,0,0)->(10,10,5) misses this box. The
        # Z-first elbow (0,0,0)->(0,0,5)->(10,10,5) crosses it on its
        # second leg; the XY-first elbow (0,0,0)->(10,10,0)->(10,10,5)
        # does not.
        executor, conn = _make_executor(
            {}, fences=[BoxFence("post", 4, 6, 4, 6, 4, 6)]
        )
        with pytest.raises(FenceViolation):
            executor.plan("G1 X10 Y10 Z5 F600")
        assert conn.sent == []

    def test_dual_elbow_fence_check_catches_the_other_ordering_too(self):
        """Symmetric to the above: a fence only the XY-first elbow enters
        (and the Z-first elbow and the diagonal both miss) must also be
        caught — proving both orderings are actually checked, not just one.
        """
        from laguna.robot.macron.fences import BoxFence
        # XY-first elbow (0,0,0)->(10,10,0)->(10,10,5) crosses this box on
        # its first leg (z=0, within [0,2]); Z-first elbow
        # (0,0,0)->(0,0,5)->(10,10,5) never enters x=[8,10]/y=[8,10] at
        # z=[0,2] (its first leg stays at x=y=0, its second leg is at z=5).
        executor, conn = _make_executor(
            {}, fences=[BoxFence("post", 8, 10, 8, 10, 0, 2)]
        )
        with pytest.raises(FenceViolation):
            executor.plan("G1 X10 Y10 Z5 F600")
        assert conn.sent == []

    def test_moves_x_and_theta_concurrently_without_z(self):
        """X/Y paired with Theta alone (no Z change) is also a concurrent
        pair — the "caller picks" case where Theta joins an XY move without
        needing the Z/Theta group at all. Theta travels further than X/Y
        here (20 vs 5), so this covers the opposite branch from the Z
        tests above: Theta is the "long" leg (left at the nominal feed
        rate, ramp untouched) and X/Y is the "short" one — scaled by
        k=5/20=0.25 from Theta's currently-configured ramp (read via
        single-axis A6 ACL/DCL queries, not a group — Theta alone never
        touches C2). X/Y's own prior ramp *and speed* are read too and
        restored once both legs finish.
        """
        responses = {
            "A6 ACL": "20",
            "A6 DCL": "16",
            "C1 INI 1 2": "0",
            "C1 ACL": "12",  # X/Y's own ramp before this move — restored after
            "C1 DCL": "10",
            "C1 SPD": "8",  # X/Y's own speed before this move — restored after
            "A6 SPD 10": "10",
            "A6 BMT 20": "0",
            "A6 MIF": "1",
            "C1 ACL 5": "5",
            "C1 DCL 4": "4",
            "C1 SPD 2.5": "2.5",
            "C1 BMT 5 0": "0",
            "C1 MIF": "1",
            "C1 ACL 12": "12",  # restore
            "C1 DCL 10": "10",
            "C1 SPD 8": "8",  # restore
            "A1 ACP": "5",  # post-move resync of X/Y/Theta from hardware
            "A2 ACP": "0",
            "A6 ACP": "20",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X5 A20 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "A6 ACL", "A6 DCL",  # read Theta's ramp (scaling reference)
            "C1 INI 1 2", "C1 ACL", "C1 DCL", "C1 SPD",  # read X/Y's own ramp/speed (to restore later)
            "A6 SPD 10", "A6 BMT 20",  # Theta, unscaled
            "C1 ACL 5", "C1 DCL 4", "C1 SPD 2.5", "C1 BMT 5 0",  # X/Y, scaled by k=0.25
            "A6 MIF", "C1 MIF",
            "C1 ACL 12", "C1 DCL 10", "C1 SPD 8",  # X/Y's ramp/speed restored
            "A1 ACP", "A2 ACP", "A6 ACP",  # resync _current_pos/_current_theta from hardware
        ]

    def test_moves_x_and_theta_concurrently_with_theta_as_the_short_leg(self):
        """Opposite branch from the test above: X/Y travels further than
        Theta (20 vs 5), so X/Y is "long" (nominal feed rate, ramp
        untouched, X/Y's ramp read via C1 ACL/DCL) and Theta is scaled
        down by k=5/20=0.25 — the single-axis-responder mirror of the Z
        "short leg" case. Theta's own prior ramp *and speed* are restored
        once both legs finish.
        """
        responses = {
            "C1 INI 1 2": "0",
            "C1 ACL": "20",
            "C1 DCL": "16",
            "A6 ACL": "25",  # Theta's own ramp before this move — restored after
            "A6 DCL": "20",
            "A6 SPD": "8",  # Theta's own speed before this move — restored after
            "A6 ACL 5": "5",
            "A6 DCL 4": "4",
            "A6 SPD 2.5": "2.5",
            "A6 BMT 5": "0",
            "A6 MIF": "1",
            "C1 SPD 10": "10",
            "C1 BMT 20 0": "0",
            "C1 MIF": "1",
            "A6 ACL 25": "25",  # restore
            "A6 DCL 20": "20",
            "A6 SPD 8": "8",  # restore
            "A1 ACP": "20",  # post-move resync of X/Y/Theta from hardware
            "A2 ACP": "0",
            "A6 ACP": "5",
        }
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 X20 A5 F600")
        executor.execute(trajectory)
        assert conn.sent == [
            "C1 INI 1 2", "C1 ACL", "C1 DCL",  # read X/Y's ramp (scaling reference)
            "A6 ACL", "A6 DCL", "A6 SPD",  # read Theta's own ramp/speed (to restore later)
            "A6 ACL 5", "A6 DCL 4", "A6 SPD 2.5", "A6 BMT 5",  # Theta, scaled by k=0.25
            "C1 SPD 10", "C1 BMT 20 0",  # X/Y, unscaled
            "A6 MIF", "C1 MIF",
            "A6 ACL 25", "A6 DCL 20", "A6 SPD 8",  # Theta's ramp/speed restored
            "A1 ACP", "A2 ACP", "A6 ACP",  # resync _current_pos/_current_theta from hardware
        ]

    def test_z_only_move_uses_a_single_axis_command_no_ramp_scaling(self):
        """A move touching only Z (no X/Y, no Theta) is a single leg, same
        as an XY-only or Theta-only move — no concurrency, no ramp
        decomposition, ACL/DCL never touched.
        """
        responses = {"A5 SPD 10": "10", "A5 BMT 3": "0", "A5 MIF": "1", "A5 ACP": "3"}
        executor, conn = _make_executor(responses)
        trajectory = executor.plan("G1 Z3 F600")
        executor.execute(trajectory)
        assert conn.sent == ["A5 SPD 10", "A5 BMT 3", "A5 MIF", "A5 ACP"]

    def test_dry_run_sends_nothing(self):
        executor, conn = _make_executor({}, dry_run=True)
        trajectory = executor.plan("G1 X10 Y10 F600")
        executor.execute(trajectory)
        assert conn.sent == []

    def test_dry_run_describes_every_leg_kind(self, caplog):
        """Dry-run logging covers each responder kind (z, theta, z_theta)
        combined with an XY leg, without sending anything. Each line uses
        a fresh executor so every case starts from (0,0,0)/theta=0 and
        genuinely touches the axes it claims to.
        """
        for line in ("G1 X10 Z3 F600", "G1 X10 A90 F600", "G1 X10 Z3 A90 F600"):
            executor, conn = _make_executor({}, dry_run=True)
            trajectory = executor.plan(line)
            executor.execute(trajectory)
            assert conn.sent == []

    def test_confirm_cb_can_abort_before_sending(self):
        executor, conn = _make_executor({"C1 INI 1 2": "0"}, confirm_cb=lambda move: False)
        trajectory = executor.plan("G1 X10")
        with pytest.raises(GCodeExecutionAborted):
            executor.execute(trajectory)
        assert "C1 BMT 10 0" not in conn.sent

    def test_confirm_cb_receives_the_move(self):
        seen = []
        executor, conn = _make_executor(
            {
                "C1 INI 1 2": "0",
                "C1 BMT 10 0": "0",
                "C1 SPD": "20",  # no F word — read X/Y's own speed to predict duration
                "C1 MIF": "1",
                "A1 ACP": "10",
                "A2 ACP": "0",
            },
            confirm_cb=lambda move: seen.append(move) or True,
        )
        trajectory = executor.plan("G1 X10")
        executor.execute(trajectory)
        assert len(seen) == 1
        assert seen[0].target == (10.0, 0.0, 0.0)


class TestExecutorHomeDwellPause:
    def test_g28_raises_while_homing_is_disabled(self):
        """HomingProcedure.home_all() raises unconditionally while physical
        obstructions block several of the limit switches it depends on
        (plan step 2, from 5c170d3). G28 therefore cannot run, and must
        fail before touching the wire rather than jogging into a blocked
        switch.
        """
        executor, conn = _make_executor({})
        trajectory = executor.plan("G28")
        with pytest.raises(NotImplementedError, match="Homing is temporarily disabled"):
            executor.execute(trajectory)
        assert conn.sent == []  # nothing reached the controller

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
