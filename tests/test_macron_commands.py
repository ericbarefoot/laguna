"""Tests for MMCCommands: exact ASCII token formatting and IOMap-gated helpers.

All offline — driven entirely against FakeSnapConnection, asserting the
exact command strings sent match the vendor-verified grammar (A<n>/C<n>
prefixes, no brackets).
"""

import pytest

from laguna.robot.macron import commands as macron_commands
from laguna.robot.macron.commands import (
    ALL_AXES,
    IOMap,
    MMCCommands,
    THETA_AXIS,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    poll_until_move_finished,
    predicted_move_s,
)
from laguna.robot.macron.connection import SnapMotionError
from tests.macron_fixtures import FakeSnapConnection


class TestTokenFormat:
    def test_single_axis_read(self):
        conn = FakeSnapConnection({"A1 ACP": "12.000"})
        cmd = MMCCommands(conn)
        assert cmd.get_actual_position(X_AXIS) == 12.0
        assert conn.sent == ["A1 ACP"]

    def test_single_axis_write(self):
        conn = FakeSnapConnection({"A2 SPD 5000": "5000"})
        cmd = MMCCommands(conn)
        cmd.set_speed(Y_AXIS, 5000)
        assert conn.sent == ["A2 SPD 5000"]

    def test_group_init(self):
        conn = FakeSnapConnection({"C1 INI 1 2 3": "0"})
        cmd = MMCCommands(conn)
        cmd.init_group(1, 2, 3)
        assert conn.sent == ["C1 INI 1 2 3"]

    def test_group_init_rejects_more_than_six_axes(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        with pytest.raises(ValueError):
            cmd.init_group(1, 2, 3, 4, 5, 6, 7)
        assert conn.sent == []  # never sent — rejected before touching the wire

    def test_global_command_no_prefix(self):
        conn = FakeSnapConnection({"INB 3": "1"})
        cmd = MMCCommands(conn)
        assert cmd.read_input_bit(3) is True
        assert conn.sent == ["INB 3"]

    def test_set_output_bit(self):
        conn = FakeSnapConnection({"SOB 4 1": "0"})
        cmd = MMCCommands(conn)
        cmd.set_output_bit(4, True)
        assert conn.sent == ["SOB 4 1"]

    def test_group_move_formats_multiple_params(self):
        conn = FakeSnapConnection({"C1 BMT 10 20 30": "0"})
        cmd = MMCCommands(conn)
        cmd.group_begin_move_to(10, 20, 30)
        assert conn.sent == ["C1 BMT 10 20 30"]

    def test_z_axis_index_five(self):
        conn = FakeSnapConnection({"A5 ACP": "0"})
        cmd = MMCCommands(conn)
        cmd.get_actual_position(Z_AXIS)
        assert conn.sent == ["A5 ACP"]

    def test_theta_axis_index_six(self):
        conn = FakeSnapConnection({"A6 ACP": "0"})
        cmd = MMCCommands(conn)
        cmd.get_actual_position(THETA_AXIS)
        assert conn.sent == ["A6 ACP"]

    def test_all_axes_is_the_four_commandable_motion_axes(self):
        # X/Y (commander) + Z/Theta (responder) — the only exposed/commandable
        # motion axes; slots 3/4/7/8 are internal encoder-only slots on their
        # respective nodes and are not modeled as Axis objects at all.
        assert ALL_AXES == (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)
        assert len(ALL_AXES) == 4


class TestValidateSoftLimits:
    def test_all_limits_within_threshold_passes(self):
        conn = FakeSnapConnection(
            {
                "A1 NLT": "-1000", "A1 PLT": "1000",
                "A2 NLT": "-500", "A2 PLT": "500",
                "A5 NLT": "-100", "A5 PLT": "100",
                "A6 NLT": "-50", "A6 PLT": "50",
            }
        )
        cmd = MMCCommands(conn)
        result = cmd.validate_soft_limits()
        assert result["X"] == (-1000.0, 1000.0)
        assert result["Theta"] == (-50.0, 50.0)

    def test_garbage_limits_raise_and_name_axes(self):
        # Reproduces the real observed state on this hardware: X and Y have
        # uninitialized (~±8.2e8) limits, Z and Theta are fine.
        conn = FakeSnapConnection(
            {
                "A1 NLT": "-822536056", "A1 PLT": "822536056",
                "A2 NLT": "-822536056", "A2 PLT": "822536056",
                "A5 NLT": "-500000", "A5 PLT": "500000",
                "A6 NLT": "-500000", "A6 PLT": "500000",
            }
        )
        cmd = MMCCommands(conn)
        with pytest.raises(SnapMotionError) as exc_info:
            cmd.validate_soft_limits()
        assert "X" in str(exc_info.value)
        assert "Y" in str(exc_info.value)
        assert "Z" not in str(exc_info.value)


class TestIOMapGatedBrakeHelpers:
    def test_disengage_brake_sends_correct_command_using_default_channel(self):
        # y_brake_output=4 (SOB 4) is confirmed (eab-2026-07-16/17.dsm) and
        # is IOMap's default — should work with zero configuration.
        conn = FakeSnapConnection({"SOB 4 1": "0"})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        cmd.disengage_brake(Y_AXIS, io_map)
        assert conn.sent == ["SOB 4 1"]

    def test_disengage_brake_raises_when_channel_explicitly_unset(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap(y_brake_output=None)
        with pytest.raises(ValueError):
            cmd.disengage_brake(Y_AXIS, io_map)
        assert conn.sent == []  # never reached the wire

    def test_brake_is_disengaged_uses_confirmed_y_status_channel(self):
        # y_brake_status_input=8 (INB 8) is confirmed by eab-2026-07-16/17.dsm
        # and is IOMap's default — should work with zero configuration.
        conn = FakeSnapConnection({"INB 8": "1"})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        assert cmd.brake_is_disengaged(Y_AXIS, io_map) is True

    def test_brake_is_disengaged_raises_not_implemented_for_z(self):
        # z_brake_status_input lives on the responder's own input bank
        # (TNamedIO ModuleNumber=1) and has no ASCII addressing path from
        # here — this is an architectural gap, not a "not yet probed" one,
        # so it must raise NotImplementedError rather than ValueError.
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap()  # z_brake_status_input defaults to None
        with pytest.raises(NotImplementedError):
            cmd.brake_is_disengaged(Z_AXIS, io_map)
        assert conn.sent == []  # never reached the wire

    def test_non_brake_axis_always_reports_free(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        assert cmd.brake_is_disengaged(X_AXIS, io_map) is True
        assert conn.sent == []


class TestPredictedMoveS:
    """predicted_move_s feeds the sparse move-completion polling — see
    commands.py's "Move-completion polling" note."""

    def test_distance_over_speed(self):
        assert predicted_move_s(20.0, 10.0) == 2.0

    def test_negative_distance_is_treated_as_magnitude(self):
        # Callers pass raw target-minus-current deltas, which are signed.
        assert predicted_move_s(-20.0, 10.0) == 2.0

    @pytest.mark.parametrize("speed", [None, 0.0, -5.0])
    def test_unknown_or_nonpositive_speed_predicts_nothing(self, speed):
        # 0.0 means "poll immediately (but still sparsely)" rather than
        # sleeping on a bogus prediction.
        assert predicted_move_s(20.0, speed) == 0.0


class TestPollUntilMoveFinished:
    """The actual fix for the group-motion stall: query the controller as
    little as possible while a move is in flight. Confirmed on hardware
    that tight polling during group interpolation makes this controller
    stop answering the wire entirely — see commands.py's module note."""

    @pytest.fixture
    def sparse(self, monkeypatch):
        """Restore realistic pacing (the suite-wide autouse fixture in
        conftest.py zeroes it), with a fake clock that advances only when
        the code sleeps — so timeouts are genuinely exercised without the
        suite waiting in real time.
        """
        slept = []
        now = {"t": 0.0}

        def fake_sleep(seconds):
            slept.append(seconds)
            now["t"] += seconds

        monkeypatch.setattr(macron_commands, "SPARSE_POLL_INTERVAL_S", 0.5)
        monkeypatch.setattr(macron_commands, "PREDICTED_SLEEP_FRACTION", 0.85)
        monkeypatch.setattr(macron_commands.time, "sleep", fake_sleep)
        monkeypatch.setattr(macron_commands.time, "monotonic", lambda: now["t"])
        return slept

    def test_returns_true_when_finished(self):
        assert poll_until_move_finished(lambda: True) is True

    def test_returns_false_on_timeout(self):
        assert poll_until_move_finished(lambda: False, timeout_s=0.0) is False

    def test_sleeps_through_most_of_the_predicted_duration_first(self, sparse):
        poll_until_move_finished(lambda: True, predicted_s=2.0, timeout_s=30.0)
        # 0.85 * 2.0s slept up front, before a single query went out.
        assert sparse == [pytest.approx(1.7)]

    def test_a_long_move_costs_only_a_couple_of_queries(self, sparse):
        """The regression this guards: the old tight loop issued ~40 MIF
        queries for a 2s move; this must issue a small handful."""
        calls = {"n": 0}

        def is_finished():
            calls["n"] += 1
            return calls["n"] >= 3  # finishes on the 3rd query

        assert poll_until_move_finished(is_finished, predicted_s=2.0, timeout_s=30.0) is True
        assert calls["n"] == 3
        # One predicted-duration sleep, then one sparse interval per retry.
        assert sparse == [pytest.approx(1.7), 0.5, 0.5]

    def test_unknown_duration_polls_immediately_but_still_sparsely(self, sparse):
        calls = {"n": 0}

        def is_finished():
            calls["n"] += 1
            return calls["n"] >= 2

        poll_until_move_finished(is_finished, predicted_s=0.0, timeout_s=30.0)
        assert sparse == [0.5]  # no up-front sleep, still the sparse interval between queries

    def test_predicted_sleep_never_overshoots_the_timeout(self, sparse):
        # A wildly optimistic prediction must not sleep past the deadline
        # and turn a timeout into an unbounded wait: the up-front sleep is
        # clamped to timeout_s (2.0, not 0.85 * 1000).
        assert poll_until_move_finished(lambda: False, predicted_s=1000.0, timeout_s=2.0) is False
        assert sparse[0] == pytest.approx(2.0)
        # Only the one sparse-interval retry after that — the deadline is
        # checked after a query, so an unfinished move gets a last look at
        # the deadline rather than being abandoned a poll early.
        assert sparse == [pytest.approx(2.0), 0.5]
