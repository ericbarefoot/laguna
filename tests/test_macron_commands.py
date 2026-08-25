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


class TestIOMapGatedHomeAndLimitSwitches:
    def test_read_home_switch_uses_confirmed_default_channels(self):
        conn = FakeSnapConnection({"INB 1": "1", "INB 3": "0", "INB 5": "1"})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        assert cmd.read_home_switch(X_AXIS, io_map) is True
        assert cmd.read_home_switch(Y_AXIS, io_map) is False
        assert cmd.read_home_switch(Z_AXIS, io_map) is True
        assert conn.sent == ["INB 1", "INB 3", "INB 5"]

    def test_read_home_switch_raises_when_channel_unset(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap(x_home_input=None)
        with pytest.raises(ValueError):
            cmd.read_home_switch(X_AXIS, io_map)
        assert conn.sent == []

    def test_read_home_switch_rejects_theta(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        with pytest.raises(ValueError):
            cmd.read_home_switch(THETA_AXIS, IOMap())
        assert conn.sent == []

    def test_read_limit_switch_uses_confirmed_default_channels(self):
        conn = FakeSnapConnection({"INB 2": "1", "INB 4": "0", "INB 6": "1"})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        assert cmd.read_limit_switch(X_AXIS, io_map) is True
        assert cmd.read_limit_switch(Y_AXIS, io_map) is False
        assert cmd.read_limit_switch(Z_AXIS, io_map) is True
        assert conn.sent == ["INB 2", "INB 4", "INB 6"]

    def test_read_limit_switch_raises_when_channel_unset(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap(y_limit_input=None)
        with pytest.raises(ValueError):
            cmd.read_limit_switch(Y_AXIS, io_map)
        assert conn.sent == []

    def test_read_limit_switch_raises_not_implemented_for_theta(self):
        # theta_limit_input lives on the responder's own input bank
        # (TNamedIO ModuleNumber=1) and has no ASCII addressing path from
        # here — architectural gap, not "not yet probed", so
        # NotImplementedError rather than ValueError.
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap()  # theta_limit_input defaults to None
        with pytest.raises(NotImplementedError):
            cmd.read_limit_switch(THETA_AXIS, io_map)
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

    def test_default_timeout_uses_the_floor_for_a_short_predicted_move(self, sparse):
        # predicted_s=2.0 * TIMEOUT_MARGIN(1.2) = 2.4, below MIN_TIMEOUT_S
        # (10.0) — a short move that actually gets stuck must not wait a
        # flat 30s to be caught; the floor still gives it a reasonable
        # amount of slack (predicted_s excludes accel/decel ramp time), but
        # not an excessive one.
        assert poll_until_move_finished(lambda: False, predicted_s=2.0) is False
        # up-front sleep is 0.85*2.0=1.7, then sparse retries until the
        # 10.0s floor elapses: 1.7 + 8*0.5 = 5.7... the exact retry count
        # only matters insofar as the deadline is respected — check the
        # total elapsed instead of an exact sleep sequence.
        assert sum(sparse) == pytest.approx(10.0, abs=0.5)

    def test_default_timeout_scales_up_for_a_long_predicted_move(self, sparse):
        # predicted_s=20.0 * TIMEOUT_MARGIN(1.2) = 24.0, above the 10.0
        # floor — a legitimately long move gets proportionally more room
        # instead of racing a fixed 30s (or, worse, being cut off before
        # its own predicted duration if the floor were lower than this).
        assert poll_until_move_finished(lambda: False, predicted_s=20.0) is False
        assert sum(sparse) == pytest.approx(24.0, abs=1.0)

    def test_explicit_timeout_still_overrides_the_default(self, sparse):
        # Passing timeout_s explicitly (even 0.0) must bypass the
        # predicted-duration-scaled default entirely — existing callers
        # that want a fixed timeout must not be silently rescaled.
        assert poll_until_move_finished(lambda: False, predicted_s=20.0, timeout_s=3.0) is False
        assert sum(sparse) == pytest.approx(3.0, abs=1.0)

    def test_resolve_timeout_s_matches_poll_until_move_finished(self):
        from laguna.robot.macron.commands import resolve_timeout_s

        assert resolve_timeout_s(predicted_s=2.0, timeout_s=None) == 10.0  # floor wins
        assert resolve_timeout_s(predicted_s=20.0, timeout_s=None) == pytest.approx(24.0)
        assert resolve_timeout_s(predicted_s=20.0, timeout_s=5.0) == 5.0  # explicit wins
        assert resolve_timeout_s(predicted_s=0.0, timeout_s=0.0) == 0.0  # explicit 0.0 is not None


class TestEnaBanned:
    """ENA is refused at every layer. Addressing it on a responder-node axis
    (A5/Z, A6/Theta) crashes this controller: the node stops answering
    entirely (error 70 on everything, including reads that worked moments
    earlier) and must be reflashed. Confirmed on hardware 2026-08-02,
    reproduced from a bare tio terminal with no laguna code involved, using
    a bare ENA *read* with no argument."""

    def test_set_enable_is_banned(self):
        cmd = MMCCommands(FakeSnapConnection({}))
        with pytest.raises(RuntimeError, match="banned"):
            cmd.set_enable(Z_AXIS, True)

    def test_get_enable_is_banned(self):
        cmd = MMCCommands(FakeSnapConnection({}))
        with pytest.raises(RuntimeError, match="banned"):
            cmd.get_enable(Z_AXIS)

    def test_banned_for_commander_axes_too(self):
        """Evidence only covers the responder, but nothing needs ENA at all
        and the blast radius warrants a blanket refusal."""
        cmd = MMCCommands(FakeSnapConnection({}))
        with pytest.raises(RuntimeError, match="banned"):
            cmd.set_enable(X_AXIS, True)

    def test_nothing_reaches_the_wire(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        for call in (lambda: cmd.set_enable(Z_AXIS, True), lambda: cmd.get_enable(Z_AXIS)):
            with pytest.raises(RuntimeError):
                call()
        assert conn.sent == []

    def test_read_axis_state_does_not_query_ena(self):
        """read_axis_state() used to read ENA — that batch would have
        crashed the responder every time it was called on Z or Theta."""
        conn = FakeSnapConnection({
            "A5 ACP": "0", "A5 COP": "0", "A5 DEP": "0", "A5 ENP": "0",
            "A5 SPD": "0", "A5 ACL": "0", "A5 DCL": "0", "A5 MTR": "1",
            "A5 MIF": "1", "A5 CAB": "0", "A5 CAP": "0", "A5 CAT": "0",
            "A5 NLT": "0", "A5 PLT": "0",
        })
        MMCCommands(conn).read_axis_state(Z_AXIS)
        assert not any("ENA" in c for c in conn.sent)


class TestReadAxisStateResilience:
    """read_axis_state() queries each register independently so one failing
    query does not blow away every other field that already succeeded.
    Ported from e5f8a95 (plan step 3) — this is what made it possible to
    see, live, that a stalled Y axis's stepper-side bookkeeping (ACP/COP/
    DEP/MTR/MIF) was fine while its encoder/capture registers had gone
    unreachable."""

    ALL_OK = {
        "A1 ACP": "1", "A1 COP": "2", "A1 DEP": "3", "A1 ENP": "4",
        "A1 SPD": "5", "A1 ACL": "6", "A1 DCL": "7", "A1 MTR": "1",
        "A1 MIF": "1", "A1 CAB": "0", "A1 CAP": "8", "A1 CAT": "0",
        "A1 NLT": "-9", "A1 PLT": "9",
    }

    def test_happy_path_populates_every_field_and_no_errors(self):
        state = MMCCommands(FakeSnapConnection(self.ALL_OK)).read_axis_state(X_AXIS)
        assert state.actual_position == 1.0
        assert state.positive_limit == 9.0
        assert state.errors == {}

    def test_one_failing_query_does_not_lose_the_others(self):
        responses = dict(self.ALL_OK)
        responses["A1 ENP"] = SnapMotionError(70)      # encoder unreachable
        state = MMCCommands(FakeSnapConnection(responses)).read_axis_state(X_AXIS)
        assert "encoder_position" in state.errors      # recorded...
        assert state.encoder_position == 0.0           # ...and left at its default
        assert state.actual_position == 1.0            # everything else survived
        assert state.positive_limit == 9.0

    def test_several_failures_are_all_recorded_by_field_name(self):
        responses = dict(self.ALL_OK)
        for cmd in ("A1 ENP", "A1 CAB", "A1 CAP", "A1 CAT"):
            responses[cmd] = SnapMotionError(70)
        state = MMCCommands(FakeSnapConnection(responses)).read_axis_state(X_AXIS)
        assert set(state.errors) == {
            "encoder_position", "capture_bit", "capture_position", "capture_has_tripped",
        }
        assert state.actual_position == 1.0            # stepper side still readable

    def test_still_never_queries_ena(self):
        conn = FakeSnapConnection(self.ALL_OK)
        MMCCommands(conn).read_axis_state(X_AXIS)
        assert not any("ENA" in c for c in conn.sent)
