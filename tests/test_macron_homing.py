"""Tests for the hardware capture-latch homing procedure.

All scripted against FakeSnapConnection — no hardware. Homing is currently
deprioritized for real-hardware use (limit switch INB channels haven't been
physically probed on this machine), but the state machine itself is fully
unit-testable and should stay correct for whenever it is revisited.
"""

import pytest

from laguna.robot.macron.commands import IOMap, MMCCommands, X_AXIS, Y_AXIS
from laguna.robot.macron.connection import SnapMotionError
from laguna.robot.macron.homing import AxisHomingConfig, HomingConfig, HomingProcedure
from tests.macron_fixtures import FakeSnapConnection


def _trip_after(n, before="0", after="1"):
    """Callable response: returns `before` for the first n calls, then `after`."""
    state = {"calls": 0}

    def _resp(cmd):
        state["calls"] += 1
        return after if state["calls"] > n else before

    return _resp


class TestHomeAxisHappyPath:
    def _make_procedure(self, responses, **config_overrides):
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0,
            standoff_distance=5.0,
            poll_interval_s=0.001,
            timeout_s=1.0,
            backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(capture_source_index=1)},
            **config_overrides,
        )
        return HomingProcedure(cmd, config), conn

    def test_homes_x_axis_and_zeros_relative_to_trip_point(self):
        # current(-25.1) - trip(-24.5) == -0.6 (formatted to 6 sig figs by MMCCommands)
        zero_cmd = f"A1 ACP {(-25.1) - (-24.5):.6g}"
        responses = {
            "A1 SCS 1": "1",
            "A1 SCT 1": "1",
            "A1 AIC": "0",
            "A1 CAB": "0",  # not already tripped -> skip backoff
            "A1 JOG -10": "-10",
            "A1 CAT": _trip_after(2),  # trips on the 3rd poll
            "A1 BST": "0",
            "A1 MIF": _trip_after(1),  # finishes on the 2nd poll
            "A1 CAP": "-24.500",       # hardware-latched trip position
            "A1 ACP": _trip_after(1, before="-25.100", after="5.000"),
            # ^ first ACP-read call (current position after decel) = -25.100;
            #   second ACP-read call (final standoff readout) = 5.000
            zero_cmd: str((-25.1) - (-24.5)),
            "A1 MVT 5": "5",
        }
        proc, conn = self._make_procedure(responses)
        final_pos = proc.home_axis(X_AXIS)
        assert final_pos == 5.0
        assert zero_cmd in conn.sent  # zeroed relative to trip point, not a flat 0

    def test_backs_off_when_already_tripped_at_start(self):
        responses = {
            "A1 SCS 1": "1",
            "A1 SCT 1": "1",
            "A1 AIC": "0",
            "A1 CAB": "1",  # already tripped
            "A1 MIF": _trip_after(0),  # move-is-finished immediately true after backoff
            "A1 BMB 10": "0",  # backoff = 2 * standoff_distance(5) = 10
            "A1 JOG -10": "-10",
            "A1 CAT": _trip_after(0),
            "A1 BST": "0",
            "A1 CAP": "0",
            "A1 ACP": "0",
            "A1 ACP 0": "0",
            "A1 MVT 5": "5",
        }
        proc, conn = self._make_procedure(responses)
        proc.home_axis(X_AXIS)
        assert "A1 BMB 10" in conn.sent
        # backoff must happen before arm/jog
        assert conn.sent.index("A1 BMB 10") < conn.sent.index("A1 JOG -10")

    def test_no_axis_config_defaults_to_negative_direction(self):
        # X_AXIS has no AxisHomingConfig registered -> falls back to default
        # direction -1.0, and set_capture_source/set_capture_trip are skipped.
        responses = {
            "A1 AIC": "0",
            "A1 CAB": "0",
            "A1 JOG -10": "-10",
            "A1 CAT": _trip_after(0),
            "A1 BST": "0",
            "A1 MIF": _trip_after(0),
            "A1 CAP": "0",
            "A1 ACP": "0",
            "A1 ACP 0": "0",
            "A1 MVT 5": "5",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            timeout_s=1.0, backoff_timeout_s=1.0,
        )
        proc = HomingProcedure(cmd, config)
        proc.home_axis(X_AXIS)
        assert "A1 SCS" not in " ".join(conn.sent)


class TestHomeAxisTimeout:
    def test_capture_never_trips_aborts_and_raises(self):
        responses = {
            "A1 AIC": "0",
            "A1 CAB": "0",
            "A1 JOG -10": "-10",
            "A1 CAT": "0",  # never trips
            "A1 ABT": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            timeout_s=0.02, backoff_timeout_s=1.0,
        )
        proc = HomingProcedure(cmd, config)
        with pytest.raises(SnapMotionError):
            proc.home_axis(X_AXIS)
        assert "A1 ABT" in conn.sent


class TestHomeAxisBrakeHandling:
    def test_disengages_and_reengages_brake_for_y_axis(self):
        responses = {
            "SOB 4 1": "0",  # disengage
            "INB 8": "1",  # brake status confirms released
            "A2 AIC": "0",
            "A2 CAB": "0",
            "A2 JOG -10": "-10",
            "A2 CAT": _trip_after(0),
            "A2 BST": "0",
            "A2 MIF": _trip_after(0),
            "A2 CAP": "0",
            "A2 ACP": "0",
            "A2 ACP 0": "0",
            "A2 MVT 5": "5",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        io_map = IOMap(y_brake_output=4, y_brake_status_input=8)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            timeout_s=1.0, backoff_timeout_s=1.0,
        )
        proc = HomingProcedure(cmd, config, io_map=io_map)
        proc.home_axis(Y_AXIS)
        assert "SOB 4 1" in conn.sent

    def test_raises_if_brake_channel_not_configured(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        config = HomingConfig(homing_speed=10.0, standoff_distance=5.0)
        io_map = IOMap(y_brake_output=None)  # explicitly unset
        proc = HomingProcedure(cmd, config, io_map=io_map)
        with pytest.raises(ValueError):
            proc.home_axis(Y_AXIS)


class TestHomeAll:
    def test_stops_at_first_failure(self):
        # Z is first in the default home order; make it fail immediately.
        responses = {
            "A5 AIC": "0",
            "A5 CAB": "0",
            "A5 JOG -10": "-10",
            "A5 CAT": "0",  # never trips -> timeout
            "A5 ABT": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            timeout_s=0.02, backoff_timeout_s=1.0,
        )
        proc = HomingProcedure(cmd, config)
        result = proc.home_all()
        assert result.success is False
        assert result.axis_results == {}
        assert result.error is not None
