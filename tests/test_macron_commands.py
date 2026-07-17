"""Tests for MMCCommands: exact ASCII token formatting and IOMap-gated helpers.

All offline — driven entirely against FakeSnapConnection, asserting the
exact command strings sent match the vendor-verified grammar (A<n>/C<n>
prefixes, no brackets).
"""

import pytest

from laguna.robot.macron.commands import (
    ALL_AXES,
    IOMap,
    MMCCommands,
    THETA_AXIS,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
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
    def test_disengage_brake_raises_when_channel_unset(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap()  # y_brake_output defaults to None — not yet probed
        with pytest.raises(ValueError):
            cmd.disengage_brake(Y_AXIS, io_map)
        assert conn.sent == []  # never reached the wire

    def test_disengage_brake_sends_correct_command_once_configured(self):
        conn = FakeSnapConnection({"SOB 4 1": "0"})
        cmd = MMCCommands(conn)
        io_map = IOMap(y_brake_output=4)
        cmd.disengage_brake(Y_AXIS, io_map)
        assert conn.sent == ["SOB 4 1"]

    def test_brake_is_disengaged_uses_confirmed_z_status_channel(self):
        # z_brake_status_input=1 (INB 1) is confirmed by eab-2026-07-16.dsm
        # and is IOMap's default — should work with zero configuration.
        conn = FakeSnapConnection({"INB 1": "1"})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        assert cmd.brake_is_disengaged(Z_AXIS, io_map) is True

    def test_brake_is_disengaged_raises_for_unconfigured_y(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap()  # y_brake_status_input defaults to None
        with pytest.raises(ValueError):
            cmd.brake_is_disengaged(Y_AXIS, io_map)

    def test_non_brake_axis_always_reports_free(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        io_map = IOMap()
        assert cmd.brake_is_disengaged(X_AXIS, io_map) is True
        assert conn.sent == []
