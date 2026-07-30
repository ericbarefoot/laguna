"""Tests for MMCCommands' mm_per_unit / coordinate_offset_mm conversion —
the software workaround for the 15 mm/unit gantry finding (see
docs/GANTRY_UNIT_CALIBRATION.md). Offline, driven against FakeSnapConnection.
"""

from laguna.robot.macron.commands import MMCCommands, THETA_AXIS, X_AXIS, Y_AXIS, Z_AXIS
from tests.macron_fixtures import FakeSnapConnection


class TestPositionConversion:
    def test_get_actual_position_applies_scale(self):
        conn = FakeSnapConnection({"A1 ACP": "10"})
        cmd = MMCCommands(conn, mm_per_unit=15.0)
        assert cmd.get_actual_position(X_AXIS) == 150.0

    def test_get_actual_position_applies_offset(self):
        conn = FakeSnapConnection({"A1 ACP": "10"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, coordinate_offset_mm={"X": 5.0})
        assert cmd.get_actual_position(X_AXIS) == 155.0

    def test_theta_is_never_converted(self):
        conn = FakeSnapConnection({"A6 ACP": "10"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, coordinate_offset_mm={"Theta": 100.0})
        assert cmd.get_actual_position(THETA_AXIS) == 10.0

    def test_default_mm_per_unit_is_identity(self):
        conn = FakeSnapConnection({"A1 ACP": "10"})
        cmd = MMCCommands(conn)
        assert cmd.get_actual_position(X_AXIS) == 10.0

    def test_set_actual_position_converts_mm_to_raw_and_back(self):
        conn = FakeSnapConnection({"A1 ACP 10": "10"})
        cmd = MMCCommands(conn, mm_per_unit=15.0)
        result = cmd.set_actual_position(X_AXIS, 150.0)
        assert conn.sent == ["A1 ACP 10"]
        assert result == 150.0

    def test_move_to_converts_absolute_position(self):
        conn = FakeSnapConnection({"A1 MVT 10": "0"})
        cmd = MMCCommands(conn, mm_per_unit=15.0)
        cmd.move_to(X_AXIS, 150.0)
        assert conn.sent == ["A1 MVT 10"]

    def test_move_to_applies_offset_before_scaling(self):
        # world mm 155, offset 5 -> raw position 150, then /15 -> 10 raw units
        conn = FakeSnapConnection({"A1 MVT 10": "0"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, coordinate_offset_mm={"X": 5.0})
        cmd.move_to(X_AXIS, 155.0)
        assert conn.sent == ["A1 MVT 10"]

    def test_move_by_delta_ignores_offset(self):
        # a relative delta must NOT have the offset subtracted — only scale applies
        conn = FakeSnapConnection({"A1 MVB 10": "0"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, coordinate_offset_mm={"X": 100.0})
        cmd.move_by(X_AXIS, 150.0)
        assert conn.sent == ["A1 MVB 10"]


class TestVelocityConversion:
    def test_set_speed_scale_only_no_offset(self):
        conn = FakeSnapConnection({"A1 SPD 2": "2"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, coordinate_offset_mm={"X": 1000.0})
        result = cmd.set_speed(X_AXIS, 30.0)
        assert conn.sent == ["A1 SPD 2"]
        assert result == 30.0

    def test_theta_speed_unconverted(self):
        conn = FakeSnapConnection({"A6 SPD 5": "5"})
        cmd = MMCCommands(conn, mm_per_unit=15.0)
        assert cmd.set_speed(THETA_AXIS, 5.0) == 5.0


class TestGroupMotionConversion:
    def test_group_begin_move_to_converts_each_axis_by_position(self):
        conn = FakeSnapConnection({"C1 BMT 10 0 0": "0"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, group_axes=(X_AXIS, Y_AXIS, Z_AXIS))
        cmd.group_begin_move_to(150.0, 0.0, 0.0)
        assert conn.sent == ["C1 BMT 10 0 0"]

    def test_group_begin_move_by_is_scale_only(self):
        conn = FakeSnapConnection({"C1 BMB 10 0 0": "0"})
        cmd = MMCCommands(
            conn, mm_per_unit=15.0, coordinate_offset_mm={"X": 1000.0},
            group_axes=(X_AXIS, Y_AXIS, Z_AXIS),
        )
        cmd.group_begin_move_by(150.0, 0.0, 0.0)
        assert conn.sent == ["C1 BMB 10 0 0"]

    def test_group_set_speed_uses_first_group_axis_scale(self):
        conn = FakeSnapConnection({"C1 SPD 2": "2"})
        cmd = MMCCommands(conn, mm_per_unit=15.0, group_axes=(X_AXIS, Y_AXIS, Z_AXIS))
        assert cmd.group_set_speed(30.0) == 30.0
        assert conn.sent == ["C1 SPD 2"]

    def test_default_group_axes_is_xyz(self):
        conn = FakeSnapConnection({"C1 BMT 1 1 1": "0"})
        cmd = MMCCommands(conn)  # defaults: mm_per_unit=1.0, group_axes=(X,Y,Z)
        cmd.group_begin_move_to(1.0, 1.0, 1.0)
        assert conn.sent == ["C1 BMT 1 1 1"]
