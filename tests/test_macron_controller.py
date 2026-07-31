"""Tests for GantryController: config-driven construction and the
FlumeLab subsystem interface (connect/disconnect/get_status/stop).
"""

import pytest

from laguna.robot.macron.commands import THETA_AXIS, X_AXIS, Y_AXIS, Z_AXIS
from laguna.robot.macron.connection import EthernetConnection, RS232Connection
from laguna.robot.macron.controller import GantryController
from laguna.robot.macron.fences import BoxFence
from laguna.robot.macron.pi_bridge import PiGantryConnection
from tests.macron_fixtures import FakeSnapConnection

BASE_CONFIG = {
    "transport": "socket_bridge",
    "host": "red.dyn.ucr.edu",
    "bridge_port": 9700,
    "group_index": 1,
    "safe_mode": True,
    "axes": [
        {"name": "X", "index": 1},
        {"name": "Y", "index": 2, "brake_output": 4, "brake_status_input": 8},
        {"name": "Z", "index": 5},
        {"name": "Theta", "index": 6},
    ],
    "homing": {"speed_mm_s": 10.0, "standoff_mm": 5.0, "order": ["Z", "X", "Y"]},
    "fences": [{"type": "box", "name": "bed", "x": [0, 500], "y": [0, 300], "z": [0, 5]}],
}


class TestFromConfigTransport:
    def test_socket_bridge_transport(self):
        controller = GantryController.from_config(BASE_CONFIG)
        assert isinstance(controller._connection, RS232Connection)
        assert controller._connection.port == "socket://red.dyn.ucr.edu:9700"

    def test_pi_agent_transport(self):
        cfg = dict(
            BASE_CONFIG,
            transport="pi_agent",
            ssh_user="oak",
            ssh_key="~/.ssh/id_ed25519",
            remote_serial_device="/dev/ttyUSB0",
        )
        controller = GantryController.from_config(cfg)
        assert isinstance(controller._connection, PiGantryConnection)
        assert controller._connection.ssh_user == "oak"
        assert controller._connection.safe_mode is True

    def test_ethernet_transport(self):
        cfg = dict(BASE_CONFIG, transport="ethernet", ethernet={"host": "10.0.0.5", "port": 23})
        controller = GantryController.from_config(cfg)
        assert isinstance(controller._connection, EthernetConnection)
        assert controller._connection.host == "10.0.0.5"

    def test_rs232_direct_transport(self):
        cfg = dict(BASE_CONFIG, transport="rs232", rs232={"port": "/dev/ttyUSB0", "baud": 19200})
        controller = GantryController.from_config(cfg)
        assert isinstance(controller._connection, RS232Connection)
        assert controller._connection.port == "/dev/ttyUSB0"
        assert controller._connection.baudrate == 19200

    def test_unknown_transport_raises(self):
        cfg = dict(BASE_CONFIG, transport="carrier_pigeon")
        with pytest.raises(ValueError):
            GantryController.from_config(cfg)


class TestFromConfigAxesAndIOMap:
    def test_axes_resolved_in_config_order(self):
        controller = GantryController.from_config(BASE_CONFIG)
        assert controller._axes == (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)

    def test_io_map_picks_up_confirmed_and_configured_channels(self):
        controller = GantryController.from_config(BASE_CONFIG)
        io_map = controller._io_map
        assert io_map.y_brake_output == 4
        assert io_map.y_brake_status_input == 8
        # z_brake_status_input / theta_limit_input live on the responder's
        # own input bank — unreachable via ASCII, so they stay None even
        # when unconfigured in the axes list (see IOMap docstring).
        assert io_map.z_brake_status_input is None
        assert io_map.theta_limit_input is None

    def test_missing_axes_falls_back_to_default_four(self):
        cfg = dict(BASE_CONFIG)
        cfg.pop("axes")
        controller = GantryController.from_config(cfg)
        assert controller._axes == (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)

    def test_gcode_axes_default_to_xyz(self):
        controller = GantryController.from_config(BASE_CONFIG)
        assert controller.gcode._axes == (X_AXIS, Y_AXIS, Z_AXIS)


class TestFromConfigHomingAndFences:
    def test_homing_order_resolved_from_names(self):
        controller = GantryController.from_config(BASE_CONFIG)
        assert controller.homing._config.home_order == (Z_AXIS, X_AXIS, Y_AXIS)

    def test_homing_order_unknown_axis_raises(self):
        cfg = dict(BASE_CONFIG, homing={"order": ["NotAnAxis"]})
        with pytest.raises(KeyError):
            GantryController.from_config(cfg)

    def test_fences_built_from_config(self):
        controller = GantryController.from_config(BASE_CONFIG)
        assert len(controller.fence_registry) == 1
        fence = controller.fence_registry.get("bed")
        assert isinstance(fence, BoxFence)

    def test_unknown_fence_type_raises(self):
        cfg = dict(BASE_CONFIG, fences=[{"type": "sphere", "name": "x"}])
        with pytest.raises(ValueError):
            GantryController.from_config(cfg)

    def test_no_fences_gives_empty_registry(self):
        cfg = dict(BASE_CONFIG)
        cfg.pop("fences")
        controller = GantryController.from_config(cfg)
        assert len(controller.fence_registry) == 0


class TestSubsystemInterface:
    def _make_controller(self, responses=None):
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn)
        return controller, conn

    def test_subsystem_name_is_gantry(self):
        assert GantryController.subsystem_name == "gantry"

    def test_connection_property_exposes_the_underlying_transport(self):
        """Regression test: TopographicProfiler.scan() calls
        gantry.connection.start_scan(...) — this must be a real public
        attribute, not just something a MagicMock-based test fixture
        happens to tolerate. Caught on real hardware 2026-07-28: profiler
        tests used MagicMock() for `gantry` and manually set
        `gantry.connection = ...`, which silently worked even though
        GantryController only ever stored the connection as the private
        `_connection` — no test using a real GantryController instance
        caught the gap until an actual scan script hit
        AttributeError: 'GantryController' object has no attribute 'connection'.
        """
        controller, conn = self._make_controller()
        assert controller.connection is conn

    def test_connect_reports_true_on_success(self):
        controller, conn = self._make_controller()
        assert controller.connect() is True
        assert conn.is_connected is True

    def test_disconnect_clears_connected_state(self):
        controller, conn = self._make_controller()
        controller.connect()
        controller.disconnect()
        assert conn.is_connected is False

    def test_get_status_when_disconnected(self):
        controller, _conn = self._make_controller()
        status = controller.get_status()
        assert status["subsystem"] == "gantry"
        assert status["is_connected"] is False
        assert "positions" not in status

    def test_get_status_when_connected_reads_all_axis_positions(self):
        responses = {"A1 ACP": "1.000", "A2 ACP": "2.000", "A5 ACP": "5.000", "A6 ACP": "6.000"}
        controller, _conn = self._make_controller(responses)
        controller.connect()
        status = controller.get_status()
        assert status["positions"] == {"X": 1.0, "Y": 2.0, "Z": 5.0, "Theta": 6.0}

    def test_stop_aborts_all_axes(self):
        responses = {f"A{i} ABT": "0" for i in (1, 2, 5, 6)}
        responses.update({f"A{i} MTR 0": "0" for i in (1, 2, 5, 6)})
        controller, conn = self._make_controller(responses)
        controller.connect()
        controller.stop()
        assert "A1 ABT" in conn.sent
        assert "A5 ABT" in conn.sent

    def test_stop_never_raises_even_on_errors(self):
        controller, _conn = self._make_controller({})  # every command hits an unscripted response
        controller.connect()
        controller.stop()  # must not propagate


class TestMoveTo:
    def _make_controller(self, responses=None, mm_per_unit=15.0):
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn, mm_per_unit=mm_per_unit)
        return controller, conn

    def test_vector_move_routes_through_fence_checked_gcode_path(self):
        # X=150mm, Y=0, Z=0 (raw 10 0 0 at mm_per_unit=15), Theta=0 (unconverted).
        # DEBUG PATCH (branch debug/e415117-no-cross-node-group): the group
        # never spans Z (the responder node) — see GCodeExecutor's class
        # docstring — and Z is unchanged here, so no Z leg is sent at all.
        # Theta is read live (A6 ACP) and, since it's already at 0, no
        # Theta move is sent at all (move_to() — blocking MVT — is banned
        # regardless; see commands.py).
        responses = {
            "C1 INI 1 2": "0",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "A6 ACP": "0",
        }
        controller, conn = self._make_controller(responses)
        assert controller.move_to([150.0, 0.0, 0.0, 0.0]) is True
        assert conn.sent == ["C1 INI 1 2", "C1 BMT 10 0", "C1 MIF", "A6 ACP"]

    def test_vector_move_length_mismatch_raises(self):
        controller, _conn = self._make_controller({})
        with pytest.raises(ValueError):
            controller.move_to([1.0, 2.0])

    def test_vector_move_is_fence_checked(self):
        cfg = dict(BASE_CONFIG, fences=[{"type": "box", "name": "bed", "x": [0, 10], "y": [0, 10], "z": [0, 10]}])
        controller = GantryController.from_config(cfg)
        from laguna.robot.macron.fences import FenceViolation

        with pytest.raises(FenceViolation):
            controller.move_to([500.0, 500.0, 5.0, 0.0])

    def test_keyword_move_backfills_other_cartesian_axes_and_routes_through_gcode(self):
        # Only X given -> Y/Z backfilled via a live get_actual_position()
        # read, then the whole thing goes through the same coordinated
        # gcode path as the vector form (C1 INI/SPD/BMT/MIF), not a
        # single-axis A1 MVT.
        responses = {
            "A2 ACP": "0",  # Y backfill
            "A5 ACP": "0",  # Z backfill
            "C1 INI 1 2": "0",
            "C1 SPD 0.133333": "0.133333",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
        }
        controller, conn = self._make_controller(responses)
        assert controller.move_to(X=150.0, speed=2.0) is True
        assert conn.sent == [
            "A2 ACP", "A5 ACP", "C1 INI 1 2", "C1 SPD 0.133333", "C1 BMT 10 0", "C1 MIF",
        ]

    def test_keyword_move_is_fence_checked(self):
        from laguna.robot.macron.fences import BoxFence, FenceViolation

        conn = FakeSnapConnection({"A2 ACP": "0", "A5 ACP": "0"})
        controller = GantryController(
            connection=conn, mm_per_unit=15.0,
            fences=[BoxFence("bed", 0, 10, 0, 10, 0, 10)],
        )
        with pytest.raises(FenceViolation):
            controller.move_to(X=500.0)  # Y/Z backfill to 0,0 (in-bounds); X clearly outside

    def test_theta_only_keyword_move_does_not_touch_cartesian_axes(self):
        # A pure Theta move must not query, move, or otherwise touch X/Y/Z
        # at all — no ACP reads for X/Y/Z, no C1 group commands.
        # DEBUG PATCH (branch debug/e415117-no-cross-node-group): Theta now
        # reads its live position first (A6 ACP) and moves non-blocking
        # (A6 BMT + A6 MIF poll) instead of a blocking A6 MVT — move_to()
        # is banned; see commands.py.
        controller, conn = self._make_controller({
            "A6 ACP": "0", "A6 BMT 90": "0", "A6 MIF": "1",
        })
        assert controller.move_to(Theta=90.0) is True
        assert conn.sent == ["A6 ACP", "A6 BMT 90", "A6 MIF"]

    def test_theta_move_skipped_entirely_when_already_at_target(self):
        # DEBUG PATCH: the vector move_to() form always passes a Theta
        # value (0.0 if the caller doesn't care) — if Theta is already
        # there, no move (blocking or not) should be sent at all.
        controller, conn = self._make_controller({"A6 ACP": "0"})
        assert controller.move_to(Theta=0.0) is True
        assert conn.sent == ["A6 ACP"]

    def test_keyword_move_of_unconfigured_axis_raises(self):
        conn = FakeSnapConnection({})
        controller = GantryController(connection=conn, axes=(X_AXIS, Y_AXIS))  # no Z configured
        with pytest.raises(ValueError):
            controller.move_to(Z=1.0)

    def test_no_vector_or_keywords_raises(self):
        controller, _conn = self._make_controller({})
        with pytest.raises(ValueError):
            controller.move_to()


class TestHomeEnableDisableWaitForMove:
    def _make_controller(self, responses=None):
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn)
        return controller, conn

    def test_home_delegates_to_homing_home_all(self, monkeypatch):
        controller, _conn = self._make_controller({})
        from laguna.robot.macron.homing import HomingResult

        monkeypatch.setattr(controller.homing, "home_all", lambda: HomingResult(success=True, axis_results={}))
        assert controller.home() is True

    def test_home_reports_failure(self, monkeypatch):
        controller, _conn = self._make_controller({})
        from laguna.robot.macron.homing import HomingResult

        monkeypatch.setattr(
            controller.homing, "home_all",
            lambda: HomingResult(success=False, axis_results={}, error="timeout"),
        )
        assert controller.home() is False

    def test_enable_enables_motor_and_drive_on_every_axis(self):
        responses = {f"A{i} MTR 1": "1" for i in (1, 2, 5, 6)}
        responses.update({f"A{i} ENA 1": "1" for i in (1, 2, 5, 6)})
        controller, conn = self._make_controller(responses)
        controller.enable()
        assert "A1 MTR 1" in conn.sent
        assert "A1 ENA 1" in conn.sent

    def test_disable_disables_drive_and_motor_on_every_axis(self):
        responses = {f"A{i} ENA 0": "0" for i in (1, 2, 5, 6)}
        responses.update({f"A{i} MTR 0": "0" for i in (1, 2, 5, 6)})
        controller, conn = self._make_controller(responses)
        controller.disable()
        assert "A1 ENA 0" in conn.sent
        assert "A1 MTR 0" in conn.sent

    def test_wait_for_move_polls_until_finished(self):
        calls = {"n": 0}

        def mif_response(cmd):
            calls["n"] += 1
            return "1" if calls["n"] >= 2 else "0"

        controller, _conn = self._make_controller({"C1 MIF": mif_response})
        controller.wait_for_move(timeout=5.0)
        assert calls["n"] == 2

    def test_wait_for_move_times_out(self):
        controller, _conn = self._make_controller({"C1 MIF": "0"})
        with pytest.raises(TimeoutError):
            controller.wait_for_move(timeout=0.05)
