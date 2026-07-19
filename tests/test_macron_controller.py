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
