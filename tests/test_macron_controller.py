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

    def test_gcode_axes_default_to_xy_group_with_separate_z(self):
        # Z can't join the coordinated group on this hardware (confirmed:
        # C1 INI 1 2 succeeds, C1 INI 1 2 5 fails with error 1010) — the
        # group is X/Y only, and Z drives via a separate leg. See
        # GCodeExecutor / gcode.py's "Z/XY node split" note.
        controller = GantryController.from_config(BASE_CONFIG)
        assert controller.gcode._axes == (X_AXIS, Y_AXIS)
        assert controller.gcode._z_axis == Z_AXIS


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

    def test_connect_enables_and_releases_brakes_on_y_and_z_when_safe_mode_false(self):
        # Enable-then-release, per axis, strictly in that order (see
        # _release_brakes_on_connect's docstring): motor torque must be
        # holding before a fail-safe brake like Z's is released.
        responses = {
            "A2 MTR 1": "1", "A2 ENA 1": "1", "SOB 4 1": "0",
            "A5 MTR 1": "1", "A5 ENA 1": "1", "SOB 5 1": "0",
        }
        conn = FakeSnapConnection(responses)
        controller = GantryController(connection=conn, safe_mode=False)
        assert controller.connect() is True
        assert conn.sent == [
            "A2 MTR 1", "A2 ENA 1", "SOB 4 1",
            "A5 MTR 1", "A5 ENA 1", "SOB 5 1",
        ]

    def test_connect_does_not_touch_x_or_theta_on_brake_release(self):
        # X/Theta have no brake at all -- confirm connect() never touches them.
        responses = {
            "A2 MTR 1": "1", "A2 ENA 1": "1", "SOB 4 1": "0",
            "A5 MTR 1": "1", "A5 ENA 1": "1", "SOB 5 1": "0",
        }
        conn = FakeSnapConnection(responses)
        controller = GantryController(connection=conn, safe_mode=False)
        controller.connect()
        assert not any(cmd.startswith("A1 ") for cmd in conn.sent)  # X
        assert not any(cmd.startswith("A6 ") for cmd in conn.sent)  # Theta

    def test_connect_does_not_touch_brakes_when_safe_mode_true(self):
        # safe_mode=True (the default) must leave the connection exactly as
        # found — MTR/SOB are both output-setting commands safe_mode is
        # supposed to block; any attempt here would hit FakeSnapConnection's
        # "no scripted response" assertion since responses is empty.
        conn = FakeSnapConnection({})
        controller = GantryController(connection=conn, safe_mode=True)
        assert controller.connect() is True
        assert conn.sent == []

    def test_connect_skips_brake_release_if_channel_unconfigured_without_raising(self):
        from laguna.robot.macron.commands import IOMap

        responses = {
            "A2 MTR 1": "1", "A2 ENA 1": "1",
            "A5 MTR 1": "1", "A5 ENA 1": "1",
        }
        conn = FakeSnapConnection(responses)
        io_map = IOMap(y_brake_output=None, z_brake_output=None)
        controller = GantryController(connection=conn, safe_mode=False, io_map=io_map)
        assert controller.connect() is True  # must not raise despite unconfigured channels

    def test_connect_continues_to_next_axis_if_enable_fails(self):
        from laguna.robot.macron.connection import SnapMotionError

        responses = {
            "A2 MTR 1": SnapMotionError(99),  # Y fails to enable
            "A5 MTR 1": "1", "A5 ENA 1": "1", "SOB 5 1": "0",  # Z still processed
        }
        conn = FakeSnapConnection(responses)
        controller = GantryController(connection=conn, safe_mode=False)
        assert controller.connect() is True  # connect() itself still succeeds
        assert "SOB 4 1" not in conn.sent  # Y's brake was never touched — enable failed first
        assert "SOB 5 1" in conn.sent

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
        # Z is unchanged from the start position (0,0,0) -> no Z leg, and the
        # coordinated group is X/Y only (Z can't join it on this hardware —
        # see gcode.py's "Z/XY node split" note).
        # Theta moves via begin_move_to (BMT) + poll (MIF), not the blocking
        # MVT — see AxisHandle.move_to() in commands.py.
        responses = {
            "C1 INI 1 2": "0",
            "C1 BMT 10 0": "0",
            "C1 MIF": "1",
            "A6 BMT 0": "0",
            "A6 MIF": "1",
        }
        controller, conn = self._make_controller(responses)
        assert controller.move_to([150.0, 0.0, 0.0, 0.0]) is True
        assert conn.sent == ["C1 INI 1 2", "C1 BMT 10 0", "C1 MIF", "A6 BMT 0", "A6 MIF"]

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
        # single-axis A1 MVT. Z backfills to 0, matching the start position,
        # so no Z leg is needed — the coordinated group is X/Y only (Z
        # can't join it on this hardware, see gcode.py's "Z/XY node split"
        # note).
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
        # at all — no ACP reads, no C1 group commands. Moves via BMT + poll
        # (MIF), not the blocking MVT — see AxisHandle.move_to().
        controller, conn = self._make_controller({"A6 BMT 90": "90", "A6 MIF": "1"})
        assert controller.move_to(Theta=90.0) is True
        assert conn.sent == ["A6 BMT 90", "A6 MIF"]

    def test_keyword_move_of_unconfigured_axis_raises(self):
        conn = FakeSnapConnection({})
        controller = GantryController(connection=conn, axes=(X_AXIS, Y_AXIS))  # no Z configured
        with pytest.raises(ValueError):
            controller.move_to(Z=1.0)

    def test_no_vector_or_keywords_raises(self):
        controller, _conn = self._make_controller({})
        with pytest.raises(ValueError):
            controller.move_to()


class TestSetPosition:
    """set_position() mirrors laguna.weir.SaflWeirController.set_elevation()
    — recalibrates position registers (ACP), commands no motion. Unlike
    move_to(), it never backfills or fence-checks: only the given axes are
    touched at all.
    """

    def _make_controller(self, responses=None, mm_per_unit=15.0):
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn, mm_per_unit=mm_per_unit)
        return controller, conn

    def test_vector_sets_every_configured_axis(self):
        # X=150mm, Y=30mm, Z=45mm (raw 10, 2, 3 at mm_per_unit=15),
        # Theta=90 (unconverted, rotary).
        responses = {
            "A1 ACP 10": "10", "A2 ACP 2": "2", "A5 ACP 3": "3", "A6 ACP 90": "90",
        }
        controller, conn = self._make_controller(responses)
        assert controller.set_position([150.0, 30.0, 45.0, 90.0]) is True
        assert conn.sent == ["A1 ACP 10", "A2 ACP 2", "A5 ACP 3", "A6 ACP 90"]

    def test_vector_length_mismatch_raises(self):
        controller, _conn = self._make_controller({})
        with pytest.raises(ValueError):
            controller.set_position([1.0, 2.0])

    def test_keyword_only_touches_given_axes(self):
        # Only Y given -> only A2 ACP is sent. No backfill reads (unlike
        # move_to), no other axis touched at all.
        controller, conn = self._make_controller({"A2 ACP 2": "2"})
        assert controller.set_position(Y=30.0) is True
        assert conn.sent == ["A2 ACP 2"]

    def test_keyword_of_unconfigured_axis_raises(self):
        conn = FakeSnapConnection({})
        controller = GantryController(connection=conn, axes=(X_AXIS, Y_AXIS))  # no Z configured
        with pytest.raises(ValueError):
            controller.set_position(Z=1.0)

    def test_no_vector_or_keywords_raises(self):
        controller, _conn = self._make_controller({})
        with pytest.raises(ValueError):
            controller.set_position()


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

    def test_home_raises_while_homing_is_disabled(self):
        # Unmocked home_all() — homing is temporarily gated off (physical
        # obstructions block several limit switches on the real machine —
        # see HomingProcedure.home_all()), and home() must let that raise
        # propagate rather than swallow it into a False return.
        controller, _conn = self._make_controller({})
        with pytest.raises(NotImplementedError):
            controller.home()

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


class TestSetSafeModeBrakeSync:
    """set_safe_mode() keeps Y/Z's brakes in sync with the transition:
    turning safe_mode off releases them (motor enabled first, brake
    released second); turning it back on re-engages them. See
    _enable_and_release_brakes()/_engage_brakes() in controller.py.
    """

    def _make_controller(self, responses=None, safe_mode=True):
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn, safe_mode=safe_mode)
        controller._is_connected = True  # simulate an already-connected gantry
        return controller, conn

    def test_turning_safe_mode_off_enables_and_releases_brakes(self):
        responses = {
            "A2 MTR 1": "1", "A2 ENA 1": "1", "SOB 4 1": "0",
            "A5 MTR 1": "1", "A5 ENA 1": "1", "SOB 5 1": "0",
        }
        controller, conn = self._make_controller(responses, safe_mode=True)
        assert controller.set_safe_mode(False) is True
        assert conn.sent == [
            "A2 MTR 1", "A2 ENA 1", "SOB 4 1",
            "A5 MTR 1", "A5 ENA 1", "SOB 5 1",
        ]

    def test_turning_safe_mode_on_engages_brakes(self):
        responses = {"SOB 4 0": "0", "SOB 5 0": "0"}
        controller, conn = self._make_controller(responses, safe_mode=False)
        assert controller.set_safe_mode(True) is True
        assert conn.sent == ["SOB 4 0", "SOB 5 0"]

    def test_no_op_when_not_connected(self):
        conn = FakeSnapConnection({})  # any brake/enable command would raise (unscripted)
        controller = GantryController(connection=conn, safe_mode=True)
        assert controller.set_safe_mode(False) is True
        assert conn.sent == []
        assert controller._safe_mode is False

    def test_pi_gantry_connection_engages_brakes_after_reconnect_when_turning_on(self, monkeypatch):
        from laguna.robot.macron.pi_bridge import PiGantryConnection

        conn = PiGantryConnection(host="fake", ssh_user="oak", remote_serial_device="/dev/fake")
        controller = GantryController(connection=conn, safe_mode=False)
        controller._is_connected = True
        monkeypatch.setattr(controller, "disconnect", lambda: None)
        monkeypatch.setattr(controller, "connect", lambda: True)
        engaged = []
        monkeypatch.setattr(controller, "_engage_brakes", lambda: engaged.append(True))

        assert controller.set_safe_mode(True) is True
        assert engaged == [True]

    def test_pi_gantry_connection_does_not_engage_brakes_when_turning_off(self, monkeypatch):
        # connect() itself releases brakes when safe_mode is already False
        # (tested separately in TestSubsystemInterface) — set_safe_mode()
        # must not also call _engage_brakes() in this direction.
        from laguna.robot.macron.pi_bridge import PiGantryConnection

        conn = PiGantryConnection(host="fake", ssh_user="oak", remote_serial_device="/dev/fake")
        controller = GantryController(connection=conn, safe_mode=True)
        controller._is_connected = True
        monkeypatch.setattr(controller, "disconnect", lambda: None)
        monkeypatch.setattr(controller, "connect", lambda: True)
        engaged = []
        monkeypatch.setattr(controller, "_engage_brakes", lambda: engaged.append(True))

        assert controller.set_safe_mode(False) is True
        assert engaged == []

    def test_pi_gantry_connection_reconnect_failure_propagates_false(self, monkeypatch):
        from laguna.robot.macron.pi_bridge import PiGantryConnection

        conn = PiGantryConnection(host="fake", ssh_user="oak", remote_serial_device="/dev/fake")
        controller = GantryController(connection=conn, safe_mode=True)
        controller._is_connected = True
        monkeypatch.setattr(controller, "disconnect", lambda: None)
        monkeypatch.setattr(controller, "connect", lambda: False)
        engaged = []
        monkeypatch.setattr(controller, "_engage_brakes", lambda: engaged.append(True))

        assert controller.set_safe_mode(False) is False
        assert engaged == []  # never reached — reconnect failed first
