"""Tests for GantryController: config-driven construction and the
FlumeLab subsystem interface (connect/disconnect/get_status/stop).
"""

import pytest

from laguna.config import Config
from laguna.robot.macron.commands import THETA_AXIS, X_AXIS, Y_AXIS, Z_AXIS
from laguna.robot.macron.connection import EthernetConnection, RS232Connection
from laguna.robot.macron.controller import GantryController
from laguna.robot.macron.fences import BoxFence
from laguna.robot.macron.pi_bridge import PiGantryConnection
from laguna.robot.macron.position_store import GantryPositionStore
from tests.macron_fixtures import FakeSnapConnection


def _cfg(gantry_dict: dict) -> Config:
    """Wrap a bare 'gantry:' section dict as the Config object from_config() expects."""
    return Config(defaults={"gantry": gantry_dict})

# Deliberately still socket_bridge: that transport is retired as the *default*
# (pi_agent is now, see src/laguna/config.py) but the code path remains
# supported, and it's the cheapest fixture here — it builds an RS232Connection
# without needing SSH parameters, and no test below actually connects.
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
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
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
        controller = GantryController.from_config(_cfg(cfg))
        assert isinstance(controller._connection, PiGantryConnection)
        assert controller._connection.ssh_user == "oak"
        assert controller._connection.safe_mode is True

    def test_ethernet_transport(self):
        cfg = dict(BASE_CONFIG, transport="ethernet", ethernet={"host": "10.0.0.5", "port": 23})
        controller = GantryController.from_config(_cfg(cfg))
        assert isinstance(controller._connection, EthernetConnection)
        assert controller._connection.host == "10.0.0.5"

    def test_rs232_direct_transport(self):
        cfg = dict(BASE_CONFIG, transport="rs232", rs232={"port": "/dev/ttyUSB0", "baud": 19200})
        controller = GantryController.from_config(_cfg(cfg))
        assert isinstance(controller._connection, RS232Connection)
        assert controller._connection.port == "/dev/ttyUSB0"
        assert controller._connection.baudrate == 19200

    def test_unknown_transport_raises(self):
        cfg = dict(BASE_CONFIG, transport="carrier_pigeon")
        with pytest.raises(ValueError):
            GantryController.from_config(_cfg(cfg))


class TestFromConfigAxesAndIOMap:
    def test_axes_resolved_in_config_order(self):
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert controller._axes == (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)

    def test_io_map_picks_up_confirmed_and_configured_channels(self):
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
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
        controller = GantryController.from_config(_cfg(cfg))
        assert controller._axes == (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)

    def test_soft_limits_resolved_from_axes_config(self):
        cfg = dict(BASE_CONFIG)
        cfg["axes"] = [
            {"name": "X", "index": 1, "soft_negative_limit_mm": -5, "soft_positive_limit_mm": 495},
            {"name": "Y", "index": 2, "brake_output": 4, "brake_status_input": 8},
            {"name": "Z", "index": 5},
            {"name": "Theta", "index": 6},
        ]
        controller = GantryController.from_config(_cfg(cfg))
        assert controller._soft_limits == {"X": (-5, 495)}

    def test_axis_with_no_soft_limits_configured_is_absent(self):
        # Y has no soft_*_limit_mm keys at all in BASE_CONFIG — must not
        # appear in _soft_limits, not appear as (None, None).
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert controller._soft_limits == {}

    def test_gcode_group_is_xy_only_with_z_held_separately(self):
        """Z cannot join the commander-node coordinated group on this
        hardware, so the executor takes the two commander axes as its
        group and Z as its own axis — see gcode.py's module docstring."""
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert controller.gcode._axes == (X_AXIS, Y_AXIS)
        assert controller.gcode._z_axis == Z_AXIS

    def test_theta_group_is_wired_to_a_second_mmc_commands_on_group_2(self):
        """Z and Theta share the responder node, so they get their own
        coordinated group (default index 2) via a second MMCCommands
        sharing the gantry's connection — see issue #24."""
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert controller.theta_cmd is not controller.cmd
        assert controller.theta_cmd._group == 2
        assert controller.theta_cmd._group_axes == (Z_AXIS, THETA_AXIS)
        assert controller.gcode._theta_cmd is controller.theta_cmd
        assert controller.gcode._theta_axis == THETA_AXIS

    def test_theta_group_index_is_configurable(self):
        config = dict(BASE_CONFIG, theta_group_index=7)
        controller = GantryController.from_config(_cfg(config))
        assert controller.theta_cmd._group == 7
        assert controller.gcode._theta_group_index == 7

    def test_per_axis_mm_per_unit_override_reaches_both_mmc_commands(self):
        """An axis's "mm_per_unit" key in config's axes list (e.g. Z's
        confirmed 13.5, distinct from the shared mm_per_acp_unit default)
        must reach both self.cmd and self.theta_cmd — Z is addressed
        through either depending on whether it's moving alone or grouped
        with Theta (see GCodeExecutor._begin_z_leg / _begin_zt_leg).
        """
        cfg = dict(BASE_CONFIG, mm_per_acp_unit=15.0)
        cfg["axes"] = [
            {"name": "X", "index": 1},
            {"name": "Y", "index": 2, "brake_output": 4, "brake_status_input": 8},
            {"name": "Z", "index": 5, "mm_per_unit": 13.5},
            {"name": "Theta", "index": 6},
        ]
        controller = GantryController.from_config(_cfg(cfg))
        assert controller.cmd._axis_mm_per_unit == {"Z": 13.5}
        assert controller.theta_cmd._axis_mm_per_unit == {"Z": 13.5}
        assert controller.cmd._unit_for(Z_AXIS) == 13.5
        assert controller.cmd._unit_for(X_AXIS) == 15.0  # unaffected, still the shared default


class TestFromConfigHomingAndFences:
    def test_homing_order_resolved_from_names(self):
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert controller.homing._config.home_order == (Z_AXIS, X_AXIS, Y_AXIS)

    def test_homing_order_unknown_axis_raises(self):
        cfg = dict(BASE_CONFIG, homing={"order": ["NotAnAxis"]})
        with pytest.raises(KeyError):
            GantryController.from_config(_cfg(cfg))

    def test_axis_configs_default_to_home_switch_and_default_direction(self):
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        x_cfg = controller.homing._config.axis_config(X_AXIS)
        assert x_cfg.input_index == 1  # io_map.x_home_input
        assert x_cfg.trip_on_high is False  # default matches confirmed hardware
        assert x_cfg.homing_direction == -1.0

    def test_axis_configs_only_built_for_axes_in_home_order(self):
        # BASE_CONFIG's home_order is [Z, X, Y] — Theta never gets an
        # AxisHomingConfig (no home/limit switch homing support for it here).
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert controller.homing._config.axis_config(THETA_AXIS) is None

    def test_home_switch_can_be_set_to_limit(self):
        cfg = dict(BASE_CONFIG)
        cfg["axes"] = [
            {"name": "X", "index": 1, "home_switch": "limit"},
            {"name": "Y", "index": 2, "brake_output": 4, "brake_status_input": 8},
            {"name": "Z", "index": 5},
            {"name": "Theta", "index": 6},
        ]
        controller = GantryController.from_config(_cfg(cfg))
        x_cfg = controller.homing._config.axis_config(X_AXIS)
        assert x_cfg.input_index == 2  # io_map.x_limit_input

    def test_home_trip_on_high_overridable_per_axis(self):
        cfg = dict(BASE_CONFIG)
        cfg["axes"] = [
            {"name": "X", "index": 1, "home_trip_on_high": True},
            {"name": "Y", "index": 2, "brake_output": 4, "brake_status_input": 8},
            {"name": "Z", "index": 5},
            {"name": "Theta", "index": 6},
        ]
        controller = GantryController.from_config(_cfg(cfg))
        assert controller.homing._config.axis_config(X_AXIS).trip_on_high is True

    def test_unknown_home_switch_value_raises(self):
        cfg = dict(BASE_CONFIG)
        cfg["axes"] = [{"name": "X", "index": 1, "home_switch": "banana"}]
        cfg["homing"] = {"order": ["X"]}
        with pytest.raises(ValueError):
            GantryController.from_config(_cfg(cfg))

    def test_fences_built_from_config(self):
        controller = GantryController.from_config(_cfg(BASE_CONFIG))
        assert len(controller.fence_registry) == 1
        fence = controller.fence_registry.get("bed")
        assert isinstance(fence, BoxFence)

    def test_unknown_fence_type_raises(self):
        cfg = dict(BASE_CONFIG, fences=[{"type": "sphere", "name": "x"}])
        with pytest.raises(ValueError):
            GantryController.from_config(_cfg(cfg))

    def test_no_fences_gives_empty_registry(self):
        cfg = dict(BASE_CONFIG)
        cfg.pop("fences")
        controller = GantryController.from_config(_cfg(cfg))
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
        responses = {"A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0"}
        controller, conn = self._make_controller(responses)
        assert controller.connect() is True
        assert conn.is_connected is True

    def test_connect_while_already_connected_is_a_no_op(self):
        """A second connect() call must not re-invoke the underlying
        transport's connect() — on PiGantryConnection that overwrites the
        live SSH channel/agent handles with a fresh session before tearing
        down the old one, leaking the old SSH session, reader thread, and
        remote gantry_agent.py process (which holds the serial port
        exclusively), so the *new* agent can't get ready either — both ends
        up unrecoverable short of killing the whole process."""
        responses = {"A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0"}
        controller, conn = self._make_controller(responses)
        assert controller.connect() is True
        sent_after_first_connect = list(conn.sent)
        assert controller.connect() is True
        assert conn.sent == sent_after_first_connect  # nothing sent to the wire again
        assert conn.connect_calls == 1

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

    def test_get_position_returns_vector_in_axes_order(self):
        responses = {"A1 ACP": "1.000", "A2 ACP": "2.000", "A5 ACP": "5.000", "A6 ACP": "6.000"}
        controller, _conn = self._make_controller(responses)
        controller.connect()
        assert controller.get_position() == [1.0, 2.0, 5.0, 6.0]

    def test_estop_aborts_all_axes(self):
        """estop() is the zero-decel abort. This used to be what stop() did;
        the unified safety vocabulary moved it here (see laguna.safety)."""
        responses = {f"A{i} ABT": "0" for i in (1, 2, 5, 6)}
        responses.update({f"A{i} MTR 0": "0" for i in (1, 2, 5, 6)})
        controller, conn = self._make_controller(responses)
        controller.connect()
        controller.estop()
        assert "A1 ABT" in conn.sent
        assert "A5 ABT" in conn.sent

    def test_stop_decelerates_rather_than_aborting(self):
        """stop() is now the CLEAN stop: decelerate on each axis's own ramp
        and park the brakes, so nothing stalls and the encoder isn't
        disturbed. No ABT."""
        responses = {f"A{i} BST": "0" for i in (1, 2, 5, 6)}
        responses.update({"SOB 4 0": "0", "SOB 5 0": "0"})
        controller, conn = self._make_controller(responses)
        controller.connect()
        controller.stop()
        assert "A1 BST" in conn.sent
        assert not any("ABT" in c for c in conn.sent), "stop() must not hard-abort"

    def test_pause_decelerates_without_touching_brakes(self):
        """pause() has to be cheap enough to use liberally — the gantry stays
        immediately movable, with no re-enable cycle."""
        responses = {f"A{i} BST": "0" for i in (1, 2, 5, 6)}
        controller, conn = self._make_controller(responses)
        controller.connect()
        controller.pause()
        assert "A1 BST" in conn.sent
        assert not any("SOB" in c or "ABT" in c for c in conn.sent)

    def test_safety_verbs_never_raise_even_on_errors(self):
        controller, _conn = self._make_controller({})  # every command unscripted
        controller.connect()
        controller.pause()
        controller.stop()
        controller.estop()   # none may propagate


class TestMoveTo:
    def _make_controller(self, responses=None, mm_per_unit=15.0):
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn, mm_per_unit=mm_per_unit)
        return controller, conn

    def test_vector_move_routes_through_fence_checked_gcode_path(self):
        # X=150mm, Y=0, Z=0 (raw 10 0 0 at mm_per_unit=15), Theta=0 (unconverted).
        # the group
        # never spans Z (the responder node) — see GCodeExecutor's class
        # docstring — and Z is unchanged here, so no Z leg is sent at all.
        # Theta is read live (A6 ACP) and, since it's already at 0, no
        # Theta move is sent at all (move_to() — blocking MVT — is banned
        # regardless; see commands.py).
        responses = {
            "C1 INI 1 2": "0",
            "C1 BMT 10 0": "0",
            "C1 SPD": "20",  # no F word/speed given — read X/Y's own speed to predict duration
            "C1 MIF": "1",
            "A1 ACP": "150",  # post-move resync of X/Y from hardware
            "A2 ACP": "0",
            "A6 ACP": "0",
        }
        controller, conn = self._make_controller(responses)
        assert controller.move_to([150.0, 0.0, 0.0, 0.0]) is True
        assert conn.sent == [
            "C1 INI 1 2", "C1 BMT 10 0", "C1 SPD", "C1 MIF", "A1 ACP", "A2 ACP", "A6 ACP",
        ]

    def test_vector_move_length_mismatch_raises(self):
        controller, _conn = self._make_controller({})
        with pytest.raises(ValueError):
            controller.move_to([1.0, 2.0])

    def test_vector_move_is_fence_checked(self):
        cfg = dict(BASE_CONFIG, fences=[{"type": "box", "name": "bed", "x": [0, 10], "y": [0, 10], "z": [0, 10]}])
        controller = GantryController.from_config(_cfg(cfg))
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
            "A1 ACP": "150",  # post-move resync of X/Y from hardware
        }
        controller, conn = self._make_controller(responses)
        assert controller.move_to(X=150.0, speed=2.0) is True
        assert conn.sent == [
            "A2 ACP", "A5 ACP", "C1 INI 1 2", "C1 SPD 0.133333", "C1 BMT 10 0", "C1 MIF",
            "A1 ACP", "A2 ACP",
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
        # Theta now
        # reads its live position first (A6 ACP) and moves non-blocking
        # (A6 BMT + A6 MIF poll) instead of a blocking A6 MVT — move_to()
        # is banned; see commands.py.
        controller, conn = self._make_controller({
            "A6 ACP": "0", "A6 BMT 90": "0", "A6 MIF": "1",
        })
        assert controller.move_to(Theta=90.0) is True
        assert conn.sent == ["A6 ACP", "A6 BMT 90", "A6 MIF"]

    def test_theta_move_skipped_entirely_when_already_at_target(self):
        # the vector move_to() form always passes a Theta
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

    def test_locate_limit_switch_delegates_to_homing_and_accepts_axis_forms(self, monkeypatch):
        controller, _conn = self._make_controller({})
        seen = []
        monkeypatch.setattr(
            controller.homing, "locate_limit_switch",
            lambda axis: seen.append(axis) or 42.0,
        )
        monkeypatch.setattr(controller.gcode, "sync_position_from_hardware", lambda: None)
        assert controller.locate_limit_switch("X") == 42.0
        assert controller.locate_limit_switch(controller.x) == 42.0
        assert seen == [X_AXIS, X_AXIS]  # resolved to the underlying Axis both times

    def test_home_axis_delegates_and_accepts_axis_forms(self, monkeypatch):
        controller, _conn = self._make_controller({})
        seen = []
        monkeypatch.setattr(
            controller.homing, "home_axis",
            lambda axis: seen.append(axis) or 5.0,
        )
        monkeypatch.setattr(controller.gcode, "sync_position_from_hardware", lambda: None)
        assert controller.home_axis("Y") == 5.0
        assert controller.home_axis(Y_AXIS) == 5.0
        assert seen == [Y_AXIS, Y_AXIS]

    def test_home_axis_resyncs_gcode_position_even_on_failure(self, monkeypatch):
        # home_axis()/home()/locate_limit_switch() all move hardware
        # directly, bypassing GCodeExecutor — without resyncing its
        # cached position afterward, the next move_to() plans against a
        # stale position and can send a near-zero leg that scales ACL/DCL
        # to 0 (escape 16/17). Must resync even when homing raises.
        from laguna.robot.macron.connection import SnapMotionError

        controller, _conn = self._make_controller({})

        def _raise(axis):
            raise SnapMotionError(0, "switch not reached")

        monkeypatch.setattr(controller.homing, "home_axis", _raise)
        resynced = []
        monkeypatch.setattr(
            controller.gcode, "sync_position_from_hardware", lambda: resynced.append(True)
        )
        with pytest.raises(SnapMotionError):
            controller.home_axis(X_AXIS)
        assert resynced == [True]

    def test_home_resyncs_gcode_position(self, monkeypatch):
        controller, _conn = self._make_controller({})
        from laguna.robot.macron.homing import HomingResult

        monkeypatch.setattr(
            controller.homing, "home_all", lambda: HomingResult(success=True, axis_results={})
        )
        resynced = []
        monkeypatch.setattr(
            controller.gcode, "sync_position_from_hardware", lambda: resynced.append(True)
        )
        controller.home()
        assert resynced == [True]

    def test_enable_turns_on_every_motor_and_never_sends_ena(self):
        # ENA on a responder-node axis crashes the controller — confirmed on
        # hardware 2026-08-02 and reproduced from a bare tio terminal. The
        # DSM program enables the axes at power-up, so MTR alone is enough.
        responses = {f"A{i} MTR 1": "1" for i in (1, 2, 5, 6)}
        controller, conn = self._make_controller(responses)
        controller.enable()
        assert conn.sent == ["A1 MTR 1", "A2 MTR 1", "A5 MTR 1", "A6 MTR 1"]
        assert not any("ENA" in c for c in conn.sent)

    def test_disable_turns_off_every_motor_and_never_sends_ena(self):
        responses = {f"A{i} MTR 0": "0" for i in (1, 2, 5, 6)}
        controller, conn = self._make_controller(responses)
        controller.disable()
        assert conn.sent == ["A1 MTR 0", "A2 MTR 0", "A5 MTR 0", "A6 MTR 0"]
        assert not any("ENA" in c for c in conn.sent)

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


class TestAxisHandles:
    """Per-axis handles (plan steps 4 + 6, from 5c170d3). lab.gantry.y
    exposes the same commands as self.cmd, bound to one axis."""

    def _make_controller(self, responses=None, safe_mode=True):
        conn = FakeSnapConnection(responses or {})
        return GantryController(connection=conn, safe_mode=safe_mode), conn

    def test_dynamic_attribute_per_configured_axis(self):
        controller, _ = self._make_controller()
        assert controller.x.name == "X"
        assert controller.theta.index == 6

    def test_axis_lookup_by_name(self):
        controller, _ = self._make_controller()
        assert controller.axis("Z").index == 5

    def test_unknown_axis_name_raises_listing_configured_axes(self):
        controller, _ = self._make_controller()
        with pytest.raises(ValueError, match="No axis named"):
            controller.axis("W")

    def test_handle_delegates_to_the_right_axis_prefix(self):
        controller, conn = self._make_controller({"A2 SPD 5": "5"})
        controller.y.set_speed(5)
        assert conn.sent == ["A2 SPD 5"]

    def test_motion_is_blocked_by_safe_mode_before_reaching_the_wire(self):
        """RS232/Ethernet transports have no gate of their own, so this
        client-side check is the only thing between a bare
        lab.gantry.y.move_to() and the wire on those transports."""
        controller, conn = self._make_controller(safe_mode=True)
        from laguna.robot.macron.connection import SnapMotionError
        with pytest.raises(SnapMotionError, match="safe_mode"):
            controller.y.begin_move_to(10)
        assert conn.sent == []

    def test_stopping_is_never_gated(self):
        controller, conn = self._make_controller({"A2 STP": "0"}, safe_mode=True)
        controller.y.stop()
        assert conn.sent == ["A2 STP"]

    def test_enable_disable_send_mtr_only_never_ena(self):
        controller, conn = self._make_controller({"A2 MTR 1": "1", "A2 MTR 0": "0"})
        controller.y.enable()
        controller.y.disable()
        assert conn.sent == ["A2 MTR 1", "A2 MTR 0"]
        assert not any("ENA" in c for c in conn.sent)

    def test_axis_without_a_brake_raises(self):
        controller, _ = self._make_controller()
        with pytest.raises(ValueError, match="no brake"):
            controller.x.engage_brake()

    def test_reads_home_and_limit_switches_using_default_io_map(self):
        controller, conn = self._make_controller({"INB 1": "1", "INB 2": "0"})
        assert controller.x.read_home_switch() is True
        assert controller.x.read_limit_switch() is False
        assert conn.sent == ["INB 1", "INB 2"]

    def test_theta_limit_switch_raises_not_implemented(self):
        controller, _ = self._make_controller()
        with pytest.raises(NotImplementedError):
            controller.theta.read_limit_switch()


class TestBrakeVerbs:
    """GantryController.engage_brake/disengage_brake accept a name, an Axis,
    or a handle — same effect as the handle method."""

    def _make_controller(self, responses=None):
        conn = FakeSnapConnection(responses or {})
        return GantryController(connection=conn), conn

    @pytest.mark.parametrize("axis_ref", ["Y", Y_AXIS])
    def test_accepts_name_or_axis_object(self, axis_ref):
        controller, conn = self._make_controller({"SOB 4 0": "0"})
        controller.engage_brake(axis_ref)
        assert conn.sent == ["SOB 4 0"]

    def test_accepts_a_handle(self):
        controller, conn = self._make_controller({"SOB 4 1": "0"})
        controller.disengage_brake(controller.y)
        assert conn.sent == ["SOB 4 1"]

    def test_rejects_a_nonsense_reference(self):
        controller, _ = self._make_controller()
        with pytest.raises(TypeError):
            controller.engage_brake(42)


class TestHomeAndLimitSwitchVerbs:
    """GantryController.read_home_switch/read_limit_switch accept a name,
    an Axis, or a handle — same effect as the handle method."""

    def _make_controller(self, responses=None):
        conn = FakeSnapConnection(responses or {})
        return GantryController(connection=conn), conn

    @pytest.mark.parametrize("axis_ref", ["X", X_AXIS])
    def test_read_home_switch_accepts_name_or_axis_object(self, axis_ref):
        controller, conn = self._make_controller({"INB 1": "1"})
        assert controller.read_home_switch(axis_ref) is True
        assert conn.sent == ["INB 1"]

    def test_read_limit_switch_accepts_a_handle(self):
        controller, conn = self._make_controller({"INB 4": "0"})
        assert controller.read_limit_switch(controller.y) is False
        assert conn.sent == ["INB 4"]


class TestSetPosition:
    """set_position() declares where the gantry already is (ACP write) —
    the interim way to re-reference it while homing is disabled. Commands
    no motion."""

    def _make_controller(self, responses=None, mm_per_unit=15.0):
        conn = FakeSnapConnection(responses or {})
        return GantryController(connection=conn, mm_per_unit=mm_per_unit), conn

    def test_keyword_form_writes_only_the_given_axes(self):
        responses = {
            "A1 ACP 10": "10",
            # set_position() resyncs gcode's _current_pos afterward — see
            # sync_position_from_hardware.
            "A1 ACP": "10", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
        }
        controller, conn = self._make_controller(responses)
        assert controller.set_position(X=150.0) is True
        assert conn.sent == ["A1 ACP 10", "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP"]

    def test_vector_form_writes_every_axis(self):
        responses = {
            "A1 ACP 1": "1", "A2 ACP 2": "2", "A5 ACP 3": "3", "A6 ACP 4": "4",
            # set_position() resyncs gcode's _current_pos afterward:
            "A1 ACP": "1", "A2 ACP": "2", "A5 ACP": "3", "A6 ACP": "4",
        }
        controller, conn = self._make_controller(responses)
        controller.set_position([15.0, 30.0, 45.0, 4.0])   # Theta unconverted
        assert conn.sent == [
            "A1 ACP 1", "A2 ACP 2", "A5 ACP 3", "A6 ACP 4",
            "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP",  # resync
        ]

    def test_commands_no_motion(self):
        responses = {
            "A1 ACP 10": "10",
            "A1 ACP": "10", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
        }
        controller, conn = self._make_controller(responses)
        controller.set_position(X=150.0)
        assert not any(m in c for c in conn.sent for m in ("BMT", "BMB", "MVT", "JOG"))

    def test_vector_length_mismatch_raises(self):
        controller, _ = self._make_controller()
        with pytest.raises(ValueError):
            controller.set_position([1.0, 2.0])

    def test_no_arguments_raises(self):
        controller, _ = self._make_controller()
        with pytest.raises(ValueError):
            controller.set_position()


class TestSoftStop:
    """soft_stop() decelerates on each axis's own ramp (BST) and leaves
    brakes and motors alone — the gentle counterpart to stop()'s hard
    zero-decel abort."""

    def _make_controller(self, responses=None):
        conn = FakeSnapConnection(responses or {})
        return GantryController(connection=conn), conn

    def test_sends_bst_to_every_axis(self):
        responses = {f"A{i} BST": "0" for i in (1, 2, 5, 6)}
        # soft_stop() resyncs gcode's _current_pos afterward (best-effort,
        # never raises) — see sync_position_from_hardware.
        responses.update({f"A{i} ACP": "0" for i in (1, 2, 5, 6)})
        controller, conn = self._make_controller(responses)
        controller.soft_stop()
        assert conn.sent == [
            "A1 BST", "A2 BST", "A5 BST", "A6 BST",
            "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP",
        ]

    def test_leaves_brakes_and_motors_alone(self):
        responses = {f"A{i} BST": "0" for i in (1, 2, 5, 6)}
        responses.update({f"A{i} ACP": "0" for i in (1, 2, 5, 6)})
        controller, conn = self._make_controller(responses)
        controller.soft_stop()
        assert not any(("SOB" in c or "MTR" in c) for c in conn.sent)

    def test_one_failing_axis_does_not_stop_the_others(self):
        controller, conn = self._make_controller({})  # every command unscripted
        controller.soft_stop()                        # must not raise
        assert conn.sent[:4] == ["A1 BST", "A2 BST", "A5 BST", "A6 BST"]
        # The resync attempt afterward also fails (unscripted A1 ACP), but
        # is caught and logged, not raised — see soft_stop()'s try/except.
        assert conn.sent[4:] == ["A1 ACP"]


class TestPositionPersistence:
    """position_checkpoint_file: last-known axis positions are written at
    the end of move_to()/set_position()/stop()/soft_stop()/home()/
    home_axis()/locate_limit_switch(), so a power cycle (which wipes the
    PLC's ACP registers) can be recovered from via restore_last_position()
    without a fresh home() run. Off by default (position_checkpoint_file
    is None), and every other test class in this file constructs its
    controller(s) without it — see the passing exact conn.sent== assertions
    elsewhere, which would break if persistence sent wire commands
    unconditionally."""

    def _make_controller(self, tmp_path, responses=None, connect=True):
        path = str(tmp_path / "gantry_position.json")
        conn = FakeSnapConnection(responses or {})
        controller = GantryController(connection=conn, position_checkpoint_file=path)
        if connect:
            controller.connect()
        return controller, conn, path

    def test_disabled_by_default(self):
        conn = FakeSnapConnection({})
        controller = GantryController(connection=conn)  # no position_checkpoint_file
        assert controller._position_store is None

    def test_stop_persists_the_live_position(self, tmp_path):
        responses = {f"A{i} BST": "0" for i in (1, 2, 5, 6)}
        responses.update({"SOB 4 0": "0", "SOB 5 0": "0"})
        responses.update({"A1 ACP": "10", "A2 ACP": "20", "A5 ACP": "30", "A6 ACP": "40"})
        controller, _conn, path = self._make_controller(tmp_path, responses)
        controller.stop()

        data = GantryPositionStore(path).load()
        assert data is not None
        assert data["positions"] == {"X": 10.0, "Y": 20.0, "Z": 30.0, "Theta": 40.0}

    def test_soft_stop_persists_the_live_position(self, tmp_path):
        responses = {f"A{i} BST": "0" for i in (1, 2, 5, 6)}
        responses.update({"A1 ACP": "1", "A2 ACP": "2", "A5 ACP": "3", "A6 ACP": "4"})
        controller, _conn, path = self._make_controller(tmp_path, responses)
        controller.soft_stop()

        data = GantryPositionStore(path).load()
        assert data["positions"] == {"X": 1.0, "Y": 2.0, "Z": 3.0, "Theta": 4.0}

    def test_move_to_persists_all_axis_positions_not_just_the_moved_one(self, tmp_path):
        # Theta-only move (see TestMoveTo.test_theta_only_keyword_move...):
        # persistence still reads and saves every configured axis, not just
        # the one this call moved.
        responses = {
            "A6 ACP": "5", "A6 BMT 90": "0", "A6 MIF": "1",
            "A1 ACP": "1", "A2 ACP": "2", "A5 ACP": "3",
        }
        controller, _conn, path = self._make_controller(tmp_path, responses)
        assert controller.move_to(Theta=90.0) is True

        data = GantryPositionStore(path).load()
        assert data["positions"] == {"X": 1.0, "Y": 2.0, "Z": 3.0, "Theta": 5.0}

    def test_set_position_persists(self, tmp_path):
        responses = {
            "A1 ACP 10": "10",
            "A1 ACP": "10", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
        }
        controller, _conn, path = self._make_controller(tmp_path, responses)
        assert controller.set_position(X=10.0) is True

        data = GantryPositionStore(path).load()
        assert data["positions"] == {"X": 10.0, "Y": 0.0, "Z": 0.0, "Theta": 0.0}

    _CONNECT_ACP_RESPONSES = {"A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0"}

    def test_home_axis_persists_the_live_position(self, tmp_path, monkeypatch):
        controller, _conn, path = self._make_controller(tmp_path, dict(self._CONNECT_ACP_RESPONSES))
        monkeypatch.setattr(controller.homing, "home_axis", lambda axis: 5.0)
        monkeypatch.setattr(controller.gcode, "sync_position_from_hardware", lambda: None)
        monkeypatch.setattr(
            controller, "cmd",
            type("_C", (), {"get_actual_position": staticmethod(lambda axis: 7.0)})(),
        )
        controller.home_axis(X_AXIS)

        data = GantryPositionStore(path).load()
        assert data is not None
        assert data["positions"] == {"X": 7.0, "Y": 7.0, "Z": 7.0, "Theta": 7.0}

    def test_home_persists_the_live_position(self, tmp_path, monkeypatch):
        from laguna.robot.macron.homing import HomingResult

        controller, _conn, path = self._make_controller(tmp_path, dict(self._CONNECT_ACP_RESPONSES))
        monkeypatch.setattr(
            controller.homing, "home_all", lambda: HomingResult(success=True, axis_results={})
        )
        monkeypatch.setattr(controller.gcode, "sync_position_from_hardware", lambda: None)
        monkeypatch.setattr(
            controller, "cmd",
            type("_C", (), {"get_actual_position": staticmethod(lambda axis: 9.0)})(),
        )
        controller.home()

        data = GantryPositionStore(path).load()
        assert data is not None
        assert data["positions"] == {"X": 9.0, "Y": 9.0, "Z": 9.0, "Theta": 9.0}

    def test_locate_limit_switch_persists_the_live_position(self, tmp_path, monkeypatch):
        controller, _conn, path = self._make_controller(tmp_path, dict(self._CONNECT_ACP_RESPONSES))
        monkeypatch.setattr(controller.homing, "locate_limit_switch", lambda axis: 42.0)
        monkeypatch.setattr(controller.gcode, "sync_position_from_hardware", lambda: None)
        monkeypatch.setattr(
            controller, "cmd",
            type("_C", (), {"get_actual_position": staticmethod(lambda axis: 3.0)})(),
        )
        controller.locate_limit_switch(X_AXIS)

        data = GantryPositionStore(path).load()
        assert data is not None
        assert data["positions"] == {"X": 3.0, "Y": 3.0, "Z": 3.0, "Theta": 3.0}

    def test_not_persisted_when_disconnected(self, tmp_path):
        controller, _conn, path = self._make_controller(tmp_path, responses={}, connect=False)
        controller.stop()  # hits unscripted commands, but safety verbs never raise
        assert GantryPositionStore(path).load() is None

    def test_restore_last_position_applies_the_saved_positions(self, tmp_path):
        path = str(tmp_path / "gantry_position.json")
        GantryPositionStore(path).save({"X": 11.0, "Y": 22.0, "Z": 33.0, "Theta": 44.0})
        responses = {
            "A1 ACP 11": "11", "A2 ACP 22": "22", "A5 ACP 33": "33", "A6 ACP 44": "44",
            # set_position()'s own end-of-call persistence re-reads every axis,
            # and connect() now resyncs _current_pos from the same reads:
            "A1 ACP": "11", "A2 ACP": "22", "A5 ACP": "33", "A6 ACP": "44",
        }
        conn = FakeSnapConnection(responses)
        controller = GantryController(connection=conn, position_checkpoint_file=path)
        controller.connect()
        conn.sent.clear()

        assert controller.restore_last_position() is True
        assert conn.sent[:4] == ["A1 ACP 11", "A2 ACP 22", "A5 ACP 33", "A6 ACP 44"]

    def test_restore_last_position_false_when_no_store_configured(self):
        conn = FakeSnapConnection({})
        controller = GantryController(connection=conn)  # no position_checkpoint_file
        controller.connect()
        assert controller.restore_last_position() is False

    def test_restore_last_position_false_when_no_checkpoint_exists_yet(self, tmp_path):
        controller, _conn, _path = self._make_controller(tmp_path, responses={})
        assert controller.restore_last_position() is False

    def test_restore_last_position_never_sends_ena(self, tmp_path):
        path = str(tmp_path / "gantry_position.json")
        GantryPositionStore(path).save({"X": 1.0, "Y": 2.0, "Z": 3.0, "Theta": 4.0})
        responses = {
            "A1 ACP 1": "1", "A2 ACP 2": "2", "A5 ACP 3": "3", "A6 ACP 4": "4",
            "A1 ACP": "1", "A2 ACP": "2", "A5 ACP": "3", "A6 ACP": "4",
        }
        conn = FakeSnapConnection(responses)
        controller = GantryController(connection=conn, position_checkpoint_file=path)
        controller.connect()
        controller.restore_last_position()
        assert not any("ENA" in c for c in conn.sent)


class TestConnectBrakeRelease:
    """connect() turns Y/Z's motors on and releases their brakes when motion
    is already permitted (plan step 10, from e5f8a95). Once the motor holds
    torque an engaged brake serves no purpose and is a hazard — the next
    move would stall against it, which is what corrupted Y's encoder
    feedback on 2026-07-30."""

    RESPONSES = {
        "A2 MTR 1": "1", "SOB 4 1": "0",   # Y: motor on, brake released
        "A5 MTR 1": "1", "SOB 5 1": "0",   # Z: motor on, brake released
        # connect()'s own _current_pos resync (sync_position_from_hardware):
        "A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
    }

    def _make_controller(self, safe_mode, responses=None):
        conn = FakeSnapConnection(responses if responses is not None else dict(self.RESPONSES))
        return GantryController(connection=conn, safe_mode=safe_mode), conn

    def test_releases_brakes_when_motion_is_permitted(self):
        controller, conn = self._make_controller(safe_mode=False)
        assert controller.connect() is True
        assert conn.sent == [
            "A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP",  # _current_pos resync
            "A2 MTR 1", "SOB 4 1", "A5 MTR 1", "SOB 5 1",
        ]

    def test_motor_on_strictly_before_brake_release(self):
        """Z's brake is fail-safe/spring-engaged: releasing it before the
        motor holds torque could drop a loaded Z under gravity."""
        controller, conn = self._make_controller(safe_mode=False)
        controller.connect()
        assert conn.sent.index("A5 MTR 1") < conn.sent.index("SOB 5 1")

    def test_never_sends_ena(self):
        """The source commit sent ENA here, which would crash the responder
        node on every connect with motion enabled."""
        controller, conn = self._make_controller(safe_mode=False)
        controller.connect()
        assert not any("ENA" in c for c in conn.sent)

    def test_does_nothing_while_safe_mode_is_on(self):
        """MTR and SOB are both output-setting commands, which safe_mode's
        guarantee has to cover."""
        controller, conn = self._make_controller(safe_mode=True)
        assert controller.connect() is True
        assert conn.sent == ["A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP"]  # _current_pos resync only

    def test_only_touches_braked_axes(self):
        controller, conn = self._make_controller(safe_mode=False)
        controller.connect()
        assert not any(c.startswith(("A1 ", "A6 ")) and "ACP" not in c for c in conn.sent)

    def test_a_failing_axis_does_not_block_the_other(self):
        """A faulted axis is logged and skipped, not allowed to abort the
        whole connect — the other axis still gets its brake released."""
        from laguna.robot.macron.connection import SnapMotionError
        responses = {
            "A2 MTR 1": SnapMotionError(70),   # Y's motor won't come on
            "A5 MTR 1": "1", "SOB 5 1": "0",
            "A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
        }
        controller, conn = self._make_controller(safe_mode=False, responses=responses)
        assert controller.connect() is True             # must not raise
        assert "SOB 4 1" not in conn.sent               # Y's brake stayed engaged...
        assert "SOB 5 1" in conn.sent                   # ...but Z still got released


class TestConnectSoftLimits:
    """connect() writes configured NLT/PLT once motion is permitted — see
    GantryController._apply_soft_limits(). NLT/PLT writes are blocked by
    the safe_mode allowlist exactly like any other motion-adjacent write,
    so this only fires once safe_mode=False, alongside the existing
    brake-release step."""

    BRAKE_RESPONSES = {
        "A2 MTR 1": "1", "SOB 4 1": "0", "A5 MTR 1": "1", "SOB 5 1": "0",
        "A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
    }

    def _make_controller(self, safe_mode, soft_limits, responses=None):
        merged = dict(self.BRAKE_RESPONSES)
        merged.update(responses or {})
        conn = FakeSnapConnection(merged)
        return (
            GantryController(connection=conn, safe_mode=safe_mode, soft_limits=soft_limits),
            conn,
        )

    def test_writes_and_validates_configured_limits(self):
        controller, conn = self._make_controller(
            False, {"X": (-5, 495)},
            {"A1 NLT -5": "-5", "A1 PLT 495": "495", "A1 NLT": "-5", "A1 PLT": "495"},
        )
        assert controller.connect() is True
        assert "A1 NLT -5" in conn.sent
        assert "A1 PLT 495" in conn.sent
        # validate_soft_limits() read-back happens after the writes
        assert conn.sent.index("A1 PLT 495") < conn.sent.index("A1 NLT")

    def test_only_configured_axes_are_touched(self):
        controller, conn = self._make_controller(
            False, {"X": (-5, 495)},
            {"A1 NLT -5": "-5", "A1 PLT 495": "495", "A1 NLT": "-5", "A1 PLT": "495"},
        )
        controller.connect()
        assert not any(
            c.startswith(("A2 NLT", "A2 PLT", "A5 NLT", "A5 PLT", "A6 NLT", "A6 PLT"))
            for c in conn.sent
        )

    def test_a_single_bound_can_be_set_alone(self):
        controller, conn = self._make_controller(
            False, {"X": (-5, None)}, {"A1 NLT -5": "-5", "A1 NLT": "-5", "A1 PLT": "0"},
        )
        controller.connect()
        assert "A1 NLT -5" in conn.sent
        assert not any(c.startswith("A1 PLT ") for c in conn.sent)  # no PLT write, only its read-back

    def test_does_nothing_while_safe_mode_is_on(self):
        controller, conn = self._make_controller(True, {"X": (-5, 495)})
        assert controller.connect() is True
        assert not any("NLT" in c or "PLT" in c for c in conn.sent)

    def test_no_configured_limits_is_a_no_op(self):
        controller, conn = self._make_controller(False, None)
        assert controller.connect() is True
        assert not any("NLT" in c or "PLT" in c for c in conn.sent)

    def test_a_failing_axis_does_not_block_connect_or_the_other_axis(self):
        from laguna.robot.macron.connection import SnapMotionError

        controller, conn = self._make_controller(
            False, {"X": (-5, None), "Y": (-1, 400)},
            {
                "A1 NLT -5": SnapMotionError(0, "boom"),
                "A2 NLT -1": "-1", "A2 PLT 400": "400", "A2 NLT": "-1", "A2 PLT": "400",
            },
        )
        assert controller.connect() is True  # must not raise
        assert "A2 NLT -1" in conn.sent  # Y still got its limits despite X failing

    def test_validation_failure_does_not_raise(self):
        # A garbage read-back makes validate_soft_limits() raise internally
        # — _apply_soft_limits() must swallow that, not let it fail connect().
        controller, conn = self._make_controller(
            False, {"X": (-5, 495)},
            {
                "A1 NLT -5": "-5", "A1 PLT 495": "495",
                "A1 NLT": "-822536056", "A1 PLT": "495",
            },
        )
        assert controller.connect() is True


class TestSetSafeMode:
    """set_safe_mode() flips the flag and syncs Y/Z's brakes to match."""

    def _make_controller(self, safe_mode=True, responses=None):
        conn = FakeSnapConnection(responses or {
            "A2 MTR 1": "1", "SOB 4 1": "0", "A5 MTR 1": "1", "SOB 5 1": "0",
            "SOB 4 0": "0", "SOB 5 0": "0",
            # connect()'s own _current_pos resync (sync_position_from_hardware):
            "A1 ACP": "0", "A2 ACP": "0", "A5 ACP": "0", "A6 ACP": "0",
        })
        return GantryController(connection=conn, safe_mode=safe_mode), conn

    def test_disabling_releases_brakes(self):
        controller, conn = self._make_controller(safe_mode=True)
        controller.connect()
        conn.sent.clear()
        assert controller.set_safe_mode(False) is True
        assert conn.sent == ["A2 MTR 1", "SOB 4 1", "A5 MTR 1", "SOB 5 1"]

    def test_enabling_engages_brakes_and_leaves_motors_on(self):
        controller, conn = self._make_controller(safe_mode=False)
        controller.connect()
        conn.sent.clear()
        assert controller.set_safe_mode(True) is True
        assert conn.sent == ["SOB 4 0", "SOB 5 0"]
        assert not any("MTR" in c for c in conn.sent)

    def test_flag_propagates_to_the_transport_gate(self):
        controller, conn = self._make_controller(safe_mode=True)
        conn.safe_mode = True
        controller.set_safe_mode(False)
        assert controller._safe_mode is False
        assert conn.safe_mode is False

    def test_no_brake_traffic_when_not_connected(self):
        """connect() applies the release side itself, with whatever
        safe_mode is set to by then."""
        controller, conn = self._make_controller(safe_mode=True)
        controller.set_safe_mode(False)
        assert conn.sent == []

    def test_axis_handles_see_the_new_flag_immediately(self):
        """The handles' gate closes over self._safe_mode, so flipping it
        must take effect without rebuilding them."""
        from laguna.robot.macron.connection import SnapMotionError
        controller, _ = self._make_controller(safe_mode=True)
        with pytest.raises(SnapMotionError):
            controller.y.begin_move_to(1)
        controller.set_safe_mode(False)
        controller.y._check_motion_allowed("test")   # no longer raises
