"""Tests for SaflFlowController — construct, disconnected safety verbs, and
the MQTT request/reply command path across its two channels (Fuji VFD +
shared ClearCore valve axis).

See FakeMqttSubscriber in mqtt_fixtures.py for the double used here in
place of a real broker.
"""

import math

from laguna.config import Config
from laguna.flow import SaflFlowController

from mqtt_fixtures import FakeMqttSubscriber


def _make_flow(**config):
    mqtt = FakeMqttSubscriber()
    controller = SaflFlowController(
        {
            "vfd_topic_status": "node/vfd",
            "vfd_topic_commands": "node/vfd/commands",
            "vfd_topic_replies": "node/vfd/replies",
            "valve_topic_status": "node/valve",
            "valve_topic_commands": "node/valve/commands",
            "valve_topic_replies": "node/valve/replies",
            **config,
        },
        mqtt,
    )
    return controller, mqtt


def _accept_all_vfd_replies(mqtt, **fields):
    mqtt.auto_reply_on(
        "node/vfd/commands",
        "node/vfd/replies",
        lambda payload: {"request_id": payload["request_id"], **fields},
    )


def _accept_all_valve_replies(mqtt, **fields):
    mqtt.auto_reply_on(
        "node/valve/commands",
        "node/valve/replies",
        lambda payload: {"request_id": payload["request_id"], **fields},
    )


class TestSaflFlowControllerDisconnected:
    def test_construct(self):
        controller, _ = _make_flow()
        assert controller.subsystem_name == "flow"

    def test_pause_never_raises(self):
        controller, _ = _make_flow()
        note = controller.pause()
        assert note is not None

    def test_resume_never_raises(self):
        controller, _ = _make_flow()
        note = controller.resume()
        assert note is not None

    def test_stop_never_raises(self):
        controller, _ = _make_flow()
        note = controller.stop()
        assert note is not None

    def test_estop_never_raises(self):
        controller, _ = _make_flow()
        note = controller.estop()
        assert note is not None

    def test_disconnect_is_safe_when_never_connected(self):
        controller, _ = _make_flow()
        controller.disconnect()  # must not raise

    def test_get_status_reports_disconnected(self):
        controller, _ = _make_flow()
        status = controller.get_status()
        assert status["is_connected"] is False


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 7.0


class TestSaflFlowControllerConnected:
    def test_connect_subscribes_to_all_four_topics(self):
        controller, mqtt = _make_flow()
        assert controller.connect() is True
        for topic in ("node/vfd", "node/vfd/replies", "node/valve", "node/valve/replies"):
            assert topic in mqtt._topics

    def test_set_flowrate_acked_updates_cached_setpoint_and_logs(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.set_flowrate(12.5) is True
        assert controller.get_flowrate() == 12.5
        assert event_log.rows == [(7.0, "flow", "set_flowrate", "ok", "flowrate_lpm=12.50")]

    def test_set_flowrate_not_acked_leaves_cached_setpoint_unchanged(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=False)

        assert controller.set_flowrate(12.5) is False
        assert controller.get_flowrate() == 0.0

    def test_set_flowrate_times_out_without_a_reply(self):
        controller, mqtt = _make_flow(command_timeout_s=0.05)
        controller.connect()
        assert controller.set_flowrate(12.5) is False

    def test_start_and_stop_publish_to_the_vfd_channel(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        assert controller.start() is True
        assert controller.stop() is None
        commands = [payload["command"] for topic, payload in mqtt.published if topic == "node/vfd/commands"]
        assert commands == ["start_motor", "stop_motor"]

    def test_qin_and_qaux_publish_to_the_valve_channel_and_log(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_valve_replies(mqtt, ok=True)
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        controller.qin = True
        controller.qaux = False

        assert controller.qin is True
        assert controller.qaux is False
        assert event_log.rows == [
            (7.0, "flow", "qin", "ok", "state=True"),
            (7.0, "flow", "qaux", "ok", "state=False"),
        ]
        commands = [payload["command"] for topic, payload in mqtt.published if topic == "node/valve/commands"]
        assert commands == ["set_io", "set_io"]

    def test_qin_setter_raises_without_a_reply(self):
        """A valve command that never gets acked must not be silently assumed
        to have happened — it raises rather than caching a state the
        hardware never confirmed."""
        controller, mqtt = _make_flow(command_timeout_s=0.05)
        controller.connect()
        try:
            controller.qin = True
            assert False, "expected RuntimeError"
        except RuntimeError:
            pass
        assert controller.qin is False

    def test_estop_closes_both_valves_and_stops_pump_independently(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        _accept_all_valve_replies(mqtt, ok=True)
        assert controller.estop() is None
        assert controller.qin is False
        assert controller.qaux is False

    def test_get_status_reads_latest_vfd_status_message(self):
        controller, mqtt = _make_flow()
        controller.connect()
        mqtt.push("node/vfd", {"state_message": "running", "e_stop": False, "setpoint": 42.0})
        status = controller.get_status()
        assert status["vfd_state"] == "running"
        assert status["vfd_estop"] is False
        assert status["vfd_setpoint_hz"] == 42.0


class TestSaflFlowControllerSimulated:
    """simulated: True skips MQTT entirely — commands succeed (and log
    normally), readings come back NaN/None rather than fabricated data."""

    def test_connect_succeeds_without_a_broker(self):
        controller, mqtt = _make_flow(simulated=True)
        assert controller.connect() is True
        assert controller._is_connected is True
        assert mqtt.connected_called == 0

    def test_set_flowrate_start_stop_all_succeed_and_log(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.set_flowrate(15.0) is True
        assert controller.start() is True
        assert controller.stop() is None

        actions = [row[2] for row in event_log.rows]
        assert actions == ["set_flowrate", "start", "stop"]

    def test_get_flowrate_echoes_the_commanded_setpoint_not_nan(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        controller.set_flowrate(22.5)
        assert controller.get_flowrate() == 22.5

    def test_get_status_vfd_fields_are_nan_or_none(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        status = controller.get_status()
        assert status["is_connected"] is True
        assert status["vfd_state"] is None
        assert status["vfd_estop"] is None
        assert math.isnan(status["vfd_setpoint_hz"])

    def test_qin_qaux_succeed_and_log(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        controller.qin = True
        assert controller.qin is True


class TestSaflFlowControllerFromConfig:
    """from_config() derives topics from mqtt.node_name — see
    config.py's comment above the "weir" section."""

    def test_topics_derived_from_node_name(self):
        config = Config(defaults={"flow": {}, "mqtt": {"node_name": "UCRS Confluence Node 1"}})
        controller = SaflFlowController.from_config(config)
        assert controller._vfd_status_topic == "UCRS Confluence Node 1/Fuji_Frenic_VFD"
        assert controller._vfd_commands_topic == "UCRS Confluence Node 1/Fuji_Frenic_VFD/commands"
        assert controller._valve_status_topic == "UCRS Confluence Node 1/flow_valve"
        assert controller._valve_commands_topic == "UCRS Confluence Node 1/flow_valve/commands"
