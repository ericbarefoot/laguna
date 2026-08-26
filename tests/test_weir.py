"""Tests for SaflWeirController — construct, disconnected safety verbs, and
the MQTT request/reply command path.

The weir is now an MQTT client of the confluence node on red.lab: status
streams in async on a topic, and motion/config commands block for a
correlated reply (laguna.mqtt.request_reply) rather than a synchronous
serial round-trip — see FakeMqttSubscriber in mqtt_fixtures.py for the
double used here in place of a real broker.
"""

import math

from laguna.weir import SaflWeirController

from mqtt_fixtures import FakeMqttSubscriber


def _make_weir(**config):
    mqtt = FakeMqttSubscriber()
    controller = SaflWeirController(
        {
            "topic_status": "node/weir",
            "topic_commands": "node/weir/commands",
            "topic_replies": "node/weir/replies",
            **config,
        },
        mqtt,
    )
    return controller, mqtt


def _accept_all_replies(mqtt, **fields):
    """Auto-reply to every command on node/weir/replies, echoing request_id."""
    mqtt.auto_reply_on(
        "node/weir/commands",
        "node/weir/replies",
        lambda payload: {"request_id": payload["request_id"], **fields},
    )


class TestSaflWeirControllerDisconnected:
    def test_construct(self):
        controller, _ = _make_weir()
        assert controller.subsystem_name == "weir"

    def test_pause_never_raises(self):
        controller, _ = _make_weir()
        note = controller.pause()
        assert note is not None

    def test_resume_never_raises(self):
        controller, _ = _make_weir()
        controller.resume()  # nothing to restore; must not raise

    def test_stop_never_raises(self):
        controller, _ = _make_weir()
        note = controller.stop()
        assert note is not None

    def test_estop_never_raises(self):
        controller, _ = _make_weir()
        note = controller.estop()
        assert note is not None

    def test_disconnect_is_safe_when_never_connected(self):
        controller, _ = _make_weir()
        controller.disconnect()  # must not raise

    def test_get_status_reports_disconnected(self):
        controller, _ = _make_weir()
        status = controller.get_status()
        assert status == {"is_connected": False, "elevation_mm": None}


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 3.0


class TestSaflWeirControllerConnected:
    def test_connect_subscribes_to_status_and_replies(self):
        controller, mqtt = _make_weir()
        assert controller.connect() is True
        assert "node/weir" in mqtt._topics
        assert "node/weir/replies" in mqtt._topics

    def test_go_to_elevation_accepted(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=True)

        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())
        assert controller.go_to_elevation(42.0) is True
        assert event_log.rows == [(3.0, "weir", "go_to_elevation", "ok", "target_mm=42.00")]

    def test_go_to_elevation_rejected_by_hardware(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=False)
        assert controller.go_to_elevation(42.0) is False

    def test_go_to_elevation_times_out_without_a_reply(self):
        """No auto-reply configured — the confluence node never answers."""
        controller, mqtt = _make_weir(command_timeout_s=0.05)
        controller.connect()
        assert controller.go_to_elevation(42.0) is False

    def test_get_elevation_is_nan_before_any_status_message(self):
        controller, mqtt = _make_weir()
        controller.connect()
        assert math.isnan(controller.get_elevation())

    def test_get_elevation_reads_latest_status_message(self):
        controller, mqtt = _make_weir()
        controller.connect()
        mqtt.push("node/weir", {"elevation_mm": 123.4, "is_moving": False})
        assert controller.get_elevation() == 123.4

    def test_wait_for_move_returns_once_status_reports_not_moving(self):
        controller, mqtt = _make_weir()
        controller.connect()
        mqtt.push("node/weir", {"elevation_mm": 10.0, "is_moving": False})
        controller.wait_for_move(timeout=1.0)  # must not block/raise

    def test_home_succeeds_on_ack(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, ok=True)
        assert controller.home() is True

    def test_clear_faults_succeeds_on_ack(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, ok=True)
        assert controller.clear_faults() is True

    def test_pause_stop_estop_publish_a_stop_command(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, ok=True)
        assert controller.pause() is None
        assert controller.stop() is None
        assert controller.estop() is None
        commands = [payload["command"] for _topic, payload in mqtt.published]
        assert commands == ["stop", "stop", "stop"]


class TestSaflWeirControllerSimulated:
    """simulated: True skips MQTT entirely — commands succeed (and log
    normally), readings come back NaN rather than a fabricated elevation."""

    def test_connect_succeeds_without_a_broker(self):
        controller, mqtt = _make_weir(simulated=True)
        assert controller.connect() is True
        assert controller._is_connected is True
        assert mqtt.connected_called == 0

    def test_go_to_elevation_succeeds_and_logs(self):
        controller, _ = _make_weir(simulated=True)
        controller.connect()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.go_to_elevation(300.0) is True
        assert event_log.rows == [(3.0, "weir", "go_to_elevation", "ok", "target_mm=300.00")]

    def test_get_elevation_is_nan_not_a_fabricated_reading(self):
        controller, _ = _make_weir(simulated=True)
        controller.connect()
        controller.go_to_elevation(300.0)
        assert math.isnan(controller.get_elevation())

    def test_get_status_elevation_is_nan(self):
        controller, _ = _make_weir(simulated=True)
        controller.connect()
        status = controller.get_status()
        assert status["is_connected"] is True
        assert math.isnan(status["elevation_mm"])

    def test_home_succeeds_instantly(self):
        controller, _ = _make_weir(simulated=True)
        controller.connect()
        assert controller.home() is True

    def test_safety_verbs_still_work(self):
        controller, _ = _make_weir(simulated=True)
        controller.connect()
        assert controller.pause() is None
        assert controller.stop() is None
        assert controller.estop() is None
