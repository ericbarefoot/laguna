"""Tests for SaflWeirController — construct, disconnected safety verbs, and
the MQTT request/reply command path.

The weir is now an MQTT client of the confluence node on red.lab: status
streams in async on a topic, and motion/config commands block for a
correlated reply (laguna.mqtt.request_reply) rather than a synchronous
serial round-trip — see FakeMqttSubscriber in mqtt_fixtures.py for the
double used here in place of a real broker.
"""

import math

import pytest

from laguna.config import Config
from laguna.weir import SaflWeirController

from mqtt_fixtures import FakeMqttSubscriber

import time


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

    def test_connect_fails_if_broker_handshake_never_completes(self):
        """Regression: connect() used to return True as soon as the async
        MQTT handshake was *started*, not once it actually completed."""
        controller, mqtt = _make_weir()
        mqtt.wait_until_connected = lambda timeout=5.0, poll_interval=0.05: False
        assert controller.connect() is False

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

    def test_get_velocity_falls_back_to_cached_setpoint_before_any_status(self):
        """No status message yet — falls back to the locally cached value,
        which itself starts NaN until set_velocity() is called."""
        controller, mqtt = _make_weir()
        controller.connect()
        assert math.isnan(controller.get_velocity())

    def test_get_velocity_reads_live_status_over_local_cache(self):
        """velocity_setpoint on the status topic reflects the ClearCore's
        real VelSetPoint register (see Teknic_ClearCore_funcs.py) and takes
        priority over whatever set_velocity() last cached locally — a stale
        local cache (e.g. from a previous connection) must not shadow a
        live hardware reading."""
        controller, mqtt = _make_weir()
        controller.connect()
        controller._velocity_mm_per_sec = 999.0  # stale local cache
        mqtt.push("node/weir", {"elevation_mm": 10.0, "velocity_setpoint": 5.0})
        assert controller.get_velocity() == 5.0

    def test_get_velocity_falls_back_when_status_omits_velocity_setpoint(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, ok=True)
        controller.set_velocity(12.5)
        mqtt.push("node/weir", {"elevation_mm": 10.0})  # no velocity_setpoint key
        assert controller.get_velocity() == 12.5

    def test_get_status_then_get_velocity_agree_on_one_status_message(self):
        """Regression: get_latest() used to drain the whole queue on every
        call, so whichever of get_status()/get_velocity() ran second would
        find the queue empty and fall back to a stale locally-cached value
        instead of seeing the same live message the first call just read —
        get_status() would report the real hardware setpoint while
        get_velocity() silently reported an unrelated older one."""
        controller, mqtt = _make_weir()
        controller.connect()
        controller._velocity_mm_per_sec = 5.0  # stale local cache
        mqtt.push("node/weir", {"elevation_mm": 10.0, "velocity_setpoint": 3.0})
        status = controller.get_status()
        assert status["motor"]["velocity_setpoint"] == 3.0
        assert controller.get_velocity() == 3.0

    def test_wait_for_move_times_out_if_elevation_never_reaches_target(self):
        """No fresh status ever reports an elevation near the target — e.g.
        the gate stalled short of it — so this must time out rather than
        return early on an is_moving flag (which is no longer consulted
        at all)."""
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=True)
        mqtt.push("node/weir", {"elevation_mm": 0.0, "is_moving": False})  # stale, pre-move
        controller.go_to_elevation(300.0)
        # no status ever arrives near 300.0 — gate never gets there
        start = time.monotonic()
        controller.wait_for_move(timeout=0.3)
        elapsed = time.monotonic() - start
        assert elapsed >= 0.3

    def test_wait_for_move_returns_once_elevation_reaches_target(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=True)
        controller.go_to_elevation(10.0)
        mqtt.push("node/weir", {"elevation_mm": 10.0, "is_moving": False})
        controller.wait_for_move(timeout=1.0)  # must not block/raise

    def test_wait_for_move_returns_within_position_tolerance(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=True)
        controller.go_to_elevation(300.0)
        mqtt.push("node/weir", {"elevation_mm": 300.3, "is_moving": True})  # within 0.5mm, is_moving ignored
        controller.wait_for_move(timeout=1.0)  # must not time out

    def test_wait_for_move_ignores_pre_move_backlog_but_honors_fresh_status(self):
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=True)
        mqtt.push("node/weir", {"elevation_mm": 0.0, "is_moving": False})  # stale, pre-move
        controller.go_to_elevation(300.0)  # drains the stale backlog on accept
        mqtt.push("node/weir", {"elevation_mm": 150.0, "is_moving": True})  # fresh, mid-move
        mqtt.push("node/weir", {"elevation_mm": 300.0, "is_moving": False})  # fresh, arrived
        controller.wait_for_move(timeout=1.0)  # must not time out

    def test_wait_for_move_raises_if_called_before_any_go_to_elevation(self):
        controller, mqtt = _make_weir()
        controller.connect()
        mqtt.push("node/weir", {"elevation_mm": 10.0, "is_moving": False})
        with pytest.raises(RuntimeError):
            controller.wait_for_move(timeout=1.0)

    def test_wait_for_move_sleeps_through_predicted_duration_before_polling(self):
        """When set_velocity() has established a known speed, the initial
        sleep should cover most of the predicted move time rather than
        polling from t=0 — this is what keeps a long move from spamming
        the status topic every SPARSE_POLL_INTERVAL_S for its whole
        duration."""
        controller, mqtt = _make_weir()
        controller.connect()
        _accept_all_replies(mqtt, accepted=True)
        mqtt.push("node/weir", {"elevation_mm": 0.0, "is_moving": False})
        controller.set_velocity(100.0)  # mm/s — predicted 10mm move takes 0.1s
        controller.go_to_elevation(10.0)
        mqtt.push("node/weir", {"elevation_mm": 10.0, "is_moving": False})
        start = time.monotonic()
        controller.wait_for_move(timeout=5.0)
        elapsed = time.monotonic() - start
        # predicted_s=0.1s * PREDICTED_SLEEP_FRACTION=0.85 ~= 0.085s initial
        # sleep, well under the 5.0s timeout — must not wait anywhere near it
        assert elapsed < 1.0

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

    def test_get_velocity_is_nan_not_a_fabricated_reading(self):
        controller, _ = _make_weir(simulated=True)
        controller.connect()
        controller.set_velocity(10.0)
        assert math.isnan(controller.get_velocity())

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


class TestSaflWeirControllerFromConfig:
    """from_config() derives topics from mqtt.node_name — see
    config.py's comment above the "weir" section."""

    def test_topics_derived_from_node_name(self):
        config = Config(defaults={"weir": {}, "mqtt": {"node_name": "UCRS Confluence Node 1"}})
        controller = SaflWeirController.from_config(config)
        assert controller._status_topic == "UCRS Confluence Node 1/weir"
        assert controller._commands_topic == "UCRS Confluence Node 1/weir/commands"
        assert controller._replies_topic == "UCRS Confluence Node 1/weir/replies"

    def test_explicit_topic_overrides_derived_default(self):
        config = Config(
            defaults={
                "weir": {"topic_status": "custom/status"},
                "mqtt": {"node_name": "UCRS Confluence Node 1"},
            }
        )
        controller = SaflWeirController.from_config(config)
        assert controller._status_topic == "custom/status"
        assert controller._commands_topic == "UCRS Confluence Node 1/weir/commands"
