"""Tests for MqttSubscriber.

All offline — no real broker involved. paho-mqtt is stubbed via sys.modules
injection so the test suite runs without paho installed. Each test documents
one behavioral assumption; failures on hardware reveal which assumption the
AL1342 or broker violated.
"""

import json
import sys
import time
from types import ModuleType
from unittest.mock import MagicMock, patch

import pytest

from laguna.mqtt import MqttSubscriber


# ---------------------------------------------------------------------------
# FakeClient — module-level paho.mqtt.client.Client replacement
# ---------------------------------------------------------------------------


class FakeClient:
    """Test double for paho.mqtt.client.Client.

    Supports all lifecycle hooks used by MqttSubscriber and exposes
    fire_* helpers to simulate incoming callbacks.
    """

    def __init__(self, client_id=""):
        self.client_id = client_id
        self.on_connect = None
        self.on_disconnect = None
        self.on_message = None
        self.subscriptions = []
        self.loop_started = False
        self.loop_stopped = False
        self.disconnected = False
        self.connected_host = None

    def reconnect_delay_set(self, min_delay, max_delay):
        pass

    def connect(self, host, port, keepalive=60):
        self.connected_host = host

    def subscribe(self, topic, qos=0):
        self.subscriptions.append(topic)

    def loop_start(self):
        self.loop_started = True

    def loop_stop(self):
        self.loop_stopped = True

    def disconnect(self):
        self.disconnected = True

    # --- test helpers ---

    def fire_connect(self, rc=0):
        """Simulate broker connection callback."""
        if self.on_connect:
            self.on_connect(self, None, {}, rc)

    def fire_disconnect(self, rc=0):
        """Simulate broker disconnect callback."""
        if self.on_disconnect:
            self.on_disconnect(self, None, rc)

    def fire_message(self, topic: str, payload: dict):
        """Simulate an incoming MQTT message with a JSON payload."""
        msg = MagicMock()
        msg.topic = topic
        msg.payload = json.dumps(payload).encode("utf-8")
        if self.on_message:
            self.on_message(self, None, msg)

    def fire_message_raw(self, topic: str, raw: bytes):
        """Simulate an incoming MQTT message with a raw (non-JSON) payload."""
        msg = MagicMock()
        msg.topic = topic
        msg.payload = raw
        if self.on_message:
            self.on_message(self, None, msg)


def _make_paho_stub():
    """Return a paho module stub tree with FakeClient installed as Client."""
    stub = ModuleType("paho")
    stub.mqtt = ModuleType("paho.mqtt")
    stub.mqtt.client = ModuleType("paho.mqtt.client")
    stub.mqtt.client.Client = FakeClient
    return stub


@pytest.fixture
def paho_stub():
    """Inject the paho stub into sys.modules for the duration of the test."""
    stub = _make_paho_stub()
    with patch.dict(sys.modules, {
        "paho": stub,
        "paho.mqtt": stub.mqtt,
        "paho.mqtt.client": stub.mqtt.client,
    }):
        yield stub


def _sub(config_overrides=None, **kwargs):
    config = {
        "broker_host": "red.lab",
        "broker_port": 1883,
        "client_id": "test",
        "topics": [],
    }
    if config_overrides:
        config.update(config_overrides)
    return MqttSubscriber(config, **kwargs)


def _connected_sub(paho_stub, config_overrides=None):
    """Build and connect a MqttSubscriber, fire the on_connect callback."""
    sub = _sub(config_overrides)
    sub.connect()
    sub._client.fire_connect(rc=0)
    return sub


# ---------------------------------------------------------------------------
# Connection lifecycle
# ---------------------------------------------------------------------------


class TestMqttSubscriberConnect:
    def test_connect_returns_true_on_success(self, paho_stub):
        sub = _sub()
        ok = sub.connect()
        assert ok is True

    def test_connect_starts_loop(self, paho_stub):
        sub = _sub()
        sub.connect()
        assert sub._client.loop_started

    def test_on_connect_callback_sets_is_connected(self, paho_stub):
        sub = _sub()
        sub.connect()
        sub._client.fire_connect(rc=0)
        assert sub._is_connected is True

    def test_on_connect_rc_nonzero_does_not_set_connected(self, paho_stub):
        """If the broker refuses the connection (rc != 0), is_connected stays False.
        On hardware: rc=5 = not authorized; add credentials to Mosquitto config.
        rc=3 = broker unavailable; confirm Mosquitto is running on red.lab.
        """
        sub = _sub()
        sub.connect()
        sub._client.fire_connect(rc=5)
        assert sub._is_connected is False

    def test_wait_until_connected_returns_true_once_handshake_completes(self, paho_stub):
        """connect() only starts the async handshake; wait_until_connected()
        is what a caller needing to publish() right after connecting (e.g.
        weir/flow's command path) should block on instead."""
        import threading

        sub = _sub()
        sub.connect()

        def fire_soon():
            time.sleep(0.05)
            sub._client.fire_connect(rc=0)

        threading.Thread(target=fire_soon).start()
        assert sub.wait_until_connected(timeout=2.0) is True

    def test_wait_until_connected_times_out_if_handshake_never_completes(self, paho_stub):
        sub = _sub()
        sub.connect()
        # on_connect never fires
        assert sub.wait_until_connected(timeout=0.2) is False

    def test_disconnect_stops_loop(self, paho_stub):
        sub = _connected_sub(paho_stub)
        client = sub._client  # save ref before disconnect() nulls it
        sub.disconnect()
        assert client.loop_stopped
        assert client.disconnected

    def test_disconnect_clears_is_connected(self, paho_stub):
        sub = _connected_sub(paho_stub)
        sub.disconnect()
        assert sub._is_connected is False

    def test_disconnect_when_not_connected_is_safe(self):
        """Disconnect without a prior connect should not raise."""
        sub = _sub()
        sub.disconnect()  # no client — should silently no-op

    def test_on_disconnect_unexpected_clears_connected(self, paho_stub):
        """If broker drops the connection (rc != 0), is_connected goes False.
        On hardware: rc=7 usually means the broker restarted. Check
        'systemctl status mosquitto' on the Pi; the client will auto-reconnect
        if reconnect_delay_set is active.
        """
        sub = _connected_sub(paho_stub)
        assert sub._is_connected is True
        sub._client.fire_disconnect(rc=7)
        assert sub._is_connected is False

    def test_connect_without_paho_installed(self):
        """If paho-mqtt is not installed, connect() returns False rather than raising.
        On hardware: run 'pip install paho-mqtt>=1.6' on the laguna PC.
        """
        with patch.dict(sys.modules, {"paho": None, "paho.mqtt": None, "paho.mqtt.client": None}):
            sub = _sub()
            result = sub.connect()
        assert result is False


# ---------------------------------------------------------------------------
# Topic subscription
# ---------------------------------------------------------------------------


class TestMqttSubscriberSubscribe:
    def test_initial_topics_are_subscribed_on_connect(self, paho_stub):
        """Topics listed at construction are subscribed when on_connect fires."""
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000", "laguna/gauge/water_level_mm"]})
        assert "laguna/od2000" in sub._client.subscriptions
        assert "laguna/gauge/water_level_mm" in sub._client.subscriptions

    def test_subscribe_after_connect_calls_client_subscribe(self, paho_stub):
        sub = _connected_sub(paho_stub)
        sub.subscribe("laguna/od2000")
        assert "laguna/od2000" in sub._client.subscriptions

    def test_subscribe_before_connect_stored_for_later(self, paho_stub):
        """subscribe() before connect() stores the topic; it is sent to the broker
        when on_connect fires.
        """
        sub = _sub()
        sub.subscribe("laguna/od2000")
        assert "laguna/od2000" in sub._topics
        sub.connect()
        sub._client.fire_connect()
        assert "laguna/od2000" in sub._client.subscriptions

    def test_duplicate_subscribe_does_not_duplicate_queue(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        sub.subscribe("laguna/od2000")  # second subscribe to same topic
        assert sub._topics.count("laguna/od2000") == 1

    def test_on_connect_resubscribes_after_reconnect(self, paho_stub):
        """Topics survive a broker disconnect + reconnect cycle.
        On hardware: if the Pi reboots mid-experiment, the broker restarts and
        the client must re-subscribe. reconnect_delay_set ensures the client
        tries to reconnect; on_connect ensures topics are re-registered.
        """
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        sub._client.fire_disconnect(rc=1)
        sub._client.subscriptions.clear()
        sub._client.fire_connect()  # simulate reconnect
        assert "laguna/od2000" in sub._client.subscriptions

    def test_no_wildcard_in_default_topic(self, paho_stub):
        """The AL1342 does NOT support MQTT wildcards (+ or #). The laguna
        subscriber uses exact topics set by the AL1342 callback URL.
        """
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        assert "#" not in sub._topics[0]
        assert "+" not in sub._topics[0]


# ---------------------------------------------------------------------------
# Message buffering — drain / get_latest
# ---------------------------------------------------------------------------


class TestMqttSubscriberMessages:
    def test_on_message_puts_into_topic_queue(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        sub._client.fire_message("laguna/od2000", {"code": "event", "cid": 10})
        items = sub.drain("laguna/od2000")
        assert len(items) == 1
        assert items[0]["cid"] == 10

    def test_drain_clears_the_queue(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        sub._client.fire_message("laguna/od2000", {"x": 1})
        sub._client.fire_message("laguna/od2000", {"x": 2})
        first_drain = sub.drain("laguna/od2000")
        second_drain = sub.drain("laguna/od2000")
        assert len(first_drain) == 2
        assert len(second_drain) == 0

    def test_drain_returns_messages_in_order(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        for i in range(5):
            sub._client.fire_message("laguna/od2000", {"seq": i})
        items = sub.drain("laguna/od2000")
        assert [it["seq"] for it in items] == [0, 1, 2, 3, 4]

    def test_drain_unknown_topic_returns_empty(self, paho_stub):
        sub = _sub()
        sub.connect()
        result = sub.drain("laguna/nonexistent")
        assert result == []

    def test_get_latest_returns_last_message(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        for i in range(3):
            sub._client.fire_message("laguna/od2000", {"seq": i})
        latest = sub.get_latest("laguna/od2000")
        assert latest["seq"] == 2

    def test_get_latest_drains_all(self, paho_stub):
        """get_latest() discards all but the last. Use drain() to get all messages."""
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        for i in range(10):
            sub._client.fire_message("laguna/od2000", {"seq": i})
        sub.get_latest("laguna/od2000")
        assert sub.drain("laguna/od2000") == []

    def test_get_latest_no_messages_returns_none(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        assert sub.get_latest("laguna/od2000") is None

    def test_non_json_payload_does_not_crash(self, paho_stub):
        """If the AL1342 publishes malformed JSON (e.g. firmware bug), the
        subscriber stores the raw string rather than crashing. On hardware:
        if drain() returns strings instead of dicts, the payload format has
        changed or the device is sending binary data without IODD encoding.
        """
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        sub._client.fire_message_raw("laguna/od2000", b"not-json-at-all")
        items = sub.drain("laguna/od2000")
        assert len(items) == 1
        assert isinstance(items[0], str)

    def test_messages_on_different_topics_stay_separate(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000", "laguna/gauge/water_level_mm"]})
        sub._client.fire_message("laguna/od2000", {"src": "od2000"})
        sub._client.fire_message("laguna/gauge/water_level_mm", {"src": "gauge"})
        od_items = sub.drain("laguna/od2000")
        gauge_items = sub.drain("laguna/gauge/water_level_mm")
        assert od_items[0]["src"] == "od2000"
        assert gauge_items[0]["src"] == "gauge"

    def test_message_on_unsubscribed_topic_is_ignored(self, paho_stub):
        """Message arriving on a topic not in _queues is silently dropped.
        On hardware: if MQTT messages appear to be lost, confirm the exact
        subscribed topic matches what the AL1342 is publishing to — topics
        are matched by exact string, not by pattern.
        """
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        sub._client.fire_message("laguna/something_else", {"x": 1})
        assert sub.drain("laguna/od2000") == []


# ---------------------------------------------------------------------------
# get_status
# ---------------------------------------------------------------------------


class TestMqttSubscriberStatus:
    def test_status_disconnected(self):
        sub = _sub({"topics": ["laguna/od2000"]})
        status = sub.get_status()
        assert status["is_connected"] is False

    def test_status_connected(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        status = sub.get_status()
        assert status["is_connected"] is True
        assert "laguna/od2000" in status["subscribed_topics"]

    def test_status_queue_depth(self, paho_stub):
        sub = _connected_sub(paho_stub, {"topics": ["laguna/od2000"]})
        for _ in range(3):
            sub._client.fire_message("laguna/od2000", {})
        status = sub.get_status()
        assert status["queue_depths"]["laguna/od2000"] == 3

    def test_status_shows_broker_address(self):
        sub = _sub({"broker_host": "red.lab", "broker_port": 1883})
        assert "red.lab" in sub.get_status()["broker"]
        assert "1883" in sub.get_status()["broker"]
