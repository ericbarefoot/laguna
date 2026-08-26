"""Shared fake MqttSubscriber double for weir/flow/gauge subsystem tests.

Mirrors the FakeMqttSubscriber in test_rangefinder.py — same double, same
shape — extended with `publish()` and reply-injection so it can also drive
the weir/flow request/reply path (laguna.mqtt.request_reply.request()).
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional


class FakeMqttSubscriber:
    """Minimal double for MqttSubscriber, injectable into weir/flow/gauge."""

    subsystem_name = "mqtt"

    def __init__(self):
        self._is_connected = False
        self._queues: Dict[str, list] = {}
        self._topics: List[str] = []
        self.connected_called = 0
        self.disconnect_called = 0
        self.published: List[tuple] = []  # (topic, payload)

    def connect(self) -> bool:
        self._is_connected = True
        self.connected_called += 1
        return True

    def disconnect(self) -> None:
        self._is_connected = False
        self.disconnect_called += 1

    def subscribe(self, topic: str) -> None:
        if topic not in self._queues:
            self._queues[topic] = []
            self._topics.append(topic)

    def push(self, topic: str, payload: dict) -> None:
        """Test helper: inject a message as if MQTT delivered it."""
        self._queues.setdefault(topic, []).append(payload)

    def drain(self, topic: str) -> list:
        items = list(self._queues.get(topic, []))
        self._queues[topic] = []
        return items

    def get_latest(self, topic: str) -> Optional[dict]:
        items = self._queues.get(topic, [])
        last = items[-1] if items else None
        self._queues[topic] = []
        return last

    def get_status(self) -> dict:
        return {"is_connected": self._is_connected}

    def publish(self, topic: str, payload: Any, qos: Optional[int] = None) -> None:
        """Record the publish. If `auto_reply_on(topic, ...)` was set up for
        this exact topic, immediately push a reply as if the confluence node
        answered synchronously."""
        self.published.append((topic, payload))
        auto_replies = getattr(self, "_auto_replies", {})
        entry = auto_replies.get(topic)
        if entry is not None:
            reply_topic, build_reply = entry
            self.push(reply_topic, build_reply(payload))

    def auto_reply_on(self, commands_topic: str, replies_topic: str, build_reply) -> None:
        """Test helper: whenever publish() is called on `commands_topic`, push
        a reply built from the published command envelope by
        `build_reply(payload) -> dict` onto `replies_topic`. `build_reply`
        should echo `payload["request_id"]`. Supports multiple independent
        command channels (e.g. flow's VFD and valve axes) by keying on topic.
        """
        if not hasattr(self, "_auto_replies"):
            self._auto_replies: Dict[str, tuple] = {}
        self._auto_replies[commands_topic] = (replies_topic, build_reply)
