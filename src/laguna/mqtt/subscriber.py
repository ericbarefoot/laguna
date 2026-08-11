"""MQTT subscriber subsystem for laguna.

Wraps paho-mqtt with the standard laguna connect/disconnect/get_status interface
and a thread-safe per-topic message buffer so callers can drain samples at any
rate without missing messages.
"""

from __future__ import annotations

import json
import logging
import queue
import uuid
from collections import deque
from typing import TYPE_CHECKING, Any, Dict, List, Optional

if TYPE_CHECKING:
    from ..config import Config
    from ..timing.clock import ExperimentClock
    from ..timing.event_log import EventLog

logger = logging.getLogger(__name__)


class MqttSubscriber:
    """MQTT subscriber with per-topic message buffering.

    Wraps paho-mqtt with the standard laguna subsystem interface
    (connect/disconnect/get_status). The paho network loop runs in a
    background thread; all callbacks are thread-safe.
    """

    subsystem_name = "mqtt"

    def __init__(
        self,
        config: Dict[str, Any],
        event_log: Optional["EventLog"] = None,
        clock: Optional["ExperimentClock"] = None,
    ):
        """Build the subscriber from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary with keys:
                broker_host, broker_port, client_id, keepalive,
                topics (list of initial topics to subscribe), qos.
            event_log: Optional EventLog instance; if provided, each incoming
                message is logged immediately (in the paho thread).
            clock: Unused; reserved for future ExperimentClock integration.
        """
        self._host = config.get("broker_host", "red.lab")
        self._port = int(config.get("broker_port", 1883))
        # Suffixed with a per-instance UUID so two subsystems built from the
        # same config dict (e.g. od2000 + wtt12l in example_07, both reading
        # "mqtt" config) never collide on client ID — a broker disconnects
        # the older client whenever a new connection reuses its ID, which
        # sends both instances into an endless reconnect fight. Observed on
        # hardware as rc=7 (MQTT_ERR_CONN_LOST) alternating between the two
        # topics; see MQTT_AL1342_SETUP.md.
        base_client_id = config.get("client_id", "laguna")
        self._client_id = f"{base_client_id}-{uuid.uuid4().hex[:8]}"
        self._keepalive = int(config.get("keepalive", 60))
        self._initial_topics: List[str] = list(config.get("topics", []))
        self._qos = int(config.get("qos", 0))
        self._event_log = event_log

        self._client = None
        self._is_connected = False
        # Per-topic inbound queue (filled in paho thread, drained by caller)
        self._queues: Dict[str, queue.Queue] = {}
        self._topics: List[str] = []

        for topic in self._initial_topics:
            self._queues[topic] = queue.Queue()
            self._topics.append(topic)

    @classmethod
    def from_config(cls, config: "Config") -> "MqttSubscriber":
        """Build from the lab's Config (its 'mqtt:' section)."""
        return cls(config.get("mqtt"))

    # ------------------------------------------------------------------
    # Subsystem lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect to the MQTT broker and start the background network loop.

        Returns:
            True if the connection was established, False on error.
        """
        try:
            import paho.mqtt.client as mqtt  # type: ignore[import]
        except ImportError:
            logger.error("paho-mqtt is not installed; run: pip install paho-mqtt>=1.6")
            return False

        client = mqtt.Client(client_id=self._client_id)
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            client.connect(self._host, self._port, keepalive=self._keepalive)
        except Exception as e:
            logger.error("Failed to connect to MQTT broker at %s:%d: %s", self._host, self._port, e)
            return False

        self._client = client
        client.loop_start()
        logger.info("MQTT subscriber connecting to %s:%d", self._host, self._port)
        return True

    def disconnect(self) -> None:
        """Stop the background loop and disconnect from the broker."""
        client = self._client
        self._client = None
        self._is_connected = False
        if client is not None:
            try:
                client.loop_stop()
                client.disconnect()
            except Exception:
                pass
        logger.info("MQTT subscriber disconnected")

    def get_status(self) -> Dict[str, Any]:
        """Return connection state and per-topic queue depths (no I/O)."""
        return {
            "is_connected": self._is_connected,
            "broker": f"{self._host}:{self._port}",
            "subscribed_topics": list(self._topics),
            "queue_depths": {t: self._queues[t].qsize() for t in self._topics},
        }

    # ------------------------------------------------------------------
    # Topic management and message access
    # ------------------------------------------------------------------

    def subscribe(self, topic: str) -> None:
        """Subscribe to an additional topic.

        Safe to call before or after connect(); if already connected the
        subscription is sent immediately, otherwise it is applied in
        on_connect once the broker connection is established.
        """
        if topic not in self._queues:
            self._queues[topic] = queue.Queue()
            self._topics.append(topic)
        if self._is_connected and self._client is not None:
            self._client.subscribe(topic, qos=self._qos)

    def get_latest(self, topic: str) -> Optional[Dict[str, Any]]:
        """Return the most recent message for a topic, or None if queue is empty.

        Drains the entire queue and discards all but the last item.
        """
        if topic not in self._queues:
            return None
        q = self._queues[topic]
        last = None
        while True:
            try:
                last = q.get_nowait()
            except queue.Empty:
                break
        return last

    def drain(self, topic: str) -> List[Dict[str, Any]]:
        """Return and remove all buffered messages for a topic."""
        if topic not in self._queues:
            return []
        q = self._queues[topic]
        items = []
        while True:
            try:
                items.append(q.get_nowait())
            except queue.Empty:
                break
        return items

    # ------------------------------------------------------------------
    # paho callbacks (run in paho's network thread)
    # ------------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc):
        """Handle broker connection completion (paho callback)."""
        if rc == 0:
            self._is_connected = True
            for topic in self._topics:
                client.subscribe(topic, qos=self._qos)
            logger.info("MQTT connected to %s:%d; subscribed to %s", self._host, self._port, self._topics)
        else:
            logger.error("MQTT connection refused (rc=%d)", rc)

    def _on_disconnect(self, client, userdata, rc):
        """Handle broker disconnection (paho callback)."""
        self._is_connected = False
        if rc != 0:
            logger.warning("MQTT disconnected unexpectedly (rc=%d); will retry", rc)

    def _on_message(self, client, userdata, msg):
        """Handle inbound message (paho callback)."""
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            payload = msg.payload.decode("utf-8", errors="replace")

        topic = msg.topic
        if topic in self._queues:
            self._queues[topic].put(payload)

        if self._event_log is not None:
            try:
                self._event_log.log(subsystem="mqtt", event_type="message",
                                    result=topic, notes=str(payload)[:200])
            except Exception:
                pass
