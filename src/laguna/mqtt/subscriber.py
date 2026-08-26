"""MQTT subscriber subsystem for laguna.

Wraps paho-mqtt with the standard laguna connect/disconnect/get_status interface
and a thread-safe per-topic message buffer so callers can drain samples at any
rate without missing messages.
"""

from __future__ import annotations

import json
import logging
import queue
import time
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
        # Most recent message seen per topic via get_latest() — see that
        # method's docstring for why this exists separately from the queue.
        self._last_seen: Dict[str, Dict[str, Any]] = {}

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

    def wait_until_connected(self, timeout: float = 5.0, poll_interval: float = 0.05) -> bool:
        """Block until the broker handshake actually completes, or timeout.

        connect() only starts paho's background connect/loop — the real
        handshake completes asynchronously via the on_connect callback,
        which is what actually flips _is_connected. A caller that publishes
        (or expects subscriptions to be live) immediately after connect()
        can lose this race: publish() raises "MqttSubscriber is not
        connected" if called before the handshake finishes, and
        subscribe() silently defers rather than failing, so neither one
        surfaces the gap on its own. Call this after connect() before
        relying on either.

        Returns:
            True once connected; False if `timeout` elapses first.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._is_connected:
                return True
            time.sleep(poll_interval)
        return self._is_connected

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
            logger.info("MQTT subscribed to %s", topic)

    def publish(self, topic: str, payload: Any, qos: Optional[int] = None) -> None:
        """Publish a message to the broker.

        Args:
            topic: Topic to publish on.
            payload: JSON-serialized if not already a str/bytes (dicts are
                the common case — command/reply envelopes).
            qos: Overrides the subscriber's default qos for this publish only.

        Raises:
            RuntimeError: If not connected.
        """
        if self._client is None or not self._is_connected:
            raise RuntimeError("MqttSubscriber is not connected")
        body = payload if isinstance(payload, (str, bytes)) else json.dumps(payload)
        self._client.publish(topic, body, qos=self._qos if qos is None else qos)

    def get_latest(self, topic: str) -> Optional[Dict[str, Any]]:
        """Return the most recently published message for a topic, or None.

        Coalesces any backlog into a per-topic "last seen" slot (discarding
        older duplicates) and returns that slot, rather than handing the one
        buffered message to whichever caller happens to ask first and
        leaving every other caller reading None until the next publish.
        This matters because several read-only accessors on one subsystem
        (e.g. get_elevation()/get_velocity()/get_status() on the weir) can
        all call get_latest() on the same status topic close together — with
        pure drain-and-discard semantics, only the first of those calls
        would see the message and the rest would get None/stale fallbacks,
        which is exactly the get_velocity()-vs-get_status() mismatch this
        was built to fix. Only drain() actually forgets a topic's last-seen
        message (see its docstring) — that is the deliberate way to say "I
        don't trust anything buffered before this point," e.g.
        go_to_elevation() using it to discard a pre-move status reading.
        """
        if topic not in self._queues:
            return None
        q = self._queues[topic]
        while True:
            try:
                self._last_seen[topic] = q.get_nowait()
            except queue.Empty:
                break
        return self._last_seen.get(topic)

    def drain(self, topic: str) -> List[Dict[str, Any]]:
        """Return and remove all buffered messages for a topic.

        Also forgets that topic's get_latest() "last seen" message, so a
        subsequent get_latest() call returns None until a message arrives
        after this call — the explicit way to discard backlog a caller
        knows is stale (see get_latest()'s docstring).
        """
        self._last_seen.pop(topic, None)
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
            logger.info("MQTT connected to %s:%d", self._host, self._port)
            # Re-subscribe to topics registered before this connection (or
            # left over from before a drop, on a reconnect) — a fresh
            # first-time connect has none yet, since subscribe() calls
            # from FlumeLab/subsystem connect() happen just after this
            # callback returns, not before.
            for topic in self._topics:
                client.subscribe(topic, qos=self._qos)
                logger.info("MQTT subscribed to %s", topic)
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
