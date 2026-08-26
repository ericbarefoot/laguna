"""Water level measurement subsystem."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Optional
import logging

from ..mqtt import MqttSubscriber
from ..subsystem_logging import SubsystemLogging

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)


class WaterLevelSensor(ABC):
    """Abstract interface for water surface elevation measurement.

    Concrete implementations wrap whatever distance/level sensor is
    physically deployed, converting its raw reading into a water surface
    elevation in mm.
    """

    subsystem_name = "gauge"

    @abstractmethod
    def connect(self) -> bool:
        """Open the hardware connection.

        Returns:
            True if the connection was established, False otherwise.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the hardware connection and release any held resources."""
        ...

    @abstractmethod
    def read_mm(self) -> float:
        """Return water surface elevation in mm (instantaneous reading)."""
        ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Return a snapshot of the sensor's last reading and connection state.

        Returns:
            Dictionary of status fields. Contents are backend-specific, but
            always include at least `is_connected` and `elevation_mm`.
        """
        ...


class SaflWaterLevelSensor(WaterLevelSensor, SubsystemLogging):
    """Water level sensor backed by a Massa ultrasonic sensor, via MQTT.

    The Massa sensor is wired to the confluence node on red.lab, which
    polls it over USB serial and publishes readings to MQTT — this class
    no longer talks to serial itself, it subscribes to that topic (see
    laguna.mqtt.MqttSubscriber). Massa measures a downward-looking distance
    to the water surface; this class converts that into an elevation by
    subtracting it from a fixed reference offset (`offset_mm`), so
    elevation increases as the water rises and the measured distance
    shrinks.

    Because readings now arrive asynchronously (at whatever interval
    confluence's job schedule publishes on) rather than on-demand, read_mm()
    no longer triggers a fresh hardware read — it returns the most recently
    published sample, raising if none has arrived yet.
    """

    def __init__(self, config: Dict[str, Any], mqtt_subscriber: MqttSubscriber):
        """Build the sensor from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary (see
                Config._get_defaults()'s 'gauge' section for the expected
                shape). Recognized keys:
                - topic: MQTT topic the confluence Massa_Ultrasonic
                  interface publishes readings on (default
                  'SAFL Confluence Node 1/Massa_Ultrasonic').
                - sensor_index: Index into the Massa interface's per-device
                  arrays (dist_mm, signal_strength, ...) — confluence polls
                  every configured Massa ID in one message, so this picks
                  out which array element is this gauge's sensor
                  (default 0).
                - offset_mm: Elevation (mm) that corresponds to a Massa
                  reading of zero distance; used as `offset_mm -
                  dist_mm` for every reading (default 0.0).
                - log_level / event_log_verbosity: see laguna.subsystem_logging
                  (both default 'INFO').
                - simulated: skip MQTT entirely and return NaN readings
                  (default False).
            mqtt_subscriber: MqttSubscriber for MQTT operations.
        """
        self._topic = config.get("topic", "SAFL Confluence Node 1/Massa_Ultrasonic")
        self._sensor_index = int(config.get("sensor_index", 0))
        self._offset_mm = config.get("offset_mm", 0.0)
        self.log_level = config.get("log_level", "INFO")
        self.event_log_verbosity = config.get("event_log_verbosity", "INFO")
        self._simulated = config.get("simulated", False)
        self._mqtt = mqtt_subscriber
        self._is_connected = False
        self._last_read: Optional[Dict[str, Any]] = None

    @classmethod
    def from_config(cls, config: "Config") -> "SaflWaterLevelSensor":
        """Build from the lab's Config (its 'gauge:' section and shared 'mqtt:' section)."""
        section = config.get("gauge")
        mqtt_subscriber = MqttSubscriber(config.get("mqtt"))
        return cls(section, mqtt_subscriber)

    def connect(self) -> bool:
        """Connect the underlying MQTT subscriber and subscribe to the Massa topic.

        Returns:
            True if connected successfully (or if simulated).
        """
        if self._simulated:
            self._is_connected = True
            return True
        if not self._mqtt._is_connected:
            ok = self._mqtt.connect()
            if not ok:
                return False
        self._mqtt.subscribe(self._topic)
        self._is_connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect the underlying MQTT subscriber. Safe to call when already disconnected."""
        if self._simulated:
            self._is_connected = False
            return
        self._mqtt.disconnect()
        self._is_connected = False

    def _poll(self) -> None:
        """Drain the Massa topic and cache the latest message for our sensor_index."""
        for msg in self._mqtt.drain(self._topic):
            try:
                self._last_read = {
                    "dist_mm": msg["dist_mm"][self._sensor_index],
                    "temperature_c": msg["massa temperature"][self._sensor_index],
                    "signal_strength": msg["signal_strength"][self._sensor_index],
                }
            except (KeyError, IndexError, TypeError):
                logger.warning("Malformed Massa_Ultrasonic message on %s: %r", self._topic, msg)

    def read_mm(self) -> float:
        """Return the most recently published water surface elevation in mm.

        Unlike the old serial version, this does not trigger a fresh
        hardware read — it drains and returns from the MQTT topic.

        Raises:
            RuntimeError: If no reading has arrived yet.
        """
        if self._simulated:
            return float("nan")
        self._poll()
        if self._last_read is None:
            raise RuntimeError(
                f"{self.__class__.__name__}: no reading received yet on {self._topic!r}"
            )
        elevation_mm = float(self._offset_mm) - float(self._last_read["dist_mm"]) * 10.0
        # Operational log only, not the archival event log — a reading
        # measures the experiment's state without changing it, so it's
        # data, not a "step taken." A caller polling this on a schedule
        # that wants specific readings archived as milestones can do so
        # explicitly via lab.log_note() or runner.py's log_as_event opt-in
        # — see laguna.subsystem_logging's module docstring.
        logger.info("read_mm: elevation_mm=%.2f", elevation_mm)
        return elevation_mm

    def read_mm_smoothed(self) -> float:
        """Return a smoothed elevation reading.

        The confluence Massa interface does not currently publish a
        rolling average (see Interfaces/Massa_Ultrasonic/Massa_funcs.py on
        the confluence side), so there is no MQTT equivalent of the old
        driver's FIFO moving average yet. Returns NaN until confluence
        publishes one.
        """
        return float("nan")

    def get_status(self) -> Dict[str, Any]:
        """Return connection state and the most recent reading, without polling hardware.

        Drains any buffered MQTT messages (non-blocking) but does not wait
        for a new one.

        Returns:
            Dict with `is_connected`, `elevation_mm`, `temperature_c`, and
            `signal_strength`. All reading-derived fields are None until a
            message has arrived at least once.
        """
        if self._simulated:
            return {
                "is_connected": self._is_connected,
                "elevation_mm": float("nan") if self._is_connected else None,
                "temperature_c": None,
                "signal_strength": None,
            }
        if self._is_connected:
            self._poll()
        elevation_mm = None
        temperature_c = None
        signal_strength = None
        if self._last_read is not None:
            elevation_mm = float(self._offset_mm) - float(self._last_read["dist_mm"]) * 10.0
            temperature_c = self._last_read.get("temperature_c")
            signal_strength = self._last_read.get("signal_strength")
        return {
            "is_connected": self._is_connected,
            "elevation_mm": elevation_mm,
            "temperature_c": temperature_c,
            "signal_strength": signal_strength,
        }
