"""Rangefinder subsystem classes backed by an AL1342 MQTT data stream."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Dict, Optional, Tuple

from laguna.mqtt import MqttSubscriber

if TYPE_CHECKING:
    from laguna.config import Config

from .al1342 import read_pdin_hex, write_acyclic
from .calibration import LinearCalibration
from .decoders import decode_dp4200_wtt12l_analog_pdin, decode_od2000_pdin


class RangefinderSubsystem:
    """Base rangefinder subsystem backed by an AL1342 MQTT data stream.

    Provides both a continuous MQTT-based stream for background monitoring
    (get_distance_mm, get_latest_sample) and synchronous HTTP on-demand access
    for interactive use (activate, deactivate, read_mm).

    Prefer OD2000Rangefinder or WTT12LRangefinder subclasses in practice,
    which set subsystem_name and configure the correct decode/calibration.

    Args:
        config: Dict with keys: topic (MQTT topic), pdin_port (1-8),
            offset_mm (mounting offset), al1342_host (IP for HTTP access),
            calibration_file (optional LinearCalibration CSV), simulated
            (bool, disables real I/O when True).
        mqtt_subscriber: MqttSubscriber instance for MQTT operations.
    """

    subsystem_name = "rangefinder"

    def __init__(self, config: Dict[str, Any], mqtt_subscriber: MqttSubscriber):
        """Initialize a rangefinder subsystem.

        Args:
            config: Configuration dict (see class docstring for keys).
            mqtt_subscriber: MqttSubscriber for MQTT operations.
        """
        self._topic = config.get("topic", "laguna/od2000")
        self._pdin_port = int(config.get("pdin_port", 1))
        self._offset_mm = float(config.get("offset_mm", 0.0))
        self._al1342_host = config.get("al1342_host")
        self._simulated = config.get("simulated", False)
        self._mqtt = mqtt_subscriber

        calibration_file = config.get("calibration_file")
        self._calibration: Optional[LinearCalibration] = (
            LinearCalibration.from_csv(calibration_file) if calibration_file else None
        )

        self._latest_sample: Optional[Tuple[float, float]] = None  # (wall_time, distance_mm)
        self._sample_count = 0
        self._t_first_sample: Optional[float] = None
        self._is_connected = False

    @classmethod
    def from_config(cls, config: "Config") -> "RangefinderSubsystem":
        """Build a rangefinder subsystem from lab configuration.

        Args:
            config: Lab Config with subsystem section (by subsystem_name)
                and shared 'mqtt:' section.

        Returns:
            Rangefinder subsystem instance with a private MqttSubscriber.
        """
        section = config.get(cls.subsystem_name)
        mqtt_subscriber = MqttSubscriber(config.get("mqtt"))
        return cls(section, mqtt_subscriber)

    # ------------------------------------------------------------------
    # Subsystem lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect the underlying MQTT subscriber and subscribe to OD2000 topic.

        Blocks (up to a few seconds) for the broker handshake to actually
        complete before returning — connect() alone only starts it
        asynchronously (see MqttSubscriber.wait_until_connected()).

        Returns:
            True if connected successfully; False if the broker handshake
            didn't complete in time.
        """
        if self._simulated:
            self._is_connected = True
            return True
        if not self._mqtt._is_connected:
            ok = self._mqtt.connect()
            if not ok or not self._mqtt.wait_until_connected():
                return False
        self._mqtt.subscribe(self._topic)
        self._is_connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect the underlying MQTT subscriber."""
        if self._simulated:
            self._is_connected = False
            return
        self._mqtt.disconnect()
        self._is_connected = False

    def get_status(self) -> Dict[str, Any]:
        """Return connection state and the most recent reading (no I/O)."""
        if self._simulated:
            return {
                "is_connected": self._is_connected,
                "topic": self._topic,
                "pdin_port": self._pdin_port,
                "latest_distance_mm": float("nan"),
                "latest_wall_time": time.time(),
                "sample_count": 0,
                "achieved_rate_hz": float("nan"),
            }
        distance_mm = None
        wall_time = None
        if self._latest_sample is not None:
            wall_time, distance_mm = self._latest_sample
        duration = (
            time.time() - self._t_first_sample
            if self._t_first_sample is not None
            else None
        )
        achieved_hz = (
            self._sample_count / duration
            if duration is not None and duration > 0
            else None
        )
        return {
            "is_connected": self._is_connected and self._mqtt._is_connected,
            "topic": self._topic,
            "pdin_port": self._pdin_port,
            "latest_distance_mm": distance_mm,
            "latest_wall_time": wall_time,
            "sample_count": self._sample_count,
            "achieved_rate_hz": achieved_hz,
        }

    # ------------------------------------------------------------------
    # Readings
    # ------------------------------------------------------------------

    def _poll(self) -> None:
        """Drain MQTT queue and update internal latest reading cache."""
        messages = self._mqtt.drain(self._topic)
        for msg in messages:
            try:
                hex_str = self._extract_pdin_hex(msg)
                decoded = self._decode(hex_str)
                wall_time = time.time()
                self._latest_sample = (wall_time, decoded["distance_mm"] + self._offset_mm)
                self._sample_count += 1
                if self._t_first_sample is None:
                    self._t_first_sample = wall_time
            except Exception:
                pass

    def get_distance_mm(self) -> Optional[float]:
        """Return the most recent distance reading in mm, or None if no data yet."""
        if self._simulated:
            return float("nan")
        self._poll()
        if self._latest_sample is None:
            return None
        return self._latest_sample[1]

    def get_latest_sample(self) -> Optional[Tuple[float, float]]:
        """Return (wall_time_unix, distance_mm) for the most recent reading."""
        if self._simulated:
            return (time.time(), float("nan"))
        self._poll()
        return self._latest_sample

    # ------------------------------------------------------------------
    # On-demand HTTP access (mirrors laguna.weir's synchronous shape)
    # ------------------------------------------------------------------

    def _require_al1342_host(self) -> None:
        if not self._al1342_host:
            raise RuntimeError(
                f"{type(self).__name__} requires 'al1342_host' in config for "
                "on-demand HTTP access (activate/deactivate/read_mm)"
            )

    def activate(self) -> None:
        """Turn on whatever the sensor needs to produce valid readings.

        Default is a no-op; override per device (e.g. for laser emitter control).
        """
        pass

    def deactivate(self) -> None:
        """Undo activate(). Default is a no-op — override per device."""
        pass

    def read_mm(self, timeout: float = 5.0) -> float:
        """Take a single on-demand HTTP reading with offset and calibration.

        Args:
            timeout: HTTP request timeout in seconds (default 5.0).

        Returns:
            Calibrated and offset distance in millimeters.

        Raises:
            RuntimeError: If 'al1342_host' not in config, or AL1342
                returns non-200 code.
        """
        if self._simulated:
            return float("nan")
        self._require_al1342_host()
        hex_str = read_pdin_hex(self._al1342_host, self._pdin_port, timeout=timeout)
        decoded = self._decode(hex_str)
        if self._calibration is not None:
            value = self._calibration.apply(self._calibration_raw_value(decoded))
        else:
            value = decoded["distance_mm"]
        return value + self._offset_mm

    def _calibration_raw_value(self, decoded: Dict[str, Any]) -> float:
        """Extract the field from decoded values used for calibration.

        Override for devices calibrated against fields other than distance_mm.

        Args:
            decoded: Decoded PDIN values dict.

        Returns:
            Calibration input value (default: distance_mm).
        """
        return decoded["distance_mm"]

    # ------------------------------------------------------------------
    # Overridable decode hooks
    # ------------------------------------------------------------------

    def _extract_pdin_hex(self, msg: Dict[str, Any]) -> str:
        """Extract PDIN hex string from AL1342 MQTT message envelope.

        Args:
            msg: MQTT message dict from AL1342.

        Returns:
            Hex-encoded PDIN payload string.
        """
        key = f"/iolinkmaster/port[{self._pdin_port}]/iolinkdevice/pdin"
        return msg["data"]["payload"][key]["data"]

    def _decode(self, hex_str: str) -> Dict[str, Any]:
        """Decode raw PDIN hex string to engineering values.

        Args:
            hex_str: Hex-encoded PDIN payload.

        Returns:
            Dict of decoded values (layout depends on device subclass).
        """
        return decode_od2000_pdin(hex_str)


class OD2000Rangefinder(RangefinderSubsystem):
    """OD2000 7002T15 rangefinder subsystem with laser control.

    Activates and deactivates the laser emitter via IO-Link acyclic write
    (IODD index 97/0: value "00" enables laser, "01" disables).
    """

    subsystem_name = "od2000"

    def _decode(self, hex_str: str) -> Dict[str, Any]:
        """Decode OD2000 PDIN hex string."""
        return decode_od2000_pdin(hex_str)

    def activate(self) -> None:
        """Enable the OD2000 laser emitter."""
        if self._simulated:
            return
        self._require_al1342_host()
        write_acyclic(self._al1342_host, self._pdin_port, index=97, subindex=0, value="00")

    def deactivate(self) -> None:
        """Disable the OD2000 laser emitter."""
        if self._simulated:
            return
        self._require_al1342_host()
        write_acyclic(self._al1342_host, self._pdin_port, index=97, subindex=0, value="01")


class WTT12LRangefinder(RangefinderSubsystem):
    """WTT12L PowerProx rangefinder subsystem via DP4200 analog input bridge.

    The sensor's analog output is digitized through an ifm DP4200 IO-Link
    analog-input bridge. Laser control not available on this path.
    """

    subsystem_name = "wtt12l"

    def _decode(self, hex_str: str) -> Dict[str, Any]:
        """Decode DP4200 PDIN hex string (WTT12L via analog path)."""
        return decode_dp4200_wtt12l_analog_pdin(hex_str)

    def _calibration_raw_value(self, decoded: Dict[str, Any]) -> float:
        """Return current_ma (not distance_mm) for calibration.

        Args:
            decoded: Decoded PDIN values dict.

        Returns:
            Current in mA from channel 1 (used as calibration input).
        """
        return decoded["current_ma"]
