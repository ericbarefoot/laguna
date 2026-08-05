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
    """Base rangefinder subsystem backed by an AL1342 MQTT data stream, with
    on-demand HTTP access (activate/deactivate/read_mm) for interactive use.

    Delegates connection lifecycle to the provided MqttSubscriber, then
    decodes incoming PDIN payloads and caches the latest reading — this
    MQTT path is for continuous/background monitoring and is what scans
    use internally. activate()/read_mm() are a separate, synchronous HTTP
    path (mirrors laguna.weir.SaflWeirController's get_elevation() shape)
    for one-off interactive reads, e.g. ``lab.od2000.connect();
    lab.od2000.activate(); lab.od2000.read_mm()``.

    Prefer the OD2000Rangefinder / WTT12LRangefinder subclasses below,
    which set subsystem_name and the correct decode/calibration wiring for
    each device — this base class is still constructible directly (as
    existing tests/scripts do) with the OD2000 decode as the default.

    Args:
        config: Dict with keys:
            topic: MQTT topic the AL1342 publishes PDIN events on.
            pdin_port: IO-Link port the sensor is connected to (1-8).
            offset_mm: Physical mounting offset, added to every reading
                (both the MQTT and read_mm() paths).
            al1342_host: AL1342 IP address (raw IP — it has no DNS of its
                own), required only for activate()/deactivate()/read_mm().
            calibration_file: Optional path to a LinearCalibration CSV (see
                laguna.rangefinder.calibration and
                scripts/calibrate_rangefinder.py). If given, read_mm()
                applies it instead of returning the raw decoded distance.
            simulated: Skip the real MQTT/AL1342-HTTP paths entirely —
                connect() succeeds with no broker needed, every reading is
                NaN rather than fabricated (default False; see
                laguna.simulation's module docstring).
        mqtt_subscriber: A connected or unconnected MqttSubscriber instance.
    """

    subsystem_name = "rangefinder"

    def __init__(self, config: Dict[str, Any], mqtt_subscriber: MqttSubscriber):
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
        """Build from the lab's Config: this subsystem's own section (by
        cls.subsystem_name) plus a fresh MqttSubscriber built from the
        shared 'mqtt:' section — each rangefinder gets its own client ID
        (see MqttSubscriber's docstring on why two instances can't share
        one), so a private MqttSubscriber per instance is the correct
        default even when several rangefinders read the same broker.
        """
        section = config.get(cls.subsystem_name)
        mqtt_subscriber = MqttSubscriber(config.get("mqtt"))
        return cls(section, mqtt_subscriber)

    # ------------------------------------------------------------------
    # Subsystem lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect the underlying MQTT subscriber and subscribe to OD2000 topic.

        Returns:
            True if connected successfully.
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
        """Drain the MQTT queue and update cached latest reading."""
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
        """Turn on whatever the sensor needs to produce valid readings (e.g.
        a laser emitter). Default is a no-op — override per device.
        """
        pass

    def deactivate(self) -> None:
        """Undo activate(). Default is a no-op — override per device."""
        pass

    def read_mm(self, timeout: float = 5.0) -> float:
        """Take a single on-demand HTTP reading (not the MQTT-cached
        stream), applying offset_mm and, if configured, a LinearCalibration.

        Raises:
            RuntimeError: If 'al1342_host' wasn't given in config, or the
                AL1342 returns a non-200 code (e.g. wrong pdin_port).
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
        """Which decoded field a configured calibration was fitted against.
        Override for devices calibrated against something other than
        distance_mm (see WTT12LRangefinder, calibrated against current_ma)."""
        return decoded["distance_mm"]

    # ------------------------------------------------------------------
    # Overridable decode hooks
    # ------------------------------------------------------------------

    def _extract_pdin_hex(self, msg: Dict[str, Any]) -> str:
        """Extract pdin hex string from AL1342 MQTT event envelope."""
        key = f"/iolinkmaster/port[{self._pdin_port}]/iolinkdevice/pdin"
        return msg["data"]["payload"][key]["data"]

    def _decode(self, hex_str: str) -> Dict[str, Any]:
        """Decode raw PDIN hex to engineering values. Override to update layout."""
        return decode_od2000_pdin(hex_str)


class OD2000Rangefinder(RangefinderSubsystem):
    """OD2000 rangefinder subsystem: ``lab.od2000.connect(); lab.od2000.activate();
    lab.od2000.read_mm()``. Laser activate/deactivate via IO-Link acyclic
    write (IODD index 97/0, inverted convention — "00" = laser on, "01" =
    off, confirmed on hardware 2026-07-28, see
    examples/example_06_od2000_acyclic_read.py).
    """

    subsystem_name = "od2000"

    def _decode(self, hex_str: str) -> Dict[str, Any]:
        return decode_od2000_pdin(hex_str)

    def activate(self) -> None:
        if self._simulated:
            return
        self._require_al1342_host()
        write_acyclic(self._al1342_host, self._pdin_port, index=97, subindex=0, value="00")

    def deactivate(self) -> None:
        if self._simulated:
            return
        self._require_al1342_host()
        write_acyclic(self._al1342_host, self._pdin_port, index=97, subindex=0, value="01")


class WTT12LRangefinder(RangefinderSubsystem):
    """WTT12L PowerProx rangefinder subsystem, read via its analog output
    through an ifm DP4200 IO-Link analog-input bridge (the sensor's own
    native IO-Link process data never validated on this AL1342 — see
    docs/WTT12L_POWERPROX_SETUP.md).

    activate()/deactivate() are intentionally left as the base class's
    no-ops: the DP4200 bridge has no laser/emitter control path ("no
    programmatic laser control on this path", same doc).
    """

    subsystem_name = "wtt12l"

    def _decode(self, hex_str: str) -> Dict[str, Any]:
        return decode_dp4200_wtt12l_analog_pdin(hex_str)

    def _calibration_raw_value(self, decoded: Dict[str, Any]) -> float:
        # Calibrated directly against current_ma, not this decoder's own
        # distance_mm (which already bakes in an unconfirmed current->mm
        # span) — see scripts/calibrate_rangefinder.py's module docstring.
        return decoded["current_ma"]
