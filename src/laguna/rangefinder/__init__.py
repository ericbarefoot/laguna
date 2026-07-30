"""OD2000 laser rangefinder subsystem.

Wraps MqttSubscriber to provide a typed interface for SICK OD2000 distance
readings delivered via an ifm AL1342 IO-Link master over MQTT.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

from laguna.mqtt import MqttSubscriber

from .al1342 import read_pdin_hex, write_acyclic
from .calibration import LinearCalibration


def decode_od2000_pdin(hex_str: str) -> Dict[str, Any]:
    """Decode a raw OD2000 7002T15 PDIN hex string to engineering values.

    The OD2000 process data is 6 bytes (12 hex chars):
      bytes 0-3  big-endian int32: distance in nm
      byte  4    scale (normally 0)
      byte  5    bit 0 = Q1, bit 1 = Q2

    Args:
        hex_str: 12-character hex string from AL1342 pdin payload field.

    Returns:
        Dict with distance_nm, distance_mm, scale, q1, q2.
    """
    raw = bytes.fromhex(hex_str)
    distance_nm = int.from_bytes(raw[0:4], "big", signed=True)
    return {
        "distance_nm": distance_nm,
        "distance_mm": distance_nm / 1_000_000,
        "scale": raw[4],
        "q1": bool(raw[5] & 0x01),
        "q2": bool(raw[5] & 0x02),
    }


def decode_wtt12l_pdin(hex_str: str) -> Dict[str, Any]:
    """Decode a raw SICK WTT12L PowerProx PDIN hex string to engineering values.

    Per SICK "Technical Information: Photoelectric sensors, SICK Smart
    Sensors / IO-Link" (www.sick.com/8022709), table 6 ("Process data
    structure - WTTxx with or without Smart Task 'Base Logic'"), the WTT12L
    process data is 4 bytes (8 hex chars), big-endian:
      bytes 0-1  unsigned int16: distance to object, mm
      byte  2    reserved
      byte  3    bit 0 = QL1, bit 1 = QL2, bits 2-7 reserved

    Status: unconfirmed and currently unreachable. On hardware (2026-07-28),
    the WTT12L-A2523's native IO-Link process data never validated — the
    AL1342 consistently returned code 530 ("invalid process data") for
    `pdin/getdata`, and every dynamic ISDU read (Detection mode, Sender
    configuration, Distance to object, etc.) returned IDX_NOTAVAIL, while
    static identification ISDUs (vendor/product/serial) read back fine. The
    root cause was never isolated — see docs/WTT12L_POWERPROX_SETUP.md. The
    working path ended up being the sensor's analog output routed through an
    ifm DP4200 IO-Link analog-input bridge instead; see
    decode_dp4200_wtt12l_analog_pdin() for that path's (hardware-confirmed)
    decoder.

    Args:
        hex_str: 8-character hex string from AL1342 pdin payload field.

    Returns:
        Dict with distance_mm, ql1, ql2.
    """
    raw = bytes.fromhex(hex_str)
    distance_mm = int.from_bytes(raw[0:2], "big", signed=False)
    return {
        "distance_mm": distance_mm,
        "ql1": bool(raw[3] & 0x01),
        "ql2": bool(raw[3] & 0x02),
    }


def decode_dp4200_wtt12l_analog_pdin(
    hex_str: str,
    near_mm: float = 100.0,
    far_mm: float = 1400.0,
) -> Dict[str, Any]:
    """Decode a WTT12L-A2523 distance reading taken via its analog output,
    digitized by an ifm DP4200 IO-Link analog-input bridge.

    Used because the WTT12L's own IO-Link process data never validated on
    this AL1342 (see decode_wtt12l_pdin()'s docstring) — the sensor's Qa
    analog output (4-20 mA, un-taught default span near_mm..far_mm) is fed
    into a DP4200 channel instead, and the DP4200 is the thing actually
    plugged into the AL1342 IO-Link port.

    The DP4200 process data is 4 bytes (8 hex chars), big-endian, two
    16-bit fields — one per DP4200 input channel:
      bytes 0-1  channel 1 raw reading, confirmed to be current in µA
      bytes 2-3  channel 2 raw reading — confirmed on hardware 2026-07-28 to
                 sit at a constant 0xFD01 regardless of target distance,
                 consistent with an unconnected/open channel 2 input; not
                 decoded here.

    Byte layout and the "channel 1 = µA" interpretation are confirmed on
    hardware: two readings at physically distinct distances (600 mm ->
    10.384 mA, 1115 mm -> ~16.13 mA) both back-solve to an implied sensor
    full-scale distance within ~5% of the WTT12L-A2523's actual rated max
    range (1,400 mm) — strong cross-check, though the exact near_mm/far_mm
    span was never independently confirmed via the sensor's own teach
    parameters (it's the datasheet's stated un-taught default). Decoded
    distance carries a wider error margin than the OD2000 or a validated
    native WTT12L reading — expect ~20-30 mm of slop on top of the sensor's
    own ±15-20 mm accuracy spec.

    Args:
        hex_str: 8-character hex string from AL1342 pdin payload field.
        near_mm: Distance corresponding to 4 mA (un-taught default = 100 mm
            per the WTT12L-A2523 datasheet).
        far_mm: Distance corresponding to 20 mA (un-taught default = the
            sensor's rated max range, 1,400 mm for the -A2523 variant).

    Returns:
        Dict with current_ma (channel 1) and distance_mm.
    """
    raw = bytes.fromhex(hex_str)
    channel1_raw = int.from_bytes(raw[0:2], "big", signed=False)
    current_ma = channel1_raw / 1000.0
    distance_mm = near_mm + (current_ma - 4.0) / 16.0 * (far_mm - near_mm)
    return {
        "current_ma": current_ma,
        "distance_mm": distance_mm,
    }


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
        mqtt_subscriber: A connected or unconnected MqttSubscriber instance.
    """

    subsystem_name = "rangefinder"

    def __init__(self, config: Dict[str, Any], mqtt_subscriber: MqttSubscriber):
        self._topic = config.get("topic", "laguna/od2000")
        self._pdin_port = int(config.get("pdin_port", 1))
        self._offset_mm = float(config.get("offset_mm", 0.0))
        self._al1342_host = config.get("al1342_host")
        self._mqtt = mqtt_subscriber

        calibration_file = config.get("calibration_file")
        self._calibration: Optional[LinearCalibration] = (
            LinearCalibration.from_csv(calibration_file) if calibration_file else None
        )

        self._latest_sample: Optional[Tuple[float, float]] = None  # (wall_time, distance_mm)
        self._sample_count = 0
        self._t_first_sample: Optional[float] = None
        self._is_connected = False

    # ------------------------------------------------------------------
    # Subsystem lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect the underlying MQTT subscriber and subscribe to OD2000 topic.

        Returns:
            True if connected successfully.
        """
        if not self._mqtt._is_connected:
            ok = self._mqtt.connect()
            if not ok:
                return False
        self._mqtt.subscribe(self._topic)
        self._is_connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect the underlying MQTT subscriber."""
        self._mqtt.disconnect()
        self._is_connected = False

    def get_status(self) -> Dict[str, Any]:
        """Return connection state and the most recent reading (no I/O)."""
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
        self._poll()
        if self._latest_sample is None:
            return None
        return self._latest_sample[1]

    def get_latest_sample(self) -> Optional[Tuple[float, float]]:
        """Return (wall_time_unix, distance_mm) for the most recent reading."""
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
        self._require_al1342_host()
        write_acyclic(self._al1342_host, self._pdin_port, index=97, subindex=0, value="00")

    def deactivate(self) -> None:
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
