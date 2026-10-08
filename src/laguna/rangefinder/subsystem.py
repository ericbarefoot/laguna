"""Rangefinder subsystem classes read on demand from an AL1342 IO-Link master over HTTP."""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, Dict, Optional, Tuple

if TYPE_CHECKING:
    from laguna.config import Config

from .al1342 import read_pdin_hex, write_acyclic
from .calibration import LinearCalibration
from .decoders import decode_dp4200_wtt12l_analog_pdin, decode_od2000_pdin

logger = logging.getLogger(__name__)

#: HTTP timeout for the laser write a safety verb issues. Short on purpose:
#: a halt must not sit for the usual 5 s waiting on an unreachable AL1342
#: while other subsystems wait their turn behind it.
SAFETY_WRITE_TIMEOUT_S = 2.0


class RangefinderSubsystem:
    """Base rangefinder subsystem: synchronous reads over the AL1342's HTTP API.

    Every reading is a poll of the IO-Link master's process-data endpoint
    (``read_mm``), the same path the Pi-side profiler uses during a scan, so
    live readings, scans and calibration all go through one decode. The
    AL1342's MQTT push is not used: it tops out near 2 Hz where HTTP polling
    delivers hundreds of samples per second (see docs/MQTT_AL1342_SETUP.md),
    and nothing was ever configured to publish rangefinder data to it
    (GH #22). That leaves the master's MQTT features free for other tools.

    Prefer OD2000Rangefinder or WTT12LRangefinder subclasses in practice,
    which set subsystem_name and configure the correct decode/calibration.

    Args:
        config: Dict with keys: pdin_port (1-8), offset_mm (mounting
            offset), al1342_host (IP of the IO-Link master, required for any
            real read), calibration_file (optional LinearCalibration CSV),
            simulated (bool, disables real I/O when True).
    """

    subsystem_name = "rangefinder"

    def __init__(self, config: Dict[str, Any]):
        """Initialize a rangefinder subsystem.

        Args:
            config: Configuration dict (see class docstring for keys).
        """
        if "topic" in config:
            logger.warning(
                "%s: config key 'topic' is ignored — rangefinders poll the AL1342 over HTTP "
                "and no longer subscribe to MQTT. Remove it from the config.",
                self.subsystem_name,
            )
        self._pdin_port = int(config.get("pdin_port", 1))
        self._offset_mm = float(config.get("offset_mm", 0.0))
        self._al1342_host = config.get("al1342_host")
        self._simulated = config.get("simulated", False)

        calibration_file = config.get("calibration_file")
        self._calibration: Optional[LinearCalibration] = (
            LinearCalibration.from_csv(calibration_file) if calibration_file else None
        )

        self._latest_sample: Optional[Tuple[float, float]] = None  # (wall_time, distance_mm)
        self._sample_count = 0
        self._is_connected = False
        #: Whether *this object* last switched the emitter on. The Pi-side
        #: profiler can also switch it on without telling us, so this only
        #: decides what resume() restores — never whether a halt acts.
        self._laser_on = False
        self._laser_on_before_pause = False

    @classmethod
    def from_config(cls, config: "Config") -> "RangefinderSubsystem":
        """Build a rangefinder subsystem from lab configuration.

        Args:
            config: Lab Config with a section named by ``subsystem_name``.

        Returns:
            Rangefinder subsystem instance.
        """
        return cls(config.get(cls.subsystem_name))

    # ------------------------------------------------------------------
    # Subsystem lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Confirm the AL1342 answers a process-data read for this port.

        There is no persistent connection — every reading is its own HTTP
        request — so "connected" means a probe read just succeeded. That
        catches a wrong host or port here, at connect time, instead of as a
        mysteriously empty stream later (the failure #22 reported).

        Returns:
            True if the probe read succeeded (or simulated); False if no
            ``al1342_host`` is configured or the read failed. The reason is
            logged.
        """
        if self._simulated:
            self._is_connected = True
            return True
        if not self._al1342_host:
            logger.error(
                "%s: 'al1342_host' is not configured — cannot read the sensor",
                self.subsystem_name,
            )
            return False
        try:
            self.read_mm()
        except Exception as exc:
            logger.error(
                "%s: probe read of AL1342 %s port %d failed: %s",
                self.subsystem_name, self._al1342_host, self._pdin_port, exc,
            )
            return False
        self._is_connected = True
        return True

    def disconnect(self) -> None:
        """Mark disconnected. Nothing is held open, so there is nothing to release."""
        self._is_connected = False

    def get_status(self) -> Dict[str, Any]:
        """Return connection state and the last on-demand reading (no I/O).

        Readings only happen when something asks for one (``read_mm()``), so
        ``latest_*`` are as old as the last caller's read, not a live feed.
        """
        if self._simulated:
            return {
                "is_connected": self._is_connected,
                "pdin_port": self._pdin_port,
                "latest_distance_mm": float("nan"),
                "latest_wall_time": time.time(),
                "sample_count": 0,
            }
        wall_time = distance_mm = None
        if self._latest_sample is not None:
            wall_time, distance_mm = self._latest_sample
        return {
            "is_connected": self._is_connected,
            "pdin_port": self._pdin_port,
            "latest_distance_mm": distance_mm,
            "latest_wall_time": wall_time,
            "sample_count": self._sample_count,
        }

    # ------------------------------------------------------------------
    # HTTP access (synchronous — the only way readings are taken)
    # ------------------------------------------------------------------

    def _require_al1342_host(self) -> None:
        if not self._al1342_host:
            raise RuntimeError(
                f"{type(self).__name__} requires 'al1342_host' in config for "
                "on-demand HTTP access (activate/deactivate/read_mm)"
            )

    def activate(self, timeout: float = 5.0) -> None:
        """Turn on whatever the sensor needs to produce valid readings.

        Default is a no-op; override per device (e.g. for laser emitter control).

        Args:
            timeout: HTTP timeout, seconds, for devices controlled over HTTP.
        """
        pass

    def deactivate(self, timeout: float = 5.0) -> None:
        """Undo activate(). Default is a no-op — override per device.

        Args:
            timeout: HTTP timeout, seconds, for devices controlled over HTTP.
        """
        pass

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # A rangefinder is a passive sensor: nothing here moves, and a transect
    # in flight is driven and aborted by the gantry (its pause/stop/estop
    # stops the Pi-side profiler), not by this object. What it does own is
    # the emitter, so every tier reduces to "emitter off". There is no data
    # to discard, so these return a note only when the emitter could not be
    # confirmed off — which a human reading the event log needs to know.

    def _emitter_off(self) -> Optional[str]:
        """Switch the emitter off, best-effort and bounded. Never raises."""
        try:
            self.deactivate(timeout=SAFETY_WRITE_TIMEOUT_S)
        except Exception as exc:
            logger.error("%s: could not switch the emitter off: %s", self.subsystem_name, exc)
            return f"{self.subsystem_name} emitter state unknown — could not switch it off: {exc}"
        self._laser_on = False
        return None

    def pause(self) -> Optional[str]:
        """Switch the emitter off; resume() restores it if this object had it on."""
        self._laser_on_before_pause = self._laser_on
        return self._emitter_off()

    def resume(self) -> Optional[str]:
        """Switch the emitter back on if it was on when pause() was called."""
        if not self._laser_on_before_pause:
            return None
        self._laser_on_before_pause = False
        try:
            self.activate()
        except Exception as exc:
            logger.error("%s: could not restore the emitter: %s", self.subsystem_name, exc)
            return f"{self.subsystem_name} emitter was not restored after pause: {exc}"
        self._laser_on = True
        return None

    def stop(self) -> Optional[str]:
        """Switch the emitter off. Not resumable: start a new run instead."""
        self._laser_on_before_pause = False
        return self._emitter_off()

    def estop(self) -> Optional[str]:
        """Same as stop(): the emitter has no harder halt. Never raises."""
        self._laser_on_before_pause = False
        return self._emitter_off()

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
        distance_mm = value + self._offset_mm
        self._latest_sample = (time.time(), distance_mm)
        self._sample_count += 1
        return distance_mm

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

    def activate(self, timeout: float = 5.0) -> None:
        """Enable the OD2000 laser emitter.

        Args:
            timeout: HTTP timeout, seconds.
        """
        if not self._simulated:
            self._require_al1342_host()
            write_acyclic(
                self._al1342_host, self._pdin_port, index=97, subindex=0, value="00",
                timeout=timeout,
            )
        self._laser_on = True

    def deactivate(self, timeout: float = 5.0) -> None:
        """Disable the OD2000 laser emitter.

        Args:
            timeout: HTTP timeout, seconds.
        """
        if not self._simulated:
            self._require_al1342_host()
            write_acyclic(
                self._al1342_host, self._pdin_port, index=97, subindex=0, value="01",
                timeout=timeout,
            )
        self._laser_on = False


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
