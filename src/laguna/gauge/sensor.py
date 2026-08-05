"""Water level measurement subsystem."""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Dict, Any, Optional
import logging

from ..subsystem_logging import SubsystemLogging

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)

try:
    from safl_ocean_hardware.massa import MassaSensor as _MassaSensor
except ImportError:
    _MassaSensor = None


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
    """Water level sensor backed by a Massa ultrasonic distance sensor.

    The Massa sensor measures a downward-looking distance to the water
    surface; this class converts that into an elevation by subtracting it
    from a fixed reference offset (`offset_mm`), so elevation increases as
    the water rises and the measured distance shrinks.
    """

    def __init__(self, config: Dict[str, Any]):
        """Build the sensor from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary (see
                Config._get_defaults()'s 'gauge' section for the expected
                shape). Recognized keys:
                - port: Serial device for the Massa sensor
                  (default '/dev/ttyUSB2').
                - sensor_ids: Massa device IDs to poll on the shared serial
                  bus (default [0]). Only the first ID's reading is used by
                  read_mm()/get_status() below.
                - offsets: Per-sensor-ID offsets (cm) forwarded to the
                  underlying MassaSensor driver; not used by this class's
                  own mm-based elevation calculation (default None).
                - offset_mm: Elevation (mm) that corresponds to a Massa
                  reading of zero distance; used as `offset_mm -
                  distance_mm` for every reading in this class
                  (default 0.0).
                - log_level / event_log_verbosity: see laguna.subsystem_logging
                  (both default 'INFO').
                - simulated: build a SimulatedMassaSensor instead of the
                  real driver — see laguna.simulation (default False).
        """
        self._port = config.get("port", "/dev/ttyUSB2")
        self._sensor_ids = config.get("sensor_ids", [0])
        self._offsets = config.get("offsets", None)
        self._offset_mm = config.get("offset_mm", 0.0)
        self.log_level = config.get("log_level", "INFO")
        self.event_log_verbosity = config.get("event_log_verbosity", "INFO")
        self._simulated = config.get("simulated", False)
        self._sensor = None
        self._is_connected = False
        self._last_read: Optional[dict] = None

    @classmethod
    def from_config(cls, config: "Config") -> "SaflWaterLevelSensor":
        """Build from the lab's Config (its 'gauge:' section)."""
        return cls(config.get("gauge"))

    def connect(self) -> bool:
        """Open the serial connection to the Massa sensor(s).

        Returns:
            True if connected. False if safl_ocean_hardware isn't installed,
            or if the connection attempt raised an exception (the exception
            is logged, not propagated).
        """
        if self._simulated:
            from ..simulation import SimulatedMassaSensor

            self._sensor = SimulatedMassaSensor()
            self._is_connected = self._sensor.connect()
            return self._is_connected
        if _MassaSensor is None:
            logger.warning(
                "safl_ocean_hardware is not installed; SaflWaterLevelSensor cannot connect"
            )
            return False
        try:
            self._sensor = _MassaSensor(self._port, self._sensor_ids, self._offsets)
            self._is_connected = self._sensor.connect()
            return self._is_connected
        except Exception as e:
            logger.error(f"Failed to connect water level sensor: {e}")
            return False

    def disconnect(self) -> None:
        """Close the serial connection, if open. Safe to call when already disconnected."""
        if self._sensor and self._is_connected:
            self._sensor.disconnect()
        self._is_connected = False

    def read_mm(self) -> float:
        """Read and return instantaneous water surface elevation in mm.

        Triggers a synchronous read on the underlying Massa sensor (a
        request/response exchange over serial), updates the cached
        `_last_read` used by get_status(), and returns the elevation for
        the first configured sensor ID.
        """
        result = self._sensor.read()
        self._last_read = result
        # Distance decreases as water rises: elevation = offset - distance_cm * 10
        elevation_mm = self._offset_mm - result["distance_cm"] * 10.0
        # Operational log only, not the archival event log — a reading
        # measures the experiment's state without changing it, so it's
        # data, not a "step taken." A caller polling this on a schedule
        # that wants specific readings archived as milestones can do so
        # explicitly via lab.log_note() or runner.py's log_as_event opt-in
        # — see laguna.subsystem_logging's module docstring.
        logger.info("read_mm: elevation_mm=%.2f", elevation_mm)
        return elevation_mm

    def read_mm_smoothed(self) -> float:
        """Read and return elevation using the sensor's built-in FIFO moving average.

        Like read_mm(), this triggers a fresh sensor read, but returns the
        elevation computed from the Massa driver's rolling average of
        recent distance samples rather than the single latest reading —
        useful for reducing noise from surface ripples.

        Returns:
            Smoothed elevation in mm, or NaN if the driver has no moving
            average available yet (e.g. immediately after connecting).
        """
        self._last_read = self._sensor.read()
        avg_list = getattr(self._sensor, "dist_cm_array_moving_avg", [])
        if avg_list:
            return self._offset_mm - avg_list[0] * 10.0
        return float("nan")

    def get_status(self) -> Dict[str, Any]:
        """Return connection state and the most recent reading, without polling hardware.

        Unlike read_mm(), this does not talk to the sensor — it reports
        values derived from whatever the last read_mm()/read_mm_smoothed()
        call cached.

        Returns:
            Dict with `is_connected`, `elevation_mm`, `temperature_c`, and
            `signal_strength`. All reading-derived fields are None until a
            read has been performed at least once.
        """
        elevation_mm = None
        temperature_c = None
        signal_strength = None
        if self._last_read is not None:
            elevation_mm = self._offset_mm - self._last_read["distance_cm"] * 10.0
            temperature_c = self._last_read.get("temperature")
            signal_strength = self._last_read.get("signal_strength")
        return {
            "is_connected": self._is_connected,
            "elevation_mm": elevation_mm,
            "temperature_c": temperature_c,
            "signal_strength": signal_strength,
        }
