"""Water level measurement subsystem."""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)

try:
    from safl_ocean_hardware.massa import MassaSensor as _MassaSensor
except ImportError:
    _MassaSensor = None


class WaterLevelSensor(ABC):
    subsystem_name = "gauge"

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read_mm(self) -> float:
        """Return water surface elevation in mm (instantaneous reading)."""
        ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]: ...


class SaflWaterLevelSensor(WaterLevelSensor):
    """Water level sensor backed by a Massa ultrasonic distance sensor."""

    def __init__(self, config: Dict[str, Any]):
        self._port = config.get("port", "/dev/ttyUSB2")
        self._sensor_ids = config.get("sensor_ids", [0])
        self._offsets = config.get("offsets", None)
        self._offset_mm = config.get("offset_mm", 0.0)
        self._sensor = None
        self._is_connected = False
        self._last_read: Optional[dict] = None

    def connect(self) -> bool:
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
        if self._sensor and self._is_connected:
            self._sensor.disconnect()
        self._is_connected = False

    def read_mm(self) -> float:
        """Read and return instantaneous water surface elevation in mm."""
        result = self._sensor.read()
        self._last_read = result
        # Distance decreases as water rises: elevation = offset - distance_cm * 10
        return self._offset_mm - result["distance_cm"] * 10.0

    def read_mm_smoothed(self) -> float:
        """Read and return elevation using the sensor's built-in FIFO moving average."""
        self._sensor.read()
        avg_list = getattr(self._sensor, "dist_cm_array_moving_avg", [])
        if avg_list:
            return self._offset_mm - avg_list[0] * 10.0
        return float("nan")

    def get_status(self) -> Dict[str, Any]:
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
