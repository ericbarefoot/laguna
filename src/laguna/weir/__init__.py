"""Weir (tailgate) elevation control subsystem."""

from abc import ABC, abstractmethod
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)

try:
    from safl_ocean_hardware.motor import TeknicMotor as _TeknicMotor
except ImportError:
    _TeknicMotor = None


class WeirController(ABC):
    subsystem_name = "weir"

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def set_elevation(self, mm: float) -> bool:
        """Move weir to target elevation in mm."""
        ...

    @abstractmethod
    def get_elevation(self) -> float:
        """Return current weir elevation in mm."""
        ...

    @abstractmethod
    def home(self) -> bool: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]: ...


class SaflWeirController(WeirController):
    """Weir controller backed by a Teknic stepper motor."""

    def __init__(self, config: Dict[str, Any]):
        self._port = config.get("port", "/dev/ttyUSB0")
        self._baudrate = config.get("baudrate", 9600)
        self.steps_per_mm = config.get("steps_per_mm", 1000.0)
        self.home_offset_mm = config.get("home_offset_mm", 0.0)
        self._motor = None
        self._is_connected = False

    def connect(self) -> bool:
        if _TeknicMotor is None:
            logger.warning(
                "safl_ocean_hardware is not installed; SaflWeirController cannot connect"
            )
            return False
        try:
            self._motor = _TeknicMotor(self._port, self._baudrate)
            self._is_connected = self._motor.connect()
            return self._is_connected
        except Exception as e:
            logger.error(f"Failed to connect weir controller: {e}")
            return False

    def disconnect(self) -> None:
        if self._motor and self._is_connected:
            self._motor.disconnect()
        self._is_connected = False

    def set_elevation(self, mm: float) -> bool:
        steps = int((mm - self.home_offset_mm) * self.steps_per_mm)
        return self._motor.move_to_position(steps)

    def get_elevation(self) -> float:
        return self._motor.get_position() / self.steps_per_mm + self.home_offset_mm

    def home(self) -> bool:
        return self._motor.home()

    def stop(self) -> None:
        self._motor.stop()

    def get_status(self) -> Dict[str, Any]:
        raw_steps = self._motor.get_position() if self._is_connected else None
        elevation_mm = (
            raw_steps / self.steps_per_mm + self.home_offset_mm
            if raw_steps is not None
            else None
        )
        return {
            "is_connected": self._is_connected,
            "elevation_mm": elevation_mm,
            "raw_steps": raw_steps,
        }
