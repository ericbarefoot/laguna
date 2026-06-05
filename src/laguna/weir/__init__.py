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
    def set_elevation(self, mm: float) -> None:
        """Reset the current position register to mm without moving the weir."""
        ...

    @abstractmethod
    def go_to_elevation(self, mm: float) -> bool:
        """Move weir to target elevation in mm."""
        ...

    @abstractmethod
    def get_elevation(self) -> float:
        """Return current weir elevation in mm."""
        ...

    @abstractmethod
    def set_velocity(self, mm_per_sec: float) -> None:
        """Set motor move speed in mm/s (applies to the next go_to_elevation call)."""
        ...

    @abstractmethod
    def get_velocity(self) -> float:
        """Return current velocity setpoint in mm/s."""
        ...

    @abstractmethod
    def enable(self) -> None:
        """Enable motor drive."""
        ...

    @abstractmethod
    def disable(self) -> None:
        """Disable motor drive (allows manual repositioning)."""
        ...

    @abstractmethod
    def wait_for_move(self, timeout: float = 30.0) -> None:
        """Block until the current move completes or timeout elapses."""
        ...

    @abstractmethod
    def clear_faults(self) -> bool: ...

    @abstractmethod
    def home(self) -> bool: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]: ...


class SaflWeirController(WeirController):
    """Weir controller backed by a Teknic ClearCore stepper motor.

    The ClearCore firmware handles all unit conversion internally (configured
    on its SD card), so positions and velocities are passed through in mm and
    mm/s respectively with no scaling applied here.
    """

    def __init__(self, config: Dict[str, Any]):
        self._port = config.get("port", "/dev/ttyUSB0")
        self._baudrate = config.get("baudrate", 9600)
        self._home_offset_mm = config.get("home_offset_mm", 0.0)
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

    def set_elevation(self, mm: float) -> None:
        self._motor.set_absoulute_position(mm)

    def go_to_elevation(self, mm: float) -> bool:
        return self._motor.move_to_position(mm)

    def get_elevation(self) -> float:
        return self._motor.get_position()

    def set_velocity(self, mm_per_sec: float) -> None:
        self._motor.set_velocity(mm_per_sec)

    def get_velocity(self) -> float:
        status = self._motor.poll_status()
        return status.get("VelSetPoint", float("nan"))

    def enable(self) -> None:
        self._motor.enable()

    def disable(self) -> None:
        self._motor.disable()

    def wait_for_move(self, timeout: float = 30.0) -> None:
        self._motor.wait_for_HLFB(timeout)

    def clear_faults(self) -> bool:
        self._motor.clear_faults()
        return True

    def home(self) -> bool:
        return self._motor.find_home(home_position_mm=self._home_offset_mm)

    def stop(self) -> None:
        self._motor.stop()

    def get_status(self) -> Dict[str, Any]:
        if not self._is_connected:
            return {"is_connected": False, "elevation_mm": None}
        motor_status = self._motor.poll_status()
        return {
            "is_connected": self._is_connected,
            "elevation_mm": motor_status.get("position"),
            "motor": motor_status,
        }
