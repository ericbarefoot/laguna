"""Pump flow control subsystem."""

from abc import ABC, abstractmethod
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)

try:
    from safl_ocean_hardware.vfd import VFD as _VFD
except ImportError:
    _VFD = None

try:
    from safl_ocean_hardware.motor import TeknicMotor as _TeknicMotor
except ImportError:
    _TeknicMotor = None


class FlowController(ABC):
    subsystem_name = "flow"

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def set_flowrate(self, lpm: float) -> bool:
        """Set pump flow rate in L/min."""
        ...

    @abstractmethod
    def get_flowrate(self) -> float: ...

    @abstractmethod
    def start(self) -> bool: ...

    @abstractmethod
    def stop(self) -> bool: ...

    @abstractmethod
    def clear_faults(self) -> bool: ...

    @property
    @abstractmethod
    def qin(self) -> bool: ...

    @qin.setter
    @abstractmethod
    def qin(self, state: bool) -> None: ...

    @property
    @abstractmethod
    def qaux(self) -> bool: ...

    @qaux.setter
    @abstractmethod
    def qaux(self, state: bool) -> None: ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]: ...


class SaflFlowController(FlowController):
    """Flow controller backed by a Fuji VFD pump and Teknic motor for solenoid IO.

    NOTE: In production, the TeknicMotor instance should be shared with
    SaflWeirController rather than creating a separate connection to the same
    physical motor. This implementation creates its own instance for
    self-contained first-draft purposes.
    """

    def __init__(self, config: Dict[str, Any]):
        self._vfd_port = config.get("vfd_port", "/dev/ttyUSB1")
        self._vfd_slave_id = config.get("vfd_slave_id", 1)
        self._motor_port = config.get("motor_port", "/dev/ttyUSB0")
        self._motor_baudrate = config.get("motor_baudrate", 9600)
        self.C0 = config.get("C0", 4.902)
        self.C1 = config.get("C1", 58.49)
        self.C2 = config.get("C2", 0.08956)

        self._vfd = None
        self._motor = None
        self._is_connected = False
        self._current_flowrate = 0.0
        self._qin_state = False
        self._qaux_state = False

    def connect(self) -> bool:
        if _VFD is None or _TeknicMotor is None:
            logger.warning(
                "safl_ocean_hardware is not installed; SaflFlowController cannot connect"
            )
            return False
        try:
            self._vfd = _VFD(self._vfd_port, self._vfd_slave_id)
            self._motor = _TeknicMotor(self._motor_port, self._motor_baudrate)
            vfd_ok = self._vfd.connect()
            motor_ok = self._motor.connect()
            self._is_connected = vfd_ok and motor_ok
            return self._is_connected
        except Exception as e:
            logger.error(f"Failed to connect flow controller: {e}")
            return False

    def disconnect(self) -> None:
        if self._vfd and self._is_connected:
            self._vfd.disconnect()
        if self._motor and self._is_connected:
            self._motor.disconnect()
        self._is_connected = False

    def set_flowrate(self, lpm: float) -> bool:
        self._vfd.set_freq_from_flowrate(lpm, self.C0, self.C1, self.C2)
        self._current_flowrate = lpm
        return True

    def get_flowrate(self) -> float:
        return self._current_flowrate

    def start(self) -> bool:
        return self._vfd.start()

    def stop(self) -> bool:
        return self._vfd.stop()

    def clear_faults(self) -> bool:
        return self._vfd.clear_faults()

    @property
    def qin(self) -> bool:
        return self._qin_state

    @qin.setter
    def qin(self, state: bool) -> None:
        self._motor.set_io(0, state)
        self._qin_state = state

    @property
    def qaux(self) -> bool:
        return self._qaux_state

    @qaux.setter
    def qaux(self, state: bool) -> None:
        self._motor.set_io(1, state)
        self._qaux_state = state

    def get_status(self) -> Dict[str, Any]:
        status: Dict[str, Any] = {
            "is_connected": self._is_connected,
            "flowrate_lpm": self._current_flowrate,
            "qin_open": self._qin_state,
            "qaux_open": self._qaux_state,
        }
        if self._is_connected and self._vfd is not None:
            vfd_state = self._vfd.poll_state()
            status["vfd_state"] = vfd_state.get("state_message")
            status["vfd_estop"] = vfd_state.get("e_stop")
            self._vfd.poll_setpoint()
            status["vfd_setpoint_hz"] = getattr(self._vfd, "setpoint", None)
        return status
