"""Pump flow control subsystem."""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
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
    """Abstract interface for pump flow rate and inlet/aux valve control.

    Concrete implementations drive whatever VFD/pump hardware and solenoid
    valves are physically deployed, exposing flow rate in L/min regardless
    of the underlying protocol.
    """

    subsystem_name = "flow"

    @abstractmethod
    def connect(self) -> bool:
        """Open the hardware connection(s).

        Returns:
            True if all required connections were established, False
            otherwise.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the hardware connection(s) and release any held resources."""
        ...

    @abstractmethod
    def set_flowrate(self, lpm: float) -> bool:
        """Set pump flow rate in L/min."""
        ...

    @abstractmethod
    def get_flowrate(self) -> float:
        """Return the most recently commanded flow rate in L/min.

        This is the setpoint last sent via set_flowrate(), not a live
        measurement from a flow sensor.
        """
        ...

    @abstractmethod
    def start(self) -> bool:
        """Start the pump at its current flow rate setpoint.

        Returns:
            True if the start command was sent successfully.
        """
        ...

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # There is deliberately no abstract `stop() -> bool` pump primitive here:
    # `stop()` belongs to the shared safety vocabulary, so how a backend
    # actually stops its drive is its own private business (SaflFlowController
    # uses `_vfd_stop()`). All four verbs must return a note about anything a
    # human needs to know, or None, and must never raise.

    @abstractmethod
    def pause(self) -> Optional[str]:
        """Quiesce the pump, remembering the setpoint so resume() can restore it."""
        ...

    @abstractmethod
    def resume(self) -> Optional[str]:
        """Restore the setpoint captured by pause() and restart the pump."""
        ...

    @abstractmethod
    def stop(self) -> Optional[str]:
        """End cleanly: stop the pump, leaving the valves as they are."""
        ...

    @abstractmethod
    def estop(self) -> Optional[str]:
        """Pump off and both valves closed, each step attempted independently."""
        ...

    @abstractmethod
    def clear_faults(self) -> bool:
        """Clear any latched VFD fault/alarm state.

        Returns:
            True if faults were cleared successfully.
        """
        ...

    @property
    @abstractmethod
    def qin(self) -> bool:
        """Whether the inlet solenoid valve is currently commanded open."""
        ...

    @qin.setter
    @abstractmethod
    def qin(self, state: bool) -> None: ...

    @property
    @abstractmethod
    def qaux(self) -> bool:
        """Whether the auxiliary solenoid valve is currently commanded open."""
        ...

    @qaux.setter
    @abstractmethod
    def qaux(self, state: bool) -> None: ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Return a snapshot of pump/VFD status and valve states.

        Returns:
            Dictionary of status fields. Contents are backend-specific, but
            always include at least `is_connected` and `flowrate_lpm`.
        """
        ...


class SaflFlowController(FlowController):
    """Flow controller backed by a Fuji VFD pump and Teknic motor for solenoid IO.

    NOTE: In production, the TeknicMotor instance should be shared with
    SaflWeirController rather than creating a separate connection to the same
    physical motor. This implementation creates its own instance for
    self-contained first-draft purposes.
    """

    def __init__(self, config: Dict[str, Any]):
        """Build the controller from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary (see
                Config._get_defaults()'s 'flow' section for the expected
                shape). Recognized keys:
                - vfd_port: Serial device for the Fuji VFD (Modbus RTU)
                  (default '/dev/ttyUSB1').
                - vfd_slave_id: Modbus slave address of the VFD (default 1).
                - motor_port: Serial device for the Teknic ClearCore that
                  drives the qin/qaux solenoid digital outputs
                  (default '/dev/ttyUSB0').
                - motor_baudrate: Baud rate for the ClearCore connection
                  (default 9600).
                - C0, C1, C2: Coefficients of the quadratic pump calibration
                  curve `freq_hz = C2*Q^2 + C1*Q + C0` (Q in L/min) used by
                  set_flowrate() to convert a requested flow rate into a VFD
                  drive frequency. These are empirically fit per pump/
                  plumbing configuration — they are not physical constants,
                  just curve-fit coefficients for this specific installed
                  pump. Defaults (4.902, 58.49, 0.08956) match the values in
                  Config._get_defaults(); override per-installation as
                  needed. The resulting frequency is clamped to [0, 60] Hz
                  by the underlying VFD driver.
        """
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
        """Open connections to both the VFD (Modbus) and the ClearCore (serial).

        Both connections must succeed for this to report success; if either
        safl_ocean_hardware is missing or either device fails to connect,
        `is_connected` is left False.

        Returns:
            True only if both the VFD and motor connections succeeded.
        """
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
        """Close both the VFD and motor connections, if open.

        Safe to call when already disconnected.
        """
        if self._vfd is not None:
            self._vfd.disconnect()
        if self._motor is not None:
            self._motor.disconnect()
        self._vfd = None
        self._motor = None
        self._is_connected = False

    def _require_connected(self) -> None:
        """Raise RuntimeError if connect() hasn't succeeded yet."""
        if not self._is_connected:
            raise RuntimeError(f"{self.__class__.__name__} is not connected")

    def set_flowrate(self, lpm: float) -> bool:
        """Set the pump's target flow rate.

        Converts `lpm` to a VFD drive frequency using the quadratic
        calibration curve `C2*Q^2 + C1*Q + C0` (see __init__ for details on
        C0/C1/C2), clamps it to the VFD's [0, 60] Hz range, and writes it as
        the new setpoint. This does not itself start the pump — call
        start() to begin running at the new setpoint.

        Returns:
            True (the underlying driver does not report setpoint-write
            failure separately from a communication exception).

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._vfd.set_freq_from_flowrate(lpm, self.C0, self.C1, self.C2)
        self._current_flowrate = lpm
        return True

    def get_flowrate(self) -> float:
        """Return the most recently commanded flow rate in L/min.

        This is a locally cached setpoint, not a live sensor measurement —
        it is available even when disconnected, reflecting whatever was
        last passed to set_flowrate().
        """
        return self._current_flowrate

    def start(self) -> bool:
        """Start the pump at its current VFD frequency setpoint.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        return self._vfd.start()

    def _vfd_stop(self) -> bool:
        """Stop the pump drive itself. See stop() for the safety verb."""
        self._require_connected()
        return self._vfd.stop()

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def pause(self) -> Optional[str]:
        """Stop the pump, remembering the setpoint so resume() can restore it.

        Pausing a flume stops the water: the hydraulic condition is part of
        the experiment, so leaving the pump running would mean the experiment
        continues while everything else is held. The cost is that resuming
        needs a re-stabilisation period — that is inherent, not a defect.

        Never raises; a pause that throws leaves the rest of the rig running.
        """
        self._paused_flowrate = self._current_flowrate
        try:
            self._vfd_stop()
        except Exception as exc:
            logger.error("Could not stop the pump for pause: %s", exc)
            return f"pump may still be running: {exc}"
        return f"pump stopped, setpoint {self._paused_flowrate} L/min held for resume"

    def resume(self) -> Optional[str]:
        """Restore the setpoint captured by pause() and restart the pump."""
        target = getattr(self, "_paused_flowrate", None)
        try:
            if target:
                self.set_flowrate(target)
            self.start()
        except Exception as exc:
            logger.error("Could not restart the pump on resume: %s", exc)
            return f"pump did not restart: {exc}"
        return None

    def stop(self) -> Optional[str]:
        """End cleanly: stop the pump, leave the valves as they are.

        The valve positions are part of the experiment's configuration, not a
        hazard on their own — only estop() forces them shut.
        """
        try:
            self._vfd_stop()
        except Exception as exc:
            logger.error("Could not stop the pump: %s", exc)
            return f"pump may still be running: {exc}"
        return None

    def estop(self) -> Optional[str]:
        """Pump off and both valves closed, each attempted independently.

        Every step is guarded separately: a failure closing qin must not
        prevent qaux from being closed, and neither must prevent the pump
        being stopped. Never raises.
        """
        problems = []
        for label, action in (
            ("stop the pump", lambda: self._vfd_stop()),
            ("close qin", lambda: setattr(self, "qin", False)),
            ("close qaux", lambda: setattr(self, "qaux", False)),
        ):
            try:
                action()
            except Exception as exc:
                logger.error("ESTOP: could not %s: %s", label, exc)
                problems.append(label)
        return f"could not: {', '.join(problems)}" if problems else None

    def clear_faults(self) -> bool:
        """Clear any latched VFD alarm state.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        return self._vfd.clear_faults()

    @property
    def qin(self) -> bool:
        """Whether the inlet solenoid valve is currently commanded open.

        This reflects the last value written via the setter, not a live
        hardware readback.
        """
        return self._qin_state

    @qin.setter
    def qin(self, state: bool) -> None:
        """Open (True) or close (False) the inlet solenoid valve.

        Drives digital output channel 0 on the shared ClearCore controller.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.set_io(0, state)
        self._qin_state = state

    @property
    def qaux(self) -> bool:
        """Whether the auxiliary solenoid valve is currently commanded open.

        This reflects the last value written via the setter, not a live
        hardware readback.
        """
        return self._qaux_state

    @qaux.setter
    def qaux(self, state: bool) -> None:
        """Open (True) or close (False) the auxiliary solenoid valve.

        Drives digital output channel 1 on the shared ClearCore controller.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.set_io(1, state)
        self._qaux_state = state

    def get_status(self) -> Dict[str, Any]:
        """Return connection state, flow setpoint, valve states, and raw VFD status.

        When connected, this also polls the VFD over Modbus for its current
        state message, e-stop flag, and drive frequency setpoint — so this
        call is not free of hardware I/O like the qin/qaux property getters.

        Returns:
            Dict with `is_connected`, `flowrate_lpm`, `qin_open`, and
            `qaux_open`; when connected, also includes `vfd_state`,
            `vfd_estop`, and `vfd_setpoint_hz`.
        """
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
