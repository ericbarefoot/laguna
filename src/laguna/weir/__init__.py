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
    """Abstract interface for tailgate weir elevation control.

    Concrete implementations drive whatever motor/actuator raises or lowers
    the flume's tailgate weir, exposing elevation and velocity in physical
    units (mm, mm/s) regardless of the underlying hardware protocol.
    """

    subsystem_name = "weir"

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
    def set_elevation(self, mm: float) -> None:
        """Redefine the controller's notion of the current elevation as `mm`.

        This recalibrates the position reference without commanding any
        motion — it does not move the weir. Use it to re-reference the
        controller after the weir has been repositioned by other means
        (e.g. manually, or as the final step of a homing routine). To
        actually move the weir, use go_to_elevation().
        """
        ...

    @abstractmethod
    def go_to_elevation(self, mm: float) -> bool:
        """Command the weir to move to an absolute target elevation in mm.

        Returns:
            True if the move command was accepted by the hardware.
        """
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
    def clear_faults(self) -> bool:
        """Clear any latched motor fault state.

        Returns:
            True if faults were cleared successfully.
        """
        ...

    @abstractmethod
    def home(self) -> bool:
        """Run the homing routine to establish a reference position.

        Returns:
            True if homing completed successfully, False on timeout or fault.
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """Immediately halt any in-progress move."""
        ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Return a snapshot of controller/motor status.

        Returns:
            Dictionary of status fields. Contents are backend-specific, but
            always include at least `is_connected` and `elevation_mm`.
        """
        ...


class SaflWeirController(WeirController):
    """Weir controller backed by a Teknic ClearCore stepper motor.

    The ClearCore firmware handles all unit conversion internally (configured
    on its SD card), so positions and velocities are passed through in mm and
    mm/s respectively with no scaling applied here.
    """

    def __init__(self, config: Dict[str, Any]):
        """Build the controller from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary (see
                Config._get_defaults()'s 'weir' section for the expected
                shape). Recognized keys:
                - port: Serial device for the ClearCore controller
                  (default '/dev/ttyUSB0').
                - baudrate: Serial baud rate (default 9600).
                - home_offset_mm: Position value written to the controller's
                  position register once home() finds the limit switch
                  (default 0.0).
        """
        self._port = config.get("port", "/dev/ttyUSB0")
        self._baudrate = config.get("baudrate", 9600)
        self._home_offset_mm = config.get("home_offset_mm", 0.0)
        self._motor = None
        self._is_connected = False

    def connect(self) -> bool:
        """Open the serial connection to the ClearCore controller.

        Returns:
            True if connected. False if safl_ocean_hardware isn't installed,
            or if the connection attempt raised an exception (the exception
            is logged, not propagated).
        """
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
        """Close the serial connection, if open. Safe to call when already disconnected."""
        if self._motor is not None:
            self._motor.disconnect()
        self._motor = None
        self._is_connected = False

    def _require_connected(self) -> None:
        """Raise RuntimeError if connect() hasn't succeeded yet."""
        if not self._is_connected:
            raise RuntimeError(f"{self.__class__.__name__} is not connected")

    def set_elevation(self, mm: float) -> None:
        """Redefine the ClearCore's internal position register as `mm`.

        Sends the ClearCore 'pset' command, which only recalibrates what the
        firmware believes its current position is — it commands no motion.
        This is the same recalibration home() performs after the limit
        switch trips. To actually move the weir, use go_to_elevation().

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.set_absolute_position(mm)

    def go_to_elevation(self, mm: float) -> bool:
        """Command the weir to move to an absolute elevation of `mm`.

        Sends the ClearCore absolute-move sequence and returns as soon as
        the move has been issued — it does not block until arrival. Call
        wait_for_move() to block until the move finishes.

        Returns:
            True if the move command was sent successfully.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        return self._motor.move_to_position(mm)

    def get_elevation(self) -> float:
        """Query and return the ClearCore's current position in mm.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        return self._motor.get_position()

    def set_velocity(self, mm_per_sec: float) -> None:
        """Set the move speed used by the *next* go_to_elevation() call.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.set_velocity(mm_per_sec)

    def get_velocity(self) -> float:
        """Return the ClearCore's current velocity setpoint in mm/s.

        Returns:
            The polled 'VelSetPoint' status field, or NaN if it isn't
            present in the returned status dict.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        status = self._motor.poll_status()
        return status.get("VelSetPoint", float("nan"))

    def enable(self) -> None:
        """Enable the motor drive so it can accept move commands.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.enable()

    def disable(self) -> None:
        """Disable the motor drive, allowing the weir to be repositioned by hand.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.disable()

    def wait_for_move(self, timeout: float = 30.0) -> None:
        """Block until the ClearCore's HLFB (step-active) signal clears, or timeout elapses.

        Unlike go_to_elevation(), this call blocks the caller — it polls
        hardware status in a loop until motion stops or the timeout expires.

        Args:
            timeout: Maximum seconds to wait before giving up (default 30).

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.wait_for_HLFB(timeout)

    def clear_faults(self) -> bool:
        """Send the ClearCore 'clear' command to clear latched motor faults.

        Returns:
            True (the underlying driver has no way to report clear-fault
            failure).

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.clear_faults()
        return True

    def home(self) -> bool:
        """Jog to the negative limit switch and re-reference the position register.

        Drives the axis toward the negative limit at a fixed firmware jog
        speed (not configurable from here) until the limit switch trips,
        then writes `home_offset_mm` (from the config passed to __init__)
        into the position register — the same 'pset' recalibration
        set_elevation() performs. This call blocks until the limit switch
        trips, a fault is detected, or an internal timeout elapses.

        Returns:
            True if the limit switch was found successfully; False on
            timeout or motor fault.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        return self._motor.find_home(home_position_mm=self._home_offset_mm)

    def stop(self) -> None:
        """Immediately halt any in-progress move.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        self._motor.stop()

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def _halt(self) -> Optional[str]:
        """Halt any in-progress move. Never raises — shared by every tier,
        because the weir holds its elevation mechanically and so has exactly
        one safe action regardless of severity."""
        try:
            self._require_connected()
            self._motor.stop()
        except Exception as exc:
            logger.error("Could not halt the weir: %s", exc)
            return f"weir may still be moving: {exc}"
        return None

    def pause(self) -> Optional[str]:
        """Halt any in-progress elevation move."""
        return self._halt()

    def resume(self) -> Optional[str]:
        """Nothing to restore — the weir holds its elevation mechanically."""
        return None

    def stop(self) -> Optional[str]:
        """Halt the move. Same action as pause: the weir has one stop."""
        return self._halt()

    def estop(self) -> Optional[str]:
        """Halt the move. Same action as pause: the weir has one stop."""
        return self._halt()

    def get_status(self) -> Dict[str, Any]:
        """Return connection state, current elevation, and raw motor status.

        Returns:
            Dict with `is_connected` and `elevation_mm` (None when not
            connected); when connected, also includes the full raw status
            dict from the motor driver under the `motor` key.
        """
        if not self._is_connected:
            return {"is_connected": False, "elevation_mm": None}
        motor_status = self._motor.poll_status()
        return {
            "is_connected": self._is_connected,
            "elevation_mm": motor_status.get("position"),
            "motor": motor_status,
        }
