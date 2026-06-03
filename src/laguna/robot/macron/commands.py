"""Typed ASCII command interface for the Snap2Motion MMC/OEM controller.

Command token format (NEEDS HARDWARE VERIFICATION):
  Single-axis: X[N] CMD [params...]   e.g. "X[1] ACP" to read axis 1 position
  Group:       G[N] CMD [params...]   e.g. "G[1] BMT 100.0 200.0 5.0"
  Global:      CMD [params...]        e.g. "INB 3" to read input bit 3

The exact prefix characters ('X', 'G') must be confirmed on hardware. Change
AXIS_TOKEN_FMT and GROUP_TOKEN_FMT below if the controller uses different syntax.

Positions and velocities are in whatever user units the controller is configured
for. If CountsPerUserUnit is set to the correct belt-pitch conversion on the
controller, commands are effectively in mm and mm/s.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import logging

from .connection import SnapConnection, SnapMotionError

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Token format — adjust these if hardware verification reveals different syntax
# ---------------------------------------------------------------------------
AXIS_TOKEN_FMT = "X[{n}]"   # single-axis commands
GROUP_TOKEN_FMT = "G[{n}]"  # coordinated-group commands


# ---------------------------------------------------------------------------
# Axis enumeration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Axis:
    """Maps a named axis to its firmware axis index (1-based)."""
    name: str
    index: int

    def token(self) -> str:
        return AXIS_TOKEN_FMT.format(n=self.index)


# Default axis map matching the UCR demo program (XXPrime=1, Y=2, Z=3, Theta=4).
# Override in config if the controller uses different indices.
X_AXIS = Axis("X", 1)
Y_AXIS = Axis("Y", 2)
Z_AXIS = Axis("Z", 3)
THETA_AXIS = Axis("Theta", 4)

ALL_AXES = (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)


# ---------------------------------------------------------------------------
# State dataclasses
# ---------------------------------------------------------------------------

@dataclass
class AxisState:
    """Snapshot of a single axis's status."""
    actual_position: float = 0.0       # ACP — stepper/commanded tracker
    commanded_position: float = 0.0    # COP — last commanded setpoint
    destination_position: float = 0.0  # DEP — target of current move
    encoder_position: float = 0.0      # ENP — raw encoder count (closed-loop feedback)
    speed: float = 0.0                 # SPD
    accel: float = 0.0                 # ACL
    decel: float = 0.0                 # DCL
    motor_on: bool = False             # MTR
    enabled: bool = False              # ENA
    move_is_finished: bool = True      # MIF
    capture_bit: bool = False          # CAB — live state of capture input
    capture_position: float = 0.0      # CAP — position latched at capture event
    capture_has_tripped: bool = False  # CAT — latch flag (cleared by ArmCapture)
    negative_limit: float = 0.0        # NLT — software negative travel limit
    positive_limit: float = 0.0        # PLT — software positive travel limit


@dataclass
class GroupState:
    """Snapshot of a coordinated group's status."""
    speed: float = 0.0
    accel: float = 0.0
    decel: float = 0.0
    motor_on: bool = False
    move_is_finished: bool = True
    commanded_position: tuple[float, ...] = field(default_factory=tuple)
    destination_position: float = 0.0


@dataclass
class IOMap:
    """Digital IO pin mapping for brakes and limit switches.

    Indices come from the TNamedIO definitions in the UCR program.
    Verify against the Modusystems wiring diagram — the channel 16 IO
    indices in Snap2Motion may or may not map 1:1 to INB/SOB indices.
    """
    y_brake_output: int = 4          # SOB index for Y electromagnetic brake
    z_brake_output: int = 5          # SOB index for Z electromagnetic brake
    y_brake_status_input: int = 8    # INB index for Y brake engaged/disengaged feedback
    z_brake_status_input: int = 1    # INB index for Z brake engaged/disengaged feedback
    # Limit switch capture sources — set via SCS command before arming capture
    x_limit_capture_source: int = 1  # UNVERIFIED — needs wiring diagram
    y_limit_capture_source: int = 2  # UNVERIFIED
    z_limit_capture_source: int = 3  # UNVERIFIED


# ---------------------------------------------------------------------------
# Command interface
# ---------------------------------------------------------------------------

class MMCCommands:
    """Formats, sends, and parses all Snap2Motion ASCII commands.

    Every public method maps to one or a small sequence of ASCII commands.
    Methods that can both set and get follow the firmware convention: pass a
    value to set, omit it (or pass None) to read the current value.

    All position/velocity values are in controller user units (assumed mm / mm/s
    if CountsPerUserUnit is correctly configured on the controller).
    """

    def __init__(self, connection: SnapConnection, group_index: int = 1):
        self._conn = connection
        self._group = group_index

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ax(self, axis: Axis) -> str:
        return axis.token()

    def _gx(self) -> str:
        return GROUP_TOKEN_FMT.format(n=self._group)

    def _send(self, cmd: str) -> float:
        raw = self._conn.send(cmd)
        try:
            return float(raw)
        except ValueError:
            raise SnapMotionError(0, f"Non-numeric response to {cmd!r}: {raw!r}")

    def _fmt_params(self, *values: float) -> str:
        return " ".join(f"{v:.6g}" for v in values)

    # ------------------------------------------------------------------
    # Motor & enable
    # ------------------------------------------------------------------

    def set_motor(self, axis: Axis, on: bool) -> bool:
        """Enable or disable a single-axis motor drive. Returns current state."""
        val = 1 if on else 0
        return bool(self._send(f"{self._ax(axis)} MTR {val}"))

    def get_motor(self, axis: Axis) -> bool:
        return bool(self._send(f"{self._ax(axis)} MTR"))

    def set_motor_group(self, on: bool) -> bool:
        """Enable or disable all motors in the coordinated group."""
        val = 1 if on else 0
        return bool(self._send(f"{self._gx()} MTR {val}"))

    def set_enable(self, axis: Axis, enabled: bool) -> bool:
        val = 1 if enabled else 0
        return bool(self._send(f"{self._ax(axis)} ENA {val}"))

    def get_enable(self, axis: Axis) -> bool:
        return bool(self._send(f"{self._ax(axis)} ENA"))

    # ------------------------------------------------------------------
    # Position & kinematics — single axis
    # ------------------------------------------------------------------

    def get_actual_position(self, axis: Axis) -> float:
        """Read the axis stepper position tracker (ACP)."""
        return self._send(f"{self._ax(axis)} ACP")

    def set_actual_position(self, axis: Axis, value: float) -> float:
        """Set/zero the actual position register (ACP). Returns new value."""
        return self._send(f"{self._ax(axis)} ACP {value:.6g}")

    def get_encoder_position(self, axis: Axis) -> float:
        """Read raw encoder position (ENP). Distinct from ACP — use to detect lost steps."""
        return self._send(f"{self._ax(axis)} ENP")

    def set_encoder_position(self, axis: Axis, value: float) -> float:
        """Zero or offset the encoder position register."""
        return self._send(f"{self._ax(axis)} ENP {value:.6g}")

    def get_commanded_position(self, axis: Axis) -> float:
        return self._send(f"{self._ax(axis)} COP")

    def get_destination_position(self, axis: Axis) -> float:
        """Read the target position of the current or most recent move (DEP)."""
        return self._send(f"{self._ax(axis)} DEP")

    def set_speed(self, axis: Axis, value: float) -> float:
        return self._send(f"{self._ax(axis)} SPD {value:.6g}")

    def get_speed(self, axis: Axis) -> float:
        return self._send(f"{self._ax(axis)} SPD")

    def set_accel(self, axis: Axis, value: float) -> float:
        return self._send(f"{self._ax(axis)} ACL {value:.6g}")

    def get_accel(self, axis: Axis) -> float:
        return self._send(f"{self._ax(axis)} ACL")

    def set_decel(self, axis: Axis, value: float) -> float:
        return self._send(f"{self._ax(axis)} DCL {value:.6g}")

    def get_decel(self, axis: Axis) -> float:
        return self._send(f"{self._ax(axis)} DCL")

    def set_negative_limit(self, axis: Axis, value: float) -> float:
        """Set software negative travel limit (NLT). Motion beyond this raises an error."""
        return self._send(f"{self._ax(axis)} NLT {value:.6g}")

    def get_negative_limit(self, axis: Axis) -> float:
        return self._send(f"{self._ax(axis)} NLT")

    def set_positive_limit(self, axis: Axis, value: float) -> float:
        """Set software positive travel limit (PLT)."""
        return self._send(f"{self._ax(axis)} PLT {value:.6g}")

    def get_positive_limit(self, axis: Axis) -> float:
        return self._send(f"{self._ax(axis)} PLT")

    # ------------------------------------------------------------------
    # Single-axis motion
    # ------------------------------------------------------------------

    def begin_move_to(self, axis: Axis, position: float) -> None:
        """Non-blocking absolute move (BMT). Returns immediately; poll MIF to wait."""
        self._send(f"{self._ax(axis)} BMT {position:.6g}")

    def begin_move_by(self, axis: Axis, delta: float) -> None:
        """Non-blocking relative move (BMB)."""
        self._send(f"{self._ax(axis)} BMB {delta:.6g}")

    def move_to(self, axis: Axis, position: float) -> None:
        """Blocking absolute move (MVT). TCP response held until move completes."""
        self._send(f"{self._ax(axis)} MVT {position:.6g}")

    def move_by(self, axis: Axis, delta: float) -> None:
        """Blocking relative move (MVB)."""
        self._send(f"{self._ax(axis)} MVB {delta:.6g}")

    def jog(self, axis: Axis, speed: float) -> float:
        """Start continuous velocity motion at speed (JOG). Pass 0 to stop.

        Speed sign determines direction. Returns the axis speed after command.
        """
        return self._send(f"{self._ax(axis)} JOG {speed:.6g}")

    def begin_stop(self, axis: Axis) -> None:
        """Controlled deceleration stop (BST)."""
        self._send(f"{self._ax(axis)} BST")

    def abort(self, axis: Axis) -> None:
        """Immediate stop with no decel ramp (ABT). Use for emergencies."""
        self._send(f"{self._ax(axis)} ABT")

    def stop(self, axis: Axis) -> None:
        """Immediate stop (STP)."""
        self._send(f"{self._ax(axis)} STP")

    def move_is_finished(self, axis: Axis) -> bool:
        """Poll whether the axis move has completed (MIF)."""
        return bool(self._send(f"{self._ax(axis)} MIF"))

    # ------------------------------------------------------------------
    # Group motion (coordinated multi-axis)
    # ------------------------------------------------------------------

    def init_group(self, *axis_indices: int) -> None:
        """Initialize the coordinated group with specified axis indices (INI).

        Call once after connecting if the group hasn't been initialized in firmware.
        e.g. init_group(1, 2, 3) sets up a 3-axis XYZ group.
        """
        indices = " ".join(str(i) for i in axis_indices)
        self._send(f"{self._gx()} INI {indices}")

    def group_begin_move_to(self, *positions: float) -> None:
        """Non-blocking coordinated absolute move (BMT on group)."""
        self._send(f"{self._gx()} BMT {self._fmt_params(*positions)}")

    def group_begin_move_by(self, *deltas: float) -> None:
        """Non-blocking coordinated relative move (BMB on group)."""
        self._send(f"{self._gx()} BMB {self._fmt_params(*deltas)}")

    def group_move_to(self, *positions: float) -> None:
        """Blocking coordinated absolute move (MVT on group)."""
        self._send(f"{self._gx()} MVT {self._fmt_params(*positions)}")

    def group_move_by(self, *deltas: float) -> None:
        """Blocking coordinated relative move (MVB on group)."""
        self._send(f"{self._gx()} MVB {self._fmt_params(*deltas)}")

    def append_move_to(self, *positions: float) -> None:
        """Queue an absolute waypoint into the curve buffer (AMT).

        Must be called after group_begin_move_to to chain waypoints for
        smooth blended trajectory. The controller executes them in sequence.
        """
        self._send(f"{self._gx()} AMT {self._fmt_params(*positions)}")

    def append_move_by(self, *deltas: float) -> None:
        """Queue a relative waypoint into the curve buffer (AMB)."""
        self._send(f"{self._gx()} AMB {self._fmt_params(*deltas)}")

    def append_arc(self, radius: float, theta: float, phi: float, extra: Optional[float] = None) -> None:
        """Queue an arc segment (ARC). 3D arc takes radius, theta, phi, plus one extra param."""
        if extra is not None:
            self._send(f"{self._gx()} ARC {radius:.6g} {theta:.6g} {phi:.6g} {extra:.6g}")
        else:
            self._send(f"{self._gx()} ARC {radius:.6g} {theta:.6g} {phi:.6g}")

    def clear_curve_buffer(self) -> None:
        """Clear the group's queued waypoints (CLR)."""
        self._send(f"{self._gx()} CLR")

    def group_begin_stop(self) -> None:
        self._send(f"{self._gx()} BST")

    def group_abort(self) -> None:
        self._send(f"{self._gx()} ABT")

    def group_stop(self) -> None:
        self._send(f"{self._gx()} STP")

    def group_move_is_finished(self) -> bool:
        return bool(self._send(f"{self._gx()} MIF"))

    def group_set_speed(self, value: float) -> float:
        return self._send(f"{self._gx()} SPD {value:.6g}")

    def group_get_speed(self) -> float:
        return self._send(f"{self._gx()} SPD")

    def group_set_accel(self, value: float) -> float:
        return self._send(f"{self._gx()} ACL {value:.6g}")

    def group_set_decel(self, value: float) -> float:
        return self._send(f"{self._gx()} DCL {value:.6g}")

    def group_set_actual_position(self, *positions: float) -> None:
        """Zero or offset all group axes simultaneously (ACP on group)."""
        self._send(f"{self._gx()} ACP {self._fmt_params(*positions)}")

    # ------------------------------------------------------------------
    # Capture mechanism — precise limit-switch / event position latching
    # ------------------------------------------------------------------

    def set_capture_source(self, axis: Axis, source_index: int) -> None:
        """Configure which digital input index triggers capture for this axis (SCS).

        source_index corresponds to the INB index of the limit switch.
        Must be called before arm_capture.
        """
        self._send(f"{self._ax(axis)} SCS {source_index}")

    def set_capture_trip(self, axis: Axis, trip_on_high: bool) -> None:
        """Set capture trip polarity (SCT). True = trip on input HIGH."""
        val = 1 if trip_on_high else 0
        self._send(f"{self._ax(axis)} SCT {val}")

    def arm_capture(self, axis: Axis) -> None:
        """Arm the hardware capture latch (AIC).

        Once armed, the next input event on the configured source latches the
        current axis position into the capture register (read with get_capture_position).
        capture_has_tripped() returns True after the event; arm again to re-use.
        """
        self._send(f"{self._ax(axis)} AIC")

    def get_capture_bit(self, axis: Axis) -> bool:
        """Read the live state of the capture input (CAB). Not latched."""
        return bool(self._send(f"{self._ax(axis)} CAB"))

    def get_capture_position(self, axis: Axis) -> float:
        """Read the hardware-latched position at the moment of the capture event (CAP).

        This is more precise than polling actual_position because it is
        timestamped at the interrupt level rather than at the poll interval.
        """
        return self._send(f"{self._ax(axis)} CAP")

    def capture_has_tripped(self, axis: Axis) -> bool:
        """Return True if a capture event has occurred since last arm_capture (CAT)."""
        return bool(self._send(f"{self._ax(axis)} CAT"))

    # ------------------------------------------------------------------
    # Digital IO — brakes and general-purpose inputs/outputs
    # ------------------------------------------------------------------

    def read_input_bit(self, index: int) -> bool:
        """Read digital input by index 1–48 (INB). No axis prefix required.

        Used for: limit switch status, brake status, external sensors.
        Index mapping depends on controller wiring — see IOMap for defaults.
        """
        return bool(self._send(f"INB {index}"))

    def set_output_bit(self, index: int, state: bool) -> None:
        """Set digital output by index 1–12 (SOB).

        Used for: brake engagement/disengagement, indicator lights.
        """
        val = 1 if state else 0
        self._send(f"SOB {index} {val}")

    def read_iso_input(self, index: int) -> bool:
        """Read isolated IO input by index 1–18 (ISI). Uses IsoIO expansion module."""
        return bool(self._send(f"ISI {index}"))

    def set_iso_output(self, index: int, state: bool) -> None:
        """Set isolated IO output by index 1–8 (ISO)."""
        val = 1 if state else 0
        self._send(f"ISO {index} {val}")

    # ------------------------------------------------------------------
    # Brake control helpers (uses IOMap indices)
    # ------------------------------------------------------------------

    def disengage_brake(self, axis: Axis, io_map: IOMap) -> None:
        """Disengage the electromagnetic brake on Y or Z axis.

        Brake output ON = brake disengaged (spring-return design: power releases brake).
        """
        if axis is Y_AXIS:
            self.set_output_bit(io_map.y_brake_output, True)
        elif axis is Z_AXIS:
            self.set_output_bit(io_map.z_brake_output, True)

    def engage_brake(self, axis: Axis, io_map: IOMap) -> None:
        """Engage the electromagnetic brake on Y or Z axis."""
        if axis is Y_AXIS:
            self.set_output_bit(io_map.y_brake_output, False)
        elif axis is Z_AXIS:
            self.set_output_bit(io_map.z_brake_output, False)

    def brake_is_disengaged(self, axis: Axis, io_map: IOMap) -> bool:
        """Read brake feedback status. True = brake is currently disengaged (released)."""
        if axis is Y_AXIS:
            return self.read_input_bit(io_map.y_brake_status_input)
        elif axis is Z_AXIS:
            return self.read_input_bit(io_map.z_brake_status_input)
        return True  # axes without brakes are always "free"

    # ------------------------------------------------------------------
    # Full axis state snapshot
    # ------------------------------------------------------------------

    def read_axis_state(self, axis: Axis) -> AxisState:
        """Read all readable axis properties in one batch of queries."""
        return AxisState(
            actual_position=self.get_actual_position(axis),
            commanded_position=self.get_commanded_position(axis),
            destination_position=self.get_destination_position(axis),
            encoder_position=self.get_encoder_position(axis),
            speed=self.get_speed(axis),
            accel=self.get_accel(axis),
            decel=self.get_decel(axis),
            motor_on=self.get_motor(axis),
            enabled=self.get_enable(axis),
            move_is_finished=self.move_is_finished(axis),
            capture_bit=self.get_capture_bit(axis),
            capture_position=self.get_capture_position(axis),
            capture_has_tripped=self.capture_has_tripped(axis),
            negative_limit=self.get_negative_limit(axis),
            positive_limit=self.get_positive_limit(axis),
        )

    # ------------------------------------------------------------------
    # Startup sequence
    # ------------------------------------------------------------------

    def startup(self, axes: tuple[Axis, ...] = ALL_AXES) -> None:
        """Enable motors for the given axes.

        Does not disengage brakes — call disengage_brake separately before
        commanding motion on Y or Z, and re-engage after stopping.
        """
        for axis in axes:
            self.set_motor(axis, True)
            logger.debug("Motor enabled: %s", axis.name)

    def shutdown(self, axes: tuple[Axis, ...] = ALL_AXES, io_map: Optional[IOMap] = None) -> None:
        """Stop all motion, engage brakes, disable motors."""
        for axis in axes:
            try:
                self.abort(axis)
            except SnapMotionError:
                pass
        if io_map:
            for axis in (Y_AXIS, Z_AXIS):
                try:
                    self.engage_brake(axis, io_map)
                except SnapMotionError:
                    pass
        for axis in axes:
            try:
                self.set_motor(axis, False)
            except SnapMotionError:
                pass
