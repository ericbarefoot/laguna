"""Typed ASCII command interface for the Snap2Motion MMC/OEM controller.

Command token format (verified against the vendor's shipped ASCII-interpreter
source, extracted from the compiled help/reference DSM, and independently
against hardware-tested code already deployed on the lab's bridge Pi):

  Single-axis: A<N> CMD [params...]   e.g. "A1 ACP" to read axis 1 position
  Group:       C<N> CMD [params...]   e.g. "C1 BMT 100.0 200.0 5.0"
  Global:      CMD [params...]        e.g. "INB 3" to read input bit 3

Whitespace between the axis/group prefix and the command, and between
parameters, is optional (the firmware's tokenizer recognizes a fixed
grammar rather than splitting on delimiters) — this module always emits a
single space for readability.

Positions and velocities are in whatever user units the controller is
configured for. If CountsPerUserUnit is set to the correct belt-pitch
conversion on the controller, commands are effectively in mm and mm/s.

This machine has 8 real axes, not 4: the local controller drives X(1)/Y(2)/
Z(3)/Theta(4), and a second networked "Responder" PLC node adds 4 more
(5-8), addressed transparently through the same grammar. Names/roles for
axes 5-8 are not yet known — see AXIS_5..AXIS_8 below.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import logging

from .connection import SnapConnection, SnapMotionError

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Token format — confirmed against the vendor's ASCII interpreter source
# ---------------------------------------------------------------------------
AXIS_TOKEN_FMT = "A{n}"   # single-axis commands
GROUP_TOKEN_FMT = "C{n}"  # coordinated-group commands

# Uninitialized/garbage software limits observed on this hardware are large
# (~±8.2e8); anything beyond this threshold is treated as "not really set".
GARBAGE_LIMIT_THRESHOLD = 1e6


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


# Local controller axes, matching the user's current project files
# (eab-2026-07-16.dsm, 600011-00-eab4.dsm): X=1, Y=2, Z=3, Theta=4.
X_AXIS = Axis("X", 1)
Y_AXIS = Axis("Y", 2)
Z_AXIS = Axis("Z", 3)
THETA_AXIS = Axis("Theta", 4)

# Axes 5-8 live on a second, physically-present networked "Responder" PLC
# node (DistributedAxis[5..8] in 600011-00-eab4.dsm) but are addressed
# through the exact same ASCII grammar as the local axes. Their real-world
# names/roles are not yet known — rename these once the user identifies
# what they drive. Not included in ALL_AXES (the local group's default) so
# existing startup/shutdown/group behavior is unaffected until then.
AXIS_5 = Axis("Axis5", 5)
AXIS_6 = Axis("Axis6", 6)
AXIS_7 = Axis("Axis7", 7)
AXIS_8 = Axis("Axis8", 8)

ALL_AXES = (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)
RESPONDER_AXES = (AXIS_5, AXIS_6, AXIS_7, AXIS_8)

# A coordinated group (C<n>INI) can span at most 6 axes, so the local 4-axis
# group and the 4 Responder axes cannot be combined into a single group.


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
    """Digital IO channel mapping for brakes and home/limit switches.

    The vendor's Snap2Motion project files (both the older demo program and
    the current production/jog-test files) declare these signals as IsoIO
    expansion-board channels — but that board is not physically installed
    on this machine (ISI/ISO commands fail with error 263). The user has
    since physically rewired all of these signals onto the controller's
    native INB/SOB bus instead; the project files were never updated to
    reflect this, so do NOT reuse any IsoIO channel number from a .dsm file
    here.

    Only two channels are currently confirmed, both native, both from
    eab-2026-07-16.dsm (treated as authoritative over the vendor demo file
    and 600011-00-eab4.dsm where they disagree):
      - z_brake_status_input = INB 1
      - theta_limit_input    = INB 2
    Every other field defaults to None and must be physically probed
    (toggle each switch/output, diff INB/SOB snapshots) before use — see
    sandbox/probe_gantry.py. Brake-control methods below raise ValueError
    if asked to use a channel that hasn't been set.
    """
    y_brake_output: Optional[int] = None
    z_brake_output: Optional[int] = None
    y_brake_status_input: Optional[int] = None
    z_brake_status_input: Optional[int] = 1   # INB 1 — confirmed (eab-2026-07-16.dsm)

    x_home_input: Optional[int] = None
    x_limit_input: Optional[int] = None
    y_home_input: Optional[int] = None
    y_limit_input: Optional[int] = None
    z_home_input: Optional[int] = None
    z_limit_input: Optional[int] = None
    theta_limit_input: Optional[int] = 2      # INB 2 — confirmed (eab-2026-07-16.dsm)

    # Capture sources for the hardware capture-latch homing mechanism (SCS).
    # Homing is currently deprioritized — left unset until needed.
    x_limit_capture_source: Optional[int] = None
    y_limit_capture_source: Optional[int] = None
    z_limit_capture_source: Optional[int] = None


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

    def validate_soft_limits(
        self, axes: tuple[Axis, ...] = ALL_AXES
    ) -> dict[str, tuple[float, float]]:
        """Read PLT/NLT for each axis and flag any that look uninitialized/garbage.

        Returns {axis_name: (negative_limit, positive_limit)} for every axis
        checked. Raises SnapMotionError(0) naming any axes whose limits
        exceed GARBAGE_LIMIT_THRESHOLD in magnitude — on this hardware, X
        and Y are known to currently have uninitialized limits (~±8.2e8),
        meaning software position limiting is NOT active for them. Callers
        that need to guarantee soft-limit protection before allowing motion
        should catch this and refuse to proceed rather than ignore it.
        """
        results: dict[str, tuple[float, float]] = {}
        bad: list[str] = []
        for axis in axes:
            neg = self.get_negative_limit(axis)
            pos = self.get_positive_limit(axis)
            results[axis.name] = (neg, pos)
            if abs(neg) > GARBAGE_LIMIT_THRESHOLD or abs(pos) > GARBAGE_LIMIT_THRESHOLD:
                bad.append(axis.name)
        if bad:
            raise SnapMotionError(
                0,
                f"Uninitialized/garbage software position limits on axes: {', '.join(bad)}. "
                "Soft-limit protection is NOT active for these axes until PLT/NLT are set "
                "to real values.",
            )
        return results

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

        Call once after connecting if the group hasn't been initialized in
        firmware. e.g. init_group(1, 2, 3) sets up a 3-axis XYZ group. A
        group can hold at most 6 axes.
        """
        if len(axis_indices) > 6:
            raise ValueError(
                f"A coordinated group can hold at most 6 axes, got {len(axis_indices)}"
            )
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
        """Read native digital input by index (INB). No axis prefix required.

        Used for: limit switch status, brake status, external sensors. This
        hardware's native input bus has only 8 bits (INB 9+ raises error 31,
        despite generic docs allowing up to 48) — index mapping depends on
        controller wiring, see IOMap for the confirmed/unconfirmed channels.
        """
        return bool(self._send(f"INB {index}"))

    def set_output_bit(self, index: int, state: bool) -> None:
        """Set native digital output by index (SOB).

        Used for: brake engagement/disengagement, indicator lights.
        """
        val = 1 if state else 0
        self._send(f"SOB {index} {val}")

    def read_iso_input(self, index: int) -> bool:
        """Read isolated IO input by index 1–18 (ISI). Uses IsoIO expansion module.

        This machine does not have the IsoIO board installed — calling this
        will raise SnapMotionError(263). Kept for other Snap2Motion hardware
        that does have the board; not used elsewhere in this codebase for
        this machine's brakes/limit switches (those are wired to native
        INB/SOB — see IOMap).
        """
        return bool(self._send(f"ISI {index}"))

    def set_iso_output(self, index: int, state: bool) -> None:
        """Set isolated IO output by index 1–8 (ISO).

        This machine does not have the IsoIO board installed — calling this
        will raise SnapMotionError(263). See read_iso_input.
        """
        val = 1 if state else 0
        self._send(f"ISO {index} {val}")

    # ------------------------------------------------------------------
    # Brake control helpers (uses IOMap indices)
    # ------------------------------------------------------------------

    def disengage_brake(self, axis: Axis, io_map: IOMap) -> None:
        """Disengage the electromagnetic brake on Y or Z axis.

        Brake output ON = brake disengaged (spring-return design: power
        releases brake). Raises ValueError if the relevant IOMap channel
        hasn't been set yet (physical probing required — see IOMap).
        """
        if axis == Y_AXIS:
            if io_map.y_brake_output is None:
                raise ValueError("io_map.y_brake_output is not set — probe the native SOB channel first")
            self.set_output_bit(io_map.y_brake_output, True)
        elif axis == Z_AXIS:
            if io_map.z_brake_output is None:
                raise ValueError("io_map.z_brake_output is not set — probe the native SOB channel first")
            self.set_output_bit(io_map.z_brake_output, True)

    def engage_brake(self, axis: Axis, io_map: IOMap) -> None:
        """Engage the electromagnetic brake on Y or Z axis.

        Raises ValueError if the relevant IOMap channel hasn't been set yet.
        """
        if axis == Y_AXIS:
            if io_map.y_brake_output is None:
                raise ValueError("io_map.y_brake_output is not set — probe the native SOB channel first")
            self.set_output_bit(io_map.y_brake_output, False)
        elif axis == Z_AXIS:
            if io_map.z_brake_output is None:
                raise ValueError("io_map.z_brake_output is not set — probe the native SOB channel first")
            self.set_output_bit(io_map.z_brake_output, False)

    def brake_is_disengaged(self, axis: Axis, io_map: IOMap) -> bool:
        """Read brake feedback status. True = brake is currently disengaged (released).

        Raises ValueError if the relevant IOMap channel hasn't been set yet
        (this is currently the case for Y — only Z's status input is
        confirmed on this hardware).
        """
        if axis == Y_AXIS:
            if io_map.y_brake_status_input is None:
                raise ValueError(
                    "io_map.y_brake_status_input is not set — probe the native INB channel first"
                )
            return self.read_input_bit(io_map.y_brake_status_input)
        elif axis == Z_AXIS:
            if io_map.z_brake_status_input is None:
                raise ValueError(
                    "io_map.z_brake_status_input is not set — probe the native INB channel first"
                )
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
                except (SnapMotionError, ValueError):
                    pass
        for axis in axes:
            try:
                self.set_motor(axis, False)
            except SnapMotionError:
                pass
