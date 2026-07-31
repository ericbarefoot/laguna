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

This machine has 8 physical axis slots split across two PLC nodes,
addressed transparently through the same grammar:
  - Commander (local controller), slots 1-4: X(1), Y(2), encoder(3), encoder(4)
  - Responder (second networked PLC node), slots 5-8: Z(5), Theta(6), encoder(7), encoder(8)
Axes 3/4/7/8 are internal encoder-only slots, not exposed/commandable motion
axes — confirmed via the vendor's developers and live hardware queries — so
this module does not model them as Axis objects or send A3/A4/A7/A8 commands.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional
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
# Move-completion polling
# ---------------------------------------------------------------------------
#
# DEBUG PATCH (branch debug/e415117-no-cross-node-group): confirmed on
# hardware 2026-07-31 that polling MIF in a tight loop while a coordinated
# group move is in flight destabilizes this controller. A C1 INI/BMT
# oscillation test polling C1 MIF every 0.05s failed reproducibly on its
# 6th move (~13s in) — the Pi-side agent logged a >5s serial read timeout
# on C1 MIF, i.e. the controller stopped answering the wire entirely, not
# a network/SSH fault. The identical test polling sparsely (one check near
# the predicted finish time, then every 0.5s) ran 60 moves over 142s with
# zero failures. Single-axis (A<n> BMT/MIF) moves polled at the same 0.05s
# never failed across 15 cycles, so the trigger is specifically
# high-frequency querying *during group interpolation*, not query rate
# alone and not motion alone.
#
# Every wait-for-move loop in this driver therefore sleeps through most of
# the move's predicted duration before its first query, then polls slowly.
# Both knobs are module-level so tests can zero them out (see
# tests/conftest.py) rather than sleeping in real time.
SPARSE_POLL_INTERVAL_S = 0.5

# Fraction of a move's predicted duration to sleep through before the first
# query. Below 1.0 so that a slightly-optimistic prediction (accel/decel
# ramps make real moves run longer than distance/speed) still lands the
# first poll before completion rather than long after it.
PREDICTED_SLEEP_FRACTION = 0.85


def predicted_move_s(distance: float, speed: Optional[float]) -> float:
    """Nominal duration of a move, for sparse move-completion polling.

    Deliberately ignores accel/decel ramps, which only make the real move
    take *longer* than this — combined with PREDICTED_SLEEP_FRACTION being
    below 1.0, that keeps the first poll safely before completion rather
    than after it. Returns 0.0 when the speed is unknown or non-positive
    (e.g. a G-code program that never specified an F word, so the
    controller is using whatever SPD it already had), which just means
    polling starts immediately — still at the sparse interval.

    Units only have to be consistent between the two arguments (mm and
    mm/s, or degrees and degrees/s for Theta).
    """
    if not speed or speed <= 0:
        return 0.0
    return abs(distance) / speed


def poll_until_move_finished(
    is_finished: Callable[[], bool],
    predicted_s: float = 0.0,
    timeout_s: float = 30.0,
) -> bool:
    """Wait for a move to finish, querying the controller as little as possible.

    Sleeps through PREDICTED_SLEEP_FRACTION of `predicted_s` before the
    first `is_finished()` call, then polls every SPARSE_POLL_INTERVAL_S.
    See this module's "Move-completion polling" note for why the tight
    polling this replaces is actively harmful on this hardware.

    Args:
        is_finished: Callable returning True once the move has completed —
            typically MMCCommands.move_is_finished/group_move_is_finished
            bound to an axis (each call is one MIF query on the wire).
        predicted_s: Expected move duration in seconds, if known (distance
            / speed at the call site). 0.0 means "no idea" — polling then
            starts immediately, still at the sparse interval.
        timeout_s: Give up after this long. The initial predicted sleep
            counts against it, and is clamped so it can never overshoot it.

    Returns:
        True if the move finished, False if `timeout_s` elapsed first. The
        caller decides what a timeout means (abort and raise, or warn and
        continue) — see the call sites in gcode.py/homing.py/controller.py.
    """
    deadline = time.monotonic() + timeout_s
    initial_sleep = max(0.0, min(predicted_s * PREDICTED_SLEEP_FRACTION, timeout_s))
    if initial_sleep > 0:
        time.sleep(initial_sleep)
    while True:
        if is_finished():
            return True
        if time.monotonic() > deadline:
            return False
        time.sleep(SPARSE_POLL_INTERVAL_S)


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


# X/Y are on the commander (local controller); Z/Theta are on the
# responder (second networked PLC node), addressed transparently through
# the same ASCII grammar. Confirmed via the vendor's developers and live
# hardware queries on 2026-07-17 (A5/A6 ACP returned real position values;
# A1/A2/A5 PLT returned sane soft-limit numbers rather than uninitialized
# garbage). Slots 3/4/7/8 are internal encoder-only slots on their
# respective nodes — not exposed/commandable motion axes — so they are not
# modeled as Axis objects here.
X_AXIS = Axis("X", 1)
Y_AXIS = Axis("Y", 2)
Z_AXIS = Axis("Z", 5)
THETA_AXIS = Axis("Theta", 6)

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
    """Digital IO channel mapping for brakes and home/limit switches.

    Earlier notes on this integration called these "IsoIO expansion-board
    channels" per the vendor's .dsm project files. That characterization was
    a misreading: the IsoIO board genuinely isn't installed on this machine
    (ISI/ISO commands fail with error 263), but the .dsm's own TNamedIO
    records for these signals use Type=1 (plain digital input) with
    ModuleNumber=16 ($10) — the vendor's own runtime (standard.inc) treats
    ModuleNumber=16 as the LOCAL/commander native input bus, not an IsoIO
    designation. So most of these were never IsoIO channels at all; they're
    commander-native INB/SOB reads, decoded directly from the .dsm's
    Named-IO block declarations (eab-2026-07-16.dsm and eab-2026-07-17.dsm
    agree exactly) and cross-checked against a live INB 1-8 read on
    2026-07-17 (values 1,1,1,0,1,1,0,0 — consistent with this table). Not
    yet physically toggle-tested switch-by-switch.

    Commander native IO decode (ModuleNumber=16 in both .dsm files):
      - x_home_input         = INB 1  (XXHome)
      - x_limit_input        = INB 2  (XXLim)
      - y_home_input         = INB 3  (YHome)
      - y_limit_input        = INB 4  (YLim)
      - z_home_input         = INB 5  (Zhome)
      - z_limit_input        = INB 6  (ZLim)
      - (INB 7 unused/spare in the .dsm)
      - y_brake_status_input = INB 8  (Y_Brake_Status)
      - y_brake_output       = SOB 4  (Y_Brake)
      - z_brake_output       = SOB 5  (Z_Brake)

    Two signals are the exception: Z_Brake_Status and TLim (Theta's limit
    switch) both have ModuleNumber=1 in the .dsm, not 16 — per the vendor's
    own local/remote rule, that means they live on the RESPONDER's own
    input bank (index 1 and 2 there), not the commander's — even though the
    output side of the Z brake (SOB 5) is still on the commander. Worse:
    there is no ASCII text command that reaches the responder's own inputs
    at all — confirmed by tracing the interpreter's dispatch code, INB is a
    flat, non-scoped call straight into the local InputBit() function with
    no axis/node prefix in the grammar. The only path to a remote node's IO
    in this firmware family is the GUI-configured Named IO block feature
    (which resolves ModuleNumber internally on the controller) or the
    separate, vendor-encrypted Binary Commands node protocol used for
    responder axis motion — neither is reachable from here. So
    z_brake_status_input and theta_limit_input default to None and are
    architecturally unimplemented, not just unprobed: brake_is_disengaged()
    raises NotImplementedError (not the usual ValueError) if asked to use
    them, until some other path to read the responder's IO is built.
    """
    y_brake_output: Optional[int] = 4    # SOB 4 — confirmed (eab-2026-07-16/17.dsm)
    z_brake_output: Optional[int] = 5    # SOB 5 — confirmed (eab-2026-07-16/17.dsm)
    y_brake_status_input: Optional[int] = 8   # INB 8 — confirmed (eab-2026-07-16/17.dsm)
    z_brake_status_input: Optional[int] = None  # on responder — unreachable via ASCII, see above

    x_home_input: Optional[int] = 1      # INB 1 — confirmed (eab-2026-07-16/17.dsm)
    x_limit_input: Optional[int] = 2     # INB 2 — confirmed (eab-2026-07-16/17.dsm)
    y_home_input: Optional[int] = 3      # INB 3 — confirmed (eab-2026-07-16/17.dsm)
    y_limit_input: Optional[int] = 4     # INB 4 — confirmed (eab-2026-07-16/17.dsm)
    z_home_input: Optional[int] = 5      # INB 5 — confirmed (eab-2026-07-16/17.dsm)
    z_limit_input: Optional[int] = 6     # INB 6 — confirmed (eab-2026-07-16/17.dsm)
    theta_limit_input: Optional[int] = None  # on responder — unreachable via ASCII, see above

    # Capture sources for the hardware capture-latch homing mechanism (SCS).
    # Homing is currently deprioritized — left unset until needed.
    x_limit_capture_source: Optional[int] = None
    y_limit_capture_source: Optional[int] = None
    z_limit_capture_source: Optional[int] = None


# ---------------------------------------------------------------------------
# Command interface
# ---------------------------------------------------------------------------

# DEBUG PATCH (branch debug/e415117-no-cross-node-group): blocking motion
# primitives (MVT/MVB, single-axis and group) are banned outright — they
# hold the wire's request/response round trip open until the firmware
# reports the physical move complete, for however long that takes,
# including axes that are already at the requested target (e.g.
# GantryController.move_to()'s old unconditional Theta-branch MVT, which
# fired on every vector move regardless of whether Theta had actually
# moved). Every caller now uses the non-blocking begin_move_to/begin_move_by
# (+ move_is_finished polling) instead, which never leaves a
# request/response pair open for an unbounded stretch — see also
# GCodeExecutor's Z/XY split, which was migrated the same way. No caller
# in this codebase used the blocking group forms at all.
_BLOCKING_MOTION_BANNED = (
    "{blocking}() is banned — it blocks on the wire until the physical move "
    "completes (or the read times out), for however long that takes, even "
    "for a zero-distance move to an already-current position. Use "
    "{nonblocking}() and poll move_is_finished()/group_move_is_finished() instead."
)


class MMCCommands:
    """Formats, sends, and parses all Snap2Motion ASCII commands.

    Every public method maps to one or a small sequence of ASCII commands.
    Methods that can both set and get follow the firmware convention: pass a
    value to set, omit it (or pass None) to read the current value.

    All position/velocity values are in real mm / mm/s — this class converts
    to/from the controller's raw ACP user units internally via mm_per_unit
    and coordinate_offset_mm (see docs/GANTRY_UNIT_CALIBRATION.md: on this
    hardware 1 raw unit = 15mm on X/Y/Z, not 1mm, until that's fixed at the
    Snap2Motion/DSM source). This is deliberately the single choke point for
    that conversion — every position/velocity-reading or -writing method
    below applies it, so callers never see raw units. Theta (rotary) is
    exempt — it has its own separate, already-correct conversion, unrelated
    to this finding.
    """

    def __init__(
        self,
        connection: SnapConnection,
        group_index: int = 1,
        mm_per_unit: float = 1.0,
        coordinate_offset_mm: Optional[Dict[str, float]] = None,
        group_axes: tuple[Axis, ...] = (X_AXIS, Y_AXIS, Z_AXIS),
    ):
        """
        Args:
            connection: Transport to send formatted ASCII commands over.
            group_index: Coordinated-group index (the `C<N>` prefix).
            mm_per_unit: Real mm per raw controller (ACP) unit, applied to
                every linear-axis (X/Y/Z) position/velocity value. Defaults
                to 1.0 (no conversion) — pass the value from
                config's `gantry.mm_per_acp_unit` to apply the
                docs/GANTRY_UNIT_CALIBRATION.md workaround. Not applied to
                Theta.
            coordinate_offset_mm: Optional {axis_name: offset_mm} real-mm
                translation from the gantry's raw zero to a real-world
                origin, applied to position (not velocity/delta) values for
                linear axes. Axes not present in the dict get 0.0.
            group_axes: The axes coordinated-group commands' positional
                arguments map to, in order — must match how the group was
                configured (e.g. via GCodeExecutor/GantryController). Used
                only to look up per-axis mm_per_unit/offset for group moves.
        """
        self._conn = connection
        self._group = group_index
        self._mm_per_unit = mm_per_unit
        self._offset = coordinate_offset_mm or {}
        self._group_axes = group_axes

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
    # Unit conversion — real mm <-> raw controller units (linear axes only)
    # ------------------------------------------------------------------

    @staticmethod
    def _is_linear(axis: Axis) -> bool:
        return axis.name != "Theta"

    def _pos_to_raw(self, axis: Axis, value_mm: float) -> float:
        """Real mm -> raw units for an absolute position (applies offset)."""
        if not self._is_linear(axis):
            return value_mm
        return (value_mm - self._offset.get(axis.name, 0.0)) / self._mm_per_unit

    def _pos_to_mm(self, axis: Axis, value_raw: float) -> float:
        """Raw units -> real mm for an absolute position (applies offset)."""
        if not self._is_linear(axis):
            return value_raw
        return value_raw * self._mm_per_unit + self._offset.get(axis.name, 0.0)

    def _delta_to_raw(self, axis: Axis, value_mm: float) -> float:
        """Real mm(/s) -> raw units for a relative delta/velocity/accel (no offset)."""
        if not self._is_linear(axis):
            return value_mm
        return value_mm / self._mm_per_unit

    def _delta_to_mm(self, axis: Axis, value_raw: float) -> float:
        """Raw units -> real mm(/s) for a relative delta/velocity/accel (no offset)."""
        if not self._is_linear(axis):
            return value_raw
        return value_raw * self._mm_per_unit

    def _group_pos_to_raw(self, *positions: float) -> tuple:
        return tuple(self._pos_to_raw(axis, v) for axis, v in zip(self._group_axes, positions))

    def _group_delta_to_raw(self, *deltas: float) -> tuple:
        return tuple(self._delta_to_raw(axis, v) for axis, v in zip(self._group_axes, deltas))

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
        """Read the axis stepper position tracker (ACP), in real mm."""
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} ACP"))

    def set_actual_position(self, axis: Axis, value: float) -> float:
        """Set/zero the actual position register (ACP), given real mm. Returns new value in mm."""
        raw = self._pos_to_raw(axis, value)
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} ACP {raw:.6g}"))

    def get_encoder_position(self, axis: Axis) -> float:
        """Read raw encoder position (ENP) in real mm. Distinct from ACP — use to detect lost steps."""
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} ENP"))

    def set_encoder_position(self, axis: Axis, value: float) -> float:
        """Zero or offset the encoder position register, given real mm."""
        raw = self._pos_to_raw(axis, value)
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} ENP {raw:.6g}"))

    def get_commanded_position(self, axis: Axis) -> float:
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} COP"))

    def get_destination_position(self, axis: Axis) -> float:
        """Read the target position of the current or most recent move (DEP), in real mm."""
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} DEP"))

    def set_speed(self, axis: Axis, value: float) -> float:
        raw = self._delta_to_raw(axis, value)
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} SPD {raw:.6g}"))

    def get_speed(self, axis: Axis) -> float:
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} SPD"))

    def set_accel(self, axis: Axis, value: float) -> float:
        raw = self._delta_to_raw(axis, value)
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} ACL {raw:.6g}"))

    def get_accel(self, axis: Axis) -> float:
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} ACL"))

    def set_decel(self, axis: Axis, value: float) -> float:
        raw = self._delta_to_raw(axis, value)
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} DCL {raw:.6g}"))

    def get_decel(self, axis: Axis) -> float:
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} DCL"))

    def set_negative_limit(self, axis: Axis, value: float) -> float:
        """Set software negative travel limit (NLT), given real mm. Motion beyond this raises an error."""
        raw = self._pos_to_raw(axis, value)
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} NLT {raw:.6g}"))

    def get_negative_limit(self, axis: Axis) -> float:
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} NLT"))

    def set_positive_limit(self, axis: Axis, value: float) -> float:
        """Set software positive travel limit (PLT), given real mm."""
        raw = self._pos_to_raw(axis, value)
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} PLT {raw:.6g}"))

    def get_positive_limit(self, axis: Axis) -> float:
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} PLT"))

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
        """Non-blocking absolute move (BMT), given real mm. Returns immediately; poll MIF to wait."""
        raw = self._pos_to_raw(axis, position)
        self._send(f"{self._ax(axis)} BMT {raw:.6g}")

    def begin_move_by(self, axis: Axis, delta: float) -> None:
        """Non-blocking relative move (BMB), given real mm."""
        raw = self._delta_to_raw(axis, delta)
        self._send(f"{self._ax(axis)} BMB {raw:.6g}")

    def move_to(self, axis: Axis, position: float) -> None:
        """Banned — see _BLOCKING_MOTION_BANNED."""
        raise RuntimeError(_BLOCKING_MOTION_BANNED.format(blocking="move_to", nonblocking="begin_move_to"))

    def move_by(self, axis: Axis, delta: float) -> None:
        """Banned — see _BLOCKING_MOTION_BANNED."""
        raise RuntimeError(_BLOCKING_MOTION_BANNED.format(blocking="move_by", nonblocking="begin_move_by"))

    def jog(self, axis: Axis, speed: float) -> float:
        """Start continuous velocity motion at speed (mm/s) (JOG). Pass 0 to stop.

        Speed sign determines direction. Returns the axis speed (mm/s) after command.
        """
        raw = self._delta_to_raw(axis, speed)
        return self._delta_to_mm(axis, self._send(f"{self._ax(axis)} JOG {raw:.6g}"))

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
        """Non-blocking coordinated absolute move (BMT on group), given real mm."""
        self._send(f"{self._gx()} BMT {self._fmt_params(*self._group_pos_to_raw(*positions))}")

    def group_begin_move_by(self, *deltas: float) -> None:
        """Non-blocking coordinated relative move (BMB on group), given real mm."""
        self._send(f"{self._gx()} BMB {self._fmt_params(*self._group_delta_to_raw(*deltas))}")

    def group_move_to(self, *positions: float) -> None:
        """Banned — see _BLOCKING_MOTION_BANNED."""
        raise RuntimeError(
            _BLOCKING_MOTION_BANNED.format(blocking="group_move_to", nonblocking="group_begin_move_to")
        )

    def group_move_by(self, *deltas: float) -> None:
        """Banned — see _BLOCKING_MOTION_BANNED."""
        raise RuntimeError(
            _BLOCKING_MOTION_BANNED.format(blocking="group_move_by", nonblocking="group_begin_move_by")
        )

    def append_move_to(self, *positions: float) -> None:
        """Queue an absolute waypoint into the curve buffer (AMT), given real mm.

        Must be called after group_begin_move_to to chain waypoints for
        smooth blended trajectory. The controller executes them in sequence.
        """
        self._send(f"{self._gx()} AMT {self._fmt_params(*self._group_pos_to_raw(*positions))}")

    def append_move_by(self, *deltas: float) -> None:
        """Queue a relative waypoint into the curve buffer (AMB), given real mm."""
        self._send(f"{self._gx()} AMB {self._fmt_params(*self._group_delta_to_raw(*deltas))}")

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
        """Set the group's coordinated move speed, given real mm/s.

        Converted using the group's first axis — coordinated speed is a
        single scalar shared by all group axes, so this assumes uniform
        mm_per_unit across the group (true for this hardware: X/Y/Z all
        measured at the same 15 mm/unit ratio, see
        docs/GANTRY_UNIT_CALIBRATION.md).
        """
        raw = self._delta_to_raw(self._group_axes[0], value) if self._group_axes else value
        result = self._send(f"{self._gx()} SPD {raw:.6g}")
        return self._delta_to_mm(self._group_axes[0], result) if self._group_axes else result

    def group_get_speed(self) -> float:
        result = self._send(f"{self._gx()} SPD")
        return self._delta_to_mm(self._group_axes[0], result) if self._group_axes else result

    def group_set_accel(self, value: float) -> float:
        raw = self._delta_to_raw(self._group_axes[0], value) if self._group_axes else value
        result = self._send(f"{self._gx()} ACL {raw:.6g}")
        return self._delta_to_mm(self._group_axes[0], result) if self._group_axes else result

    def group_set_decel(self, value: float) -> float:
        raw = self._delta_to_raw(self._group_axes[0], value) if self._group_axes else value
        result = self._send(f"{self._gx()} DCL {raw:.6g}")
        return self._delta_to_mm(self._group_axes[0], result) if self._group_axes else result

    def group_set_actual_position(self, *positions: float) -> None:
        """Zero or offset all group axes simultaneously (ACP on group), given real mm."""
        self._send(f"{self._gx()} ACP {self._fmt_params(*self._group_pos_to_raw(*positions))}")

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
        """Read the hardware-latched position at the moment of the capture event (CAP), in real mm.

        This is more precise than polling actual_position because it is
        timestamped at the interrupt level rather than at the poll interval.
        """
        return self._pos_to_mm(axis, self._send(f"{self._ax(axis)} CAP"))

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
                raise NotImplementedError(
                    "Z brake status lives on the responder node's own input bank "
                    "(TNamedIO ModuleNumber=1, index 1 in eab-2026-07-16/17.dsm) and "
                    "is not reachable via the plain ASCII INB command from the "
                    "commander — there is no node-scoped addressing in this "
                    "firmware's ASCII grammar. Needs Named IO GUI config or the "
                    "Binary Commands node protocol to expose this value; see "
                    "docs/MACRON_GANTRY.md."
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
