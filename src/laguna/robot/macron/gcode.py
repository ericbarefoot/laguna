"""G-code parsing and fence-checked execution for the macron gantry.

Supported G/M codes:
  G0, G1   — rapid / linear moves (treated identically; no distinct "rapid"
             speed concept in the underlying protocol)
  G2, G3   — arcs (clockwise / counter-clockwise), specified with I/J center
             offsets or an R radius. Tessellated into short linear segments
             at parse time — see _tessellate_arc for why.
  G4       — dwell (P milliseconds or S seconds)
  G28      — home (drives HomingProcedure.home_all(); no per-axis selection
             yet — always homes everything HomingProcedure is configured for)
  G90/G91  — absolute / relative distance mode
  G21      — millimeters (asserted; this is the only unit this driver uses)
  G20      — inches — NOT supported, raises GCodeError
  M0, M1   — program pause (delegates to confirm_cb; see GCodeExecutor)
  M114     — position query — accepted as a no-op here; callers should read
             position via MMCCommands.read_axis_state() directly instead

E and S words (extruder / spindle, common in slicer output) are accepted
and ignored wherever they appear. Any other G/M code raises GCodeError
naming it, rather than silently skipping potentially-motion-relevant
instructions.

Arc handling note: the vendor's native ARC command (radius/theta/phi
parameters, see MMCCommands.append_arc) has never been confirmed against
real hardware — the exact parameter semantics are ambiguous in the
extracted documentation. Rather than guess at an unverified wire format,
G2/G3 arcs are tessellated here into a sequence of short straight-line
moves, each individually fence-checked and executed as an ordinary
coordinated group move. This fully supports arc G-code without sending any
command whose semantics aren't verified.

Z/XY node split: confirmed on hardware that the coordinated-group command
cannot include Z — `C1 INI 1 2` (X, Y — the commander) succeeds, `C1 INI 1
2 5` (adding Z — the responder, a separate networked PLC node) fails with
error 1010. So a LINEAR move that changes both Z and X/Y can't be sent as
one simultaneous 3D move at all on this hardware; _split_cross_node_moves
(run in GCodeExecutor.plan(), before fence-checking) splits any such move
into a Z-only leg followed by an XY-only leg, Z-first. Z-first was chosen
for this gantry's actual use (subtractive CNC / sensor positioning, not
additive/layer deposition) — Z reaches its target (e.g. retracting/
diving to a probe height) before XY travels, rather than the other way
round. This means a G-code program authored for genuine simultaneous 3D
motion (e.g. a helical G2/G3, or slicer layer-change lines combining an
XY travel with a Z step) is only ever approximated here as a Z-then-XY
zigzag, and the two legs' timing no longer reflects the original line's
vector feed rate (each leg runs at the full nominal F, not a fraction of
it) — this is a hard hardware limitation, not a bug to route around.
Splitting happens before the fence check specifically so the checked
waypoints match the path that is actually executed (an L-shaped path, not
the nominal diagonal) — see _split_cross_node_moves.
"""

from __future__ import annotations

import logging
import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from .commands import Axis, MMCCommands, X_AXIS, Y_AXIS, Z_AXIS
from .connection import SnapMotionError
from .fences import CheckedTrajectory, FenceViolation, Point3D, TrajectoryChecker
from .homing import HomingProcedure

logger = logging.getLogger(__name__)


class GCodeError(Exception):
    """Raised for G-code this parser/executor doesn't support or can't resolve."""


class GCodeExecutionAborted(Exception):
    """Raised when confirm_cb rejects a motion or control segment during execute()."""


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

_COMMENT_PAREN = re.compile(r"\([^)]*\)")
_COMMENT_SEMI = re.compile(r";.*$")
_WORD_RE = re.compile(r"([A-Za-z])\s*([+-]?[0-9]*\.?[0-9]+)")


def _strip_comments(line: str) -> str:
    """Remove `(...)` and `;`-to-end-of-line G-code comments, and trailing/leading whitespace."""
    line = _COMMENT_PAREN.sub(" ", line)
    line = _COMMENT_SEMI.sub("", line)
    return line.strip()


def _parse_words(line: str) -> Dict[str, float]:
    """Split a comment-stripped G-code line into {letter: value} words, e.g. {'G': 1, 'X': 10.0}.

    Letters are uppercased; a line with the same letter repeated keeps only
    the last occurrence (dict construction overwrites earlier keys).
    """
    return {letter.upper(): float(number) for letter, number in _WORD_RE.findall(line)}


@dataclass
class GCodeMove:
    """One resolved, ready-to-execute step of a parsed G-code program."""

    kind: str  # "LINEAR" | "HOME" | "DWELL" | "PAUSE"
    target: Optional[Point3D] = None
    feed_mm_s: Optional[float] = None
    dwell_s: Optional[float] = None
    source_line: str = ""


@dataclass
class GCodeProgram:
    """An ordered sequence of resolved moves, ready for fence-checking/execution."""

    moves: List[GCodeMove] = field(default_factory=list)

    def to_waypoints(self, start: Point3D) -> List[Point3D]:
        """Expand this program into the flat XYZ waypoint list TrajectoryChecker expects.

        Only LINEAR and HOME moves contribute waypoints (DWELL/PAUSE don't
        move). G28's landing point is assumed to be the conventional origin
        for pre-flight checking purposes — the real homed position is
        whatever HomingProcedure actually finds, but this is only used to
        validate what happens *after* the G28 in the same program.
        """
        waypoints = [start]
        pos = start
        for move in self.moves:
            if move.kind == "LINEAR" and move.target is not None:
                waypoints.append(move.target)
                pos = move.target
            elif move.kind == "HOME":
                pos = move.target if move.target is not None else pos
                waypoints.append(pos)
        return waypoints


# Tolerance for deciding whether a position actually changed. Needed
# because mm values round-trip through raw controller units and back (see
# MMCCommands._pos_to_raw/_pos_to_mm, formatted to 6 significant figures
# on the wire) — exact float equality is too fragile against that
# rounding noise to reliably tell "moved" from "reported the same spot
# back with a rounding wobble."
_POSITION_EPSILON_MM = 1e-3


def _moved(a: float, b: float) -> bool:
    return abs(a - b) > _POSITION_EPSILON_MM


def _split_cross_node_moves(moves: List[GCodeMove], start: Point3D) -> List[GCodeMove]:
    """Split any LINEAR move that changes both Z and X/Y into a Z-only leg
    followed by an XY-only leg (Z-first) — see the module docstring's
    "Z/XY node split" note for why this is necessary on this hardware.

    Moves that only change Z, or only change X/Y (the overwhelming common
    case — most G-code, including slicer output, only combines the two on
    a layer-change-style line), pass through unchanged. HOME/DWELL/PAUSE
    moves are untouched.
    """
    result: List[GCodeMove] = []
    pos = start
    for move in moves:
        if move.kind != "LINEAR" or move.target is None:
            result.append(move)
            continue
        tx, ty, tz = move.target
        px, py, pz = pos
        if _moved(tz, pz) and (_moved(tx, px) or _moved(ty, py)):
            result.append(GCodeMove(
                kind="LINEAR", target=(px, py, tz),
                feed_mm_s=move.feed_mm_s, source_line=move.source_line,
            ))
            result.append(GCodeMove(
                kind="LINEAR", target=(tx, ty, tz),
                feed_mm_s=move.feed_mm_s, source_line=move.source_line,
            ))
        else:
            result.append(move)
        pos = move.target
    return result


def _radius_to_ij(start: Point3D, end: Point3D, radius: float, clockwise: bool) -> Tuple[float, float]:
    """Convert G2/G3 R<radius> form into I/J center offsets from `start`."""
    x1, y1 = start[0], start[1]
    x2, y2 = end[0], end[1]
    dx, dy = x2 - x1, y2 - y1
    chord = math.hypot(dx, dy)
    if chord == 0:
        raise GCodeError("G2/G3 with R form requires distinct start/end points")
    if abs(radius) < chord / 2:
        raise GCodeError("G2/G3 radius is too small to reach the requested end point")
    h = math.sqrt(max(radius * radius - (chord / 2) ** 2, 0.0))
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    ux, uy = -dy / chord, dx / chord  # unit vector perpendicular to the chord
    sign = 1 if (radius > 0) == clockwise else -1
    cx = mx + sign * h * ux
    cy = my + sign * h * uy
    return cx - x1, cy - y1


def _tessellate_arc(
    start: Point3D,
    end: Point3D,
    i: float,
    j: float,
    clockwise: bool,
    max_chord_error: float = 0.05,
) -> List[Point3D]:
    """Tessellate an XY-plane circular arc into short line segments.

    i, j are the offsets from `start` to the arc center (standard G2/G3
    semantics). Z is linearly interpolated across the arc (helical
    support). Segment count is chosen via the classic sagitta estimate so
    the chord never deviates from the true arc by more than
    max_chord_error (mm).
    """
    cx = start[0] + i
    cy = start[1] + j
    radius = math.hypot(i, j)
    if radius <= 0:
        raise GCodeError("G2/G3 arc has zero radius (I/J both 0)")

    start_angle = math.atan2(start[1] - cy, start[0] - cx)
    end_angle = math.atan2(end[1] - cy, end[0] - cx)

    if clockwise:
        while end_angle >= start_angle:
            end_angle -= 2 * math.pi
    else:
        while end_angle <= start_angle:
            end_angle += 2 * math.pi

    sweep = end_angle - start_angle
    if radius > max_chord_error:
        max_angle_step = 2 * math.acos(1 - max_chord_error / radius)
    else:
        max_angle_step = math.pi / 8
    n_segments = max(1, math.ceil(abs(sweep) / max_angle_step))

    points: List[Point3D] = []
    for step in range(1, n_segments + 1):
        t = step / n_segments
        angle = start_angle + sweep * t
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        z = start[2] + (end[2] - start[2]) * t
        points.append((x, y, z))
    return points


class GCodeParser:
    """Parses the G-code subset documented in the module docstring."""

    def __init__(self) -> None:
        """Create a parser with fresh state (absolute mode, no feed rate set).

        A GCodeParser instance is stateful across the lines of a single
        parse() call (distance mode, current position, last feed rate),
        but that state is reset at the start of every parse() call, so a
        single instance can safely be reused for multiple programs.
        """
        self._absolute = True
        self._position: Point3D = (0.0, 0.0, 0.0)
        self._feed_mm_s: Optional[float] = None

    def parse(self, text: str, start: Point3D = (0.0, 0.0, 0.0)) -> GCodeProgram:
        """Parse a multi-line G-code program into a GCodeProgram of resolved moves.

        Resets parser state (absolute mode, feed rate) before parsing, then
        processes the text line by line, tracking position/mode as it goes
        so that relative moves, unspecified axes, and carried-over feed
        rates all resolve correctly.

        Args:
            text: Raw G-code source, one instruction per line.
            start: Starting XYZ position moves are resolved relative/
                absolute to (default origin).

        Returns:
            A GCodeProgram containing one GCodeMove per resolved motion/
            control instruction (arcs expand into multiple LINEAR moves).

        Raises:
            GCodeError: If any line uses an unsupported G/M code, or a
                supported code with invalid/missing parameters. The error
                is re-raised with the offending line number and text
                prepended.
        """
        self._absolute = True
        self._position = start
        self._feed_mm_s = None
        moves: List[GCodeMove] = []

        for lineno, raw_line in enumerate(text.splitlines(), start=1):
            line = _strip_comments(raw_line)
            if not line:
                continue
            words = _parse_words(line)
            try:
                moves.extend(self._handle_line(words, raw_line))
            except GCodeError as exc:
                raise GCodeError(f"line {lineno}: {exc} ({raw_line.strip()!r})") from exc

        return GCodeProgram(moves=moves)

    # -- dispatch --------------------------------------------------------

    def _handle_line(self, words: Dict[str, float], raw_line: str) -> List[GCodeMove]:
        """Dispatch one line's parsed words to the matching `_g<N>`/`_m<N>` handler method.

        Args:
            words: Parsed {letter: value} words for the line (from
                _parse_words). A "G" or "M" word selects the handler; a
                line with neither produces no moves.
            raw_line: Original line text, threaded through into any
                resulting GCodeMove.source_line / error message.

        Returns:
            Zero or more GCodeMove objects produced by the handler.

        Raises:
            GCodeError: If the line specifies a G/M code with no matching
                `_g<N>`/`_m<N>` method (i.e. an unsupported code).
        """
        if "G" in words:
            code = int(words["G"])
            handler = getattr(self, f"_g{code}", None)
            if handler is None:
                raise GCodeError(f"unsupported G-code: G{code}")
            return handler(words, raw_line)
        if "M" in words:
            code = int(words["M"])
            handler = getattr(self, f"_m{code}", None)
            if handler is None:
                raise GCodeError(f"unsupported M-code: M{code}")
            return handler(words, raw_line)
        return []  # blank/comment-only line, or a bare parameter continuation

    # -- shared helpers ----------------------------------------------------

    def _resolve_target(self, words: Dict[str, float]) -> Point3D:
        """Resolve a move's target XYZ from the current position, distance mode, and given words.

        In absolute mode (G90), an axis word gives its new coordinate
        directly; any axis not mentioned in `words` stays at its current
        value. In relative mode (G91), an axis word is a delta added to
        the current position; an omitted axis contributes zero.
        """
        x, y, z = self._position
        if self._absolute:
            x = words.get("X", x)
            y = words.get("Y", y)
            z = words.get("Z", z)
        else:
            x = x + words.get("X", 0.0)
            y = y + words.get("Y", 0.0)
            z = z + words.get("Z", 0.0)
        return (x, y, z)

    def _resolve_feed(self, words: Dict[str, float]) -> Optional[float]:
        """Update and return the parser's carried-over feed rate in mm/s.

        G-code feed rate (F word) is conventionally mm/min; this converts
        to mm/s on the way in. If `words` has no F word, the previously
        set feed rate (from an earlier line) is returned unchanged — feed
        rate persists across moves until explicitly overridden, per
        standard G-code semantics.
        """
        if "F" in words:
            self._feed_mm_s = words["F"] / 60.0  # G-code feed rate is mm/min
        return self._feed_mm_s

    # -- G-codes -------------------------------------------------------

    def _g0(self, words, raw_line):
        """G0 (rapid move) — treated identically to G1; see module docstring."""
        return self._linear_move(words, raw_line)

    def _g1(self, words, raw_line):
        """G1 (linear move) — resolve target/feed and emit a single LINEAR move."""
        return self._linear_move(words, raw_line)

    def _linear_move(self, words: Dict[str, float], raw_line: str) -> List[GCodeMove]:
        """Shared G0/G1 implementation: resolve target and feed, advance position, emit one move."""
        target = self._resolve_target(words)
        feed = self._resolve_feed(words)
        self._position = target
        return [GCodeMove(kind="LINEAR", target=target, feed_mm_s=feed, source_line=raw_line)]

    def _g2(self, words, raw_line):
        """G2 (clockwise arc) — tessellate and emit as a series of LINEAR moves."""
        return self._arc_move(words, raw_line, clockwise=True)

    def _g3(self, words, raw_line):
        """G3 (counter-clockwise arc) — tessellate and emit as a series of LINEAR moves."""
        return self._arc_move(words, raw_line, clockwise=False)

    def _arc_move(self, words: Dict[str, float], raw_line: str, clockwise: bool) -> List[GCodeMove]:
        """Shared G2/G3 implementation.

        Resolves the arc's end point and center (from I/J offsets or an R
        radius via _radius_to_ij), tessellates it into short line segments
        (see _tessellate_arc / module docstring for why arcs are never sent
        as a single vendor ARC command), and emits one LINEAR GCodeMove per
        segment, all sharing this line's feed rate.

        Raises:
            GCodeError: If neither I/J nor R is given, or if the R form's
                radius can't reach the requested end point (see
                _radius_to_ij), or if the resolved center gives a
                zero-radius arc (see _tessellate_arc).
        """
        start = self._position
        end = self._resolve_target(words)
        feed = self._resolve_feed(words)
        if "I" in words or "J" in words:
            i = words.get("I", 0.0)
            j = words.get("J", 0.0)
        elif "R" in words:
            i, j = _radius_to_ij(start, end, words["R"], clockwise)
        else:
            raise GCodeError("G2/G3 requires I/J center offsets or an R radius")
        points = _tessellate_arc(start, end, i, j, clockwise)
        self._position = end
        return [
            GCodeMove(kind="LINEAR", target=p, feed_mm_s=feed, source_line=raw_line) for p in points
        ]

    def _g4(self, words, raw_line):
        """G4 (dwell) — P is milliseconds (RepRap convention), S is seconds; defaults to 0."""
        if "P" in words:
            seconds = words["P"] / 1000.0  # P is milliseconds (RepRap convention)
        elif "S" in words:
            seconds = words["S"]
        else:
            seconds = 0.0
        return [GCodeMove(kind="DWELL", dwell_s=seconds, source_line=raw_line)]

    def _g20(self, words, raw_line):
        """G20 (inch units) — unsupported; this driver only ever operates in mm."""
        raise GCodeError("G20 (inch units) is not supported — this driver assumes mm (G21)")

    def _g21(self, words, raw_line):
        """G21 (millimeter units) — accepted as a no-op; mm is the only unit this driver uses."""
        return []  # mm is the only unit supported; nothing to resolve

    def _g28(self, words, raw_line):
        """G28 (home) — emit a HOME move targeting the conventional origin.

        No per-axis selection is supported: this always homes everything
        HomingProcedure.home_all() is configured for. The (0, 0, 0) target
        is only a pre-flight-checking convention (see
        GCodeProgram.to_waypoints) — the actual post-home position is
        whatever HomingProcedure finds on hardware.
        """
        return [GCodeMove(kind="HOME", target=(0.0, 0.0, 0.0), source_line=raw_line)]

    def _g90(self, words, raw_line):
        """G90 (absolute distance mode) — subsequent axis words are absolute coordinates."""
        self._absolute = True
        return []

    def _g91(self, words, raw_line):
        """G91 (relative distance mode) — subsequent axis words are deltas from the current position."""
        self._absolute = False
        return []

    # -- M-codes -------------------------------------------------------

    def _m0(self, words, raw_line):
        """M0 (unconditional program pause) — emit a PAUSE move handled via confirm_cb at execute time."""
        return [GCodeMove(kind="PAUSE", source_line=raw_line)]

    def _m1(self, words, raw_line):
        """M1 (optional program pause) — treated identically to M0 here; see GCodeExecutor._execute_pause."""
        return [GCodeMove(kind="PAUSE", source_line=raw_line)]

    def _m114(self, words, raw_line):
        """M114 (position query) — accepted as a no-op; read live position via MMCCommands directly."""
        return []  # position query — no motion; read state via MMCCommands directly


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------

class GCodeExecutor:
    """Parses, fence-checks, and executes G-code against the gantry.

    plan() is the only way to obtain a CheckedTrajectory; execute() only
    accepts the exact CheckedTrajectory object plan() returned for the same
    program (checked by identity), so the straight-line segments that were
    fence-checked are the ones actually followed.

    X/Y drive via the coordinated group (fences.py checks XYZ spatially,
    and this G-code subset has no rotary/Theta motion concept); Z drives
    via a separate single-axis command, never simultaneously with X/Y —
    see the module docstring's "Z/XY node split" note for why (Z lives on
    a different networked PLC node than X/Y, and this firmware's
    coordinated-group feature can't span that boundary). plan() splits any
    move that would need both into a Z-only leg followed by an XY-only
    leg before fence-checking, so what gets checked matches what actually
    runs.
    """

    def __init__(
        self,
        cmd: MMCCommands,
        checker: TrajectoryChecker,
        homing: Optional[HomingProcedure] = None,
        axes: Tuple[Axis, Axis] = (X_AXIS, Y_AXIS),
        z_axis: Axis = Z_AXIS,
        group_index: int = 1,
        confirm_cb: Optional[Callable[[GCodeMove], bool]] = None,
        dry_run: bool = False,
    ):
        """Configure an executor bound to a specific command interface, fence checker, and axis mapping.

        Args:
            cmd: Live MMCCommands wrapper the executor sends group moves
                and homing/IO commands through.
            checker: TrajectoryChecker used by plan() to fence-check the
                waypoints a parsed program would visit.
            homing: HomingProcedure driving G28. Required only if the
                program being executed actually contains a G28 — omitting
                it is fine for programs with no homing move.
            axes: The two coordinated-group axes, in (X, Y) order — must
                have exactly 2 elements. Confirmed on hardware that Z
                cannot be part of this group (see module docstring).
            z_axis: The axis driven as a separate single-axis leg whenever
                a move changes Z (see module docstring's "Z/XY node split"
                note).
            group_index: Coordinated-group index (the `C<N>` in the ASCII
                protocol) used for all group moves this executor issues.
            confirm_cb: Optional callback invoked before each LINEAR/HOME/
                PAUSE move; returning falsy raises GCodeExecutionAborted
                and stops execution. If None, all moves proceed
                unconfirmed.
            dry_run: If True, execute() logs what it would send instead of
                calling into `cmd`/`homing` at all — no hardware I/O
                occurs.

        Raises:
            ValueError: If `axes` does not have exactly 2 elements.
        """
        if len(axes) != 2:
            raise ValueError(
                "GCodeExecutor's coordinated group covers exactly 2 axes (X, Y) — "
                "Z moves separately as its own leg, see module docstring"
            )
        self._cmd = cmd
        self._checker = checker
        self._homing = homing
        self._axes = axes
        self._z_axis = z_axis
        self._group_index = group_index
        self._confirm_cb = confirm_cb
        self._dry_run = dry_run
        self._current_pos: Point3D = (0.0, 0.0, 0.0)
        self._pending_program: Optional[GCodeProgram] = None
        self._pending_trajectory: Optional[CheckedTrajectory] = None
        self._group_initialized = False

    def plan(self, text: str) -> CheckedTrajectory:
        """Parse G-code, split any move that would need simultaneous Z+XY
        motion into a Z-only leg followed by an XY-only leg (see module
        docstring's "Z/XY node split" note), and fence-check the result.

        Raises FenceViolation if any segment enters an exclusion zone. The
        returned CheckedTrajectory must be passed to execute() unmodified —
        it is the only object execute() will accept.
        """
        program = GCodeParser().parse(text, start=self._current_pos)
        program.moves = _split_cross_node_moves(program.moves, self._current_pos)
        waypoints = program.to_waypoints(self._current_pos)
        trajectory = self._checker.check_and_wrap(waypoints)
        self._pending_program = program
        self._pending_trajectory = trajectory
        return trajectory

    def execute(self, trajectory: CheckedTrajectory) -> None:
        """Execute a trajectory previously returned by plan(). Never call with anything else."""
        if not isinstance(trajectory, CheckedTrajectory):
            raise TypeError("execute() requires a CheckedTrajectory produced by plan()")
        if not trajectory.is_safe:
            raise trajectory.violations[0]
        if trajectory is not self._pending_trajectory or self._pending_program is None:
            raise RuntimeError(
                "execute() must be called with the exact CheckedTrajectory object "
                "returned by the most recent plan() call"
            )
        program = self._pending_program

        for move in program.moves:
            if move.kind == "LINEAR":
                self._execute_linear(move)
            elif move.kind == "HOME":
                self._execute_home(move)
            elif move.kind == "DWELL":
                self._execute_dwell(move)
            elif move.kind == "PAUSE":
                self._execute_pause(move)
            else:
                raise GCodeError(f"executor does not know how to run move kind {move.kind!r}")

        self._pending_program = None
        self._pending_trajectory = None

    # -- internal execution steps --------------------------------------

    def _confirm(self, move: GCodeMove) -> bool:
        """Ask `confirm_cb` (if configured) whether `move` should proceed; default to True if unset."""
        if self._confirm_cb is None:
            return True
        return bool(self._confirm_cb(move))

    def _init_group(self) -> None:
        """Initialize the coordinated (X, Y) group (`C<group_index> INI <axis indices>`).

        Sent at most once per GCodeExecutor instance, lazily on the first
        XY-leg execution — MMCCommands.init_group's own docstring says
        "call once after connecting"; re-sending it on every move (the
        previous behavior here) contradicted that contract. Programs that
        only ever move Z never touch the group at all.
        """
        if self._group_initialized:
            return
        indices = [axis.index for axis in self._axes]
        if self._dry_run:
            logger.info("[dry-run] C%d INI %s", self._group_index, " ".join(str(i) for i in indices))
            self._group_initialized = True
            return
        self._cmd.init_group(*indices)
        self._group_initialized = True

    def _execute_linear(self, move: GCodeMove) -> None:
        """Run one LINEAR move: confirm, then dispatch to the Z-only or
        XY-only leg depending on whether this move's Z differs from the
        position before it (see _split_cross_node_moves — after splitting,
        a single LINEAR move never changes both). Updates `_current_pos`
        regardless of dry-run.

        Raises:
            GCodeExecutionAborted: If confirm_cb rejects this move.
            SnapMotionError: If the move doesn't finish within the poll
                timeout.
        """
        if not self._confirm(move):
            raise GCodeExecutionAborted(f"aborted by confirm_cb: {move.source_line!r}")

        target_z = move.target[2]
        _, _, current_z = self._current_pos
        is_z_leg = _moved(target_z, current_z)

        if self._dry_run:
            logger.info("[dry-run] %s", self._describe_linear(move, is_z_leg))
            self._current_pos = move.target
            return

        if is_z_leg:
            self._execute_z_leg(move)
        else:
            self._execute_xy_leg(move)
        self._current_pos = move.target

    def _execute_z_leg(self, move: GCodeMove) -> None:
        """Move Z alone via a single-axis command. Z can't be part of the
        coordinated group on this hardware (see module docstring) — this
        is the only way Z ever moves."""
        if move.feed_mm_s is not None:
            self._cmd.set_speed(self._z_axis, move.feed_mm_s)
        self._cmd.begin_move_to(self._z_axis, move.target[2])
        self._poll_axis_move_finished(self._z_axis)

    def _execute_xy_leg(self, move: GCodeMove) -> None:
        """Move X/Y via the coordinated group (Z already at its target — see _split_cross_node_moves)."""
        self._init_group()
        if move.feed_mm_s is not None:
            self._cmd.group_set_speed(move.feed_mm_s)
        self._cmd.group_begin_move_to(move.target[0], move.target[1])
        self._poll_group_move_finished()

    def _describe_linear(self, move: GCodeMove, is_z_leg: bool) -> str:
        """Render the ASCII commands _execute_linear would send, for dry-run logging."""
        parts = []
        if is_z_leg:
            if move.feed_mm_s is not None:
                parts.append(f"{self._z_axis.token()} SPD {move.feed_mm_s:.6g}")
            parts.append(f"{self._z_axis.token()} BMT {move.target[2]:.6g}")
        else:
            if move.feed_mm_s is not None:
                parts.append(f"C{self._group_index} SPD {move.feed_mm_s:.6g}")
            fmt_pos = " ".join(f"{v:.6g}" for v in move.target[:2])
            parts.append(f"C{self._group_index} BMT {fmt_pos}")
        return "; ".join(parts)

    def _poll_axis_move_finished(
        self, axis: Axis, timeout_s: float = 30.0, poll_interval_s: float = 0.05
    ) -> None:
        """Block until the given axis's move-finished flag is set, aborting the move on timeout.

        Raises:
            SnapMotionError: If the move hasn't finished within
                `timeout_s`. The in-progress move is aborted (`abort()`)
                before raising.
        """
        deadline = time.monotonic() + timeout_s
        while not self._cmd.move_is_finished(axis):
            if time.monotonic() > deadline:
                self._cmd.abort(axis)
                raise SnapMotionError(
                    0, f"{axis.name} move did not finish within {timeout_s:.0f}s — aborted"
                )
            time.sleep(poll_interval_s)

    def _poll_group_move_finished(self, timeout_s: float = 30.0, poll_interval_s: float = 0.05) -> None:
        """Block until the group's move-finished flag is set, aborting the move on timeout.

        Args:
            timeout_s: Maximum seconds to wait before aborting (default 30).
            poll_interval_s: Sleep between polls (default 0.05s).

        Raises:
            SnapMotionError: If the move hasn't finished within
                `timeout_s`. The in-progress group move is aborted
                (`group_abort()`) before raising.
        """
        deadline = time.monotonic() + timeout_s
        while not self._cmd.group_move_is_finished():
            if time.monotonic() > deadline:
                self._cmd.group_abort()
                raise SnapMotionError(0, f"Group move did not finish within {timeout_s:.0f}s — aborted")
            time.sleep(poll_interval_s)

    def _execute_home(self, move: GCodeMove) -> None:
        """Run a G28 HOME move: confirm, then delegate to HomingProcedure.home_all().

        This blocks the caller for as long as home_all() takes. On success,
        `_current_pos` is set to the move's target (the conventional
        origin from GCodeParser._g28), not read back from hardware.

        Raises:
            GCodeExecutionAborted: If confirm_cb rejects this move.
            GCodeError: If no HomingProcedure was configured, or if
                home_all() reports failure.
        """
        if not self._confirm(move):
            raise GCodeExecutionAborted(f"aborted by confirm_cb: {move.source_line!r}")
        if self._dry_run:
            logger.info("[dry-run] home_all()")
        else:
            if self._homing is None:
                raise GCodeError("G28 requires a HomingProcedure but none was configured")
            result = self._homing.home_all()
            if not result.success:
                raise GCodeError(f"homing failed during G28: {result.error}")
        self._current_pos = move.target if move.target is not None else (0.0, 0.0, 0.0)

    def _execute_dwell(self, move: GCodeMove) -> None:
        """Run a G4 DWELL move by sleeping for `move.dwell_s` seconds (0 if unset).

        This is not confirmed via confirm_cb — dwells are treated as
        harmless and always proceed.
        """
        if self._dry_run:
            logger.info("[dry-run] dwell %.3fs", move.dwell_s or 0.0)
            return
        time.sleep(move.dwell_s or 0.0)

    def _execute_pause(self, move: GCodeMove) -> None:
        """Run an M0/M1 PAUSE move: entirely delegated to confirm_cb.

        There is no hardware command for a "pause" — this move exists only
        so confirm_cb gets a chance to block execution (e.g. to prompt an
        operator) before continuing to the next move.

        Raises:
            GCodeExecutionAborted: If confirm_cb is set and rejects this
                pause. If no confirm_cb is configured, the pause is a
                silent no-op and execution continues immediately.
        """
        if not self._confirm(move):
            raise GCodeExecutionAborted(f"paused/aborted by confirm_cb: {move.source_line!r}")
