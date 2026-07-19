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
    line = _COMMENT_PAREN.sub(" ", line)
    line = _COMMENT_SEMI.sub("", line)
    return line.strip()


def _parse_words(line: str) -> Dict[str, float]:
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
        self._absolute = True
        self._position: Point3D = (0.0, 0.0, 0.0)
        self._feed_mm_s: Optional[float] = None

    def parse(self, text: str, start: Point3D = (0.0, 0.0, 0.0)) -> GCodeProgram:
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
        if "F" in words:
            self._feed_mm_s = words["F"] / 60.0  # G-code feed rate is mm/min
        return self._feed_mm_s

    # -- G-codes -------------------------------------------------------

    def _g0(self, words, raw_line):
        return self._linear_move(words, raw_line)

    def _g1(self, words, raw_line):
        return self._linear_move(words, raw_line)

    def _linear_move(self, words: Dict[str, float], raw_line: str) -> List[GCodeMove]:
        target = self._resolve_target(words)
        feed = self._resolve_feed(words)
        self._position = target
        return [GCodeMove(kind="LINEAR", target=target, feed_mm_s=feed, source_line=raw_line)]

    def _g2(self, words, raw_line):
        return self._arc_move(words, raw_line, clockwise=True)

    def _g3(self, words, raw_line):
        return self._arc_move(words, raw_line, clockwise=False)

    def _arc_move(self, words: Dict[str, float], raw_line: str, clockwise: bool) -> List[GCodeMove]:
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
        if "P" in words:
            seconds = words["P"] / 1000.0  # P is milliseconds (RepRap convention)
        elif "S" in words:
            seconds = words["S"]
        else:
            seconds = 0.0
        return [GCodeMove(kind="DWELL", dwell_s=seconds, source_line=raw_line)]

    def _g20(self, words, raw_line):
        raise GCodeError("G20 (inch units) is not supported — this driver assumes mm (G21)")

    def _g21(self, words, raw_line):
        return []  # mm is the only unit supported; nothing to resolve

    def _g28(self, words, raw_line):
        return [GCodeMove(kind="HOME", target=(0.0, 0.0, 0.0), source_line=raw_line)]

    def _g90(self, words, raw_line):
        self._absolute = True
        return []

    def _g91(self, words, raw_line):
        self._absolute = False
        return []

    # -- M-codes -------------------------------------------------------

    def _m0(self, words, raw_line):
        return [GCodeMove(kind="PAUSE", source_line=raw_line)]

    def _m1(self, words, raw_line):
        return [GCodeMove(kind="PAUSE", source_line=raw_line)]

    def _m114(self, words, raw_line):
        return []  # position query — no motion; read state via MMCCommands directly


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------

class GCodeExecutor:
    """Parses, fence-checks, and executes G-code against the gantry.

    plan() is the only way to obtain a CheckedTrajectory; execute() only
    accepts the exact CheckedTrajectory object plan() returned for the same
    program (checked by identity), and drives motion via coordinated group
    moves — never independent single-axis moves — so the straight-line
    segments that were fence-checked are the ones actually followed.

    Only X/Y/Z are driven (fences.py only checks XYZ spatially, and this
    initial G-code subset has no rotary/Theta motion concept).
    """

    def __init__(
        self,
        cmd: MMCCommands,
        checker: TrajectoryChecker,
        homing: Optional[HomingProcedure] = None,
        axes: Tuple[Axis, Axis, Axis] = (X_AXIS, Y_AXIS, Z_AXIS),
        group_index: int = 1,
        confirm_cb: Optional[Callable[[GCodeMove], bool]] = None,
        dry_run: bool = False,
    ):
        if len(axes) != 3:
            raise ValueError("GCodeExecutor drives exactly 3 Cartesian axes (X, Y, Z)")
        self._cmd = cmd
        self._checker = checker
        self._homing = homing
        self._axes = axes
        self._group_index = group_index
        self._confirm_cb = confirm_cb
        self._dry_run = dry_run
        self._current_pos: Point3D = (0.0, 0.0, 0.0)
        self._pending_program: Optional[GCodeProgram] = None
        self._pending_trajectory: Optional[CheckedTrajectory] = None

    def plan(self, text: str) -> CheckedTrajectory:
        """Parse G-code and fence-check the resulting trajectory.

        Raises FenceViolation if any segment enters an exclusion zone. The
        returned CheckedTrajectory must be passed to execute() unmodified —
        it is the only object execute() will accept.
        """
        program = GCodeParser().parse(text, start=self._current_pos)
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

        if any(move.kind == "LINEAR" for move in program.moves):
            self._init_group()

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
        if self._confirm_cb is None:
            return True
        return bool(self._confirm_cb(move))

    def _init_group(self) -> None:
        indices = [axis.index for axis in self._axes]
        if self._dry_run:
            logger.info("[dry-run] C%d INI %s", self._group_index, " ".join(str(i) for i in indices))
            return
        self._cmd.init_group(*indices)

    def _execute_linear(self, move: GCodeMove) -> None:
        if not self._confirm(move):
            raise GCodeExecutionAborted(f"aborted by confirm_cb: {move.source_line!r}")
        if self._dry_run:
            logger.info("[dry-run] %s", self._describe_linear(move))
            self._current_pos = move.target
            return
        if move.feed_mm_s is not None:
            self._cmd.group_set_speed(move.feed_mm_s)
        self._cmd.group_begin_move_to(*move.target)
        self._poll_group_move_finished()
        self._current_pos = move.target

    def _describe_linear(self, move: GCodeMove) -> str:
        parts = []
        if move.feed_mm_s is not None:
            parts.append(f"C{self._group_index} SPD {move.feed_mm_s:.6g}")
        fmt_pos = " ".join(f"{v:.6g}" for v in move.target)
        parts.append(f"C{self._group_index} BMT {fmt_pos}")
        return "; ".join(parts)

    def _poll_group_move_finished(self, timeout_s: float = 30.0, poll_interval_s: float = 0.05) -> None:
        deadline = time.monotonic() + timeout_s
        while not self._cmd.group_move_is_finished():
            if time.monotonic() > deadline:
                self._cmd.group_abort()
                raise SnapMotionError(0, f"Group move did not finish within {timeout_s:.0f}s — aborted")
            time.sleep(poll_interval_s)

    def _execute_home(self, move: GCodeMove) -> None:
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
        if self._dry_run:
            logger.info("[dry-run] dwell %.3fs", move.dwell_s or 0.0)
            return
        time.sleep(move.dwell_s or 0.0)

    def _execute_pause(self, move: GCodeMove) -> None:
        if not self._confirm(move):
            raise GCodeExecutionAborted(f"paused/aborted by confirm_cb: {move.source_line!r}")
