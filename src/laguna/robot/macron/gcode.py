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

Node topology and the A (Theta) word: this hardware has two PLC nodes —
commander (X=1, Y=2) and responder (Z=5, Theta=6, the rotary axis). A
coordinated-group command cannot span both: `C1 INI 1 2` (X, Y) succeeds,
`C1 INI 1 2 5` (adding Z) fails with error 1010. Since Z and Theta share
the responder node, they *can* form their own group (`C2 INI 5 6`) —
this module accepts an optional `A` word (RepRap/CNC convention for a
rotary axis) on `G0`/`G1` lines, resolved into `GCodeMove.theta` exactly
like X/Y/Z, and represents Theta motion through the same LINEAR move
Z/X/Y already use, rather than as a separate concept.

Concurrent responder/commander legs: when one LINEAR move changes both
X/Y (commander) and Z and/or Theta (responder), `GCodeExecutor` cannot
send it as a single group command (the node-boundary limit above), but it
does not have to run the two halves sequentially either. Since Z and
Theta already share a node, `GCodeExecutor._execute_concurrent_pair`
fires the responder leg's `BMT` (single-axis, or the `C2` Z/Theta group
if both are moving) and the commander leg's `C1` `BMT` back-to-back,
non-blocking, then polls both to completion — the two legs genuinely
interpolate at the same time on hardware, which is a much closer
approximation of true multi-axis coordination than running one leg fully
to completion before starting the next.

Matched trapezoids, not just matched start times: firing two BMTs
together only makes them *start* together — with no further adjustment,
a short leg (say Z dips 5mm while X/Y travels 500mm) would reach its
target almost immediately and then sit idle while the other leg finishes
alone, which is barely better than the old sequential split. So before
issuing either BMT, `_execute_concurrent_pair` compares the two legs'
distances: whichever is shorter has its speed *and* accel/decel scaled
down by the same ratio `k` (short distance ÷ long distance), read fresh
from whichever the long leg's controller-configured ramp is at that
moment. Scaling every rate by `k` scales every phase's distance by `k`
but leaves every phase's *duration* unchanged (`t = v/a` and the
accel-phase distance `v²/(2a)` both cancel the common factor the same
way), so the shorter leg's whole trapezoid — ramp-up, cruise, ramp-down —
takes exactly as long as the longer leg's, at every instant covering
exactly `k` times the longer leg's progress. The two legs accelerate
together, cruise together, and decelerate together, not just start
together. The longer leg is left entirely alone (nominal commanded feed
rate, whatever accel/decel it already had) — only the shorter leg's
numbers change, and even those are restored to their own pre-scaling
values once both legs finish: this driver otherwise never touches
ACL/DCL (every other move leaves it at whatever the controller already
has), so a scaled-down ramp must not outlive the one move it was
computed for — left stale, it would silently slow every later move on
that axis, including pause()/stop()'s deceleration.

For a Z+Theta responder leg specifically, "distance" is
`max(|delta Z|, |delta Theta|)`, not Z alone — Z (mm) and Theta (raw
controller units) aren't truly commensurate, and there's no verified
hardware data on how the shared `C<theta_group_index>` SPD/ACL/DCL
actually governs a combined interpolation (see
`MMCCommands.group_set_speed`'s "assumes uniform mm_per_unit across the
group" docstring, known false for this pairing). Using Z alone let a
Theta-dominated move (a big rotation with a tiny Z step) get scaled down
to a crawl on the assumption it was short, badly undershooting the
default 30s poll timeout on what should have been an ordinary move;
max() is a conservative stand-in that avoids silently discounting
whichever axis is actually doing the work.

This still isn't a guaranteed diagonal, for two reasons neither leg's
own timing can fix: the commander and responder are two independent
node-local interpolators with no verified guarantee of simultaneous
start at the motor level (the responder is reached over a separate
networked link), and a G-code program authored for genuine simultaneous
N-axis motion still won't reproduce an exact vector path — this is a
hard hardware limitation (no cross-node group support), not a bug to
route around.

Because of that residual uncertainty, the actual swept path can't be
assumed to be a single line or a single L-shaped elbow — the only thing
guaranteed is that each axis moves monotonically from start to end at
its own rate, so the true envelope is the bounding box between start and
end. Rather than adding bounding-box-vs-fence geometry to fences.py,
`GCodeExecutor.plan()` conservatively fence-checks *both* possible elbow
orderings (Z-first and XY-first — the two extreme corners of that box)
through the existing `TrajectoryChecker`, via `GCodeProgram.to_waypoints`'s
`z_first` parameter, before allowing the move. See `GCodeExecutor.plan()`
and `_execute_concurrent_pair`.
"""

from __future__ import annotations

import logging
import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from .commands import (
    Axis, MMCCommands, THETA_AXIS, X_AXIS, Y_AXIS, Z_AXIS,
    poll_until_move_finished, predicted_move_s, resolve_timeout_s,
)
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


# Tolerance for deciding whether a position actually changed. Needed because
# mm values round-trip through raw controller units and back — exact float
# equality is too fragile against that rounding noise.
_POSITION_EPSILON_MM = 1e-3


def _moved(a: float, b: float) -> bool:
    return abs(a - b) > _POSITION_EPSILON_MM


@dataclass
class GCodeMove:
    """One resolved, ready-to-execute step of a parsed G-code program."""

    kind: str  # "LINEAR" | "HOME" | "DWELL" | "PAUSE"
    target: Optional[Point3D] = None
    theta: Optional[float] = None
    feed_mm_s: Optional[float] = None
    dwell_s: Optional[float] = None
    source_line: str = ""


@dataclass
class GCodeProgram:
    """An ordered sequence of resolved moves, ready for fence-checking/execution."""

    moves: List[GCodeMove] = field(default_factory=list)

    def to_waypoints(self, start: Point3D, z_first: bool = True) -> List[Point3D]:
        """Expand this program into the flat XYZ waypoint list TrajectoryChecker expects.

        Only LINEAR and HOME moves contribute waypoints (DWELL/PAUSE don't
        move; Theta never does — fences.py only checks XYZ). G28's landing
        point is assumed to be the conventional origin for pre-flight
        checking purposes — the real homed position is whatever
        HomingProcedure actually finds, but this is only used to validate
        what happens *after* the G28 in the same program.

        A move that changes both Z and X/Y runs as two concurrent legs, not
        a single line (see the module docstring's "concurrent responder/
        commander legs" note) — its true swept path is the bounding box
        between start and end, not one line. `z_first` picks which of that
        box's two extreme corners this waypoint list represents (the
        Z-first or the XY-first elbow); `GCodeExecutor.plan()` calls this
        twice, once per ordering, to conservatively check both.
        """
        waypoints = [start]
        pos = start
        for move in self.moves:
            if move.kind == "LINEAR" and move.target is not None:
                tx, ty, tz = move.target
                px, py, pz = pos
                if _moved(tz, pz) and (_moved(tx, px) or _moved(ty, py)):
                    elbow = (px, py, tz) if z_first else (tx, ty, pz)
                    waypoints.append(elbow)
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
        """Create a parser with fresh state (absolute mode, no feed rate set).

        A GCodeParser instance is stateful across the lines of a single
        parse() call (distance mode, current position, last feed rate),
        but that state is reset at the start of every parse() call, so a
        single instance can safely be reused for multiple programs.
        """
        self._absolute = True
        self._position: Point3D = (0.0, 0.0, 0.0)
        self._theta: float = 0.0
        self._feed_mm_s: Optional[float] = None

    def parse(
        self, text: str, start: Point3D = (0.0, 0.0, 0.0), start_theta: float = 0.0
    ) -> GCodeProgram:
        """Parse a multi-line G-code program into a GCodeProgram of resolved moves.

        Resets parser state (absolute mode, feed rate) before parsing, then
        processes the text line by line, tracking position/mode as it goes
        so that relative moves, unspecified axes, and carried-over feed
        rates all resolve correctly.

        Args:
            text: Raw G-code source, one instruction per line.
            start: Starting XYZ position moves are resolved relative/
                absolute to (default origin).
            start_theta: Starting Theta (rotary axis) position `A` words
                are resolved relative/absolute to (default 0). Only
                matters for programs that use `A` in relative mode (G91).

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
        self._theta = start_theta
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

    def _resolve_theta(self, words: Dict[str, float]) -> Optional[float]:
        """Resolve a move's Theta target from an `A` word or None.

        None if this line doesn't touch Theta at all.

        Unlike X/Y/Z (which always resolve to a value — an axis not
        mentioned just stays at its current position), Theta has no
        Cartesian home in fences.py's model, so a line with no `A` word
        must produce no Theta motion whatsoever, not an implicit "hold
        current position" target — see GCodeExecutor's leg classification.
        """
        if "A" not in words:
            return None
        if self._absolute:
            return words["A"]
        return self._theta + words["A"]

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
        """Shared G0/G1 implementation: resolve target/theta/feed, advance position, emit one move."""
        target = self._resolve_target(words)
        theta = self._resolve_theta(words)
        feed = self._resolve_feed(words)
        self._position = target
        if theta is not None:
            self._theta = theta
        return [
            GCodeMove(kind="LINEAR", target=target, theta=theta, feed_mm_s=feed, source_line=raw_line)
        ]

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


def _zt_distance(move: GCodeMove, current_pos: Point3D, current_theta: float) -> float:
    """"Distance" travelled by a Z/Theta group leg.

    For duration prediction and speed/ramp scaling (see
    GCodeExecutor._execute_concurrent_pair).

    Z (mm) and Theta (raw controller units, not a physical length) aren't
    truly commensurate — there's no verified hardware data on how the
    C<theta_group_index> group's single shared SPD/ACL/DCL actually governs
    a combined Z+Theta interpolation (see MMCCommands.group_set_speed's
    "assumes uniform mm_per_unit across the group" docstring, which is
    known false for this pairing). Using max() rather than Z alone is a
    conservative choice: it guarantees neither axis's real travel is
    silently discounted to near-zero when computing which leg is "longer"
    and by how much to scale the other — using Z alone let a
    Theta-dominated move (e.g. a big rotation with a tiny Z step) get
    scaled down to a crawl, badly undershooting the 30s poll timeout on
    what should have been an ordinary move.
    """
    dz = abs(move.target[2] - current_pos[2])
    dtheta = abs(move.theta - current_theta) if move.theta is not None else 0.0
    return max(dz, dtheta)


class GCodeExecutor:
    """Parses, fence-checks, and executes G-code against the gantry.

    plan() is the only way to obtain a CheckedTrajectory; execute() only
    accepts the exact CheckedTrajectory object plan() returned for the same
    program (checked by identity), so the straight-line segments that were
    fence-checked are the ones actually followed.

    X/Y drive via the commander-node coordinated group (`C<group_index>`).
    Z and Theta live on the responder node and drive via whichever of
    three shapes a given move actually needs: a single-axis command (Z
    alone, or Theta alone), or — when both change together — their own
    coordinated group (`C<theta_group_index>`, only available if
    `theta_cmd` is configured). A move that changes both the commander
    axes and the responder axes cannot be sent as one group command (see
    the module docstring's node-topology note), so `_execute_concurrent_pair`
    fires both legs' BMTs non-blocking, back-to-back, and polls both to
    completion — genuinely concurrent, not the commander-then-responder
    sequence this used to be. plan() fence-checks both possible elbow
    orderings of any such move before allowing it (see
    GCodeProgram.to_waypoints).
    """

    def __init__(
        self,
        cmd: MMCCommands,
        checker: TrajectoryChecker,
        homing: Optional[HomingProcedure] = None,
        axes: Tuple[Axis, Axis] = (X_AXIS, Y_AXIS),
        z_axis: Axis = Z_AXIS,
        theta_axis: Axis = THETA_AXIS,
        group_index: int = 1,
        theta_cmd: Optional[MMCCommands] = None,
        theta_group_index: int = 2,
        confirm_cb: Optional[Callable[[GCodeMove], bool]] = None,
        dry_run: bool = False,
    ):
        """Configure an executor bound to a specific command interface, fence checker, and axis mapping.

        Args:
            cmd: Live MMCCommands wrapper the executor sends the X/Y group
                moves, single-axis Z/Theta moves, and homing/IO commands
                through.
            checker: TrajectoryChecker used by plan() to fence-check the
                waypoints a parsed program would visit.
            homing: HomingProcedure driving G28. Required only if the
                program being executed actually contains a G28 — omitting
                it is fine for programs with no homing move.
            axes: The two commander-node coordinated-group axes, in (X, Y)
                order — must have exactly 2 elements. Confirmed on
                hardware that Z can't join this group (see module
                docstring).
            z_axis: The axis driven whenever a move changes Z and Theta
                doesn't move with it.
            theta_axis: The axis driven whenever a move changes Theta (the
                `A` word) and Z doesn't move with it.
            group_index: Coordinated-group index (the `C<N>` in the ASCII
                protocol) used for the X/Y group.
            theta_cmd: A second MMCCommands instance (sharing the same
                connection, configured with `group_axes=(z_axis,
                theta_axis)`) used for the Z/Theta coordinated group, sent
                only when a move changes both Z and Theta together. A
                move that needs this group raises GCodeError if `theta_cmd`
                wasn't given — Z-only and Theta-only moves never need it.
            theta_group_index: Coordinated-group index used for the
                Z/Theta group — must differ from `group_index`.
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
                "GCodeExecutor's commander-node group covers exactly 2 axes (X, Y) — "
                "Z/Theta move separately, see module docstring"
            )
        self._cmd = cmd
        self._checker = checker
        self._homing = homing
        self._axes = axes
        self._z_axis = z_axis
        self._theta_axis = theta_axis
        self._group_index = group_index
        self._theta_cmd = theta_cmd
        self._theta_group_index = theta_group_index
        self._confirm_cb = confirm_cb
        self._dry_run = dry_run
        self._current_pos: Point3D = (0.0, 0.0, 0.0)
        self._current_theta: float = 0.0
        self._pending_program: Optional[GCodeProgram] = None
        self._pending_trajectory: Optional[CheckedTrajectory] = None
        # INI is sent once per executor, lazily on the first leg that needs
        # each group — see _init_group/_init_theta_group and reset_group_init.
        self._group_initialized = False
        self._theta_group_initialized = False

    def plan(self, text: str) -> CheckedTrajectory:
        """Parse G-code and fence-check both elbow orderings.

        Covers both elbow orderings a concurrent XY+Z(Theta) leg's swept
        bounding box could take.

        A move that changes both X/Y and Z runs as two concurrent legs, not
        a single line or a fixed elbow (see the module docstring's
        "concurrent responder/commander legs" note) — its true path lies
        somewhere in the bounding box between start and end. Rather than
        checking that box directly, this checks both extreme corners
        (Z-first and XY-first orderings, via GCodeProgram.to_waypoints)
        through the existing TrajectoryChecker — conservative, and needs no
        new geometry in fences.py.

        Raises FenceViolation if either ordering enters an exclusion zone.
        The returned CheckedTrajectory must be passed to execute()
        unmodified — it is the only object execute() will accept.
        """
        program = GCodeParser().parse(text, start=self._current_pos, start_theta=self._current_theta)
        xy_first_violations = self._checker.check_trajectory(
            program.to_waypoints(self._current_pos, z_first=False)
        )
        if xy_first_violations:
            raise xy_first_violations[0]
        trajectory = self._checker.check_and_wrap(
            program.to_waypoints(self._current_pos, z_first=True)
        )
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
        """Initialize the commander-node (X, Y) group (`C<group_index> INI <indices>`).

        Sent at most once per executor, lazily on the first leg that needs
        it — MMCCommands.init_group's own docstring says "call once after
        connecting", and re-sending it per move (the original behaviour
        here) contradicted that: a 40-move run re-sent an identical
        `C1 INI 1 2` 40 times. Programs that never move X/Y never touch
        this group at all.

        Call reset_group_init() if the controller may have lost its group
        state (power-cycle, reflash) — GantryController does so on
        connect().
        """
        if self._group_initialized:
            return
        indices = [axis.index for axis in self._axes]
        if self._dry_run:
            logger.info("[dry-run] C%d INI %s", self._group_index, " ".join(str(i) for i in indices))
        else:
            logger.debug("C%d INI %s", self._group_index, " ".join(str(i) for i in indices))
            self._cmd.init_group(*indices)
        self._group_initialized = True

    def _init_theta_group(self) -> None:
        """Initialize the responder-node (Z, Theta) coordinated group.

        Sends `C<theta_group_index> INI <z_index> <theta_index>`.

        Sent at most once per executor, lazily on the first leg that needs
        it — mirrors _init_group. Only reached when a move changes both Z
        and Theta together (see _leg_kinds); Z-only and Theta-only moves
        never touch this group.

        Raises:
            GCodeError: If no `theta_cmd` was configured — a move that
                reaches here has no other way to run.
        """
        if self._theta_cmd is None:
            raise GCodeError(
                "This move changes both Z and Theta (A) together, which "
                "requires the Z/Theta coordinated group — configure "
                "GCodeExecutor with theta_cmd=... (see the class docstring)"
            )
        if self._theta_group_initialized:
            return
        indices = [self._z_axis.index, self._theta_axis.index]
        if self._dry_run:
            logger.info(
                "[dry-run] C%d INI %s", self._theta_group_index, " ".join(str(i) for i in indices)
            )
        else:
            logger.debug(
                "C%d INI %s", self._theta_group_index, " ".join(str(i) for i in indices)
            )
            self._theta_cmd.init_group(*indices)
        self._theta_group_initialized = True

    def reset_group_init(self) -> None:
        """Forget that coordinated groups were initialized.

        So the next leg that needs one re-sends its INI.

        Call after anything that may have cleared the controller's own
        group state — a power-cycle, a reflash, or a reconnect that might
        span one. See _init_group/_init_theta_group.
        """
        self._group_initialized = False
        self._theta_group_initialized = False

    def sync_position_from_hardware(self) -> None:
        """Refresh `_current_pos`/`_current_theta` from a live ACP read of every axis.

        `_current_pos`/`_current_theta` default to (0, 0, 0)/0.0 at
        construction — a fine placeholder only until this is called, never
        a claim about where the gantry actually is. Nothing seeds them
        from hardware afterward except this and each real move's own
        post-move resync (_sync_position_from_hardware, the touched-axes-
        only version) — so a controller that reconnects (or is freshly
        constructed) while the gantry is parked away from wherever the
        cache defaults to will otherwise plan every subsequent move
        against a phantom starting position. Concretely: the gantry
        physically at (100, 100, 200), a reconnect, then
        `move_to([0, 0, 0, 0])` — target (0,0,0) matches the still-default
        (0,0,0) cache, `_leg_kinds` sees nothing moved, and the call
        no-ops instead of homing. Call this once connected, before any
        move relies on `_current_pos` being true.
        """
        x_axis, y_axis = self._axes
        self._current_pos = (
            self._cmd.get_actual_position(x_axis),
            self._cmd.get_actual_position(y_axis),
            self._cmd.get_actual_position(self._z_axis),
        )
        self._current_theta = self._cmd.get_actual_position(self._theta_axis)

    def _leg_kinds(self, move: GCodeMove) -> Tuple[bool, str]:
        """Classify a LINEAR move into (touches_xy, responder_kind).

        Relative to the executor's current position.

        responder_kind is one of "none", "z", "theta", "z_theta" — which
        responder-node command shape (no motion, single-axis Z,
        single-axis Theta, or the Z/Theta group) this move's Z/Theta
        portion needs. Combined with touches_xy, this drives every
        dispatch decision in _execute_linear/_describe_linear.
        """
        tx, ty, tz = move.target
        px, py, pz = self._current_pos
        touches_xy = _moved(tx, px) or _moved(ty, py)
        touches_z = _moved(tz, pz)
        touches_theta = move.theta is not None and _moved(move.theta, self._current_theta)
        if touches_z and touches_theta:
            responder = "z_theta"
        elif touches_z:
            responder = "z"
        elif touches_theta:
            responder = "theta"
        else:
            responder = "none"
        return touches_xy, responder

    def _advance_position(self, move: GCodeMove) -> None:
        """Update `_current_pos`/`_current_theta` to the commanded target (dry-run only).

        Assumes move.target/move.theta were reached exactly — fine for
        dry-run, which never touches hardware. Real moves use
        _sync_position_from_hardware() instead; see that method's
        docstring for why trusting the commanded target isn't safe once
        real motion is involved.
        """
        self._current_pos = move.target
        if move.theta is not None:
            self._current_theta = move.theta

    def _sync_position_from_hardware(self, touches_xy: bool, responder: str) -> None:
        """Refresh `_current_pos`/`_current_theta` from a live ACP read, for whichever axes this move touched.

        Raw controller units don't divide evenly into real mm (see
        MMCCommands._pos_to_mm/_pos_to_raw), so the position actually
        reached after a move is almost never bit-for-bit equal to the
        commanded float target — trusting the target (the old behaviour)
        let that quantization residue silently diverge from hardware
        truth. GantryController.move_to() backfills any axis it doesn't
        explicitly target with a *live* get_actual_position() read (see
        its docstring), so on the next move that live reading could
        differ from this executor's stale cached target by more than
        _POSITION_EPSILON_MM — _leg_kinds then sees a phantom "moved" on
        an axis nobody asked to move, forcing an unwanted
        _execute_concurrent_pair leg whose near-zero distance scales
        ACL/DCL down to 0 raw units (ASCII escape 16/17, "0 Or
        Negative"). Re-reading actual position for exactly the axes this
        move commanded keeps the cache honest instead of drifting.

        Only the axes this move actually commanded are re-read — axes it
        didn't touch were already reconciled the last time they moved (or
        are still at their initial/homed value), so re-reading them here
        would just be wasted round-trips.
        """
        if touches_xy:
            x_axis, y_axis = self._axes
            self._current_pos = (
                self._cmd.get_actual_position(x_axis),
                self._cmd.get_actual_position(y_axis),
                self._current_pos[2],
            )
        if responder in ("z", "z_theta"):
            self._current_pos = (
                self._current_pos[0],
                self._current_pos[1],
                self._cmd.get_actual_position(self._z_axis),
            )
        if responder in ("theta", "z_theta"):
            self._current_theta = self._cmd.get_actual_position(self._theta_axis)

    def _execute_linear(self, move: GCodeMove) -> None:
        """Run one LINEAR move via whichever leg(s) it actually needs.

        A move touching only X/Y, only Z, or only Theta runs as a single
        leg exactly as before. A move touching X/Y together with Z and/or
        Theta can't be sent as one group command (node-topology limit —
        see module docstring), so it runs as two concurrent legs via
        _execute_concurrent_pair. Updates `_current_pos`/`_current_theta`
        regardless of dry-run — from the commanded target in dry-run (no
        hardware to read), from a live ACP re-read for a real move (see
        _sync_position_from_hardware).

        Raises:
            GCodeExecutionAborted: If confirm_cb rejects this move.
            SnapMotionError: If a leg doesn't finish within its poll
                timeout.
        """
        if not self._confirm(move):
            raise GCodeExecutionAborted(f"aborted by confirm_cb: {move.source_line!r}")

        touches_xy, responder = self._leg_kinds(move)

        if self._dry_run:
            logger.info("[dry-run] %s", self._describe_linear(move, touches_xy, responder))
            self._advance_position(move)
            return

        # One line per tessellated segment — deliberately DEBUG, not INFO.
        # An arc can expand into dozens of these (see _tessellate_arc); the
        # broad "move_to() started/completed" pair a caller sees is logged
        # once, at the GantryController layer — see laguna.subsystem_logging's
        # module docstring for the tier split.
        logger.debug("%s", self._describe_linear(move, touches_xy, responder))

        if touches_xy and responder != "none":
            self._execute_concurrent_pair(move, responder)
        elif touches_xy:
            self._execute_xy_leg(move)
        elif responder == "z":
            self._execute_z_leg(move)
        elif responder == "theta":
            self._execute_theta_leg(move)
        elif responder == "z_theta":
            self._execute_zt_leg(move)
        # responder == "none" and not touches_xy: nothing actually moves.

        self._sync_position_from_hardware(touches_xy, responder)

    def _begin_z_leg(
        self, move: GCodeMove, speed: Optional[float] = None, ramp: Optional[Tuple[float, float]] = None
    ) -> None:
        speed = move.feed_mm_s if speed is None else speed
        if ramp is not None:
            accel, decel = ramp
            self._cmd.set_accel(self._z_axis, accel)
            self._cmd.set_decel(self._z_axis, decel)
        if speed is not None:
            self._cmd.set_speed(self._z_axis, speed)
        self._cmd.begin_move_to(self._z_axis, move.target[2])

    def _poll_z_leg(
        self,
        move: GCodeMove,
        current_pos: Point3D,
        speed: Optional[float] = None,
        already_elapsed_s: float = 0.0,
    ) -> None:
        distance = abs(move.target[2] - current_pos[2])
        speed = move.feed_mm_s if speed is None else speed
        if speed is None:
            # No F word and no caller-supplied speed: begin_z_leg left the
            # axis's SPD untouched, so read it back live rather than
            # predicting off an unknown speed — predicted_move_s(d, None)
            # is 0.0, which collapses the timeout to MIN_TIMEOUT_S
            # regardless of how long the move actually takes.
            speed = self._read_responder_speed("z")
        predicted_s = max(0.0, predicted_move_s(distance, speed) - already_elapsed_s)
        self._poll_axis_move_finished(self._z_axis, predicted_s=predicted_s)

    def _execute_z_leg(self, move: GCodeMove) -> None:
        """Move Z alone via a single-axis command."""
        current_pos = self._current_pos
        self._begin_z_leg(move)
        self._poll_z_leg(move, current_pos)

    def _begin_theta_leg(
        self, move: GCodeMove, speed: Optional[float] = None, ramp: Optional[Tuple[float, float]] = None
    ) -> None:
        speed = move.feed_mm_s if speed is None else speed
        if ramp is not None:
            accel, decel = ramp
            self._cmd.set_accel(self._theta_axis, accel)
            self._cmd.set_decel(self._theta_axis, decel)
        if speed is not None:
            self._cmd.set_speed(self._theta_axis, speed)
        self._cmd.begin_move_to(self._theta_axis, move.theta)

    def _poll_theta_leg(
        self,
        move: GCodeMove,
        current_theta: float,
        speed: Optional[float] = None,
        already_elapsed_s: float = 0.0,
    ) -> None:
        distance = abs(move.theta - current_theta)
        speed = move.feed_mm_s if speed is None else speed
        if speed is None:
            # See _poll_z_leg's note — same live-read fallback.
            speed = self._read_responder_speed("theta")
        predicted_s = max(0.0, predicted_move_s(distance, speed) - already_elapsed_s)
        self._poll_axis_move_finished(self._theta_axis, predicted_s=predicted_s)

    def _execute_theta_leg(self, move: GCodeMove) -> None:
        """Move Theta alone via a single-axis command."""
        current_theta = self._current_theta
        self._begin_theta_leg(move)
        self._poll_theta_leg(move, current_theta)

    def _begin_zt_leg(
        self, move: GCodeMove, speed: Optional[float] = None, ramp: Optional[Tuple[float, float]] = None
    ) -> None:
        self._init_theta_group()
        speed = move.feed_mm_s if speed is None else speed
        if ramp is not None:
            accel, decel = ramp
            self._theta_cmd.group_set_accel(accel)
            self._theta_cmd.group_set_decel(decel)
        if speed is not None:
            self._theta_cmd.group_set_speed(speed)
        self._theta_cmd.group_begin_move_to(move.target[2], move.theta)

    def _poll_zt_leg(
        self,
        move: GCodeMove,
        current_pos: Point3D,
        current_theta: float,
        speed: Optional[float] = None,
        already_elapsed_s: float = 0.0,
    ) -> None:
        distance = _zt_distance(move, current_pos, current_theta)
        speed = move.feed_mm_s if speed is None else speed
        if speed is None:
            # See _poll_z_leg's note — same live-read fallback.
            speed = self._read_responder_speed("z_theta")
        predicted_s = max(0.0, predicted_move_s(distance, speed) - already_elapsed_s)
        self._poll_theta_group_move_finished(predicted_s=predicted_s)

    def _execute_zt_leg(self, move: GCodeMove) -> None:
        """Move Z and Theta together via coordinated group.

        Via their own coordinated group (`C<theta_group_index>`) — both live
        on the responder node, so this group doesn't cross the node boundary
        the X/Y group can't cross. See the module docstring.
        """
        current_pos = self._current_pos
        current_theta = self._current_theta
        self._begin_zt_leg(move)
        self._poll_zt_leg(move, current_pos, current_theta)

    def _begin_xy_leg(
        self, move: GCodeMove, speed: Optional[float] = None, ramp: Optional[Tuple[float, float]] = None
    ) -> None:
        self._init_group()
        speed = move.feed_mm_s if speed is None else speed
        if ramp is not None:
            accel, decel = ramp
            self._cmd.group_set_accel(accel)
            self._cmd.group_set_decel(decel)
        if speed is not None:
            self._cmd.group_set_speed(speed)
        self._cmd.group_begin_move_to(move.target[0], move.target[1])

    def _poll_xy_leg(
        self,
        move: GCodeMove,
        current_pos: Point3D,
        speed: Optional[float] = None,
        already_elapsed_s: float = 0.0,
    ) -> None:
        distance = math.hypot(move.target[0] - current_pos[0], move.target[1] - current_pos[1])
        speed = move.feed_mm_s if speed is None else speed
        if speed is None:
            # See _poll_z_leg's note — same live-read fallback.
            speed = self._read_xy_speed()
        predicted_s = max(0.0, predicted_move_s(distance, speed) - already_elapsed_s)
        self._poll_group_move_finished(predicted_s=predicted_s)

    def _execute_xy_leg(self, move: GCodeMove) -> None:
        """Move X/Y via the commander-node coordinated group."""
        current_pos = self._current_pos
        self._begin_xy_leg(move)
        self._poll_xy_leg(move, current_pos)

    def _read_xy_speed(self) -> float:
        """Return the X/Y group's currently configured SPD.

        For restoring after _execute_concurrent_pair scales it down — see
        that method.
        """
        self._init_group()
        return self._cmd.group_get_speed()

    def _read_responder_speed(self, responder: str) -> float:
        """Return the responder leg's currently configured SPD.

        Mirrors _read_xy_speed for the other side of
        _execute_concurrent_pair.
        """
        if responder == "z":
            return self._cmd.get_speed(self._z_axis)
        if responder == "theta":
            return self._cmd.get_speed(self._theta_axis)
        self._init_theta_group()
        return self._theta_cmd.group_get_speed()

    def _read_xy_ramp(self) -> Tuple[float, float]:
        """Return the X/Y group's currently configured (accel, decel).

        For scaling down the other leg's ramp to match — see
        _execute_concurrent_pair. Initializes the group first (lazily, like
        every other XY-group access) since querying ACL/DCL before INI is
        untested.
        """
        self._init_group()
        return self._cmd.group_get_accel(), self._cmd.group_get_decel()

    def _set_xy_ramp(self, accel: float, decel: float) -> None:
        self._cmd.group_set_accel(accel)
        self._cmd.group_set_decel(decel)

    def _set_xy_speed(self, speed: float) -> None:
        self._cmd.group_set_speed(speed)

    def _read_responder_ramp(self, responder: str) -> Tuple[float, float]:
        """Return the responder leg's currently configured (accel, decel).

        For whichever command shape `responder` names — mirrors
        _read_xy_ramp for the other side of _execute_concurrent_pair.
        """
        if responder == "z":
            return self._cmd.get_accel(self._z_axis), self._cmd.get_decel(self._z_axis)
        if responder == "theta":
            return self._cmd.get_accel(self._theta_axis), self._cmd.get_decel(self._theta_axis)
        self._init_theta_group()
        return self._theta_cmd.group_get_accel(), self._theta_cmd.group_get_decel()

    def _set_responder_ramp(self, responder: str, accel: float, decel: float) -> None:
        if responder == "z":
            self._cmd.set_accel(self._z_axis, accel)
            self._cmd.set_decel(self._z_axis, decel)
        elif responder == "theta":
            self._cmd.set_accel(self._theta_axis, accel)
            self._cmd.set_decel(self._theta_axis, decel)
        else:  # "z_theta"
            self._theta_cmd.group_set_accel(accel)
            self._theta_cmd.group_set_decel(decel)

    def _set_responder_speed(self, responder: str, speed: float) -> None:
        if responder == "z":
            self._cmd.set_speed(self._z_axis, speed)
        elif responder == "theta":
            self._cmd.set_speed(self._theta_axis, speed)
        else:  # "z_theta"
            self._theta_cmd.group_set_speed(speed)

    def _execute_concurrent_pair(self, move: GCodeMove, responder: str) -> None:
        """Fire the responder and commander legs as independent non-blocking BMTs.

        Fire the responder leg (Z, Theta, or Z+Theta) and the commander
        XY leg back-to-back, then poll both to completion.

        Whichever leg travels *less* distance is scaled down — speed,
        accel, and decel all multiplied by the same ratio `k` (its
        distance divided by the longer leg's) — so both legs' trapezoidal
        velocity profiles take exactly the same time: scaling every rate
        by the same factor scales every phase's distance by that factor
        but leaves every phase's *duration* unchanged (t = v/a and
        v²/(2a) both cancel the common factor identically). The longer leg
        keeps the nominal commanded feed rate and whatever accel/decel it
        already had; the shorter leg's ramp is read fresh from the
        controller and scaled from that reference, so both legs start
        accelerating together, reach cruise together, and decelerate
        together — not just start together like a naive same-speed pair
        would (see the module docstring's "concurrent responder/commander
        legs" note for why even this still isn't a guaranteed diagonal).

        The shorter leg's *own* pre-scaling SPD and ramp are read and
        restored once both legs finish — this driver otherwise never
        touches ACL/DCL/SPD (every other move leaves them at whatever the
        controller already has), so a scaled-down value must not outlive
        this one move. Left stale, it would silently slow every later move
        on that axis (including how fast pause()/stop() can decelerate it)
        until some future concurrent-pair move happened to overwrite it
        again.

        The "nominal" speed the longer leg keeps and the shorter leg
        scales from is the move's F word if it has one — if not (no F at
        all), it's read live from whatever SPD the longer leg's axis/group
        is currently configured at, so the legs still get duration-matched
        instead of each running at its own independently-stale SPD (the
        original bug this whole scheme exists to prevent, just triggered
        by two different axes never having matched to begin with rather
        than one drifting after a prior scaled move). The longer leg's own
        SPD is never rewritten in this case — it's already exactly the
        value just read, so leaving it alone is correct, not an oversight.

        Skipped entirely (both legs left untouched, running at whatever
        SPD/ACL/DCL each already has) only if the reference leg's
        currently-configured ACL/DCL reads back as 0 or negative (e.g.
        right after group INI, before this session has ever set a ramp on
        it — querying ACL/DCL immediately after INI is untested): scaling
        a 0-or-negative reference produces a 0-or-negative ACL/DCL for the
        other leg, which the controller rejects outright (ASCII escape
        codes 16/17, "User Accels/Decels 0 Or Negative").
        """
        current_pos = self._current_pos
        current_theta = self._current_theta

        dist_xy = math.hypot(move.target[0] - current_pos[0], move.target[1] - current_pos[1])
        if responder == "theta":
            dist_responder = abs(move.theta - current_theta)
        elif responder == "z_theta":
            dist_responder = _zt_distance(move, current_pos, current_theta)
        else:  # "z"
            dist_responder = abs(move.target[2] - current_pos[2])

        xy_speed: Optional[float] = move.feed_mm_s
        responder_speed: Optional[float] = move.feed_mm_s
        xy_ramp: Optional[Tuple[float, float]] = None
        responder_ramp: Optional[Tuple[float, float]] = None
        restore_xy_ramp: Optional[Tuple[float, float]] = None
        restore_responder_ramp: Optional[Tuple[float, float]] = None
        restore_xy_speed: Optional[float] = None
        restore_responder_speed: Optional[float] = None

        if dist_xy > 0 and dist_responder > 0:
            if dist_xy >= dist_responder:
                k = dist_responder / dist_xy
                ref_accel, ref_decel = self._read_xy_ramp()
                if ref_accel > 0 and ref_decel > 0:
                    nominal_feed = (
                        move.feed_mm_s if move.feed_mm_s is not None else self._read_xy_speed()
                    )
                    restore_responder_ramp = self._read_responder_ramp(responder)
                    restore_responder_speed = self._read_responder_speed(responder)
                    responder_speed = k * nominal_feed
                    responder_ramp = (k * ref_accel, k * ref_decel)
                else:
                    logger.warning(
                        "X/Y group ACL/DCL read back as %r/%r (0 or negative) — "
                        "skipping concurrent-pair scaling for this move; legs "
                        "will not be duration-matched",
                        ref_accel, ref_decel,
                    )
            else:
                k = dist_xy / dist_responder
                ref_accel, ref_decel = self._read_responder_ramp(responder)
                if ref_accel > 0 and ref_decel > 0:
                    nominal_feed = (
                        move.feed_mm_s
                        if move.feed_mm_s is not None
                        else self._read_responder_speed(responder)
                    )
                    restore_xy_ramp = self._read_xy_ramp()
                    restore_xy_speed = self._read_xy_speed()
                    xy_speed = k * nominal_feed
                    xy_ramp = (k * ref_accel, k * ref_decel)
                else:
                    logger.warning(
                        "%s ACL/DCL read back as %r/%r (0 or negative) — "
                        "skipping concurrent-pair scaling for this move; legs "
                        "will not be duration-matched",
                        responder, ref_accel, ref_decel,
                    )

        # Everything from here on must restore whichever ramp was scaled,
        # even if a leg never finishes (poll_until_move_finished timeout,
        # aborted mid-flight) or a begin/poll call raises for any other
        # reason — see this method's docstring on why a scaled-down ramp
        # can never be allowed to outlive this one move. abort()/
        # group_abort() (issued by the poll helpers on timeout) are
        # immediate, not ramped, so writing ACL/DCL afterward doesn't fight
        # an in-flight stop.
        try:
            if responder == "z":
                self._begin_z_leg(move, speed=responder_speed, ramp=responder_ramp)
            elif responder == "theta":
                self._begin_theta_leg(move, speed=responder_speed, ramp=responder_ramp)
            else:  # "z_theta"
                self._begin_zt_leg(move, speed=responder_speed, ramp=responder_ramp)
            self._begin_xy_leg(move, speed=xy_speed, ramp=xy_ramp)
            fired_at = time.monotonic()

            # Both BMTs are already on the wire and physically running
            # concurrently by this point — polling them is necessarily
            # sequential (one Python call at a time), but each poll's
            # predicted-duration sleep must count from `fired_at`, not
            # from whenever its own poll call happens to start. Without
            # already_elapsed_s, the second leg polled here would blindly
            # sleep through its own full predicted duration *again*, on
            # top of however long the first leg's poll already took —
            # even though (successfully duration-matched) it likely
            # finished at almost the same real time as the first leg.
            # That's what was making the REPL return ~10-15s after the
            # gantry had visibly already stopped.
            if responder == "z":
                self._poll_z_leg(move, current_pos, speed=responder_speed)
            elif responder == "theta":
                self._poll_theta_leg(move, current_theta, speed=responder_speed)
            else:  # "z_theta"
                self._poll_zt_leg(move, current_pos, current_theta, speed=responder_speed)
            self._poll_xy_leg(
                move, current_pos, speed=xy_speed,
                already_elapsed_s=time.monotonic() - fired_at,
            )
        finally:
            # Best-effort: if the try block above already failed (e.g. a
            # poll timeout), that's the failure the caller needs to see —
            # don't let a second failure here (e.g. a dropped connection)
            # replace it in the traceback and hide why the move actually
            # stopped. Log and move on instead of re-raising.
            try:
                if restore_responder_ramp is not None:
                    self._set_responder_ramp(responder, *restore_responder_ramp)
                if restore_responder_speed is not None:
                    self._set_responder_speed(responder, restore_responder_speed)
                if restore_xy_ramp is not None:
                    self._set_xy_ramp(*restore_xy_ramp)
                if restore_xy_speed is not None:
                    self._set_xy_speed(restore_xy_speed)
            except Exception:
                logger.exception(
                    "Failed to restore ramp/speed after a concurrent-pair move — "
                    "an axis may be left with a scaled-down ACL/DCL/SPD"
                )

    def _describe_linear(self, move: GCodeMove, touches_xy: bool, responder: str) -> str:
        """Render the ASCII commands _execute_linear would send, for dry-run logging.

        For a concurrent pair (touches_xy and responder != "none"), the
        real execute() scales one leg's speed/ramp down using a value read
        live from the controller (see _execute_concurrent_pair) — dry-run
        never touches hardware, so it can't reproduce that read, and shows
        the plain nominal feed rate on both legs with a note that the real
        run will differ.
        """
        parts = []
        if responder == "z":
            if move.feed_mm_s is not None:
                parts.append(f"{self._z_axis.token()} SPD {move.feed_mm_s:.6g}")
            parts.append(f"{self._z_axis.token()} BMT {move.target[2]:.6g}")
        elif responder == "theta":
            if move.feed_mm_s is not None:
                parts.append(f"{self._theta_axis.token()} SPD {move.feed_mm_s:.6g}")
            parts.append(f"{self._theta_axis.token()} BMT {move.theta:.6g}")
        elif responder == "z_theta":
            if move.feed_mm_s is not None:
                parts.append(f"C{self._theta_group_index} SPD {move.feed_mm_s:.6g}")
            parts.append(f"C{self._theta_group_index} BMT {move.target[2]:.6g} {move.theta:.6g}")
        if touches_xy:
            if move.feed_mm_s is not None:
                parts.append(f"C{self._group_index} SPD {move.feed_mm_s:.6g}")
            fmt_pos = " ".join(f"{v:.6g}" for v in move.target[:2])
            parts.append(f"C{self._group_index} BMT {fmt_pos}")
        if touches_xy and responder != "none" and move.feed_mm_s is not None:
            parts.append(
                "(concurrent pair: one leg's speed/ramp will be scaled down "
                "from a live controller read at execute time — not shown here)"
            )
        return "; ".join(parts)

    def _poll_group_move_finished(
        self, timeout_s: Optional[float] = None, predicted_s: float = 0.0
    ) -> None:
        """Block until the X/Y group's move-finished flag is set, aborting the move on timeout.

        Polls sparsely via commands.poll_until_move_finished — querying
        C<n> MIF in a tight loop during group interpolation is what
        destabilized this controller; see that function's module note.

        Args:
            timeout_s: Maximum seconds to wait before aborting. Defaults
                to poll_until_move_finished's own predicted-duration-scaled
                timeout when omitted — see that function's docstring.
            predicted_s: Expected move duration, slept through before the
                first query (see _predicted_move_s).

        Raises:
            SnapMotionError: If the move hasn't finished within
                `timeout_s`. The in-progress group move is aborted
                (`group_abort()`) before raising.
        """
        if not poll_until_move_finished(
            self._cmd.group_move_is_finished, predicted_s=predicted_s, timeout_s=timeout_s
        ):
            self._cmd.group_abort()
            effective_timeout = resolve_timeout_s(predicted_s, timeout_s)
            raise SnapMotionError(
                0, f"X/Y group move did not finish within {effective_timeout:.0f}s — aborted"
            )

    def _poll_theta_group_move_finished(
        self, timeout_s: Optional[float] = None, predicted_s: float = 0.0
    ) -> None:
        """Block until the Z/Theta group's move-finished flag is set, aborting the move on timeout.

        Mirrors _poll_group_move_finished, against `theta_cmd` instead of
        `cmd` — same sparse-polling discipline, same reason, same
        `timeout_s` default.

        Raises:
            SnapMotionError: If the move hasn't finished within
                `timeout_s`. The in-progress group move is aborted
                (`group_abort()`) before raising.
        """
        if not poll_until_move_finished(
            self._theta_cmd.group_move_is_finished, predicted_s=predicted_s, timeout_s=timeout_s
        ):
            self._theta_cmd.group_abort()
            effective_timeout = resolve_timeout_s(predicted_s, timeout_s)
            raise SnapMotionError(
                0, f"Z/Theta group move did not finish within {effective_timeout:.0f}s — aborted"
            )

    def _poll_axis_move_finished(
        self, axis: Axis, timeout_s: Optional[float] = None, predicted_s: float = 0.0
    ) -> None:
        """Block until a single axis's move-finished flag is set, aborting the move on timeout.

        Used for the independent Z and Theta legs. Polls sparsely for the
        same reason as the group pollers (see commands.py) — single-axis
        polling was never shown to destabilize the controller the way
        group polling was, but there's no reason to query harder than
        needed. Same `timeout_s` default as the group pollers.

        Raises:
            SnapMotionError: If the move hasn't finished within
                `timeout_s`. The in-progress move is aborted
                (`abort(axis)`) before raising.
        """
        if not poll_until_move_finished(
            lambda: self._cmd.move_is_finished(axis), predicted_s=predicted_s, timeout_s=timeout_s
        ):
            self._cmd.abort(axis)
            effective_timeout = resolve_timeout_s(predicted_s, timeout_s)
            raise SnapMotionError(
                0, f"{axis.name} move did not finish within {effective_timeout:.0f}s — aborted"
            )

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
            logger.debug("home_all() (G28)")
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
        logger.debug("dwell %.3fs", move.dwell_s or 0.0)
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
