"""Exclusion zone (fence) geometry and trajectory safety checking.

Fences define regions of the 3D work envelope the robot must never enter.
The enforcement design makes it structurally impossible to execute a trajectory
without first checking it: GCodeExecutor only accepts a CheckedTrajectory, which
can only be created by TrajectoryChecker — so the check cannot be skipped.

Fence types:
  BoxFence       — axis-aligned bounding box
  CylinderFence  — vertical cylinder (useful for mounting posts, sensors)

Checking is analytic, not sampled: each fence answers exactly whether a
segment (or a swept region, below) touches it. Sampling every 0.5 mm used to
miss a fence thinner than that, or a path grazing a cylinder along a chord
shorter than one step.

Two swept shapes are checked:

- ``check_segment`` — a straight 3D line. Used for single-axis moves (scan
  passes), where the path really is a line.
- ``check_ribbon`` — the X/Y straight line, with Z free to be anywhere in its
  start–end range at any point along it. This is what a G-code move that
  changes both X/Y and Z actually sweeps on this hardware: X/Y run as one
  coordinated group (so they stay on the line), but Z is on a different PLC
  node and runs as an independent leg — see gcode.py's module docstring. The
  old check (two "elbow" paths at the corners of the bounding box) missed the
  interior entirely, including the straight diagonal itself.

Non-finite coordinates (NaN, inf) raise ValueError rather than passing:
every comparison with NaN is false, so a NaN would otherwise sail through.

Only XYZ coordinates are checked. The Theta (rotational) axis is not considered
in spatial fence tests — if instrument orientation affects reach, define a BoxFence
that conservatively covers the swept volume.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, Sequence

Point3D = tuple[float, float, float]


# ---------------------------------------------------------------------------
# Fence types
# ---------------------------------------------------------------------------

class Fence(ABC):
    """A 3D exclusion-zone geometry that trajectories are checked against."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Unique identifier, used in the registry and violation messages."""
        ...

    @abstractmethod
    def contains(self, x: float, y: float, z: float) -> bool:
        """Return True if the point (x, y, z) is inside this exclusion zone."""
        ...

    @abstractmethod
    def describe(self) -> str:
        """Human-readable description for violation messages."""
        ...

    @abstractmethod
    def first_hit_on_ribbon(
        self, start: Point3D, end: Point3D, z_lo: float, z_hi: float
    ) -> Optional[Point3D]:
        """First point inside this fence on the X/Y line start→end, with Z anywhere in [z_lo, z_hi].

        A straight 3D segment is checked by passing ``z_lo == z_hi`` per
        sub-interval — see TrajectoryChecker.check_segment, which handles
        the Z-varies-with-X/Y case separately.

        Returns:
            A representative violating point, or None if the region misses
            this fence entirely.
        """
        ...

    @abstractmethod
    def first_hit_on_segment(self, start: Point3D, end: Point3D) -> Optional[Point3D]:
        """First point inside this fence along the straight 3D segment start→end, or None."""
        ...


def _clip_slab(p: float, d: float, lo: float, hi: float, t0: float, t1: float) -> Optional[tuple[float, float]]:
    """Clip parameter interval [t0, t1] of p + t*d to the slab lo <= x <= hi (inclusive)."""
    if d == 0.0:
        return (t0, t1) if lo <= p <= hi else None
    ta = (lo - p) / d
    tb = (hi - p) / d
    if ta > tb:
        ta, tb = tb, ta
    t0 = max(t0, ta)
    t1 = min(t1, tb)
    return (t0, t1) if t0 <= t1 else None


def _clip_disc(
    px: float, py: float, dx: float, dy: float, cx: float, cy: float, r: float,
    t0: float, t1: float,
) -> Optional[tuple[float, float]]:
    """Clip [t0, t1] of (px, py) + t*(dx, dy) to the closed disc of radius r about (cx, cy)."""
    fx, fy = px - cx, py - cy
    a = dx * dx + dy * dy
    c = fx * fx + fy * fy - r * r
    if a == 0.0:
        return (t0, t1) if c <= 0.0 else None
    b = 2.0 * (fx * dx + fy * dy)
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    t0 = max(t0, (-b - root) / (2.0 * a))
    t1 = min(t1, (-b + root) / (2.0 * a))
    return (t0, t1) if t0 <= t1 else None


def _at(start: Point3D, end: Point3D, t: float) -> Point3D:
    return (
        start[0] + t * (end[0] - start[0]),
        start[1] + t * (end[1] - start[1]),
        start[2] + t * (end[2] - start[2]),
    )


@dataclass
class BoxFence(Fence):
    """Axis-aligned bounding box exclusion zone.

    Example — protect a sensor mount between x=100–150, y=200–250, z=0–200::

        BoxFence("sensor_mount", 100, 150, 200, 250, 0, 200)
    """
    _name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @property
    def name(self) -> str:
        """Unique identifier for this fence."""
        return self._name

    def contains(self, x: float, y: float, z: float) -> bool:
        """True if (x, y, z) is inside the box, inclusive of its bounds."""
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )

    def first_hit_on_segment(self, start: Point3D, end: Point3D) -> Optional[Point3D]:
        """First point inside the box along start→end, or None."""
        span: Optional[tuple[float, float]] = (0.0, 1.0)
        for i, (lo, hi) in enumerate(
            ((self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        ):
            span = _clip_slab(start[i], end[i] - start[i], lo, hi, *span)
            if span is None:
                return None
        return _at(start, end, span[0])

    def first_hit_on_ribbon(
        self, start: Point3D, end: Point3D, z_lo: float, z_hi: float
    ) -> Optional[Point3D]:
        """First point inside the box on the X/Y line, Z anywhere in [z_lo, z_hi], or None."""
        if z_hi < self.z_min or z_lo > self.z_max:
            return None
        span: Optional[tuple[float, float]] = (0.0, 1.0)
        for i, (lo, hi) in enumerate(((self.x_min, self.x_max), (self.y_min, self.y_max))):
            span = _clip_slab(start[i], end[i] - start[i], lo, hi, *span)
            if span is None:
                return None
        x, y, _ = _at(start, end, span[0])
        return (x, y, min(max(z_lo, self.z_min), self.z_max))

    def describe(self) -> str:
        """Human-readable description for violation messages."""
        return (
            f"BoxFence({self._name!r} "
            f"x=[{self.x_min},{self.x_max}] "
            f"y=[{self.y_min},{self.y_max}] "
            f"z=[{self.z_min},{self.z_max}])"
        )


@dataclass
class CylinderFence(Fence):
    """Vertical cylinder exclusion zone.

    Useful for upright posts, sensor probes, cable bundles, and other
    obstacles with circular cross-sections.

    Example — protect a 30 mm radius post at x=300, y=100::

        CylinderFence("flow_sensor_post", 300.0, 100.0, 30.0, 0.0, 300.0)
    """
    _name: str
    center_x: float
    center_y: float
    radius: float
    z_min: float
    z_max: float

    @property
    def name(self) -> str:
        """Unique identifier for this fence."""
        return self._name

    def contains(self, x: float, y: float, z: float) -> bool:
        """True if (x, y, z) is inside the cylinder, inclusive of its bounds."""
        return (
            math.hypot(x - self.center_x, y - self.center_y) <= self.radius
            and self.z_min <= z <= self.z_max
        )

    def first_hit_on_segment(self, start: Point3D, end: Point3D) -> Optional[Point3D]:
        """First point inside the cylinder along start→end, or None."""
        span = _clip_slab(start[2], end[2] - start[2], self.z_min, self.z_max, 0.0, 1.0)
        if span is None:
            return None
        span = _clip_disc(
            start[0], start[1], end[0] - start[0], end[1] - start[1],
            self.center_x, self.center_y, self.radius, *span,
        )
        return None if span is None else _at(start, end, span[0])

    def first_hit_on_ribbon(
        self, start: Point3D, end: Point3D, z_lo: float, z_hi: float
    ) -> Optional[Point3D]:
        """First point inside the cylinder on the X/Y line, Z anywhere in [z_lo, z_hi], or None."""
        if z_hi < self.z_min or z_lo > self.z_max:
            return None
        span = _clip_disc(
            start[0], start[1], end[0] - start[0], end[1] - start[1],
            self.center_x, self.center_y, self.radius, 0.0, 1.0,
        )
        if span is None:
            return None
        x, y, _ = _at(start, end, span[0])
        return (x, y, min(max(z_lo, self.z_min), self.z_max))

    def describe(self) -> str:
        """Human-readable description for violation messages."""
        return (
            f"CylinderFence({self._name!r} "
            f"center=({self.center_x},{self.center_y}) "
            f"r={self.radius} "
            f"z=[{self.z_min},{self.z_max}])"
        )


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

class FenceRegistry:
    """Mutable collection of active fences."""

    def __init__(self) -> None:
        """Create an empty registry."""
        self._fences: dict[str, Fence] = {}

    def add(self, fence: Fence) -> None:
        """Register a fence.

        Args:
            fence: The fence to add.

        Raises:
            ValueError: A fence with this name is already registered.
        """
        if fence.name in self._fences:
            raise ValueError(f"A fence named {fence.name!r} already exists. Remove it first.")
        self._fences[fence.name] = fence

    def remove(self, name: str) -> None:
        """Unregister a fence by name.

        Raises:
            KeyError: No fence with this name is registered.
        """
        if name not in self._fences:
            raise KeyError(f"No fence named {name!r}")
        del self._fences[name]

    def get(self, name: str) -> Fence:
        """Look up a registered fence by name.

        Raises:
            KeyError: No fence with this name is registered.
        """
        return self._fences[name]

    def list(self) -> list[Fence]:
        """All registered fences, in no particular order."""
        return list(self._fences.values())

    def __len__(self) -> int:
        """Count of registered fences."""
        return len(self._fences)

    def __contains__(self, name: str) -> bool:
        """True if a fence with this name is registered."""
        return name in self._fences


# ---------------------------------------------------------------------------
# Violation type
# ---------------------------------------------------------------------------

class FenceViolation(Exception):
    """Raised when a trajectory intersects an exclusion zone.

    Carries the specific fence hit, the violating point, and — for segment
    checks — the segment endpoints so the caller can report or visualize it.
    """

    def __init__(
        self,
        fence: Fence,
        point: Point3D,
        segment: tuple[Point3D, Point3D] | None = None,
    ):
        """Build the violation and its message.

        Args:
            fence: The exclusion zone that was entered.
            point: The specific (x, y, z) found inside the fence.
            segment: The trajectory segment (start, end) being checked when
                the violation was found, if this came from a segment check
                rather than a single-point check.
        """
        self.fence = fence
        self.point = point
        self.segment = segment
        loc = f"point {point}" if segment is None else f"segment {segment[0]}→{segment[1]}"
        super().__init__(
            f"Trajectory enters exclusion zone {fence.describe()} at {loc} "
            f"(x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f})"
        )


# ---------------------------------------------------------------------------
# Checked trajectory — the type GCodeExecutor requires
# ---------------------------------------------------------------------------

@dataclass
class CheckedTrajectory:
    """A waypoint sequence that has been validated against the active fence registry.

    GCodeExecutor.execute() only accepts this type — passing a raw list or
    string raises TypeError. This makes it structurally impossible to send
    motion commands without the fence check having been run.

    If violations is non-empty the trajectory is NOT safe; GCodeExecutor will
    refuse to execute it. TrajectoryChecker.check_and_wrap() raises FenceViolation
    before creating a CheckedTrajectory with violations, so in normal usage this
    list will always be empty.
    """
    waypoints: list[Point3D]
    checked_at: datetime
    violations: list[FenceViolation] = field(default_factory=list)

    @property
    def is_safe(self) -> bool:
        """True if no violations were recorded.

        Always true in normal usage — see class docstring.
        """
        return len(self.violations) == 0


# ---------------------------------------------------------------------------
# Trajectory checker
# ---------------------------------------------------------------------------

def _require_finite(*points: Point3D) -> None:
    for point in points:
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError(f"Fence check needs finite (x, y, z) coordinates, got {point!r}")


class TrajectoryChecker:
    """Validates waypoint sequences against a FenceRegistry.

    Exact, not sampled — see the module docstring for the two swept shapes
    (``check_segment`` and ``check_ribbon``) and which moves use which.
    """

    def __init__(self, registry: FenceRegistry):
        """Bind a checker to a fence registry.

        Args:
            registry: Fences to check trajectories against.
        """
        self._registry = registry

    def check_point(self, x: float, y: float, z: float) -> list[FenceViolation]:
        """Return violations for a single point (may hit more than one fence).

        Args:
            x: X coordinate.
            y: Y coordinate.
            z: Z coordinate.

        Returns:
            List of FenceViolation objects for any fences containing this point.

        Raises:
            ValueError: A coordinate is NaN or infinite.
        """
        _require_finite((x, y, z))
        return [
            FenceViolation(f, (x, y, z))
            for f in self._registry.list()
            if f.contains(x, y, z)
        ]

    def check_segment(self, p1: Point3D, p2: Point3D) -> list[FenceViolation]:
        """Return one violation per fence the straight segment p1→p2 touches.

        Args:
            p1: Segment start point (x, y, z).
            p2: Segment end point (x, y, z).

        Raises:
            ValueError: A coordinate is NaN or infinite.
        """
        _require_finite(p1, p2)
        violations: list[FenceViolation] = []
        for fence in self._registry.list():
            hit = fence.first_hit_on_segment(p1, p2)
            if hit is not None:
                violations.append(FenceViolation(fence, hit, (p1, p2)))
        return violations

    def check_ribbon(self, p1: Point3D, p2: Point3D) -> list[FenceViolation]:
        """Like check_segment, but with Z free anywhere in its p1–p2 range along the X/Y line.

        The swept region of a move whose X/Y and Z legs run independently —
        see the module docstring.

        Raises:
            ValueError: A coordinate is NaN or infinite.
        """
        _require_finite(p1, p2)
        z_lo, z_hi = min(p1[2], p2[2]), max(p1[2], p2[2])
        violations: list[FenceViolation] = []
        for fence in self._registry.list():
            hit = fence.first_hit_on_ribbon(p1, p2, z_lo, z_hi)
            if hit is not None:
                violations.append(FenceViolation(fence, hit, (p1, p2)))
        return violations

    def check_trajectory(
        self, waypoints: Sequence[Point3D], independent_z: bool = False
    ) -> list[FenceViolation]:
        """Check every leg of a waypoint sequence. Returns all violations.

        Args:
            waypoints: Points visited in order.
            independent_z: Check each leg as a ribbon (check_ribbon) rather
                than a straight segment — for G-code moves, whose Z leg
                isn't coordinated with X/Y.
        """
        if len(waypoints) < 2:
            if len(waypoints) == 1:
                return self.check_point(*waypoints[0])
            return []

        check = self.check_ribbon if independent_z else self.check_segment
        violations: list[FenceViolation] = []
        for i in range(len(waypoints) - 1):
            violations.extend(check(waypoints[i], waypoints[i + 1]))
        return violations

    def check_and_wrap(
        self, waypoints: Sequence[Point3D], independent_z: bool = False
    ) -> CheckedTrajectory:
        """Check the trajectory and return a CheckedTrajectory if safe.

        Raises FenceViolation (the first one found) if any part of the path
        enters an exclusion zone. This is the primary entry point for callers
        who want the check-then-execute pattern enforced.

        To inspect ALL violations before deciding what to do, call
        check_trajectory() directly instead.

        Args:
            waypoints: Points visited in order.
            independent_z: See check_trajectory().
        """
        violations = self.check_trajectory(waypoints, independent_z=independent_z)
        if violations:
            raise violations[0]
        return CheckedTrajectory(
            waypoints=list(waypoints),
            checked_at=datetime.now(),
        )
