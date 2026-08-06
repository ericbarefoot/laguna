"""Exclusion zone (fence) geometry and trajectory safety checking.

Fences define regions of the 3D work envelope the robot must never enter.
The enforcement design makes it structurally impossible to execute a trajectory
without first checking it: GCodeExecutor only accepts a CheckedTrajectory, which
can only be created by TrajectoryChecker — so the check cannot be skipped.

Fence types:
  BoxFence       — axis-aligned bounding box
  CylinderFence  — vertical cylinder (useful for mounting posts, sensors)

Checking is done by sampling each line segment at resolution_mm intervals and
testing every interpolated point. This catches trajectories that pass through a
fence without landing a waypoint inside it.

Only XYZ coordinates are checked. The Theta (rotational) axis is not considered
in spatial fence tests — if instrument orientation affects reach, define a BoxFence
that conservatively covers the swept volume.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from typing import Sequence

Point3D = tuple[float, float, float]

DEFAULT_RESOLUTION_MM = 0.5


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

class TrajectoryChecker:
    """Validates waypoint sequences against a FenceRegistry.

    Segments are sampled at resolution_mm intervals so that a trajectory
    passing *through* a fence is caught even if no waypoint lands inside it.
    """

    def __init__(
        self,
        registry: FenceRegistry,
        resolution_mm: float = DEFAULT_RESOLUTION_MM,
    ):
        """Bind a checker to a fence registry.

        Args:
            registry: Fences to check trajectories against.
            resolution_mm: Sampling interval along each segment.

        Raises:
            ValueError: ``resolution_mm`` is not positive.
        """
        if resolution_mm <= 0:
            raise ValueError(f"resolution_mm must be positive, got {resolution_mm}")
        self._registry = registry
        self.resolution_mm = resolution_mm

    def check_point(self, x: float, y: float, z: float) -> list[FenceViolation]:
        """Return violations for a single point (may hit more than one fence).

        Args:
            x: X coordinate.
            y: Y coordinate.
            z: Z coordinate.

        Returns:
            List of FenceViolation objects for any fences containing this point.
        """
        return [
            FenceViolation(f, (x, y, z))
            for f in self._registry.list()
            if f.contains(x, y, z)
        ]

    def check_segment(
        self, p1: Point3D, p2: Point3D
    ) -> list[FenceViolation]:
        """Sample p1→p2 at resolution_mm steps; return all fence violations found.

        Args:
            p1: Segment start point (x, y, z).
            p2: Segment end point (x, y, z).

        Returns:
            List of FenceViolation objects for any fence intersections found.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)

        if length == 0.0:
            return [
                FenceViolation(f, p1, (p1, p2))
                for f in self._registry.list()
                if f.contains(*p1)
            ]

        n_steps = max(1, math.ceil(length / self.resolution_mm))
        violations: list[FenceViolation] = []
        seen: set[str] = set()  # avoid duplicate violations for the same fence

        for i in range(n_steps + 1):
            t = i / n_steps
            x = p1[0] + t * dx
            y = p1[1] + t * dy
            z = p1[2] + t * dz
            for fence in self._registry.list():
                if fence.name not in seen and fence.contains(x, y, z):
                    violations.append(FenceViolation(fence, (x, y, z), (p1, p2)))
                    seen.add(fence.name)  # one violation per fence per segment

        return violations

    def check_trajectory(self, waypoints: Sequence[Point3D]) -> list[FenceViolation]:
        """Check every segment in a full waypoint sequence. Returns all violations."""
        if len(waypoints) < 2:
            if len(waypoints) == 1:
                return self.check_point(*waypoints[0])
            return []

        violations: list[FenceViolation] = []
        for i in range(len(waypoints) - 1):
            violations.extend(self.check_segment(waypoints[i], waypoints[i + 1]))
        return violations

    def check_and_wrap(self, waypoints: Sequence[Point3D]) -> CheckedTrajectory:
        """Check the trajectory and return a CheckedTrajectory if safe.

        Raises FenceViolation (the first one found) if any part of the path
        enters an exclusion zone. This is the primary entry point for callers
        who want the check-then-execute pattern enforced.

        To inspect ALL violations before deciding what to do, call
        check_trajectory() directly instead.
        """
        violations = self.check_trajectory(waypoints)
        if violations:
            raise violations[0]
        return CheckedTrajectory(
            waypoints=list(waypoints),
            checked_at=datetime.now(),
        )
