"""Tests for exclusion-zone (fence) geometry and trajectory safety checking."""

import pytest

from laguna.robot.macron.fences import (
    BoxFence,
    CheckedTrajectory,
    CylinderFence,
    FenceRegistry,
    FenceViolation,
    TrajectoryChecker,
)


class TestBoxFence:
    def test_contains_point_inside(self):
        fence = BoxFence("post", 100, 150, 200, 250, 0, 200)
        assert fence.contains(125, 225, 100) is True

    def test_does_not_contain_point_outside(self):
        fence = BoxFence("post", 100, 150, 200, 250, 0, 200)
        assert fence.contains(0, 0, 0) is False

    def test_boundary_is_inclusive(self):
        fence = BoxFence("post", 100, 150, 200, 250, 0, 200)
        assert fence.contains(100, 200, 0) is True
        assert fence.contains(150, 250, 200) is True


class TestCylinderFence:
    def test_contains_point_within_radius(self):
        fence = CylinderFence("post", 300.0, 100.0, 30.0, 0.0, 300.0)
        assert fence.contains(310, 105, 50) is True

    def test_does_not_contain_point_outside_radius(self):
        fence = CylinderFence("post", 300.0, 100.0, 30.0, 0.0, 300.0)
        assert fence.contains(400, 400, 50) is False

    def test_does_not_contain_point_outside_z_range(self):
        fence = CylinderFence("post", 300.0, 100.0, 30.0, 0.0, 300.0)
        assert fence.contains(300, 100, 500) is False


class TestFenceRegistry:
    def test_add_and_get(self):
        registry = FenceRegistry()
        fence = BoxFence("a", 0, 1, 0, 1, 0, 1)
        registry.add(fence)
        assert registry.get("a") is fence
        assert len(registry) == 1
        assert "a" in registry

    def test_duplicate_name_rejected(self):
        registry = FenceRegistry()
        registry.add(BoxFence("a", 0, 1, 0, 1, 0, 1))
        with pytest.raises(ValueError):
            registry.add(BoxFence("a", 5, 6, 5, 6, 5, 6))

    def test_remove(self):
        registry = FenceRegistry()
        registry.add(BoxFence("a", 0, 1, 0, 1, 0, 1))
        registry.remove("a")
        assert "a" not in registry
        assert len(registry) == 0

    def test_remove_missing_raises(self):
        registry = FenceRegistry()
        with pytest.raises(KeyError):
            registry.remove("nope")


class TestTrajectoryChecker:
    def test_no_violation_when_registry_empty(self):
        checker = TrajectoryChecker(FenceRegistry())
        assert checker.check_trajectory([(0, 0, 0), (100, 100, 100)]) == []

    def test_waypoint_landing_inside_fence_detected(self):
        registry = FenceRegistry()
        registry.add(BoxFence("post", 40, 60, 40, 60, 0, 100))
        checker = TrajectoryChecker(registry)
        violations = checker.check_trajectory([(0, 0, 0), (50, 50, 50)])
        assert len(violations) == 1
        assert violations[0].fence.name == "post"

    def test_pass_through_fence_with_no_waypoint_inside_detected(self):
        # A straight line from (0,0,0) to (100,0,0) passes directly through
        # a fence centered at x=50, even though neither endpoint is inside it.
        registry = FenceRegistry()
        registry.add(BoxFence("obstacle", 40, 60, -10, 10, -10, 10))
        checker = TrajectoryChecker(registry)
        violations = checker.check_trajectory([(0, 0, 0), (100, 0, 0)])
        assert len(violations) == 1
        assert violations[0].fence.name == "obstacle"
        assert violations[0].segment == ((0, 0, 0), (100, 0, 0))

    def test_segment_missing_fence_entirely_no_violation(self):
        registry = FenceRegistry()
        registry.add(BoxFence("obstacle", 40, 60, -10, 10, -10, 10))
        checker = TrajectoryChecker(registry)
        violations = checker.check_trajectory([(0, 100, 0), (100, 100, 0)])
        assert violations == []

    def test_single_waypoint_checked_as_point(self):
        registry = FenceRegistry()
        registry.add(BoxFence("post", 0, 10, 0, 10, 0, 10))
        checker = TrajectoryChecker(registry)
        violations = checker.check_trajectory([(5, 5, 5)])
        assert len(violations) == 1

    def test_check_and_wrap_returns_checked_trajectory_when_safe(self):
        checker = TrajectoryChecker(FenceRegistry())
        result = checker.check_and_wrap([(0, 0, 0), (10, 10, 10)])
        assert isinstance(result, CheckedTrajectory)
        assert result.is_safe is True
        assert result.waypoints == [(0, 0, 0), (10, 10, 10)]

    def test_check_and_wrap_raises_on_violation(self):
        registry = FenceRegistry()
        registry.add(BoxFence("post", 0, 10, 0, 10, 0, 10))
        checker = TrajectoryChecker(registry)
        with pytest.raises(FenceViolation):
            checker.check_and_wrap([(5, 5, 5), (20, 20, 20)])

    def test_a_fence_thinner_than_any_sampling_step_is_still_caught(self):
        """Exact, not sampled: a 0.01 mm-thick wall used to slip between
        0.5 mm sample points."""
        registry = FenceRegistry()
        registry.add(BoxFence("sheet", 50.0, 50.01, -10, 10, -10, 10))
        checker = TrajectoryChecker(registry)
        assert len(checker.check_segment((0, 0, 0), (100, 0, 0))) == 1

    def test_a_short_chord_through_a_cylinder_is_caught(self):
        registry = FenceRegistry()
        registry.add(CylinderFence("post", 50.0, 0.0, 1.0, 0.0, 10.0))
        checker = TrajectoryChecker(registry)
        # y=0.99 cuts a ~0.28 mm chord — shorter than the old 0.5 mm step.
        assert len(checker.check_segment((0, 0.99, 5), (100, 0.99, 5))) == 1
        assert checker.check_segment((0, 1.01, 5), (100, 1.01, 5)) == []

    def test_touching_a_boundary_counts_as_entering(self):
        registry = FenceRegistry()
        registry.add(BoxFence("box", 10, 20, 0, 10, 0, 10))
        checker = TrajectoryChecker(registry)
        assert len(checker.check_segment((0, 5, 5), (10, 5, 5))) == 1

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
    def test_non_finite_coordinates_are_refused_not_passed(self, bad):
        """Every comparison with NaN is False, so a NaN point used to read
        as "inside no fence"."""
        registry = FenceRegistry()
        registry.add(BoxFence("everything", -1e9, 1e9, -1e9, 1e9, -1e9, 1e9))
        checker = TrajectoryChecker(registry)
        with pytest.raises(ValueError):
            checker.check_point(bad, 0, 0)
        with pytest.raises(ValueError):
            checker.check_segment((0, 0, 0), (bad, 0, 0))
        with pytest.raises(ValueError):
            checker.check_ribbon((0, 0, 0), (0, 0, bad))
        with pytest.raises(ValueError):
            checker.check_trajectory([(bad, 0, 0)])


class TestRibbon:
    """check_ribbon: X/Y straight, Z anywhere in its range along the line."""

    def test_catches_a_fence_on_the_straight_diagonal(self):
        """The old two-elbow check missed this: neither corner path touches
        a fence sitting on the diagonal itself."""
        registry = FenceRegistry()
        registry.add(BoxFence("post", 4, 6, 4, 6, 2, 3))
        checker = TrajectoryChecker(registry)
        assert len(checker.check_ribbon((0, 0, 0), (10, 10, 5))) == 1

    def test_catches_what_an_independent_z_leg_could_reach(self):
        """Z may finish early: at x=5 it could already be at 5, though the
        straight diagonal is only at 2.5 there."""
        registry = FenceRegistry()
        registry.add(BoxFence("shelf", 4, 6, -1, 1, 4.5, 6))
        checker = TrajectoryChecker(registry)
        assert checker.check_segment((0, 0, 0), (10, 0, 5)) == []
        assert len(checker.check_ribbon((0, 0, 0), (10, 0, 5))) == 1

    def test_ignores_fences_off_the_xy_line(self):
        registry = FenceRegistry()
        registry.add(BoxFence("beside", 4, 6, 20, 30, 0, 100))
        registry.add(CylinderFence("post", 5, 25, 2, 0, 100))
        checker = TrajectoryChecker(registry)
        assert checker.check_ribbon((0, 0, 0), (10, 10, 50)) == []

    def test_ignores_fences_outside_the_z_range(self):
        registry = FenceRegistry()
        registry.add(BoxFence("high", 0, 10, 0, 10, 60, 70))
        checker = TrajectoryChecker(registry)
        assert checker.check_ribbon((0, 0, 0), (10, 10, 50)) == []

    def test_check_trajectory_uses_ribbons_when_z_is_independent(self):
        registry = FenceRegistry()
        registry.add(BoxFence("shelf", 4, 6, -1, 1, 4.5, 6))
        checker = TrajectoryChecker(registry)
        waypoints = [(0, 0, 0), (10, 0, 5)]
        assert checker.check_trajectory(waypoints) == []
        assert len(checker.check_trajectory(waypoints, independent_z=True)) == 1


class TestCheckedTrajectory:
    def test_is_safe_true_when_no_violations(self):
        from datetime import datetime

        traj = CheckedTrajectory(waypoints=[(0, 0, 0)], checked_at=datetime.now())
        assert traj.is_safe is True

    def test_is_safe_false_when_violations_present(self):
        from datetime import datetime

        fence = BoxFence("a", 0, 1, 0, 1, 0, 1)
        violation = FenceViolation(fence, (0.5, 0.5, 0.5))
        traj = CheckedTrajectory(
            waypoints=[(0.5, 0.5, 0.5)], checked_at=datetime.now(), violations=[violation]
        )
        assert traj.is_safe is False
