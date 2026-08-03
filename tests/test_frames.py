"""Tests for affine reference frames.

The headline behaviour: two instruments mounted at different places on the
carriage, asked for the same experiment coordinate, must produce different
gantry commands that land their measurement points on the same physical spot.
"""

import numpy as np
import pytest

from laguna.frames import AffineTransform, FrameRegistry, InstrumentFrame
from laguna.scanner.mounting import SensorMounting
from laguna.scanner.pointcloud import SurfaceScan

#: Representative rig: sensors offset on the carriage, origin at a corner.
RIG = {
    "experiment": {"translation": [500.0, 300.0, 0.0]},
    "instruments": {
        "gocator": {"translation": [0.0, 0.0, -325.0]},
        "od2000": {"translation": [52.0, -18.0, 0.0]},
        "wtt12l": {"translation": [52.0, 31.0, 0.0]},
    },
}


class TestAffineTransform:
    def test_identity_leaves_points_alone(self):
        p = np.array([1.0, 2.0, 3.0])
        np.testing.assert_allclose(AffineTransform.identity().apply(p), p)

    def test_translation(self):
        t = AffineTransform.from_translation([10, 20, 30])
        np.testing.assert_allclose(t.apply([1.0, 2.0, 3.0]), [11, 22, 33])

    def test_rotation_z_90_degrees(self):
        t = AffineTransform.from_rotation_z(90.0)
        np.testing.assert_allclose(t.apply([1.0, 0.0, 0.0]), [0, 1, 0], atol=1e-12)

    def test_accepts_single_point_and_array(self):
        t = AffineTransform.from_translation([1, 0, 0])
        assert t.apply(np.zeros(3)).shape == (3,)
        assert t.apply(np.zeros((5, 3))).shape == (5, 3)

    def test_inverse_round_trips(self):
        t = AffineTransform.from_translation([3, -4, 5]) @ AffineTransform.from_rotation_z(37.0)
        p = np.random.default_rng(0).normal(size=(20, 3))
        np.testing.assert_allclose(t.inverse().apply(t.apply(p)), p, atol=1e-9)

    def test_composition_order_is_left_applied_last(self):
        rot = AffineTransform.from_rotation_z(90.0)
        shift = AffineTransform.from_translation([10, 0, 0])
        # (shift @ rot) rotates first, then shifts
        np.testing.assert_allclose(
            (shift @ rot).apply([1.0, 0.0, 0.0]), [10, 1, 0], atol=1e-12
        )
        np.testing.assert_allclose(
            (rot @ shift).apply([1.0, 0.0, 0.0]), [0, 11, 0], atol=1e-12
        )

    def test_transform_is_rigid(self):
        t = AffineTransform.from_translation([5, 6, 7]) @ AffineTransform.from_rotation_z(23.0)
        pts = np.random.default_rng(1).normal(size=(50, 3))
        out = t.apply(pts)
        np.testing.assert_allclose(
            np.linalg.norm(pts[:25] - pts[25:], axis=1),
            np.linalg.norm(out[:25] - out[25:], axis=1),
        )

    def test_scaling_matrix_rejected(self):
        """A scale would silently resize real geometry."""
        m = np.eye(4)
        m[0, 0] = 2.0
        with pytest.raises(ValueError, match="scales or shears"):
            AffineTransform(m)

    def test_mirroring_matrix_rejected(self):
        m = np.eye(4)
        m[0, 0] = -1.0
        with pytest.raises(ValueError, match="mirrors the data"):
            AffineTransform(m)

    def test_bad_shape_and_bottom_row_rejected(self):
        with pytest.raises(ValueError, match="must be 4x4"):
            AffineTransform(np.eye(3))
        m = np.eye(4)
        m[3, 0] = 1.0
        with pytest.raises(ValueError, match="bottom row"):
            AffineTransform(m)

    def test_from_axis_map_matches_sensor_mounting(self):
        spec = {"scan_x": "-Y", "scan_y": "+X", "scan_z": "+Z"}
        t = AffineTransform.from_axis_map(**spec)
        np.testing.assert_allclose(
            t.matrix[:3, :3], SensorMounting(**spec).matrix
        )

    def test_from_axis_map_rejects_mirroring(self):
        with pytest.raises(ValueError, match="mirrors"):
            AffineTransform.from_axis_map(scan_x="+Y", scan_y="+X", scan_z="+Z")

    def test_from_config_applies_rotation_before_translation(self):
        """translation reads in the target frame — 'move the origin there'."""
        t = AffineTransform.from_config({"rotation_deg": 90.0, "translation": [10, 0, 0]})
        np.testing.assert_allclose(t.apply([1.0, 0.0, 0.0]), [10, 1, 0], atol=1e-12)

    def test_from_config_empty_is_identity(self):
        assert AffineTransform.from_config(None).is_identity
        assert AffineTransform.from_config({}).is_identity

    def test_from_config_unknown_key_rejected(self):
        with pytest.raises(ValueError, match="unknown transform key"):
            AffineTransform.from_config({"offset": [1, 2, 3]})

    def test_to_dict_round_trips(self):
        t = AffineTransform.from_translation([1, 2, 3]) @ AffineTransform.from_rotation_z(45.0)
        np.testing.assert_allclose(AffineTransform.from_config(t.to_dict()).matrix, t.matrix)


class TestFrameRegistry:
    @pytest.fixture
    def registry(self):
        return FrameRegistry.from_config(RIG)

    def test_unconfigured_instrument_has_zero_offset(self, registry):
        """The honest default: assume it measures at the commanded point, so
        a rig with no frames: section behaves exactly as before."""
        np.testing.assert_allclose(registry.frame_for("mystery").offset, [0, 0, 0])

    def test_offsets_from_config(self, registry):
        np.testing.assert_allclose(registry.frame_for("od2000").offset, [52, -18, 0])

    def test_two_instruments_hit_the_same_physical_point(self, registry):
        """The whole point of the feature."""
        target = [100.0, 200.0, 0.0]
        for name in ("od2000", "wtt12l"):
            gantry = registry.gantry_target_for(name, target)
            landed = registry.to_experiment(name, [0.0, 0.0, 0.0], gantry)
            np.testing.assert_allclose(landed, target, atol=1e-9)

    def test_different_instruments_need_different_gantry_commands(self, registry):
        target = [100.0, 200.0, 0.0]
        a = registry.gantry_target_for("od2000", target)
        b = registry.gantry_target_for("wtt12l", target)
        assert not np.allclose(a, b)
        # exactly the difference of their mounts
        np.testing.assert_allclose(a - b, [0.0, 49.0, 0.0])

    def test_retarget_is_independent_of_experiment_frame(self):
        """Swapping instruments at a known gantry position is purely the
        difference of the two mounts, so the experiment origin can't matter."""
        with_frame = FrameRegistry.from_config(RIG)
        without = FrameRegistry.from_config(
            {"instruments": RIG["instruments"]}
        )
        a = with_frame.retarget("od2000", "wtt12l", [10, 10, 0])
        b = without.retarget("od2000", "wtt12l", [10, 10, 0])
        np.testing.assert_allclose(a, b)
        np.testing.assert_allclose(a, [10.0, -39.0, 0.0])

    def test_retarget_puts_the_second_instrument_where_the_first_was(self, registry):
        start = np.array([10.0, 10.0, 0.0])
        seen = registry.to_experiment("od2000", [0, 0, 0], start)
        moved = registry.retarget("od2000", "wtt12l", start)
        np.testing.assert_allclose(
            registry.to_experiment("wtt12l", [0, 0, 0], moved), seen, atol=1e-9
        )

    def test_experiment_frame_round_trips(self, registry):
        p = np.array([[1.0, 2.0, 3.0]])
        np.testing.assert_allclose(
            registry.experiment_to_gantry(registry.gantry_to_experiment(p)), p, atol=1e-9
        )

    def test_experiment_offset_makes_coordinates_positive(self, registry):
        """The motivating use case: origin at a corner, everything positive."""
        assert (registry.gantry_to_experiment(np.array([-400.0, -250.0, 5.0])) > 0).all()

    def test_gantry_position_shifts_observations(self, registry):
        a = registry.to_experiment("od2000", [0, 0, 0], [0, 0, 0])
        b = registry.to_experiment("od2000", [0, 0, 0], [10, 0, 0])
        np.testing.assert_allclose(b - a, [10, 0, 0])

    def test_unknown_frames_key_rejected(self):
        with pytest.raises(ValueError, match="unknown frames key"):
            FrameRegistry.from_config({"instrument": {}})

    def test_describe_summarises_offsets(self, registry):
        d = registry.describe()
        assert d["instruments"]["wtt12l"] == [52.0, 31.0, 0.0]
        assert d["experiment_is_identity"] is False

    def test_reference_point_offsets_the_measurement(self):
        """A sensor whose dot isn't at its own frame origin."""
        r = FrameRegistry().add(
            InstrumentFrame(
                "probe",
                mount=AffineTransform.from_translation([10, 0, 0]),
                reference_point=[0, 0, -50],
            )
        )
        np.testing.assert_allclose(r.frame_for("probe").offset, [10, 0, -50])
        np.testing.assert_allclose(
            r.gantry_target_for("probe", [0, 0, 0]), [-10, 0, 50]
        )

    def test_bad_reference_point_rejected(self):
        with pytest.raises(ValueError, match="reference_point"):
            InstrumentFrame("x", reference_point=[1, 2])


def make_scan(mounting=None, **meta):
    return SurfaceScan(
        z_mm=np.array([[1.0, 2.0], [3.0, 4.0]]),
        x_mm=np.array([0.0, 10.0]),
        y_mm=np.array([0.0, 20.0]),
        metadata=meta,
        is_uniform=True,
        mounting=mounting or SensorMounting(),
    )


class TestPlaceScan:
    def test_places_a_scan_using_metadata_start(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        pts = registry.place_scan(scan)
        # travel (sensor Y, 0..20) sits on gantry X from 700, then +500 origin
        assert pts[:, 0].min() == pytest.approx(1200.0)
        assert pts[:, 2].min() == pytest.approx(1.0 - 325.0)

    def test_explicit_gantry_start_overrides_metadata(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        pts = registry.place_scan(scan, gantry_start=[0.0, 0.0, 0.0])
        assert pts[:, 0].min() == pytest.approx(500.0)

    def test_missing_start_position_raises(self):
        with pytest.raises(ValueError, match="needs the gantry position"):
            FrameRegistry.from_config(RIG).place_scan(make_scan())

    def test_unknown_axis_in_metadata_raises(self):
        scan = make_scan(gantry_axis="Theta", gantry_start_mm=1.0)
        with pytest.raises(ValueError, match="X/Y/Z"):
            FrameRegistry.from_config(RIG).place_scan(scan)

    def test_double_rotation_is_refused(self):
        """The scan already rotates itself into gantry orientation; a
        rotation in the instrument frame too would turn it twice."""
        registry = FrameRegistry.from_config(
            {"instruments": {"gocator": {"axes": {"scan_x": "-Y", "scan_y": "+X", "scan_z": "+Z"}}}}
        )
        scan = make_scan(
            SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z"),
            gantry_axis="X",
            gantry_start_mm=0.0,
        )
        with pytest.raises(ValueError, match="would turn the data twice"):
            registry.place_scan(scan)

    def test_translation_only_frame_is_fine_with_a_rotated_scan(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(
            SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z"),
            gantry_axis="X",
            gantry_start_mm=0.0,
        )
        assert registry.place_scan(scan).shape == (4, 3)
