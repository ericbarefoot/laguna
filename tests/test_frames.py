"""Tests for affine reference frames.

The headline behaviour: two instruments mounted at different places on the
carriage, asked for the same experiment coordinate, must produce different
gantry commands that land their measurement points on the same physical spot.
"""

import numpy as np
import pytest

from laguna.frames import AffineTransform, FrameRegistry, InstrumentFrame, orient_scan
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

    def test_experiment_point_for_inverts_gantry_target_for(self, registry):
        """The natural counterpart to gantry_target_for(): given wherever
        the gantry actually is, what experiment point is this instrument
        measuring right now."""
        target = [100.0, 200.0, 0.0]
        for name in ("od2000", "wtt12l", "gocator", "unconfigured"):
            gantry = registry.gantry_target_for(name, target)
            back = registry.experiment_point_for(name, gantry)
            np.testing.assert_allclose(back, target, atol=1e-9)

    def test_experiment_point_for_differs_per_instrument_at_the_same_gantry_position(self, registry):
        """Two instruments commanded to the same gantry position are NOT
        measuring the same experiment point — same asymmetry as
        gantry_target_for(), just read the other direction."""
        gantry = [500.0, 300.0, 0.0]
        a = registry.experiment_point_for("od2000", gantry)
        b = registry.experiment_point_for("wtt12l", gantry)
        assert not np.allclose(a, b)

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

    def test_gantry_target_for_reference_point_override(self, registry):
        """A one-off target other than the instrument's configured
        measurement point — e.g. a Gocator swath edge instead of its
        centerline — without touching the registered InstrumentFrame."""
        default = registry.gantry_target_for("gocator", [0, 0, 0])
        overridden = registry.gantry_target_for("gocator", [0, 0, 0], reference_point=[10, 0, 0])
        np.testing.assert_allclose(overridden - default, [-10, 0, 0])

    def test_gantry_target_for_reference_point_override_uses_mount_rotation(self):
        """The override still goes through the instrument's own mount
        rotation, not gantry-frame axes directly — this is what makes it
        correct for a rotated mount like the Gocator's."""
        r = FrameRegistry().add(
            InstrumentFrame(
                "gocator",
                mount=AffineTransform.from_axis_map(scan_x="-Y", scan_y="+X", scan_z="+Z"),
            )
        )
        # sensor +X (across the laser) maps to gantry -Y under this mount.
        target = r.gantry_target_for("gocator", [0, 0, 0], reference_point=[10, 0, 0])
        np.testing.assert_allclose(target, [0, 10, 0])


def make_scan(mounting=None, y_mm=None, x_mm=None, **meta):
    x_mm = np.array([0.0, 10.0]) if x_mm is None else x_mm
    y_mm = np.array([0.0, 20.0]) if y_mm is None else y_mm
    return SurfaceScan(
        z_mm=np.ones((len(y_mm), len(x_mm))),
        x_mm=x_mm,
        y_mm=y_mm,
        metadata=meta,
        is_uniform=True,
        mounting=mounting or SensorMounting(),
    )


class TestOrientScan:
    def test_places_a_scan_using_metadata_start(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        pts = orient_scan(scan, frames=registry).to_points()
        # travel (sensor Y, 0..20) sits on gantry X from 700, then +500 origin
        assert pts[:, 0].min() == pytest.approx(1200.0)
        assert pts[:, 2].min() == pytest.approx(1.0 - 325.0)

    def test_full_gantry_start_metadata_gives_the_real_static_axis_position(self):
        """gantry_start_mm alone only records the travel axis — the other
        two axes used to be silently assumed to be at 0, which is almost
        never true. scan_with_gantry() now also records the full commanded
        position as metadata['gantry_start']; when present, use it instead
        of guessing 0 for Y/Z."""
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(
            gantry_axis="X", gantry_start_mm=700.0,
            gantry_start=[700.0, 450.0, 20.0],  # real static Y/Z, not 0
        )
        pts = orient_scan(scan, frames=registry).to_points()
        # Y: real static 450 + 300 (RIG's experiment translation) = 750,
        # not 300 (which is what a wrongly-assumed-0 Y would have given).
        assert pts[:, 1].min() == pytest.approx(750.0)

    def test_explicit_gantry_start_overrides_metadata(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        pts = orient_scan(scan, frames=registry, gantry_start=[0.0, 0.0, 0.0]).to_points()
        assert pts[:, 0].min() == pytest.approx(500.0)

    def test_missing_start_position_raises(self):
        with pytest.raises(ValueError, match="needs the gantry position"):
            orient_scan(make_scan(), frames=FrameRegistry.from_config(RIG))

    def test_unknown_axis_in_metadata_raises(self):
        scan = make_scan(gantry_axis="Theta", gantry_start_mm=1.0)
        with pytest.raises(ValueError, match="X/Y/Z"):
            orient_scan(scan, frames=FrameRegistry.from_config(RIG))

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
            orient_scan(scan, frames=registry)

    def test_translation_only_frame_is_fine_with_a_rotated_scan(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(
            SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z"),
            gantry_axis="X",
            gantry_start_mm=0.0,
        )
        assert orient_scan(scan, frames=registry).to_points().shape == (4, 3)

    def test_negative_direction_pass_is_not_mirrored(self):
        """The sensor is encoderless: its own Y is just acquisition order,
        centred symmetrically around 0 regardless of which real-world
        direction the gantry moved. Without correcting for the recorded
        travel direction (gantry_start_mm -> gantry_end_mm), a pass that
        travels in the negative direction along its axis comes out mirrored
        — this is the bug this test guards against.
        """
        registry = FrameRegistry.from_config({"instruments": {"gocator": {"translation": [0.0, 0.0, 0.0]}}})
        mounting = SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z")
        scan = make_scan(
            mounting,
            y_mm=np.array([-175.0, -87.5, 0.0, 87.5, 175.0]),
            gantry_axis="X",
            gantry_start_mm=1449.99,
            gantry_end_mm=1100.0,
        )
        # 5 rows x 2 cols (x_mm has 2 entries) -> each row appears twice;
        # take one column's worth to check the per-row progression.
        pts = orient_scan(scan, frames=registry).to_points()
        row_values = pts[::2, 0]
        expected = [1449.99, 1362.49, 1274.99, 1187.49, 1099.99]
        np.testing.assert_allclose(row_values, expected, atol=1e-2)  # float32 (default dtype)
        # Monotonically decreasing (matching the real negative-direction
        # travel), not increasing — that's what "not mirrored" means here.
        assert np.all(np.diff(row_values) < 0)

    def test_positive_direction_pass_is_anchored_to_the_real_start(self):
        """Same fix, opposite direction — also confirms the old flat-offset
        formula's ~half-pass-length systematic offset (from not anchoring
        to the first-acquired point) is gone: row 0 must land exactly on
        gantry_start_mm, not gantry_start_mm - length/2.
        """
        registry = FrameRegistry.from_config({"instruments": {"gocator": {"translation": [0.0, 0.0, 0.0]}}})
        mounting = SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z")
        scan = make_scan(
            mounting,
            y_mm=np.array([-175.0, -87.5, 0.0, 87.5, 175.0]),
            gantry_axis="X",
            gantry_start_mm=1100.0,
            gantry_end_mm=1449.99,
        )
        pts = orient_scan(scan, frames=registry).to_points()
        row_values = pts[::2, 0]
        expected = [1100.0, 1187.5, 1275.0, 1362.5, 1449.99]
        np.testing.assert_allclose(row_values, expected, atol=1e-2)  # float32 (default dtype)
        assert np.all(np.diff(row_values) > 0)

    def test_missing_gantry_end_falls_back_to_positive_with_a_warning(self, caplog):
        """No gantry_end_mm means the travel direction can't be determined
        — falls back to the old assume-positive behaviour rather than
        raising, since older saved scans may not have it, but warns since
        the result could be mirrored if that assumption is wrong."""
        registry = FrameRegistry.from_config({"instruments": {"gocator": {"translation": [0.0, 0.0, 0.0]}}})
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)  # no gantry_end_mm
        with caplog.at_level("WARNING"):
            pts = orient_scan(scan, frames=registry).to_points()
        assert pts[:, 0].min() == pytest.approx(700.0)
        assert any("can't be determined" in r.message for r in caplog.records)

    def test_returns_a_surface_scan_not_a_raw_array(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        oriented = orient_scan(scan, frames=registry)
        assert isinstance(oriented, SurfaceScan)
        assert oriented is not scan  # original left untouched

    def test_original_scan_is_not_mutated(self):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        original_x = scan.x_mm.copy()
        orient_scan(scan, frames=registry)
        np.testing.assert_array_equal(scan.x_mm, original_x)

    def test_result_is_per_cell_with_identity_mounting(self):
        """The transform can rotate, which a uniform grid's compact 1D
        x_mm/y_mm can't represent in general — the result is always
        downgraded to per-cell storage with mounting reset to identity, so
        a second orient_scan() call (or to_points()) on it is a no-op
        transform."""
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        oriented = orient_scan(scan, frames=registry)
        assert oriented.is_uniform is False
        assert oriented.mounting.is_identity
        assert oriented.z_mm.shape == scan.z_mm.shape

    def test_invalid_cells_are_preserved_as_nan(self):
        """save_npz() on the result should still reflect the original grid
        — invalid cells must survive orientation as NaN, not get silently
        dropped the way the old flatten-and-drop return value did."""
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        scan.z_mm[0, 0] = np.nan
        oriented = orient_scan(scan, frames=registry)
        assert oriented.z_mm.shape == scan.z_mm.shape
        assert np.isnan(oriented.z_mm[0, 0])
        assert oriented.valid_count == scan.valid_count

    def test_output_writes_a_file_inferred_from_suffix(self, tmp_path):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        out = tmp_path / "oriented.csv"
        orient_scan(scan, frames=registry, output=out)
        assert out.exists()
        assert "x_mm,y_mm,z_mm" in out.read_text()

    def test_output_unknown_suffix_raises(self, tmp_path):
        registry = FrameRegistry.from_config(RIG)
        scan = make_scan(gantry_axis="X", gantry_start_mm=700.0)
        with pytest.raises(ValueError, match="unknown output format"):
            orient_scan(scan, frames=registry, output=tmp_path / "oriented.txt")
