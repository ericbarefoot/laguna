"""Tests for the sensor->gantry axis mapping.

The Gocator names its axes from its own optics: X across the laser line, Y
along travel. On this rig it is mounted rotated 90 degrees, so a gantry move
along X arrives as the sensor's Y — the confusion these tests pin down.
"""

import numpy as np
import pytest

from laguna.scanner.mounting import SensorMounting
from laguna.scanner.pointcloud import SurfaceScan

#: The rig's actual mounting (sensor rotated 90 degrees about Z).
RIG = {"scan_x": "-Y", "scan_y": "+X", "scan_z": "+Z"}


class TestSensorMounting:
    def test_default_is_identity(self):
        m = SensorMounting()
        assert m.is_identity
        assert m.grid_axes() == ("Y", "X")

    def test_rig_mounting_maps_travel_to_gantry_x(self):
        m = SensorMounting(**RIG)
        assert m.gantry_axis_of("scan_y") == "+X"   # travel
        assert m.gantry_axis_of("scan_x") == "-Y"   # across the laser
        # rows run along travel, so with this mounting rows step along gantry X
        assert m.grid_axes() == ("X", "Y")

    def test_rig_mounting_is_a_proper_rotation(self):
        assert np.linalg.det(SensorMounting(**RIG).matrix) == pytest.approx(1.0)

    def test_bare_swap_rejected_as_mirroring(self):
        """A swap with no sign flip has determinant -1: it reflects the data
        rather than rotating it, so real geometry comes back mirrored."""
        with pytest.raises(ValueError, match="mirrors the data"):
            SensorMounting(scan_x="+Y", scan_y="+X", scan_z="+Z")

    def test_duplicate_target_axis_rejected(self):
        with pytest.raises(ValueError, match="same gantry axis"):
            SensorMounting(scan_x="+Y", scan_y="+Y", scan_z="+Z")

    def test_unsigned_and_lowercase_accepted(self):
        m = SensorMounting(scan_x="y", scan_y="-x", scan_z="Z")
        assert m.gantry_axis_of("scan_x") == "+Y"
        assert m.gantry_axis_of("scan_y") == "-X"

    @pytest.mark.parametrize("bad", ["+W", "", "XY", 5, None])
    def test_malformed_axis_rejected(self, bad):
        with pytest.raises(ValueError):
            SensorMounting(scan_x=bad)

    def test_from_config_none_is_identity(self):
        assert SensorMounting.from_config(None).is_identity
        assert SensorMounting.from_config({}).is_identity

    def test_from_config_unknown_key_rejected(self):
        with pytest.raises(ValueError, match="unknown mounting key"):
            SensorMounting.from_config({"scan_q": "+X"})

    def test_to_dict_round_trips(self):
        m = SensorMounting(**RIG)
        assert SensorMounting.from_config(m.to_dict()).matrix.tolist() == m.matrix.tolist()

    def test_apply_to_points_permutes_and_signs(self):
        m = SensorMounting(**RIG)
        # sensor (x=1, y=2, z=3) -> gantry (X=+sy=2, Y=-sx=-1, Z=+sz=3)
        out = m.apply_to_points(np.array([[1.0, 2.0, 3.0]]))
        np.testing.assert_allclose(out, [[2.0, -1.0, 3.0]])

    def test_transform_is_rigid(self):
        """A proper rotation preserves distances — the check that catches a
        mapping that silently distorts or reflects geometry."""
        m = SensorMounting(**RIG)
        rng = np.random.default_rng(0)
        pts = rng.normal(size=(100, 3))
        out = m.apply_to_points(pts)
        d_in = np.linalg.norm(pts[:50] - pts[50:], axis=1)
        d_out = np.linalg.norm(out[:50] - out[50:], axis=1)
        np.testing.assert_allclose(d_in, d_out)


def make_scan(mounting=None):
    """2x2 uniform surface; z=NaN at (1,0) so one point is dropped."""
    return SurfaceScan(
        z_mm=np.array([[1.0, 2.0], [np.nan, 4.0]]),
        x_mm=np.array([0.0, 10.0]),
        y_mm=np.array([0.0, 20.0]),
        is_uniform=True,
        mounting=mounting or SensorMounting(),
    )


class TestSurfaceScanFrames:
    def test_identity_mounting_leaves_points_unchanged(self):
        scan = make_scan()
        np.testing.assert_array_equal(
            scan.to_points(), scan.to_points(frame="sensor")
        )

    def test_to_points_defaults_to_gantry_frame(self):
        scan = make_scan(SensorMounting(**RIG))
        gantry = scan.to_points()
        sensor = scan.to_points(frame="sensor")
        # gantry X is sensor Y; gantry Y is -sensor X
        np.testing.assert_allclose(gantry[:, 0], sensor[:, 1])
        np.testing.assert_allclose(gantry[:, 1], -sensor[:, 0])
        np.testing.assert_allclose(gantry[:, 2], sensor[:, 2])

    def test_travel_span_lands_on_the_commanded_gantry_axis(self):
        """The whole point: a gantry X move must show up as gantry X extent,
        not gantry Y. Sensor Y (travel) spans 20mm, sensor X spans 10mm."""
        scan = make_scan(SensorMounting(**RIG))
        pts = scan.to_points()
        assert pts[:, 0].max() - pts[:, 0].min() == pytest.approx(20.0)  # gantry X
        assert pts[:, 1].max() - pts[:, 1].min() == pytest.approx(10.0)  # gantry Y

    def test_grid_axes_reports_gantry_axes(self):
        assert make_scan().grid_axes == ("Y", "X")
        assert make_scan(SensorMounting(**RIG)).grid_axes == ("X", "Y")

    def test_frame_applies_to_the_keep_invalid_path_too(self):
        scan = make_scan(SensorMounting(**RIG))
        gantry = scan.to_points(drop_invalid=False)
        sensor = scan.to_points(drop_invalid=False, frame="sensor")
        assert len(gantry) == 4
        np.testing.assert_allclose(gantry[:, 0], sensor[:, 1])

    def test_unknown_frame_rejected(self):
        with pytest.raises(ValueError, match="frame must be"):
            make_scan().to_points(frame="world")

    def test_non_uniform_point_cloud_is_transformed_too(self):
        scan = SurfaceScan(
            z_mm=np.array([[1.0, 2.0]]),
            x_mm=np.array([[3.0, 4.0]]),
            y_mm=np.array([[5.0, 6.0]]),
            is_uniform=False,
            mounting=SensorMounting(**RIG),
        )
        pts = scan.to_points()
        np.testing.assert_allclose(pts[0], [5.0, -3.0, 1.0])

    def test_npz_records_the_mounting_and_grid_axes(self, tmp_path):
        """A future reader must be able to tell which frame a saved file is
        in without guessing."""
        import ast

        path = make_scan(SensorMounting(**RIG)).save_npz(tmp_path / "s.npz")
        meta = ast.literal_eval(str(np.load(path, allow_pickle=True)["metadata"].item()))
        assert meta["mounting"] == RIG
        assert meta["grid_axes"] == ["X", "Y"]

    def test_rescale_y_preserves_mounting(self):
        scan = make_scan(SensorMounting(**RIG))
        scan.metadata["travel_speed_mm_s"] = 10.0
        assert scan.rescale_y(20.0).mounting.to_dict() == RIG
