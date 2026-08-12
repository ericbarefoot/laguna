"""Tests for laguna.viz.

matplotlib is an optional dependency (pip install 'laguna[viz]') — every
test here needs it, so the whole module is skipped, not individual tests,
when it isn't installed.
"""

import numpy as np
import pandas as pd
import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from laguna.robot.macron.profiler import ProfileResult  # noqa: E402
from laguna.scanner.pointcloud import SurfaceScan  # noqa: E402
from laguna.survey import Pass, Traverse  # noqa: E402
from laguna.viz import Landmark, plot_acquisition, plot_trajectory  # noqa: E402


def make_uniform_scan(**meta):
    z = np.array([[1.0, 2.0], [np.nan, 4.0]])
    return SurfaceScan(
        z_mm=z, x_mm=np.array([0.0, 10.0]), y_mm=np.array([0.0, 20.0]),
        metadata=meta, is_uniform=True,
    )


def make_profile_df():
    return pd.DataFrame({
        "pos_mm": [100.0, 150.0, 200.0],
        "distance_mm": [5.0, 6.0, 7.0],
        "height_mm": [5.0, 6.0, 7.0],
        "experiment_x_mm": [110.0, 160.0, 210.0],
        "experiment_y_mm": [50.0, 50.0, 50.0],
        "experiment_z_mm": [-10.0, -9.0, -8.0],
    })


class TestPlotAcquisitionDispatch:
    def test_plots_a_surface_scan(self):
        fig, axes = plot_acquisition(make_uniform_scan())
        assert fig.get_axes()
        assert axes.shape == (2, 2)

    def test_plots_a_profile_result(self):
        result = ProfileResult(path="fake.csv", metadata={}, df=make_profile_df())
        fig, axes = plot_acquisition(result)
        assert fig.get_axes()

    def test_plots_a_bare_dataframe(self):
        fig, axes = plot_acquisition(make_profile_df())
        assert fig.get_axes()

    def test_unrecognized_type_raises(self):
        with pytest.raises(TypeError, match="doesn't know how to handle"):
            plot_acquisition(object())

    def test_dataframe_missing_experiment_columns_raises(self):
        df = pd.DataFrame({"pos_mm": [1.0, 2.0]})
        with pytest.raises(ValueError, match="experiment_x_mm"):
            plot_acquisition(df)

    def test_profile_result_with_no_dataframe_raises(self):
        result = ProfileResult(path="fake.csv", metadata={}, df=None)
        with pytest.raises(ValueError, match="no DataFrame"):
            plot_acquisition(result)

    def test_un_oriented_scan_warns_but_still_plots(self, caplog):
        from laguna.scanner.mounting import SensorMounting

        scan = SurfaceScan(
            z_mm=np.array([[1.0, 2.0]]), x_mm=np.array([0.0, 10.0]), y_mm=np.array([0.0]),
            is_uniform=True, mounting=SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z"),
        )
        with caplog.at_level("WARNING"):
            fig, axes = plot_acquisition(scan)
        assert fig.get_axes()
        assert any("not experiment-frame" in r.message for r in caplog.records)


class TestFigureReuse:
    def test_creates_a_figure_when_none_given(self):
        fig, axes = plot_acquisition(make_uniform_scan())
        assert fig is not None

    def test_plots_into_a_given_figure(self):
        import matplotlib.pyplot as plt

        fig = plt.figure()
        returned_fig, axes = plot_acquisition(make_uniform_scan(), fig=fig)
        assert returned_fig is fig

    def test_returned_axes_can_be_adjusted_afterward(self):
        """The whole point of returning axes — the caller can tweak limits,
        titles, etc. without having to reach into fig.get_axes()."""
        fig, axes = plot_acquisition(make_uniform_scan())
        ax_xy = axes[0, 0]
        ax_xy.set_xlim(-5, 5)
        assert ax_xy.get_xlim() == (-5, 5)

    def test_title_defaults_to_a_summary(self):
        fig, axes = plot_acquisition(make_uniform_scan())
        assert fig._suptitle is not None
        assert "SurfaceScan" in fig._suptitle.get_text()

    def test_explicit_title_is_used(self):
        fig, axes = plot_acquisition(make_uniform_scan(), title="my custom title")
        assert fig._suptitle.get_text() == "my custom title"


class TestLandmarkOverlay:
    """Landmarks are bare experiment-coordinate boxes — no frame transform
    involved (unlike the gantry-frame fence geometry this replaced; see
    laguna.viz's module docstring for why)."""

    def test_landmark_drawn_on_every_plane(self):
        landmarks = [Landmark("wall", 0, 100, 0, 100, 0, 100)]
        fig, axes = plot_acquisition(make_uniform_scan(), landmarks=landmarks)
        for ax in fig.get_axes()[:3]:
            assert len(ax.patches) == 1

    def test_no_landmarks_means_no_patches(self):
        fig, axes = plot_acquisition(make_uniform_scan())
        for ax in fig.get_axes()[:3]:
            assert len(ax.patches) == 0

    def test_multiple_landmarks_all_drawn(self):
        landmarks = [
            Landmark("a", 0, 10, 0, 10, 0, 10),
            Landmark("b", 20, 30, 20, 30, 20, 30),
        ]
        fig, axes = plot_acquisition(make_uniform_scan(), landmarks=landmarks)
        for ax in fig.get_axes()[:3]:
            assert len(ax.patches) == 2

    def test_landmark_bounds_used_directly_no_transform(self):
        landmarks = [Landmark("wall", 500.0, 600.0, 300.0, 400.0, 0, 100)]
        fig, axes = plot_acquisition(make_uniform_scan(), landmarks=landmarks)
        patch = axes[0, 0].patches[0]  # xy plane
        assert patch.get_x() == pytest.approx(500.0)
        assert patch.get_y() == pytest.approx(300.0)
        assert patch.get_width() == pytest.approx(100.0)
        assert patch.get_height() == pytest.approx(100.0)


class TestDataPanel:
    def test_surface_panel_is_a_heatmap(self):
        fig, axes = plot_acquisition(make_uniform_scan())
        data_ax = axes[1, 1]
        assert len(data_ax.images) == 1  # imshow, uniform grid
        assert data_ax.get_xlabel() == "X (mm)"
        assert data_ax.get_ylabel() == "Y (mm)"

    def test_surface_panel_uses_scatter_for_point_cloud(self):
        scan = SurfaceScan(
            z_mm=np.array([[1.0, 2.0], [3.0, 4.0]]),
            x_mm=np.array([[0.0, 10.0], [1.0, 11.0]]),
            y_mm=np.array([[0.0, 1.0], [20.0, 21.0]]),
            is_uniform=False,
        )
        fig, axes = plot_acquisition(scan)
        data_ax = axes[1, 1]
        assert len(data_ax.images) == 0
        assert len(data_ax.collections) >= 1  # scatter

    def test_surface_panel_handles_all_invalid(self):
        scan = SurfaceScan(
            z_mm=np.full((2, 2), np.nan), x_mm=np.array([0.0, 1.0]), y_mm=np.array([0.0, 1.0]),
            is_uniform=True,
        )
        fig, axes = plot_acquisition(scan)  # must not raise
        assert fig.get_axes()

    def test_profile_panel_uses_height_when_present(self):
        fig, axes = plot_acquisition(make_profile_df())
        data_ax = axes[1, 1]
        assert data_ax.get_ylabel() == "calibrated height (mm)"

    def test_profile_panel_falls_back_without_height(self):
        df = make_profile_df().drop(columns=["height_mm"])
        fig, axes = plot_acquisition(df)
        data_ax = axes[1, 1]
        assert "no calibration" in data_ax.get_ylabel()


# ---------------------------------------------------------------------------
# plot_trajectory()
# ---------------------------------------------------------------------------


def make_survey():
    return Traverse(
        start=(251.0, 136.0, 0.0), end=(601.0, 136.0, 0.0),
        instruments=("od2000", "wtt12l"), speed=30.0,
    )


class TestPlotTrajectory:
    def test_plots_a_survey(self):
        fig, axes = plot_trajectory(make_survey())
        assert fig.get_axes()
        assert axes.shape == (2, 2)

    def test_plots_a_bare_list_of_passes(self):
        passes = [
            Pass(index=0, start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), instrument="od2000"),
        ]
        fig, axes = plot_trajectory(passes)
        assert fig.get_axes()

    def test_empty_passes_raises(self):
        with pytest.raises(ValueError, match="no passes"):
            plot_trajectory([])

    def test_one_line_per_pass(self):
        fig, axes = plot_trajectory(make_survey())
        ax_xy = axes[0, 0]
        assert len(ax_xy.lines) == 2  # two instruments -> two passes

    def test_title_defaults_to_pass_count(self):
        fig, axes = plot_trajectory(make_survey())
        assert "2 passes" in fig._suptitle.get_text()

    def test_explicit_title_is_used(self):
        fig, axes = plot_trajectory(make_survey(), title="my plan")
        assert fig._suptitle.get_text() == "my plan"

    def test_landmarks_drawn_on_every_plane(self):
        landmarks = [Landmark("wall", 0, 1000, 0, 1000, 0, 1000)]
        fig, axes = plot_trajectory(make_survey(), landmarks=landmarks)
        for ax in fig.get_axes()[:3]:
            assert len(ax.patches) == 1

    def test_data_panel_lists_every_pass(self):
        fig, axes = plot_trajectory(make_survey())
        data_ax = axes[1, 1]
        # ax.table() stores cells keyed by (row, col); 2 data rows + 1 header
        table = next(iter(data_ax.tables)) if hasattr(data_ax, "tables") else None
        assert table is not None
        rows = {r for (r, c) in table.get_celld()}
        assert len(rows) == 3  # header + 2 passes
