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

    def test_surface_panel_downsamples_a_flat_merged_scan_proportionally(self):
        """Tile.stitch()'s merged output is a flat (1, N) grid — a
        row/col-split stride locks row_stride at 1 (only one row) and caps
        the whole panel at ~1200 plotted points regardless of how large N
        actually is. A single flat stride over all N cells must scale with
        N instead."""
        n = 50_000
        rng = np.random.default_rng(0)
        scan = SurfaceScan(
            z_mm=rng.uniform(0, 1, size=(1, n)),
            x_mm=rng.uniform(0, 1000, size=(1, n)),
            y_mm=rng.uniform(0, 1000, size=(1, n)),
            is_uniform=False,
        )
        fig, axes = plot_acquisition(scan)
        data_ax = axes[1, 1]
        plotted = data_ax.collections[0].get_offsets().shape[0]
        # old row/col-split logic would cap this near ~1200 no matter n;
        # the fixed single-stride logic scales with n (n // stride).
        assert plotted > 5000

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


class TestDownsamplingKnobs:
    def _big_flat_scan(self, n=20_000):
        rng = np.random.default_rng(0)
        return SurfaceScan(
            z_mm=rng.uniform(0, 1, size=(1, n)),
            x_mm=rng.uniform(0, 1000, size=(1, n)),
            y_mm=rng.uniform(0, 1000, size=(1, n)),
            is_uniform=False,
        )

    def test_max_points_caps_the_footprint_panels(self):
        scan = self._big_flat_scan()
        fig, axes = plot_acquisition(scan, max_points=500)
        plotted = axes[0, 0].collections[0].get_offsets().shape[0]
        assert plotted <= 500

    def test_max_points_caps_the_heatmap_panel(self):
        scan = self._big_flat_scan()
        fig, axes = plot_acquisition(scan, max_points=500)
        plotted = axes[1, 1].collections[0].get_offsets().shape[0]
        assert plotted <= 500

    def test_sample_fraction_scales_with_dataset_size(self):
        scan = self._big_flat_scan(n=20_000)
        fig, axes = plot_acquisition(scan, sample_fraction=0.1)
        plotted = axes[0, 0].collections[0].get_offsets().shape[0]
        assert 1500 <= plotted <= 2500  # ~10% of 20,000, stride-rounded

    def test_sample_fraction_takes_precedence_over_max_points(self):
        scan = self._big_flat_scan(n=20_000)
        fig, axes = plot_acquisition(scan, max_points=1, sample_fraction=0.1)
        plotted = axes[0, 0].collections[0].get_offsets().shape[0]
        assert plotted > 1000  # sample_fraction's ~2000, not max_points' 1

    def test_sample_fraction_out_of_range_rejected(self):
        with pytest.raises(ValueError, match="sample_fraction"):
            plot_acquisition(self._big_flat_scan(), sample_fraction=1.5)

    def test_sample_fraction_zero_rejected(self):
        with pytest.raises(ValueError, match="sample_fraction"):
            plot_acquisition(self._big_flat_scan(), sample_fraction=0.0)

    def test_non_positive_max_points_rejected(self):
        with pytest.raises(ValueError, match="max_points"):
            plot_acquisition(self._big_flat_scan(), max_points=0)

    def test_downsampling_applies_to_uniform_heatmap_too(self):
        """The imshow (uniform-grid) path uses a scaled row/col split, not
        the flat stride — check it actually shrinks, not just the scatter
        path exercised above."""
        z = np.random.default_rng(0).uniform(0, 1, size=(2000, 3000))
        scan = SurfaceScan(
            z_mm=z, x_mm=np.linspace(0, 1000, 3000), y_mm=np.linspace(0, 1000, 2000),
            is_uniform=True,
        )
        fig, axes = plot_acquisition(scan, max_points=1000)
        im = axes[1, 1].images[0]
        assert im.get_array().size <= 2000  # generous slack for rounding

    def test_downsampling_applies_to_profile_panel(self):
        big_df = pd.DataFrame({
            "pos_mm": np.arange(10_000, dtype=float),
            "height_mm": np.sin(np.arange(10_000)),
            "experiment_x_mm": np.arange(10_000, dtype=float),
            "experiment_y_mm": np.zeros(10_000),
            "experiment_z_mm": np.zeros(10_000),
        })
        fig, axes = plot_acquisition(big_df, sample_fraction=0.01)
        line = axes[1, 1].lines[0]
        assert len(line.get_xdata()) <= 200

    def test_cmap_is_applied_to_heatmap(self):
        fig, axes = plot_acquisition(make_uniform_scan(), cmap="plasma")
        assert axes[1, 1].images[0].get_cmap().name == "plasma"

    def test_figsize_is_applied_to_a_new_figure(self):
        fig, axes = plot_acquisition(make_uniform_scan(), figsize=(4.0, 3.0))
        assert fig.get_size_inches() == pytest.approx([4.0, 3.0])

    def test_figsize_ignored_when_fig_given(self):
        import matplotlib.pyplot as plt

        existing = plt.figure(figsize=(7.0, 5.0))
        fig, axes = plot_acquisition(make_uniform_scan(), fig=existing, figsize=(1.0, 1.0))
        assert fig.get_size_inches() == pytest.approx([7.0, 5.0])


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
