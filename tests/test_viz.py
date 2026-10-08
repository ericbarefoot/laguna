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
from laguna.frames import AffineTransform, FrameRegistry, InstrumentFrame  # noqa: E402
from laguna.scanner.mounting import SensorMounting  # noqa: E402
from laguna.survey import Pass, Tile, Traverse  # noqa: E402
from laguna.viz import Landmark, plot_acquisition, plot_survey_plan, plot_trajectory, plot_trigger_delay  # noqa: E402


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


# plot_survey_plan()
# ---------------------------------------------------------------------------


def make_plan_frames():
    return FrameRegistry(
        experiment_from_gantry=AffineTransform.from_config(
            {"translation": [2200, 670, 0], "rotation_deg": 180}),
        instruments={"gocator": InstrumentFrame(
            "gocator", AffineTransform.from_translation([597.0, 312.0, 0.0]))},
    )


def make_plan_tile():
    swath, span = 1500.0, 2100.0
    return Tile(origin=[700.0, -1052.0, 100.0], length_mm=600.0, width_mm=span, swath_mm=swath,
                overlap=2 - span / swath - 1e-9, axis="X", scan_speed=100.0, travel_speed=100.0,
                accel_mm_s2=1500.0)


class FakePlanScanner:
    mounting = SensorMounting(scan_x="+Y", scan_y="-X", scan_z="+Z")

    def get_active_area(self):
        return {"x_mm": -765.0, "width_mm": 1500.0}


class FakePlanLab:
    def __init__(self):
        import types

        self.frames = make_plan_frames()
        self.gocator = FakePlanScanner()
        self.config = types.SimpleNamespace(get=lambda key: {"axes": [
            {"name": "X", "soft_negative_limit_mm": -5, "soft_positive_limit_mm": 2130},
            {"name": "Y", "soft_negative_limit_mm": -5, "soft_positive_limit_mm": 1200},
        ]})


class TestPlotSurveyPlan:
    def test_planned_swaths_and_origins_without_a_lab(self):
        fig, ax = plot_survey_plan(make_plan_tile(), make_plan_frames())
        assert "2 passes" in ax.get_title()
        # two swath bands, no soft limits or footprints without a lab
        assert len(ax.patches) == 2

    def test_gantry_origin_is_mapped_through_the_experiment_frame(self):
        fig, ax = plot_survey_plan(make_plan_tile(), make_plan_frames())
        marks = [ln.get_xydata()[0] for ln in ax.lines if ln.get_marker() == "x"]
        np.testing.assert_allclose(marks[0], [2200.0, 670.0])  # 180 deg frame: gantry 0,0 is the far corner

    def test_with_a_lab_adds_limits_footprints_and_carriage_paths(self):
        fig, ax = plot_survey_plan(make_plan_tile(), lab=FakePlanLab())
        # 2 swath bands + soft-limit envelope + 2 actual footprints
        assert len(ax.patches) == 5
        assert sum(ln.get_linestyle() == "--" for ln in ax.lines) == 2  # one carriage path per pass

    def test_explicit_limits_without_a_lab(self):
        fig, ax = plot_survey_plan(make_plan_tile(), make_plan_frames(),
                                   gantry_limits={"X": (-5, 2130), "Y": (-5, 1200)})
        assert len(ax.patches) == 3

    def test_lead_in_and_lead_out_are_drawn_as_dotted_segments(self):
        fig, ax = plot_survey_plan(make_plan_tile(), make_plan_frames())
        dotted = [ln for ln in ax.lines if ln.get_linestyle() == ":" and len(ln.get_xdata()) == 2]
        assert len(dotted) == 4                      # one lead-in and one lead-out per pass

    def test_no_accel_means_no_dotted_segments(self):
        tile = make_plan_tile(); tile.accel_mm_s2 = None
        fig, ax = plot_survey_plan(tile, make_plan_frames())
        assert not [ln for ln in ax.lines if ln.get_linestyle() == ":" and len(ln.get_xdata()) == 2]

    def test_draws_into_a_given_axes(self):
        import matplotlib.pyplot as plt

        _, existing = plt.subplots()
        fig, ax = plot_survey_plan(make_plan_tile(), make_plan_frames(), ax=existing, title="mine")
        assert ax is existing and ax.get_title() == "mine"

    def test_traverse_without_a_swath_draws_only_paths(self):
        fig, ax = plot_survey_plan(make_survey(), make_plan_frames())
        assert len(ax.patches) == 0

    def test_empty_passes_raises(self):
        with pytest.raises(ValueError, match="no passes"):
            plot_survey_plan([], make_plan_frames())

    def test_needs_frames_or_a_lab(self):
        with pytest.raises(ValueError, match="frames"):
            plot_survey_plan(make_plan_tile())


# plot_trigger_delay()
# ---------------------------------------------------------------------------


class TestPlotTriggerDelay:
    def test_repeats_at_one_speed(self):
        fig, (ax, bx) = plot_trigger_delay([100.0] * 5, [-12.0, -13.0, -14.0, -13.0, -13.0])
        assert len(ax.lines) >= 3 and "n=5" in " ".join(t.get_text() for t in ax.get_legend().get_texts())
        assert "mean 65.0 ms" in " ".join(t.get_text() for t in bx.texts)

    def test_several_speeds_adds_the_fit_line(self):
        v = [50.0, 50.0, 100.0, 100.0, 150.0, 150.0]
        fig, (ax, bx) = plot_trigger_delay(v, [-2 * s * 0.06 for s in v])
        assert any("fit:" in t.get_text() for t in ax.get_legend().get_texts())

    def test_a_single_measurement_still_plots(self):
        fig, _ = plot_trigger_delay([100.0], [-13.0])
        assert fig.get_axes()

    def test_nan_offsets_are_dropped(self):
        fig, (ax, _) = plot_trigger_delay([100.0, 100.0], [-13.0, float("nan")])
        assert "n=1" in " ".join(t.get_text() for t in ax.get_legend().get_texts())

    @pytest.mark.parametrize("v, d", [([], []), ([1.0, 2.0], [1.0])])
    def test_bad_input_is_rejected(self, v, d):
        with pytest.raises(ValueError):
            plot_trigger_delay(v, d)
