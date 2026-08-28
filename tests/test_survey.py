"""Tests for multi-pass survey planning.

The geometry is where the mistakes hide: an off-by-one in the pass count
leaves an unimaged gap down the middle of a bed, and nobody notices until
the data is being stitched.
"""

import numpy as np
import pytest

from laguna.robot.macron.commands import Axis, ramp_distance_mm, ramp_time_s
from laguna.scanner.mounting import SensorMounting
from laguna.survey import Pass, Tile, Traverse, SurveyRunner


class TestRampKinematics:
    """Tests for ramp_distance_mm / ramp_time_s.

    The accel-ramp math the tile-scan lead-in (issue #58) is built on.
    """

    @pytest.mark.parametrize("feed_rate,accel,expected_distance,expected_time", [
        (20.0, 100.0, 2.0, 0.2),      # v^2/2a = 400/200, v/a = 0.2
        (100.0, 50.0, 100.0, 2.0),    # v^2/2a = 10000/100, v/a = 2.0
        (10.0, 10.0, 5.0, 1.0),
    ])
    def test_matches_kinematics_formula(self, feed_rate, accel, expected_distance, expected_time):
        assert ramp_distance_mm(feed_rate, accel) == pytest.approx(expected_distance)
        assert ramp_time_s(feed_rate, accel) == pytest.approx(expected_time)

    @pytest.mark.parametrize("feed_rate,accel", [
        (0.0, 100.0), (-5.0, 100.0), (None, 100.0),
        (20.0, 0.0), (20.0, -1.0), (20.0, None),
    ])
    def test_non_positive_or_unknown_input_gives_zero(self, feed_rate, accel):
        assert ramp_distance_mm(feed_rate, accel) == 0.0
        assert ramp_time_s(feed_rate, accel) == 0.0


class TestTileGeometry:
    def _tile(self, **kw):
        base = dict(origin=(0.0, 0.0, 0.0), length_mm=1000.0, width_mm=2000.0,
                    swath_mm=1000.0, overlap=0.0)
        base.update(kw)
        return Tile(**base)

    def test_exact_fit_needs_no_extra_pass(self):
        assert len(self._tile(width_mm=2000.0, swath_mm=1000.0)) == 2

    def test_a_partial_swath_still_gets_a_pass(self):
        """A region 2.5 swaths wide needs 3 passes. Rounding down would leave
        an unimaged strip, which is far worse than an extra pass."""
        assert len(self._tile(width_mm=2500.0, swath_mm=1000.0)) == 3

    def test_narrower_than_one_swath_is_a_single_pass(self):
        assert len(self._tile(width_mm=200.0, swath_mm=1000.0)) == 1

    def test_width_exactly_one_swath_with_overlap_is_not_duplicated(self):
        """The overcounting bug: ceil(width / pitch) with overlap > 0 and
        width == swath gave 2 — both offsets clamp to 0
        (max(0, width - swath) == 0), so the 'extra' pass was an exact
        duplicate of the first, not new coverage. Coverage-only checks
        (coverage_mm() >= width) can't catch this — a duplicate pass still
        "covers" the region, it just wastes a whole redundant traverse.
        """
        survey = self._tile(width_mm=1000.0, swath_mm=1000.0, overlap=0.1)
        passes = survey.passes()
        assert len(passes) == 1
        offsets = {p.start[1] for p in passes}  # step axis defaults to Y
        assert len(offsets) == len(passes), "no two passes should share an offset"

    def test_no_pass_duplicates_another_pass_offset(self):
        """General form of the duplicate-pass bug: across a range of
        width/swath/overlap combinations, no two passes should land at the
        same step-axis offset — each pass must contribute new coverage."""
        for width, swath, overlap in [
            (1000.0, 1000.0, 0.1), (900.0, 1000.0, 0.3), (1000.0, 1000.0, 0.0),
            (1500.0, 1000.0, 0.4), (2000.0, 1000.0, 0.1),
        ]:
            survey = self._tile(width_mm=width, swath_mm=swath, overlap=overlap)
            passes = survey.passes()
            offsets = [p.start[1] for p in passes]
            assert len(set(offsets)) == len(offsets), (
                f"duplicate pass offset with width={width} swath={swath} overlap={overlap}: "
                f"{offsets}"
            )

    def test_overlap_increases_the_pass_count(self):
        wide = self._tile(width_mm=2000.0, swath_mm=1000.0, overlap=0.0)
        lapped = self._tile(width_mm=2000.0, swath_mm=1000.0, overlap=0.5)
        assert len(lapped) > len(wide)

    def test_pitch_accounts_for_overlap(self):
        assert self._tile(swath_mm=1000.0, overlap=0.2).pitch_mm == pytest.approx(800.0)

    def test_coverage_is_never_less_than_the_region(self):
        """The property that actually matters: no gaps."""
        for width, swath, overlap in [
            (2000.0, 1000.0, 0.0), (2500.0, 1000.0, 0.1),
            (1234.0, 500.0, 0.25), (100.0, 900.0, 0.1),
        ]:
            survey = self._tile(width_mm=width, swath_mm=swath, overlap=overlap)
            assert survey.coverage_mm() >= width - 1e-9, (
                f"gap left with width={width} swath={swath} overlap={overlap}"
            )

    def test_passes_step_along_the_step_axis(self):
        survey = self._tile(axis="X", width_mm=2000.0, swath_mm=1000.0)
        offsets = [p.start[1] for p in survey]     # Y is the step axis
        assert offsets == sorted(offsets)
        assert offsets[0] == 0.0

    def test_traverse_runs_along_the_travel_axis(self):
        p = self._tile(axis="X", length_mm=750.0).passes()[0]
        assert p.end[0] - p.start[0] == pytest.approx(750.0)
        assert p.end[1] == p.start[1]

    def test_serpentine_alternates_direction(self):
        """Halves repositioning travel — the gantry doesn't drive back to the
        same side after every pass."""
        passes = self._tile(serpentine=True, width_mm=3000.0, swath_mm=1000.0).passes()
        assert passes[0].end[0] > passes[0].start[0]
        assert passes[1].end[0] < passes[1].start[0]
        assert passes[2].end[0] > passes[2].start[0]

    def test_serpentine_can_be_disabled(self):
        passes = self._tile(serpentine=False, width_mm=3000.0, swath_mm=1000.0).passes()
        assert all(p.end[0] > p.start[0] for p in passes)

    def test_no_accel_leaves_start_and_cruise_start_identical(self):
        """Default (no ramp lead-in): backward-compatible with pre-#58
        behavior — start IS the swath boundary, cruise_start is unset."""
        passes = self._tile(width_mm=3000.0, swath_mm=1000.0, scan_speed=20.0).passes()
        for p in passes:
            assert p.cruise_start is None
            assert p.measure_start == p.start

    def test_accel_shifts_start_behind_the_true_boundary(self):
        """With accel_mm_s2 set, Pass.start (the commanded/ramp start) moves
        back along the travel axis by the accel-ramp distance, while
        cruise_start keeps the true swath boundary the plan asked for —
        this is the fix for issue #58's travel-direction seam."""
        survey = self._tile(
            axis="X", width_mm=3000.0, swath_mm=1000.0,
            scan_speed=20.0, accel_mm_s2=100.0,
        )
        expected_ramp = ramp_distance_mm(20.0, 100.0)
        assert expected_ramp == pytest.approx(2.0)  # v^2/2a = 400/200
        passes = survey.passes()
        for p in passes:
            assert p.cruise_start is not None
            # length_mm/measure_start reflect the true swath, unaffected by
            # the ramp lead-in.
            assert p.measure_start == p.cruise_start
            assert p.length_mm == pytest.approx(survey.length_mm)

    def test_ramp_lead_in_lands_on_the_correct_side_per_serpentine_leg(self):
        """Forward passes ramp in from behind (smaller travel coordinate);
        reversed serpentine passes ramp in from the far side (larger
        coordinate) — either way the axis is already at scan_speed when it
        crosses cruise_start, from whichever direction it's travelling."""
        survey = self._tile(
            axis="X", serpentine=True, width_mm=3000.0, swath_mm=1000.0,
            scan_speed=20.0, accel_mm_s2=100.0,
        )
        ramp = ramp_distance_mm(20.0, 100.0)
        passes = survey.passes()
        # pass 0: forward (start=origin -> end=+length): ramp start is
        # BEHIND cruise_start (smaller travel-axis coordinate).
        assert passes[0].end[0] > passes[0].cruise_start[0]
        assert passes[0].start[0] == pytest.approx(passes[0].cruise_start[0] - ramp)
        # pass 1: reversed (cruise_start=+length -> end=origin): ramp start
        # is further along than cruise_start (larger travel-axis coordinate).
        assert passes[1].end[0] < passes[1].cruise_start[0]
        assert passes[1].start[0] == pytest.approx(passes[1].cruise_start[0] + ramp)

    def test_zero_scan_speed_means_no_lead_in_even_with_accel_set(self):
        """ramp_distance_mm's own guard (feed_rate <= 0 -> 0.0) must reach
        through Tile.passes() as "no lead-in needed", not a crash."""
        passes = self._tile(
            width_mm=1000.0, swath_mm=1000.0, accel_mm_s2=100.0,
        ).passes()  # scan_speed left unset
        assert passes[0].cruise_start is None
        assert passes[0].start == passes[0].measure_start

    def test_indices_are_sequential_from_zero(self):
        assert [p.index for p in self._tile(width_mm=3000.0, swath_mm=1000.0)] == [0, 1, 2]

    @pytest.mark.parametrize("kw,match", [
        ({"swath_mm": 0.0}, "swath_mm must be positive"),
        ({"overlap": 1.0}, "never advance"),
        ({"overlap": -0.1}, "overlap must be"),
        ({"width_mm": 0.0}, "must be positive"),
        ({"axis": "Q"}, "axis must be one of"),
    ])
    def test_impossible_geometry_is_rejected(self, kw, match):
        with pytest.raises(ValueError, match=match):
            self._tile(**kw)

    def test_step_axis_must_differ_from_travel_axis(self):
        with pytest.raises(ValueError, match="must differ"):
            self._tile(axis="X", step_axis="X")

    def test_origin_must_have_three_components(self):
        with pytest.raises(ValueError, match="3 components"):
            self._tile(origin=(0.0, 0.0))

    def test_speed_sets_both_scan_and_travel_speed(self):
        p = self._tile(speed=50.0).passes()[0]
        assert p.scan_speed == 50.0
        assert p.travel_speed == 50.0

    def test_speed_does_not_override_an_explicit_travel_speed(self):
        p = self._tile(speed=50.0, travel_speed=90.0).passes()[0]
        assert p.scan_speed == 50.0
        assert p.travel_speed == 90.0


class TestTraverse:
    def test_one_pass_per_instrument_per_repeat(self):
        survey = Traverse(
            start=(0, 0, 0), end=(100, 0, 0),
            instruments=("od2000", "wtt12l"), repeats=3,
        )
        assert len(survey) == 6

    def test_instrument_order_is_preserved(self):
        survey = Traverse(
            start=(0, 0, 0), end=(100, 0, 0), instruments=("od2000", "wtt12l"),
        )
        assert [p.instrument for p in survey] == ["od2000", "wtt12l"]

    def test_every_pass_covers_identical_ground(self):
        """The point of a multi-instrument transect — the frames layer makes
        the same experiment coordinates valid for differently-mounted
        instruments."""
        survey = Traverse(
            start=(10, 20, 0), end=(110, 20, 0),
            instruments=("od2000", "wtt12l"), repeats=2,
        )
        assert len({(p.start, p.end) for p in survey}) == 1

    def test_travel_speed_is_independent_of_scan_speed(self):
        survey = Traverse(
            start=(0, 0, 0), end=(100, 0, 0),
            scan_speed=20.0, travel_speed=80.0,
        )
        assert survey.passes()[0].scan_speed == 20.0
        assert survey.passes()[0].travel_speed == 80.0

    def test_speed_sets_both_scan_and_travel_speed(self):
        survey = Traverse(start=(0, 0, 0), end=(100, 0, 0), speed=50.0)
        assert survey.passes()[0].scan_speed == 50.0
        assert survey.passes()[0].travel_speed == 50.0

    def test_speed_does_not_override_an_explicit_scan_or_travel_speed(self):
        survey = Traverse(
            start=(0, 0, 0), end=(100, 0, 0),
            speed=50.0, scan_speed=20.0,
        )
        assert survey.passes()[0].scan_speed == 20.0
        assert survey.passes()[0].travel_speed == 50.0

    def test_repeats_must_be_positive(self):
        with pytest.raises(ValueError, match="repeats must be"):
            Traverse(start=(0, 0, 0), end=(1, 0, 0), repeats=0)

    def test_at_least_one_instrument_required(self):
        with pytest.raises(ValueError, match="at least one instrument"):
            Traverse(start=(0, 0, 0), end=(1, 0, 0), instruments=())

    def test_bare_string_instrument_is_one_instrument_not_six_characters(self):
        """A bare instrument key ("wtt12l") is a str, which is itself a
        Sequence[str] — without normalizing it, one instrument produced one
        pass per character instead of a single pass."""
        survey = Traverse(start=(0, 0, 0), end=(100, 0, 0), instruments="wtt12l")
        assert [p.instrument for p in survey] == ["wtt12l"]

    def test_start_must_have_three_components(self):
        with pytest.raises(ValueError, match="3 components"):
            Traverse(start=(0, 0), end=(1, 0, 0))

    def test_end_must_have_three_components(self):
        with pytest.raises(ValueError, match="3 components"):
            Traverse(start=(0, 0, 0), end=(1, 0))


class TestCosting:
    def test_duration_from_length_and_scan_speed(self):
        survey = Tile(
            origin=(0, 0, 0), length_mm=1000.0, width_mm=2000.0,
            swath_mm=1000.0, overlap=0.0, scan_speed=20.0,
        )
        # two passes, 1000mm each, at 20mm/s
        assert survey.duration_s() == pytest.approx(100.0)

    def test_duration_is_none_without_a_scan_speed(self):
        survey = Tile(origin=(0, 0, 0), length_mm=1000.0, width_mm=1000.0,
                              swath_mm=1000.0)
        assert survey.duration_s() is None

    def test_describe_lists_every_pass(self):
        survey = Tile(origin=(0, 0, 0), length_mm=100.0, width_mm=2000.0,
                              swath_mm=1000.0, overlap=0.0)
        text = survey.describe()
        assert "2 passes" in text
        assert text.count("tile") == 2


def _raw_scan(gantry_start, gantry_end_mm=20.0):
    """A tiny raw (un-oriented) uniform surface, as scan_with_gantry() would
    hand SurveyRunner — 2x2 cells, no NaNs, travel along gantry X."""
    from laguna.scanner.pointcloud import SensorMounting, SurfaceScan

    return SurfaceScan(
        z_mm=np.ones((2, 2)),
        x_mm=np.array([0.0, 10.0]),
        y_mm=np.array([0.0, 20.0]),
        metadata={
            "gantry_axis": "X",
            "gantry_start_mm": gantry_start[0],
            "gantry_end_mm": gantry_end_mm,
            "gantry_start": list(gantry_start),
        },
        is_uniform=True,
        mounting=SensorMounting(),
    )


class TestTileStitch:
    def _passes(self):
        return [
            Pass(index=0, start=(0.0, 0.0, 0.0), end=(20.0, 0.0, 0.0),
                 instrument="gocator", axis="X", label="tile 1/2"),
            Pass(index=1, start=(0.0, 500.0, 0.0), end=(20.0, 500.0, 0.0),
                 instrument="gocator", axis="X", label="tile 2/2"),
        ]

    def _frames(self):
        from laguna.frames import FrameRegistry

        return FrameRegistry.from_config({"instruments": {"gocator": {"translation": [0, 0, 0]}}})

    def test_merges_every_pass_into_one_flat_scan(self):
        tile = Tile(origin=(0, 0, 0), length_mm=20.0, width_mm=520.0, swath_mm=500.0)
        passes = self._passes()
        results = [_raw_scan([0.0, 0.0, 0.0]), _raw_scan([0.0, 500.0, 0.0])]
        merged = tile.stitch(passes, results, self._frames())
        assert merged.shape == (1, 8)  # 2 passes x 4 cells each, no NaNs
        assert merged.valid_count == 8

    def test_stitched_points_land_at_each_pass_own_offset(self):
        """The point of stitching: pass 2's points show up 500mm further in
        Y than pass 1's, not on top of them."""
        tile = Tile(origin=(0, 0, 0), length_mm=20.0, width_mm=520.0, swath_mm=500.0)
        passes = self._passes()
        results = [_raw_scan([0.0, 0.0, 0.0]), _raw_scan([0.0, 500.0, 0.0])]
        merged = tile.stitch(passes, results, self._frames())
        y = merged.to_points()[:, 1]
        assert y.min() == pytest.approx(0.0)
        assert y.max() == pytest.approx(520.0)  # 500 offset + 20 sensor-local extent

    def test_metadata_records_the_merge(self):
        tile = Tile(origin=(0, 0, 0), length_mm=20.0, width_mm=520.0, swath_mm=500.0)
        passes = self._passes()
        results = [_raw_scan([0.0, 0.0, 0.0]), _raw_scan([0.0, 500.0, 0.0])]
        merged = tile.stitch(passes, results, self._frames())
        assert merged.metadata["stitched"] is True
        assert merged.metadata["stitch_pass_count"] == 2
        assert merged.metadata["stitch_pass_labels"] == ["tile 1/2", "tile 2/2"]
        assert merged.metadata["stitch_pass_point_counts"] == [4, 4]
        assert merged.metadata["stitch_instruments"] == ["gocator"]

    def test_result_is_exportable_like_any_surface_scan(self, tmp_path):
        tile = Tile(origin=(0, 0, 0), length_mm=20.0, width_mm=520.0, swath_mm=500.0)
        passes = self._passes()
        results = [_raw_scan([0.0, 0.0, 0.0]), _raw_scan([0.0, 500.0, 0.0])]
        merged = tile.stitch(passes, results, self._frames())
        path = merged.save_csv(tmp_path / "stitched.csv")
        assert path.exists()

    def test_mismatched_lengths_rejected(self):
        tile = Tile(origin=(0, 0, 0), length_mm=20.0, width_mm=520.0, swath_mm=500.0)
        with pytest.raises(ValueError, match="one result per pass"):
            tile.stitch(self._passes(), [_raw_scan([0.0, 0.0, 0.0])], self._frames())

    def test_empty_rejected(self):
        tile = Tile(origin=(0, 0, 0), length_mm=20.0, width_mm=520.0, swath_mm=500.0)
        with pytest.raises(ValueError, match="at least one"):
            tile.stitch([], [], self._frames())


class FakeScan:
    """Stand-in for a SurfaceScan — just enough for _run_pass's result_note."""

    valid_count = 42


class FakeScanner:
    def __init__(self):
        self.acquired = []

    def acquire(self, gantry=None, **kw):
        self.acquired.append(kw)
        return FakeScan()


class FakeScannerWithActiveArea(FakeScanner):
    """A FakeScanner that also reports a live active area and mounting, for
    edge-align tests. Mounting lives here (not on FakeLab's frames), matching
    the real GocatorScanner.mounting/frames.instruments.gocator split —
    frames.instruments.gocator is deliberately translation-only, since
    orient_scan() refuses to run if both it and the scan's own mounting
    carry a rotation."""

    def __init__(self, x_mm=-750.0, width_mm=1500.0, mounting=None):
        super().__init__()
        self._active_area = {"x_mm": x_mm, "width_mm": width_mm}
        self.mounting = mounting or SensorMounting()

    def get_active_area(self):
        return dict(self._active_area)


class FakeLab:
    def __init__(self):
        from laguna.frames import FrameRegistry

        self.placed = []
        self.frames = FrameRegistry.from_config({
            "instruments": {"gocator": {"translation": [0, 0, 0]}}
        })
        self.gocator = FakeScanner()
        self.gantry = object()

        class _Clock:
            def elapsed(self):
                return 1.0

            def wall_time(self):
                return 2.0

        self.clock = _Clock()

        class _EventLog:
            def __init__(self):
                self.rows = []

            def log(self, *args, **kwargs):
                self.rows.append((args, kwargs))

        self.event_log = _EventLog()

    def place(self, instrument, point, speed=None, reference_point=None):
        self.placed.append((instrument, tuple(point), speed, reference_point))
        return True


class TestSurveyRunner:
    def _survey(self):
        return Tile(
            origin=(0.0, 0.0, 0.0), length_mm=100.0, width_mm=2000.0,
            swath_mm=1000.0, overlap=0.0, scan_speed=20.0,
        )

    def test_dry_run_moves_nothing(self):
        lab = FakeLab()
        done = SurveyRunner(lab, self._survey()).run(dry_run=True)
        assert len(done) == 2
        assert lab.placed == []
        assert lab.gocator.acquired == []

    def test_dry_run_logs_experiment_and_gantry_coordinates(self, caplog):
        """The sanity-check use case: read the plan in both frames before
        committing to real motion, without needing to run it for real."""
        import logging

        lab = FakeLab()
        with caplog.at_level(logging.INFO, logger="laguna.survey"):
            SurveyRunner(lab, self._survey()).run(dry_run=True)
        assert "experiment" in caplog.text
        assert "gantry" in caplog.text
        # tile origin (0,0,0), no mount offset configured for this FakeLab —
        # experiment and gantry coincide, so both frames show the same values.
        assert "(0.0, 0.0, 0.0)" in caplog.text

    def test_logged_gantry_target_matches_what_actually_gets_commanded(self, caplog):
        """The logged gantry target must be computed the same way as the
        actual placement — same resolved reference_point, not a stand-in."""
        import logging

        lab = FakeLab()
        lab.gocator = FakeScannerWithActiveArea(x_mm=-750.0, width_mm=1500.0)
        with caplog.at_level(logging.INFO, logger="laguna.survey"):
            SurveyRunner(lab, self._survey()).run()
        # identity mount here (FakeLab's default) -> min edge (-750) is used,
        # giving gantry_target = experiment(0,0,0) - offset(-750,0,0) = (750,0,0)
        assert lab.placed[0][3] == [-750.0, 0.0, 0.0]  # reference_point actually placed with
        assert "(750.0, 0.0, 0.0)" in caplog.text

    def test_each_pass_positions_the_instrument_then_measures(self):
        lab = FakeLab()
        SurveyRunner(lab, self._survey()).run()
        assert len(lab.placed) == 2
        assert len(lab.gocator.acquired) == 2
        # place() targets the INSTRUMENT's measuring point, not the gantry's
        # commanded point — that is what makes one plan valid for several.
        assert lab.placed[0][0] == "gocator"

    def test_results_empty_by_default(self):
        """Holding every scan in memory is real cost for a long survey — off
        unless the caller explicitly opts in."""
        lab = FakeLab()
        SurveyRunner(lab, self._survey()).run()
        runner = SurveyRunner(lab, self._survey())
        runner.run()
        assert runner.results == []

    def test_keep_results_collects_one_per_pass_in_order(self):
        lab = FakeLab()
        runner = SurveyRunner(lab, self._survey())
        done = runner.run(keep_results=True)
        assert len(runner.results) == len(done) == 2
        assert all(isinstance(r, FakeScan) for r in runner.results)

    def test_keep_results_ignored_during_dry_run(self):
        lab = FakeLab()
        runner = SurveyRunner(lab, self._survey())
        runner.run(dry_run=True, keep_results=True)
        assert runner.results == []

    def test_place_uses_scan_speed_when_no_travel_speed_set(self):
        lab = FakeLab()
        SurveyRunner(lab, self._survey()).run()
        assert lab.placed[0][2] == 20.0  # the survey's scan_speed

    def test_place_uses_travel_speed_when_set(self):
        survey = Tile(
            origin=(0.0, 0.0, 0.0), length_mm=100.0, width_mm=2000.0,
            swath_mm=1000.0, overlap=0.0, scan_speed=20.0, travel_speed=80.0,
        )
        lab = FakeLab()
        SurveyRunner(lab, survey).run()
        assert lab.placed[0][2] == 80.0
        # the scan itself still runs at scan_speed — travel_speed only
        # affects the pre-scan repositioning move. scanner.acquire()'s own
        # kwarg name (feed_rate_mm_s) is a separate, unrenamed API.
        assert lab.gocator.acquired[0]["feed_rate_mm_s"] == 20.0

    def test_edge_align_uses_active_area_min_edge_with_identity_mount(self):
        """No mounting rotation configured: sensor X and step_axis (Y) point
        the same way (matrix[step_i, 0] >= 0), so the near/offset edge is
        the active area's minimum X."""
        lab = FakeLab()
        lab.gocator = FakeScannerWithActiveArea(x_mm=-750.0, width_mm=1500.0)
        SurveyRunner(lab, self._survey()).run()
        assert lab.placed[0][3] == [-750.0, 0.0, 0.0]

    def test_edge_align_uses_active_area_max_edge_when_mount_sign_flips(self):
        """With the real rig's mounting (scan_x: -Y), increasing sensor X
        moves toward -step_axis, so the near/offset edge is the FAR
        (max-X) boundary of the active area, not the min. The reference
        point comes back already rotated into gantry directions — sensor
        X=750 (the max edge) lands on gantry -Y — since it's read off the
        scanner's own mounting, not frames.instruments.gocator (which stays
        translation-only; see that config block's comment)."""
        lab = FakeLab()
        lab.gocator = FakeScannerWithActiveArea(
            x_mm=-750.0, width_mm=1500.0,
            mounting=SensorMounting(scan_x="-Y", scan_y="+X", scan_z="+Z"),
        )
        SurveyRunner(lab, self._survey()).run()
        assert lab.placed[0][3] == [0.0, -750.0, 0.0]

    def test_edge_align_skipped_when_scanner_has_no_active_area(self):
        """FakeScanner (no get_active_area) is the common case for
        rangefinder-style instruments — must not crash, must fall back to
        the instrument's normal configured reference point."""
        lab = FakeLab()
        SurveyRunner(lab, self._survey()).run()
        assert lab.placed[0][3] is None

    def test_traverse_never_edge_aligns(self):
        """A Traverse pass has no swath to align — even an instrument that
        supports get_active_area() must not get a reference_point override."""
        survey = Traverse(start=(0, 0, 0), end=(100, 0, 0), instruments=("gocator",))
        lab = FakeLab()
        lab.gocator = FakeScannerWithActiveArea()
        SurveyRunner(lab, survey).run()
        assert lab.placed[0][3] is None

    def test_checkpoint_skips_completed_passes(self):
        """A tile of a wide bed can be the longest thing an experiment
        does; an interruption must not restart it."""

        class Store:
            def __init__(self):
                self.done = {0}

            def is_complete(self, i):
                return i in self.done

            def mark_complete(self, i, **kw):
                self.done.add(i)

        lab = FakeLab()
        runner = SurveyRunner(lab, self._survey(), checkpoint=Store())
        done = runner.run()
        assert [p.index for p in done] == [1]

    def test_completed_passes_are_checkpointed(self):
        class Store:
            def __init__(self):
                self.marked = []

            def is_complete(self, i):
                return False

            def mark_complete(self, i, **kw):
                self.marked.append(i)

        store = Store()
        SurveyRunner(FakeLab(), self._survey(), checkpoint=store).run()
        assert store.marked == [0, 1]

    def test_pending_is_everything_without_a_checkpoint(self):
        assert len(SurveyRunner(FakeLab(), self._survey()).pending()) == 2

    def test_each_pass_writes_an_event_log_row(self):
        """A tile ran with no trace in the event log — the one
        cross-subsystem index other tooling reads — was indistinguishable
        from a tile that never ran at all."""
        lab = FakeLab()
        SurveyRunner(lab, self._survey()).run()
        assert len(lab.event_log.rows) == 2
        (args, kwargs) = lab.event_log.rows[0]
        assert args[1] == "gocator"
        assert args[2] == "survey_pass"

    def test_a_failed_pass_is_logged_and_reraised(self):
        """A pass failing partway through a survey is the 'run is no longer
        doing what it was told' case the rest of the codebase escalates —
        it must not be silently swallowed and the survey must not continue
        past it with an unrecorded gap."""

        class ExplodingScanner(FakeScanner):
            def acquire(self, gantry=None, **kw):
                raise RuntimeError("scanner unreachable")

        lab = FakeLab()
        lab.gocator = ExplodingScanner()
        with pytest.raises(RuntimeError, match="scanner unreachable"):
            SurveyRunner(lab, self._survey()).run()

        assert len(lab.event_log.rows) == 1
        (args, kwargs) = lab.event_log.rows[0]
        assert "error" in kwargs.get("result", args[3] if len(args) > 3 else "")

    def test_rangefinder_pass_without_a_scan_speed_raises_clearly(self):
        """acquire_scan() (the rangefinder path) has no configured-spec
        fallback the way GocatorScanner.acquire() does — a Pass reaching it
        with scan_speed=None used to fail deep inside FlumeLab with a
        message that didn't name which pass or survey was responsible."""
        survey = Traverse(
            start=(0, 0, 0), end=(100, 0, 0), instruments=("od2000",),
        )
        lab = FakeLab()
        lab.od2000 = object()  # no acquire() -> goes through acquire_scan()

        def _boom(*a, **kw):
            raise AssertionError("acquire_scan() should not be reached")

        lab.acquire_scan = _boom
        with pytest.raises(ValueError, match="scan_speed"):
            SurveyRunner(lab, survey).run()

    def test_rangefinder_pass_end_covers_every_configured_axis(self):
        """gantry_target_for() only returns X/Y/Z — Theta is outside the
        Cartesian frame model — but acquire_scan() requires one value per
        *configured* gantry axis. A pass on a gantry with a Theta axis used
        to build a 3-long end vector and fail deep inside acquire_scan()
        with a length-mismatch ValueError."""
        survey = Traverse(
            start=(0, 0, 0), end=(100, 0, 0), instruments=("od2000",), scan_speed=20.0,
        )
        lab = FakeLab()
        lab.od2000 = object()  # no acquire() -> goes through acquire_scan()

        class FakeGantry:
            _axes = [Axis("X", 1), Axis("Y", 2), Axis("Z", 5), Axis("Theta", 6)]

            class cmd:
                @staticmethod
                def get_actual_position(axis):
                    return 42.0  # Theta's live position — not part of the frame model

        lab.gantry = FakeGantry()

        seen = {}

        def _acquire_scan(instrument, start=None, end=None, feed_rate_mm_s=None, output=None, axis=None):
            seen["end"] = end
            seen["axis"] = axis

            class _Result:
                path = "fake.csv"

            return _Result()

        lab.acquire_scan = _acquire_scan
        runner = SurveyRunner(lab, survey)
        runner.run(keep_results=True)
        assert seen["end"] == [100.0, 0.0, 0.0, 42.0]  # X, Y, Z, then Theta backfilled
        assert seen["axis"] == "X"
        assert runner.results[0].path == "fake.csv"  # ProfileResult, not the Gocator SurfaceScan branch


class TestRampLeadInIntegration:
    """SurveyRunner wiring for the issue #58 fix.

    A Tile with accel_mm_s2 (explicit, or auto-filled from the live axis)
    commands the gantry from the ramp start while telling the scanner the
    true swath boundary.
    """

    class FakeAxisHandle:
        """Reports a fixed accel, nothing else — that's all `_fill_tile_accel()` reads."""

        def __init__(self, accel_mm_s2):
            """Store the accel this handle reports."""
            self._accel = accel_mm_s2

        def get_accel(self):
            """Return the configured accel, mm/s^2."""
            return self._accel

    class FakeGantryWithAccel:
        """A gantry whose axes report a fixed configured accel."""

        def __init__(self, accel_mm_s2=100.0):
            """Store the accel every axis handle will report."""
            self._accel = accel_mm_s2

        def axis(self, name):
            """Return a FakeAxisHandle reporting this gantry's accel."""
            return TestRampLeadInIntegration.FakeAxisHandle(self._accel)

    def _survey(self, **kw):
        base = dict(
            origin=(0.0, 0.0, 0.0), length_mm=100.0, width_mm=1000.0,
            swath_mm=1000.0, overlap=0.0, scan_speed=20.0, axis="X",
        )
        base.update(kw)
        return Tile(**base)

    def test_explicit_accel_shifts_the_placed_point_not_the_scan_anchor(self):
        """lab.place() gets the ramp start; the scanner gets the boundary.

        `Pass.start` (ramp start) is what's placed; `Pass.cruise_start`
        (the true boundary) reaches the scanner via `cruise_start_mm`.
        """
        survey = self._survey(accel_mm_s2=100.0)
        lab = FakeLab()
        SurveyRunner(lab, survey).run()

        expected_ramp = ramp_distance_mm(20.0, 100.0)
        placed_point = lab.placed[0][1]
        cruise_start = survey.passes()[0].cruise_start
        assert placed_point[0] == pytest.approx(cruise_start[0] - expected_ramp)
        assert lab.gocator.acquired[0]["cruise_start_mm"] == pytest.approx(cruise_start[0])
        assert lab.gocator.acquired[0]["settle_s"] == pytest.approx(
            ramp_time_s(20.0, 100.0)
        )

    def test_no_accel_places_directly_at_the_boundary_as_before(self):
        """Backward compatible: no accel_mm_s2 means no behavior change.

        Without it, placement and the scanner call are identical to
        pre-#58 behavior.
        """
        survey = self._survey()  # accel_mm_s2 left unset
        lab = FakeLab()
        SurveyRunner(lab, survey).run()
        assert lab.placed[0][1][0] == pytest.approx(0.0)  # tile origin, no shift
        assert "cruise_start_mm" not in lab.gocator.acquired[0]

    def test_run_auto_fills_accel_from_the_live_axis(self):
        """A Tile left with accel_mm_s2=None gets it from the live axis.

        `gantry.axis(...).get_accel()` is read before any pass runs, so
        callers don't have to read it themselves.
        """
        survey = self._survey()  # accel_mm_s2 left unset
        lab = FakeLab()
        lab.gantry = self.FakeGantryWithAccel(accel_mm_s2=50.0)
        SurveyRunner(lab, survey).run()
        assert survey.accel_mm_s2 == 50.0
        assert "cruise_start_mm" in lab.gocator.acquired[0]

    def test_explicit_accel_is_not_overwritten_by_auto_fill(self):
        """An explicit accel_mm_s2 wins over whatever the live axis reports."""
        survey = self._survey(accel_mm_s2=25.0)
        lab = FakeLab()
        lab.gantry = self.FakeGantryWithAccel(accel_mm_s2=999.0)
        SurveyRunner(lab, survey).run()
        assert survey.accel_mm_s2 == 25.0

    def test_unreadable_accel_logs_a_warning_and_runs_without_a_lead_in(self, caplog):
        """An accel read failure degrades gracefully, with a clear reason.

        A gantry that can't report accel (unconnected axis, older
        firmware) must not fail the survey — it just loses the seam fix,
        with a warning explaining why, per CLAUDE.md's requirement that a
        pause/degraded-mode's cause be legible in the log.
        """
        import logging

        class BrokenGantry:
            """Raises on any axis lookup, simulating an unreadable accel."""

            def axis(self, name):
                """Simulate a gantry axis that can't be reached."""
                raise RuntimeError("not connected")

        survey = self._survey()  # accel_mm_s2 left unset
        lab = FakeLab()
        lab.gantry = BrokenGantry()
        with caplog.at_level(logging.WARNING, logger="laguna.survey"):
            SurveyRunner(lab, survey).run()
        assert survey.accel_mm_s2 is None
        assert "issue #58" in caplog.text or "seam" in caplog.text
        assert "cruise_start_mm" not in lab.gocator.acquired[0]
