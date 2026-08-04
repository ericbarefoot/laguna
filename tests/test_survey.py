"""Tests for multi-pass survey planning.

The geometry is where the mistakes hide: an off-by-one in the pass count
leaves an unimaged gap down the middle of a bed, and nobody notices until
the data is being stitched.
"""

import pytest

from laguna.survey import Pass, RasterSurvey, RepeatTransect, SurveyRunner


class TestRasterGeometry:
    def _raster(self, **kw):
        base = dict(origin=(0.0, 0.0, 0.0), length_mm=1000.0, width_mm=2000.0,
                    swath_mm=1000.0, overlap=0.0)
        base.update(kw)
        return RasterSurvey(**base)

    def test_exact_fit_needs_no_extra_pass(self):
        assert len(self._raster(width_mm=2000.0, swath_mm=1000.0)) == 2

    def test_a_partial_swath_still_gets_a_pass(self):
        """A region 2.5 swaths wide needs 3 passes. Rounding down would leave
        an unimaged strip, which is far worse than an extra pass."""
        assert len(self._raster(width_mm=2500.0, swath_mm=1000.0)) == 3

    def test_narrower_than_one_swath_is_a_single_pass(self):
        assert len(self._raster(width_mm=200.0, swath_mm=1000.0)) == 1

    def test_width_exactly_one_swath_with_overlap_is_not_duplicated(self):
        """The overcounting bug: ceil(width / pitch) with overlap > 0 and
        width == swath gave 2 — both offsets clamp to 0
        (max(0, width - swath) == 0), so the 'extra' pass was an exact
        duplicate of the first, not new coverage. Coverage-only checks
        (coverage_mm() >= width) can't catch this — a duplicate pass still
        "covers" the region, it just wastes a whole redundant traverse.
        """
        survey = self._raster(width_mm=1000.0, swath_mm=1000.0, overlap=0.1)
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
            survey = self._raster(width_mm=width, swath_mm=swath, overlap=overlap)
            passes = survey.passes()
            offsets = [p.start[1] for p in passes]
            assert len(set(offsets)) == len(offsets), (
                f"duplicate pass offset with width={width} swath={swath} overlap={overlap}: "
                f"{offsets}"
            )

    def test_overlap_increases_the_pass_count(self):
        wide = self._raster(width_mm=2000.0, swath_mm=1000.0, overlap=0.0)
        lapped = self._raster(width_mm=2000.0, swath_mm=1000.0, overlap=0.5)
        assert len(lapped) > len(wide)

    def test_pitch_accounts_for_overlap(self):
        assert self._raster(swath_mm=1000.0, overlap=0.2).pitch_mm == pytest.approx(800.0)

    def test_coverage_is_never_less_than_the_region(self):
        """The property that actually matters: no gaps."""
        for width, swath, overlap in [
            (2000.0, 1000.0, 0.0), (2500.0, 1000.0, 0.1),
            (1234.0, 500.0, 0.25), (100.0, 900.0, 0.1),
        ]:
            survey = self._raster(width_mm=width, swath_mm=swath, overlap=overlap)
            assert survey.coverage_mm() >= width - 1e-9, (
                f"gap left with width={width} swath={swath} overlap={overlap}"
            )

    def test_passes_step_along_the_step_axis(self):
        survey = self._raster(axis="X", width_mm=2000.0, swath_mm=1000.0)
        offsets = [p.start[1] for p in survey]     # Y is the step axis
        assert offsets == sorted(offsets)
        assert offsets[0] == 0.0

    def test_traverse_runs_along_the_travel_axis(self):
        p = self._raster(axis="X", length_mm=750.0).passes()[0]
        assert p.end[0] - p.start[0] == pytest.approx(750.0)
        assert p.end[1] == p.start[1]

    def test_serpentine_alternates_direction(self):
        """Halves repositioning travel — the gantry doesn't drive back to the
        same side after every pass."""
        passes = self._raster(serpentine=True, width_mm=3000.0, swath_mm=1000.0).passes()
        assert passes[0].end[0] > passes[0].start[0]
        assert passes[1].end[0] < passes[1].start[0]
        assert passes[2].end[0] > passes[2].start[0]

    def test_serpentine_can_be_disabled(self):
        passes = self._raster(serpentine=False, width_mm=3000.0, swath_mm=1000.0).passes()
        assert all(p.end[0] > p.start[0] for p in passes)

    def test_indices_are_sequential_from_zero(self):
        assert [p.index for p in self._raster(width_mm=3000.0, swath_mm=1000.0)] == [0, 1, 2]

    @pytest.mark.parametrize("kw,match", [
        ({"swath_mm": 0.0}, "swath_mm must be positive"),
        ({"overlap": 1.0}, "never advance"),
        ({"overlap": -0.1}, "overlap must be"),
        ({"width_mm": 0.0}, "must be positive"),
        ({"axis": "Q"}, "axis must be one of"),
    ])
    def test_impossible_geometry_is_rejected(self, kw, match):
        with pytest.raises(ValueError, match=match):
            self._raster(**kw)

    def test_step_axis_must_differ_from_travel_axis(self):
        with pytest.raises(ValueError, match="must differ"):
            self._raster(axis="X", step_axis="X")

    def test_origin_must_have_three_components(self):
        with pytest.raises(ValueError, match="3 components"):
            self._raster(origin=(0.0, 0.0))


class TestRepeatTransect:
    def test_one_pass_per_instrument_per_repeat(self):
        survey = RepeatTransect(
            start=(0, 0, 0), end=(100, 0, 0),
            instruments=("od2000", "wtt12l"), repeats=3,
        )
        assert len(survey) == 6

    def test_instrument_order_is_preserved(self):
        survey = RepeatTransect(
            start=(0, 0, 0), end=(100, 0, 0), instruments=("od2000", "wtt12l"),
        )
        assert [p.instrument for p in survey] == ["od2000", "wtt12l"]

    def test_every_pass_covers_identical_ground(self):
        """The point of a multi-instrument transect — the frames layer makes
        the same experiment coordinates valid for differently-mounted
        instruments."""
        survey = RepeatTransect(
            start=(10, 20, 0), end=(110, 20, 0),
            instruments=("od2000", "wtt12l"), repeats=2,
        )
        assert len({(p.start, p.end) for p in survey}) == 1

    def test_repeats_must_be_positive(self):
        with pytest.raises(ValueError, match="repeats must be"):
            RepeatTransect(start=(0, 0, 0), end=(1, 0, 0), repeats=0)

    def test_at_least_one_instrument_required(self):
        with pytest.raises(ValueError, match="at least one instrument"):
            RepeatTransect(start=(0, 0, 0), end=(1, 0, 0), instruments=())

    def test_start_must_have_three_components(self):
        with pytest.raises(ValueError, match="3 components"):
            RepeatTransect(start=(0, 0), end=(1, 0, 0))

    def test_end_must_have_three_components(self):
        with pytest.raises(ValueError, match="3 components"):
            RepeatTransect(start=(0, 0, 0), end=(1, 0))


class TestCosting:
    def test_duration_from_length_and_feed_rate(self):
        survey = RasterSurvey(
            origin=(0, 0, 0), length_mm=1000.0, width_mm=2000.0,
            swath_mm=1000.0, overlap=0.0, feed_rate_mm_s=20.0,
        )
        # two passes, 1000mm each, at 20mm/s
        assert survey.duration_s() == pytest.approx(100.0)

    def test_duration_is_none_without_a_feed_rate(self):
        survey = RasterSurvey(origin=(0, 0, 0), length_mm=1000.0, width_mm=1000.0,
                              swath_mm=1000.0)
        assert survey.duration_s() is None

    def test_describe_lists_every_pass(self):
        survey = RasterSurvey(origin=(0, 0, 0), length_mm=100.0, width_mm=2000.0,
                              swath_mm=1000.0, overlap=0.0)
        text = survey.describe()
        assert "2 passes" in text
        assert text.count("raster") == 2


class FakeScanner:
    def __init__(self):
        self.acquired = []

    def acquire(self, gantry=None, **kw):
        self.acquired.append(kw)


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

    def place(self, instrument, point, speed=None):
        self.placed.append((instrument, tuple(point)))
        return True


class TestSurveyRunner:
    def _survey(self):
        return RasterSurvey(
            origin=(0.0, 0.0, 0.0), length_mm=100.0, width_mm=2000.0,
            swath_mm=1000.0, overlap=0.0, feed_rate_mm_s=20.0,
        )

    def test_dry_run_moves_nothing(self):
        lab = FakeLab()
        done = SurveyRunner(lab, self._survey()).run(dry_run=True)
        assert len(done) == 2
        assert lab.placed == []
        assert lab.gocator.acquired == []

    def test_each_pass_positions_the_instrument_then_measures(self):
        lab = FakeLab()
        SurveyRunner(lab, self._survey()).run()
        assert len(lab.placed) == 2
        assert len(lab.gocator.acquired) == 2
        # place() targets the INSTRUMENT's measuring point, not the gantry's
        # commanded point — that is what makes one plan valid for several.
        assert lab.placed[0][0] == "gocator"

    def test_checkpoint_skips_completed_passes(self):
        """A raster of a wide bed can be the longest thing an experiment
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
        """A raster ran with no trace in the event log — the one
        cross-subsystem index other tooling reads — was indistinguishable
        from a raster that never ran at all."""
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

    def test_rangefinder_pass_without_a_feed_rate_raises_clearly(self):
        """acquire_scan() (the rangefinder path) has no configured-spec
        fallback the way GocatorScanner.acquire() does — a Pass reaching it
        with feed_rate_mm_s=None used to fail deep inside FlumeLab with a
        message that didn't name which pass or survey was responsible."""
        survey = RepeatTransect(
            start=(0, 0, 0), end=(100, 0, 0), instruments=("od2000",),
        )
        lab = FakeLab()
        lab.od2000 = object()  # no acquire() -> goes through acquire_scan()

        def _boom(*a, **kw):
            raise AssertionError("acquire_scan() should not be reached")

        lab.acquire_scan = _boom
        with pytest.raises(ValueError, match="feed_rate_mm_s"):
            SurveyRunner(lab, survey).run()
