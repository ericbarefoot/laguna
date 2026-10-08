"""Config-driven surveys (GH #47): ``surveys:`` in the YAML, scheduled like any subsystem."""

import pytest
import yaml

from laguna.experiment.runner import setup_run
from laguna.survey import Tile, Traverse
from laguna.survey_config import build_survey, survey_instruments

TILE = {
    "kind": "tile", "instrument": "gocator", "origin": [0, 0, 0],
    "length_mm": 100.0, "width_mm": 400.0, "swath_mm": 200.0, "overlap": 0.0,
    "scan_speed": 20.0, "interval_s": 600,
}

ROI_TILE = {
    "kind": "tile", "roi": {"x_mm": [100, 700], "y_mm": [0, 2400], "z_mm": 50},
    "swath_mm": 1000.0, "scan_speed": 20.0, "interval_s": 600,
}


class _Frames:
    """Stands in for a FrameRegistry whose experiment frame is rotated 90 degrees."""

    def experiment_axis_for(self, gantry_axis):
        return {"X": "Y", "Y": "X"}[gantry_axis]


class _Lab:
    frames = _Frames()

    class gocator:        # noqa: N801 - mimics the attribute a lab exposes
        @staticmethod
        def get_active_area():
            return {"width_mm": 1000.0}


class TestBuildSurvey:
    def test_tile_is_built_from_its_own_constructor_arguments(self):
        survey, options = build_survey("bed", TILE)
        assert isinstance(survey, Tile)
        assert len(survey) == 2 and survey.scan_speed == 20.0
        assert options == {}

    def test_traverse_is_built_too(self):
        survey, _ = build_survey("line", {
            "kind": "traverse", "start": [0, 0, 0], "end": [100, 0, 0],
            "instruments": ["gocator", "od2000"], "repeats": 2, "scan_speed": 10.0,
            "trigger_at": [0, 60],
        })
        assert isinstance(survey, Traverse) and len(survey) == 4
        assert survey_instruments(survey) == ("gocator", "od2000")

    def test_scheduling_keys_are_not_planner_arguments(self):
        build_survey("bed", {**TILE, "trigger_at": [0]})      # does not raise

    def test_runner_options_are_split_out(self):
        _, options = build_survey("bed", {**TILE, "max_scan_speed_mm_s": 40.0})
        assert options == {"max_scan_speed_mm_s": 40.0}

    @pytest.mark.parametrize("kind", [None, "raster", ""])
    def test_a_missing_or_unknown_kind_is_refused(self, kind):
        spec = {k: v for k, v in TILE.items() if k != "kind"}
        if kind is not None:
            spec["kind"] = kind
        with pytest.raises(ValueError, match="'kind'"):
            build_survey("bed", spec)

    def test_a_typo_is_an_error_not_a_silent_default(self):
        with pytest.raises(ValueError, match=r"unknown key.*scan_spede"):
            build_survey("bed", {**TILE, "scan_spede": 99})

    def test_a_missing_required_field_names_the_survey(self):
        spec = {k: v for k, v in TILE.items() if k != "length_mm"}
        with pytest.raises(ValueError, match=r"\[surveys\.bed\].*length_mm"):
            build_survey("bed", spec)

    def test_the_planners_own_validation_still_applies(self):
        with pytest.raises(ValueError):
            build_survey("bed", {**TILE, "swath_mm": -5})

    def test_use_schedule_is_refused(self):
        with pytest.raises(ValueError, match="use_schedule"):
            build_survey("bed", {**TILE, "use_schedule": True})

    def test_auto_swath_checks_geometry_with_a_stand_in_at_setup(self):
        survey, _ = build_survey("bed", {**TILE, "swath_mm": "auto"})
        assert survey.swath_mm == TILE["width_mm"]

    def test_auto_swath_reads_the_live_active_area(self):
        class Lab:
            class gocator:
                @staticmethod
                def get_active_area():
                    return {"x_mm": -100.0, "width_mm": 200.0}

        survey, _ = build_survey("bed", {**TILE, "swath_mm": "auto"}, lab=Lab())
        assert survey.swath_mm == 200.0 and len(survey) == 2

    def test_auto_swath_without_an_active_area_is_refused(self):
        class Lab:
            gocator = object()

        with pytest.raises(ValueError, match="active area"):
            build_survey("bed", {**TILE, "swath_mm": "auto"}, lab=Lab())


GANTRY = {"transport": "pi_agent", "safe_mode": True, "axes": [{"name": "X", "index": 1}]}


def _config(tmp_path, surveys, **sections):
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump({
        "gantry": GANTRY, "gocator": {"ip": "192.168.1.10"}, "surveys": surveys, **sections,
    }))
    return str(path)


class TestSetupRunWiring:
    def test_a_survey_is_scheduled_like_any_subsystem(self, tmp_path):
        lab = setup_run(_config(tmp_path, {"bed": TILE}), simulate=True)
        jobs = [j for j in lab.scheduler._recurring if j["subsystem"] == "survey"]
        assert [j["name"] for j in jobs] == ["bed"]

    def test_a_roi_survey_is_scheduled_and_swath_auto_is_deferred_to_each_firing(self, tmp_path):
        spec = {**ROI_TILE, "swath_mm": "auto"}
        lab = setup_run(_config(tmp_path, {"bed": spec}), simulate=True)
        assert [j["name"] for j in lab.scheduler._recurring if j["subsystem"] == "survey"] == ["bed"]

    def test_trigger_at_schedules_one_shot_firings(self, tmp_path):
        spec = {k: v for k, v in TILE.items() if k != "interval_s"} | {"trigger_at": [10, 20]}
        lab = setup_run(_config(tmp_path, {"bed": spec}), simulate=True)
        shots = [j for j in lab.scheduler._oneshot if j["subsystem"] == "survey"]
        assert len(shots) == 2
        assert not [j for j in lab.scheduler._recurring if j["subsystem"] == "survey"]

    @pytest.mark.parametrize("mutate, message", [
        ({"scan_spede": 1}, "unknown key"),
        ({"instrument": "wtt12l"}, "aren't configured"),
        ({"swath_mm": -1}, "swath"),
    ])
    def test_bad_plans_fail_at_setup_before_anything_connects(self, tmp_path, mutate, message):
        with pytest.raises(ValueError, match=message):
            setup_run(_config(tmp_path, {"bed": {**TILE, **mutate}}), simulate=True)

    def test_an_unscheduled_survey_is_refused(self, tmp_path):
        spec = {k: v for k, v in TILE.items() if k != "interval_s"}
        with pytest.raises(ValueError, match="interval_s' or 'trigger_at"):
            setup_run(_config(tmp_path, {"bed": spec}), simulate=True)

    def test_conflicting_triggers_are_refused(self, tmp_path):
        with pytest.raises(ValueError, match="mutually exclusive"):
            setup_run(_config(tmp_path, {"bed": {**TILE, "trigger_at": [0]}}), simulate=True)

    def test_a_survey_with_no_gantry_is_refused(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({"gocator": {"ip": "1.2.3.4"}, "surveys": {"bed": TILE}}))
        with pytest.raises(ValueError, match="no 'gantry:'"):
            setup_run(str(path), simulate=True)

    def test_surveys_must_be_a_mapping(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({"gantry": GANTRY, "gocator": {"ip": "1.2.3.4"}, "surveys": [1]}))
        with pytest.raises(ValueError, match="mapping"):
            setup_run(str(path), simulate=True)

    def test_a_config_without_surveys_is_unchanged(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({"gantry": GANTRY, "gocator": {"ip": "1.2.3.4"}}))
        lab = setup_run(str(path), simulate=True)
        assert all(j["subsystem"] != "survey" for j in lab.scheduler._recurring)


class _FakeRun:
    def __init__(self, root):
        self.root = root
        self.outputs = []
        self.stamps = 0

    def path_for(self, subsystem, default_dir, configured_dir=None):
        from pathlib import Path

        return Path(self.root) / subsystem

    def stamp(self, runtime_s):
        self.stamps += 1
        return {"run_id": "R", "runtime_s": runtime_s}

    def record_output(self, subsystem, path, runtime_s=None, **extra):
        self.outputs.append((subsystem, str(path), extra))


def _firing_lab(tmp_path):
    """A survey-capable lab double: FakeLab plus the run/escalate surface the closure uses."""
    from tests.test_survey import FakeLab

    lab = FakeLab()
    lab.run = _FakeRun(tmp_path)
    lab.escalations = []
    lab.escalate = lambda msg, *a, **k: lab.escalations.append(msg)
    lab._subsystems = {"gocator": lab.gocator, "gantry": lab.gantry}
    lab.gocator._run_stamp = None
    return lab


class TestScheduledFiring:
    def _action(self, lab, spec=None):
        from laguna.experiment.runner import _make_survey_action

        return _make_survey_action(lab, "bed", dict(spec or TILE))

    def test_a_firing_runs_every_pass_and_records_its_checkpoint(self, tmp_path):
        lab = _firing_lab(tmp_path)
        self._action(lab)()
        assert len(lab.gocator.acquired) == 2
        assert lab.escalations == []
        (subsystem, path, extra), = lab.run.outputs
        assert subsystem == "surveys" and path.endswith("bed_0001.checkpoint.json")
        assert extra == {"survey": "bed", "passes": 2}

    def test_each_firing_measures_the_whole_plan_again(self, tmp_path):
        """A reused checkpoint would find every pass 'complete' and scan nothing."""
        lab = _firing_lab(tmp_path)
        action = self._action(lab)
        action()
        action()
        assert len(lab.gocator.acquired) == 4
        paths = sorted(p.name for p in (tmp_path / "surveys").glob("*.json"))
        assert paths == ["bed_0001.checkpoint.json", "bed_0002.checkpoint.json"]

    def test_checkpoints_are_kept_not_deleted(self, tmp_path):
        lab = _firing_lab(tmp_path)
        self._action(lab)()
        assert len(list((tmp_path / "surveys").glob("*.json"))) == 1

    def test_the_gocator_is_stamped_with_the_run_for_provenance(self, tmp_path):
        lab = _firing_lab(tmp_path)
        self._action(lab)()
        assert lab.gocator._run_stamp == {"run_id": "R", "runtime_s": 1.0}

    def test_a_failed_pass_escalates_and_names_the_checkpoint(self, tmp_path):
        lab = _firing_lab(tmp_path)

        def explode(gantry=None, **kw):
            raise RuntimeError("scanner dropped")

        lab.gocator.acquire = explode
        self._action(lab)()                                      # never raises
        (message,) = lab.escalations
        assert "scanner dropped" in message and "bed_0001.checkpoint.json" in message
        assert lab.event_log.rows[-1][1]["result"].startswith("error:")
        assert lab.run.outputs == []                             # not recorded as finished

    def test_a_halt_is_logged_but_does_not_escalate_again(self, tmp_path):
        from laguna.robot.macron import MotionHalted
        from laguna.robot.macron.halt import HaltLevel

        lab = _firing_lab(tmp_path)

        def halted(gantry=None, **kw):
            raise MotionHalted(HaltLevel.PAUSE, "paused by operator")

        lab.gocator.acquire = halted
        self._action(lab)()
        assert lab.escalations == []
        assert lab.event_log.rows[-1][1]["result"].startswith("interrupted:")

    def test_a_planning_failure_at_firing_time_escalates(self, tmp_path):
        lab = _firing_lab(tmp_path)
        spec = {**TILE, "swath_mm": "auto"}                      # FakeScanner has no active area
        self._action(lab, spec)()
        assert len(lab.escalations) == 1 and "active area" in lab.escalations[0]

    def test_auto_swath_is_read_afresh_each_firing(self, tmp_path):
        from tests.test_survey import FakeScannerWithActiveArea

        lab = _firing_lab(tmp_path)
        lab.gocator = lab._subsystems["gocator"] = FakeScannerWithActiveArea(width_mm=200.0)
        lab.gocator._run_stamp = None
        action = self._action(lab, {**TILE, "swath_mm": "auto"})
        action()
        lab.gocator._active_area["width_mm"] = 400.0             # sensor reconfigured between firings
        action()
        assert len(lab.gocator.acquired) == 2 + 1               # 2 swaths of 200, then 1 of 400


class TestRoiForm:
    def test_roi_matches_tile_from_roi(self):
        survey, _ = build_survey("bed", ROI_TILE)
        expected = Tile.from_roi(
            [100, 700], [0, 2400], 50, swath_mm=1000.0, scan_speed=20.0
        )
        assert [(p.start, p.end) for p in survey.passes()] == [
            (p.start, p.end) for p in expected.passes()
        ]
        assert len(survey) == 3 and survey.scan_speed == 20.0

    def test_min_overlap_and_single_pass_centring_are_passed_through(self):
        spec = {**ROI_TILE, "min_overlap": 0.5}
        assert len(build_survey("bed", spec)[0]) > 3
        narrow = {**ROI_TILE, "roi": {"x_mm": [0, 100], "y_mm": [400, 700], "z_mm": 0},
                  "center_single_pass": False}
        survey, _ = build_survey("bed", narrow)
        assert len(survey) == 1 and survey.passes()[0].start[1] == pytest.approx(400.0)

    def test_swath_auto_reads_the_live_active_area(self):
        spec = {k: v for k, v in ROI_TILE.items() if k != "swath_mm"}
        survey, _ = build_survey("bed", {**spec, "swath_mm": "auto"}, lab=_Lab())
        assert survey.swath_mm == 1000.0

    def test_setup_time_validation_needs_no_lab_for_auto_swath(self):
        survey, _ = build_survey("bed", {**ROI_TILE, "swath_mm": "auto"})
        assert isinstance(survey, Tile)

    def test_gantry_axis_is_resolved_against_the_labs_frames(self):
        survey, _ = build_survey("bed", {**ROI_TILE, "gantry_axis": "X"}, lab=_Lab())
        assert survey.axis == "Y"
        build_survey("bed", {**ROI_TILE, "gantry_axis": "X"})    # setup-time: no lab, no error

    def test_axis_and_gantry_axis_together_are_refused_even_at_setup(self):
        with pytest.raises(ValueError, match="not both"):
            build_survey("bed", {**ROI_TILE, "axis": "X", "gantry_axis": "X"})

    @pytest.mark.parametrize("key", ["origin", "length_mm", "width_mm", "overlap", "step_axis"])
    def test_hand_given_geometry_contradicts_the_region(self, key):
        value = {"origin": [0, 0, 0], "step_axis": "Y"}.get(key, 1.0)
        with pytest.raises(ValueError, match=key):
            build_survey("bed", {**ROI_TILE, key: value})

    def test_roi_needs_exactly_x_y_and_z(self):
        for roi in ({"x_mm": [0, 1], "y_mm": [0, 1]}, {"x_mm": [0, 1], "y_mm": [0, 1],
                                                       "z_mm": 0, "w": 1}, [0, 1]):
            with pytest.raises(ValueError, match="'roi' must be a mapping"):
                build_survey("bed", {**ROI_TILE, "roi": roi})

    def test_a_reversed_region_is_refused_naming_the_survey(self):
        bad = {**ROI_TILE, "roi": {"x_mm": [700, 100], "y_mm": [0, 2400], "z_mm": 50}}
        with pytest.raises(ValueError, match=r"\[surveys\.bed\].*max > min"):
            build_survey("bed", bad)

    def test_roi_is_only_for_tiles(self):
        with pytest.raises(ValueError, match="only valid for kind 'tile'"):
            build_survey("line", {"kind": "traverse", "roi": ROI_TILE["roi"],
                                  "start": [0, 0, 0], "end": [1, 0, 0]})

    def test_a_typo_is_still_an_error(self):
        with pytest.raises(ValueError, match=r"unknown key.*scan_spede"):
            build_survey("bed", {**ROI_TILE, "scan_spede": 9})
