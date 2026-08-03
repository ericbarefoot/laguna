"""Tests for run identity and the runtime<->wall timeline.

The mapping is the interesting part. ExperimentClock keeps its start time and
pause offset privately and they die with the process, so after a run there
was no way to convert a scan file's wall-clock timestamp back into experiment
runtime. Pauses make it piecewise, and FlumeLab.pause() now stops the clock —
so this is the normal case, not a corner case.
"""

import json

import pytest

from laguna import FlumeLab
from laguna.run_context import RunContext, new_run_id


class TestRunId:
    def test_is_sortable_and_unique(self):
        a, b = new_run_id(), new_run_id()
        assert a != b, "two runs in the same second must not share an id"
        # Timestamp-first, so ids sort chronologically by name.
        assert a[:8].isdigit() and "T" in a

    def test_context_generates_one_by_default(self):
        assert RunContext().run_id

    def test_explicit_id_is_kept(self):
        assert RunContext(run_id="fixed").run_id == "fixed"


class TestRunDirectory:
    def test_disabled_by_default_so_nothing_moves(self):
        """A rig that hasn't opted in must keep writing exactly where it did."""
        ctx = RunContext()
        assert ctx.directory is None
        assert str(ctx.path_for("gocator", "./data/scans")) == "data/scans"

    def test_enabled_root_gives_each_subsystem_a_subdirectory(self, tmp_path):
        ctx = RunContext(root=str(tmp_path), run_id="R1")
        path = ctx.path_for("gocator", "./data/scans")
        assert path == tmp_path / "R1" / "gocator"
        assert path.is_dir()

    def test_directory_is_created_on_use(self, tmp_path):
        ctx = RunContext(root=str(tmp_path), run_id="R2")
        assert ctx.directory.is_dir()


class TestTimeline:
    def _ctx(self, tmp_path):
        return RunContext(root=str(tmp_path), run_id="R")

    def test_runtime_matches_wall_when_never_paused(self, tmp_path):
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 1000.0
        assert ctx.runtime_at(1060.0) == pytest.approx(60.0)

    def test_a_pause_shifts_the_two_timelines_apart(self, tmp_path):
        """The whole reason the manifest exists: 600s of runtime is a
        different wall instant depending on how long the run was paused."""
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 1000.0
        ctx.pauses = [[1030.0, 1070.0]]          # 40s paused
        # 100s of wall time, 40 of it paused -> 60s of runtime
        assert ctx.runtime_at(1100.0) == pytest.approx(60.0)

    def test_multiple_pauses_accumulate(self, tmp_path):
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 0.0
        ctx.pauses = [[10.0, 20.0], [30.0, 45.0]]   # 10s + 15s
        assert ctx.runtime_at(100.0) == pytest.approx(75.0)

    def test_a_pause_after_the_instant_does_not_count(self, tmp_path):
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 0.0
        ctx.pauses = [[80.0, 90.0]]
        assert ctx.runtime_at(50.0) == pytest.approx(50.0)

    def test_an_instant_inside_a_pause_clamps(self, tmp_path):
        """Runtime must not advance while paused."""
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 0.0
        ctx.pauses = [[10.0, 50.0]]
        assert ctx.runtime_at(30.0) == pytest.approx(10.0)

    def test_wall_at_inverts_runtime_at(self, tmp_path):
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 1000.0
        ctx.pauses = [[1030.0, 1070.0]]
        for runtime in (0.0, 10.0, 30.0, 60.0):
            assert ctx.runtime_at(ctx.wall_at(runtime)) == pytest.approx(runtime)

    def test_an_open_pause_is_handled(self, tmp_path):
        """A run killed while paused still leaves a usable timeline."""
        ctx = self._ctx(tmp_path)
        ctx.started_wall = 0.0
        ctx.pauses = [[10.0, None]]
        assert ctx.runtime_at(50.0) == pytest.approx(10.0)

    def test_double_pause_does_not_open_two_intervals(self, tmp_path):
        """Two open intervals would make the mapping ambiguous."""
        ctx = self._ctx(tmp_path)
        ctx.started()
        ctx.paused()
        ctx.paused()
        assert len([p for p in ctx.pauses if p[1] is None]) == 1

    def test_ending_closes_an_open_pause(self, tmp_path):
        ctx = self._ctx(tmp_path)
        ctx.started()
        ctx.paused()
        ctx.ended()
        assert ctx.pauses[-1][1] is not None

    def test_runtime_without_a_start_is_an_error(self, tmp_path):
        with pytest.raises(RuntimeError, match="no recorded start"):
            self._ctx(tmp_path).runtime_at(10.0)


class TestManifest:
    def test_written_on_every_change_not_just_at_the_end(self, tmp_path):
        """A run killed mid-flight is exactly when reconstructing a timeline
        by hand is hardest, so the manifest must already be on disk."""
        ctx = RunContext(root=str(tmp_path), run_id="R")
        ctx.started()
        assert (tmp_path / "R" / "run.json").exists()

    def test_round_trips(self, tmp_path):
        ctx = RunContext(root=str(tmp_path), run_id="R")
        ctx.started()
        ctx.paused()
        ctx.resumed()
        ctx.record_output("gocator", "/tmp/scan.npz", 12.5, points=100)
        ctx.ended()

        loaded = RunContext.load(str(tmp_path / "R"))
        assert loaded.run_id == "R"
        assert len(loaded.pauses) == 1 and loaded.pauses[0][1] is not None
        assert loaded.outputs[0]["subsystem"] == "gocator"
        assert loaded.outputs[0]["points"] == 100

    def test_loads_from_the_file_or_the_directory(self, tmp_path):
        ctx = RunContext(root=str(tmp_path), run_id="R")
        ctx.started()
        assert RunContext.load(str(tmp_path / "R" / "run.json")).run_id == "R"

    def test_write_never_raises(self, tmp_path):
        """Manifest trouble must not take down an experiment."""
        ctx = RunContext(root=str(tmp_path / "nested"), run_id="R")
        ctx.started()
        ctx.root = tmp_path / "\0illegal"      # unwritable
        assert ctx.write() is None             # logged, not raised

    def test_no_manifest_when_run_dir_is_disabled(self):
        ctx = RunContext()
        ctx.started()
        assert ctx.write() is None

    def test_stamp_carries_both_timelines(self):
        stamp = RunContext(run_id="R").stamp(runtime_s=42.0)
        assert stamp["run_id"] == "R"
        assert stamp["runtime_s"] == 42.0
        assert "wall_time" in stamp


class TestFlumeLabIntegration:
    def test_a_run_id_exists_from_construction(self):
        assert FlumeLab().run.run_id

    def test_run_dir_off_unless_configured(self):
        assert FlumeLab().run.directory is None

    def test_pause_and_resume_are_recorded(self):
        lab = FlumeLab()
        lab.clock.start()
        lab.run.started()
        lab.pause()
        assert lab.run.pauses and lab.run.pauses[-1][1] is None
        lab.resume_from_pause()
        assert lab.run.pauses[-1][1] is not None

    def test_the_recorded_timeline_matches_the_clock(self):
        """The manifest has to agree with the clock it is describing —
        otherwise converting a file timestamp gives the wrong runtime."""
        import time

        lab = FlumeLab()
        lab.clock.start()
        lab.run.started()
        time.sleep(0.05)
        lab.pause()
        time.sleep(0.1)
        lab.resume_from_pause()
        time.sleep(0.05)

        wall, runtime = lab.clock.now()
        assert lab.run.runtime_at(wall) == pytest.approx(runtime, abs=0.03)

    def test_start_opens_the_run_and_disconnect_all_closes_it(self, tmp_path):
        """start()/disconnect_all() is the production entry point used by
        run_blocking() — unlike experiment(), it never called run.started()/
        ended() at all, leaving run.json permanently unanchored (started_wall
        stayed None) for every real run.
        """
        lab = FlumeLab()
        assert lab.run.started_wall is None

        thread = lab.start(duration=0.05)
        thread.join(timeout=5.0)
        assert lab.run.started_wall is not None

        lab.disconnect_all()
        assert lab.run.ended_wall is not None

    def test_disconnect_all_before_any_start_does_not_fabricate_a_run(self):
        """disconnect_all() is also called defensively/for cleanup outside a
        run (e.g. at the top of a script); it must not stamp ended_wall for
        a run that never started."""
        lab = FlumeLab()
        lab.disconnect_all()
        assert lab.run.started_wall is None
        assert lab.run.ended_wall is None

    def test_run_id_appears_in_the_event_log(self, tmp_path):
        from laguna.timing import EventLog

        lab = FlumeLab()
        log = tmp_path / "events.csv"
        lab.event_log = EventLog(str(log))
        with lab.experiment(checkpoint_file=str(tmp_path / "cp.json")):
            pass
        assert lab.run.run_id in log.read_text()


class TestFilenameCollisions:
    """Behavioral coverage lives in TestSaveScan (test_gocator_scanner.py) and
    TestProfilerScan (test_profiler.py) — each saves twice in quick
    succession and asserts the two paths actually differ, rather than only
    checking that a '%f' format specifier appears in the source."""
