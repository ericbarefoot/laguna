"""Clock, scheduler, checkpoint and schedule-loading behaviour that protects the data.

The failure modes here are quiet ones — nothing crashes, a capture just never
happens, or a log row records the wrong runtime — so each test names the
specific way data used to go missing.
"""

from __future__ import annotations

import io
import json
import threading
import time

import pandas as pd
import pytest

from laguna.schedule import ExperimentSchedule
from laguna.timing import CheckpointCorruptError, CheckpointStore, ExperimentClock, Scheduler


class _RecordingLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, event_type, result="ok", notes="", refers_to=None):
        self.rows.append((round(runtime_s, 3), subsystem, event_type, result))
        return len(self.rows)


def _wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


# ---------------------------------------------------------------------------
# Clock
# ---------------------------------------------------------------------------


class TestClock:
    def test_elapsed_is_frozen_after_stop(self):
        """It used to keep counting with wall time, so every runtime logged
        after a run ended was wrong."""
        clock = ExperimentClock()
        clock.start()
        time.sleep(0.05)
        clock.stop()
        frozen = clock.elapsed()
        time.sleep(0.1)
        assert clock.elapsed() == pytest.approx(frozen, abs=1e-6)
        assert clock.now()[1] == pytest.approx(frozen, abs=1e-6)

    def test_stopping_while_paused_freezes_at_the_paused_runtime(self):
        clock = ExperimentClock()
        clock.start()
        time.sleep(0.05)
        clock.pause()
        paused_at = clock.elapsed()
        time.sleep(0.05)
        clock.stop()
        time.sleep(0.05)
        assert clock.elapsed() == pytest.approx(paused_at, abs=1e-3)

    def test_concurrent_pause_resume_never_breaks_a_reader(self):
        """A read landing mid-transition used to see a None pause start (a
        TypeError that silently killed the scheduler thread)."""
        clock = ExperimentClock()
        clock.start()
        errors, stop = [], threading.Event()

        def reader():
            last = 0.0
            while not stop.is_set():
                try:
                    now = clock.elapsed()
                except Exception as exc:  # pragma: no cover - the failure being tested
                    errors.append(exc)
                    return
                if now < last - 1e-9:
                    errors.append(AssertionError(f"runtime went backwards {last} -> {now}"))
                    return
                last = now

        t = threading.Thread(target=reader)
        t.start()
        for _ in range(2000):
            clock.pause()
            clock.resume()
        stop.set()
        t.join()
        assert errors == []


# ---------------------------------------------------------------------------
# Scheduler
# ---------------------------------------------------------------------------


def _scheduler(speed=100.0):
    clock = ExperimentClock(speed_factor=speed)
    log = _RecordingLog()
    return Scheduler(clock, log), clock, log


class TestSchedulerNeverResumesTheClock:
    def test_run_refuses_a_paused_clock(self):
        """Regression: run() resumed the clock itself, so a bare lab.resume()
        restarted the schedule straight out of an estop."""
        scheduler, clock, _ = _scheduler()
        clock.start()
        clock.pause()
        with pytest.raises(RuntimeError, match="paused"):
            scheduler.run(1.0)
        with pytest.raises(RuntimeError, match="paused"):
            scheduler.run_async(1.0)
        assert clock.is_paused

    def test_run_refuses_a_second_concurrent_loop(self):
        """A quick pause-resume used to leave two loops running, double-firing."""
        scheduler, clock, _ = _scheduler(speed=1.0)
        clock.start()
        t = scheduler.run_async(5.0)
        assert _wait_for(lambda: scheduler.is_running)
        with pytest.raises(RuntimeError, match="already running"):
            scheduler.run_async(5.0)
        scheduler.stop()
        t.join(timeout=2)
        assert not scheduler.is_running


class TestSchedulerFiring:
    def test_an_action_due_exactly_at_the_end_fires(self):
        """The end check came before the firing loops, dropping it."""
        scheduler, clock, _ = _scheduler()
        fired = threading.Event()
        scheduler.at(runtime_s=2.0, action=fired.set, name="final_capture")
        clock.start()
        scheduler.run(2.0)
        assert fired.wait(1.0)

    def test_pausing_does_not_restart_a_repeat_interval(self):
        """Each run() used to reset every interval, so pause/resume cycles
        shorter than the interval postponed a capture forever."""
        scheduler, clock, _ = _scheduler(speed=1.0)
        fired = []
        scheduler.repeat(every=0.3, action=lambda: fired.append(clock.elapsed()), name="capture")
        clock.start()
        for _ in range(3):
            t = scheduler.run_async(0.15)  # never a full interval in one go
            t.join(timeout=2)
            clock.resume()
        scheduler.run(0.1)
        assert _wait_for(lambda: len(fired) >= 1), "the interval restarted on every resume"

    def test_a_firing_after_stop_is_skipped_and_logged(self):
        """A firing dispatched just before stop() used to run just after it."""
        scheduler, clock, log = _scheduler()
        ran = []
        token = threading.Event()
        token.set()  # this run has already been stopped
        scheduler._fire(lambda: ran.append(True), "flow", "update", 1.0, token)
        assert _wait_for(lambda: log.rows)
        assert ran == []
        assert log.rows[0][2:] == ("update", "skipped: scheduler stopped")

    def test_the_gate_can_veto_a_firing(self):
        scheduler, clock, log = _scheduler()
        scheduler.gate = lambda: "rig is ESTOPPED"
        ran = []
        scheduler._fire(lambda: ran.append(True), "flow", "update", 1.0, threading.Event())
        assert _wait_for(lambda: log.rows)
        assert ran == []
        assert "ESTOPPED" in log.rows[0][3]

    def test_a_gate_that_raises_fails_closed(self):
        scheduler, clock, log = _scheduler()

        def broken():
            raise RuntimeError("state unknown")

        scheduler.gate = broken
        ran = []
        scheduler._fire(lambda: ran.append(True), "flow", "update", 1.0, threading.Event())
        assert _wait_for(lambda: log.rows)
        assert ran == []

    def test_a_crashed_loop_is_recorded_and_the_clock_paused(self):
        """It used to die silently with the clock still running."""
        scheduler, clock, log = _scheduler()
        clock.start()
        scheduler._loop = lambda duration, stop_event: (_ for _ in ()).throw(RuntimeError("boom"))
        scheduler.run(1.0)
        assert any(row[1:3] == ("scheduler", "run") and "boom" in row[3] for row in log.rows)
        assert clock.is_paused
        assert not scheduler.is_running

    def test_skipped_backlog_is_written_to_the_event_log(self):
        scheduler, clock, log = _scheduler(speed=1.0)
        scheduler._MAX_CATCHUP = 2
        scheduler.repeat(every=0.001, action=lambda: None, name="dense")
        clock.start()
        time.sleep(0.05)  # fall far behind before the first poll
        scheduler.run(0.06)
        assert any("skipped" in row[3] and "fell behind" in row[3] for row in log.rows)


# ---------------------------------------------------------------------------
# Checkpoint
# ---------------------------------------------------------------------------


class TestCheckpoint:
    def test_a_fresh_start_moves_the_old_checkpoint_aside_instead_of_deleting_it(self, tmp_path):
        """resume=False used to delete the only record of which passes of a
        crashed run had completed."""
        path = tmp_path / "cp.json"
        CheckpointStore(str(path)).mark_complete(1, 10.0, 1000.0)
        CheckpointStore(str(path), resume=False)
        kept = list(tmp_path.glob("cp.json.*.bak"))
        assert len(kept) == 1
        assert json.loads(kept[0].read_text())["events"][0]["id"] == 1

    def test_clear_moves_aside_too(self, tmp_path):
        path = tmp_path / "cp.json"
        store = CheckpointStore(str(path))
        store.mark_complete(1, 10.0, 1000.0)
        store.clear()
        assert not path.exists()
        assert len(list(tmp_path.glob("cp.json.*.bak"))) == 1

    @pytest.mark.parametrize("content", ["{not json", "[1, 2]", '{"events": 5}', '{"other": []}'])
    def test_a_corrupt_checkpoint_is_kept_and_reported_not_treated_as_empty(self, tmp_path, content):
        """It used to resume as if nothing had completed, re-run every pass,
        and overwrite the evidence."""
        path = tmp_path / "cp.json"
        path.write_text(content)
        with pytest.raises(CheckpointCorruptError):
            CheckpointStore(str(path), resume=True)
        corrupt = list(tmp_path.glob("cp.json.*.corrupt"))
        assert len(corrupt) == 1 and corrupt[0].read_text() == content

    def test_resume_reloads_completed_events(self, tmp_path):
        path = tmp_path / "cp.json"
        CheckpointStore(str(path)).mark_complete(3, 10.0, 1000.0)
        assert CheckpointStore(str(path), resume=True).is_complete(3)

    def test_experiment_exposes_its_checkpoint(self, tmp_path):
        """experiment() built a store and dropped it, so nothing could record into it."""
        from laguna import FlumeLab

        lab = FlumeLab()
        with lab.experiment(checkpoint_file=str(tmp_path / "cp.json")):
            lab.checkpoint.mark_complete(0, 1.0, 2.0)
        assert CheckpointStore(str(tmp_path / "cp.json"), resume=True).is_complete(0)


# ---------------------------------------------------------------------------
# Schedule loading
# ---------------------------------------------------------------------------


def _schedule(csv_text):
    return ExperimentSchedule.from_dataframe(pd.read_csv(io.StringIO(csv_text)))


class TestScheduleValidation:
    def test_out_of_order_times_are_refused(self):
        """np.interp silently mis-read these: 10 L/min came out as 15."""
        with pytest.raises(ValueError, match="strictly increasing"):
            _schedule("time_s,pump_flow_lpm\n0,0\n120,30\n60,10\n")

    def test_repeated_times_are_refused(self):
        with pytest.raises(ValueError, match="strictly increasing"):
            _schedule("time_s,pump_flow_lpm\n0,0\n60,10\n60,20\n")

    def test_a_blank_setpoint_is_refused(self):
        """A blank became NaN and went straight to set_flowrate()."""
        with pytest.raises(ValueError, match="pump_flow_lpm.*blank"):
            _schedule("time_s,pump_flow_lpm\n0,0\n60,\n120,30\n")

    def test_a_blank_valve_cell_is_refused(self):
        """bool(NaN) is True — a blank valve cell opened the valve."""
        with pytest.raises(ValueError, match="qin_open.*blank"):
            _schedule("time_s,qin_open\n0,False\n60,\n120,True\n")

    def test_a_well_formed_schedule_still_loads(self):
        s = _schedule("time_s,pump_flow_lpm,qin_open\n0,0,False\n60,10,True\n120,30,True\n")
        assert s.pump_flow(60) == pytest.approx(10.0)
        assert bool(s.qin_open(60)) is True
