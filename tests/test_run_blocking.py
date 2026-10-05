"""run_blocking(): a pause from anywhere stays a pause, not a disconnect."""

from __future__ import annotations

import signal
import threading
import time

import pytest

from laguna import FlumeLab
from laguna.experiment import runner
from laguna.safety import SafetyState


def _wait(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def captured_signals(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    handlers = {}
    monkeypatch.setattr(signal, "signal", lambda sig, fn: handlers.__setitem__(sig, fn))
    return handlers


def test_an_escalated_pause_waits_for_resume_instead_of_disconnecting(captured_signals):
    """Regression: only a pause run_blocking itself started counted as a
    pause; one escalated from inside the run (a failed scan, a PAUSE file)
    fell through to break and disconnect_all() — unrecoverable."""
    lab = FlumeLab()
    disconnected = threading.Event()
    lab.disconnect_all = disconnected.set
    done = threading.Event()

    def drive():
        runner.run_blocking(lab, duration=60)
        done.set()

    t = threading.Thread(target=drive, daemon=True)
    t.start()
    assert _wait(lambda: lab.scheduler.is_running)

    lab.escalate("scan failed")
    assert _wait(lambda: not lab.scheduler.is_running)
    time.sleep(0.6)  # longer than run_blocking's join poll
    assert not disconnected.is_set(), "an escalated pause tore the run down"
    assert lab.safety_state is SafetyState.PAUSED

    captured_signals[signal.SIGUSR2](signal.SIGUSR2, None)  # kill -USR2: resume
    assert _wait(lambda: lab.safety_state is SafetyState.RUNNING and lab.scheduler.is_running)

    lab.end_run()
    assert done.wait(3)
    assert disconnected.is_set()


def test_a_stray_resume_signal_does_not_cancel_the_next_pause(captured_signals):
    lab = FlumeLab()
    lab.disconnect_all = lambda: None
    t = threading.Thread(target=runner.run_blocking, args=(lab, 60), daemon=True)
    t.start()
    assert _wait(lambda: lab.scheduler.is_running)

    captured_signals[signal.SIGUSR2](signal.SIGUSR2, None)  # stray, nothing paused
    captured_signals[signal.SIGUSR1](signal.SIGUSR1, None)  # now pause
    time.sleep(0.8)
    assert lab.safety_state is SafetyState.PAUSED, "the stray SIGUSR2 resumed the new pause"
    lab.end_run()
    t.join(3)


def test_a_blank_schedule_flag_is_refused_not_read_as_true(tmp_path):
    """astype(bool) read a blank cell as True: a spurious scan at every blank row."""
    import io

    import pandas as pd

    from laguna.schedule import ExperimentSchedule

    lab = FlumeLab()
    schedule = ExperimentSchedule.from_dataframe(
        pd.read_csv(io.StringIO("time_s,gocator\n0,True\n60,\n120,False\n"))
    )
    with pytest.raises(ValueError, match="gocator.*blank"):
        runner.schedule_action(
            lab, {"use_schedule": True}, "gocator", "scan",
            action=lambda: None, exp_schedule=schedule, schedule_col="gocator",
        )
