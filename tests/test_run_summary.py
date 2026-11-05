"""print_summary() describes the current run only, and says where it stands."""

import time

import pytest

from laguna.core import FlumeLab
from laguna.safety import SafetyState


@pytest.fixture
def lab(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    cfg = tmp_path / "config.yaml"
    cfg.write_text(
        f"timing:\n  event_log: {tmp_path / 'events.csv'}\n  run_dir: {tmp_path}\n"
        f"  checkpoint_file: {tmp_path / 'ckpt.json'}\n"
    )
    return FlumeLab(config_file=str(cfg))


def _summary(lab, capsys):
    lab.print_summary()
    return capsys.readouterr().out


def test_header_reflects_state(lab, capsys):
    assert "No Run Started" in _summary(lab, capsys)
    lab._start_wall = time.time()
    assert "Run In Progress" in _summary(lab, capsys)
    lab._safety_state = SafetyState.PAUSED
    assert "Run Paused" in _summary(lab, capsys)
    lab._safety_state = SafetyState.STOPPED
    out = _summary(lab, capsys)
    assert "Run Ended" in out and "Experiment Complete" not in out
    lab._safety_state = SafetyState.ESTOPPED
    assert "Run E-Stopped" in _summary(lab, capsys)


def test_counts_only_cover_this_run(lab, capsys):
    for _ in range(3):
        lab.event_log.log(0.0, "weir", "go_to_elevation")
    time.sleep(0.02)
    lab._start_wall = time.time()
    lab.event_log.log(1.0, "weir", "go_to_elevation")
    out = _summary(lab, capsys)
    line = next(l for l in out.splitlines() if "go_to_elevation" in l)
    assert line.split()[-1] == "1×"


def test_captures_listed_from_notes(lab, capsys):
    lab._start_wall = time.time()
    lab.event_log.log(
        1.0, "pi_cameras", "capture", notes="host=sirius.lab file=/data/pi/a.jpg"
    )
    out = _summary(lab, capsys)
    assert "pi_cameras" in out and "1 images" in out and "/data/pi/" in out


def test_runtime_shows_duration(lab, capsys):
    lab._start_wall = time.time()
    lab._duration = 300
    assert "of 300 s" in _summary(lab, capsys)



def test_completion_ends_the_run(lab, capsys):
    lab._start_wall = time.time()
    lab._on_run_complete()
    assert lab.safety_state is SafetyState.STOPPED
    assert "Run Ended" in capsys.readouterr().out


def test_completion_through_a_real_scheduler_ends_the_run(lab, capsys):
    thread = lab.start(0.3)
    thread.join(timeout=5)
    assert not thread.is_alive()
    assert lab.safety_state is SafetyState.STOPPED
    assert "Run Ended" in capsys.readouterr().out


def test_resumed_run_ends_itself_too(lab, capsys):
    thread = lab.start(1.0)
    time.sleep(0.2)
    lab.pause()
    thread.join(timeout=5)
    capsys.readouterr()
    resumed = lab.resume()
    assert resumed is not None
    resumed.join(timeout=10)
    assert not resumed.is_alive()
    assert lab.safety_state is SafetyState.STOPPED
    assert "Run Ended" in capsys.readouterr().out


def test_failed_end_run_escalates_to_estop_and_still_prints(lab, capsys, monkeypatch):
    lab._start_wall = time.time()

    def boom(reason="manual"):
        raise RuntimeError("stop blew up")

    monkeypatch.setattr(lab, "end_run", boom)
    lab._on_run_complete()  # must not raise out of the scheduler thread
    assert lab.safety_state is SafetyState.ESTOPPED
    assert "Run E-Stopped" in capsys.readouterr().out


def test_summary_survives_an_unknown_safety_state(lab, capsys):
    class _Odd:
        name = "ODD"
        severity = 99

    lab._start_wall = time.time()
    lab._safety_state = _Odd()
    assert "Run Odd" in _summary(lab, capsys)


def test_natural_end_waits_for_an_in_flight_scan_before_stopping_subsystems(lab):
    events = []

    class Scanner:
        subsystem_name = "scanner"

        def stop(self):
            events.append("stop")

        def get_status(self):
            return {}

    lab.add(Scanner())
    lab.scheduler.at(0.1, lambda: (time.sleep(0.8), events.append("scan_done")), subsystem="scanner", name="scan")
    thread = lab.start(0.3)
    thread.join(timeout=10)
    assert events == ["scan_done", "stop"]
    assert lab.safety_state is SafetyState.STOPPED
