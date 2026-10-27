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
