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


class _Flow:
    """Minimal flow subsystem recording what the completion callback does."""

    def __init__(self, stop_note=None):
        self.stop_note = stop_note
        self.calls = []
        self.qin = True
        self.qaux = False
        self.pump_stops = 0

    def stop(self):
        self.pump_stops += 1
        return self.stop_note


def test_completion_ends_run_and_closes_open_valves(lab, capsys):
    flow = _Flow()
    lab._subsystems["flow"] = flow
    lab._start_wall = time.time()
    lab._on_run_complete()
    assert lab.safety_state is SafetyState.STOPPED
    assert flow.pump_stops >= 1
    assert flow.qin is False and flow.qaux is False
    assert "Run Ended" in capsys.readouterr().out


def test_completion_leaves_valves_if_pump_not_confirmed_stopped(lab, capsys):
    flow = _Flow(stop_note="pump may still be running")
    lab._subsystems["flow"] = flow
    lab._start_wall = time.time()
    lab._on_run_complete()
    assert flow.qin is True


def test_completion_closes_other_valve_if_one_fails(lab):
    class Bad(_Flow):
        def __init__(self):
            self.stop_note = None
            self.qaux = True
            self.pump_stops = 0

        @property
        def qin(self):
            return True

        @qin.setter
        def qin(self, v):
            raise RuntimeError("no ack")

    flow = Bad()
    lab._subsystems["flow"] = flow
    lab._start_wall = time.time()
    lab._on_run_complete()
    assert flow.qaux is False
