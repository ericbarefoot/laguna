"""Tests for setup_run()'s log_as_event opt-in: passive status-polling
closures (gauge/weir) only write to the archival event log when their
config section explicitly asks for it — a reading measures the
experiment's state without changing it, so by default it's operational-log
detail, not an archival milestone (see laguna.subsystem_logging's module
docstring).
"""

import csv

import yaml

from laguna.experiment.runner import setup_run


class _FakeGauge:
    subsystem_name = "gauge"
    _is_connected = True

    def connect(self):
        return True

    def read_mm(self):
        return 42.0


class _FakeWeir:
    subsystem_name = "weir"

    def connect(self):
        return True

    def get_status(self):
        return {"is_connected": True, "elevation_mm": 12.5, "motor": {}}


def _config_path(tmp_path, section_name, section_cfg):
    cfg = {section_name: section_cfg, "timing": {"event_log": str(tmp_path / "events.csv")}}
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return str(path)


def _fire_registered_action(lab, subsystem_name):
    action = next(e["action"] for e in lab.scheduler._recurring if e["subsystem"] == subsystem_name)
    action()


def _event_rows(tmp_path, subsystem, event_type):
    with open(tmp_path / "events.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    return [r for r in rows if r["subsystem"] == subsystem and r["event_type"] == event_type]


class TestGaugeLogAsEvent:
    def test_default_does_not_archive_the_reading(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "laguna.gauge.sensor.SaflWaterLevelSensor.from_config",
            classmethod(lambda cls, config: _FakeGauge()),
        )
        lab = setup_run(_config_path(tmp_path, "gauge", {"interval_s": 5}))
        _fire_registered_action(lab, "gauge")
        assert _event_rows(tmp_path, "gauge", "read_mm") == []

    def test_log_as_event_true_archives_the_reading(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "laguna.gauge.sensor.SaflWaterLevelSensor.from_config",
            classmethod(lambda cls, config: _FakeGauge()),
        )
        lab = setup_run(_config_path(tmp_path, "gauge", {"interval_s": 3600, "log_as_event": True}))
        _fire_registered_action(lab, "gauge")
        rows = _event_rows(tmp_path, "gauge", "read_mm")
        assert len(rows) == 1
        assert "elevation_mm=42.00" in rows[0]["notes"]


class TestWeirStatusLogAsEvent:
    def test_default_does_not_archive_the_status_poll(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "laguna.weir.controller.SaflWeirController.from_config",
            classmethod(lambda cls, config: _FakeWeir()),
        )
        lab = setup_run(_config_path(tmp_path, "weir", {"interval_s": 20}))
        _fire_registered_action(lab, "weir")
        assert _event_rows(tmp_path, "weir", "get_status") == []

    def test_log_as_event_true_archives_the_status_poll(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "laguna.weir.controller.SaflWeirController.from_config",
            classmethod(lambda cls, config: _FakeWeir()),
        )
        lab = setup_run(_config_path(tmp_path, "weir", {"interval_s": 3600, "log_as_event": True}))
        _fire_registered_action(lab, "weir")
        assert len(_event_rows(tmp_path, "weir", "get_status")) == 1
