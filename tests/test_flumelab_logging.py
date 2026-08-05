"""Tests for FlumeLab's two-tier logging: log_note() (archival), the global
debug switch, and the operational log file (see laguna.subsystem_logging's
module docstring for the tier split this all implements).
"""

import csv

from laguna import FlumeLab


class TestLogNote:
    def _lab(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(f"timing:\n  event_log: {tmp_path / 'events.csv'}\n")
        return FlumeLab(str(path))

    def test_writes_an_operator_note_row(self, tmp_path):
        lab = self._lab(tmp_path)
        lab.log_note("operator restarted the pump after a fault")
        with open(tmp_path / "events.csv", newline="") as f:
            rows = list(csv.reader(f))
        assert rows[1][4:8] == [
            "operator", "note", "ok", "operator restarted the pump after a fault",
        ]

    def test_returns_its_own_event_id(self, tmp_path):
        lab = self._lab(tmp_path)
        assert lab.log_note("first note") == 1
        assert lab.log_note("second note") == 2

    def test_refers_to_a_prior_event(self, tmp_path):
        lab = self._lab(tmp_path)
        scan_id = lab.event_log.log(lab.clock.elapsed(), "gocator", "scan", result="error: timeout")
        lab.log_note("sensor cable was loose", refers_to=scan_id)
        with open(tmp_path / "events.csv", newline="") as f:
            rows = list(csv.reader(f))
        assert rows[2][-1] == str(scan_id)
