"""Tests for EventLog: event_id/refers_to columns and resume-safe ID continuation."""

import csv

from laguna.timing.event_log import EventLog


class TestEventId:
    def test_ids_are_monotonic_starting_at_1(self, tmp_path):
        log = EventLog(str(tmp_path / "events.csv"))
        first = log.log(0.0, "weir", "go_to_elevation")
        second = log.log(1.0, "flow", "start")
        assert (first, second) == (1, 2)

    def test_log_returns_the_assigned_id(self, tmp_path):
        log = EventLog(str(tmp_path / "events.csv"))
        assert log.log(0.0, "gantry", "move_to") == 1

    def test_refers_to_defaults_to_empty(self, tmp_path):
        path = tmp_path / "events.csv"
        log = EventLog(str(path))
        log.log(0.0, "gauge", "read_mm")
        with open(path, newline="") as f:
            rows = list(csv.reader(f))
        assert rows[1][-1] == ""

    def test_refers_to_records_the_referenced_id(self, tmp_path):
        path = tmp_path / "events.csv"
        log = EventLog(str(path))
        scan_id = log.log(0.0, "gocator", "scan", result="error: timeout")
        log.log(1.0, "operator", "note", notes="sensor was unplugged", refers_to=scan_id)
        with open(path, newline="") as f:
            rows = list(csv.reader(f))
        assert rows[2][-1] == str(scan_id)


class TestResumeContinuesEventId:
    def test_reopening_an_existing_log_continues_the_id_sequence(self, tmp_path):
        path = tmp_path / "events.csv"
        first_log = EventLog(str(path))
        first_log.log(0.0, "weir", "go_to_elevation")
        first_log.log(1.0, "flow", "start")
        first_log.close()

        resumed_log = EventLog(str(path))
        third_id = resumed_log.log(2.0, "gauge", "read_mm")
        assert third_id == 3

    def test_header_is_not_rewritten_on_reopen(self, tmp_path):
        path = tmp_path / "events.csv"
        EventLog(str(path)).log(0.0, "weir", "go_to_elevation")
        EventLog(str(path)).log(1.0, "flow", "start")
        with open(path, newline="") as f:
            rows = list(csv.reader(f))
        assert rows[0][0] == "event_id"
        assert len(rows) == 3  # header + 2 rows, not 2 headers + 2 rows

    def test_survives_a_malformed_trailing_row(self, tmp_path):
        """A crash mid-write could leave a partial last line — resuming
        must fall back to the last well-formed row's event_id, not choke
        or restart at 1 and collide with rows already in the file."""
        path = tmp_path / "events.csv"
        log = EventLog(str(path))
        log.log(0.0, "weir", "go_to_elevation")
        log.log(1.0, "flow", "start")
        log.close()
        with open(path, "a") as f:
            f.write("garbled,partial,row\n")

        resumed = EventLog(str(path))
        assert resumed.log(2.0, "gauge", "read_mm") == 3


class TestCrashRecovery:
    def test_a_partial_last_row_is_terminated_before_appending(self, tmp_path):
        """A crash mid-row used to leave the next row glued onto it, losing both."""
        import csv

        path = tmp_path / "events.csv"
        log = EventLog(str(path))
        log.log(1.0, "weir", "move")
        log.close()
        with open(path, "a") as f:
            f.write("2,2026-01-01T00:00:00+00:00,0,2.0,gauge,read")  # no newline: crash
        log = EventLog(str(path))
        log.log(3.0, "flow", "start")
        log.close()
        rows = list(csv.reader(open(path, newline="")))
        assert rows[-1][4:6] == ["flow", "start"]

    def test_an_empty_existing_file_gets_its_header(self, tmp_path):
        path = tmp_path / "events.csv"
        path.write_text("")
        log = EventLog(str(path))
        log.log(1.0, "weir", "move")
        log.close()
        assert path.read_text().splitlines()[0].startswith("event_id,")

    def test_a_row_logged_after_close_is_kept(self, tmp_path):
        """Late rows (a scheduled action finishing after experiment() closed
        the log) used to raise in that thread and vanish."""
        path = tmp_path / "events.csv"
        log = EventLog(str(path))
        log.close()
        log.log(9.0, "gocator", "scan", result="points=10")
        log.close()
        assert "gocator,scan,points=10" in path.read_text()
