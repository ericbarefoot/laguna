"""Tests for offline rehearsal.

The value of a rehearsal is that it exercises the real scheduler, clock,
frames and safety machinery — so these check the seams where "simulated"
could quietly stop meaning anything.
"""

import pytest

from laguna import FlumeLab
from laguna.simulation import SimulatedSnapConnection, simulate_config


class TestSimulatedTransport:
    def test_a_move_is_readable_afterwards(self):
        """Not a scripted lookup table: a move followed by a position read
        must return what the move asked for, or a rehearsal proves nothing."""
        conn = SimulatedSnapConnection()
        conn.connect()
        conn.send("A1 BMT 150")
        assert float(conn.send("A1 ACP")) == pytest.approx(150.0)

    def test_relative_moves_accumulate(self):
        conn = SimulatedSnapConnection()
        conn.send("A1 BMB 10")
        conn.send("A1 BMB 5")
        assert float(conn.send("A1 ACP")) == pytest.approx(15.0)

    def test_a_group_move_drives_its_member_axes(self):
        """C1 addresses X and Y together. Without this a rehearsed move looked
        like it did nothing, hiding exactly the mistakes rehearsal exists to
        catch."""
        conn = SimulatedSnapConnection()
        conn.send("C1 BMT 100 200")
        assert float(conn.send("A1 ACP")) == pytest.approx(100.0)
        assert float(conn.send("A2 ACP")) == pytest.approx(200.0)

    def test_position_can_be_redefined(self):
        conn = SimulatedSnapConnection()
        conn.send("A1 ACP 42")
        assert float(conn.send("A1 ACP")) == pytest.approx(42.0)

    def test_moves_report_finished(self):
        assert SimulatedSnapConnection().send("A1 MIF") == "1"

    def test_speed_round_trips(self):
        conn = SimulatedSnapConnection()
        conn.send("A1 SPD 12.5")
        assert float(conn.send("A1 SPD")) == pytest.approx(12.5)

    def test_unknown_commands_do_not_raise(self):
        """A rehearsal must not fall over on a command the model doesn't
        model — that would make it less useful the more code it exercises."""
        assert SimulatedSnapConnection().send("A1 ZZZ 1") == "0"
        assert SimulatedSnapConnection().send("garbage") == "0"

    def test_records_what_was_sent(self):
        conn = SimulatedSnapConnection()
        conn.send("A1 BMT 5")
        assert conn.sent == ["A1 BMT 5"]


class TestSimulateConfig:
    def test_gantry_transport_is_swapped(self):
        out = simulate_config({"gantry": {"transport": "pi_agent", "host": "red.lab"}})
        assert out["gantry"]["transport"] == "simulated"

    def test_safe_mode_is_lifted(self):
        """Otherwise every move is refused and the rehearsal only proves the
        safety gate works."""
        out = simulate_config({"gantry": {"safe_mode": True}})
        assert out["gantry"]["safe_mode"] is False

    def test_gocator_is_marked_simulated(self):
        assert simulate_config({"gocator": {"ip": "1.2.3.4"}})["gocator"]["simulated"] is True

    def test_subsystems_are_not_removed(self):
        """A rehearsal must exercise the same set the real run would, or it
        proves nothing about the schedule."""
        cfg = {"gantry": {}, "gocator": {}, "weir": {}, "flow": {}}
        assert set(simulate_config(cfg)) == set(cfg)

    def test_the_original_config_is_not_mutated(self):
        cfg = {"gantry": {"transport": "pi_agent"}}
        simulate_config(cfg)
        assert cfg["gantry"]["transport"] == "pi_agent"


class TestFlumeLabSimulation:
    def test_flag_is_off_by_default(self):
        assert FlumeLab().simulate is False

    def test_flag_rewrites_the_config(self, tmp_path):
        import yaml

        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({
            "gantry": {"transport": "pi_agent", "safe_mode": True,
                       "axes": [{"name": "X", "index": 1}]},
        }))
        lab = FlumeLab(str(path), simulate=True)
        assert lab.config.get("gantry")["transport"] == "simulated"

    def test_a_gantry_connects_and_moves_with_no_hardware(self, tmp_path):
        import yaml

        from laguna.robot.macron.controller import GantryController

        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({
            "gantry": {
                "transport": "pi_agent", "safe_mode": True, "mm_per_acp_unit": 15.0,
                "axes": [{"name": "X", "index": 1}, {"name": "Y", "index": 2}],
            },
        }))
        lab = FlumeLab(str(path), simulate=True)
        lab.add(GantryController.from_config(lab.config.get("gantry")))
        assert lab.connect_all() is True

        lab.gantry.move_to(X=150.0)
        assert lab.gantry.get_status()["positions"]["X"] == pytest.approx(150.0)


class TestSimulatedScanner:
    def _scanner(self):
        from laguna.scanner import GocatorScanner

        s = GocatorScanner({"ip": "1.2.3.4", "simulated": True,
                            "travel_speed_mm_s": 20.0, "fixed_length_mm": 200.0})
        assert s.connect() is True
        return s

    def test_connects_without_the_sdk_or_a_sensor(self):
        self._scanner()

    def test_returns_a_surface_of_the_right_shape(self):
        from laguna.scanner.simulation import SIM_COLS, SIM_ROWS

        scanner = self._scanner()
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        assert scan.shape == (SIM_ROWS, SIM_COLS)

    def test_no_data_cells_become_nan(self):
        """The synthetic surface carries the real 0x8000 sentinel, so the
        NaN path is genuinely exercised rather than assumed."""
        import numpy as np

        scanner = self._scanner()
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        assert np.isnan(scan.z_mm).any()
        assert scan.valid_count < scan.z_mm.size

    def test_export_paths_work_end_to_end(self, tmp_path):
        scanner = self._scanner()
        scanner._output_dir = tmp_path
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        written = scanner.save_scan(scan, formats=("npz",))
        assert written["npz"].exists()

    def test_a_normal_scan_does_not_report_a_discard(self):
        """Regression: the safety-verb unification made scan cleanup call
        stop(), so every successful scan logged a DISCARDED note — which
        would have drowned the real ones in the event log."""
        scanner = self._scanner()
        scanner.start()
        scanner.receive_surface(timeout_s=1.0)
        scanner._end_acquisition()
        assert scanner.pause() is None, "a completed scan reported a discard"

    def test_an_interrupted_scan_still_reports_one(self):
        scanner = self._scanner()
        scanner.start()
        note = scanner.pause()
        assert note and "DISCARD" in note.upper()
