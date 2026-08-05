"""Tests for offline rehearsal.

The value of a rehearsal is that it exercises the real scheduler, clock,
frames and safety machinery — so these check the seams where "simulated"
could quietly stop meaning anything.
"""

import csv

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

    def test_gantry_and_gocator_are_kept(self):
        """These have real simulated backends — a rehearsal must exercise
        them, or it proves nothing about the schedule."""
        cfg = {"gantry": {}, "gocator": {}}
        assert set(simulate_config(cfg)) == {"gantry", "gocator"}

    def test_every_registry_subsystem_is_marked_simulated_not_dropped(self):
        """Every laguna.registry.SUBSYSTEM_REGISTRY entry has a simulated
        backend now (SimulatedTeknicMotor/VFD/MassaSensor, and a
        `simulated` flag pi_cameras/dslr_cameras/od2000/wtt12l check
        directly) — a rehearsal must exercise all of them, or it proves
        nothing about their schedules."""
        cfg = {
            "gantry": {}, "gocator": {}, "weir": {}, "flow": {}, "gauge": {},
            "pi_cameras": {}, "dslr_cameras": {}, "od2000": {}, "wtt12l": {},
        }
        out = simulate_config(cfg)
        assert set(out) == set(cfg)
        for section in ("weir", "flow", "gauge", "pi_cameras", "dslr_cameras",
                        "od2000", "wtt12l"):
            assert out[section]["simulated"] is True

    def test_unknown_sections_with_no_simulated_backend_are_dropped(self, monkeypatch):
        """_NO_SIMULATED_BACKEND is empty today (every registered subsystem
        has a simulated path), but the drop mechanism itself must still
        work for whatever gets added to the registry next without one —
        exercised here with a synthetic section name."""
        import laguna.simulation as simulation

        monkeypatch.setattr(simulation, "_NO_SIMULATED_BACKEND", ("not_yet_simulated",))
        cfg = {"gantry": {}, "not_yet_simulated": {"port": "/dev/ttyUSB9"}}
        assert set(simulate_config(cfg)) == {"gantry"}

    def test_the_original_config_is_not_mutated(self):
        cfg = {"gantry": {"transport": "pi_agent"}}
        simulate_config(cfg)
        assert cfg["gantry"]["transport"] == "pi_agent"


class TestSetupRunSimulation:
    """setup_run() builds subsystems via lab.add_all(), which keys off
    lab.config.explicit_sections — simulate=True has to reach that (see
    FlumeLab.__init__'s explicit_sections intersection with the
    simulate_config()-rewritten config_dict), or a section dropped there
    would still get built here for real."""

    def _config_path(self, tmp_path):
        import yaml

        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({
            "gantry": {"transport": "pi_agent", "safe_mode": True,
                       "axes": [{"name": "X", "index": 1}, {"name": "Y", "index": 2}]},
            "gocator": {"ip": "192.168.1.10"},
            "weir": {"port": "/dev/ttyUSB0"},
            "flow": {"vfd_port": "/dev/ttyUSB1"},
        }))
        return str(path)

    def test_weir_and_flow_are_now_constructed_and_connect_under_simulate(
        self, tmp_path
    ):
        """weir/flow have simulated backends now (SimulatedTeknicMotor/VFD) —
        constructed and successfully connected under simulate=True even
        though safl_ocean_hardware isn't installed in this environment,
        which is exactly the point: no real driver is needed to rehearse."""
        from laguna.experiment.runner import setup_run

        lab = setup_run(self._config_path(tmp_path), simulate=True)

        assert "weir" in lab._subsystems
        assert "flow" in lab._subsystems
        assert lab.weir._is_connected is True
        assert lab.flow._is_connected is True

    def test_every_registry_subsystem_connects_end_to_end(self, tmp_path):
        """The full point of extending simulate=True past gantry/gocator:
        a rehearsal should exercise weir/flow/gauge/camera/rangefinder
        schedules too, not just motion — every subsystem in
        laguna.registry.SUBSYSTEM_REGISTRY connects successfully with no
        real hardware anywhere."""
        import math

        import yaml

        from laguna.experiment.runner import setup_run
        from laguna.registry import SUBSYSTEM_REGISTRY

        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({
            "gantry": {"transport": "pi_agent", "safe_mode": True,
                       "axes": [{"name": "X", "index": 1}]},
            "gocator": {"ip": "192.168.1.10"},
            "weir": {"port": "/dev/ttyUSB0"},
            "flow": {"vfd_port": "/dev/ttyUSB1"},
            "gauge": {"port": "/dev/ttyUSB2"},
            "pi_cameras": {"hosts": ["pi1.local"]},
            "dslr_cameras": {"cameras": {"Camera1": {}}},
            "od2000": {"topic": "laguna/od2000", "pdin_port": 2},
            "wtt12l": {"topic": "laguna/wtt12l", "pdin_port": 7},
        }))

        lab = setup_run(str(path), simulate=True)

        assert set(lab._subsystems) == set(SUBSYSTEM_REGISTRY)
        # connect_all() already ran inside setup_run(); every subsystem
        # must have actually connected, not just been constructed.
        for name, subsystem in lab._subsystems.items():
            status = subsystem.get_status()
            assert status.get("is_connected", True) is True, f"{name} did not connect"

        # Readings are NaN, not fabricated — commands (already exercised by
        # the other per-subsystem simulated tests) are what a rehearsal is
        # actually checking here.
        assert math.isnan(lab.gauge.read_mm())
        assert math.isnan(lab.weir.get_elevation())
        assert math.isnan(lab.od2000.get_distance_mm())
        assert math.isnan(lab.wtt12l.get_distance_mm())

    def test_gantry_still_gets_simulated_config_through_setup_run(self, tmp_path):
        from laguna.experiment.runner import setup_run

        lab = setup_run(self._config_path(tmp_path), simulate=True)
        assert lab.gantry.connect() is True  # succeeds with no hardware present

    def test_speed_factor_requires_simulate(self, tmp_path):
        from laguna.experiment.runner import setup_run

        with pytest.raises(ValueError, match="simulate=True"):
            setup_run(self._config_path(tmp_path), speed_factor=10.0)


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
        lab.add(GantryController.from_config(lab.config))
        assert lab.connect_all() is True

        lab.gantry.move_to(X=150.0)
        assert lab.gantry.get_status()["positions"]["X"] == pytest.approx(150.0)

    def test_event_log_gets_a_simulated_suffix_by_default(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(f"timing:\n  event_log: {tmp_path / 'experiment_events.csv'}\n")
        lab = FlumeLab(str(path), simulate=True)
        assert lab.event_log._path == tmp_path / "experiment_events_simulated.csv"
        assert not (tmp_path / "experiment_events.csv").exists()

    def test_explicit_event_log_path_is_still_suffixed(self, tmp_path):
        """A rehearsal must never be able to land in the same file as a real
        run's — even if timing.event_log was set explicitly, since the same
        config is often reused for both a real run and its rehearsal."""
        path = tmp_path / "cfg.yaml"
        path.write_text(f"timing:\n  event_log: {tmp_path / 'my_events.csv'}\n")
        lab = FlumeLab(str(path), simulate=True)
        assert lab.event_log._path == tmp_path / "my_events_simulated.csv"

    def test_real_run_does_not_get_the_suffix(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(f"timing:\n  event_log: {tmp_path / 'experiment_events.csv'}\n")
        lab = FlumeLab(str(path))
        assert lab.event_log._path == tmp_path / "experiment_events.csv"

    def test_simulate_mode_row_written_at_construction(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(f"timing:\n  event_log: {tmp_path / 'events.csv'}\n")
        FlumeLab(str(path), simulate=True)
        with open(tmp_path / "events_simulated.csv", newline="") as f:
            rows = list(csv.DictReader(f))
        marker_rows = [r for r in rows if r["event_type"] == "simulate_mode"]
        assert len(marker_rows) == 1
        assert marker_rows[0]["subsystem"] == "flume_lab"

    def test_real_run_has_no_simulate_mode_row(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(f"timing:\n  event_log: {tmp_path / 'events.csv'}\n")
        FlumeLab(str(path))
        with open(tmp_path / "events.csv", newline="") as f:
            rows = list(csv.DictReader(f))
        assert [r for r in rows if r["event_type"] == "simulate_mode"] == []


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

    def test_surface_size_is_fixed_and_small_regardless_of_scan_config(self):
        """A rehearsal must not accumulate large files even if the real
        config asks for a big fixed_length_mm/high frame rate — the
        simulated surface is always the same small fixed grid (see
        laguna.scanner.simulation's SIM_ROWS/SIM_COLS), not scaled to
        whatever a real scan would have produced."""
        from laguna.scanner import GocatorScanner
        from laguna.scanner.simulation import SIM_COLS, SIM_ROWS

        big_scanner = GocatorScanner({
            "ip": "1.2.3.4", "simulated": True,
            "travel_speed_mm_s": 20.0, "fixed_length_mm": 5000.0,  # a large real pass
        })
        big_scanner.connect()
        big_scanner.start()
        scan = big_scanner.receive_surface(timeout_s=1.0)

        assert scan.shape == (SIM_ROWS, SIM_COLS)
        # float64 x/y/z per point, well under a megabyte either way —
        # nowhere near the GB range a long real scan can reach.
        assert scan.z_mm.nbytes < 1_000_000

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


class TestClockAcceleration:
    """A rehearsal is only useful if it finishes sooner than the run it
    rehearses — but only if it still fires the same events in the same order.
    """

    def test_speed_factor_scales_runtime(self):
        import time

        from laguna.timing import ExperimentClock

        clock = ExperimentClock(speed_factor=100.0)
        clock.start()
        time.sleep(0.05)
        # 0.05 real seconds at 100x is ~5 experiment seconds.
        assert clock.elapsed() == pytest.approx(5.0, rel=0.35)

    def test_default_is_real_time(self):
        from laguna.timing import ExperimentClock

        assert ExperimentClock().speed_factor == 1.0

    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_non_positive_speed_rejected(self, bad):
        from laguna.timing import ExperimentClock

        with pytest.raises(ValueError, match="must be positive"):
            ExperimentClock(speed_factor=bad)

    def test_pause_still_freezes_an_accelerated_clock(self):
        import time

        from laguna.timing import ExperimentClock

        clock = ExperimentClock(speed_factor=50.0)
        clock.start()
        time.sleep(0.02)
        clock.pause()
        during = clock.elapsed()
        time.sleep(0.05)
        assert clock.elapsed() == pytest.approx(during, rel=0.01)

    def test_acceleration_needs_simulate(self):
        """Real hardware cannot be sped up — a gantry takes as long as it
        takes — so accelerating a live run would just make the schedule
        outrun the machine."""
        with pytest.raises(ValueError, match="only applies to simulate"):
            FlumeLab(simulate=False, speed_factor=10.0)

    def test_the_same_events_fire_as_in_real_time(self):
        """The property that makes acceleration trustworthy: it changes how
        fast, not what happens."""
        import time

        def run(simulate, speed):
            lab = FlumeLab(simulate=simulate, speed_factor=speed)
            seen = []
            lab.scheduler.repeat(every=1, subsystem="t",
                                 action=lambda: seen.append(round(lab.clock.elapsed())))
            lab.scheduler.at(runtime_s=2, subsystem="t",
                             action=lambda: seen.append("oneshot"))
            lab.clock.start()
            started = time.time()
            lab.scheduler.run(duration=3.0)
            return seen, time.time() - started

        # Real time as the reference, then the same schedule accelerated.
        slow, slow_wall = run(True, 1.0)
        fast, fast_wall = run(True, 20.0)
        assert fast == slow, "acceleration changed the event sequence"
        assert fast_wall < slow_wall / 5

    def test_recurring_events_do_not_drift(self):
        """_next used to rebase on the observed time, so every firing drifted
        late by however far the poll overshot, and the error accumulated."""
        lab = FlumeLab(simulate=True, speed_factor=200.0)
        seen = []
        lab.scheduler.repeat(every=10, subsystem="t",
                             action=lambda: seen.append(lab.clock.elapsed()))
        lab.clock.start()
        lab.scheduler.run(duration=100.0)
        assert len(seen) >= 5
        for i, t in enumerate(seen, start=1):
            assert t == pytest.approx(i * 10, abs=1.5), f"firing {i} drifted to {t}"

    def test_a_fast_clock_does_not_swallow_firings(self):
        """At high speed the clock advances several intervals per poll. Firing
        once per poll would silently under-report what the real run does."""
        lab = FlumeLab(simulate=True, speed_factor=200.0)
        seen = []
        lab.scheduler.repeat(every=1, subsystem="t", action=lambda: seen.append(1))
        lab.clock.start()
        lab.scheduler.run(duration=50.0)
        # 50 experiment-seconds at 1s intervals: expect ~49-50, not ~5.
        assert len(seen) >= 40, f"only {len(seen)} firings; the backlog was dropped"

    def test_the_manifest_stays_truthful_when_accelerated(self):
        """An accelerated run's file timestamps must still convert to the
        right experiment runtime, or the manifest lies about the rehearsal."""
        import time

        lab = FlumeLab(simulate=True, speed_factor=40.0)
        lab.clock.start()
        lab.run.started()
        time.sleep(0.05)
        wall, runtime = lab.clock.now()
        assert lab.run.runtime_at(wall) == pytest.approx(runtime, rel=0.05)

    def test_speed_factor_is_recorded_in_the_manifest(self, tmp_path):
        """A file written during a rehearsal is otherwise indistinguishable
        from a real run's, and its timestamps would convert wrongly."""
        from laguna.run_context import RunContext

        ctx = RunContext(root=str(tmp_path), run_id="R", speed_factor=25.0)
        ctx.started()
        assert RunContext.load(str(tmp_path / "R")).speed_factor == 25.0
