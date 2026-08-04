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

    def test_gantry_and_gocator_are_kept(self):
        """These have real simulated backends — a rehearsal must exercise
        them, or it proves nothing about the schedule."""
        cfg = {"gantry": {}, "gocator": {}}
        assert set(simulate_config(cfg)) == {"gantry", "gocator"}

    def test_subsystems_with_no_simulated_backend_are_dropped(self):
        """weir/flow/gauge/cameras/rangefinders have no simulated transport —
        a Modbus VFD, a serial stepper, an ultrasonic sensor, SSH to a Pi.
        Constructing the real controller classes for these under
        simulate=True would silently contact real hardware during what is
        supposed to be a hardware-free rehearsal, so they must be dropped
        rather than passed through untouched."""
        cfg = {
            "gantry": {}, "gocator": {}, "weir": {}, "flow": {}, "gauge": {},
            "pi_cameras": {}, "dslr_cameras": {}, "od2000": {}, "wtt12l": {},
        }
        out = simulate_config(cfg)
        assert set(out) == {"gantry", "gocator"}

    def test_no_hardware_sections_left_untouched(self):
        """The specific defect: simulate_config() used to leave weir/flow (and
        everything else without a simulated backend) byte-for-byte identical
        to the input, so setup_run() built the real SaflWeirController/
        SaflFlowController against real hardware during simulate=True."""
        cfg = {"weir": {"port": "/dev/ttyUSB0"}}
        assert "weir" not in simulate_config(cfg)

    def test_the_original_config_is_not_mutated(self):
        cfg = {"gantry": {"transport": "pi_agent"}}
        simulate_config(cfg)
        assert cfg["gantry"]["transport"] == "pi_agent"


class TestSetupRunSimulation:
    """setup_run() reads its own local raw-YAML dict to decide which
    subsystems to build — a separate object from FlumeLab's own
    lab.config.config_dict. simulate=True has to reach both, or the section
    check here (`if "weir" in cfg`) still finds it and builds the real
    controller against real hardware."""

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

    def test_simulate_flows_through_to_setup_runs_own_subsystem_construction(
        self, tmp_path, monkeypatch
    ):
        from laguna.experiment.runner import setup_run

        constructed = []
        monkeypatch.setattr(
            "laguna.weir.SaflWeirController.__init__",
            lambda self, config: constructed.append("weir"),
        )
        monkeypatch.setattr(
            "laguna.flow.SaflFlowController.__init__",
            lambda self, config: constructed.append("flow"),
        )

        lab = setup_run(self._config_path(tmp_path), simulate=True)

        assert constructed == [], "weir/flow must not be constructed under simulate=True"
        assert "weir" not in lab._subsystems
        assert "flow" not in lab._subsystems

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
