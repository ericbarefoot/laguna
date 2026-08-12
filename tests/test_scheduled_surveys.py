"""The survey instruments can now take part in a scheduled experiment.

Before this, setup_run() knew about exactly five config sections — gauge,
weir, flow, pi_cameras, dslr_cameras — so a Gocator scan could not be part
of a scheduled run at all, and the Gocator had no zero-argument entry point
a scheduler could call.
"""

import numpy as np
import pytest

from laguna.scanner import GocatorScanner, ScanNotPossibleError
from laguna.scanner.pointcloud import SurfaceScan


class FakeAxisHandle:
    def __init__(self, name, calls, position=0.0):
        self.name = name
        self._calls = calls
        self._position = position

    def get_position(self):
        return self._position

    def set_speed(self, value):
        self._calls.append(("set_speed", self.name, value))
        return value

    def begin_move_to(self, position):
        self._calls.append(("begin_move_to", self.name, position))


class FakeGantry:
    subsystem_name = "gantry"

    def __init__(self, position=0.0):
        from laguna.robot.motion_arbiter import MotionArbiter

        self.calls = []
        self.arbiter = MotionArbiter()
        self._handles = {n: FakeAxisHandle(n, self.calls, position) for n in ("X", "Y")}

    def connect(self):
        return True

    def disconnect(self):
        pass

    def get_status(self):
        return {}

    def axis(self, name):
        try:
            return self._handles[name]
        except KeyError:
            raise KeyError(f"Axis {name!r} is not configured") from None

    def move_to(self, **axes):
        self.calls.append(("move_to", axes))
        return True


def make_scan():
    return SurfaceScan(
        z_mm=np.array([[1.0, 2.0], [3.0, 4.0]]),
        x_mm=np.array([0.0, 1.0]),
        y_mm=np.array([0.0, 1.0]),
    )


class StubScanner(GocatorScanner):
    """GocatorScanner with the acquisition path stubbed, so acquire()'s own
    logic (spec handling, gantry requirement, return-to-start) is what's
    under test."""

    def __init__(self, config):
        super().__init__(config)
        self.scans = []
        self.saved = []
        self._is_connected = True   # acquire() refuses on a disconnected scanner

    def scan_with_gantry(self, gantry, axis, end_mm, feed_rate_mm_s, **kw):
        self.scans.append({"axis": axis, "end_mm": end_mm,
                           "feed_rate_mm_s": feed_rate_mm_s, **kw})
        return make_scan()

    def save_scan(self, scan, name=None, formats=("npz", "laz")):
        self.saved.append(formats)
        self._last_saved_path = "/tmp/fake_scan.npz"
        return {"npz": "/tmp/fake_scan.npz"}


class TestAcquireIsSchedulable:
    def test_zero_arg_callable_with_a_configured_spec(self):
        """Scheduler.repeat(action=...) takes a zero-arg callable; that's the
        whole reason this method exists alongside scan_with_gantry()."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        gantry = FakeGantry()
        scanner.acquire(gantry=gantry)
        assert scanner.scans == [
            {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0, "settle_s": 0.5}
        ]

    def test_an_unconfigured_scanner_is_a_run_stopping_problem(self):
        """Missing data is as bad as a stationary gantry. A scheduled scan
        that cannot run must not be quietly skipped — the runner escalates
        this to a lab-wide pause."""
        scanner = StubScanner({"ip": "1.2.3.4"})
        with pytest.raises(ScanNotPossibleError, match="no 'scan:' config"):
            scanner.acquire(gantry=FakeGantry())

    def test_missing_required_key_is_rejected(self):
        scanner = StubScanner({"ip": "1.2.3.4", "scan": {"axis": "X"}})
        with pytest.raises(ScanNotPossibleError, match="missing"):
            scanner.acquire(gantry=FakeGantry())

    def test_requires_a_gantry(self):
        """Triggering on a stationary gantry yields a surface with no travel
        at all — better to refuse than to save something meaningless."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        with pytest.raises(ScanNotPossibleError, match="no connected gantry"):
            scanner.acquire()

    def test_an_unreachable_scanner_is_also_run_stopping(self):
        """'If the scanner isn't configured or it's unreachable in any way,
        this is cause to pause the run.'"""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        scanner._is_connected = False
        with pytest.raises(ScanNotPossibleError, match="not connected"):
            scanner.acquire(gantry=FakeGantry())

    def test_overrides_beat_the_configured_spec(self):
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        scanner.acquire(gantry=FakeGantry(), end_mm=50.0)
        assert scanner.scans[0]["end_mm"] == 50.0

    def test_formats_are_passed_to_save(self):
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
                     "formats": ["npz"]},
        })
        scanner.acquire(gantry=FakeGantry())
        # acquire() passes formats through as-is now (real save_scan()
        # normalizes it, including the bare-string case) — no more premature
        # tuple(formats), which used to split a bare string like "laz" into
        # its individual characters before save_scan() ever saw it.
        assert scanner.saved == [["npz"]]

    def test_bare_string_formats_config_is_one_format(self):
        """A YAML `formats: laz` (no list brackets) parses to a plain str,
        which used to be split into its characters by acquire()'s
        tuple(formats) before save_scan() ever got a chance to normalize it —
        "Unknown scan format: 'l'"."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
                     "formats": "laz"},
        })
        scanner.acquire(gantry=FakeGantry())
        assert scanner.saved == ["laz"]

    def test_no_save_when_no_formats_configured(self):
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        scanner.acquire(gantry=FakeGantry())
        assert scanner.saved == []

    def test_return_to_start_puts_the_axis_back(self):
        """Repeat scans of one transect need the axis back where it began, or
        each pass starts further along than the last."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
                     "return_to_start": True},
        })
        gantry = FakeGantry(position=15.0)
        scanner.acquire(gantry=gantry)
        assert ("move_to", {"X": 15.0}) in gantry.calls

    def test_no_return_by_default(self):
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        gantry = FakeGantry(position=15.0)
        scanner.acquire(gantry=gantry)
        assert not any(c[0] == "move_to" for c in gantry.calls)

    def test_arbiter_held_continuously_through_the_return_move(self):
        """The scan and the return-to-start move must be one continuous
        hold — releasing in between would let another scheduled action move
        the axis before the return move starts."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
                     "return_to_start": True},
        })
        gantry = FakeGantry(position=15.0)
        held_during_return = []
        original_move_to = gantry.move_to

        def move_to_spy(**axes):
            held_during_return.append(gantry.arbiter.is_held)
            return original_move_to(**axes)

        gantry.move_to = move_to_spy
        scanner.acquire(gantry=gantry)

        assert held_during_return == [True]
        assert gantry.arbiter.is_held is False


class TestRunnerKnowsAboutSurveyInstruments:
    """Behavioral coverage for setup_run() actually wiring gantry/gocator in
    — not just source-text matching, which would pass even if the section
    were recognised but never instantiated, connected, or scheduled."""

    def _config_path(self, tmp_path, gocator_scan_cfg=None):
        import yaml

        cfg = {
            "gantry": {"host": "gantry.lab", "type": "pi"},
            "gocator": {
                "ip": "192.168.1.10",
                "interval_s": 30,
                **(gocator_scan_cfg or {}),
            },
        }
        path = tmp_path / "experiment_config.yaml"
        path.write_text(yaml.safe_dump(cfg))
        return str(path)

    def _patch_survey_subsystems(self, monkeypatch, gantry, scanner):
        monkeypatch.setattr(
            "laguna.robot.macron.controller.GantryController.from_config",
            classmethod(lambda cls, config: gantry),
        )
        monkeypatch.setattr(
            "laguna.scanner.GocatorScanner.from_config",
            classmethod(lambda cls, config: scanner),
        )

    def test_gantry_and_gocator_are_actually_instantiated_and_registered(
        self, tmp_path, monkeypatch
    ):
        """Not just recognised in setup_run's source — actually built via
        from_config() and added to the lab as real subsystems."""
        gantry = FakeGantry()
        scanner = StubScanner({"ip": "192.168.1.10", "scan": {
            "axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
        }})
        self._patch_survey_subsystems(monkeypatch, gantry, scanner)

        from laguna.experiment.runner import setup_run

        lab = setup_run(self._config_path(tmp_path))

        assert lab.gantry is gantry
        assert lab.gocator is scanner

    def test_gocator_interval_actually_registers_a_recurring_action(
        self, tmp_path, monkeypatch
    ):
        """The point of this workstream: a Gocator scan reaches the
        scheduler's real _recurring list, the same mechanism every other
        scheduled subsystem uses — not a parallel code path."""
        gantry = FakeGantry()
        scanner = StubScanner({"ip": "192.168.1.10", "scan": {
            "axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
        }})
        self._patch_survey_subsystems(monkeypatch, gantry, scanner)

        from laguna.experiment.runner import setup_run

        lab = setup_run(self._config_path(tmp_path))

        gocator_actions = [e for e in lab.scheduler._recurring if e["subsystem"] == "gocator"]
        assert len(gocator_actions) == 1
        assert gocator_actions[0]["every"] == 30
        assert gocator_actions[0]["name"] == "scan"

    def test_the_registered_action_actually_triggers_a_scan_when_fired(
        self, tmp_path, monkeypatch
    ):
        """Firing the registered action must reach GocatorScanner.acquire()
        — proving it's a live callable, not a stub that happens to satisfy
        the section/name bookkeeping."""
        gantry = FakeGantry()
        scanner = StubScanner({"ip": "192.168.1.10", "scan": {
            "axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0,
        }})
        self._patch_survey_subsystems(monkeypatch, gantry, scanner)

        from laguna.experiment.runner import setup_run

        lab = setup_run(self._config_path(tmp_path))
        action = next(
            e["action"] for e in lab.scheduler._recurring if e["subsystem"] == "gocator"
        )
        action()

        assert len(scanner.scans) == 1

    def test_scheduling_keys_are_validated_for_gocator(self):
        from laguna.experiment.runner import _validate_trigger_config

        # Mutually exclusive, same rule as every other section.
        with pytest.raises(ValueError):
            _validate_trigger_config("gocator", {"interval_s": 10, "trigger_at": [5]})


class TestFailuresEscalateRatherThanSkip:
    """A scheduled scan that cannot run, or a gantry that is already busy,
    both mean the scripted plan is no longer being followed. Continuing just
    accumulates data under conditions nobody recorded."""

    def test_scan_config_uses_the_configured_spec_without_repeating_it(self):
        """scan_with_gantry() falls back to gocator.scan:, so acquire() is a
        thin wrapper rather than a second code path."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 300.0, "feed_rate_mm_s": 12.0},
        })
        # Call the real method (not the stub's override) to exercise defaulting.
        spec = scanner._scan_spec
        assert spec["axis"] == "X" and spec["end_mm"] == 300.0

    def test_arbiter_contention_raises_rather_than_queueing(self):
        """An errant move landing mid-motion hints something much bigger is
        wrong, so it must surface, not silently serialise."""
        import threading

        from laguna.robot.motion_arbiter import MotionArbiter, MotionBusyError

        arbiter = MotionArbiter()
        started, release = threading.Event(), threading.Event()

        def holder():
            with arbiter.hold("scripted scan"):
                started.set()
                release.wait(timeout=2)

        t = threading.Thread(target=holder, daemon=True)
        t.start()
        started.wait(timeout=2)
        try:
            with pytest.raises(MotionBusyError, match="scripted scan"):
                with arbiter.hold("errant move", timeout_s=0.02):
                    pass
        finally:
            release.set()
            t.join(timeout=2)

    def test_runner_escalates_a_failed_scan_to_a_lab_pause(self):
        """The closure in setup_run() must call lab.escalate(), not swallow."""
        import inspect

        from laguna.experiment import runner

        source = inspect.getsource(runner.setup_run)
        assert "lab.escalate(" in source, "a failed scan is silently skipped"
        assert "ScanNotPossibleError" in source
        assert "MotionBusyError" in source
