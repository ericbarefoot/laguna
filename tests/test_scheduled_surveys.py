"""The survey instruments can now take part in a scheduled experiment.

Before this, setup_run() knew about exactly five config sections — gauge,
weir, flow, pi_cameras, dslr_cameras — so a Gocator scan could not be part
of a scheduled run at all, and the Gocator had no zero-argument entry point
a scheduler could call.
"""

import numpy as np
import pytest

from laguna.scanner import GocatorScanner
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

    def test_no_spec_returns_none_rather_than_raising(self):
        """An unconfigured scanner in a scheduled run should log and let the
        experiment continue, not crash it."""
        scanner = StubScanner({"ip": "1.2.3.4"})
        assert scanner.acquire(gantry=FakeGantry()) is None

    def test_missing_required_key_is_rejected(self):
        scanner = StubScanner({"ip": "1.2.3.4", "scan": {"axis": "X"}})
        with pytest.raises(ValueError, match="missing"):
            scanner.acquire(gantry=FakeGantry())

    def test_requires_a_gantry(self):
        """Triggering on a stationary gantry yields a surface with no travel
        at all — better to refuse than to save something meaningless."""
        scanner = StubScanner({
            "ip": "1.2.3.4",
            "scan": {"axis": "X", "end_mm": 200.0, "feed_rate_mm_s": 20.0},
        })
        with pytest.raises(ValueError, match="needs a connected gantry"):
            scanner.acquire()

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
        assert scanner.saved == [("npz",)]

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


class TestRunnerKnowsAboutSurveyInstruments:
    def test_survey_sections_are_recognised(self):
        """The hardcoded five-section tuple is what kept the survey half out
        of scheduled experiments entirely."""
        import inspect

        from laguna.experiment import runner

        source = inspect.getsource(runner.setup_run)
        for section in ("gantry", "gocator"):
            assert f'"{section}"' in source, f"{section} still absent from setup_run"

    def test_scheduling_keys_are_validated_for_gocator(self):
        from laguna.experiment.runner import _validate_trigger_config

        # Mutually exclusive, same rule as every other section.
        with pytest.raises(ValueError):
            _validate_trigger_config("gocator", {"interval_s": 10, "trigger_at": [5]})

    def test_register_action_is_reused_not_reimplemented(self):
        """_register_action was already subsystem-agnostic; the point of this
        workstream was to use it, not to write a parallel mechanism."""
        import inspect

        from laguna.experiment import runner

        source = inspect.getsource(runner.setup_run)
        assert source.count("_register_action(lab, cfg.get(\"gocator\"") == 1
