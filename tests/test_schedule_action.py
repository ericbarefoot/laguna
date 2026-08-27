"""Tests for schedule_action() — the public building block behind
setup_run()'s interval_s/trigger_at/use_schedule dispatch, promoted so a
user-defined action (e.g. a tiled survey spanning gantry+gocator) gets
the same scheduling-mode choice, validation, and overlap guard as a
built-in one.
"""

import threading
import time

import pandas as pd
import pytest

from laguna.experiment import schedule_action
from laguna.schedule import ExperimentSchedule


class _FakeScheduler:
    """Records repeat()/at() calls without a real clock/thread."""

    def __init__(self):
        self.repeats = []  # (every, action, subsystem, name)
        self.ats = []  # (runtime_s, action, subsystem, name)

    def repeat(self, every, action, subsystem="scheduler", name=""):
        self.repeats.append((every, action, subsystem, name))

    def at(self, runtime_s, action, subsystem="scheduler", name=""):
        self.ats.append((runtime_s, action, subsystem, name))


class _FakeLab:
    def __init__(self):
        self.scheduler = _FakeScheduler()
        self.escalations = []
        self._exclusion_locks = {}

    def escalate(self, problem, tier=None):
        self.escalations.append(problem)


def _registered_actions(lab):
    """All action callables actually registered (repeat + at), in order."""
    return [a for _, a, _, _ in lab.scheduler.repeats] + [a for _, a, _, _ in lab.scheduler.ats]


def test_interval_s_registers_a_repeat():
    lab = _FakeLab()
    calls = []
    schedule_action(lab, {"interval_s": 15}, "tiled_scan", "run", action=lambda: calls.append(1))
    assert len(lab.scheduler.repeats) == 1
    every, registered, subsystem, name = lab.scheduler.repeats[0]
    assert (every, subsystem, name) == (15, "tiled_scan", "run")
    registered()  # the registered callable wraps the real action — must still run it
    assert calls == [1]
    assert lab.scheduler.ats == []


def test_trigger_at_registers_one_at_per_time():
    lab = _FakeLab()
    calls = []
    schedule_action(
        lab, {"trigger_at": [0, 30, 90]}, "tiled_scan", "run",
        action=lambda: calls.append(1),
    )
    assert [t for t, *_ in lab.scheduler.ats] == [0.0, 30.0, 90.0]
    for _, registered, subsystem, name in lab.scheduler.ats:
        assert (subsystem, name) == ("tiled_scan", "run")
        registered()
    assert calls == [1, 1, 1]


def test_use_schedule_fires_at_every_schedule_row():
    lab = _FakeLab()
    action = lambda: None
    exp_schedule = ExperimentSchedule(pd.DataFrame({"time_s": [0, 60, 120]}))
    schedule_action(
        lab, {"use_schedule": True}, "tiled_scan", "run",
        action=action, exp_schedule=exp_schedule,
    )
    assert [t for t, *_ in lab.scheduler.ats] == [0.0, 60.0, 120.0]


def test_use_schedule_filters_by_schedule_col():
    lab = _FakeLab()
    action = lambda: None
    exp_schedule = ExperimentSchedule(
        pd.DataFrame({"time_s": [0, 60, 120], "tiled_scan": [1, 0, 1]})
    )
    schedule_action(
        lab, {"use_schedule": True}, "tiled_scan", "run",
        action=action, exp_schedule=exp_schedule, schedule_col="tiled_scan",
    )
    assert [t for t, *_ in lab.scheduler.ats] == [0.0, 120.0]


def test_use_schedule_without_exp_schedule_warns_and_skips():
    lab = _FakeLab()
    schedule_action(lab, {"use_schedule": True}, "tiled_scan", "run", action=lambda: None)
    assert lab.scheduler.ats == []
    assert lab.scheduler.repeats == []


def test_action_factory_receives_each_firing_time():
    lab = _FakeLab()
    seen = []
    exp_schedule = ExperimentSchedule(pd.DataFrame({"time_s": [10, 20]}))
    schedule_action(
        lab, {"use_schedule": True}, "tiled_scan", "run",
        action_factory=lambda t: (lambda: seen.append(t)),
        exp_schedule=exp_schedule,
    )
    for _, registered, _, _ in lab.scheduler.ats:
        registered()
    assert seen == [10.0, 20.0]


def test_no_scheduling_key_registers_nothing():
    lab = _FakeLab()
    schedule_action(lab, {}, "tiled_scan", "run", action=lambda: None)
    assert lab.scheduler.repeats == []
    assert lab.scheduler.ats == []


class TestMutualExclusionValidation:
    def test_use_schedule_and_interval_s_raises(self):
        lab = _FakeLab()
        with pytest.raises(ValueError):
            schedule_action(
                lab, {"use_schedule": True, "interval_s": 10}, "tiled_scan", "run",
                action=lambda: None,
            )

    def test_use_schedule_and_trigger_at_raises(self):
        lab = _FakeLab()
        with pytest.raises(ValueError):
            schedule_action(
                lab, {"use_schedule": True, "trigger_at": [0]}, "tiled_scan", "run",
                action=lambda: None,
            )

    def test_interval_s_and_trigger_at_raises(self):
        lab = _FakeLab()
        with pytest.raises(ValueError):
            schedule_action(
                lab, {"interval_s": 10, "trigger_at": [0]}, "tiled_scan", "run",
                action=lambda: None,
            )

    def test_arbitrary_user_defined_section_name_is_validated_too(self):
        """The whole point: a section that isn't a built-in subsystem name
        (e.g. a user-defined multi-subsystem hook) gets the exact same
        validation as gauge/weir/flow/etc."""
        lab = _FakeLab()
        with pytest.raises(ValueError, match="my_custom_hook"):
            schedule_action(
                lab, {"interval_s": 10, "trigger_at": [0]}, "my_custom_hook", "run",
                action=lambda: None,
            )


class TestOverlapGuard:
    """A firing that starts before the previous one finished must escalate
    (pause the lab) rather than run concurrently or silently skip — missed/
    duplicated data is worse than a pause; a schedule dense enough to cause
    this is an experiment-design problem to fix, not a runtime hiccup to
    paper over. See schedule_action()'s docstring."""

    def test_sequential_firings_never_escalate(self):
        lab = _FakeLab()
        calls = []
        schedule_action(
            lab, {"trigger_at": [0, 1]}, "tiled_scan", "run",
            action=lambda: calls.append(1),
        )
        for _, registered, _, _ in lab.scheduler.ats:
            registered()  # each call fully returns before the next starts
        assert calls == [1, 1]
        assert lab.escalations == []

    def test_overlapping_firing_escalates_instead_of_running_concurrently(self):
        lab = _FakeLab()
        entered = threading.Event()
        release = threading.Event()
        concurrent_entries = []

        def slow_action():
            concurrent_entries.append(1)
            entered.set()
            release.wait(timeout=2.0)

        schedule_action(
            lab, {"trigger_at": [0, 1]}, "tiled_scan", "run", action=slow_action,
        )
        first = lab.scheduler.ats[0][1]
        second = lab.scheduler.ats[1][1]

        t = threading.Thread(target=first)
        t.start()
        assert entered.wait(timeout=2.0), "first firing never started"

        second()  # fires while the first is still blocked inside slow_action
        release.set()
        t.join(timeout=2.0)

        assert len(concurrent_entries) == 1  # slow_action body ran exactly once
        assert len(lab.escalations) == 1
        assert "tiled_scan/run" in lab.escalations[0]

    def test_guard_releases_after_completion_so_next_firing_runs_normally(self):
        lab = _FakeLab()
        calls = []
        schedule_action(
            lab, {"trigger_at": [0, 1]}, "tiled_scan", "run",
            action=lambda: calls.append(1),
        )
        first, second = lab.scheduler.ats[0][1], lab.scheduler.ats[1][1]
        first()
        second()  # first already returned and released the lock — must run normally
        assert calls == [1, 1]
        assert lab.escalations == []

    def test_guard_releases_even_if_action_raises(self):
        """A failed action must still release the lock — otherwise one
        exception would falsely make every subsequent firing look like an
        overlap forever."""
        lab = _FakeLab()
        calls = []
        state = {"fail": True}

        def flaky():
            if state["fail"]:
                raise RuntimeError("boom")
            calls.append(1)

        schedule_action(lab, {"trigger_at": [0, 1]}, "tiled_scan", "run", action=flaky)
        first, second = lab.scheduler.ats[0][1], lab.scheduler.ats[1][1]

        with pytest.raises(RuntimeError):
            first()

        state["fail"] = False
        second()  # if the lock leaked from first()'s exception, this would escalate instead
        assert calls == [1]
        assert lab.escalations == []


class TestExclusiveWith:
    """Two differently-named actions run fully independently by default;
    exclusive_with opts a specific pair (or group) into sharing one lock,
    without touching anything else's behavior."""

    def test_different_actions_do_not_block_each_other_by_default(self):
        lab = _FakeLab()
        gantry_entered = threading.Event()
        gantry_release = threading.Event()
        camera_calls = []

        def gantry_scan():
            gantry_entered.set()
            gantry_release.wait(timeout=2.0)

        def camera_capture():
            camera_calls.append(1)

        schedule_action(lab, {"trigger_at": [0]}, "gocator", "tiled_scan", action=gantry_scan)
        schedule_action(lab, {"trigger_at": [0]}, "pi_cameras", "capture", action=camera_capture)
        scan_action = lab.scheduler.ats[0][1]
        capture_action = lab.scheduler.ats[1][1]

        t = threading.Thread(target=scan_action)
        t.start()
        assert gantry_entered.wait(timeout=2.0)

        capture_action()  # must run immediately, not wait on the scan
        gantry_release.set()
        t.join(timeout=2.0)

        assert camera_calls == [1]
        assert lab.escalations == []

    def test_shared_tag_makes_two_different_actions_mutually_exclusive(self):
        lab = _FakeLab()
        entered = threading.Event()
        release = threading.Event()
        camera_calls = []

        def gantry_scan():
            entered.set()
            release.wait(timeout=2.0)

        def camera_capture():
            camera_calls.append(1)

        schedule_action(
            lab, {"trigger_at": [0]}, "gocator", "tiled_scan", action=gantry_scan,
            exclusive_with="gantry_busy",
        )
        schedule_action(
            lab, {"trigger_at": [0]}, "pi_cameras", "capture", action=camera_capture,
            exclusive_with="gantry_busy",
        )
        scan_action = lab.scheduler.ats[0][1]
        capture_action = lab.scheduler.ats[1][1]

        t = threading.Thread(target=scan_action)
        t.start()
        assert entered.wait(timeout=2.0)

        capture_action()  # scan is mid-flight and holds the shared tag — must escalate
        release.set()
        t.join(timeout=2.0)

        assert camera_calls == []
        assert len(lab.escalations) == 1
        assert "gantry_busy" in lab.escalations[0]

    def test_shared_tag_releases_so_a_later_non_overlapping_firing_runs(self):
        lab = _FakeLab()
        calls = []
        schedule_action(
            lab, {"trigger_at": [0]}, "gocator", "tiled_scan", action=lambda: None,
            exclusive_with="gantry_busy",
        )
        schedule_action(
            lab, {"trigger_at": [0]}, "pi_cameras", "capture", action=lambda: calls.append(1),
            exclusive_with="gantry_busy",
        )
        scan_action, capture_action = lab.scheduler.ats[0][1], lab.scheduler.ats[1][1]

        scan_action()  # runs and releases immediately (lambda: None)
        capture_action()  # tag is free now — must run normally

        assert calls == [1]
        assert lab.escalations == []
