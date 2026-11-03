"""Per-subsystem ``resume_lag_s``: spin-up that elapses before runtime counts.

The behaviours worth pinning: a lagged subsystem is resumed *before* the
clock and everything else *after* it, the clock and the PAUSED state hold for
the whole lag, and a pause/stop/estop during the lag abandons the resume with
the clock frozen and the lagged subsystem re-quiesced.
"""

import threading
import time

import pytest

from laguna import FlumeLab
from laguna.safety import SafetyState


class Probe:
    """Records each verb with the clock and rig state at that instant."""

    def __init__(self, lab, name, lag=None):
        self.lab = lab
        self.subsystem_name = name
        if lag is not None:
            self.resume_lag_s = lag
        self.calls = []

    def _rec(self, verb):
        self.calls.append((verb, self.lab.clock.is_paused, self.lab.safety_state))

    def verbs(self):
        return [c[0] for c in self.calls]

    def pause(self):
        self._rec("pause")

    def resume(self):
        self._rec("resume")

    def stop(self):
        self._rec("stop")

    def estop(self):
        self._rec("estop")

    def connect(self):
        return True

    def disconnect(self):
        pass

    def get_status(self):
        return {"is_connected": True}


def _lab(lag=0.3):
    lab = FlumeLab()
    pump = Probe(lab, "flow", lag)
    cam = Probe(lab, "camera")
    lab.add(pump).add(cam)
    return lab, pump, cam


class TestLagValue:
    def test_defaults_to_no_lag(self):
        lab = FlumeLab()
        lab.add(Probe(lab, "flow"))
        assert lab._lagged_subsystems() == {}

    def test_read_from_the_subsystems_config_section(self):
        lab = FlumeLab()
        lab.config.config_dict["flow"]["resume_lag_s"] = 10
        lab.add(Probe(lab, "flow"))
        assert lab._lagged_subsystems() == {"flow": 10.0}

    def test_attribute_wins_over_config(self):
        lab = FlumeLab()
        lab.config.config_dict["flow"]["resume_lag_s"] = 10
        lab.add(Probe(lab, "flow", lag=2))
        assert lab._lagged_subsystems() == {"flow": 2.0}

    @pytest.mark.parametrize("bad", [-1, "soon", float("nan"), float("inf")])
    def test_a_bad_lag_fails_at_add_not_mid_run(self, bad):
        lab = FlumeLab()
        with pytest.raises(ValueError, match="resume_lag_s"):
            lab.add(Probe(lab, "flow", lag=bad))


class TestResumeWithLag:
    def test_lagged_subsystem_resumes_before_the_clock_and_the_rest_after(self):
        lab, pump, cam = _lab()
        lab.clock.start()
        lab.pause()
        pump.calls.clear(), cam.calls.clear()

        lab.resume()

        verb, clock_paused, state = pump.calls[0]
        assert (verb, clock_paused, state) == ("resume", True, SafetyState.PAUSED)
        verb, clock_paused, _ = cam.calls[0]
        assert (verb, clock_paused) == ("resume", False)
        assert lab.safety_state is SafetyState.RUNNING

    def test_resume_blocks_for_the_lag_and_runtime_does_not_advance(self):
        lab, _, _ = _lab(lag=0.3)
        lab.clock.start()
        lab.pause()
        frozen = lab.clock.elapsed()

        t0 = time.monotonic()
        lab.resume()

        assert time.monotonic() - t0 >= 0.3
        assert lab.clock.elapsed() - frozen < 0.1, "the lag was counted as experiment time"

    def test_logs_the_wait(self):
        lab, _, _ = _lab(lag=0.05)
        lab.clock.start()
        lab.pause()
        lab.resume()
        events = [row for row in open(lab.event_log._path).read().splitlines()]
        assert any("spinup_wait" in row for row in events)

    def test_no_lag_keeps_the_original_order(self):
        lab = FlumeLab()
        a = Probe(lab, "flow")
        lab.add(a)
        lab.clock.start()
        lab.pause()
        a.calls.clear()
        lab.resume()
        assert a.calls[0][:2] == ("resume", False), "clock must resume before subsystems"


class TestAbortDuringLag:
    def _resume_in_background(self, lab):
        out = {}
        t = threading.Thread(target=lambda: out.setdefault("r", lab.resume()), daemon=True)
        t.start()
        return t, out

    def _wait_for_lagged_resume(self, pump):
        deadline = time.monotonic() + 2
        while "resume" not in pump.verbs() and time.monotonic() < deadline:
            time.sleep(0.005)

    def test_pause_during_the_lag_abandons_the_resume_with_the_clock_frozen(self):
        lab, pump, cam = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        pump.calls.clear(), cam.calls.clear()

        t, out = self._resume_in_background(lab)
        self._wait_for_lagged_resume(pump)
        t0 = time.monotonic()
        lab.pause(reason="test")
        t.join(timeout=2)

        assert not t.is_alive() and time.monotonic() - t0 < 2, "the wait was not interrupted"
        assert out["r"] is None
        assert lab.clock.is_paused
        assert lab.safety_state is SafetyState.PAUSED
        assert "resume" not in cam.verbs(), "a non-lagged subsystem resumed into an aborted spin-up"
        assert pump.verbs()[-1] == "pause"

    def test_estop_during_the_lag_re_sends_estop_to_the_lagged_subsystem(self):
        lab, pump, cam = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        pump.calls.clear()

        t, _ = self._resume_in_background(lab)
        self._wait_for_lagged_resume(pump)
        lab.estop(reason="test")
        t.join(timeout=2)

        assert lab.safety_state is SafetyState.ESTOPPED
        assert lab.clock.is_paused
        assert pump.verbs().count("estop") >= 2, "the abort did not re-quiesce the pump"

    def test_a_trigger_still_asserted_refuses_before_any_lagged_resume(self):
        lab, pump, _ = _lab()
        lab.clock.start()
        lab.pause()
        pump.calls.clear()
        lab.safety_monitor.tripped_by = lambda tier=None: "PAUSE"
        assert lab.resume() is None
        assert "resume" not in pump.verbs()


class TestStartWithLag:
    def test_clock_is_frozen_until_the_preroll_and_lag_are_done(self):
        lab, pump, _ = _lab(lag=0.3)
        seen = {}

        def t0_action():
            seen["paused"] = lab.clock.is_paused
            seen["runtime"] = lab.clock.elapsed()

        lab.scheduler.at(0.0, t0_action, subsystem="flow", name="set")
        thread = lab.start(60)
        assert lab.clock.is_paused, "T=0 started counting before spin-up"
        time.sleep(0.15)
        assert lab.clock.is_paused and lab.clock.elapsed() < 0.01
        deadline = time.monotonic() + 2
        while lab.clock.is_paused and time.monotonic() < deadline:
            time.sleep(0.01)

        assert seen["paused"] is True and seen["runtime"] < 0.01
        assert not lab.clock.is_paused and lab.scheduler.is_running
        assert lab.clock.elapsed() < 0.2
        lab.pause()
        thread.join(timeout=2)

    def test_the_t0_action_is_not_fired_twice(self):
        lab, _, _ = _lab(lag=0.05)
        fired = []
        lab.scheduler.at(0.0, lambda: fired.append(1), subsystem="flow", name="set")
        thread = lab.start(60)
        time.sleep(0.4)
        lab.pause()
        thread.join(timeout=2)
        assert fired == [1]

    def test_only_lagged_subsystems_actions_are_prerolled(self):
        lab, _, _ = _lab(lag=0.05)
        order = []
        lab.scheduler.at(0.0, lambda: order.append(("camera", lab.clock.is_paused)),
                         subsystem="camera", name="snap")
        lab.scheduler.at(0.0, lambda: order.append(("flow", lab.clock.is_paused)),
                         subsystem="flow", name="set")
        thread = lab.start(60)
        time.sleep(0.4)
        lab.pause()
        thread.join(timeout=2)
        assert order[0] == ("flow", True)
        assert ("camera", False) in order

    def test_pause_during_the_start_lag_leaves_the_clock_at_zero_and_resume_carries_on(self):
        lab, pump, cam = _lab(lag=5)
        thread = lab.start(60)
        time.sleep(0.1)
        lab.pause(reason="test")
        thread.join(timeout=2)

        assert not thread.is_alive()
        assert lab.clock.is_paused and lab.clock.elapsed() < 0.01
        assert not lab.scheduler.is_running

        lab.safety_monitor.tripped_by = lambda tier=None: None
        lab.config.config_dict["flow"]["resume_lag_s"] = 0.05
        pump.resume_lag_s = 0.05
        resumed = lab.resume()
        assert resumed is not None and lab.scheduler.is_running
        lab.pause()
        resumed.join(timeout=2)

    def test_a_failed_preroll_action_pauses_instead_of_starting_the_clock(self):
        lab, _, _ = _lab(lag=0.05)

        def boom():
            raise RuntimeError("pump did not start")

        lab.scheduler.at(0.0, boom, subsystem="flow", name="set")
        thread = lab.start(60)
        thread.join(timeout=2)
        deadline = time.monotonic() + 2
        while lab.safety_state is not SafetyState.PAUSED and time.monotonic() < deadline:
            time.sleep(0.01)
        assert lab.safety_state is SafetyState.PAUSED
        assert lab.clock.is_paused and lab.clock.elapsed() < 0.01

    def test_no_lag_start_is_unchanged(self):
        lab = FlumeLab()
        lab.add(Probe(lab, "flow"))
        thread = lab.start(60)
        assert not lab.clock.is_paused
        lab.pause()
        thread.join(timeout=2)


class TestSchedulerPreroll:
    def test_skips_when_the_gate_vetoes_and_leaves_the_entry_unfired(self):
        lab = FlumeLab()
        fired = []
        lab.scheduler.at(0.0, lambda: fired.append(1), subsystem="flow", name="set")
        lab.pause()
        problems = lab.scheduler.preroll({"flow"})
        assert fired == [] and problems and "PAUSED" in problems[0]
        assert lab.scheduler._oneshot[0]["_fired"] is False

    def test_ignores_later_entries(self):
        lab = FlumeLab()
        fired = []
        lab.scheduler.at(30.0, lambda: fired.append(1), subsystem="flow", name="later")
        assert lab.scheduler.preroll({"flow"}) == []
        assert fired == []
