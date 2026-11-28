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
        self.resume_result = None   # a note (or an exception) to return from resume()
        self.on_resume = None       # callable run inside resume(), e.g. to land an estop

    def _rec(self, verb):
        self.calls.append((verb, self.lab.clock.is_paused, self.lab.safety_state))

    def verbs(self):
        return [c[0] for c in self.calls]

    def pause(self):
        self._rec("pause")

    def resume(self):
        self._rec("resume")
        if self.on_resume:
            self.on_resume()
        if isinstance(self.resume_result, BaseException):
            raise self.resume_result
        return self.resume_result

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
        in_wait = threading.Event()
        lab.scheduler.at(0.0, in_wait.set, subsystem="flow", name="set")
        thread = lab.start(60)
        assert in_wait.wait(2), "the pre-roll never ran"
        lab.pause(reason="test")
        thread.join(timeout=2)

        assert not thread.is_alive()
        assert lab.clock.is_paused and lab.clock.elapsed() < 0.01
        assert not lab.scheduler.is_running
        assert pump.verbs()[-1] == "pause"

        pump.resume_lag_s = 0.05
        resumed = lab.resume()
        assert resumed is not None and lab.scheduler.is_running
        lab.pause()
        resumed.join(timeout=2)

    @pytest.mark.parametrize("verb,state", [("estop", SafetyState.ESTOPPED),
                                            ("end_run", SafetyState.STOPPED)])
    def test_estop_or_end_run_during_the_start_lag_keeps_the_clock_frozen(self, verb, state):
        lab, pump, _ = _lab(lag=5)
        in_wait = threading.Event()
        lab.scheduler.at(0.0, in_wait.set, subsystem="flow", name="set")
        thread = lab.start(60)
        assert in_wait.wait(2)
        getattr(lab, verb)(reason="test")
        thread.join(timeout=2)

        assert not thread.is_alive()
        assert lab.safety_state is state
        assert lab.clock.is_paused and not lab.scheduler.is_running
        assert pump.verbs()[-1] == ("estop" if verb == "estop" else "stop")

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


class TestResumeIsNotRacedByASafetyVerb:
    """A verb landing at any point in resume() must leave the clock frozen."""

    def test_an_estop_between_the_trigger_check_and_the_lagged_resume(self):
        lab, pump, cam = _lab(lag=0.2)
        lab.clock.start()
        lab.pause()
        pump.calls.clear(), cam.calls.clear()

        def estop_after_check(tier=None):
            lab.estop(reason="test")
            return None

        lab.safety_monitor.tripped_by = estop_after_check
        assert lab.resume() is None

        assert "resume" not in pump.verbs(), "the pump was restarted under an estop"
        assert "resume" not in cam.verbs()
        assert lab.safety_state is SafetyState.ESTOPPED and lab.clock.is_paused

    @pytest.mark.parametrize("verb,state", [("pause", SafetyState.PAUSED),
                                            ("estop", SafetyState.ESTOPPED)])
    def test_a_verb_landing_after_the_wait_but_before_the_clock_restarts(self, verb, state):
        lab, pump, cam = _lab(lag=0.05)
        lab.clock.start()
        lab.pause()
        cam.calls.clear()
        real = lab._spin_up

        def spin_up_then_verb(lagged):
            ok = real(lagged)
            getattr(lab, verb)(reason="test")
            return ok

        lab._spin_up = spin_up_then_verb
        assert lab.resume() is None

        assert lab.clock.is_paused and lab.safety_state is state
        assert "resume" not in cam.verbs()

    def test_an_estop_landing_during_the_lagged_resume_itself(self):
        lab, pump, cam = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        cam.calls.clear()
        pump.on_resume = lambda: lab.estop(reason="test")

        t0 = time.monotonic()
        assert lab.resume() is None

        assert time.monotonic() - t0 < 2, "the estop's abort signal was lost"
        assert lab.clock.is_paused and lab.safety_state is SafetyState.ESTOPPED
        assert "resume" not in cam.verbs()

    def test_end_run_during_the_lag(self):
        lab, pump, cam = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        pump.calls.clear(), cam.calls.clear()
        out = {}
        t = threading.Thread(target=lambda: out.setdefault("r", lab.resume()), daemon=True)
        t.start()
        deadline = time.monotonic() + 2
        while "resume" not in pump.verbs() and time.monotonic() < deadline:
            time.sleep(0.005)
        lab.end_run(reason="test")
        t.join(timeout=2)

        assert not t.is_alive() and out["r"] is None
        assert lab.safety_state is SafetyState.STOPPED and lab.clock.is_paused
        assert pump.verbs()[-1] == "stop" and "resume" not in cam.verbs()


class TestFailedLaggedResume:
    @pytest.mark.parametrize("failure", [
        "pump did not restart: no ack",
        RuntimeError("confluence unreachable"),
    ])
    def test_a_failed_lagged_resume_does_not_start_the_clock(self, failure):
        lab, pump, cam = _lab(lag=0.05)
        lab.clock.start()
        lab.pause()
        pump.calls.clear(), cam.calls.clear()
        pump.resume_result = failure
        before = len(open(lab.event_log._path).read())

        assert lab.resume() is None

        assert lab.safety_state is SafetyState.PAUSED
        assert lab.clock.is_paused, "the clock started with no flow"
        assert "resume" not in cam.verbs()
        assert pump.verbs()[-1] == "pause", "a half-started pump was left running"
        rows = open(lab.event_log._path).read()[before:]
        assert "spinup_failed" in rows and "spinup_wait" not in rows

    def test_it_fails_fast_rather_than_waiting_out_the_lag(self):
        lab, pump, _ = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        pump.resume_result = "pump did not restart"
        t0 = time.monotonic()
        lab.resume()
        assert time.monotonic() - t0 < 2

    def test_resume_works_again_once_the_subsystem_recovers(self):
        lab, pump, cam = _lab(lag=0.05)
        lab.clock.start()
        lab.pause()
        pump.resume_result = "pump did not restart"
        assert lab.resume() is None and lab.clock.is_paused
        pump.resume_result = None
        lab.resume()
        assert not lab.clock.is_paused and lab.safety_state is SafetyState.RUNNING


class TestResumeAfterAnAbortedStart:
    def _aborted_start(self, lab):
        """start() while PAUSED: the gate refuses the pre-roll, as an early pause would."""
        lab.pause(reason="before start")
        thread = lab.start(60)
        thread.join(timeout=2)
        assert not thread.is_alive()

    def test_resume_fires_the_pending_t0_action_before_the_clock_and_instead_of_resume(self):
        lab, pump, _ = _lab(lag=0.05)
        seen = []
        lab.scheduler.at(0.0, lambda: seen.append(lab.clock.is_paused),
                         subsystem="flow", name="set")
        self._aborted_start(lab)
        assert seen == [], "the gate should have refused the pre-roll"
        pump.calls.clear()

        resumed = lab.resume()

        assert seen == [True], "T=0 action must fire once, with the clock still frozen"
        assert "resume" not in pump.verbs(), "resume() would restart at a stale setpoint"
        assert resumed is not None and lab.scheduler.is_running
        time.sleep(0.2)
        assert seen == [True], "the T=0 action fired a second time"
        lab.pause()
        resumed.join(timeout=2)

    def test_a_failed_t0_action_is_retried_not_dropped(self):
        lab, pump, _ = _lab(lag=0.05)
        attempts = []

        def flaky():
            attempts.append(1)
            if len(attempts) == 1:
                raise RuntimeError("no ack")

        lab.scheduler.at(0.0, flaky, subsystem="flow", name="set")
        thread = lab.start(60)
        thread.join(timeout=2)
        assert lab.safety_state is SafetyState.PAUSED and len(attempts) == 1

        resumed = lab.resume()

        assert len(attempts) == 2 and not lab.clock.is_paused
        assert resumed is not None
        lab.pause()
        resumed.join(timeout=2)

    def test_a_t0_action_that_fails_on_resume_keeps_the_rig_paused(self):
        lab, pump, cam = _lab(lag=0.05)

        def always_fails():
            raise RuntimeError("no ack")

        lab.scheduler.at(0.0, always_fails, subsystem="flow", name="set")
        lab.start(60).join(timeout=2)
        cam.calls.clear()

        assert lab.resume() is None
        assert lab.clock.is_paused and lab.safety_state is SafetyState.PAUSED
        assert "resume" not in cam.verbs()


class TestSchedulerThreadFailure:
    def test_an_exception_on_the_scheduler_thread_pauses_the_rig(self):
        lab, _, _ = _lab(lag=0.05)

        def boom(*a, **k):
            raise RuntimeError("scheduler blew up")

        lab.scheduler.run = boom
        thread = lab.start(60)
        thread.join(timeout=2)
        assert lab.safety_state is SafetyState.PAUSED


class TestSchedulerPendingAndGate:
    def test_pending_names_only_unfired_due_entries_of_the_given_subsystems(self):
        lab = FlumeLab()
        lab.scheduler.at(0.0, lambda: None, subsystem="flow", name="a")
        lab.scheduler.at(0.0, lambda: None, subsystem="camera", name="b")
        lab.scheduler.at(30.0, lambda: None, subsystem="weir", name="c")
        assert lab.scheduler.pending({"flow", "weir"}, up_to=0.0) == {"flow"}
        lab.scheduler.preroll({"flow"})
        assert lab.scheduler.pending({"flow", "weir"}, up_to=0.0) == set()

    def test_a_failed_action_stays_pending(self):
        lab = FlumeLab()

        def boom():
            raise RuntimeError("x")

        lab.scheduler.at(0.0, boom, subsystem="flow", name="a")
        assert lab.scheduler.preroll({"flow"})
        assert lab.scheduler.pending({"flow"}) == {"flow"}

    def test_a_gate_override_replaces_the_default_gate(self):
        lab = FlumeLab()
        lab.pause()                       # the default gate refuses while PAUSED
        fired = []
        lab.scheduler.at(0.0, lambda: fired.append(1), subsystem="flow", name="a")
        assert lab.scheduler.preroll({"flow"}, gate=lambda: None) == []
        assert fired == [1]


class TestInterruptedSpinUp:
    """A BaseException (Ctrl-C) during the lag must not leave the pump running."""

    def test_keyboard_interrupt_during_the_wait_repauses_the_pump(self, monkeypatch):
        lab, pump, cam = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        pump.calls.clear(), cam.calls.clear()

        def interrupted(timeout=None):
            raise KeyboardInterrupt

        monkeypatch.setattr(lab._spinup_abort, "wait", interrupted)
        with pytest.raises(KeyboardInterrupt):
            lab.resume()

        assert pump.verbs() == ["resume", "pause"]
        assert "resume" not in cam.verbs()
        assert lab.safety_state is SafetyState.PAUSED and lab.clock.is_paused

    def test_keyboard_interrupt_inside_a_subsystem_resume(self):
        lab, pump, cam = _lab(lag=5)
        lab.clock.start()
        lab.pause()
        pump.calls.clear()
        pump.resume_result = KeyboardInterrupt()
        with pytest.raises(KeyboardInterrupt):
            lab.resume()
        assert pump.verbs()[-1] == "pause"
        assert lab.clock.is_paused


class TestMonitorDuringTheLag:
    def test_a_pause_trigger_that_fires_again_during_the_lag_is_seen(self):
        from laguna.safety import CallableTrigger, SafetyTier

        lab, pump, cam = _lab(lag=3)
        flag = {"on": False}
        lab.safety_monitor._poll_s = 0.02
        lab.safety_monitor.add(CallableTrigger(lambda: flag["on"], tier=SafetyTier.PAUSE, name="level"))
        lab.clock.start()
        lab.safety_monitor.start()
        try:
            # First trip: the monitor pauses the rig and latches PAUSE as fired.
            flag["on"] = True
            deadline = time.time() + 2
            while lab.safety_state is not SafetyState.PAUSED and time.time() < deadline:
                time.sleep(0.01)
            assert lab.safety_state is SafetyState.PAUSED
            flag["on"] = False
            time.sleep(0.1)
            pump.calls.clear()

            result = {}
            t = threading.Thread(target=lambda: result.setdefault("r", lab.resume()))
            t.start()
            time.sleep(0.4)  # well inside the 3 s lag
            flag["on"] = True
            t.join(timeout=2)

            assert not t.is_alive(), "the monitor trip during the lag was ignored"
            assert result["r"] is None
            assert pump.verbs()[-1] == "pause"
            assert lab.safety_state is SafetyState.PAUSED and lab.clock.is_paused
        finally:
            lab.safety_monitor.stop()


class TestPauseResumeDuringStartPreroll:
    def test_the_t0_action_fires_once_and_one_spin_up_runs(self):
        lab, pump, cam = _lab(lag=0.6)
        fired = []

        def slow():
            fired.append(time.time())
            time.sleep(0.5)

        lab.scheduler.at(0.0, slow, subsystem="flow", name="setpoint")
        before = len([r for r in _event_rows(lab) if "spinup_wait" in r])
        thread = lab.start(60)
        time.sleep(0.15)          # inside the slow T=0 action
        lab.pause()
        lab.resume()
        assert len(fired) == 1, "the T=0 action ran twice"
        waits = [r for r in _event_rows(lab) if "spinup_wait" in r]
        assert len(waits) - before == 1
        assert lab.safety_state is SafetyState.RUNNING and not lab.clock.is_paused
        lab.pause()
        thread.join(timeout=3)


def _event_rows(lab):
    return [line for line in open(lab.event_log._path).read().splitlines()]


class TestSimulatedRehearsal:
    def test_estop_mid_lag_in_a_simulated_lab(self, tmp_path):
        lab = FlumeLab(simulate=True)
        pump = Probe(lab, "flow", lag=2)
        lab.add(pump)
        lab.clock.start()
        lab.pause()
        pump.calls.clear()

        t = threading.Thread(target=lab.resume)
        t.start()
        time.sleep(0.3)
        lab.estop(reason="rehearsal")
        t.join(timeout=3)

        assert not t.is_alive()
        assert lab.safety_state is SafetyState.ESTOPPED and lab.clock.is_paused
        assert pump.verbs()[0] == "resume" and "estop" in pump.verbs()
