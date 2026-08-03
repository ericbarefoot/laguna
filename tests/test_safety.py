"""Tests for the three-tier safety vocabulary and its estop triggers.

The behaviours worth pinning: one failing subsystem must never prevent the
rest from being made safe, an estop must not require hardware to be
reachable, and re-arming must be impossible while a trigger is still
asserted.
"""

import pytest

from laguna import FlumeLab
from laguna.safety import (
    CallableTrigger,
    EstopMonitor,
    SafetyState,
    SentinelFileTrigger,
)


class RecordingSubsystem:
    """Subsystem double that records which safety verbs it received."""

    def __init__(self, name="fake", fail_on=()):
        self.subsystem_name = name
        self.calls = []
        self._fail_on = set(fail_on)

    def _record(self, verb):
        self.calls.append(verb)
        if verb in self._fail_on:
            raise RuntimeError(f"{self.subsystem_name}.{verb}() deliberately failed")

    def pause(self):
        self._record("pause")

    def resume(self):
        self._record("resume")

    def estop(self):
        self._record("estop")

    def connect(self):
        return True

    def disconnect(self):
        pass

    def get_status(self):
        return {"is_connected": True}


class LegacySubsystem:
    """Predates the protocol: has stop() but none of the new verbs."""

    def __init__(self, name="legacy"):
        self.subsystem_name = name
        self.calls = []

    def stop(self):
        self.calls.append("stop")

    def connect(self):
        return True

    def disconnect(self):
        pass


class TestTriggers:
    def test_sentinel_file_trips_only_while_present(self, tmp_path):
        path = tmp_path / "ESTOP"
        trigger = SentinelFileTrigger(str(path))
        assert trigger.is_tripped() is False
        path.touch()
        assert trigger.is_tripped() is True
        assert trigger.clear() is True
        assert trigger.is_tripped() is False

    def test_clearing_a_missing_sentinel_is_not_an_error(self, tmp_path):
        assert SentinelFileTrigger(str(tmp_path / "nope")).clear() is False

    def test_callable_trigger_wraps_a_predicate(self):
        state = {"tripped": False}
        trigger = CallableTrigger(lambda: state["tripped"], name="probe")
        assert trigger.is_tripped() is False
        state["tripped"] = True
        assert trigger.is_tripped() is True

    def test_a_failing_predicate_does_not_trip(self):
        """A monitoring source that has itself broken must not be able to
        spuriously halt an experiment."""

        def boom():
            raise OSError("sensor unreachable")

        assert CallableTrigger(boom, name="broken").is_tripped() is False

    def test_monitor_reports_the_first_tripped_trigger(self, tmp_path):
        a = tmp_path / "a"
        b = tmp_path / "b"
        monitor = EstopMonitor(
            [SentinelFileTrigger(str(a), name="a"), SentinelFileTrigger(str(b), name="b")]
        )
        assert monitor.tripped_by() is None
        b.touch()
        assert monitor.tripped_by() == "b"

    def test_monitor_fires_once_not_once_per_poll(self, tmp_path):
        """A sentinel that stays on disk must not re-fire continuously."""
        path = tmp_path / "ESTOP"
        fired = []
        monitor = EstopMonitor(
            [SentinelFileTrigger(str(path))], on_trip=fired.append, poll_s=0.01
        )
        path.touch()
        monitor.start()
        try:
            import time

            time.sleep(0.15)          # many poll intervals
        finally:
            monitor.stop()
        assert len(fired) == 1

    def test_monitor_with_no_triggers_does_not_start(self):
        monitor = EstopMonitor([])
        monitor.start()
        assert monitor._thread is None


class TestSafetyVerbs:
    def _lab(self, *subsystems):
        lab = FlumeLab()
        for s in subsystems:
            lab.add(s)
        return lab

    def test_starts_running(self):
        assert self._lab().safety_state is SafetyState.RUNNING

    def test_pause_quiesces_every_subsystem_and_the_clock(self):
        a, b = RecordingSubsystem("gantry"), RecordingSubsystem("flow")
        lab = self._lab(a, b)
        lab.clock.start()
        lab.pause()
        assert a.calls == ["pause"] and b.calls == ["pause"]
        assert lab.clock.is_paused is True
        assert lab.safety_state is SafetyState.PAUSED

    def test_pause_pauses_the_experiment_clock_so_runtime_excludes_it(self):
        """Runtime must mean time under experimental conditions — a schedule
        row at t=600 should fire 600s of real experiment time in, however
        long the pause lasted."""
        import time

        lab = self._lab()
        lab.clock.start()
        time.sleep(0.05)
        lab.pause()
        during = lab.clock.elapsed()
        time.sleep(0.15)
        assert lab.clock.elapsed() == pytest.approx(during, abs=0.02)

    def test_resume_restores_subsystems_and_the_clock(self):
        a = RecordingSubsystem("flow")
        lab = self._lab(a)
        lab.clock.start()
        lab.pause()
        lab.resume_from_pause()
        assert a.calls == ["pause", "resume"]
        assert lab.clock.is_paused is False
        assert lab.safety_state is SafetyState.RUNNING

    def test_stop_is_an_alias_for_pause(self):
        """stop() has always been a pause despite the name; run_blocking()
        and existing scripts call it."""
        a = RecordingSubsystem()
        lab = self._lab(a)
        lab.clock.start()
        lab.stop()
        assert a.calls == ["pause"]
        assert lab.safety_state is SafetyState.PAUSED

    def test_estop_calls_every_subsystem(self):
        a, b = RecordingSubsystem("gantry"), RecordingSubsystem("flow")
        lab = self._lab(a, b)
        lab.estop()
        assert a.calls == ["estop"] and b.calls == ["estop"]
        assert lab.safety_state is SafetyState.ESTOPPED

    def test_estop_stops_motion_before_hydraulics(self):
        """Whatever can hit someone stops first."""
        order = []

        class Ordered(RecordingSubsystem):
            def estop(self):
                order.append(self.subsystem_name)

        lab = self._lab(Ordered("flow"), Ordered("gantry"), Ordered("weir"))
        lab.estop()
        assert order == ["gantry", "weir", "flow"]

    def test_one_failing_subsystem_does_not_abort_the_rest(self):
        """The defect this replaces: emergency_stop() called stop() unguarded
        in a loop, so a Gocator that raised when disconnected meant every
        subsystem after it was never stopped at all."""
        broken = RecordingSubsystem("gocator", fail_on=("estop",))
        after = RecordingSubsystem("flow")
        lab = self._lab(broken, after)
        lab.estop()
        assert broken.calls == ["estop"]
        assert after.calls == ["estop"], "subsystem after the failing one was skipped"
        assert lab.safety_state is SafetyState.ESTOPPED

    def test_a_failing_pause_also_does_not_abort_the_rest(self):
        broken = RecordingSubsystem("weir", fail_on=("pause",))
        after = RecordingSubsystem("flow")
        lab = self._lab(broken, after)
        lab.clock.start()
        lab.pause()
        assert after.calls == ["pause"]

    def test_legacy_subsystem_still_gets_stopped(self):
        """A subsystem predating the protocol must not be silently skipped."""
        legacy = LegacySubsystem("weir")
        lab = self._lab(legacy)
        lab.estop()
        assert legacy.calls == ["stop"]

    def test_emergency_stop_is_a_deprecated_alias(self):
        a = RecordingSubsystem()
        lab = self._lab(a)
        lab.emergency_stop()
        assert a.calls == ["estop"]
        assert lab.safety_state is SafetyState.ESTOPPED

    def test_estop_does_not_disconnect(self):
        """Unlike the old emergency_stop(), so the rig can be inspected and
        re-armed without losing the gantry's position reference."""
        a = RecordingSubsystem()
        lab = self._lab(a)
        lab.estop()
        assert "disconnect" not in a.calls


class TestRearm:
    def _lab(self, tmp_path, gantry=None):
        lab = FlumeLab()
        if gantry is not None:
            lab.add(gantry)
        lab.watch_for_estop(sentinel=str(tmp_path / "ESTOP"))
        return lab

    def test_refuses_while_a_trigger_is_still_asserted(self, tmp_path):
        sentinel = tmp_path / "ESTOP"
        lab = self._lab(tmp_path)
        try:
            sentinel.touch()
            lab.estop()
            assert lab.rearm() is False, "re-armed into a live emergency"
            assert lab.safety_state is SafetyState.ESTOPPED
        finally:
            lab.estop_monitor.stop()

    def test_succeeds_once_the_trigger_is_cleared(self, tmp_path):
        sentinel = tmp_path / "ESTOP"
        lab = self._lab(tmp_path)
        try:
            sentinel.touch()
            lab.estop()
            sentinel.unlink()
            assert lab.rearm() is True
            assert lab.safety_state is SafetyState.RUNNING
        finally:
            lab.estop_monitor.stop()

    def test_rearm_re_enables_the_gantry(self, tmp_path):
        class FakeGantry(RecordingSubsystem):
            def __init__(self):
                super().__init__("gantry")
                self.safe_mode_calls = []

            def set_safe_mode(self, enabled):
                self.safe_mode_calls.append(enabled)
                return True

        gantry = FakeGantry()
        lab = self._lab(tmp_path, gantry)
        try:
            lab.estop()
            assert lab.rearm() is True
            # False re-enables motors and releases Y/Z brakes, in that order.
            assert gantry.safe_mode_calls == [False]
        finally:
            lab.estop_monitor.stop()

    def test_stays_estopped_if_the_gantry_cannot_be_re_armed(self, tmp_path):
        class StubbornGantry(RecordingSubsystem):
            def __init__(self):
                super().__init__("gantry")

            def set_safe_mode(self, enabled):
                return False

        lab = self._lab(tmp_path, StubbornGantry())
        try:
            lab.estop()
            assert lab.rearm() is False
            assert lab.safety_state is SafetyState.ESTOPPED
        finally:
            lab.estop_monitor.stop()

    def test_cannot_resume_from_an_estop(self, tmp_path):
        lab = self._lab(tmp_path)
        try:
            lab.estop()
            with pytest.raises(RuntimeError, match="call rearm"):
                lab.resume_from_pause()
        finally:
            lab.estop_monitor.stop()


class TestEstopMonitorIntegration:
    def test_a_sentinel_file_fires_an_estop_end_to_end(self, tmp_path):
        """The headline path: touch a file from anywhere, everything halts."""
        import time

        sentinel = tmp_path / "ESTOP"
        subsystem = RecordingSubsystem("gantry")
        lab = FlumeLab()
        lab.add(subsystem)
        lab.estop_monitor._poll_s = 0.01
        lab.watch_for_estop(sentinel=str(sentinel))
        try:
            sentinel.touch()
            deadline = time.time() + 2.0
            while lab.safety_state is not SafetyState.ESTOPPED and time.time() < deadline:
                time.sleep(0.01)
            assert lab.safety_state is SafetyState.ESTOPPED
            assert subsystem.calls == ["estop"]
        finally:
            lab.estop_monitor.stop()

    def test_vfd_hardware_estop_is_propagated(self, tmp_path):
        """The pump drive has a real e-stop circuit; software should follow
        it rather than pretending to be the only authority."""
        flow = RecordingSubsystem("flow")
        flow.get_status = lambda: {"is_connected": True, "vfd_estop": True}
        lab = FlumeLab()
        lab.add(flow)
        lab.watch_for_estop(sentinel=None)
        try:
            assert lab.estop_monitor.tripped_by() == "vfd_hardware_estop"
        finally:
            lab.estop_monitor.stop()
