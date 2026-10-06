"""Safety verbs on the passive subsystems: gauge, rangefinders, Pi cameras (GH #42).

``FlumeLab._for_each_subsystem()`` used to skip a subsystem lacking a verb with
no log line, so a pause/estop on a rig containing one looked identical to a
clean halt. These subsystems now implement all four verbs, and a future one
that doesn't is reported in the event log.
"""

import pytest

from laguna import FlumeLab
from laguna.camera.network import CameraArray
from laguna.gauge import SaflWaterLevelSensor
from laguna.rangefinder import OD2000Rangefinder, WTT12LRangefinder
from laguna.rangefinder import subsystem as rangefinder_module
from laguna.safety import Quiescible
from laguna.timing import EventLog
from tests.mqtt_fixtures import FakeMqttSubscriber

VERBS = ("pause", "resume", "stop", "estop")


def _od2000(**extra) -> OD2000Rangefinder:
    config = {"pdin_port": 2, "al1342_host": "192.168.1.251", **extra}
    return OD2000Rangefinder(config)


@pytest.fixture
def writes(monkeypatch):
    """Record every IO-Link write as (value, timeout)."""
    calls = []
    monkeypatch.setattr(
        rangefinder_module, "write_acyclic",
        lambda host, port, index, subindex, value, timeout=5.0: calls.append((value, timeout)),
    )
    return calls


class TestEveryVerbExists:
    @pytest.mark.parametrize("make", [
        lambda: _od2000(),
        lambda: WTT12LRangefinder({"pdin_port": 7}),
        lambda: SaflWaterLevelSensor({"simulated": True}, FakeMqttSubscriber()),
        lambda: CameraArray(hosts=["pi1"]),
    ], ids=["od2000", "wtt12l", "gauge", "pi_cameras"])
    def test_implements_the_quiescible_protocol(self, make):
        subsystem = make()
        assert isinstance(subsystem, Quiescible)
        for verb in VERBS:
            getattr(subsystem, verb)()      # must not raise, connected or not


class TestOd2000Emitter:
    def test_pause_switches_the_laser_off_with_a_short_timeout(self, writes):
        rf = _od2000()
        assert rf.pause() is None
        assert writes == [("01", rangefinder_module.SAFETY_WRITE_TIMEOUT_S)]

    def test_resume_restores_the_laser_when_it_was_on(self, writes):
        rf = _od2000()
        rf.activate()
        rf.pause()
        writes.clear()
        assert rf.resume() is None
        assert [v for v, _ in writes] == ["00"]

    def test_resume_leaves_the_laser_off_when_it_was_off(self, writes):
        rf = _od2000()
        rf.pause()
        writes.clear()
        rf.resume()
        assert writes == []

    def test_stop_is_not_resumable(self, writes):
        rf = _od2000()
        rf.activate()
        rf.stop()
        writes.clear()
        rf.resume()
        assert writes == []

    def test_estop_is_not_resumable_even_after_a_pause(self, writes):
        rf = _od2000()
        rf.activate()
        rf.pause()
        rf.estop()
        writes.clear()
        rf.resume()
        assert writes == []

    @pytest.mark.parametrize("verb", ["pause", "stop", "estop"])
    def test_halt_switches_the_laser_off_even_if_we_never_turned_it_on(self, writes, verb):
        """The Pi-side profiler can switch the emitter on without telling this
        object, so a halt must not trust its own bookkeeping."""
        rf = _od2000()
        assert rf._laser_on is False
        getattr(rf, verb)()
        assert [v for v, _ in writes] == ["01"]

    @pytest.mark.parametrize("verb", ["pause", "stop", "estop"])
    def test_an_unreachable_al1342_is_reported_not_raised(self, monkeypatch, verb):
        def boom(*args, **kwargs):
            raise ConnectionError("AL1342 unreachable")

        monkeypatch.setattr(rangefinder_module, "write_acyclic", boom)
        note = getattr(_od2000(), verb)()
        assert note and "emitter state unknown" in note and "unreachable" in note

    def test_estop_without_a_configured_host_reports_instead_of_raising(self):
        rf = OD2000Rangefinder({"pdin_port": 2})
        note = rf.estop()
        assert note and "emitter state unknown" in note

    def test_a_failed_restore_is_reported(self, monkeypatch, writes):
        rf = _od2000()
        rf.activate()
        rf.pause()

        def boom(*args, **kwargs):
            raise ConnectionError("AL1342 unreachable")

        monkeypatch.setattr(rangefinder_module, "write_acyclic", boom)
        note = rf.resume()
        assert note and "not restored" in note

    def test_simulated_rangefinder_does_no_io_but_tracks_state(self, writes):
        rf = _od2000(simulated=True)
        rf.activate()
        rf.pause()
        rf.resume()
        assert writes == []
        assert rf._laser_on is True


class TestPassiveVerbsAreNoOps:
    def test_wtt12l_has_no_emitter_to_switch(self, writes):
        rf = WTT12LRangefinder({"pdin_port": 7, "al1342_host": "h"})
        for verb in VERBS:
            assert getattr(rf, verb)() is None
        assert writes == []

    def test_gauge_and_pi_cameras_return_nothing(self):
        for subsystem in (
            SaflWaterLevelSensor({"simulated": True}, FakeMqttSubscriber()),
            CameraArray(hosts=["pi1"]),
        ):
            for verb in VERBS:
                assert getattr(subsystem, verb)() is None


class TestLabReportsAMissingVerb:
    """A subsystem that cannot be quiesced must be visible in the event log."""

    class NoVerbs:
        subsystem_name = "mystery"

    def test_skipped_subsystem_is_logged_not_silent(self, tmp_path):
        log = tmp_path / "events.csv"
        lab = FlumeLab()
        lab.event_log = EventLog(str(log))
        lab.add(self.NoVerbs())
        lab.clock.start()
        lab.pause()
        text = log.read_text()
        assert "mystery" in text
        assert "does not implement pause()" in text

    def test_real_subsystems_with_all_verbs_are_not_flagged(self, tmp_path, writes):
        log = tmp_path / "events.csv"
        lab = FlumeLab()
        lab.event_log = EventLog(str(log))
        lab.add(_od2000())
        lab.clock.start()
        lab.pause()
        assert "does not implement" not in log.read_text()
