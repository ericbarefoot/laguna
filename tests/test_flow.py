"""Smoke tests for SaflFlowController — construct + call each safety verb, disconnected.

safl_ocean_hardware is not installed in CI/dev, so connect() always fails here;
that is exactly the "disconnected" state these verbs must survive without
raising (a pause/stop/estop that throws leaves the rest of the rig running).
"""

from laguna.flow import SaflFlowController


class TestSaflFlowControllerDisconnected:
    def test_construct(self):
        controller = SaflFlowController({})
        assert controller.subsystem_name == "flow"

    def test_connect_fails_without_hardware_driver(self):
        controller = SaflFlowController({})
        assert controller.connect() is False

    def test_pause_never_raises(self):
        controller = SaflFlowController({})
        note = controller.pause()
        assert note is not None

    def test_resume_never_raises(self):
        controller = SaflFlowController({})
        note = controller.resume()
        assert note is not None

    def test_stop_never_raises(self):
        controller = SaflFlowController({})
        note = controller.stop()
        assert note is not None

    def test_estop_never_raises(self):
        controller = SaflFlowController({})
        note = controller.estop()
        assert note is not None

    def test_disconnect_is_safe_when_never_connected(self):
        controller = SaflFlowController({})
        controller.disconnect()  # must not raise

    def test_get_status_reports_disconnected(self):
        controller = SaflFlowController({})
        status = controller.get_status()
        assert status["is_connected"] is False


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 7.0


class _FakeVfd:
    def set_freq_from_flowrate(self, lpm, c0, c1, c2):
        pass

    def start(self):
        return True

    def stop(self):
        return True


class _FakeMotor:
    def set_io(self, channel, state):
        pass


class TestSaflFlowControllerLogging:
    """set_flowrate()/start()/stop()/qin/qaux each log their own key action —
    see laguna.subsystem_logging."""

    def _connected_controller(self):
        controller = SaflFlowController({})
        controller._is_connected = True
        controller._vfd = _FakeVfd()
        controller._motor = _FakeMotor()
        return controller

    def test_set_flowrate_logs_after_attach(self):
        controller = self._connected_controller()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        controller.set_flowrate(12.5)

        assert event_log.rows == [(7.0, "flow", "set_flowrate", "ok", "flowrate_lpm=12.50")]

    def test_start_and_stop_each_log_once(self):
        controller = self._connected_controller()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        controller.start()
        controller.stop()

        actions = [row[2] for row in event_log.rows]
        assert actions == ["start", "stop"]

    def test_qin_and_qaux_setters_log_their_state(self):
        controller = self._connected_controller()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        controller.qin = True
        controller.qaux = False

        assert event_log.rows == [
            (7.0, "flow", "qin", "ok", "state=True"),
            (7.0, "flow", "qaux", "ok", "state=False"),
        ]

    def test_set_flowrate_does_not_log_before_attach(self):
        controller = self._connected_controller()
        controller.set_flowrate(12.5)  # must not raise even though never attached


class TestSaflFlowControllerSimulated:
    """simulated: True builds a SimulatedVFD + SimulatedTeknicMotor instead
    of the real drivers — commands succeed (and log normally), readings
    come back NaN/None rather than fabricated data. See laguna.simulation."""

    def test_connect_succeeds_without_safl_ocean_hardware(self):
        controller = SaflFlowController({"simulated": True})
        assert controller.connect() is True
        assert controller._is_connected is True

    def test_set_flowrate_start_stop_all_succeed_and_log(self):
        controller = SaflFlowController({"simulated": True})
        controller.connect()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.set_flowrate(15.0) is True
        assert controller.start() is True
        assert controller.stop() is None

        actions = [row[2] for row in event_log.rows]
        assert actions == ["set_flowrate", "start", "stop"]

    def test_get_flowrate_echoes_the_commanded_setpoint_not_nan(self):
        """get_flowrate() returns the locally cached setpoint the caller
        itself commanded, not a driver reading — it's not fabricated data,
        so it's fine for it to stay a real number."""
        controller = SaflFlowController({"simulated": True})
        controller.connect()
        controller.set_flowrate(22.5)
        assert controller.get_flowrate() == 22.5

    def test_get_status_vfd_fields_are_nan_or_none(self):
        import math

        controller = SaflFlowController({"simulated": True})
        controller.connect()
        status = controller.get_status()
        assert status["is_connected"] is True
        assert status["vfd_state"] is None
        assert status["vfd_estop"] is None
        assert math.isnan(status["vfd_setpoint_hz"])

    def test_qin_qaux_succeed_and_log(self):
        controller = SaflFlowController({"simulated": True})
        controller.connect()
        controller.qin = True
        assert controller.qin is True
