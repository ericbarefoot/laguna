"""Smoke tests for SaflWeirController — construct + call each safety verb, disconnected.

safl_ocean_hardware is not installed in CI/dev, so connect() always fails here;
that is exactly the "disconnected" state these verbs must survive without
raising (a pause/stop/estop that throws leaves the rest of the rig running).
"""

from laguna.weir import SaflWeirController


class TestSaflWeirControllerDisconnected:
    def test_construct(self):
        controller = SaflWeirController({})
        assert controller.subsystem_name == "weir"

    def test_connect_fails_without_hardware_driver(self):
        controller = SaflWeirController({})
        assert controller.connect() is False

    def test_pause_never_raises(self):
        controller = SaflWeirController({})
        note = controller.pause()
        assert note is not None

    def test_resume_never_raises(self):
        controller = SaflWeirController({})
        controller.resume()  # nothing to restore; must not raise

    def test_stop_never_raises(self):
        controller = SaflWeirController({})
        note = controller.stop()
        assert note is not None

    def test_estop_never_raises(self):
        controller = SaflWeirController({})
        note = controller.estop()
        assert note is not None

    def test_disconnect_is_safe_when_never_connected(self):
        controller = SaflWeirController({})
        controller.disconnect()  # must not raise

    def test_get_status_reports_disconnected(self):
        controller = SaflWeirController({})
        status = controller.get_status()
        assert status == {"is_connected": False, "elevation_mm": None}


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 3.0


class _FakeMotor:
    def move_to_position(self, mm):
        return True


class TestSaflWeirControllerLogging:
    """go_to_elevation() logs its own target_mm — see laguna.subsystem_logging."""

    def _connected_controller(self):
        controller = SaflWeirController({})
        controller._is_connected = True
        controller._motor = _FakeMotor()
        return controller

    def test_go_to_elevation_logs_after_attach(self):
        controller = self._connected_controller()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        controller.go_to_elevation(42.0)

        assert event_log.rows == [(3.0, "weir", "go_to_elevation", "ok", "target_mm=42.00")]

    def test_go_to_elevation_does_not_log_before_attach(self):
        controller = self._connected_controller()
        controller.go_to_elevation(42.0)  # must not raise even though never attached


class TestSaflWeirControllerSimulated:
    """simulated: True builds a SimulatedTeknicMotor instead of the real
    driver — commands succeed (and log normally), readings come back NaN
    rather than a fabricated elevation. See laguna.simulation."""

    def test_connect_succeeds_without_safl_ocean_hardware(self):
        controller = SaflWeirController({"simulated": True})
        assert controller.connect() is True
        assert controller._is_connected is True

    def test_go_to_elevation_succeeds_and_logs(self):
        controller = SaflWeirController({"simulated": True})
        controller.connect()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.go_to_elevation(300.0) is True
        assert event_log.rows == [(3.0, "weir", "go_to_elevation", "ok", "target_mm=300.00")]

    def test_get_elevation_is_nan_not_a_fabricated_reading(self):
        import math

        controller = SaflWeirController({"simulated": True})
        controller.connect()
        controller.go_to_elevation(300.0)
        assert math.isnan(controller.get_elevation())

    def test_get_status_elevation_is_nan(self):
        import math

        controller = SaflWeirController({"simulated": True})
        controller.connect()
        status = controller.get_status()
        assert status["is_connected"] is True
        assert math.isnan(status["elevation_mm"])

    def test_home_succeeds_instantly(self):
        controller = SaflWeirController({"simulated": True})
        controller.connect()
        assert controller.home() is True

    def test_safety_verbs_still_work(self):
        controller = SaflWeirController({"simulated": True})
        controller.connect()
        assert controller.pause() is None
        assert controller.stop() is None
        assert controller.estop() is None
