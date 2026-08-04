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
