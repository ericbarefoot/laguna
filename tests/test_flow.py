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
