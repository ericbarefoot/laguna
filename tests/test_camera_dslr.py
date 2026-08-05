"""Tests for DslrCameraSubsystem's simulated: True path — connect()/
capture_all() succeed and log normally without real gphoto2/USB, returning
placeholder filenames rather than fabricated images. See laguna.simulation's
module docstring.
"""

from laguna.camera.dslr import DslrCameraSubsystem


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 8.0


class TestDslrCameraSubsystemSimulated:
    def _connected(self):
        dslr = DslrCameraSubsystem(simulated=True)
        dslr._config = {"cameras": {"Camera1": {}, "Camera2": {}}}
        dslr.connect()
        return dslr

    def test_connect_succeeds_without_gphoto2(self):
        dslr = DslrCameraSubsystem(simulated=True)
        assert dslr.connect() is True
        assert dslr._is_connected is True
        assert dslr._camera_manager is None

    def test_capture_all_returns_a_placeholder_path_per_camera(self):
        dslr = self._connected()
        results = dslr.capture_all()
        assert set(results) == {"Camera1", "Camera2"}
        assert str(results["Camera1"]) == "<simulated>/Camera1.jpg"

    def test_capture_all_logs_each_camera(self):
        dslr = self._connected()
        event_log = _FakeEventLog()
        dslr.attach_event_log(event_log, _FakeClock())

        dslr.capture_all()

        actions = {(row[1], row[2]) for row in event_log.rows}
        assert actions == {("dslr_cameras", "capture")}
        assert len(event_log.rows) == 2

    def test_capture_all_before_connect_returns_empty(self):
        dslr = DslrCameraSubsystem(simulated=True)
        dslr._config = {"cameras": {"Camera1": {}}}
        assert dslr.capture_all() == {}

    def test_disconnect_is_safe(self):
        dslr = self._connected()
        dslr.disconnect()  # must not raise even with no real _camera_manager
        assert dslr._is_connected is False

    def test_get_status_reports_connected_with_zero_real_cameras(self):
        dslr = self._connected()
        status = dslr.get_status()
        assert status["is_connected"] is True
        assert status["num_cameras"] == 0  # no real _camera_manager to count
