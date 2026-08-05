"""Tests for CameraArray's simulated: True path — trigger_capture()/
fetch_images() succeed and log normally without real SSH, returning
placeholder filenames and NaN timing fields rather than fabricated data.
See laguna.simulation's module docstring.
"""

import math

from laguna.camera.network import CameraArray


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 5.0


class TestCameraArraySimulated:
    def test_connect_succeeds(self):
        arr = CameraArray(hosts=["pi1.local"], simulated=True)
        assert arr.connect() is True

    def test_trigger_capture_returns_success_with_nan_timing(self):
        arr = CameraArray(hosts=["pi1.local", "pi2.local"], simulated=True)
        results = arr.trigger_capture()
        assert len(results) == 2
        for r in results:
            assert r.success is True
            assert r.filename == "<simulated>"
            assert math.isnan(r.capture_time_mid_pc)
            assert math.isnan(r.latency_ms)

    def test_fetch_images_returns_placeholder_paths_and_logs(self, tmp_path):
        arr = CameraArray(hosts=["pi1.local"], simulated=True)
        event_log = _FakeEventLog()
        arr.attach_event_log(event_log, _FakeClock())

        results = arr.trigger_capture()
        paths = arr.fetch_images(results, tmp_path)

        assert paths["pi1.local"].name == "pi1.local_simulated.jpg"
        assert not paths["pi1.local"].exists()  # placeholder path, no real file
        assert event_log.rows == [(5.0, "pi_cameras", "capture", "ok",
                                    f"host=pi1.local file={paths['pi1.local']}")]

    def test_no_real_ssh_touched(self, monkeypatch):
        """The whole point: a simulated run must never call _connect()."""
        arr = CameraArray(hosts=["pi1.local"], simulated=True)

        def _fail_if_called(*a, **k):
            raise AssertionError("real SSH _connect() must not be called when simulated")

        monkeypatch.setattr(arr, "_connect", _fail_if_called)
        results = arr.trigger_capture()
        arr.fetch_images(results, "/tmp")  # must not raise
