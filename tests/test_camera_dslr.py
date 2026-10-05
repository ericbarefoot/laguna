"""Tests for DslrCameraSubsystem: N cameras bound by serial, all-or-nothing
connect, safety verbs, and the simulated path (see laguna.simulation)."""

import pytest

from laguna.camera.dslr import DslrCameraSubsystem

from dslr_fixtures import FakeBody, FakeBus, install_fake_gphoto2

EXPOSURE = {"iso": "800", "aperture": "5.6", "shutter": "1/125"}


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 8.0


def three_cameras(tmp_path):
    return {
        name: {"serial": serial, "exposure": EXPOSURE, "output_dir": str(tmp_path / name)}
        for name, serial in (("Hangang", "111"), ("Nakdong", "222"), ("Geum", "333"))
    }


@pytest.fixture
def bus(monkeypatch):
    # Deliberately not in config order: binding must not depend on port order.
    bus = FakeBus({
        "usb:001,007": FakeBody("333"),
        "usb:001,005": FakeBody("222"),
        "usb:001,006": FakeBody("111"),
    })
    install_fake_gphoto2(monkeypatch, bus)
    return bus


def connected(tmp_path):
    dslr = DslrCameraSubsystem(cameras=three_cameras(tmp_path))
    log = _FakeEventLog()
    dslr.attach_event_log(log, _FakeClock())
    assert dslr.connect()
    return dslr, log


class TestConfig:
    def test_real_camera_without_serial_is_rejected(self):
        with pytest.raises(ValueError, match="serial"):
            DslrCameraSubsystem(cameras={"A": {"exposure": EXPOSURE}})

    def test_incomplete_exposure_is_rejected(self):
        with pytest.raises(ValueError, match="shutter"):
            DslrCameraSubsystem(cameras={"A": {"serial": "1", "exposure": {"iso": "800", "aperture": "8"}}})


    def test_capture_target_applies_to_every_camera(self):
        cams = {n: {"serial": n, "exposure": EXPOSURE} for n in ("A", "B")}
        dslr = DslrCameraSubsystem(cameras=cams, capture_target="ram")
        assert {c.capture_target for c in dslr.cameras.values()} == {"ram"}

    def test_per_camera_capture_target_is_rejected(self):
        """libgphoto2 holds one target per host: mixing silently sent a
        cardless body to "card" on real hardware (90 s hang per capture)."""
        cams = {n: {"serial": n, "exposure": EXPOSURE} for n in ("A", "B")}
        cams["B"]["capture_target"] = "card"
        with pytest.raises(ValueError, match="once for"):
            DslrCameraSubsystem(cameras=cams, capture_target="ram")


class TestConnect:
    def test_three_cameras_bind_by_serial_regardless_of_port_order(self, bus, tmp_path):
        dslr, _ = connected(tmp_path)
        ports = {n: c.port for n, c in dslr.cameras.items()}
        assert ports == {"Hangang": "usb:001,006", "Nakdong": "usb:001,005", "Geum": "usb:001,007"}

    def test_one_missing_camera_fails_the_whole_connect(self, bus, tmp_path):
        del bus.ports["usb:001,007"]
        dslr = DslrCameraSubsystem(cameras=three_cameras(tmp_path))
        assert dslr.connect() is False
        assert not any(c.is_connected for c in dslr.cameras.values())
        assert all(b.open_sessions == 0 for b in bus.ports.values())

    def test_resume_rebinds_after_cameras_swap_ports(self, bus, tmp_path):
        dslr, _ = connected(tmp_path)
        a, b = bus.ports["usb:001,005"], bus.ports["usb:001,006"]
        bus.ports = {"usb:001,010": a, "usb:001,011": b, "usb:001,007": bus.ports["usb:001,007"]}
        assert dslr.resume() is None
        assert dslr.cameras["Nakdong"].port == "usb:001,010"
        assert dslr.cameras["Hangang"].port == "usb:001,011"

    def test_resume_reports_a_camera_that_did_not_come_back(self, bus, tmp_path):
        dslr, _ = connected(tmp_path)
        del bus.ports["usb:001,007"]
        assert "failed to reconnect" in dslr.resume()


class TestOutputRoot:
    def test_run_directory_only_applies_to_cameras_without_explicit_output_dir(self, tmp_path):
        cams = three_cameras(tmp_path)
        del cams["Geum"]["output_dir"]
        dslr = DslrCameraSubsystem(cameras=cams)
        dslr.set_output_root(tmp_path / "run")
        assert dslr.cameras["Geum"].output_dir == tmp_path / "run" / "Geum"
        assert dslr.cameras["Hangang"].output_dir == tmp_path / "Hangang"


class TestCaptureAll:
    def test_every_camera_captures_with_runtime_in_the_name(self, bus, tmp_path):
        dslr, log = connected(tmp_path)
        records = dslr.capture_all(runtime_s=12.5)
        assert all(r.ok for r in records.values())
        assert records["Geum"].files[0].name.startswith("Geum_t0000012.5_")
        assert [row[2] for row in log.rows].count("capture") == 3

    def test_one_failure_is_reported_and_logged_others_still_saved(self, bus, tmp_path):
        dslr, log = connected(tmp_path)
        bus.ports["usb:001,007"].fail_capture = True
        records = dslr.capture_all(runtime_s=1.0)
        assert not records["Geum"].ok
        assert records["Hangang"].ok and records["Nakdong"].ok
        failed = [row for row in log.rows if row[2] == "capture_failed"]
        assert len(failed) == 1 and "camera=Geum" in failed[0][4]

    def test_raw_plus_jpeg_logs_jpeg_as_file(self, bus, tmp_path):
        cams = three_cameras(tmp_path)
        for cfg in cams.values():
            cfg["imageformat"] = "RAW + L"
        dslr = DslrCameraSubsystem(cameras=cams)
        log = _FakeEventLog()
        dslr.attach_event_log(log, _FakeClock())
        assert dslr.connect()
        dslr.capture_all(runtime_s=1.0)
        notes = [row[4] for row in log.rows if row[2] == "capture"]
        assert all(".jpg raw=" in n and n.endswith(".cr2") for n in notes)


class TestSafetyVerbs:
    def test_pause_keeps_cameras_connected(self, bus, tmp_path):
        dslr, _ = connected(tmp_path)
        assert dslr.pause() is None
        assert all(c.is_connected for c in dslr.cameras.values())

    def test_stop_and_estop_disconnect(self, bus, tmp_path):
        for verb in ("stop", "estop"):
            dslr, _ = connected(tmp_path)
            assert getattr(dslr, verb)() is None
            assert not any(c.is_connected for c in dslr.cameras.values())
            assert all(b.open_sessions == 0 for b in bus.ports.values())

    def test_estop_never_raises_even_unconnected(self):
        assert DslrCameraSubsystem(cameras={}).estop() is None


class TestVerbsNeverBlockOnAHungCapture:
    """FlumeLab pauses subsystems one after another, so a DSLR verb that
    waited on a hung capture would delay the gantry's pause behind it."""

    def _hung(self, bus, tmp_path):
        import threading

        dslr, log = connected(tmp_path)
        body = bus.ports["usb:001,007"]
        body.capture_gate = threading.Event()
        worker = threading.Thread(target=dslr.capture_all, kwargs={"runtime_s": 1.0})
        worker.start()
        assert body.capture_started.wait(timeout=2)
        return dslr, log, body, worker

    def test_pause_returns_at_once_with_a_note(self, bus, tmp_path):
        import time

        dslr, _, body, worker = self._hung(bus, tmp_path)
        t = time.monotonic()
        note = dslr.pause()
        assert time.monotonic() - t < 0.1
        assert "in flight" in note
        body.capture_gate.set()
        worker.join(timeout=5)

    @pytest.mark.parametrize("verb", ["stop", "estop"])
    def test_halt_defers_disconnect_until_the_capture_ends(self, bus, tmp_path, verb):
        dslr, _, body, worker = self._hung(bus, tmp_path)
        assert "in flight" in getattr(dslr, verb)()
        assert body.open_sessions == 1, "closed a camera another thread is using"
        body.capture_gate.set()
        worker.join(timeout=5)
        assert all(b.open_sessions == 0 for b in bus.ports.values())

    def test_a_second_trigger_is_reported_missed_not_queued(self, bus, tmp_path):
        dslr, log, body, worker = self._hung(bus, tmp_path)
        records = dslr.capture_all(runtime_s=2.0)
        assert all(r.error == "previous capture still in progress" for r in records.values())
        body.capture_gate.set()
        worker.join(timeout=5)

    def test_resume_refuses_while_a_capture_is_in_flight(self, bus, tmp_path):
        dslr, _, body, worker = self._hung(bus, tmp_path)
        assert "still in flight" in dslr.resume()
        body.capture_gate.set()
        worker.join(timeout=5)


class TestSimulated:
    def _connected(self):
        dslr = DslrCameraSubsystem(cameras={"Camera1": {}, "Camera2": {}}, simulated=True)
        dslr.connect()
        return dslr

    def test_connect_succeeds_without_gphoto2(self):
        dslr = DslrCameraSubsystem(simulated=True)
        assert dslr.connect() is True
        assert dslr.cameras == {}

    def test_capture_all_returns_a_placeholder_per_camera(self):
        records = self._connected().capture_all(runtime_s=3.0)
        assert set(records) == {"Camera1", "Camera2"}
        assert str(records["Camera1"].files[0]).startswith("<simulated>/Camera1_t")

    def test_capture_all_logs_each_camera(self):
        dslr = self._connected()
        log = _FakeEventLog()
        dslr.attach_event_log(log, _FakeClock())
        dslr.capture_all()
        assert {(r[1], r[2]) for r in log.rows} == {("dslr_cameras", "capture")}
        assert len(log.rows) == 2

    def test_capture_before_connect_is_a_failure(self):
        dslr = DslrCameraSubsystem(cameras={"Camera1": {}}, simulated=True)
        assert not dslr.capture_all()["Camera1"].ok

    def test_safety_verbs_are_safe(self):
        dslr = self._connected()
        for verb in ("pause", "resume", "stop", "estop"):
            assert getattr(dslr, verb)() is None


class TestRunnerEscalation:
    def _lab(self, tmp_path):
        import yaml

        from laguna.experiment.runner import setup_run

        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump({
            "dslr_cameras": {"interval_s": 20, "cameras": {"Camera1": {}, "Camera2": {}}},
        }))
        lab = setup_run(str(path), simulate=True)
        action = next(job["action"] for job in lab.scheduler._recurring
                      if job["subsystem"] == "dslr_cameras")
        escalations = []
        lab.escalate = lambda problem, *a, **k: escalations.append(problem)
        return lab, action, escalations

    def test_successful_capture_does_not_escalate(self, tmp_path):
        lab, action, escalations = self._lab(tmp_path)
        action()
        assert escalations == []

    def test_a_missed_capture_pauses_the_lab(self, tmp_path):
        """A missed frame is a stop-and-fix event, not a log line."""
        lab, action, escalations = self._lab(tmp_path)
        lab.dslr_cameras.disconnect()
        action()
        assert len(escalations) == 1
        assert "Camera1: not connected" in escalations[0]
