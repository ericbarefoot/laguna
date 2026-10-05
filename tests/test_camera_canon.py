"""Tests for laguna.camera.canon against a fake gphoto2 (tests/dslr_fixtures.py).

Cover the things that would silently cost data on a real run: binding to
the wrong body, a re-enumerated port, exposure settings that don't take,
and card files deleted before their download was verified.
"""

import pytest

from laguna.camera.canon import (
    CameraIdentityError,
    CameraNotReadyError,
    CanonDslr,
    Exposure,
    port_from_device,
    udev_rule,
    usb_path_for_port,
)

from dslr_fixtures import FakeBody, FakeBus, install_fake_gphoto2

EXPOSURE = Exposure(iso="800", aperture="5.6", shutter="1/125")


def make_cam(tmp_path, serial="111", **kw):
    return CanonDslr(name="cam", serial=serial, exposure=EXPOSURE,
                     output_dir=str(tmp_path / "out"), **kw)


@pytest.fixture
def bus(monkeypatch):
    bus = FakeBus({"usb:001,005": FakeBody("111"), "usb:001,006": FakeBody("222")})
    install_fake_gphoto2(monkeypatch, bus)
    return bus


class TestBindBySerial:
    def test_binds_to_the_body_with_the_configured_serial(self, bus, tmp_path):
        cam = make_cam(tmp_path, serial="222")
        cam.connect()
        assert cam.port == "usb:001,006"

    def test_other_bodies_are_closed_again(self, bus, tmp_path):
        make_cam(tmp_path, serial="222").connect()
        assert bus.ports["usb:001,005"].open_sessions == 0
        assert bus.ports["usb:001,006"].open_sessions == 1

    def test_missing_serial_refuses_and_lists_what_was_seen(self, bus, tmp_path):
        with pytest.raises(CameraIdentityError, match=r"999.*usb:001,005=111"):
            make_cam(tmp_path, serial="999").connect()

    def test_follows_the_body_to_a_new_port_after_reenumeration(self, bus, tmp_path):
        cam = make_cam(tmp_path, serial="111")
        cam.connect()
        cam.disconnect()
        bus.move("usb:001,005", "usb:001,009")
        cam.connect()
        assert cam.port == "usb:001,009"

    def test_skip_ports_are_never_opened(self, bus, tmp_path):
        with pytest.raises(CameraIdentityError):
            make_cam(tmp_path, serial="111").connect(skip_ports=("usb:001,005",))

    def test_device_hint_tried_first_but_serial_still_wins(self, bus, tmp_path, monkeypatch):
        monkeypatch.setattr("laguna.camera.canon.port_from_device", lambda d: "usb:001,006")
        cam = make_cam(tmp_path, serial="111", device="/dev/dslr_cam")
        cam.connect()
        assert cam.port == "usb:001,005"


class TestPreflight:
    @pytest.mark.parametrize("key,value,message", [
        ("autoexposuremode", "AV", "mode dial"),
        ("focusmode", "AI Focus", "focus"),
        ("autopoweroff", "1 minute", "auto power-off"),
    ])
    def test_refuses_what_gphoto_cannot_fix(self, monkeypatch, tmp_path, key, value, message):
        install_fake_gphoto2(monkeypatch, FakeBus({"usb:001,005": FakeBody("111", **{key: value})}))
        cam = make_cam(tmp_path)
        with pytest.raises(CameraNotReadyError, match=message):
            cam.connect()
        assert not cam.is_connected

    @pytest.mark.parametrize("shots", ["0", "9"])
    def test_refuses_a_card_without_room(self, monkeypatch, tmp_path, shots):
        """No card / full / locked: a T7 then blocks ~90 s per capture."""
        install_fake_gphoto2(monkeypatch, FakeBus({"usb:001,005": FakeBody("111", availableshots=shots)}))
        with pytest.raises(CameraNotReadyError, match="inserted, not full"):
            make_cam(tmp_path).connect()

    def test_writes_exposure_and_card_target(self, bus, tmp_path):
        make_cam(tmp_path, imageformat="RAW + L").connect()
        s = bus.ports["usb:001,005"].settings
        assert (s["iso"], s["aperture"], s["shutterspeed"]) == ("800", "5.6", "1/125")
        assert s["capturetarget"] == "Memory card"
        assert s["imageformat"] == "RAW + L"

    def test_setting_that_does_not_stick_is_refused(self, bus, tmp_path):
        bus.ports["usb:001,005"].sticky.add("shutterspeed")
        with pytest.raises(CameraNotReadyError, match="shutterspeed read back"):
            make_cam(tmp_path).connect()

    def test_invalid_choice_lists_the_valid_ones(self, bus, tmp_path):
        with pytest.raises(CameraNotReadyError, match="RAW \\+ L"):
            make_cam(tmp_path, imageformat="RAW+JPEG").connect()


class TestClockSync:
    def test_preflight_sets_the_body_clock_to_the_pc(self, bus, tmp_path):
        import time

        make_cam(tmp_path).connect()
        assert abs(int(bus.ports["usb:001,005"].settings["datetimeutc"]) - time.time()) <= 2

    def test_a_body_without_the_setting_still_connects(self, bus, tmp_path, caplog):
        del bus.ports["usb:001,005"].settings["syncdatetime"]
        cam = make_cam(tmp_path)
        cam.connect()
        assert cam.is_connected and "could not sync" in caplog.text


class TestCapture:
    def test_raw_plus_jpeg_downloads_both_verified(self, bus, tmp_path):
        cam = make_cam(tmp_path, imageformat="RAW + L")
        cam.connect()
        record = cam.capture("cam_t1")
        assert record.ok, record.error
        assert sorted(p.name for p in record.files) == ["cam_t1.cr2", "cam_t1.jpg"]
        assert not list((tmp_path / "out").glob("*.part"))

    def test_truncated_download_is_an_error_and_card_copy_kept(self, bus, tmp_path):
        body = bus.ports["usb:001,005"]
        cam = make_cam(tmp_path, card_reserve_shots=100)
        cam.connect()
        body.settings["availableshots"] = "11"
        body.truncate_download = True
        record = cam.capture("cam_t1")
        assert not record.ok
        assert "size mismatch" in record.error
        assert body.card and not body.deleted
        assert not (tmp_path / "out" / "cam_t1.jpg").exists()

    def test_capture_failure_is_reported_not_raised(self, bus, tmp_path):
        cam = make_cam(tmp_path)
        cam.connect()
        bus.ports["usb:001,005"].fail_capture = True
        record = cam.capture("cam_t1")
        assert not record.ok and "capture failed" in record.error

    def test_dropped_camera_is_reported(self, bus, tmp_path):
        cam = make_cam(tmp_path)
        cam.connect()
        del bus.ports["usb:001,005"]
        assert not cam.capture("cam_t1").ok

    def test_refuses_to_overwrite_an_existing_file(self, bus, tmp_path):
        cam = make_cam(tmp_path)
        cam.connect()
        assert cam.capture("same").ok
        record = cam.capture("same")
        assert not record.ok and "refusing to overwrite" in record.error


class TestCardSpace:
    def test_nothing_deleted_while_above_reserve(self, bus, tmp_path):
        cam = make_cam(tmp_path, card_reserve_shots=10)
        cam.connect()
        cam.capture("a")
        assert bus.ports["usb:001,005"].deleted == []

    def test_oldest_verified_shot_freed_below_reserve(self, bus, tmp_path):
        body = bus.ports["usb:001,005"]
        body.settings["availableshots"] = "11"
        cam = make_cam(tmp_path, card_reserve_shots=10, imageformat="RAW + L")
        cam.connect()
        cam.capture("a")  # 10 left: at reserve, keep
        cam.capture("b")  # 9 left: free shot a (both files)
        assert sorted(n for _, n in body.deleted) == ["IMG_0001.CR2", "IMG_0001.JPG"]

    def test_files_laguna_did_not_verify_are_never_deleted(self, bus, tmp_path):
        body = bus.ports["usb:001,005"]
        body.card[("/DCIM/100CANON", "OLD_0001.JPG")] = b"x"
        cam = make_cam(tmp_path, card_reserve_shots=100)
        cam.connect()
        body.settings["availableshots"] = "11"
        cam.capture("a")
        cam.capture("b")
        assert ("/DCIM/100CANON", "OLD_0001.JPG") in body.card


class TestCardlessRamMode:
    @pytest.fixture
    def cardless(self, monkeypatch):
        # A cardless T7 in RAM mode reports its RAM buffer's room (~100k).
        body = FakeBody("111", availableshots="100000")
        install_fake_gphoto2(monkeypatch, FakeBus({"usb:001,005": body}))
        return body

    def test_refuses_a_full_ram_buffer(self, monkeypatch, tmp_path):
        body = FakeBody("111", availableshots="0")
        install_fake_gphoto2(monkeypatch, FakeBus({"usb:001,005": body}))
        with pytest.raises(CameraNotReadyError, match="RAM buffer"):
            make_cam(tmp_path, capture_target="ram").connect()

    def test_connects_with_no_card_and_targets_ram(self, cardless, tmp_path):
        make_cam(tmp_path, capture_target="ram").connect()
        assert cardless.settings["capturetarget"] == "Internal RAM"

    def test_capture_downloads_verified_and_releases_ram(self, cardless, tmp_path):
        cam = make_cam(tmp_path, capture_target="ram", imageformat="RAW + L")
        cam.connect()
        record = cam.capture("a")
        assert record.ok, record.error
        assert len(record.files) == 2
        assert cardless.card == {}, "RAM buffer not released"

    def test_failed_download_is_still_reported(self, cardless, tmp_path):
        cam = make_cam(tmp_path, capture_target="ram")
        cam.connect()
        cardless.truncate_download = True
        assert "size mismatch" in cam.capture("a").error

    def test_unknown_target_is_rejected(self, tmp_path):
        with pytest.raises(ValueError, match="capture_target"):
            make_cam(tmp_path, capture_target="cloud")


class TestDownloadRetry:
    def test_one_transient_failure_is_retried(self, bus, tmp_path, monkeypatch):
        cam = make_cam(tmp_path)
        cam.connect()
        real = cam._download
        calls = []

        def flaky(*args):
            calls.append(args)
            if len(calls) == 1:
                raise OSError("USB hiccup")
            return real(*args)

        monkeypatch.setattr(cam, "_download", flaky)
        assert cam.capture("a").ok
        assert len(calls) == 2


class TestHelpers:
    def test_port_from_device_rejects_non_usb_paths(self, tmp_path):
        assert port_from_device(str(tmp_path)) is None
        assert port_from_device("/dev/dslr_does_not_exist") is None

    def test_usb_path_for_port_reads_sysfs(self, tmp_path):
        dev = tmp_path / "1-9.1"
        dev.mkdir()
        (dev / "busnum").write_text("1\n")
        (dev / "devnum").write_text("5\n")
        assert usb_path_for_port("usb:001,005", sysfs=str(tmp_path)) == "1-9.1"
        assert usb_path_for_port("usb:001,006", sysfs=str(tmp_path)) is None

    def test_udev_rule_matches_physical_path(self):
        rule = udev_rule("dslr_hangang", "1-9.1")
        assert 'KERNELS=="1-9.1"' in rule and 'SYMLINK+="dslr_hangang"' in rule
