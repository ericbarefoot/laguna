"""Fixed exposure / white balance for the Pi camera array.

Covers the coordinator side (settings, calibration file, CameraArray wiring,
the ``laguna-picam`` CLI) and the Pi agent against a fake picamera2. No Pi is
contacted.
"""

import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import pytest

from laguna.camera import picam
from laguna.camera.exposure import (
    ExposureControls,
    ExposureSettings,
    compare_controls,
    load_calibration,
    save_calibration,
)
from laguna.camera.network import CalibrationResult, CameraArray, CaptureResult

AGENT_PATH = Path(__file__).resolve().parents[1] / "src" / "laguna" / "camera" / "agent.py"

C1 = ExposureControls(8000, 2.0, (1.9, 1.6))
C2 = ExposureControls(9000, 1.5, (2.0, 1.5))


def _load_agent():
    spec = importlib.util.spec_from_file_location("laguna_camera_agent_under_test", AGENT_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 1.0


# --------------------------------------------------------------------------- #
# ExposureControls / compare_controls
# --------------------------------------------------------------------------- #


class TestExposureControls:
    @pytest.mark.parametrize(
        "kwargs",
        [
            dict(exposure_time_us=0, analogue_gain=1.0, awb_gains=(1, 1)),
            dict(exposure_time_us=20_000_000, analogue_gain=1.0, awb_gains=(1, 1)),
            dict(exposure_time_us=8000, analogue_gain=0.5, awb_gains=(1, 1)),
            dict(exposure_time_us=8000, analogue_gain=1.0, awb_gains=(0, 1)),
            dict(exposure_time_us=8000, analogue_gain=1.0, awb_gains=(1,)),
            dict(exposure_time_us=float("nan"), analogue_gain=1.0, awb_gains=(1, 1)),
            dict(exposure_time_us=8000, analogue_gain=True, awb_gains=(1, 1)),
        ],
    )
    def test_rejects_out_of_range(self, kwargs):
        with pytest.raises(ValueError):
            ExposureControls(**kwargs)

    def test_from_dict_roundtrip(self):
        assert ExposureControls.from_dict(C1.to_dict()) == C1

    def test_from_dict_names_missing_keys(self):
        with pytest.raises(ValueError, match="analogue_gain"):
            ExposureControls.from_dict({"exposure_time_us": 8000, "awb_gains": [1, 1]})

    def test_from_dict_rejects_three_gains(self):
        with pytest.raises(ValueError, match="awb_gains"):
            ExposureControls.from_dict(
                {"exposure_time_us": 8000, "analogue_gain": 1, "awb_gains": [1, 1, 1]}
            )


class TestCompareControls:
    def test_match_within_tolerance(self):
        assert compare_controls(C1, 8100, 2.02, [1.9, 1.62]) is None

    def test_reports_each_difference(self):
        msg = compare_controls(C1, 16000, 2.0, [1.9, 1.6])
        assert "exposure_time_us" in msg and "analogue_gain" not in msg

    def test_missing_metadata_is_a_difference(self):
        assert "not reported" in compare_controls(C1, None, 2.0, [1.9, 1.6])
        assert "colour_gain" in compare_controls(C1, 8000, 2.0, None)


# --------------------------------------------------------------------------- #
# ExposureSettings
# --------------------------------------------------------------------------- #


class TestExposureSettings:
    def test_default_is_auto_and_unlocked(self):
        s = ExposureSettings.from_config(None)
        assert s.mode == "auto" and not s.locked
        assert s.controls_for("any.local") is None

    def test_unknown_mode_rejected(self):
        with pytest.raises(ValueError, match="mode"):
            ExposureSettings.from_config({"mode": "semi"})

    def test_manual_global_with_per_host_override(self):
        s = ExposureSettings.from_config(
            {
                "mode": "manual",
                "exposure_time_us": 8000,
                "analogue_gain": 2.0,
                "awb_gains": [1.9, 1.6],
                "per_host": {"pi2.local": {"exposure_time_us": 12000}},
            }
        )
        assert s.controls_for("pi1.local") == C1
        assert s.controls_for("pi2.local") == ExposureControls(12000, 2.0, (1.9, 1.6))

    def test_manual_needs_values(self):
        with pytest.raises(ValueError, match="manual"):
            ExposureSettings.from_config({"mode": "manual"})

    def test_calibrated_requires_file_setting(self):
        with pytest.raises(ValueError, match="calibration_file"):
            ExposureSettings.from_config({"mode": "calibrated"})

    def test_calibrated_missing_file_raises_with_hint(self, tmp_path):
        with pytest.raises(ValueError, match="calibrate"):
            ExposureSettings.from_config(
                {"mode": "calibrated", "calibration_file": str(tmp_path / "nope.json")}
            )

    def test_calibrated_loads_file(self, tmp_path):
        f = save_calibration(tmp_path / "cal.json", {"pi1.local": C1})
        s = ExposureSettings.from_config({"mode": "calibrated", "calibration_file": str(f)})
        assert s.controls_for("pi1.local") == C1

    def test_locked_mode_never_falls_back_to_auto_for_a_host(self, tmp_path):
        f = save_calibration(tmp_path / "cal.json", {"pi1.local": C1})
        s = ExposureSettings.from_config({"mode": "calibrated", "calibration_file": str(f)})
        with pytest.raises(KeyError):
            s.controls_for("pi2.local")
        with pytest.raises(ValueError, match="pi2.local"):
            s.validate_hosts(["pi1.local", "pi2.local"])


# --------------------------------------------------------------------------- #
# Calibration file
# --------------------------------------------------------------------------- #


class TestCalibrationFile:
    def test_roundtrip(self, tmp_path):
        f = save_calibration(tmp_path / "cal.json", {"a": C1, "b": C2}, notes={"warmup_s": 5})
        assert load_calibration(f) == {"a": C1, "b": C2}
        assert json.loads(f.read_text())["notes"] == {"warmup_s": 5}

    def test_refuses_to_overwrite_existing(self, tmp_path):
        f = save_calibration(tmp_path / "cal.json", {"a": C1})
        before = f.read_text()
        with pytest.raises(FileExistsError):
            save_calibration(f, {"a": C2})
        assert f.read_text() == before

    def test_overwrite_replaces_and_leaves_no_temp_files(self, tmp_path):
        f = save_calibration(tmp_path / "cal.json", {"a": C1})
        save_calibration(f, {"a": C2}, overwrite=True)
        assert load_calibration(f) == {"a": C2}
        assert [p.name for p in tmp_path.iterdir()] == ["cal.json"]

    def test_empty_calibration_refused(self, tmp_path):
        with pytest.raises(ValueError):
            save_calibration(tmp_path / "cal.json", {})

    @pytest.mark.parametrize(
        "text", ["not json", "[]", '{"format": 99, "hosts": {}}', '{"format": 1, "hosts": {}}']
    )
    def test_bad_files_rejected(self, tmp_path, text):
        f = tmp_path / "cal.json"
        f.write_text(text)
        with pytest.raises(ValueError):
            load_calibration(f)

    def test_invalid_entry_names_the_host(self, tmp_path):
        f = tmp_path / "cal.json"
        f.write_text(json.dumps({"format": 1, "hosts": {"pi9": {"exposure_time_us": 1}}}))
        with pytest.raises(ValueError, match="pi9"):
            load_calibration(f)


# --------------------------------------------------------------------------- #
# CameraArray wiring
# --------------------------------------------------------------------------- #


def _array(mode_cfg=None, hosts=("pi1.local",)):
    return CameraArray(
        hosts=list(hosts), exposure=ExposureSettings.from_config(mode_cfg) if mode_cfg else None
    )


MANUAL_VALUES = {"exposure_time_us": 8000, "analogue_gain": 2.0, "awb_gains": [1.9, 1.6]}
MANUAL = {"mode": "manual", **MANUAL_VALUES}


def _agent_ok(**extra):
    base = {
        "filename": "/tmp/x.jpg",
        "target_time": 1.0,
        "capture_time_mid": 1.0,
        "capture_duration_ms": 10.0,
        "latency_ms": 0.0,
        "exposure_mode": "locked",
        "exposure_time_us": 8000,
        "analogue_gain": 2.0,
        "colour_gains": [1.9, 1.6],
    }
    base.update(extra)
    return base


class TestCameraArrayExposure:
    def test_default_sends_no_controls(self, monkeypatch):
        arr = _array()
        seen = {}
        monkeypatch.setattr(
            arr, "_exec_agent",
            lambda h, a, t: (seen.setdefault("args", a), ({"filename": "f", "exposure_mode": "auto"}, None))[1],
        )  # fmt: skip
        r = arr._run_capture_on_host("pi1.local", 1.0)
        assert r.success and "--controls" not in seen["args"] and r.exposure_warning is None

    def test_locked_sends_controls_and_accepts_matching_frame(self, monkeypatch):
        arr = _array(MANUAL)
        seen = {}

        def fake(h, a, t):
            seen["args"] = a
            return _agent_ok(), None

        monkeypatch.setattr(arr, "_exec_agent", fake)
        r = arr._run_capture_on_host("pi1.local", 1.0)
        assert "--controls" in seen["args"] and "8000" in seen["args"]
        assert r.success and r.exposure_warning is None
        assert (r.exposure_time_us, r.analogue_gain, r.colour_gains) == (8000, 2.0, [1.9, 1.6])

    def test_locked_flags_a_frame_that_differs(self, monkeypatch):
        arr = _array(MANUAL)
        monkeypatch.setattr(arr, "_exec_agent", lambda *a: (_agent_ok(exposure_time_us=30000), None))
        r = arr._run_capture_on_host("pi1.local", 1.0)
        assert r.success and "exposure_time_us" in r.exposure_warning

    def test_locked_flags_an_agent_that_captured_in_auto(self, monkeypatch):
        arr = _array(MANUAL)
        monkeypatch.setattr(arr, "_exec_agent", lambda *a: (_agent_ok(exposure_mode="auto"), None))
        assert "auto" in arr._run_capture_on_host("pi1.local", 1.0).exposure_warning

    def test_locked_host_without_controls_is_refused_not_captured(self, monkeypatch):
        arr = _array(
            {"mode": "manual", "per_host": {"pi1.local": MANUAL_VALUES}}, hosts=("pi1.local",)
        )
        monkeypatch.setattr(
            arr, "_exec_agent", lambda *a: pytest.fail("must not reach the Pi without controls")
        )
        r = arr._run_capture_on_host("stranger.local", 1.0)
        assert not r.success and "stranger.local" in r.error

    def test_construction_fails_if_a_host_lacks_calibration(self, tmp_path):
        f = save_calibration(tmp_path / "cal.json", {"pi1.local": C1})
        s = ExposureSettings.from_config({"mode": "calibrated", "calibration_file": str(f)})
        with pytest.raises(ValueError, match="pi2.local"):
            CameraArray(hosts=["pi1.local", "pi2.local"], exposure=s)

    def test_from_config_reads_exposure_section(self, tmp_path):
        from laguna.config import Config

        cfg_file = tmp_path / "c.yaml"
        cfg_file.write_text(
            "pi_cameras:\n  hosts: [a.local]\n  exposure:\n    mode: manual\n"
            "    exposure_time_us: 8000\n    analogue_gain: 2.0\n    awb_gains: [1.9, 1.6]\n"
        )
        arr = CameraArray.from_config(Config(str(cfg_file)))
        assert arr.exposure.mode == "manual"
        assert arr.get_status()["exposure_mode"] == "manual"

    def test_fetch_images_logs_exposure_and_mismatch(self, monkeypatch, tmp_path):
        arr = _array(MANUAL)
        log = _FakeEventLog()
        arr.attach_event_log(log, _FakeClock())

        class _Sftp:
            def get(self, remote, local):
                Path(local).write_bytes(b"x")

            def close(self):
                pass

        class _Client:
            def open_sftp(self):
                return _Sftp()

            def close(self):
                pass

        monkeypatch.setattr(arr, "_connect", lambda h: _Client())
        res = CaptureResult(
            hostname="pi1.local", success=True, filename="/tmp/c.jpg", exposure_mode="locked",
            exposure_time_us=30000, analogue_gain=2.0, colour_gains=[1.9, 1.6],
            exposure_warning="exposure_time_us 30000 != 8000",
        )  # fmt: skip
        arr.fetch_images([res], tmp_path)
        actions = [row[1] for row in log.rows]
        assert actions == ["capture", "exposure_mismatch"]
        assert "exposure_time_us=30000" in log.rows[0][3]
        assert "colour_gains=1.9,1.6" in log.rows[0][3]
        assert log.rows[1][3].count(" ") == 2  # host=, file=, detail= — no stray spaces

    def test_log_exposure_settings_locked_and_auto(self):
        log = _FakeEventLog()
        locked = _array(MANUAL)
        locked.attach_event_log(log, _FakeClock())
        locked.log_exposure_settings()
        assert log.rows[0][1] == "exposure_settings"
        assert "mode=manual" in log.rows[0][3] and "exposure_time_us=8000" in log.rows[0][3]

        log2 = _FakeEventLog()
        auto = _array()
        auto.attach_event_log(log2, _FakeClock())
        auto.log_exposure_settings()
        assert "mode=auto" in log2.rows[0][3]

    def test_calibrate_collects_per_host_results(self, monkeypatch):
        arr = _array(hosts=("a.local", "b.local"))

        def fake(host, args, timeout):
            assert args.startswith("calibrate ") and "--warmup-s 5.000" in args
            if host == "b.local":
                return None, "SSH connect: refused"
            return {"exposure_time_us": 8000, "analogue_gain": 2.0, "colour_gains": [1.9, 1.6],
                    "converged": True, "spread": 0.01, "filename": "/tmp/p.jpg"}, None  # fmt: skip

        monkeypatch.setattr(arr, "_exec_agent", fake)
        res = arr.calibrate()
        assert res["a.local"].success and res["a.local"].controls == C1 and res["a.local"].converged
        assert not res["b.local"].success and "refused" in res["b.local"].error

    def test_calibrate_rejects_nonsense_from_agent(self, monkeypatch):
        arr = _array()
        monkeypatch.setattr(
            arr, "_exec_agent",
            lambda *a, **k: ({"exposure_time_us": 0, "analogue_gain": 2.0, "colour_gains": [1, 1]}, None),
        )  # fmt: skip
        assert not arr.calibrate()["pi1.local"].success

    def test_simulated_calibrate_does_not_pretend(self):
        arr = CameraArray(hosts=["pi1.local"], simulated=True)
        res = arr.calibrate()["pi1.local"]
        assert not res.success and "simulated" in res.error

    def test_simulated_capture_log_unchanged_in_locked_mode(self, tmp_path):
        arr = CameraArray(
            hosts=["pi1.local"], simulated=True, exposure=ExposureSettings.from_config(MANUAL)
        )
        log = _FakeEventLog()
        arr.attach_event_log(log, _FakeClock())
        paths = arr.fetch_images(arr.trigger_capture(), tmp_path)
        assert log.rows == [("pi_cameras", "capture", "ok", f"host=pi1.local file={paths['pi1.local']}")]
        assert math.isnan(arr.trigger_capture()[0].latency_ms)


# --------------------------------------------------------------------------- #
# Pi agent, against a fake picamera2
# --------------------------------------------------------------------------- #


class _FakePicamera2:
    instances = []

    def __init__(self):
        self.calls = []
        self.controls = {}
        self.closed = False
        self.metadata = {"ExposureTime": 8000, "AnalogueGain": 2.0, "ColourGains": (1.9, 1.6)}
        _FakePicamera2.instances.append(self)

    def create_still_configuration(self):
        return {"still": True}

    def configure(self, cfg):
        self.calls.append("configure")

    def set_controls(self, controls):
        self.controls.update(controls)
        self.calls.append("set_controls")

    def start(self):
        self.calls.append("start")

    def stop(self):
        self.calls.append("stop")

    def close(self):
        self.closed = True

    def capture_metadata(self):
        self.calls.append("capture_metadata")
        return dict(self.metadata)

    def capture_request(self):
        outer = self

        class _Req:
            def save(self, stream, filename):
                Path(filename).write_bytes(b"jpg")
                outer.calls.append("save")

            def get_metadata(self):
                return dict(outer.metadata)

            def release(self):
                outer.calls.append("release")

        return _Req()


@pytest.fixture
def agent(monkeypatch):
    _FakePicamera2.instances = []
    fake = types.ModuleType("picamera2")
    fake.Picamera2 = _FakePicamera2
    monkeypatch.setitem(sys.modules, "picamera2", fake)
    mod = _load_agent()
    monkeypatch.setattr(mod.time, "sleep", lambda s: None)
    return mod


CONTROLS = {"exposure_time_us": 8000, "analogue_gain": 2.0, "awb_gains": [1.9, 1.6]}


class TestAgentLocked:
    def test_disables_ae_awb_and_applies_values_before_capture(self, agent, tmp_path):
        out = agent.capture_at_time(agent.time.time(), str(tmp_path), CONTROLS)
        cam = _FakePicamera2.instances[0]
        assert cam.controls == {
            "AeEnable": False, "AwbEnable": False, "ExposureTime": 8000,
            "AnalogueGain": 2.0, "ColourGains": (1.9, 1.6),
        }  # fmt: skip
        # settle frames are discarded between starting and capturing
        assert cam.calls.index("start") < cam.calls.index("capture_metadata") < cam.calls.index("save")
        assert cam.calls.count("capture_metadata") == agent.SETTLE_FRAMES
        assert cam.closed and (tmp_path / Path(out["filename"]).name).exists()
        assert out["exposure_mode"] == "locked"
        assert (out["exposure_time_us"], out["analogue_gain"]) == (8000, 2.0)
        assert out["colour_gains"] == [1.9, 1.6]

    def test_reports_what_the_camera_actually_used(self, agent, tmp_path, monkeypatch):
        orig = _FakePicamera2.__init__

        def init(self):
            orig(self)
            self.metadata["ExposureTime"] = 30000

        monkeypatch.setattr(_FakePicamera2, "__init__", init)
        assert agent.capture_at_time(agent.time.time(), str(tmp_path), CONTROLS)["exposure_time_us"] == 30000

    def test_refuses_rather_than_capturing_in_auto_without_picamera2(self, agent, tmp_path, monkeypatch):
        monkeypatch.setitem(sys.modules, "picamera2", None)  # import raises ImportError
        picamzero = types.ModuleType("picamzero")
        picamzero.Camera = lambda: pytest.fail("must not fall back to the auto-exposure camera")
        monkeypatch.setitem(sys.modules, "picamzero", picamzero)
        out = agent.capture_at_time(agent.time.time(), str(tmp_path), CONTROLS)
        assert "error" in out and "picamera2" in out["error"]

    def test_bad_controls_error_before_touching_the_camera(self, agent, tmp_path):
        out = agent.capture_at_time(agent.time.time(), str(tmp_path), {"exposure_time_us": 8000})
        assert "error" in out and not _FakePicamera2.instances

    def test_camera_closed_even_if_capture_fails(self, agent, tmp_path, monkeypatch):
        def boom(self):
            raise RuntimeError("camera busy")

        monkeypatch.setattr(_FakePicamera2, "capture_request", boom)
        with pytest.raises(RuntimeError):
            agent.capture_at_time(agent.time.time(), str(tmp_path), CONTROLS)
        assert _FakePicamera2.instances[0].closed

    def test_target_in_the_past_errors(self, agent, tmp_path):
        out = agent.capture_at_time(agent.time.time() - 60, str(tmp_path), CONTROLS)
        assert "past" in out["error"] and not _FakePicamera2.instances

    def test_no_controls_uses_the_auto_picamzero_path(self, agent, tmp_path, monkeypatch):
        taken = []

        class _Cam:
            def take_photo(self, filename):
                taken.append(filename)

        picamzero = types.ModuleType("picamzero")
        picamzero.Camera = _Cam
        monkeypatch.setitem(sys.modules, "picamzero", picamzero)
        out = agent.capture_at_time(agent.time.time(), str(tmp_path))
        assert out["exposure_mode"] == "auto" and taken and not _FakePicamera2.instances


class TestAgentCalibrate:
    def test_reports_median_values_and_convergence(self, agent, tmp_path):
        out = agent.calibrate(str(tmp_path), warmup_s=0.0)
        cam = _FakePicamera2.instances[0]
        assert cam.controls == {"AeEnable": True, "AwbEnable": True}
        assert out["exposure_time_us"] == 8000 and out["analogue_gain"] == 2.0
        assert out["colour_gains"] == [1.9, 1.6]
        assert out["converged"] is True and Path(out["filename"]).exists()
        assert cam.closed

    def test_unsettled_scene_is_not_converged(self, agent, tmp_path, monkeypatch):
        values = iter([4000, 8000, 12000, 16000] * 50)

        def md(self):
            return {"ExposureTime": next(values), "AnalogueGain": 2.0, "ColourGains": (1.9, 1.6)}

        monkeypatch.setattr(_FakePicamera2, "capture_metadata", md)
        out = agent.calibrate(str(tmp_path), warmup_s=0.0)
        assert out["converged"] is False and out["spread"] > agent.CONVERGED_SPREAD

    def test_missing_picamera2_is_an_error(self, agent, tmp_path, monkeypatch):
        monkeypatch.setitem(sys.modules, "picamera2", None)
        assert "error" in agent.calibrate(str(tmp_path))

    def test_cli_parses_both_subcommands(self, agent):
        p = agent._parser()
        a = p.parse_args(["capture", "12.5", "/d", "--controls", "{}"])
        assert (a.action, a.target_time, a.output_dir, a.controls) == ("capture", 12.5, "/d", "{}")
        assert p.parse_args(["calibrate", "--warmup-s", "9"]).warmup_s == 9.0


# --------------------------------------------------------------------------- #
# laguna-picam
# --------------------------------------------------------------------------- #


class TestPicamExposureFlags:
    def test_exposure_args_fix_shutter_gain_and_awb(self):
        assert picam.exposure_args(C1) == [
            "--shutter", "8000", "--gain", "2", "--awbgains", "1.9,1.6",
        ]  # fmt: skip
        assert picam.exposure_args(None) == []

    def test_still_and_stream_args_include_controls_before_output(self):
        s = picam.still_args(None, None, 90, C1)
        assert s[-2:] == ["-o", "-"] and "--shutter" in s
        v = picam.stream_args(640, 480, 10, 50, C1)
        assert v[-2:] == ["-o", "-"] and "--awbgains" in v
        assert "--shutter" not in picam.still_args(None, None, 90)

    def _cfg(self, tmp_path, extra=""):
        cfg = tmp_path / "c.yaml"
        cfg.write_text(
            "pi_cameras:\n  hosts: [a.local]\n  exposure:\n    mode: manual\n"
            "    exposure_time_us: 8000\n    analogue_gain: 2.0\n    awb_gains: [1.9, 1.6]\n" + extra
        )
        return cfg

    def test_snap_locked_uses_config_controls(self, monkeypatch, tmp_path):
        seen = {}

        def fake_run(cmd, **k):
            seen["cmd"] = cmd
            import subprocess

            return subprocess.CompletedProcess(cmd, 0, b"\xff\xd8x", b"")

        monkeypatch.setattr(picam.subprocess, "run", fake_run)
        rc = picam.main(["--config", str(self._cfg(tmp_path)), "snap", "1", "--locked",
                         "-o", str(tmp_path / "o.jpg")])  # fmt: skip
        assert rc == 0 and "--shutter 8000" in seen["cmd"][-1] and "--awbgains 1.9,1.6" in seen["cmd"][-1]

    def test_locked_in_auto_config_is_an_error(self, tmp_path):
        cfg = tmp_path / "c.yaml"
        cfg.write_text("pi_cameras:\n  hosts: [a.local]\n")
        with pytest.raises(SystemExit):
            picam.main(["--config", str(cfg), "snap", "1", "--locked"])

    def test_explicit_flags_must_come_together_and_not_with_locked(self, tmp_path):
        cfg = self._cfg(tmp_path)
        with pytest.raises(SystemExit):
            picam.main(["--config", str(cfg), "snap", "1", "--gain", "2"])
        with pytest.raises(SystemExit):
            picam.main(["--config", str(cfg), "snap", "1", "--locked", "--gain", "2",
                        "--shutter-us", "8000", "--awbgains", "1,1"])  # fmt: skip


class TestPicamCalibrate:
    def _setup(self, monkeypatch, tmp_path, results, existing=False):
        cal = tmp_path / "cal.json"
        if existing:
            cal.write_text("{}")
        cfg = tmp_path / "c.yaml"
        cfg.write_text(
            f"pi_cameras:\n  hosts: [a.local, b.local]\n  exposure:\n    mode: calibrated\n"
            f"    calibration_file: {cal}\n"
        )
        monkeypatch.setattr(CameraArray, "calibrate", lambda self, warmup_s=None: results)
        monkeypatch.setattr(CameraArray, "fetch_calibration_previews", lambda *a, **k: None)
        return cfg, cal

    @staticmethod
    def _good(host, controls=C1, converged=True):
        return CalibrationResult(hostname=host, success=True, controls=controls,
                                 converged=converged, spread=0.01, filename="/tmp/p.jpg")  # fmt: skip

    def test_reading_config_does_not_need_the_calibration_file_to_exist(self, tmp_path):
        cfg, cal = self._setup_cfg_only(tmp_path)
        assert not cal.exists()
        assert picam.load_pi_settings(str(cfg))["exposure"]["mode"] == "calibrated"

    def _setup_cfg_only(self, tmp_path):
        cal = tmp_path / "cal.json"
        cfg = tmp_path / "c.yaml"
        cfg.write_text(
            f"pi_cameras:\n  hosts: [a.local]\n  exposure:\n    mode: calibrated\n"
            f"    calibration_file: {cal}\n"
        )
        return cfg, cal

    def test_saves_after_confirmation(self, monkeypatch, tmp_path):
        res = {"a.local": self._good("a.local"), "b.local": self._good("b.local", C2)}
        cfg, cal = self._setup(monkeypatch, tmp_path, res)
        assert picam.main(["--config", str(cfg), "calibrate", "--yes"]) == 0
        assert load_calibration(cal) == {"a.local": C1, "b.local": C2}
        # and the saved file is exactly what calibrated mode then loads
        assert CameraArray(
            hosts=["a.local", "b.local"],
            exposure=ExposureSettings.from_config({"mode": "calibrated", "calibration_file": str(cal)}),
        ).exposure.locked

    def test_nothing_saved_if_any_camera_failed(self, monkeypatch, tmp_path):
        res = {"a.local": self._good("a.local"),
               "b.local": CalibrationResult(hostname="b.local", success=False, error="down")}  # fmt: skip
        cfg, cal = self._setup(monkeypatch, tmp_path, res)
        assert picam.main(["--config", str(cfg), "calibrate", "--yes"]) == 1
        assert not cal.exists()

    def test_declined_confirmation_saves_nothing(self, monkeypatch, tmp_path):
        res = {"a.local": self._good("a.local"), "b.local": self._good("b.local")}
        cfg, cal = self._setup(monkeypatch, tmp_path, res)
        monkeypatch.setattr(picam.sys.stdin, "isatty", lambda: True)
        monkeypatch.setattr("builtins.input", lambda prompt="": "n")
        assert picam.main(["--config", str(cfg), "calibrate"]) == 1
        assert not cal.exists()

    def test_non_interactive_without_yes_saves_nothing(self, monkeypatch, tmp_path):
        res = {"a.local": self._good("a.local"), "b.local": self._good("b.local")}
        cfg, cal = self._setup(monkeypatch, tmp_path, res)
        monkeypatch.setattr(picam.sys.stdin, "isatty", lambda: False)
        assert picam.main(["--config", str(cfg), "calibrate"]) == 1
        assert not cal.exists()

    def test_existing_file_untouched_and_cameras_not_even_contacted(self, monkeypatch, tmp_path):
        cfg, cal = self._setup(monkeypatch, tmp_path, {}, existing=True)
        monkeypatch.setattr(
            CameraArray, "calibrate", lambda *a, **k: pytest.fail("must not contact cameras")
        )
        assert picam.main(["--config", str(cfg), "calibrate", "--yes"]) == 2
        assert cal.read_text() == "{}"

    def test_overwrite_flag_replaces(self, monkeypatch, tmp_path):
        res = {"a.local": self._good("a.local"), "b.local": self._good("b.local")}
        cfg, cal = self._setup(monkeypatch, tmp_path, res, existing=True)
        assert picam.main(["--config", str(cfg), "calibrate", "--yes", "--overwrite"]) == 0
        assert set(load_calibration(cal)) == {"a.local", "b.local"}
