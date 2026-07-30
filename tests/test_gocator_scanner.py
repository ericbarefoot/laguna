"""Tests for the Gocator 2690 scanner subsystem.

No hardware and no built GoSdk required: the ctypes layer is replaced with a
fake that records calls and serves synthetic surface messages. What's being
tested is laguna's *policy* — that the encoderless recipe is applied with the
right constants in the right order, that raw counts are scaled correctly, and
that the scan lifecycle cleans up after itself.

Hardware assumptions encoded here (a failure on real hardware means the
assumption is wrong, not the test):

  - Surface Y spacing comes from GoTransform travel speed (mm/s), not from any
    GoSetup accessor. Confirmed on hardware 2026-07-30; see
    docs/reference/gocator/GOCATOR_CONCEPTS.md §2c.
  - The encoderless recipe is TIME trigger + FIXED_LENGTH surface generation +
    SOFTWARE start trigger, with the trigger fired mid-move.
  - Surface resolutions are nanometres, offsets micrometres, grid values
    16-bit signed counts, and 0x8000 (-32768) means "no data".
  - GoTransform_SetSpeed writes to flash, so it must not be re-issued when the
    value is unchanged.
"""

from __future__ import annotations

import ctypes
import time

import numpy as np
import pytest

from laguna.scanner import gosdk as g
from laguna.scanner.gocator import GocatorScanner
from laguna.scanner.pointcloud import SurfaceScan

# ---------------------------------------------------------------------------
# Fake GoSdk
# ---------------------------------------------------------------------------

_HANDLE = 0x1000  # any non-NULL pointer value


def _val(x):
    """Unwrap a ctypes scalar to its Python value (plain values pass through).

    The production code wraps arguments in ctypes scalars (c_int32, c_double,
    ...), which real ctypes would coerce at the FFI boundary; the fake has to
    do that unwrapping itself.
    """
    return x.value if hasattr(x, "value") else x


class FakeGo:
    """Stand-in for the libGoSdk CDLL object.

    Unknown attributes resolve to a recording stub returning kOK, so the
    subsystem can be exercised without enumerating every SDK symbol.
    """

    def __init__(self, owner: "FakeLib"):
        self._owner = owner
        # Sensor-side state the subsystem reads back.
        self.travel_speed = 0.0
        self.frame_rate = 0.0
        self.trigger_source = -1
        self.scan_mode = -1
        self.generation_type = -1
        self.start_trigger = -1
        self.fixed_length = 0.0
        self.length_limit_min = 1.0
        self.length_limit_max = 5000.0
        # Mirrors the real 2690's live-queried ceiling (~443 Hz at stock
        # FOV/exposure), not the datasheet headline rate.
        self.frame_rate_limit_min = 0.001
        self.frame_rate_limit_max = 443.127
        #: Datasets to serve from GoSystem_ReceiveData, one per call.
        self.datasets: list = []
        self._dataset_index = 0

    # -- recording ------------------------------------------------------

    def _record(self, name, args):
        self._owner.calls.append((name, args))

    def __getattr__(self, name):
        # Only reached for names not defined below.
        def stub(*args):
            self._record(name, args)
            return g.kOK

        return stub

    # -- handle accessors ------------------------------------------------

    def GoSensor_Setup(self, sensor):
        self._record("GoSensor_Setup", (sensor,))
        return _HANDLE

    def GoSensor_Transform(self, sensor):
        self._record("GoSensor_Transform", (sensor,))
        return _HANDLE

    def GoSetup_SurfaceGeneration(self, setup):
        self._record("GoSetup_SurfaceGeneration", (setup,))
        return _HANDLE

    # -- setters ---------------------------------------------------------

    def GoSetup_SetScanMode(self, setup, mode):
        self._record("GoSetup_SetScanMode", (setup, mode))
        self.scan_mode = int(_val(mode))
        return g.kOK

    def GoSetup_SetTriggerSource(self, setup, source):
        self._record("GoSetup_SetTriggerSource", (setup, source))
        self.trigger_source = int(_val(source))
        return g.kOK

    def GoSetup_SetFrameRate(self, setup, rate):
        self._record("GoSetup_SetFrameRate", (setup, rate))
        self.frame_rate = float(_val(rate))
        return g.kOK

    def GoTransform_SetSpeed(self, transform, value):
        self._record("GoTransform_SetSpeed", (transform, value))
        self.travel_speed = float(_val(value))
        return g.kOK

    def GoSurfaceGenerationFixedLength_SetLength(self, surface, length):
        self._record("GoSurfaceGenerationFixedLength_SetLength", (surface, length))
        self.fixed_length = float(_val(length))
        return g.kOK

    def GoSurfaceGeneration_SetGenerationType(self, surface, gen_type):
        self._record("GoSurfaceGeneration_SetGenerationType", (surface, gen_type))
        self.generation_type = int(_val(gen_type))
        return g.kOK

    def GoSurfaceGenerationFixedLength_SetStartTrigger(self, surface, trigger):
        self._record(
            "GoSurfaceGenerationFixedLength_SetStartTrigger", (surface, trigger)
        )
        self.start_trigger = int(_val(trigger))
        return g.kOK

    # -- getters ---------------------------------------------------------

    def GoTransform_Speed(self, transform):
        return self.travel_speed

    def GoSetup_FrameRate(self, setup):
        return self.frame_rate

    def GoSetup_TriggerSource(self, setup):
        return self.trigger_source

    def GoSetup_ScanMode(self, setup):
        return self.scan_mode

    def GoSurfaceGenerationFixedLength_LengthLimitMin(self, surface):
        return self.length_limit_min

    def GoSurfaceGenerationFixedLength_LengthLimitMax(self, surface):
        return self.length_limit_max

    def GoSetup_FrameRateLimitMin(self, setup):
        return self.frame_rate_limit_min

    def GoSetup_FrameRateLimitMax(self, setup):
        return self.frame_rate_limit_max

    def GoSurfaceGeneration_GenerationType(self, surface):
        return self.generation_type

    def GoSurfaceGenerationFixedLength_StartTrigger(self, surface):
        return self.start_trigger

    def GoSurfaceGenerationFixedLength_Length(self, surface):
        return self.fixed_length

    # -- data channel ----------------------------------------------------

    def GoSystem_ReceiveData(self, system, dataset_ptr, timeout):
        self._record("GoSystem_ReceiveData", (system, timeout))
        if self._dataset_index >= len(self.datasets):
            # The real SDK blocks for the requested timeout before reporting
            # one; emulate that (capped) so the retry loop doesn't busy-spin.
            time.sleep(min(_val(timeout) / 1_000_000.0, 0.25))
            return g.kERROR_TIMEOUT
        current = self.datasets[self._dataset_index]
        self._dataset_index += 1
        self._owner.current_dataset = current
        # Hand back a non-NULL "handle"; the fake ignores its value.
        dataset_ptr._obj.value = _HANDLE
        return g.kOK

    def GoDataSet_Count(self, dataset):
        return len(self._owner.current_dataset)

    def GoDataSet_At(self, dataset, index):
        # Encode the message index in the returned pointer so message
        # accessors can find their own data.
        return _HANDLE + int(_val(index))

    def GoDataMsg_Type(self, msg):
        return self._owner._msg(msg)["type"]

    def GoDestroy(self, obj):
        self._record("GoDestroy", (obj,))
        return g.kOK

    # -- stamp -----------------------------------------------------------

    def GoStampMsg_Count(self, msg):
        return 1

    def GoStampMsg_At(self, msg, index):
        stamp = g.GoStamp()
        stamp.frameIndex = 7
        stamp.timestamp = 1024
        stamp.ptpTime = 123456
        return ctypes.pointer(stamp)

    # -- uniform surface -------------------------------------------------

    def GoUniformSurfaceMsg_Length(self, msg):
        return self._owner._msg(msg)["rows"]

    def GoUniformSurfaceMsg_Width(self, msg):
        return self._owner._msg(msg)["cols"]

    def GoUniformSurfaceMsg_XResolution(self, msg):
        return self._owner._msg(msg)["x_res"]

    def GoUniformSurfaceMsg_YResolution(self, msg):
        return self._owner._msg(msg)["y_res"]

    def GoUniformSurfaceMsg_ZResolution(self, msg):
        return self._owner._msg(msg)["z_res"]

    def GoUniformSurfaceMsg_XOffset(self, msg):
        return self._owner._msg(msg)["x_off"]

    def GoUniformSurfaceMsg_YOffset(self, msg):
        return self._owner._msg(msg)["y_off"]

    def GoUniformSurfaceMsg_ZOffset(self, msg):
        return self._owner._msg(msg)["z_off"]

    def GoUniformSurfaceMsg_RowAt(self, msg, row):
        data = self._owner._msg(msg)["data"]
        buf = (ctypes.c_int16 * data.shape[1])(*data[int(_val(row))].tolist())
        # Keep a reference so the buffer outlives this call.
        self._owner._buffers.append(buf)
        return ctypes.cast(buf, ctypes.POINTER(ctypes.c_int16))


class FakeLib:
    """Stand-in for laguna.scanner.gosdk.GoSdkLib."""

    def __init__(self):
        self.calls: list = []
        self.go = FakeGo(self)
        self.kapi = None
        self.lib_dir = "/fake/lib"
        self.current_dataset: list = []
        self._buffers: list = []

    def _msg(self, msg):
        """Resolve a fake message 'pointer' back to its dict."""
        value = int(_val(msg))
        return self.current_dataset[value - _HANDLE]

    # Mirror GoSdkLib's helper API.

    def check(self, name, status):
        if status == g.kOK:
            return status
        if status == g.kERROR_TIMEOUT:
            raise g.GoSdkTimeout(name, status)
        raise g.GoSdkError(name, status)

    def call(self, name, *args):
        fn = getattr(self.go, name)
        return self.check(name, fn(*args))

    def handle(self, name, *args):
        result = getattr(self.go, name)(*args)
        if not result:
            raise g.GoSdkError(name, g.kERROR)
        return ctypes.c_void_p(result)

    def parse_ip(self, ip):
        return g.kIpAddress()

    def call_names(self):
        return [name for name, _ in self.calls]


def make_surface_msg(rows=3, cols=4, z_res=1000, z_off=0, y_res=50_000, x_res=125_000):
    """Build a synthetic UNIFORM_SURFACE message dict.

    Raw counts ascend 0,1,2,... so scaled values are trivially predictable.
    """
    data = np.arange(rows * cols, dtype=np.int16).reshape(rows, cols)
    return {
        "type": g.GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE,
        "rows": rows,
        "cols": cols,
        "x_res": x_res,
        "y_res": y_res,
        "z_res": z_res,
        "x_off": 0,
        "y_off": 0,
        "z_off": z_off,
        "data": data,
    }


def make_stamp_msg():
    return {"type": g.GO_DATA_MESSAGE_TYPE_STAMP}


@pytest.fixture
def scanner(monkeypatch):
    """A connected GocatorScanner backed by FakeLib."""
    fake = FakeLib()
    monkeypatch.setattr(
        "laguna.scanner.gocator.GoSdkLib", lambda lib_dir=None: fake
    )
    s = GocatorScanner(
        {
            "ip": "192.168.1.10",
            "travel_speed_mm_s": 20.0,
            "frame_rate_hz": 400.0,
            "fixed_length_mm": 200.0,
            "output_dir": "/tmp/laguna_test_scans",
        }
    )
    assert s.connect() is True
    s._fake = fake  # test convenience
    return s


# ---------------------------------------------------------------------------
# Connection lifecycle
# ---------------------------------------------------------------------------


class TestConnect:
    def test_connect_constructs_and_finds_sensor_by_ip(self, scanner):
        """connect() builds the SDK objects and resolves the sensor by IP."""
        names = scanner._fake.call_names()
        assert "GoSdk_Construct" in names
        assert "GoSystem_Construct" in names
        assert "GoSystem_FindSensorByIpAddress" in names
        assert "GoSensor_Connect" in names
        # FindSensorByIpAddress must precede Connect.
        assert names.index("GoSystem_FindSensorByIpAddress") < names.index(
            "GoSensor_Connect"
        )
        assert scanner.get_status()["is_connected"] is True

    def test_connect_is_idempotent(self, scanner):
        """A second connect() is a no-op, not a second construction."""
        before = len(scanner._fake.calls)
        assert scanner.connect() is True
        assert len(scanner._fake.calls) == before

    def test_missing_sdk_returns_false_not_raises(self, monkeypatch):
        """A missing SDK must not break lab.connect_all()."""

        def boom(lib_dir=None):
            raise FileNotFoundError("no libGoSdk.so")

        monkeypatch.setattr("laguna.scanner.gocator.GoSdkLib", boom)
        s = GocatorScanner({"ip": "192.168.1.10"})
        assert s.connect() is False
        assert s.get_status()["is_connected"] is False

    def test_sdk_error_on_connect_returns_false(self, monkeypatch):
        """An unreachable sensor surfaces as False, not an exception."""
        fake = FakeLib()

        def fail_connect(sensor):
            raise g.GoSdkError("GoSensor_Connect", -1)

        fake.go.GoSensor_Connect = fail_connect
        monkeypatch.setattr(
            "laguna.scanner.gocator.GoSdkLib", lambda lib_dir=None: fake
        )
        s = GocatorScanner({"ip": "192.168.1.10"})
        assert s.connect() is False

    def test_disconnect_destroys_handles(self, scanner):
        scanner.disconnect()
        assert scanner.get_status()["is_connected"] is False
        assert "GoSensor_Disconnect" in scanner._fake.call_names()

    def test_operations_require_connection(self):
        s = GocatorScanner({"ip": "192.168.1.10"})
        with pytest.raises(RuntimeError, match="not connected"):
            s.configure()


# ---------------------------------------------------------------------------
# The encoderless recipe
# ---------------------------------------------------------------------------


class TestConfigure:
    def test_applies_surface_mode_and_time_trigger(self, scanner):
        """Surface mode + TIME trigger — the encoderless acquisition path."""
        scanner.configure()
        assert scanner._fake.go.scan_mode == g.GO_MODE_SURFACE
        assert scanner._fake.go.trigger_source == g.GO_TRIGGER_TIME

    def test_applies_fixed_length_software_start_trigger(self, scanner):
        """FIXED_LENGTH generation with a SOFTWARE start trigger, not CONTINUOUS."""
        scanner.configure()
        calls = dict(
            (name, args)
            for name, args in scanner._fake.calls
            if name.startswith("GoSurfaceGeneration")
        )
        gen_type = _val(calls["GoSurfaceGeneration_SetGenerationType"][1])
        start_trigger = _val(calls["GoSurfaceGenerationFixedLength_SetStartTrigger"][1])
        assert gen_type == g.GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH
        assert start_trigger == g.GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE

    def test_travel_speed_goes_to_gotransform(self, scanner):
        """Y scaling comes from GoTransform travel speed, in mm/s."""
        scanner.configure(travel_speed_mm_s=12.5)
        assert scanner._fake.go.travel_speed == pytest.approx(12.5)

    def test_travel_speed_not_rewritten_when_unchanged(self, scanner):
        """GoTransform_SetSpeed writes flash — don't re-issue it needlessly."""
        scanner.configure(travel_speed_mm_s=20.0)
        first = scanner._fake.call_names().count("GoTransform_SetSpeed")
        scanner.configure(travel_speed_mm_s=20.0)
        second = scanner._fake.call_names().count("GoTransform_SetSpeed")
        assert first == 1
        assert second == 1, "unchanged speed must not trigger a second flash write"

    def test_frame_rate_disables_max_rate_first(self, scanner):
        """A specific frame rate requires leaving max-frame-rate mode."""
        scanner.configure(frame_rate_hz=300.0)
        names = scanner._fake.call_names()
        assert names.index("GoSetup_EnableMaxFrameRate") < names.index(
            "GoSetup_SetFrameRate"
        )
        assert scanner._fake.go.frame_rate == pytest.approx(300.0)

    def test_frame_rate_above_sensor_limit_rejected(self, scanner):
        """The real ceiling is FOV/exposure-dependent (~443 Hz here), not the
        datasheet's headline 10 kHz — reject before the scan, not during."""
        with pytest.raises(ValueError, match="outside the sensor's current supported"):
            scanner.configure(frame_rate_hz=5000.0)

    def test_frame_rate_rechecked_after_flush(self, scanner):
        """The ceiling is dynamic: observed on hardware dropping by half once
        max-frame-rate mode was disabled. A rate that passed the pre-write
        check must still be caught if the post-flush ceiling moved below it,
        because an unachievable rate silently corrupts Y spacing."""
        fake = scanner._fake.go
        original_flush = fake.GoSensor_Flush

        def flush_and_drop_ceiling(sensor):
            fake.frame_rate_limit_max = 221.563   # ceiling halves post-flush
            return original_flush(sensor)

        fake.GoSensor_Flush = flush_and_drop_ceiling
        with pytest.raises(ValueError, match="cannot deliver that rate|maximum frame rate"):
            scanner.configure(frame_rate_hz=400.0)

    def test_frame_rate_readback_mismatch_is_trusted_over_request(self, scanner):
        """If the sensor reports a different rate than requested, believe the
        sensor — Y-spacing bookkeeping depends on the real value."""
        fake = scanner._fake.go
        original_set = fake.GoSetup_SetFrameRate

        def set_but_clamp(setup, rate):
            original_set(setup, rate)
            fake.frame_rate = 180.0   # sensor clamps to something else
            return g.kOK

        fake.GoSetup_SetFrameRate = set_but_clamp
        applied = scanner.configure(frame_rate_hz=200.0)
        assert applied["frame_rate_hz"] == pytest.approx(180.0)

    def test_flush_pushes_config_last(self, scanner):
        """GoSensor_Flush must come after the setters that need pushing."""
        scanner.configure()
        names = scanner._fake.call_names()
        assert names.index("GoSetup_SetTriggerSource") < names.index("GoSensor_Flush")

    def test_fixed_length_outside_sensor_limits_rejected(self, scanner):
        """A length the sensor can't do fails loudly before the scan."""
        scanner._fake.go.length_limit_max = 100.0
        with pytest.raises(ValueError, match="outside the sensor's supported range"):
            scanner.configure(fixed_length_mm=500.0)

    def test_status_reports_the_whole_recipe_in_words(self, scanner):
        """Status must show start trigger and generation type, not just the
        trigger source — those two are what make the recipe encoderless, and
        reading them back is how you confirm configure() actually landed."""
        scanner.configure()
        status = scanner.get_status()
        assert status["sensor_scan_mode"] == "surface"
        assert status["sensor_trigger_source"] == "time"
        assert status["sensor_surface_generation"] == "fixed_length"
        assert status["sensor_start_trigger"] == "software"
        assert status["sensor_fixed_length_mm"] == pytest.approx(200.0)

    def test_status_labels_unknown_enum_values(self, scanner):
        """An unrecognized enum must be visible, not silently mislabeled."""
        scanner.configure()
        scanner._fake.go.start_trigger = 99
        assert scanner.get_status()["sensor_start_trigger"] == "unknown(99)"

    def test_config_defaults_used_when_args_omitted(self, scanner):
        applied = scanner.configure()
        assert applied["travel_speed_mm_s"] == pytest.approx(20.0)
        assert applied["fixed_length_mm"] == pytest.approx(200.0)
        assert applied["start_trigger"] == "software"


# ---------------------------------------------------------------------------
# Scan lifecycle
# ---------------------------------------------------------------------------


class TestScanLifecycle:
    def test_trigger_requires_start(self, scanner):
        with pytest.raises(RuntimeError, match="call start\\(\\) before trigger"):
            scanner.trigger()

    def test_start_enables_data_then_starts(self, scanner):
        scanner.start()
        names = scanner._fake.call_names()
        assert names.index("GoSystem_EnableData") < names.index("GoSystem_Start")
        assert scanner.get_status()["is_running"] is True

    def test_receive_surface_returns_scan_and_destroys_dataset(self, scanner):
        scanner._fake.go.datasets = [[make_stamp_msg(), make_surface_msg()]]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        assert isinstance(scan, SurfaceScan)
        assert scan.shape == (3, 4)
        assert "GoDestroy" in scanner._fake.call_names()

    def test_receive_surface_keeps_polling_past_non_surface_datasets(self, scanner):
        """Stamp-only datasets don't end the wait — keep polling for a surface."""
        scanner._fake.go.datasets = [
            [make_stamp_msg()],
            [make_stamp_msg()],
            [make_surface_msg()],
        ]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=2.0)
        assert scan.shape == (3, 4)
        assert scanner._fake.call_names().count("GoSystem_ReceiveData") == 3

    def test_receive_surface_times_out_with_actionable_message(self, scanner):
        """No surface at all → TimeoutError naming the likely causes."""
        scanner._fake.go.datasets = []
        scanner.start()
        with pytest.raises(TimeoutError, match="software trigger"):
            scanner.receive_surface(timeout_s=0.2)

    def test_stamp_metadata_attached_to_scan(self, scanner):
        scanner._fake.go.datasets = [[make_stamp_msg(), make_surface_msg()]]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        assert scan.metadata["frame_index"] == 7
        # timestamp is in internal units; true µs = value / 1.024
        assert scan.metadata["timestamp_us"] == pytest.approx(1000.0)

    def test_scan_configures_triggers_and_stops(self, scanner):
        """scan() runs the whole sequence and leaves acquisition stopped."""
        scanner._fake.go.datasets = [[make_surface_msg()]]
        scan = scanner.scan(timeout_s=1.0)
        names = scanner._fake.call_names()
        for expected in (
            "GoSetup_SetTriggerSource",
            "GoSystem_Start",
            "GoSensor_Trigger",
            "GoSystem_ReceiveData",
            "GoSystem_Stop",
        ):
            assert expected in names, expected
        assert names.index("GoSensor_Trigger") < names.index("GoSystem_ReceiveData")
        assert scanner.get_status()["is_running"] is False
        assert scan.metadata["travel_speed_mm_s"] == pytest.approx(20.0)

    def test_scan_stops_acquisition_even_on_timeout(self, scanner):
        """A failed scan must not leave the sensor running."""
        scanner._fake.go.datasets = []
        with pytest.raises(TimeoutError):
            scanner.scan(timeout_s=0.2)
        assert scanner.get_status()["is_running"] is False
        assert "GoSystem_Stop" in scanner._fake.call_names()

    def test_default_timeout_from_length_and_speed(self, scanner):
        """200 mm at 20 mm/s = 10 s of travel, +50% headroom."""
        assert scanner._default_timeout_s() == pytest.approx(15.0)

    def test_scan_count_tracked(self, scanner):
        scanner._fake.go.datasets = [[make_surface_msg()], [make_surface_msg()]]
        scanner.scan(timeout_s=1.0)
        scanner.scan(timeout_s=1.0)
        assert scanner.get_status()["scan_count"] == 2


# ---------------------------------------------------------------------------
# Gantry coordination
# ---------------------------------------------------------------------------


class FakeAxisHandle:
    """Stand-in for macron.commands.AxisHandle (gantry.axis("X")).

    Mirrors the real one's safe_mode gating on motion-starting calls, which
    is the whole reason the scanner drives axes through AxisHandle rather
    than the ungated raw `gantry.cmd` path.
    """

    def __init__(self, name, calls, safe_mode=False):
        self.name = name
        self._calls = calls
        self._safe_mode = safe_mode

    def get_position(self):
        return 0.0

    def set_speed(self, value):
        self._calls.append(("set_speed", self.name, value))
        return value

    def begin_move_to(self, position):
        if self._safe_mode:
            raise RuntimeError(
                f"begin_move_to blocked by safe_mode (axis={self.name!r})"
            )
        self._calls.append(("begin_move_to", self.name, position))


class FakeGantry:
    def __init__(self, safe_mode=False):
        self.calls: list = []
        self._handles = {
            name: FakeAxisHandle(name, self.calls, safe_mode) for name in ("X", "Y")
        }

    def axis(self, name):
        try:
            return self._handles[name]
        except KeyError:
            raise KeyError(
                f"Axis {name!r} is not configured (configured: {list(self._handles)})"
            ) from None

    # Present but never used by the scanner — its presence would let an
    # accidental regression to the ungated path go unnoticed, so it raises.
    @property
    def cmd(self):
        raise AssertionError(
            "scan_with_gantry must drive axes via gantry.axis(...) (AxisHandle), "
            "not the ungated gantry.cmd path — see AxisHandle's safe_mode gate"
        )


class TestScanWithGantry:
    def test_motion_starts_before_trigger(self, scanner):
        """The trigger must fire mid-move, after a non-blocking begin_move_to."""
        scanner._fake.go.datasets = [[make_surface_msg()]]
        gantry = FakeGantry()
        scan = scanner.scan_with_gantry(
            gantry, axis="X", end_mm=200.0, feed_rate_mm_s=20.0, settle_s=0.0
        )
        assert [c[0] for c in gantry.calls] == ["set_speed", "begin_move_to"]
        names = scanner._fake.call_names()
        assert names.index("GoSystem_Start") < names.index("GoSensor_Trigger")
        assert scan.metadata["gantry_axis"] == "X"
        assert scan.metadata["gantry_feed_rate_mm_s"] == pytest.approx(20.0)

    def test_feed_rate_becomes_sensor_travel_speed(self, scanner):
        """The whole scheme depends on these two matching."""
        scanner._fake.go.datasets = [[make_surface_msg()]]
        gantry = FakeGantry()
        scanner.scan_with_gantry(
            gantry, axis="X", end_mm=200.0, feed_rate_mm_s=7.5, settle_s=0.0
        )
        assert scanner._fake.go.travel_speed == pytest.approx(7.5)
        assert gantry.calls[0] == ("set_speed", "X", 7.5)

    def test_unknown_axis_rejected(self, scanner):
        gantry = FakeGantry()
        with pytest.raises(KeyError, match="not configured"):
            scanner.scan_with_gantry(
                gantry, axis="Q", end_mm=10.0, feed_rate_mm_s=5.0
            )

    def test_safe_mode_blocks_the_move(self, scanner):
        """The scanner must not be a way around the gantry's safe_mode gate."""
        scanner._fake.go.datasets = [[make_surface_msg()]]
        gantry = FakeGantry(safe_mode=True)
        with pytest.raises(RuntimeError, match="safe_mode"):
            scanner.scan_with_gantry(
                gantry, axis="X", end_mm=200.0, feed_rate_mm_s=20.0, settle_s=0.0
            )
        # And acquisition is left stopped, not running.
        assert scanner.get_status()["is_running"] is False

    def test_stops_acquisition_when_scan_fails(self, scanner):
        scanner._fake.go.datasets = []
        gantry = FakeGantry()
        with pytest.raises(TimeoutError):
            scanner.scan_with_gantry(
                gantry,
                axis="X",
                end_mm=200.0,
                feed_rate_mm_s=20.0,
                settle_s=0.0,
                timeout_s=0.2,
            )
        assert scanner.get_status()["is_running"] is False


# ---------------------------------------------------------------------------
# Raw-count scaling
# ---------------------------------------------------------------------------


class TestSurfaceScaling:
    def test_z_scaling_offset_plus_resolution(self, scanner):
        """z_mm = z_off_um/1000 + z_res_nm/1e6 * raw_count."""
        scanner._fake.go.datasets = [
            [make_surface_msg(rows=2, cols=2, z_res=1_000_000, z_off=5_000)]
        ]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        # z_res 1e6 nm = 1 mm/count; z_off 5000 µm = 5 mm.
        # Raw counts are 0,1,2,3 → 5,6,7,8 mm.
        np.testing.assert_allclose(scan.z_mm, [[5.0, 6.0], [7.0, 8.0]])

    def test_x_and_y_axes_from_resolution(self, scanner):
        """Uniform surface X/Y are implied by index * resolution."""
        scanner._fake.go.datasets = [
            [make_surface_msg(rows=3, cols=4, x_res=125_000, y_res=50_000)]
        ]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        # 125_000 nm = 0.125 mm per column; 50_000 nm = 0.05 mm per row.
        np.testing.assert_allclose(scan.x_mm, [0.0, 0.125, 0.25, 0.375])
        np.testing.assert_allclose(scan.y_mm, [0.0, 0.05, 0.10])

    def test_invalid_points_become_nan(self, scanner):
        """0x8000 raw means no laser return, and must not scale to a height."""
        msg = make_surface_msg(rows=2, cols=2, z_res=1_000_000, z_off=0)
        msg["data"] = np.array(
            [[0, g.INVALID_RANGE_16BIT], [2, 3]], dtype=np.int16
        )
        scanner._fake.go.datasets = [[msg]]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        assert np.isnan(scan.z_mm[0, 1])
        assert scan.valid_count == 3

    def test_y_spacing_recorded_in_metadata(self, scanner):
        scanner._fake.go.datasets = [[make_surface_msg(y_res=50_000)]]
        scanner.start()
        scan = scanner.receive_surface(timeout_s=1.0)
        assert scan.metadata["y_spacing_mm"] == pytest.approx(0.05)


# ---------------------------------------------------------------------------
# SurfaceScan — pure, no SDK involved
# ---------------------------------------------------------------------------


def make_scan(**meta) -> SurfaceScan:
    z = np.array([[1.0, 2.0], [np.nan, 4.0]])
    return SurfaceScan(
        z_mm=z,
        x_mm=np.array([0.0, 1.0]),
        y_mm=np.array([0.0, 2.0]),
        metadata=meta,
        is_uniform=True,
    )


class TestSurfaceScan:
    def test_to_points_drops_invalid_by_default(self):
        points = make_scan().to_points()
        assert points.shape == (3, 3)
        assert not np.isnan(points).any()

    def test_to_points_can_keep_invalid(self):
        points = make_scan().to_points(drop_invalid=False)
        assert points.shape == (4, 3)

    def test_to_points_pairs_xy_correctly(self):
        """Row index drives Y, column index drives X."""
        points = make_scan().to_points()
        # z=2.0 sits at row 0, col 1 → (x=1.0, y=0.0)
        row = points[np.isclose(points[:, 2], 2.0)][0]
        assert row[0] == pytest.approx(1.0)
        assert row[1] == pytest.approx(0.0)
        # z=4.0 sits at row 1, col 1 → (x=1.0, y=2.0)
        row = points[np.isclose(points[:, 2], 4.0)][0]
        assert row[0] == pytest.approx(1.0)
        assert row[1] == pytest.approx(2.0)

    def test_save_csv_writes_header_and_valid_rows(self, tmp_path):
        path = make_scan().save_csv(tmp_path / "scan.csv")
        lines = path.read_text().strip().splitlines()
        assert lines[0] == "x_mm,y_mm,z_mm"
        assert len(lines) == 4  # header + 3 valid points

    def test_save_ply_ascii_roundtrip(self, tmp_path):
        path = make_scan().save_ply(tmp_path / "scan.ply", binary=False)
        text = path.read_text()
        assert text.startswith("ply\n")
        assert "element vertex 3" in text

    def test_save_ply_binary_size_matches_vertex_count(self, tmp_path):
        path = make_scan().save_ply(tmp_path / "scan.ply", binary=True)
        raw = path.read_bytes()
        header_end = raw.index(b"end_header\n") + len(b"end_header\n")
        assert len(raw) - header_end == 3 * 3 * 4  # 3 points * xyz * float32

    def test_save_npz_preserves_nans_and_grid(self, tmp_path):
        path = make_scan().save_npz(tmp_path / "scan.npz")
        loaded = np.load(path, allow_pickle=True)
        assert loaded["z_mm"].shape == (2, 2)
        assert np.isnan(loaded["z_mm"][1, 0])

    def test_rescale_y_scales_travel_axis(self):
        """Correcting the assumed velocity rescales Y without a re-scan."""
        scan = make_scan(travel_speed_mm_s=10.0)
        rescaled = scan.rescale_y(20.0)
        np.testing.assert_allclose(rescaled.y_mm, [0.0, 4.0])
        np.testing.assert_allclose(rescaled.x_mm, scan.x_mm)
        assert rescaled.metadata["y_rescale_factor"] == pytest.approx(2.0)
        assert rescaled.metadata["travel_speed_mm_s"] == pytest.approx(20.0)

    def test_rescale_y_leaves_original_untouched(self):
        scan = make_scan(travel_speed_mm_s=10.0)
        scan.rescale_y(20.0)
        np.testing.assert_allclose(scan.y_mm, [0.0, 2.0])

    def test_rescale_y_needs_configured_speed(self):
        with pytest.raises(ValueError, match="travel_speed_mm_s"):
            make_scan().rescale_y(20.0)


# ---------------------------------------------------------------------------
# SDK library discovery
# ---------------------------------------------------------------------------


class TestFindLibDir:
    @pytest.fixture(autouse=True)
    def _clear_env(self, monkeypatch):
        """Keep these tests hermetic — a real SDK install must not leak in."""
        monkeypatch.delenv("LAGUNA_GOSDK_LIB_DIR", raising=False)
        monkeypatch.delenv("LAGUNA_GOSDK_DIR", raising=False)

    def _install(self, path):
        """Create `path` (if needed) holding both required libraries."""
        path.mkdir(parents=True, exist_ok=True)
        (path / "libGoSdk.so").touch()
        (path / "libkApi.so").touch()
        return path

    def test_explicit_dir_wins(self, tmp_path):
        assert g.find_lib_dir(str(self._install(tmp_path))) == tmp_path

    def test_env_var_used(self, tmp_path, monkeypatch):
        monkeypatch.setenv("LAGUNA_GOSDK_LIB_DIR", str(self._install(tmp_path)))
        assert g.find_lib_dir() == tmp_path

    def test_sdk_root_env_var_appends_lib_subdir(self, tmp_path, monkeypatch):
        lib_dir = self._install(tmp_path / g._DEFAULT_LIB_SUBDIR)
        monkeypatch.setenv("LAGUNA_GOSDK_DIR", str(tmp_path))
        assert g.find_lib_dir() == lib_dir

    def test_partial_install_not_accepted(self, tmp_path, monkeypatch):
        """libGoSdk.so alone isn't enough — kApi is required too."""
        (tmp_path / "libGoSdk.so").touch()
        monkeypatch.setenv("LAGUNA_GOSDK_LIB_DIR", str(tmp_path))
        # A real install elsewhere must not rescue an explicitly-set bad path.
        monkeypatch.setattr(g, "_DEFAULT_SDK_DIRS", (str(self._install(tmp_path / "real")),))
        with pytest.raises(FileNotFoundError, match="set explicitly"):
            g.find_lib_dir()

    def test_explicit_bad_path_does_not_fall_back_to_defaults(self, tmp_path, monkeypatch):
        """Silently loading a different SDK build than the one asked for is a trap."""
        root = tmp_path / "root"
        self._install(root / g._DEFAULT_LIB_SUBDIR)
        monkeypatch.setattr(g, "_DEFAULT_SDK_DIRS", (str(root),))
        with pytest.raises(FileNotFoundError, match="no default locations were searched"):
            g.find_lib_dir(str(tmp_path / "does-not-exist"))

    def test_defaults_searched_when_nothing_explicit(self, tmp_path, monkeypatch):
        """_DEFAULT_SDK_DIRS entries are SDK roots — the lib subdir is appended."""
        root = tmp_path / "root"
        lib_dir = self._install(root / g._DEFAULT_LIB_SUBDIR)
        monkeypatch.setattr(g, "_DEFAULT_SDK_DIRS", (str(tmp_path / "nope"), str(root)))
        assert g.find_lib_dir() == lib_dir

    def test_error_names_the_build_script(self, monkeypatch, tmp_path):
        monkeypatch.setattr(g, "_DEFAULT_SDK_DIRS", (str(tmp_path / "nope"),))
        with pytest.raises(FileNotFoundError, match="scripts/build_gosdk.sh"):
            g.find_lib_dir()


class TestErrorTypes:
    def test_timeout_status_raises_timeout_subclass(self):
        lib = FakeLib()
        with pytest.raises(g.GoSdkTimeout):
            lib.check("GoSystem_ReceiveData", g.kERROR_TIMEOUT)

    def test_kerror_is_zero_not_falsy_success(self):
        """kOK is 1 and kERROR is 0 — a truthiness check would invert this."""
        lib = FakeLib()
        with pytest.raises(g.GoSdkError):
            lib.check("GoSensor_Connect", g.kERROR)
        assert lib.check("GoSensor_Connect", g.kOK) == g.kOK
