"""Tests for the RangefinderSubsystem and OD2000 PDIN decoder.

Hardware assumptions encoded here (each test documents one assumption;
a failure on real hardware means the assumption is wrong):

  - AL1342 PDIN hex is big-endian int32 distance in nm (not µm, not mm, not
    little-endian). If decode_od2000_pdin returns ~0.2 mm when you expect
    ~200 mm, the unit assumption is wrong. If the distance sign or sign is
    scrambled, the byte order is wrong.
  - The pdin path inside the AL1342 event JSON is exactly
    '/iolinkmaster/port[N]/iolinkdevice/pdin' with square-bracket notation.
  - The OD2000 range is approximately 20–1200 mm; readings outside that
    range indicate misconfig, wrong port, or a disconnected sensor.
  - Q1/Q2 switching outputs live in byte 5 of the PDIN, bits 0 and 1.
  - Scale byte (byte 4) is normally 0; non-zero values are logged but do not
    invalidate the distance reading.
"""

import pytest

from laguna.rangefinder import subsystem as rangefinder_module
from laguna.rangefinder import (
    OD2000Rangefinder,
    RangefinderSubsystem,
    WTT12LRangefinder,
    decode_dp4200_wtt12l_analog_pdin,
    decode_od2000_pdin,
    decode_wtt12l_pdin,
)


# ---------------------------------------------------------------------------
# decode_od2000_pdin — pure function, no hardware needed
# ---------------------------------------------------------------------------


def _encode_distance_nm(distance_nm: int) -> str:
    """Helper: encode a distance_nm value back to 12-char OD2000 hex string."""
    raw = distance_nm.to_bytes(4, byteorder="big", signed=True) + b"\x00\x00"
    return raw.hex().upper()


class TestDecodeOd2000Pdin:
    def test_zero_distance(self):
        """All-zero PDIN → distance_nm = 0, distance_mm = 0.0."""
        result = decode_od2000_pdin("000000000000")
        assert result["distance_nm"] == 0
        assert result["distance_mm"] == 0.0
        assert result["scale"] == 0
        assert result["q1"] is False
        assert result["q2"] is False

    def test_known_200mm(self):
        """200 mm = 200_000_000 nm = 0x0BEBC200.
        If this fails on hardware (distance_mm ~ 0.0002 not 200), the AL1342
        is publishing in µm, not nm. If the value is ~3355443200 nm, the bytes
        are little-endian instead of big-endian.
        """
        # 200_000_000 in big-endian hex = 0BEBC200
        result = decode_od2000_pdin("0BEBC2000000")
        assert result["distance_nm"] == 200_000_000
        assert abs(result["distance_mm"] - 200.0) < 0.001

    def test_roundtrip_500mm(self):
        """Roundtrip encode/decode for 500 mm."""
        hex_str = _encode_distance_nm(500_000_000)
        result = decode_od2000_pdin(hex_str)
        assert abs(result["distance_mm"] - 500.0) < 0.001

    def test_big_endian_not_little(self):
        """Confirm the decoder is big-endian.
        If hardware reads come back ~0.2 mm when the target is ~200 mm,
        try: distance_nm = int.from_bytes(raw[0:4], 'little', signed=True)
        and update the decoder.
        """
        # 0x00C8 0000 = 13_107_200 nm ≈ 13.1 mm (little-endian interpretation of 0x0000C800)
        # Big-endian of same 4 bytes: 0x0000C800 = 51_200 nm ≈ 0.051 mm
        # The test here checks a clearly asymmetric value to expose the difference
        result_big = decode_od2000_pdin("0BEBC2000000")  # big-endian 200mm
        # If we accidentally decoded little-endian the value would be completely different
        assert result_big["distance_mm"] > 100.0, (
            "Expected ~200 mm from big-endian decode; got {:.3f} mm. "
            "If hardware gives wrong values, check byte order — AL1342 may publish little-endian.".format(
                result_big["distance_mm"]
            )
        )

    def test_q1_only(self):
        """Byte 5 = 0x01 → Q1 True, Q2 False."""
        hex_str = _encode_distance_nm(200_000_000)[:-2] + "01"
        result = decode_od2000_pdin(hex_str)
        assert result["q1"] is True
        assert result["q2"] is False

    def test_q2_only(self):
        """Byte 5 = 0x02 → Q1 False, Q2 True."""
        hex_str = _encode_distance_nm(200_000_000)[:-2] + "02"
        result = decode_od2000_pdin(hex_str)
        assert result["q1"] is False
        assert result["q2"] is True

    def test_both_q_bits(self):
        """Byte 5 = 0x03 → both Q1 and Q2 True."""
        hex_str = _encode_distance_nm(300_000_000)[:-2] + "03"
        result = decode_od2000_pdin(hex_str)
        assert result["q1"] is True
        assert result["q2"] is True

    def test_nonzero_scale_byte(self):
        """Scale byte (byte 4) nonzero should not crash the decoder.
        The OD2000 normally sends scale=0. A non-zero value may appear in
        certain operating modes; the decoder should pass it through.
        """
        # scale = 0x05 in byte 4
        raw_nm = (200_000_000).to_bytes(4, "big", signed=True)
        hex_str = (raw_nm + b"\x05\x00").hex()
        result = decode_od2000_pdin(hex_str)
        assert result["scale"] == 5
        assert result["distance_nm"] == 200_000_000

    def test_negative_distance(self):
        """Negative distance_nm is representable (sensor out of range below zero).
        decode_od2000_pdin should return it without raising; caller decides
        whether to discard it.
        """
        # -1 in big-endian signed 4 bytes = FFFFFFFF
        result = decode_od2000_pdin("FFFFFFFF0000")
        assert result["distance_nm"] == -1
        assert result["distance_mm"] < 0.0

    def test_wrong_hex_length_raises(self):
        """If AL1342 sends more or fewer bytes than expected, fromhex raises.
        On hardware: if you see IndexError/ValueError here, the OD2000 process
        data layout is different from the 7002T15 IODD spec (6 bytes).
        """
        with pytest.raises((ValueError, IndexError)):
            decode_od2000_pdin("AABB")  # only 2 bytes

    def test_plausible_range(self):
        """Values in OD2000 measurement range (20–1200 mm) should decode cleanly.
        If hardware reads are consistently outside this range, check sensor
        mounting distance and set the operating range in the sensor config.
        """
        for mm in [20, 100, 500, 1000, 1200]:
            nm = mm * 1_000_000
            hex_str = _encode_distance_nm(nm)
            result = decode_od2000_pdin(hex_str)
            assert abs(result["distance_mm"] - mm) < 0.01, (
                f"Roundtrip failed for {mm} mm: got {result['distance_mm']:.3f} mm"
            )


# ---------------------------------------------------------------------------
# RangefinderSubsystem: HTTP polling (no MQTT) — GH #22
# ---------------------------------------------------------------------------


@pytest.fixture
def pdin(monkeypatch):
    """Script what the AL1342's process-data endpoint returns.

    Set ``pdin.value`` to a hex string, or ``pdin.error`` to an exception to
    raise instead. ``pdin.reads`` records (host, port, timeout) per request.
    """

    class _Pdin:
        value = _encode_distance_nm(200_000_000)
        error = None
        reads: list = []

    state = _Pdin()
    state.reads = []

    def fake_read(host, port, timeout=5.0):
        state.reads.append((host, port, timeout))
        if state.error is not None:
            raise state.error
        return state.value

    monkeypatch.setattr(rangefinder_module, "read_pdin_hex", fake_read)
    return state


def _make_rangefinder(**extra):
    config = {"pdin_port": 1, "offset_mm": 0.0, "al1342_host": "192.168.1.251", **extra}
    return RangefinderSubsystem(config)


class TestRangefinderSubsystem:
    def test_construction_needs_no_mqtt(self):
        """The whole point of #22: no broker, no subscriber, no topic."""
        rf = _make_rangefinder()
        assert not hasattr(rf, "_mqtt")

    def test_a_topic_key_is_called_out_as_ignored(self, caplog):
        with caplog.at_level("WARNING"):
            _make_rangefinder(topic="laguna/od2000")
        assert "'topic' is ignored" in caplog.text

    def test_connect_probes_the_configured_port(self, pdin):
        rf = _make_rangefinder(pdin_port=3)
        assert rf.connect() is True
        assert pdin.reads == [("192.168.1.251", 3, 5.0)]
        assert rf._is_connected is True

    def test_connect_fails_when_the_al1342_is_unreachable(self, pdin, caplog):
        """The reported failure was a rangefinder that said connected and then
        never produced a sample. A wrong host/port must fail connect() instead."""
        pdin.error = ConnectionError("no route to host")
        rf = _make_rangefinder()
        with caplog.at_level("ERROR"):
            assert rf.connect() is False
        assert rf._is_connected is False
        assert "probe read" in caplog.text and "no route to host" in caplog.text

    def test_connect_fails_when_the_port_has_no_device(self, pdin):
        pdin.error = RuntimeError("AL1342 returned code 503")
        assert _make_rangefinder().connect() is False

    def test_connect_fails_without_a_host(self):
        rf = RangefinderSubsystem({"pdin_port": 1})
        assert rf.connect() is False

    def test_disconnect_holds_nothing_open(self, pdin):
        rf = _make_rangefinder()
        rf.connect()
        rf.disconnect()
        assert rf._is_connected is False

    def test_get_status_before_any_reading(self):
        status = _make_rangefinder().get_status()
        assert status["latest_distance_mm"] is None
        assert status["sample_count"] == 0

    def test_status_advertises_no_stream_that_never_arrives(self):
        status = _make_rangefinder().get_status()
        assert "topic" not in status and "achieved_rate_hz" not in status

    def test_read_mm_returns_the_polled_distance(self, pdin):
        pdin.value = _encode_distance_nm(400_000_000)
        assert abs(_make_rangefinder().read_mm() - 400.0) < 0.001

    def test_offset_mm_applied(self, pdin):
        rf = _make_rangefinder(offset_mm=50.0)
        assert abs(rf.read_mm() - 250.0) < 0.001

    def test_a_failed_read_raises_instead_of_returning_none(self, pdin):
        """#22 reported a silent None forever; a failed poll must say why."""
        pdin.error = TimeoutError("timed out")
        with pytest.raises(TimeoutError):
            _make_rangefinder().read_mm()

    def test_a_failed_read_does_not_overwrite_the_last_good_sample(self, pdin):
        rf = _make_rangefinder()
        rf.read_mm()
        pdin.error = TimeoutError("timed out")
        with pytest.raises(TimeoutError):
            rf.read_mm()
        assert rf.get_status()["sample_count"] == 1
        assert abs(rf.get_status()["latest_distance_mm"] - 200.0) < 0.001

    def test_status_reflects_each_reading(self, pdin):
        rf = _make_rangefinder()
        rf.read_mm()
        pdin.value = _encode_distance_nm(500_000_000)
        rf.read_mm()
        status = rf.get_status()
        assert abs(status["latest_distance_mm"] - 500.0) < 0.001
        assert status["sample_count"] == 2
        assert status["latest_wall_time"] > 0


# ---------------------------------------------------------------------------
# decode_wtt12l_pdin — pure function, no hardware needed
#
# Layout taken from SICK's official "Technical Information: Photoelectric
# sensors, SICK Smart Sensors / IO-Link" (www.sick.com/8022709), table 6 —
# NOT yet confirmed against physical WTT12L-A2523 hardware. If real readings
# come back wrong, see docs/WTT12L_POWERPROX_SETUP.md Step 4 for the likely
# culprits (byte order, Process data select mode, units).
# ---------------------------------------------------------------------------


def _encode_wtt12l_distance_mm(distance_mm: int, status_byte: int = 0x00) -> str:
    """Helper: encode a distance_mm value to an 8-char WTT12L PDIN hex string."""
    raw = distance_mm.to_bytes(2, byteorder="big", signed=False) + b"\x00" + bytes([status_byte])
    return raw.hex().upper()


# ---------------------------------------------------------------------------
# On-demand HTTP path: activate()/deactivate()/read_mm() and the
# OD2000Rangefinder/WTT12LRangefinder subclasses. read_pdin_hex/write_acyclic
# are monkeypatched — no real network access.
# ---------------------------------------------------------------------------


class TestOd2000RangefinderOnDemand:
    def _make(self, **extra_config):
        config = {"pdin_port": 2, "al1342_host": "192.168.1.251", **extra_config}
        return OD2000Rangefinder(config), None

    def test_subsystem_name(self):
        assert OD2000Rangefinder.subsystem_name == "od2000"

    def test_read_mm_requires_al1342_host(self):
        rf = OD2000Rangefinder({"pdin_port": 2})
        with pytest.raises(RuntimeError):
            rf.read_mm()

    def test_read_mm_decodes_and_applies_offset(self, monkeypatch):
        rf, _mqtt = self._make(offset_mm=10.0)
        monkeypatch.setattr(rangefinder_module, "read_pdin_hex", lambda host, port, timeout=5.0: _encode_distance_nm(200_000_000))
        assert abs(rf.read_mm() - 210.0) < 0.001

    def test_activate_writes_laser_on(self, monkeypatch):
        rf, _mqtt = self._make()
        calls = []
        monkeypatch.setattr(
            rangefinder_module, "write_acyclic",
            lambda host, port, index, subindex, value, timeout=5.0: calls.append((host, port, index, subindex, value)),
        )
        rf.activate()
        assert calls == [("192.168.1.251", 2, 97, 0, "00")]

    def test_deactivate_writes_laser_off(self, monkeypatch):
        rf, _mqtt = self._make()
        calls = []
        monkeypatch.setattr(
            rangefinder_module, "write_acyclic",
            lambda host, port, index, subindex, value, timeout=5.0: calls.append(value),
        )
        rf.deactivate()
        assert calls == ["01"]

    def test_activate_requires_al1342_host(self):
        rf = OD2000Rangefinder({"pdin_port": 2})
        with pytest.raises(RuntimeError):
            rf.activate()


class TestWtt12lRangefinderOnDemand:
    def test_subsystem_name(self):
        rf = WTT12LRangefinder({"pdin_port": 7, "al1342_host": "192.168.1.251"})
        assert rf.subsystem_name == "wtt12l"

    def test_activate_deactivate_are_noops(self, monkeypatch):
        """The DP4200 analog bridge has no laser control path — activate()/
        deactivate() must not attempt any AL1342 write."""
        rf = WTT12LRangefinder({"pdin_port": 7, "al1342_host": "192.168.1.251"})

        def fail(*a, **kw):
            raise AssertionError("write_acyclic should not be called for WTT12LRangefinder")

        monkeypatch.setattr(rangefinder_module, "write_acyclic", fail)
        rf.activate()
        rf.deactivate()

    def test_read_mm_uses_dp4200_decoder(self, monkeypatch):
        rf = WTT12LRangefinder({"pdin_port": 7, "al1342_host": "192.168.1.251"})
        # "2890FD01" -> channel1_raw 0x2890 = 10384 uA -> 10.384 mA -> ~600mm (see decoder tests)
        monkeypatch.setattr(rangefinder_module, "read_pdin_hex", lambda host, port, timeout=5.0: "2890FD01")
        assert abs(rf.read_mm() - 600) < 50

    def test_calibration_applies_against_current_ma_not_distance_mm(self, monkeypatch, tmp_path):
        from laguna.rangefinder.calibration import CalibrationPoint, LinearCalibration

        cal = LinearCalibration.fit(
            "wtt12l_powerprox",
            [CalibrationPoint(known_height_mm=0.0, raw_value=0.0), CalibrationPoint(known_height_mm=100.0, raw_value=10.0)],
        )
        cal_path = tmp_path / "wtt12l_cal.csv"
        cal.to_csv(cal_path)

        rf = WTT12LRangefinder(
            {"pdin_port": 7, "al1342_host": "192.168.1.251", "calibration_file": str(cal_path)}
        )
        # current_ma = 10.384 for this hex (see decoder tests) -> real_height_mm ~= 103.84
        monkeypatch.setattr(rangefinder_module, "read_pdin_hex", lambda host, port, timeout=5.0: "2890FD01")
        assert abs(rf.read_mm() - 103.84) < 0.01


class TestDecodeWtt12lPdin:
    def test_zero_distance(self):
        result = decode_wtt12l_pdin("00000000")
        assert result["distance_mm"] == 0
        assert result["ql1"] is False
        assert result["ql2"] is False

    def test_known_500mm(self):
        """500 mm = 0x01F4, directly as mm (no nm/µm scaling, unlike the OD2000)."""
        result = decode_wtt12l_pdin("01F40000")
        assert result["distance_mm"] == 500

    def test_roundtrip_900mm(self):
        hex_str = _encode_wtt12l_distance_mm(900)
        result = decode_wtt12l_pdin(hex_str)
        assert result["distance_mm"] == 900

    def test_ql1_only(self):
        hex_str = _encode_wtt12l_distance_mm(300, status_byte=0x01)
        result = decode_wtt12l_pdin(hex_str)
        assert result["ql1"] is True
        assert result["ql2"] is False

    def test_ql2_only(self):
        hex_str = _encode_wtt12l_distance_mm(300, status_byte=0x02)
        result = decode_wtt12l_pdin(hex_str)
        assert result["ql1"] is False
        assert result["ql2"] is True

    def test_both_ql_bits(self):
        hex_str = _encode_wtt12l_distance_mm(300, status_byte=0x03)
        result = decode_wtt12l_pdin(hex_str)
        assert result["ql1"] is True
        assert result["ql2"] is True


# ---------------------------------------------------------------------------
# decode_dp4200_wtt12l_analog_pdin — pure function, no hardware needed
#
# Confirmed on hardware 2026-07-28 against the WTT12L-A2523's analog output
# (WTT12L native IO-Link process data never validated; see
# decode_wtt12l_pdin's docstring). Two real readings anchor these tests:
#   600 mm  -> pdin "2890FD01" -> channel1_raw 0x2890 = 10384 -> 10.384 mA
#   1115 mm -> pdin "3F02FD01" -> channel1_raw 0x3F02 = 16130 -> 16.130 mA
# Channel 2 (bytes 2-3) was constant "FD01" at both distances (unconnected
# input) and is intentionally not decoded.
# ---------------------------------------------------------------------------


class TestDecodeDp4200Wtt12lAnalogPdin:
    def test_known_600mm_reading(self):
        """Real hardware reading at 600 mm. Decoded value has ~20-30 mm of
        slop (see function docstring) — this checks it's in the right
        ballpark, not exact agreement."""
        result = decode_dp4200_wtt12l_analog_pdin("2890FD01")
        assert abs(result["current_ma"] - 10.384) < 0.001
        assert abs(result["distance_mm"] - 600) < 50

    def test_known_1115mm_reading(self):
        """Real hardware reading at 1115 mm."""
        result = decode_dp4200_wtt12l_analog_pdin("3F02FD01")
        assert abs(result["current_ma"] - 16.130) < 0.001
        assert abs(result["distance_mm"] - 1115) < 50

    def test_4ma_maps_to_near_mm(self):
        """4 mA (0x0FA0 = 4000 uA) should decode to exactly near_mm."""
        result = decode_dp4200_wtt12l_analog_pdin("0FA0FD01")
        assert abs(result["distance_mm"] - 100.0) < 0.001

    def test_20ma_maps_to_far_mm(self):
        """20 mA (0x4E20 = 20000 uA) should decode to exactly far_mm."""
        result = decode_dp4200_wtt12l_analog_pdin("4E20FD01")
        assert abs(result["distance_mm"] - 1400.0) < 0.001

    def test_custom_span(self):
        """near_mm/far_mm are overridable if the sensor gets taught a
        different span later."""
        result = decode_dp4200_wtt12l_analog_pdin(
            "0FA0FD01", near_mm=50.0, far_mm=2000.0
        )
        assert abs(result["distance_mm"] - 50.0) < 0.001

    def test_channel2_not_in_result(self):
        """Channel 2 (the unconnected input) should not leak into the
        decoded dict — only current_ma and distance_mm."""
        result = decode_dp4200_wtt12l_analog_pdin("2890FD01")
        assert set(result.keys()) == {"current_ma", "distance_mm"}


# ---------------------------------------------------------------------------
# simulated: True — connect()/reads succeed with no broker or AL1342 needed
# ---------------------------------------------------------------------------


class TestRangefinderSimulated:
    """simulated: True skips the AL1342 HTTP path entirely — connect()
    succeeds unconditionally, every reading is NaN rather than fabricated.
    See laguna.simulation."""

    def _rf(self, cls=OD2000Rangefinder, **extra_config):
        config = {"pdin_port": 2, "simulated": True, **extra_config}
        return cls(config)

    def test_connect_succeeds_with_no_host(self):
        rf = self._rf()
        assert rf.connect() is True
        assert rf._is_connected is True

    def test_get_status_reports_connected_with_nan_reading(self):
        import math

        rf = self._rf()
        rf.connect()
        status = rf.get_status()
        assert status["is_connected"] is True
        assert math.isnan(status["latest_distance_mm"])

    def test_read_mm_is_nan_with_no_al1342_host_needed(self):
        """Real read_mm() requires al1342_host — simulated mode needs
        neither that config key nor a real HTTP call."""
        import math

        rf = self._rf()  # no al1342_host in config
        assert math.isnan(rf.read_mm())

    def test_od2000_activate_deactivate_do_not_touch_the_network(self):
        """Real activate()/deactivate() call write_acyclic() (a real
        IO-Link HTTP write) — simulated mode must not reach it, and must
        not require al1342_host either."""
        rf = self._rf(cls=OD2000Rangefinder)
        rf.activate()    # must not raise despite no al1342_host configured
        rf.deactivate()  # ditto

    def test_wtt12l_simulated_too(self):
        import math

        rf = self._rf(cls=WTT12LRangefinder)
        assert rf.connect() is True
        assert math.isnan(rf.read_mm())
