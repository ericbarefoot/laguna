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

from laguna.rangefinder import RangefinderSubsystem, decode_od2000_pdin


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
# AL1342 JSON envelope extraction
# ---------------------------------------------------------------------------


def _make_al1342_event(pdin_port: int, pdin_hex: str, code: int = 200) -> dict:
    """Build a realistic AL1342 MQTT event payload for a given port and hex."""
    return {
        "code": "event",
        "cid": 10,
        "adr": "",
        "data": {
            "eventno": "6317",
            "srcurl": "/timer[1]/counter/datachanged",
            "payload": {
                f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/pdin": {
                    "code": code,
                    "data": pdin_hex,
                }
            },
        },
    }


class TestExtractPdinHex:
    """Tests for the JSON path used to extract PDIN hex from AL1342 events.

    The exact path /iolinkmaster/port[N]/iolinkdevice/pdin is from the AL1342
    manual §9.2.22. If the hardware produces a different path structure, these
    tests will fail with a KeyError that identifies exactly which key is wrong.
    """

    def _extract(self, msg: dict, port: int) -> str:
        key = f"/iolinkmaster/port[{port}]/iolinkdevice/pdin"
        return msg["data"]["payload"][key]["data"]

    def test_port_1_standard_path(self):
        """Port 1, standard AL1342 event envelope."""
        msg = _make_al1342_event(1, "0BEBC2000000")
        hex_str = self._extract(msg, 1)
        assert hex_str == "0BEBC2000000"

    def test_port_4_path(self):
        """Port 4 — confirm the port number appears in the key with brackets."""
        msg = _make_al1342_event(4, "0BEBC2000000")
        hex_str = self._extract(msg, 4)
        assert hex_str == "0BEBC2000000"

    def test_wrong_port_raises_keyerror(self):
        """Requesting port 2 when OD2000 is on port 1 → KeyError.
        On hardware: if you see this error, check 'pdin_port' in config.
        """
        msg = _make_al1342_event(1, "0BEBC2000000")
        with pytest.raises(KeyError):
            self._extract(msg, 2)

    def test_pdin_code_200_means_ok(self):
        """code=200 in the pdin entry means the IO-Link read succeeded.
        A non-200 code (e.g. 503) means the port has no device or the device
        is in SIO/DI mode — check IO-Link COM mode config on the OD2000.
        """
        msg = _make_al1342_event(1, "0BEBC2000000", code=200)
        assert msg["data"]["payload"]["/iolinkmaster/port[1]/iolinkdevice/pdin"]["code"] == 200

    def test_pdin_error_code_503(self):
        """code=503 means the IO-Link port has no device or is not in COM mode.
        The 'data' field may be empty or missing in this case.
        Tests that the code field is accessible without crashing on code != 200.
        """
        msg = _make_al1342_event(1, "", code=503)
        entry = msg["data"]["payload"]["/iolinkmaster/port[1]/iolinkdevice/pdin"]
        assert entry["code"] == 503

    def test_full_decode_pipeline(self):
        """End-to-end: AL1342 event → extract hex → decode PDIN."""
        msg = _make_al1342_event(1, _encode_distance_nm(350_000_000))
        hex_str = self._extract(msg, 1)
        result = decode_od2000_pdin(hex_str)
        assert abs(result["distance_mm"] - 350.0) < 0.001


# ---------------------------------------------------------------------------
# RangefinderSubsystem with a fake MqttSubscriber
# ---------------------------------------------------------------------------


class FakeMqttSubscriber:
    """Minimal double for MqttSubscriber, injectable into RangefinderSubsystem."""

    subsystem_name = "mqtt"

    def __init__(self):
        self._is_connected = False
        self._queues: dict = {}
        self._topics: list = []
        self.connected_called = 0
        self.disconnect_called = 0

    def connect(self) -> bool:
        self._is_connected = True
        self.connected_called += 1
        return True

    def disconnect(self) -> None:
        self._is_connected = False
        self.disconnect_called += 1

    def subscribe(self, topic: str) -> None:
        if topic not in self._queues:
            self._queues[topic] = []
            self._topics.append(topic)

    def push(self, topic: str, payload: dict) -> None:
        """Test helper: inject a message as if MQTT delivered it."""
        self._queues.setdefault(topic, []).append(payload)

    def drain(self, topic: str) -> list:
        items = list(self._queues.get(topic, []))
        self._queues[topic] = []
        return items

    def get_latest(self, topic: str) -> dict:
        items = self._queues.get(topic, [])
        last = items[-1] if items else None
        self._queues[topic] = []
        return last

    def get_status(self) -> dict:
        return {"is_connected": self._is_connected}


def _make_rangefinder(pdin_port: int = 1):
    mqtt = FakeMqttSubscriber()
    config = {"topic": "laguna/od2000", "pdin_port": pdin_port, "offset_mm": 0.0}
    rf = RangefinderSubsystem(config, mqtt)
    return rf, mqtt


class TestRangefinderSubsystem:
    def test_connect_delegates_to_mqtt(self):
        rf, mqtt = _make_rangefinder()
        result = rf.connect()
        assert result is True
        assert mqtt.connected_called == 1

    def test_connect_does_not_double_connect(self):
        rf, mqtt = _make_rangefinder()
        mqtt._is_connected = True  # already connected
        rf.connect()
        assert mqtt.connected_called == 0  # skipped

    def test_disconnect_delegates(self):
        rf, mqtt = _make_rangefinder()
        rf.connect()
        rf.disconnect()
        assert mqtt.disconnect_called == 1
        assert rf._is_connected is False

    def test_get_status_before_any_reading(self):
        rf, mqtt = _make_rangefinder()
        rf.connect()
        status = rf.get_status()
        assert status["is_connected"] is True
        assert status["latest_distance_mm"] is None

    def test_get_distance_from_mqtt_message(self):
        """Push a realistic AL1342 event and confirm distance_mm is returned."""
        rf, mqtt = _make_rangefinder(pdin_port=1)
        rf.connect()
        mqtt.push("laguna/od2000", _make_al1342_event(1, _encode_distance_nm(400_000_000)))
        dist = rf.get_distance_mm()
        assert dist is not None
        assert abs(dist - 400.0) < 0.001

    def test_get_distance_returns_none_when_no_messages(self):
        rf, mqtt = _make_rangefinder()
        rf.connect()
        assert rf.get_distance_mm() is None

    def test_get_latest_sample_returns_wall_time(self):
        """get_latest_sample() returns (wall_time, distance_mm)."""
        rf, mqtt = _make_rangefinder(pdin_port=1)
        rf.connect()
        mqtt.push("laguna/od2000", _make_al1342_event(1, _encode_distance_nm(200_000_000)))
        sample = rf.get_latest_sample()
        assert sample is not None
        wall_time, distance_mm = sample
        assert wall_time > 0
        assert abs(distance_mm - 200.0) < 0.001

    def test_wrong_port_in_payload_does_not_crash(self):
        """If the MQTT message has port[2] but pdin_port=1, the decode silently
        fails (the sample is dropped). On hardware: if get_distance_mm() always
        returns None despite MQTT messages arriving, check pdin_port in config.
        """
        rf, mqtt = _make_rangefinder(pdin_port=1)
        rf.connect()
        # Push a message with port 2 data — should be silently skipped
        mqtt.push("laguna/od2000", _make_al1342_event(2, _encode_distance_nm(200_000_000)))
        assert rf.get_distance_mm() is None

    def test_offset_mm_applied(self):
        """offset_mm is added to the raw distance_mm."""
        mqtt = FakeMqttSubscriber()
        config = {"topic": "laguna/od2000", "pdin_port": 1, "offset_mm": 50.0}
        rf = RangefinderSubsystem(config, mqtt)
        rf.connect()
        mqtt.push("laguna/od2000", _make_al1342_event(1, _encode_distance_nm(200_000_000)))
        dist = rf.get_distance_mm()
        assert abs(dist - 250.0) < 0.001

    def test_sample_count_increments(self):
        rf, mqtt = _make_rangefinder()
        rf.connect()
        for _ in range(5):
            mqtt.push("laguna/od2000", _make_al1342_event(1, _encode_distance_nm(300_000_000)))
        rf.get_distance_mm()
        assert rf._sample_count == 5

    def test_get_status_after_readings(self):
        rf, mqtt = _make_rangefinder()
        rf.connect()
        mqtt.push("laguna/od2000", _make_al1342_event(1, _encode_distance_nm(500_000_000)))
        rf.get_distance_mm()
        status = rf.get_status()
        assert abs(status["latest_distance_mm"] - 500.0) < 0.001
        assert status["sample_count"] == 1
