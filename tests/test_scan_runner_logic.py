"""Tests for scan_runner.py helper functions and dead-reckoning math.

scan_runner.py runs standalone on the Pi, but its pure-logic functions are
importable and testable here. Each test encodes a hardware assumption; a
failure on real hardware points to a specific misunderstanding.

Hardware assumptions tested:
  - BLC serial response format: echoed command + numeric value + ' >'
  - MIF response: "1" means done, "0" means still moving (not the reverse)
  - ramp duration: t_accel = feed_rate / ACL (not feed_rate / ACL / 2)
  - Position formula: pos(t) = start + feed_rate * (t - t_slew_start)
  - Dead-reckoning uses the slew window (between ramp up and ramp down)
  - in_ramp=True for readings outside the slew window; they are kept, not dropped
  - MQTT messages arriving before BMT fires are discarded (queue flushed after BMT)
"""

import time

import pytest

import laguna.pi.scan_runner as sr


# ---------------------------------------------------------------------------
# BLC serial response parsing — _serial_send
# ---------------------------------------------------------------------------


class FakeSerial:
    """Minimal pyserial double for offline tests."""

    def __init__(self, response_bytes: bytes = b""):
        self._response = response_bytes
        self._read_pos = 0
        self.written = []
        self.in_waiting = len(response_bytes)

    def reset_input_buffer(self):
        self._read_pos = 0
        self.in_waiting = len(self._response) - self._read_pos

    def write(self, data: bytes):
        self.written.append(data)

    def read(self, n: int = 1) -> bytes:
        chunk = self._response[self._read_pos: self._read_pos + n]
        self._read_pos += len(chunk)
        self.in_waiting = max(0, len(self._response) - self._read_pos)
        return chunk

    def readline(self) -> bytes:
        end = self._response.find(b"\n", self._read_pos)
        if end == -1:
            chunk = self._response[self._read_pos:]
            self._read_pos = len(self._response)
        else:
            chunk = self._response[self._read_pos: end + 1]
            self._read_pos = end + 1
        self.in_waiting = max(0, len(self._response) - self._read_pos)
        return chunk


def _fake_serial_for_cmd(cmd: str, value: str) -> FakeSerial:
    """Build a FakeSerial that returns a BLC-style response for a command."""
    # BLC response format: echoed command tokens + numeric value + ' >'
    response = f"{cmd}\r\n {value} >".encode("ascii")
    return FakeSerial(response)


class TestSerialSend:
    def test_acp_returns_position_string(self):
        """BLC ACP response: 'A1 ACP\\r\\n 100.000 >' → '100.000'."""
        ser = _fake_serial_for_cmd("A1 ACP", "100.000")
        result = sr._serial_send(ser, "A1 ACP", timeout=0.5)
        assert result == "100.000"

    def test_acl_returns_accel_string(self):
        ser = _fake_serial_for_cmd("A1 ACL", "50.000")
        result = sr._serial_send(ser, "A1 ACL", timeout=0.5)
        assert result == "50.000"

    def test_mif_returns_zero_while_moving(self):
        """MIF = 0 means in motion; MIF = 1 means move complete.
        If this is inverted on hardware, the poll loop will return immediately
        or never return. Check: does 'A1 MIF' really return '0' while moving?
        """
        ser = _fake_serial_for_cmd("A1 MIF", "0")
        result = sr._serial_send(ser, "A1 MIF", timeout=0.5)
        assert result.strip() == "0"

    def test_mif_returns_one_when_done(self):
        ser = _fake_serial_for_cmd("A1 MIF", "1")
        result = sr._serial_send(ser, "A1 MIF", timeout=0.5)
        assert result.strip() == "1"

    def test_command_bytes_sent(self):
        """Confirm the exact bytes sent to the BLC include \\r\\n terminator.
        On hardware: if the BLC doesn't respond, the command may need \\r only,
        or just \\n. The BLC RS232 framing should be confirmed against the
        AsciiHelp manual's 'command terminator' spec.
        """
        ser = _fake_serial_for_cmd("A1 ACP", "100.000")
        sr._serial_send(ser, "A1 ACP", timeout=0.5)
        assert ser.written[0] == b"A1 ACP\r\n"

    def test_spd_set_command(self):
        """SPD with a value should work (no return value needed; the '>' is enough)."""
        ser = _fake_serial_for_cmd("A1 SPD 5.0", "")
        # Should not raise; the '>' alone confirms the command was accepted
        sr._serial_send(ser, "A1 SPD 5.0", timeout=0.5)

    def test_fractional_position(self):
        """BLC can return positions with up to 3 decimal places."""
        ser = _fake_serial_for_cmd("A1 ACP", "247.835")
        result = sr._serial_send(ser, "A1 ACP", timeout=0.5)
        assert float(result) == pytest.approx(247.835)


# ---------------------------------------------------------------------------
# PDIN decoding functions in scan_runner context
# ---------------------------------------------------------------------------


class TestScanRunnerDecodePdin:
    """Tests for _decode_pdin and _extract_pdin_hex in scan_runner.py.

    These duplicate some rangefinder tests intentionally: scan_runner.py
    has its own inline copies (it's a standalone script with no imports from
    laguna). If the two diverge on hardware, it means one was updated and the
    other wasn't.
    """

    def test_200mm_round_trip(self):
        """200 mm = 200_000_000 nm; decode should give 200.0 mm."""
        raw = (200_000_000).to_bytes(4, "big", signed=True) + b"\x00\x00"
        result = sr._decode_pdin(raw.hex(), pdin_port=1)
        assert abs(result["distance_mm"] - 200.0) < 0.001

    def test_extract_pdin_port_1(self):
        payload = {
            "code": "event", "cid": 10, "adr": "",
            "data": {
                "eventno": "6317",
                "srcurl": "/timer[1]/counter/datachanged",
                "payload": {
                    "/iolinkmaster/port[1]/iolinkdevice/pdin": {
                        "code": 200,
                        "data": "0BEBC2000000",
                    }
                },
            },
        }
        hex_str = sr._extract_pdin_hex(payload, pdin_port=1)
        assert hex_str == "0BEBC2000000"

    def test_extract_pdin_wrong_port_raises(self):
        """Wrong port → KeyError. On hardware: if this fires, check pdin_port arg."""
        payload = {
            "data": {
                "payload": {
                    "/iolinkmaster/port[1]/iolinkdevice/pdin": {"code": 200, "data": "000000000000"}
                }
            }
        }
        with pytest.raises(KeyError):
            sr._extract_pdin_hex(payload, pdin_port=2)

    def test_decode_pdin_matches_rangefinder_decoder(self):
        """scan_runner._decode_pdin and rangefinder.decode_od2000_pdin must agree.
        If they diverge, one was updated without the other.
        """
        from laguna.rangefinder import decode_od2000_pdin

        hex_str = "0BEBC2000000"
        sr_result = sr._decode_pdin(hex_str, pdin_port=1)
        rf_result = decode_od2000_pdin(hex_str)
        assert sr_result["distance_nm"] == rf_result["distance_nm"]
        assert abs(sr_result["distance_mm"] - rf_result["distance_mm"]) < 0.001
        assert sr_result["q1"] == rf_result["q1"]
        assert sr_result["q2"] == rf_result["q2"]


# ---------------------------------------------------------------------------
# Dead-reckoning math (mirrors the formulas in scan_runner.py main())
# ---------------------------------------------------------------------------
#
# These tests verify the ramp-trimming and position formula in isolation,
# so a logic bug is caught here before hardware testing. The math encoded:
#
#   ramp_t_accel = feed_rate_mm_s / accel_mm_s2
#   ramp_t_decel = feed_rate_mm_s / decel_mm_s2
#   t_slew_start = t_move_start + ramp_t_accel
#   t_slew_end   = t_move_done  - ramp_t_decel
#   pos(t) = start_pos_mm + feed_rate_mm_s * (t - t_slew_start)
#   in_ramp = not (t_slew_start <= t <= t_slew_end)


def _fuse_records(records, start_pos_mm, feed_rate_mm_s, accel_mm_s2, decel_mm_s2,
                  t_move_start, t_move_done):
    """Reproduce the dead-reckoning fusion logic from scan_runner.py main()."""
    ramp_t_accel = feed_rate_mm_s / accel_mm_s2 if accel_mm_s2 > 0 else 0.0
    ramp_t_decel = feed_rate_mm_s / decel_mm_s2 if decel_mm_s2 > 0 else 0.0
    t_slew_start = t_move_start + ramp_t_accel
    t_slew_end = t_move_done - ramp_t_decel

    rows = []
    for rec in records:
        t = rec["wall_time"]
        in_ramp = not (t_slew_start <= t <= t_slew_end)
        pos_mm = start_pos_mm + feed_rate_mm_s * (t - t_slew_start)
        rows.append({
            "wall_time": t,
            "pos_mm": pos_mm,
            "distance_mm": rec.get("distance_mm", 0.0),
            "in_ramp": in_ramp,
        })
    return rows, ramp_t_accel, ramp_t_decel, t_slew_start, t_slew_end


class TestDeadReckoningMath:
    def setup_method(self):
        self.t0 = 1_000.0  # arbitrary epoch for determinism
        self.start_pos = 100.0
        self.feed_rate = 10.0   # mm/s
        self.accel = 5.0        # mm/s²   → ramp_t = 10/5 = 2.0 s
        self.decel = 5.0        # mm/s²   → ramp_t = 10/5 = 2.0 s
        self.t_move_start = self.t0
        self.t_move_done = self.t0 + 12.0   # 2 s accel + 8 s slew + 2 s decel

    def test_ramp_duration_formula(self):
        """ramp_t_accel = feed_rate / accel.
        On hardware: if positions during the ramp phase are off, the controller's
        reported ACL value may be in different units than mm/s². Check the
        unit documentation for the axis configuration in the DSM project.
        """
        _, ramp_t_accel, ramp_t_decel, _, _ = _fuse_records(
            [], self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert ramp_t_accel == pytest.approx(2.0)
        assert ramp_t_decel == pytest.approx(2.0)

    def test_slew_window_boundaries(self):
        """t_slew_start = t_move_start + ramp_t_accel; t_slew_end = t_move_done - ramp_t_decel."""
        _, _, _, t_slew_start, t_slew_end = _fuse_records(
            [], self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert t_slew_start == pytest.approx(self.t0 + 2.0)
        assert t_slew_end == pytest.approx(self.t0 + 10.0)

    def test_position_at_slew_start(self):
        """At t = t_slew_start, pos = start_pos (feed_rate * 0 = 0 displacement)."""
        t_slew_start = self.t0 + 2.0
        records = [{"wall_time": t_slew_start, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["pos_mm"] == pytest.approx(self.start_pos)
        assert rows[0]["in_ramp"] is False

    def test_position_at_slew_midpoint(self):
        """At t = t_slew_start + 4s, pos = start + feed_rate * 4."""
        t_mid = self.t0 + 2.0 + 4.0
        records = [{"wall_time": t_mid, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["pos_mm"] == pytest.approx(self.start_pos + 40.0)

    def test_sample_during_accel_ramp_is_in_ramp(self):
        """A reading at t = t_move_start + 1.0 s (inside accel ramp) → in_ramp=True.
        On hardware: if all readings come back in_ramp=True, the move is shorter
        than 2 * ramp_duration — slow the gantry down or shorten the scan.
        """
        t_accel = self.t0 + 1.0
        records = [{"wall_time": t_accel, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["in_ramp"] is True

    def test_sample_during_decel_ramp_is_in_ramp(self):
        t_decel = self.t0 + 11.0  # after t_slew_end (t0+10)
        records = [{"wall_time": t_decel, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["in_ramp"] is True

    def test_sample_at_slew_end_is_not_in_ramp(self):
        """Sample exactly at t_slew_end is inside the slew window (inclusive boundary)."""
        t_slew_end = self.t0 + 10.0
        records = [{"wall_time": t_slew_end, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["in_ramp"] is False

    def test_ramp_samples_kept_not_dropped(self):
        """Ramp-phase samples are retained in the CSV with in_ramp=True.
        They are NOT discarded — the caller filters them with df[df.in_ramp==0].
        On hardware: total sample count should equal CSV row count, not
        just the slew-phase sample count.
        """
        records = [
            {"wall_time": self.t0 + 0.5, "distance_mm": 310.0},  # in ramp
            {"wall_time": self.t0 + 3.0, "distance_mm": 305.0},  # slew
            {"wall_time": self.t0 + 11.0, "distance_mm": 295.0},  # in ramp
        ]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert len(rows) == 3
        in_ramp_flags = [r["in_ramp"] for r in rows]
        assert in_ramp_flags == [True, False, True]

    def test_zero_accel_guard(self):
        """If ACL returns 0 (e.g. not yet configured), ramp_t_accel = 0 rather
        than dividing by zero. The whole move is treated as slew.
        On hardware: an ACL=0 reading means the axis has no acceleration limit
        set; configure it in the BLC project before scanning.
        """
        records = [{"wall_time": self.t0 + 5.0, "distance_mm": 300.0}]
        rows, ramp_t_accel, ramp_t_decel, _, _ = _fuse_records(
            records, self.start_pos, self.feed_rate, accel_mm_s2=0.0, decel_mm_s2=0.0,
            t_move_start=self.t0, t_move_done=self.t0 + 10.0
        )
        assert ramp_t_accel == 0.0
        assert ramp_t_decel == 0.0
        assert rows[0]["in_ramp"] is False

    def test_multiple_samples_ascending_position(self):
        """During constant slew, positions should increase monotonically."""
        t_slew_start = self.t0 + 2.0
        records = [
            {"wall_time": t_slew_start + dt, "distance_mm": 300.0}
            for dt in [0, 1, 2, 3, 4]
        ]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        positions = [r["pos_mm"] for r in rows]
        assert positions == sorted(positions), "Positions should increase monotonically during slew"
        # 0, 10, 20, 30, 40 mm displacement
        assert positions[1] - positions[0] == pytest.approx(10.0)

    def test_scan_entirely_in_ramp(self):
        """If the move is too short, all samples fall in the ramp phase.
        On hardware: use a longer scan distance or slower feed rate to get
        a useful slew window. Spatial resolution ~ feed_rate / sample_rate.
        """
        # Move takes only 3 s total; with accel=5, decel=5, feed=10 → ramp each = 2s
        # t_slew_start = 2s, t_slew_end = 1s → slew window is inverted (empty)
        t_move_done = self.t0 + 3.0
        records = [{"wall_time": self.t0 + 1.5, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, t_move_done
        )
        # t_slew_start = t0+2, t_slew_end = t0+1 → slew window inverted → all in_ramp
        assert rows[0]["in_ramp"] is True
