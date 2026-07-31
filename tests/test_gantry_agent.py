"""Tests for gantry_agent.py — the Pi-resident agent that now owns both
interactive axis commands AND full topographic scans.

gantry_agent.py runs standalone on the Pi, but its pure-logic functions and
classes are importable and testable here — same pattern already used for
the (now-retired) scan_runner.py in test_scan_runner_logic.py. Several
tests below are ported verbatim from that file; the logic didn't change,
only which module hosts it.

Hardware assumptions tested:
  - BLC serial response envelope: "0 <value> >" success, "<code> >" error
    (see _parse_blc_response — duplicated from connection.py's
    _parse_response, since this agent must stay standalone/no-laguna-import)
  - MIF response: "1" means done, "0" means still moving (not the reverse)
  - ramp duration: t_accel = feed_rate / ACL (not feed_rate / ACL / 2)
  - Position formula: pos(t) = start + feed_rate * (t - t_slew_start)
  - in_ramp=True for readings outside the slew window; they are kept, not dropped
  - OD2000 data comes from polling AL1342's pdin/getdata over HTTP (confirmed
    ~380 Hz on hardware 2026-07-28), not MQTT subscribe (~2 Hz ceiling)
  - Laser on/off is OD2000 IODD index 97/0: 0 = on, 1 = off (confirmed on
    hardware 2026-07-28 — do not swap these, 1 turns the laser OFF)
  - SerialBridge.send() holds its lock for the full write+read round trip,
    so two threads (interactive command handler + scan worker) can never
    interleave bytes on the wire — the documented prior failure mode
    (pipelined commands hung the BLC controller, requiring a power-cycle)
"""

import csv
import json
import queue
import threading
import time

import pytest

import laguna.robot.macron.gantry_agent as ga


# ---------------------------------------------------------------------------
# parse_command / check_safe_mode — agent's own independent copy
# ---------------------------------------------------------------------------


class TestCheckSafeModeAgentSide:
    def test_allows_safe_query_command(self):
        ga.check_safe_mode("A1 ACP")  # must not raise

    def test_blocks_bmt(self):
        with pytest.raises(PermissionError):
            ga.check_safe_mode("A1 BMT 100")

    def test_scan_start_gate_uses_bmt_check(self):
        """This is exactly the check scan_start performs before doing
        anything else — BMT is not on the allowlist at all, so scan_start
        is unconditionally blocked while safe_mode is True, matching how a
        direct 'A1 BMT ...' command is already blocked today.
        """
        with pytest.raises(PermissionError):
            ga.check_safe_mode("A1 BMT 500.0")


# ---------------------------------------------------------------------------
# _parse_blc_response — the agent's own local parser for the scan thread
# ---------------------------------------------------------------------------


class TestParseBlcResponse:
    def test_success_envelope_returns_value(self):
        assert ga._parse_blc_response("0 12.345 >") == "12.345"

    def test_success_envelope_no_value_returns_zero(self):
        assert ga._parse_blc_response("0 >") == "0"

    def test_error_envelope_raises(self):
        with pytest.raises(ValueError):
            ga._parse_blc_response("45 >")

    def test_empty_response_raises(self):
        with pytest.raises(ValueError):
            ga._parse_blc_response(">")

    def test_mif_done_value(self):
        assert ga._parse_blc_response("0 1 >") == "1"

    def test_mif_moving_value(self):
        assert ga._parse_blc_response("0 0 >") == "0"


# ---------------------------------------------------------------------------
# PDIN decoding — ported from test_scan_runner_logic.py
# ---------------------------------------------------------------------------


class TestDecodePdin:
    def test_200mm_round_trip(self):
        raw = (200_000_000).to_bytes(4, "big", signed=True) + b"\x00\x00"
        result = ga._decode_pdin(raw.hex(), pdin_port=1)
        assert abs(result["distance_mm"] - 200.0) < 0.001

    def test_confirmed_hardware_reading(self):
        """Confirmed on hardware 2026-07-28: this exact hex string was read
        from the AL1342 with the OD2000 at a physically measured 808.4mm
        +/- 0.1mm. Decoded distance (808.28mm) matched within tolerance,
        confirming the big-endian nm assumption.
        """
        result = ga._decode_pdin("302D56F7F700", pdin_port=2)
        assert abs(result["distance_mm"] - 808.2778) < 0.001
        assert result["scale"] == 247  # NOT 0 as originally assumed — unused, unexplained
        assert result["q1"] is False
        assert result["q2"] is False

    def test_little_endian_gives_nonsensical_result(self):
        raw = bytes.fromhex("302D56F7F700")
        distance_nm_le = int.from_bytes(raw[0:4], "little", signed=True)
        assert distance_nm_le < 0

    def test_extract_pdin_from_getdata_response(self):
        resp = {"cid": -1, "data": {"value": "0BEBC2000000"}, "code": 200}
        assert ga._extract_pdin_hex_from_getdata(resp) == "0BEBC2000000"


# ---------------------------------------------------------------------------
# WTT12L PowerProx (via DP4200 bridge) decode — mirrors
# laguna.rangefinder.decode_dp4200_wtt12l_analog_pdin(), duplicated here
# per this file's standalone-deployment constraint (see module docstring).
# Hardware-confirmed readings from docs/WTT12L_POWERPROX_SETUP.md.
# ---------------------------------------------------------------------------


class TestDecodeDp4200Wtt12lPdin:
    def test_known_600mm_hardware_reading(self):
        result = ga._decode_dp4200_wtt12l_pdin("2890FD01", pdin_port=7)
        assert abs(result["current_ma"] - 10.384) < 0.001
        assert abs(result["distance_mm"] - 600) < 50  # ~20-30mm slop expected, see decode docstring

    def test_known_1115mm_hardware_reading(self):
        result = ga._decode_dp4200_wtt12l_pdin("3F02FD01", pdin_port=7)
        assert abs(result["current_ma"] - 16.130) < 0.001
        assert abs(result["distance_mm"] - 1115) < 50

    def test_channel2_ignored(self):
        """Channel 2 (bytes 2-3) is confirmed dead/unconnected on hardware —
        should not appear in the decoded dict at all."""
        result = ga._decode_dp4200_wtt12l_pdin("2890FD01", pdin_port=7)
        assert set(result.keys()) == {"current_ma", "distance_mm"}

    def test_pdin_port_accepted_but_unused(self):
        """pdin_port only exists for call-signature symmetry with
        _decode_pdin (both are used interchangeably via SENSOR_DECODERS) —
        changing it should not change the decode."""
        a = ga._decode_dp4200_wtt12l_pdin("2890FD01", pdin_port=1)
        b = ga._decode_dp4200_wtt12l_pdin("2890FD01", pdin_port=7)
        assert a == b


class TestSensorDecoders:
    def test_registry_has_both_sensors(self):
        assert set(ga.SENSOR_DECODERS) == {"od2000", "wtt12l_powerprox"}

    def test_od2000_maps_to_decode_pdin(self):
        assert ga.SENSOR_DECODERS["od2000"] is ga._decode_pdin

    def test_wtt12l_powerprox_maps_to_dp4200_decode(self):
        assert ga.SENSOR_DECODERS["wtt12l_powerprox"] is ga._decode_dp4200_wtt12l_pdin


# ---------------------------------------------------------------------------
# Laser on/off — ported from test_scan_runner_logic.py
# ---------------------------------------------------------------------------


class TestSetLaser:
    def _make_fake_conn(self, captured, code=200):
        class FakeConn:
            def __init__(self, *a, **kw):
                pass

            def request(self, method, path, body=None, headers=None):
                captured.append(json.loads(body))

            def getresponse(self):
                class R:
                    def read(self):
                        return json.dumps({"cid": -1, "code": code}).encode()
                return R()

            def close(self):
                pass

        return FakeConn

    def test_laser_on_sends_value_00(self, monkeypatch):
        captured = []
        monkeypatch.setattr(ga.http.client, "HTTPConnection", self._make_fake_conn(captured))
        assert ga._set_laser("192.168.1.251", pdin_port=2, on=True) is True
        assert captured[0]["data"]["value"] == "00"
        assert captured[0]["data"]["index"] == 97
        assert captured[0]["data"]["subindex"] == 0

    def test_laser_off_sends_value_01(self, monkeypatch):
        captured = []
        monkeypatch.setattr(ga.http.client, "HTTPConnection", self._make_fake_conn(captured))
        assert ga._set_laser("192.168.1.251", pdin_port=2, on=False) is True
        assert captured[0]["data"]["value"] == "01"

    def test_uses_correct_port_in_address(self, monkeypatch):
        captured = []
        monkeypatch.setattr(ga.http.client, "HTTPConnection", self._make_fake_conn(captured))
        ga._set_laser("192.168.1.251", pdin_port=5, on=True)
        assert "port[5]" in captured[0]["adr"]

    def test_write_failure_returns_false_not_raises(self, monkeypatch):
        class FailConn:
            def __init__(self, *a, **kw):
                pass

            def request(self, *a, **kw):
                raise ConnectionError("simulated AL1342 unreachable")

            def close(self):
                pass

        monkeypatch.setattr(ga.http.client, "HTTPConnection", FailConn)
        assert ga._set_laser("192.168.1.251", pdin_port=2, on=True) is False

    def test_non_200_response_returns_false(self, monkeypatch):
        captured = []
        monkeypatch.setattr(ga.http.client, "HTTPConnection", self._make_fake_conn(captured, code=503))
        assert ga._set_laser("192.168.1.251", pdin_port=2, on=True) is False


# ---------------------------------------------------------------------------
# HTTP polling loop — ported from test_scan_runner_logic.py
# ---------------------------------------------------------------------------


class TestPollPdinLoop:
    def test_samples_queued_while_running(self, monkeypatch):
        class FakeResponse:
            def __init__(self, hex_str):
                self._hex = hex_str

            def read(self):
                return json.dumps({"cid": -1, "data": {"value": self._hex}, "code": 200}).encode()

        class FakeConn:
            def __init__(self, *a, **kw):
                pass

            def request(self, method, path, body=None, headers=None):
                pass

            def getresponse(self):
                return FakeResponse("0BEBC2000000")

            def close(self):
                pass

        monkeypatch.setattr(ga.http.client, "HTTPConnection", FakeConn)

        out_queue: "queue.Queue" = queue.Queue()
        stop_event = threading.Event()

        def stop_after_a_few():
            time.sleep(0.05)
            stop_event.set()

        t = threading.Thread(target=stop_after_a_few)
        t.start()
        ga._poll_pdin_loop("192.168.1.251", "/iolinkmaster/port[2]/iolinkdevice/pdin/getdata",
                           out_queue, stop_event, pdin_port=2)
        t.join()

        assert not out_queue.empty()
        sample = out_queue.get_nowait()
        assert sample["distance_mm"] == pytest.approx(200.0, abs=0.001)
        assert "wall_time" in sample

    def test_reconnects_after_error(self, monkeypatch):
        class FailThenSucceedConn:
            instances = []

            def __init__(self, *a, **kw):
                self.closed = False
                FailThenSucceedConn.instances.append(self)

            def request(self, method, path, body=None, headers=None):
                if len(FailThenSucceedConn.instances) < 2:
                    raise ConnectionError("simulated transient failure")

            def getresponse(self):
                class R:
                    def read(self):
                        return json.dumps({"cid": -1, "data": {"value": "0BEBC2000000"}, "code": 200}).encode()
                return R()

            def close(self):
                self.closed = True

        monkeypatch.setattr(ga.http.client, "HTTPConnection", FailThenSucceedConn)

        out_queue: "queue.Queue" = queue.Queue()
        stop_event = threading.Event()

        def stop_after_a_few():
            time.sleep(0.05)
            stop_event.set()

        t = threading.Thread(target=stop_after_a_few)
        t.start()
        ga._poll_pdin_loop("192.168.1.251", "/iolinkmaster/port[2]/iolinkdevice/pdin/getdata",
                           out_queue, stop_event, pdin_port=2)
        t.join()

        assert len(FailThenSucceedConn.instances) >= 2


# ---------------------------------------------------------------------------
# Dead-reckoning math (mirrors the formulas inline in gantry_agent._run_scan)
# ---------------------------------------------------------------------------
#
# Reproduced here (not imported) so a logic bug is caught before hardware
# testing — same approach test_scan_runner_logic.py used for the retired
# scan_runner.py, since this fusion math is inline in _run_scan, not its
# own separately importable function.


def _fuse_records(records, start_pos_mm, feed_rate_mm_s, accel_mm_s2, decel_mm_s2,
                  t_move_start, t_move_done):
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
        self.t0 = 1_000.0
        self.start_pos = 100.0
        self.feed_rate = 10.0
        self.accel = 5.0
        self.decel = 5.0
        self.t_move_start = self.t0
        self.t_move_done = self.t0 + 12.0

    def test_ramp_duration_formula(self):
        _, ramp_t_accel, ramp_t_decel, _, _ = _fuse_records(
            [], self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert ramp_t_accel == pytest.approx(2.0)
        assert ramp_t_decel == pytest.approx(2.0)

    def test_slew_window_boundaries(self):
        _, _, _, t_slew_start, t_slew_end = _fuse_records(
            [], self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert t_slew_start == pytest.approx(self.t0 + 2.0)
        assert t_slew_end == pytest.approx(self.t0 + 10.0)

    def test_position_at_slew_start(self):
        t_slew_start = self.t0 + 2.0
        records = [{"wall_time": t_slew_start, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["pos_mm"] == pytest.approx(self.start_pos)
        assert rows[0]["in_ramp"] is False

    def test_sample_during_accel_ramp_is_in_ramp(self):
        t_accel = self.t0 + 1.0
        records = [{"wall_time": t_accel, "distance_mm": 300.0}]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert rows[0]["in_ramp"] is True

    def test_ramp_samples_kept_not_dropped(self):
        records = [
            {"wall_time": self.t0 + 0.5, "distance_mm": 310.0},
            {"wall_time": self.t0 + 3.0, "distance_mm": 305.0},
            {"wall_time": self.t0 + 11.0, "distance_mm": 295.0},
        ]
        rows, *_ = _fuse_records(
            records, self.start_pos, self.feed_rate, self.accel, self.decel,
            self.t_move_start, self.t_move_done
        )
        assert len(rows) == 3
        assert [r["in_ramp"] for r in rows] == [True, False, True]

    def test_zero_accel_guard(self):
        records = [{"wall_time": self.t0 + 5.0, "distance_mm": 300.0}]
        rows, ramp_t_accel, ramp_t_decel, _, _ = _fuse_records(
            records, self.start_pos, self.feed_rate, accel_mm_s2=0.0, decel_mm_s2=0.0,
            t_move_start=self.t0, t_move_done=self.t0 + 10.0
        )
        assert ramp_t_accel == 0.0
        assert ramp_t_decel == 0.0
        assert rows[0]["in_ramp"] is False


# ---------------------------------------------------------------------------
# SerialBridge — locking is the critical safety property
# ---------------------------------------------------------------------------


class _NonReentrantFakeSerial:
    """Raises a red flag (records a violation) if write()/read() is called
    while another call is already mid-flight — proves SerialBridge.send()'s
    lock prevents two threads from interleaving a request/response cycle on
    the wire. Without the lock, this test would catch the exact failure
    mode that previously hung the BLC controller (pipelined commands).
    """

    def __init__(self, response: bytes = b" 1.000 >"):
        self._response = response
        self.in_call = False
        self.violations = []
        self.timeout = 5.0
        self._pos = 0

    def reset_input_buffer(self):
        if self.in_call:
            self.violations.append("reset_input_buffer during another call")
        self._pos = 0

    def write(self, data):
        if self.in_call:
            self.violations.append("write during another call")
        self.in_call = True
        time.sleep(0.02)  # widen the window so a missing lock would show up

    def read(self, n=1):
        time.sleep(0.005)
        if self._pos >= len(self._response):
            return b""
        b = self._response[self._pos:self._pos + 1]
        self._pos += 1
        if b == b">":
            self.in_call = False
        return b


class TestSerialBridgeExclusiveOpen:
    """The port must be opened exclusively.

    Linux doesn't lock tty devices by default, so without exclusive=True a
    second agent (or serial_bridge.py) opens the same port and both write to
    the controller. Their bytes splice mid-command and the PLC's ASCII
    interpreter receives garbage — and this interpreter is known to have an
    input it handles by corrupting the responder's program (see
    notes/2026-07-30-responder-node-incident.md). A second opener must fail
    loudly rather than silently corrupt the wire.
    """

    def test_serial_opened_exclusively(self, monkeypatch):
        captured = {}

        class _FakeSerial:
            def __init__(self, port, **kwargs):
                captured["port"] = port
                captured.update(kwargs)

            def reset_input_buffer(self):
                pass

        monkeypatch.setattr(ga.serial, "Serial", _FakeSerial)
        ga.SerialBridge("/dev/ttyUSB0", 9600)
        assert captured.get("exclusive") is True, (
            "SerialBridge must open the port with exclusive=True — without it, "
            "two agents can write to the controller simultaneously"
        )

    def test_open_failure_propagates(self, monkeypatch):
        """A busy port must raise, not be swallowed — main() turns this into
        an actionable 'another process owns the port' message."""

        def _boom(port, **kwargs):
            raise OSError(16, "Device or resource busy")

        monkeypatch.setattr(ga.serial, "Serial", _boom)
        with pytest.raises(OSError):
            ga.SerialBridge("/dev/ttyUSB0", 9600)


class TestSerialBridgeLocking:
    def _make_bridge(self, response=b" 1.000 >"):
        bridge = ga.SerialBridge.__new__(ga.SerialBridge)  # bypass __init__'s real serial.Serial() open
        bridge._ser = _NonReentrantFakeSerial(response)
        bridge._lock = threading.Lock()
        return bridge

    def test_concurrent_sends_never_interleave(self):
        bridge = self._make_bridge()
        results = []

        def worker():
            results.append(bridge.send("A1 ACP", timeout=2.0))

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        assert bridge._ser.violations == []
        assert len(results) == 4
        assert all(r == " 1.000 >" for r in results)

    def test_single_send_returns_raw_text(self):
        bridge = self._make_bridge(response=b" 0 1.000 >")
        result = bridge.send("A1 ACP", timeout=2.0)
        assert result == " 0 1.000 >"

    def test_timeout_raises_when_no_prompt_arrives(self):
        bridge = self._make_bridge(response=b" incomplete")  # never sends '>'
        with pytest.raises(TimeoutError):
            bridge.send("A1 ACP", timeout=0.05)


# ---------------------------------------------------------------------------
# ScanState — one scan at a time
# ---------------------------------------------------------------------------


class TestScanState:
    def test_not_running_initially(self):
        state = ga.ScanState()
        assert state.is_running() is False

    def test_kwargs_passed_to_target(self):
        """The mechanism _run_scan's keyword-only `sensor` param relies on
        — kwargs given to start() must reach target alongside the
        positionally-appended stop_event."""
        received = {}
        done = threading.Event()

        def target(a, b, stop_event, *, sensor="default"):
            received["a"] = a
            received["b"] = b
            received["stop_event"] = stop_event
            received["sensor"] = sensor
            done.set()

        state = ga.ScanState()
        state.start(target, (1, 2), kwargs={"sensor": "wtt12l_powerprox"})
        assert done.wait(timeout=2.0)
        assert received["a"] == 1
        assert received["b"] == 2
        assert isinstance(received["stop_event"], threading.Event)
        assert received["sensor"] == "wtt12l_powerprox"

    def test_kwargs_optional(self):
        """start() without kwargs (the pre-existing call shape, still used
        by every test below this one) must keep working unchanged."""
        done = threading.Event()

        def target(stop_event):
            done.set()

        state = ga.ScanState()
        assert state.start(target, ()) is True
        assert done.wait(timeout=2.0)

    def test_start_launches_thread_and_reports_running(self):
        state = ga.ScanState()
        started_event = threading.Event()
        release_event = threading.Event()

        def target(stop_event):
            started_event.set()
            release_event.wait(timeout=2.0)

        assert state.start(target, ()) is True
        started_event.wait(timeout=2.0)
        assert state.is_running() is True
        release_event.set()

    def test_start_rejects_second_scan_while_first_running(self):
        state = ga.ScanState()
        release_event = threading.Event()

        def target(stop_event):
            release_event.wait(timeout=2.0)

        assert state.start(target, ()) is True
        assert state.start(target, ()) is False
        release_event.set()

    def test_start_succeeds_again_after_previous_scan_finished(self):
        state = ga.ScanState()

        def quick_target(stop_event):
            pass

        assert state.start(quick_target, ()) is True
        time.sleep(0.1)
        assert state.is_running() is False
        assert state.start(quick_target, ()) is True

    def test_request_stop_sets_the_scan_thread_stop_event(self):
        state = ga.ScanState()
        seen_stop_event = []

        def target(stop_event):
            seen_stop_event.append(stop_event)
            stop_event.wait(timeout=2.0)

        state.start(target, ())
        time.sleep(0.05)
        state.request_stop()
        assert seen_stop_event
        assert seen_stop_event[0].wait(timeout=2.0) is True

    def test_request_stop_with_no_scan_running_is_safe(self):
        state = ga.ScanState()
        state.request_stop()  # must not raise


# ---------------------------------------------------------------------------
# _run_scan — the full worker, with a scripted fake bridge
# ---------------------------------------------------------------------------


class _ScriptedBridge:
    """Fake SerialBridge that returns canned '0 <value> >' responses keyed
    by command mnemonic, and records every command sent in order — lets
    _run_scan's control flow (SPD -> BMT -> MIF poll -> ACP -> optional BST)
    be verified without real serial hardware.
    """

    def __init__(self, mif_sequence=("0.000", "0.000", "1.000"), final_acp="500.000"):
        # Realistic BLC-style float values ("1.000"/"0.000"), not bare "1"/"0" —
        # a bare-"1" fixture previously masked the exact bug this class of
        # test now exists to catch (mif_value.strip() == "1" never matches
        # "1.000", so _run_scan's MIF loop would spin forever on real
        # hardware even after the move physically finished — confirmed
        # 2026-07-28 when a real scan hung past its 50s timeout).
        self.sent = []
        self._mif_sequence = list(mif_sequence)
        self._final_acp = final_acp

    def send(self, cmd, timeout):
        self.sent.append(cmd)
        if "MIF" in cmd:
            value = self._mif_sequence.pop(0) if self._mif_sequence else "1.000"
            return f"0 {value} >"
        if "ACP" in cmd:
            return f"0 {self._final_acp} >"
        return "0 >"


@pytest.fixture
def no_network(monkeypatch):
    """Stub out laser control and OD2000 polling so _run_scan tests never
    touch a real network — they only exercise the serial/threading logic.
    """
    monkeypatch.setattr(ga, "_set_laser", lambda *a, **kw: True)

    def fake_poll(al1342_host, pdin_path, out_queue, stop_event, pdin_port, decode_fn=None):
        stop_event.wait(timeout=2.0)  # just idle until told to stop, like a slow/empty poll

    monkeypatch.setattr(ga, "_poll_pdin_loop", fake_poll)
    yield


class TestRunScan:
    def test_normal_completion_emits_scan_done(self, tmp_path, no_network, monkeypatch):
        emitted = []
        monkeypatch.setattr(ga, "_emit", lambda obj: emitted.append(obj))

        bridge = _ScriptedBridge(mif_sequence=("0.000", "1.000"))
        output = str(tmp_path / "profile.csv")
        stop_event = threading.Event()

        ga._run_scan(bridge, 1, "A1", 500.0, 5.0, "192.168.1.251", 2, output,
                     100.0, 10.0, 10.0, tmp_path / "audit.log", stop_event)

        assert any("scan_done" in msg for msg in emitted)
        done_msg = next(msg for msg in emitted if "scan_done" in msg)
        assert done_msg["id"] == 1
        assert done_msg["csv_path"] == output
        assert (tmp_path / "profile.csv").exists()

        # BMT and MIF were sent, BST was NOT (this was a normal finish, not a stop)
        assert any("BMT" in c for c in bridge.sent)
        assert not any("BST" in c for c in bridge.sent)

    def test_stop_event_triggers_bst_before_completion(self, tmp_path, no_network, monkeypatch):
        """The core STOP requirement: setting stop_event mid-scan must
        result in a BST command before the scan reaches its normal
        completion path.
        """
        emitted = []
        monkeypatch.setattr(ga, "_emit", lambda obj: emitted.append(obj))

        bridge = _ScriptedBridge(mif_sequence=("0.000", "0.000", "0.000", "0.000", "0.000"))  # never reaches "1" on its own
        output = str(tmp_path / "profile.csv")
        stop_event = threading.Event()
        stop_event.set()  # already requested before the scan even starts polling MIF

        ga._run_scan(bridge, 2, "A1", 500.0, 5.0, "192.168.1.251", 2, output,
                     100.0, 10.0, 10.0, tmp_path / "audit.log", stop_event)

        assert any("BST" in c for c in bridge.sent)
        assert any("scan_done" in msg for msg in emitted)

    def test_serial_error_emits_scan_error_not_scan_done(self, tmp_path, no_network, monkeypatch):
        emitted = []
        monkeypatch.setattr(ga, "_emit", lambda obj: emitted.append(obj))

        class FailingBridge:
            def send(self, cmd, timeout):
                raise TimeoutError("simulated BLC timeout")

        stop_event = threading.Event()
        ga._run_scan(FailingBridge(), 3, "A1", 500.0, 5.0, "192.168.1.251", 2,
                     str(tmp_path / "profile.csv"), 100.0, 10.0, 10.0,
                     tmp_path / "audit.log", stop_event)

        assert any("scan_error" in msg for msg in emitted)
        assert not any("scan_done" in msg for msg in emitted)

    def test_laser_turned_off_even_after_error(self, tmp_path, monkeypatch):
        """finally: must always turn the laser off, even if the scan
        errors out partway through — a stuck-on laser is the failure mode
        this guards against.
        """
        laser_calls = []
        monkeypatch.setattr(ga, "_set_laser", lambda host, port, on: laser_calls.append(on))
        monkeypatch.setattr(ga, "_emit", lambda obj: None)

        class FailingBridge:
            def send(self, cmd, timeout):
                raise TimeoutError("simulated failure")

        stop_event = threading.Event()
        ga._run_scan(FailingBridge(), 4, "A1", 500.0, 5.0, "192.168.1.251", 2,
                     str(tmp_path / "profile.csv"), 100.0, 10.0, 10.0,
                     tmp_path / "audit.log", stop_event)

        assert laser_calls == [True, False]


# ---------------------------------------------------------------------------
# _run_scan with sensor="wtt12l_powerprox" — the DP4200-bridge path added
# alongside the default OD2000 path
# ---------------------------------------------------------------------------


class TestRunScanWtt12lPowerprox:
    def test_laser_control_skipped(self, tmp_path, monkeypatch):
        """DP4200 is what's actually on pdin_port in this configuration —
        _set_laser() must not be called at all (see _run_scan docstring)."""
        laser_calls = []
        monkeypatch.setattr(ga, "_set_laser", lambda host, port, on: laser_calls.append(on))
        monkeypatch.setattr(ga, "_emit", lambda obj: None)

        def fake_poll(al1342_host, pdin_path, out_queue, stop_event, pdin_port, decode_fn=None):
            stop_event.wait(timeout=2.0)

        monkeypatch.setattr(ga, "_poll_pdin_loop", fake_poll)

        bridge = _ScriptedBridge(mif_sequence=("1.000",))
        stop_event = threading.Event()
        ga._run_scan(bridge, 5, "A1", 500.0, 5.0, "192.168.1.251", 7,
                     str(tmp_path / "profile.csv"), 100.0, 10.0, 10.0,
                     tmp_path / "audit.log", stop_event, sensor="wtt12l_powerprox")

        assert laser_calls == []

    def test_decode_fn_passed_through_to_poll_loop(self, tmp_path, monkeypatch):
        """The whole point of threading `sensor` through: _poll_pdin_loop
        must receive _decode_dp4200_wtt12l_pdin, not the OD2000 default."""
        monkeypatch.setattr(ga, "_set_laser", lambda *a, **kw: True)
        monkeypatch.setattr(ga, "_emit", lambda obj: None)

        received = {}

        def fake_poll(al1342_host, pdin_path, out_queue, stop_event, pdin_port, decode_fn=None):
            received["decode_fn"] = decode_fn
            stop_event.wait(timeout=2.0)

        monkeypatch.setattr(ga, "_poll_pdin_loop", fake_poll)

        bridge = _ScriptedBridge(mif_sequence=("1.000",))
        stop_event = threading.Event()
        ga._run_scan(bridge, 6, "A1", 500.0, 5.0, "192.168.1.251", 7,
                     str(tmp_path / "profile.csv"), 100.0, 10.0, 10.0,
                     tmp_path / "audit.log", stop_event, sensor="wtt12l_powerprox")

        assert received["decode_fn"] is ga._decode_dp4200_wtt12l_pdin

    def test_csv_row_has_current_ma_not_od2000_fields(self, tmp_path, monkeypatch):
        """A DP4200-sourced record has no distance_nm/q1/q2 — the CSV
        writer must handle that (blank fields), not KeyError, and must
        carry current_ma through."""
        monkeypatch.setattr(ga, "_set_laser", lambda *a, **kw: True)
        monkeypatch.setattr(ga, "_emit", lambda obj: None)

        t_sample = [None]

        def fake_poll(al1342_host, pdin_path, out_queue, stop_event, pdin_port, decode_fn=None):
            t_sample[0] = time.time()
            out_queue.put({"current_ma": 10.384, "distance_mm": 618.7, "wall_time": t_sample[0]})
            stop_event.wait(timeout=2.0)

        monkeypatch.setattr(ga, "_poll_pdin_loop", fake_poll)

        bridge = _ScriptedBridge(mif_sequence=("1.000",))
        output = str(tmp_path / "profile.csv")
        stop_event = threading.Event()
        ga._run_scan(bridge, 7, "A1", 500.0, 5.0, "192.168.1.251", 7, output,
                     100.0, 10.0, 10.0, tmp_path / "audit.log", stop_event,
                     sensor="wtt12l_powerprox")

        with open(output, newline="") as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == 1
        row = rows[0]
        assert row["current_ma"] == "10.384"
        assert row["distance_mm"] == "618.7"
        assert row["distance_nm"] == ""
        assert row["q1"] == ""
        assert row["q2"] == ""

    def test_metadata_records_sensor(self, tmp_path, monkeypatch):
        monkeypatch.setattr(ga, "_set_laser", lambda *a, **kw: True)
        monkeypatch.setattr(ga, "_emit", lambda obj: None)

        def fake_poll(al1342_host, pdin_path, out_queue, stop_event, pdin_port, decode_fn=None):
            stop_event.wait(timeout=2.0)

        monkeypatch.setattr(ga, "_poll_pdin_loop", fake_poll)

        bridge = _ScriptedBridge(mif_sequence=("1.000",))
        output = str(tmp_path / "profile.csv")
        stop_event = threading.Event()
        ga._run_scan(bridge, 8, "A1", 500.0, 5.0, "192.168.1.251", 7, output,
                     100.0, 10.0, 10.0, tmp_path / "audit.log", stop_event,
                     sensor="wtt12l_powerprox")

        with open(output.replace(".csv", "_meta.json")) as f:
            meta = json.load(f)
        assert meta["sensor"] == "wtt12l_powerprox"

    def test_unknown_sensor_emits_scan_error(self, tmp_path, monkeypatch):
        """A bad sensor value must be caught and reported as a scan_error
        (KeyError from SENSOR_DECODERS[sensor], caught inside the try),
        not crash the worker thread silently."""
        laser_calls = []
        monkeypatch.setattr(ga, "_set_laser", lambda host, port, on: laser_calls.append(on))
        emitted = []
        monkeypatch.setattr(ga, "_emit", lambda obj: emitted.append(obj))

        bridge = _ScriptedBridge(mif_sequence=("1.000",))
        stop_event = threading.Event()
        ga._run_scan(bridge, 9, "A1", 500.0, 5.0, "192.168.1.251", 7,
                     str(tmp_path / "profile.csv"), 100.0, 10.0, 10.0,
                     tmp_path / "audit.log", stop_event, sensor="nonexistent_sensor")

        assert laser_calls == []  # never got as far as turning it on
        assert any("scan_error" in msg for msg in emitted)
        assert not any("scan_done" in msg for msg in emitted)
