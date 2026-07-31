"""Tests for the persistent SSH-bridged Pi transport (PiGantryConnection).

All offline — no real SSH/paramiko involved. connect()/_await_ready() (the
only methods that touch paramiko) are exercised via a fake channel injected
directly, matching the same pattern used for EthernetConnection/
RS232Connection's fake socket/serial objects.
"""

import json
import threading

import pytest

from laguna.robot.macron.connection import COMM_TIMEOUT_CODE, SnapMotionError
from laguna.robot.macron.pi_bridge import (
    PiGantryConnection,
    SafeModeConnection,
    check_safe_mode,
    parse_command,
)
from tests.macron_fixtures import FakeSnapConnection


class _FakeChannel:
    def __init__(self):
        self.sent = []
        self.closed = False
        self._exit_ready = False
        self._stdout_queue = []
        self._stderr_queue = []

    def queue_line(self, obj):
        self._stdout_queue.append((json.dumps(obj) + "\n").encode("ascii"))

    def send(self, data):
        self.sent.append(data)

    def recv_ready(self):
        return bool(self._stdout_queue)

    def recv(self, n):
        return self._stdout_queue.pop(0) if self._stdout_queue else b""

    def recv_stderr_ready(self):
        return bool(self._stderr_queue)

    def recv_stderr(self, n):
        return self._stderr_queue.pop(0) if self._stderr_queue else b""

    def exit_status_ready(self):
        return self._exit_ready

    def close(self):
        self.closed = True


def _make_connection(**overrides):
    """Build a PiGantryConnection wired to a fake channel, with the
    background reader thread started manually (bypassing connect(), which
    would try real paramiko/SFTP) — send()/start_scan() now get their
    responses via that thread's dispatch, not by reading the channel
    themselves, so it must be running for these tests to see any response.
    """
    kwargs = dict(
        host="red.dyn.ucr.edu", ssh_user="oak", remote_serial_device="/dev/fake", timeout=1.0
    )
    kwargs.update(overrides)
    conn = PiGantryConnection(**kwargs)
    channel = _FakeChannel()
    conn._channel = channel
    conn._client = object()  # sentinel; not touched by send()/is_connected
    conn._reader_stop.clear()
    conn._reader_thread = threading.Thread(target=conn._reader_loop, daemon=True)
    conn._reader_thread.start()
    return conn, channel


class TestParseCommand:
    def test_single_axis_read(self):
        assert parse_command("A1 ACP") == ("ACP", 0)

    def test_single_axis_write(self):
        assert parse_command("A1 SPD 5000") == ("SPD", 1)

    def test_group_command(self):
        assert parse_command("C1 INI 1 2 3") == ("INI", 3)

    def test_global_command_with_arg(self):
        assert parse_command("INB 3") == ("INB", 1)

    def test_bare_global_command(self):
        assert parse_command("WHT") == ("WHT", 0)

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            parse_command("")


class TestCheckSafeMode:
    @pytest.mark.parametrize("cmd", ["WHT", "INB 1", "A1 ACP", "A1 SPD", "C1 MIF", "A4 CAT"])
    def test_allows_safe_query_commands(self, cmd):
        check_safe_mode(cmd)  # must not raise

    @pytest.mark.parametrize(
        "cmd",
        [
            "A1 BMT 100",    # absolute move
            "A1 JOG 10",     # continuous velocity
            "SOB 4 1",       # output write
            "A1 ACP 0",      # position SET — 0 args allowed (read), 1 arg (write) is not
            "A1 AIC",        # arm hardware capture
            "A1 MVT 5",      # blocking move
            "C1 INI 1 2 3",  # group init
            "A1 ABT",        # abort — not on the read-only list at all
        ],
    )
    def test_blocks_motion_and_set_commands(self, cmd):
        with pytest.raises(SnapMotionError):
            check_safe_mode(cmd)

    def test_unknown_mnemonic_blocked(self):
        with pytest.raises(SnapMotionError):
            check_safe_mode("A1 XYZ")


class TestPiGantryConnectionValidation:
    def test_requires_host(self):
        with pytest.raises(ValueError):
            PiGantryConnection(host="", ssh_user="oak", remote_serial_device="/dev/x")

    def test_requires_ssh_user(self):
        with pytest.raises(ValueError):
            PiGantryConnection(host="red", ssh_user="", remote_serial_device="/dev/x")

    def test_requires_remote_serial_device(self):
        with pytest.raises(ValueError):
            PiGantryConnection(host="red", ssh_user="oak", remote_serial_device="")

    def test_defaults_to_safe_mode(self):
        conn = PiGantryConnection(host="red", ssh_user="oak", remote_serial_device="/dev/x")
        assert conn.safe_mode is True


class TestSendSafeModeGate:
    def test_blocked_command_never_writes_to_channel(self):
        conn, channel = _make_connection(safe_mode=True)
        with pytest.raises(SnapMotionError):
            conn.send("A1 BMT 100")
        assert channel.sent == []  # never reached the wire

    def test_safe_command_is_sent_and_parsed(self):
        conn, channel = _make_connection(safe_mode=True)
        channel.queue_line({"id": 1, "raw": "0 12.000 >"})
        result = conn.send("A1 ACP")
        assert result == "12.000"
        assert len(channel.sent) == 1
        sent_payload = json.loads(channel.sent[0].decode("ascii"))
        assert sent_payload == {"id": 1, "cmd": "A1 ACP", "timeout": 1.0}

    def test_safe_mode_false_allows_motion_commands_through(self):
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "raw": "0 1.000 >"})
        result = conn.send("A1 BMT 100")
        assert result == "1.000"


class TestSendResponseHandling:
    def test_matches_response_by_id_skipping_stale_replies(self):
        conn, channel = _make_connection()
        channel.queue_line({"id": 999, "raw": "0 0.000 >"})  # stale reply for a prior request
        channel.queue_line({"id": 1, "raw": "0 5.000 >"})
        result = conn.send("A1 ACP")
        assert result == "5.000"

    def test_agent_error_response_raises_with_code(self):
        conn, channel = _make_connection()
        channel.queue_line({"id": 1, "error": "blocked by safe_mode", "code": 0})
        with pytest.raises(SnapMotionError):
            conn.send("A1 ACP")

    def test_agent_reports_hardware_error_code(self):
        conn, channel = _make_connection()
        channel.queue_line({"id": 1, "error": "OEM2T error 45", "code": 45})
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("A1 ACP")
        assert exc_info.value.code == 45

    def test_timeout_when_no_response_arrives(self):
        conn, channel = _make_connection(timeout=0.05)
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("WHT")
        assert exc_info.value.code == COMM_TIMEOUT_CODE

    def test_non_json_lines_are_skipped(self):
        conn, channel = _make_connection()
        channel._stdout_queue.append(b"not json at all\n")
        channel.queue_line({"id": 1, "raw": "0 3.000 >"})
        result = conn.send("A1 ACP")
        assert result == "3.000"

    def test_second_send_increments_request_id(self):
        conn, channel = _make_connection()
        channel.queue_line({"id": 1, "raw": "0 1.000 >"})
        conn.send("A1 ACP")
        channel.queue_line({"id": 2, "raw": "0 2.000 >"})
        result = conn.send("A1 ACP")
        assert result == "2.000"


class TestIsConnected:
    def test_true_when_channel_open(self):
        conn, channel = _make_connection()
        assert conn.is_connected is True

    def test_false_when_channel_closed(self):
        conn, channel = _make_connection()
        channel.closed = True
        assert conn.is_connected is False

    def test_false_when_never_connected(self):
        conn = PiGantryConnection(host="red", ssh_user="oak", remote_serial_device="/dev/x")
        assert conn.is_connected is False


class TestSafeModeConnection:
    def test_blocks_unsafe_command_before_reaching_inner_connection(self):
        inner = FakeSnapConnection({})
        wrapped = SafeModeConnection(inner, safe_mode=True)
        with pytest.raises(SnapMotionError):
            wrapped.send("A1 BMT 100")
        assert inner.sent == []

    def test_allows_safe_command_through_to_inner_connection(self):
        inner = FakeSnapConnection({"A1 ACP": "5.000"})
        wrapped = SafeModeConnection(inner, safe_mode=True)
        assert wrapped.send("A1 ACP") == "5.000"
        assert inner.sent == ["A1 ACP"]

    def test_safe_mode_false_passes_everything_through(self):
        inner = FakeSnapConnection({"A1 BMT 100": "1.000"})
        wrapped = SafeModeConnection(inner, safe_mode=False)
        assert wrapped.send("A1 BMT 100") == "1.000"

    def test_delegates_connect_disconnect_is_connected(self):
        inner = FakeSnapConnection({})
        wrapped = SafeModeConnection(inner)
        wrapped.connect()
        assert wrapped.is_connected is True
        wrapped.disconnect()
        assert wrapped.is_connected is False


class TestDisconnect:
    def test_sends_close_op_and_closes_channel(self):
        conn, channel = _make_connection()
        conn.disconnect()
        assert channel.closed is True
        assert len(channel.sent) == 1
        assert json.loads(channel.sent[0].decode("ascii")) == {"op": "close"}
        assert conn._channel is None


# ---------------------------------------------------------------------------
# Background reader dispatch — scan messages arrive asynchronously, not as
# a reply to any one send()/start_scan() call, so they must be routed to a
# dedicated queue rather than confused with an in-flight request's response.
# ---------------------------------------------------------------------------


class TestReaderDispatch:
    def test_scan_done_routed_to_scan_result_queue_not_pending(self):
        """A scan_done message must never be handed to a send()/start_scan()
        caller waiting on a *different* id — it only belongs in
        wait_for_scan_result(). On hardware: if this routing is wrong, a
        scan's completion could be silently swallowed by an unrelated
        interactive command's timeout-bound wait, or vice versa.
        """
        conn, channel = _make_connection()
        channel.queue_line({"scan_done": True, "id": 7, "csv_path": "/tmp/x.csv", "samples": 10})
        result = conn.wait_for_scan_result(timeout=2.0)
        assert result["scan_done"] is True
        assert result["csv_path"] == "/tmp/x.csv"

    def test_scan_error_routed_to_scan_result_queue(self):
        conn, channel = _make_connection()
        channel.queue_line({"scan_error": "serial timeout", "id": 3})
        result = conn.wait_for_scan_result(timeout=2.0)
        assert result["scan_error"] == "serial timeout"

    def test_id_matched_response_still_routes_to_the_right_send_call(self):
        """Interleaving a scan_done with an ordinary id-matched response
        must not confuse the two — each goes to its own destination.
        """
        conn, channel = _make_connection()
        channel.queue_line({"scan_done": True, "id": 99, "samples": 1})
        channel.queue_line({"id": 1, "raw": "0 42.000 >"})
        result = conn.send("A1 ACP")
        assert result == "42.000"
        scan_result = conn.wait_for_scan_result(timeout=2.0)
        assert scan_result["scan_done"] is True

    def test_pong_and_scan_stop_ack_do_not_raise_or_hang(self):
        """Housekeeping replies with no "id" must be silently absorbed by
        the reader thread, not logged as unmatched forever or crash it.
        """
        conn, channel = _make_connection()
        channel.queue_line({"pong": True})
        channel.queue_line({"scan_stop_ack": True})
        channel.queue_line({"id": 1, "raw": "0 1.000 >"})
        result = conn.send("A1 ACP")
        assert result == "1.000"


# ---------------------------------------------------------------------------
# Scan control API
# ---------------------------------------------------------------------------


class TestStartScan:
    def test_happy_path_returns_ack(self):
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({
            "id": 1, "scan_started": True,
            "start_pos_mm": 100.0, "accel_mm_s2": 10.0, "decel_mm_s2": 10.0,
        })
        ack = conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        assert ack["scan_started"] is True
        assert ack["start_pos_mm"] == 100.0
        sent_payload = json.loads(channel.sent[0].decode("ascii"))
        assert sent_payload["op"] == "scan_start"
        assert sent_payload["axis"] == "A1"
        assert sent_payload["al1342_host"] == "192.168.1.251"

    def test_sensor_defaults_to_od2000(self):
        """Callers that don't pass sensor at all (every pre-existing call
        site) must still send an explicit "od2000" — gantry_agent.py reads
        msg.get("sensor", "od2000") but sending it explicitly here removes
        any doubt about client/agent default drift."""
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({
            "id": 1, "scan_started": True,
            "start_pos_mm": 100.0, "accel_mm_s2": 10.0, "decel_mm_s2": 10.0,
        })
        conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        sent_payload = json.loads(channel.sent[0].decode("ascii"))
        assert sent_payload["sensor"] == "od2000"

    def test_sensor_passed_through_when_given(self):
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({
            "id": 1, "scan_started": True,
            "start_pos_mm": 100.0, "accel_mm_s2": 10.0, "decel_mm_s2": 10.0,
        })
        conn.start_scan("A2", 200.0, 3.0, "192.168.1.251", 7, "/tmp/out.csv",
                         sensor="wtt12l_powerprox")
        sent_payload = json.loads(channel.sent[0].decode("ascii"))
        assert sent_payload["sensor"] == "wtt12l_powerprox"

    def test_sets_is_scan_running_true_on_success(self):
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "scan_started": True, "start_pos_mm": 0.0,
                             "accel_mm_s2": 1.0, "decel_mm_s2": 1.0})
        conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        assert conn.is_scan_running is True

    def test_agent_rejection_raises_and_does_not_set_running(self):
        """On hardware: agent rejects scan_start if it was launched without
        --allow-motion, or if a scan is already in progress there.
        """
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "error": "scan already in progress"})
        with pytest.raises(SnapMotionError):
            conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        assert conn.is_scan_running is False

    def test_raises_immediately_if_already_running_client_side(self):
        """If this PC-side object already believes a scan is running, don't
        even send a second scan_start — fail fast client-side.
        """
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "scan_started": True, "start_pos_mm": 0.0,
                             "accel_mm_s2": 1.0, "decel_mm_s2": 1.0})
        conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        with pytest.raises(SnapMotionError):
            conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out2.csv")

    def test_timeout_waiting_for_ack(self):
        conn, channel = _make_connection(safe_mode=False, timeout=0.05)
        with pytest.raises(SnapMotionError) as exc_info:
            conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        assert exc_info.value.code == COMM_TIMEOUT_CODE


class TestStopScan:
    def test_sends_scan_stop_op(self):
        conn, channel = _make_connection()
        conn.stop_scan()
        assert len(channel.sent) == 1
        assert json.loads(channel.sent[0].decode("ascii")) == {"op": "scan_stop"}


class TestWaitForScanResult:
    def test_timeout_raises(self):
        conn, channel = _make_connection(timeout=1.0)
        with pytest.raises(SnapMotionError) as exc_info:
            conn.wait_for_scan_result(timeout=0.05)
        assert exc_info.value.code == COMM_TIMEOUT_CODE

    def test_clears_is_scan_running(self):
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "scan_started": True, "start_pos_mm": 0.0,
                             "accel_mm_s2": 1.0, "decel_mm_s2": 1.0})
        conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        assert conn.is_scan_running is True
        channel.queue_line({"scan_done": True, "id": 1, "samples": 5})
        conn.wait_for_scan_result(timeout=2.0)
        assert conn.is_scan_running is False


class TestSendBlockedDuringScan:
    def test_send_raises_while_scan_running(self):
        """Interactive commands are refused client-side while a scan is
        active — the agent-side serial lock would make interleaving safe,
        but disallowing it here keeps behavior predictable (no jogging an
        axis mid-scan). On hardware: if this check is bypassed, a manual
        move command could race the scan's own BMT/MIF polling — still
        safe at the wire (locked), but confusing and unsupported.
        """
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "scan_started": True, "start_pos_mm": 0.0,
                             "accel_mm_s2": 1.0, "decel_mm_s2": 1.0})
        conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        with pytest.raises(SnapMotionError, match="scan in progress"):
            conn.send("A1 ACP")

    def test_send_allowed_again_after_scan_completes(self):
        conn, channel = _make_connection(safe_mode=False)
        channel.queue_line({"id": 1, "scan_started": True, "start_pos_mm": 0.0,
                             "accel_mm_s2": 1.0, "decel_mm_s2": 1.0})
        conn.start_scan("A1", 500.0, 5.0, "192.168.1.251", 2, "/tmp/out.csv")
        channel.queue_line({"scan_done": True, "id": 1, "samples": 5})
        conn.wait_for_scan_result(timeout=2.0)

        channel.queue_line({"id": 2, "raw": "0 7.000 >"})
        result = conn.send("A1 ACP")
        assert result == "7.000"
