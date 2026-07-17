"""Tests for the persistent SSH-bridged Pi transport (PiGantryConnection).

All offline — no real SSH/paramiko involved. connect()/_await_ready() (the
only methods that touch paramiko) are exercised via a fake channel injected
directly, matching the same pattern used for EthernetConnection/
RS232Connection's fake socket/serial objects.
"""

import json

import pytest

from laguna.robot.macron.connection import COMM_TIMEOUT_CODE, SnapMotionError
from laguna.robot.macron.pi_bridge import PiGantryConnection, check_safe_mode, parse_command


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
    kwargs = dict(
        host="red.dyn.ucr.edu", ssh_user="oak", remote_serial_device="/dev/fake", timeout=1.0
    )
    kwargs.update(overrides)
    conn = PiGantryConnection(**kwargs)
    channel = _FakeChannel()
    conn._channel = channel
    conn._client = object()  # sentinel; not touched by send()/is_connected
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


class TestDisconnect:
    def test_sends_close_op_and_closes_channel(self):
        conn, channel = _make_connection()
        conn.disconnect()
        assert channel.closed is True
        assert len(channel.sent) == 1
        assert json.loads(channel.sent[0].decode("ascii")) == {"op": "close"}
        assert conn._channel is None
