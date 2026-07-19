"""Tests for the macron gantry driver's transport/parsing layer.

Covers the ASCII response envelope parsing (_parse_response), the
read-until-'>' framing logic on both transports, and probe_connection —
all offline, no hardware or real sockets/serial ports involved.
"""

import socket as socket_module

import pytest

from laguna.robot.macron.connection import (
    COMM_TIMEOUT_CODE,
    EthernetConnection,
    RS232Connection,
    SnapMotionError,
    _parse_response,
    probe_connection,
)
from tests.macron_fixtures import FakeSnapConnection


class TestParseResponse:
    """Table-driven envelope parsing: success = '0 <value> >', error = '<code> >'."""

    def test_success_with_value(self):
        assert _parse_response("0 123.000 >") == "123.000"

    def test_success_negative_value(self):
        assert _parse_response("0 -4.899 >") == "-4.899"

    def test_success_value_less_ack(self):
        # e.g. SOB, which acknowledges but the firmware still echoes "0 ... >"
        # with nothing after "0" other than the prompt.
        assert _parse_response("0 >") == "0"

    def test_error_envelope(self):
        with pytest.raises(SnapMotionError) as exc_info:
            _parse_response("45 >")
        assert exc_info.value.code == 45

    def test_error_envelope_unknown_command(self):
        with pytest.raises(SnapMotionError) as exc_info:
            _parse_response("1002 >")
        assert exc_info.value.code == 1002

    def test_does_not_flag_large_success_value_as_error(self):
        # Regression: the old threshold-based parser misidentified any
        # numeric response >= 600 as an error code, even legitimate ones.
        assert _parse_response("0 700.000 >") == "700.000"
        assert _parse_response("0 822536056.000 >") == "822536056.000"

    def test_telnet_echo_before_response(self):
        # Ethernet/telnet-style connections may echo the command back.
        assert _parse_response("A1ACP\r\n0 12.000 >") == "12.000"

    def test_banner_or_extra_lines_discarded(self):
        assert _parse_response("some banner text\r\nmore junk\r\n0 5.000 >") == "5.000"

    def test_commas_as_separators(self):
        assert _parse_response("0,12.000,>") == "12.000"

    def test_empty_response_raises(self):
        with pytest.raises(SnapMotionError) as exc_info:
            _parse_response("")
        assert exc_info.value.code == 0

    def test_whitespace_only_response_raises(self):
        with pytest.raises(SnapMotionError):
            _parse_response("   >")

    def test_unparseable_error_code_raises(self):
        with pytest.raises(SnapMotionError) as exc_info:
            _parse_response("garbage >")
        assert exc_info.value.code == 0


class _FakeSocket:
    """Minimal socket.socket double: recv() returns scripted chunks in order."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.sent = []

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, n):
        if not self._chunks:
            return b""
        chunk = self._chunks.pop(0)
        if isinstance(chunk, Exception):
            raise chunk
        return chunk

    def settimeout(self, t):
        pass

    def close(self):
        pass


class TestEthernetConnectionFraming:
    def _make(self, chunks, timeout=1.0):
        conn = EthernetConnection(host="example.invalid", timeout=timeout)
        conn._socket = _FakeSocket(chunks)
        return conn

    def test_send_assembles_response_across_multiple_recv_calls(self):
        conn = self._make([b"0 ", b"12.000", b" >"])
        assert conn.send("A1ACP") == "12.000"
        assert conn._socket.sent == [b"A1ACP\r"]

    def test_send_raises_on_error_envelope(self):
        conn = self._make([b"45 >"])
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("A99ACP")
        assert exc_info.value.code == 45

    def test_socket_timeout_raises_comm_timeout(self):
        conn = self._make([socket_module.timeout()])
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("WHT")
        assert exc_info.value.code == COMM_TIMEOUT_CODE

    def test_closed_connection_raises(self):
        conn = self._make([b""])
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("WHT")
        assert exc_info.value.code == 0

    def test_send_without_connect_raises(self):
        conn = EthernetConnection(host="example.invalid")
        with pytest.raises(SnapMotionError):
            conn.send("WHT")


class _FakeSerial:
    """Minimal pyserial.Serial double: read(1) returns one byte at a time."""

    def __init__(self, byte_string: bytes, exhausted_returns_empty=True):
        self._buf = bytearray(byte_string)
        self.is_open = True
        self.written = []
        self._exhausted_returns_empty = exhausted_returns_empty

    def write(self, data):
        self.written.append(data)

    def read(self, n):
        if not self._buf:
            return b""
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def close(self):
        self.is_open = False


class TestRS232ConnectionFraming:
    def _make(self, byte_string, timeout=1.0):
        conn = RS232Connection(port="/dev/fake", timeout=timeout)
        conn._serial = _FakeSerial(byte_string)
        return conn

    def test_send_assembles_response_one_byte_at_a_time(self):
        conn = self._make(b"0 12.000 >")
        assert conn.send("A1ACP") == "12.000"
        assert conn._serial.written == [b"A1ACP\r"]

    def test_send_raises_on_error_envelope(self):
        conn = self._make(b"45 >")
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("A99ACP")
        assert exc_info.value.code == 45

    def test_read_timeout_raises_comm_timeout(self):
        # Empty byte string => first read(1) returns b'' immediately, as
        # pyserial does when its own per-call timeout elapses.
        conn = self._make(b"")
        with pytest.raises(SnapMotionError) as exc_info:
            conn.send("WHT")
        assert exc_info.value.code == COMM_TIMEOUT_CODE

    def test_send_without_connect_raises(self):
        conn = RS232Connection(port="/dev/fake")
        with pytest.raises(SnapMotionError):
            conn.send("WHT")

    def test_rejects_empty_port(self):
        with pytest.raises(ValueError):
            RS232Connection(port="")


class TestProbeConnection:
    def test_probe_succeeds_on_valid_boolean_responses(self):
        conn = FakeSnapConnection({"WHT": "0", "INB 1": "1"})
        assert probe_connection(conn) is True

    def test_probe_fails_on_non_boolean_value(self):
        conn = FakeSnapConnection({"WHT": "0", "INB 1": "5"})
        assert probe_connection(conn) is False

    def test_probe_fails_on_error(self):
        conn = FakeSnapConnection({"WHT": SnapMotionError(1002)})
        assert probe_connection(conn) is False
