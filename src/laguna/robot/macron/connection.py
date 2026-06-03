"""Transport layer for Snap2Motion ASCII command interface.

Supports persistent TCP/IP (Ethernet) and RS232 connections.
The firmware uses a polling loop (EthernetPoll + yield), so the socket
stays open between commands — we hold it for the session lifetime.
"""

from abc import ABC, abstractmethod
import socket
import threading
import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)

DEFAULT_TCP_PORT = 23
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 5.0

_CRLF = b"\r\n"


class SnapMotionError(Exception):
    """Raised when the controller returns an error escape code or the connection fails."""

    def __init__(self, code: int, message: str = ""):
        self.code = code
        detail = f" — {message}" if message else ""
        super().__init__(f"SnapMotion error {code}{detail}")


class SnapConnection(ABC):
    """Unified send/receive interface over Ethernet or RS232."""

    @abstractmethod
    def connect(self) -> None:
        """Open the connection. Raises OSError or SnapMotionError on failure."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the connection gracefully."""
        ...

    @abstractmethod
    def send(self, command: str) -> str:
        """Send one ASCII command and return the response value string.

        Appends CRLF, blocks until the controller echoes back a CRLF-terminated
        response, and returns the stripped value string. Raises SnapMotionError if
        the response looks like a firmware escape code.
        """
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...


class EthernetConnection(SnapConnection):
    """TCP/IP transport. Default port 23 (telnet-style, as set in firmware).

    Keeps a single persistent socket open for the session. All sends are
    serialised through a threading.Lock so concurrent callers don't interleave.
    """

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_TCP_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect((self.host, self.port))
        self._socket = sock
        logger.info("Connected to controller at %s:%d", self.host, self.port)

    def disconnect(self) -> None:
        sock = self._socket
        self._socket = None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        logger.info("Disconnected from controller")

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def send(self, command: str) -> str:
        with self._lock:
            if self._socket is None:
                raise SnapMotionError(0, "Not connected")
            try:
                self._socket.sendall((command + "\r\n").encode("ascii"))
                return _parse_response(self._recv_line())
            except socket.timeout:
                raise SnapMotionError(0, f"Timeout waiting for response to: {command!r}")
            except OSError as exc:
                raise SnapMotionError(0, f"Socket error: {exc}") from exc

    def _recv_line(self) -> str:
        buf = b""
        while True:
            chunk = self._socket.recv(512)
            if not chunk:
                raise SnapMotionError(0, "Controller closed connection")
            buf += chunk
            if _CRLF in buf:
                return buf.split(_CRLF, 1)[0].decode("ascii").strip()


class RS232Connection(SnapConnection):
    """Serial RS232 transport. Default 9600 8N1 as specified in firmware standard.inc.

    Baud rates supported by the firmware: 9600, 19200, 38400, 57600, 115200.
    Requires pyserial: pip install pyserial
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        try:
            import serial  # type: ignore[import]
        except ImportError as exc:
            raise ImportError(
                "pyserial is required for RS232 connections: pip install pyserial"
            ) from exc
        self._serial = serial.Serial(
            self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
        )
        logger.info(
            "Connected to controller via RS232 on %s at %d baud", self.port, self.baudrate
        )

    def disconnect(self) -> None:
        ser = self._serial
        self._serial = None
        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
        logger.info("RS232 connection closed")

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def send(self, command: str) -> str:
        with self._lock:
            if not self.is_connected:
                raise SnapMotionError(0, "Not connected")
            try:
                self._serial.write((command + "\r\n").encode("ascii"))
                raw = self._serial.readline()
            except Exception as exc:
                raise SnapMotionError(0, f"Serial error: {exc}") from exc
            if not raw:
                raise SnapMotionError(600, f"Timeout waiting for response to: {command!r}")
            return _parse_response(raw.decode("ascii").strip())


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

# Firmware escape codes are positive integers >= 600 (based on ec_DataNotPresent=1005,
# GetCharacter timeout=600, etc.). Normal numeric responses are floats like "1.0",
# "0.0", or integer-looking strings like "3".
_ERROR_CODE_THRESHOLD = 600


def _parse_response(raw: str) -> str:
    """Return raw if it is a valid numeric response; raise SnapMotionError otherwise.

    The firmware always echoes a double-precision value. Error conditions raise
    integer escape codes. We detect errors by checking if the response is a bare
    integer >= the error threshold.
    """
    if not raw:
        raise SnapMotionError(0, "Empty response from controller")
    try:
        value = float(raw)
        # Escape codes are positive integers; normal positions/booleans can also
        # be small integers (0.0, 1.0). We only flag values that are large whole
        # numbers — firmware escape codes are all >= 600.
        if value == int(value) and int(value) >= _ERROR_CODE_THRESHOLD:
            raise SnapMotionError(int(value))
        return raw
    except ValueError:
        raise SnapMotionError(0, f"Unparseable response: {raw!r}")


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

# Two read-only global commands that any Snap2Motion controller will answer.
# WHT — watchdog has tripped (returns 0 or 1)
# INB 1 — read digital input bit 1 (returns 0 or 1)
# Neither triggers motion or changes state, so they are safe to send blind.
_PROBE_COMMANDS = ("WHT", "INB 1")


def probe_connection(conn: SnapConnection, timeout: float = 2.0) -> bool:
    """Return True if conn is a live Snap2Motion controller.

    Sends two read-only commands and verifies both return a valid 0/1 numeric
    response. Safe to call with the connection already open.

    This is deliberately conservative: other serial devices could theoretically
    answer with 0 or 1, but the two-command sequence and strict float parsing
    make false positives very unlikely.
    """
    for cmd in _PROBE_COMMANDS:
        try:
            raw = conn.send(cmd)
            value = float(raw)
            if value not in (0.0, 1.0):
                return False
        except (SnapMotionError, OSError, ValueError):
            return False
    return True


def find_rs232_port(
    baudrates: tuple[int, ...] = (9600, 115200, 57600, 38400, 19200),
    probe_timeout: float = 1.5,
) -> Optional[str]:
    """Scan all available serial ports and return the device path of the first
    one that identifies as a Snap2Motion controller.

    Tries every enumerated port at each baudrate in order. Returns None if no
    matching port is found. Raises ImportError if pyserial is not installed.

    Because other USB serial devices (flow meters, gauges, ADCs) may be attached
    at the same time, every port is probed with read-only commands before being
    accepted. The function cleans up each candidate connection whether it matches
    or not.

    Example::

        port = find_rs232_port()
        if port is None:
            raise RuntimeError("Snap2Motion controller not found on any serial port")
        conn = RS232Connection(port, baudrate=9600)
        conn.connect()
    """
    try:
        from serial.tools.list_ports import comports  # type: ignore[import]
    except ImportError as exc:
        raise ImportError(
            "pyserial is required for port discovery: pip install pyserial"
        ) from exc

    candidates = list(comports())
    logger.info("Scanning %d serial ports for Snap2Motion controller", len(candidates))

    for port_info in candidates:
        for baud in baudrates:
            conn = RS232Connection(port_info.device, baudrate=baud, timeout=probe_timeout)
            try:
                conn.connect()
                # Small settle delay — some USB-serial adapters need a moment
                time.sleep(0.1)
                if probe_connection(conn, timeout=probe_timeout):
                    conn.disconnect()
                    logger.info(
                        "Found controller on %s at %d baud", port_info.device, baud
                    )
                    return port_info.device
            except (OSError, SnapMotionError):
                pass
            finally:
                try:
                    conn.disconnect()
                except Exception:
                    pass

    logger.warning("Snap2Motion controller not found on any available serial port")
    return None


def assert_controller_present(conn: SnapConnection) -> None:
    """Verify that conn is talking to a Snap2Motion controller.

    Raises SnapMotionError(0) if the probe fails. Call this after connect()
    to catch misidentified ports or misconfigured baud rates before issuing
    any motion commands.
    """
    if not probe_connection(conn):
        raise SnapMotionError(
            0,
            "No Snap2Motion controller detected on this connection. "
            "Check port, baudrate, and cable.",
        )
