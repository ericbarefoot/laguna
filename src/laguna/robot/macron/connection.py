r"""Transport layer for Snap2Motion ASCII command interface.

Supports persistent TCP/IP (Ethernet) and RS232 connections, plus RS232 over
a raw TCP passthrough bridge via pyserial's socket:// URL scheme (see
RS232Connection).  The firmware uses a polling loop (EthernetPoll + yield),
so the socket/serial link stays open between commands — we hold it for the
session lifetime.

Wire format (verified against the vendor's shipped ASCII-interpreter source
and independently against hardware-tested code on the lab's bridge Pi):

  - Commands are submitted terminated by CR (``\\r``); LF is ignored by the
    firmware, so CRLF works too.
  - Responses terminate at a literal ``>`` prompt character, not CRLF.
  - Success responses look like ``"0 <value> >"`` (value formatted to 3
    decimal places). Error responses look like ``"<escape_code> >"`` — note
    the absence of the leading ``"0 "``. The only reliable way to
    distinguish success from error is checking whether the first
    whitespace/comma-delimited token equals the literal string ``"0"``.
"""

from abc import ABC, abstractmethod
import re
import socket
import threading
import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)

DEFAULT_TCP_PORT = 23
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 5.0

_PROMPT = b">"

# Firmware comm-timeout / "no character arrived" escape code (see standard.inc
# and the ASCII interpreter's GetCharacter timeout handling). Used whenever a
# transport gives up waiting for a '>' terminated response.
COMM_TIMEOUT_CODE = 600


class SnapMotionError(Exception):
    """Raised when the controller returns an error escape code or the connection fails."""

    def __init__(self, code: int, message: str = ""):
        """Initialize a SnapMotionError.

        Args:
            code: Error escape code from controller.
            message: Optional additional detail message.
        """
        self.code = code
        detail = f" — {message}" if message else ""
        super().__init__(f"SnapMotion error {code}{detail}")


class SnapConnection(ABC):
    """Unified send/receive interface over Ethernet, RS232, or a socket-bridged RS232 link."""

    @abstractmethod
    def connect(self) -> None:
        """Open the connection.

        Raises:
            OSError: If connection fails.
            SnapMotionError: If connection fails.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the connection gracefully."""
        ...

    @abstractmethod
    def send(self, command: str) -> str:
        """Send one ASCII command and return the response value token as a string.

        Appends CR, blocks until the controller responds with a ``>``-terminated
        reply, and returns the success value token (e.g. ``"12.345"`` or ``"0"``
        for value-less acknowledgements). Raises SnapMotionError with the
        firmware's escape code if the response is an error envelope, or with
        code COMM_TIMEOUT_CODE (600) if no ``>`` arrives before the deadline.

        Args:
            command: ASCII command string (CR will be appended).

        Returns:
            Response value token as a string.

        Raises:
            SnapMotionError: If response is an error or timeout occurs.
        """
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """True if connection is currently open."""
        ...


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
        """Initialize TCP/IP transport configuration.

        Args:
            host: Controller hostname or IP address.
            port: TCP port (default 23).
            timeout: Socket timeout in seconds (default 5.0).

        Raises:
            ValueError: If host is empty or port is out of range.
        """
        if not host:
            raise ValueError("host must be a non-empty string")
        if not (1 <= port <= 65535):
            raise ValueError(f"port must be 1–65535, got {port!r}")
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        """Open TCP/IP connection to the controller.

        Raises:
            OSError: If connection fails.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect((self.host, self.port))
        self._socket = sock
        logger.info("Connected to controller at %s:%d", self.host, self.port)

    def disconnect(self) -> None:
        """Close TCP/IP connection gracefully."""
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
        """True if socket is currently open."""
        return self._socket is not None

    def send(self, command: str) -> str:
        """Send command over TCP/IP socket.

        Args:
            command: ASCII command string.

        Returns:
            Response value token as a string.

        Raises:
            SnapMotionError: If socket is not connected, timeout occurs, or response is an error.
        """
        with self._lock:
            if self._socket is None:
                raise SnapMotionError(0, "Not connected")
            try:
                self._socket.sendall((command + "\r").encode("ascii"))
                return _parse_response(self._read_until_prompt())
            except socket.timeout:
                raise SnapMotionError(
                    COMM_TIMEOUT_CODE, f"Timeout waiting for response to: {command!r}"
                )
            except OSError as exc:
                raise SnapMotionError(0, f"Socket error: {exc}") from exc

    def _read_until_prompt(self) -> str:
        """Read from socket until ``>`` prompt is received.

        Returns:
            Raw response string (bytes decoded as ASCII).

        Raises:
            SnapMotionError: If timeout occurs or connection closes before prompt.
        """
        buf = b""
        deadline = time.monotonic() + self.timeout
        while _PROMPT not in buf:
            if time.monotonic() > deadline:
                raise SnapMotionError(COMM_TIMEOUT_CODE, "Timed out waiting for '>' prompt")
            try:
                chunk = self._socket.recv(512)
            except socket.timeout:
                raise SnapMotionError(COMM_TIMEOUT_CODE, "Timed out waiting for '>' prompt")
            if not chunk:
                raise SnapMotionError(0, "Controller closed connection")
            buf += chunk
        return buf.decode("ascii", errors="replace")


class RS232Connection(SnapConnection):
    """Serial RS232 transport. Default 9600 8N1 as specified in firmware standard.inc.

    Baud rates supported by the firmware: 9600, 19200, 38400, 57600, 115200.
    Requires pyserial: pip install pyserial

    ``port`` may be a plain device path (e.g. ``"/dev/ttyUSB0"``) or any URL
    pyserial's ``serial_for_url()`` understands — most usefully
    ``"socket://<host>:<port>"``, which lets this class transparently talk to
    a raw TCP↔serial passthrough bridge on a remote host (e.g. a Raspberry Pi
    sitting next to the controller) with no protocol-aware code on either
    side of that bridge. Plain ``serial.Serial()`` does *not* dispatch URL
    schemes — only ``serial_for_url()`` does — so this class always goes
    through ``serial_for_url()``, which handles both cases correctly.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        """Initialize RS232 transport configuration.

        Args:
            port: Device path or pyserial URL (e.g. '/dev/ttyUSB0' or 'socket://host:port').
            baudrate: Baud rate (default 9600).
            timeout: Read timeout in seconds (default 5.0).

        Raises:
            ValueError: If port is empty.
        """
        if not port:
            raise ValueError(
                "port must be a non-empty string (e.g. '/dev/ttyUSB0' or "
                "'socket://host:port'). Use find_rs232_port() to discover a "
                "local port automatically."
            )
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        """Open RS232 connection to controller.

        Raises:
            ImportError: If pyserial is not installed.
            OSError: If connection fails.
        """
        try:
            import serial  # type: ignore[import]
        except ImportError as exc:
            raise ImportError(
                "pyserial is required for RS232 connections: pip install pyserial"
            ) from exc
        self._serial = serial.serial_for_url(
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
        """Close RS232 connection gracefully."""
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
        """True if serial port is currently open."""
        return self._serial is not None and self._serial.is_open

    def send(self, command: str) -> str:
        """Send command over serial port.

        Args:
            command: ASCII command string.

        Returns:
            Response value token as a string.

        Raises:
            SnapMotionError: If not connected, timeout occurs, or response is an error.
        """
        with self._lock:
            if not self.is_connected:
                raise SnapMotionError(0, "Not connected")
            try:
                self._serial.write((command + "\r").encode("ascii"))
                raw = self._read_until_prompt()
            except SnapMotionError:
                raise
            except Exception as exc:
                raise SnapMotionError(0, f"Serial error: {exc}") from exc
            return _parse_response(raw)

    def _read_until_prompt(self) -> str:
        """Read from serial port until ``>`` prompt is received.

        Returns:
            Raw response string (bytes decoded as ASCII).

        Raises:
            SnapMotionError: If timeout occurs or no prompt received.
        """
        buf = b""
        deadline = time.monotonic() + self.timeout
        while _PROMPT not in buf:
            if time.monotonic() > deadline:
                raise SnapMotionError(COMM_TIMEOUT_CODE, "Timed out waiting for '>' prompt")
            chunk = self._serial.read(1)
            if not chunk:
                # pyserial's own per-read timeout elapsed with nothing received.
                raise SnapMotionError(COMM_TIMEOUT_CODE, "Timed out waiting for '>' prompt")
            buf += chunk
        return buf.decode("ascii", errors="replace")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

_TOKEN_SPLIT = re.compile(r"[,\s]+")


def _parse_response(raw: str) -> str:
    """Parse a raw ``>``-terminated response into its success value token.

    Response envelope (see module docstring): success is ``"0 <value> >"``,
    error is ``"<escape_code> >"``. Any text before the last line (e.g. a
    telnet-echoed command, or a banner) is discarded — only the last
    non-empty line before the prompt is meaningful.

    Args:
        raw: Raw response string including ``>`` prompt.

    Returns:
        Success value token.

    Raises:
        SnapMotionError: If response is an error or unparseable.
    """
    payload = raw.split(">", 1)[0]
    lines = [line for line in re.split(r"[\r\n]+", payload) if line.strip()]
    if not lines:
        raise SnapMotionError(0, f"Empty response: {raw!r}")
    last = lines[-1]
    tokens = [t for t in _TOKEN_SPLIT.split(last.strip()) if t]
    if not tokens:
        raise SnapMotionError(0, f"Empty response: {raw!r}")

    if tokens[0] == "0":
        return tokens[1] if len(tokens) > 1 else "0"

    try:
        code = int(float(tokens[0]))
    except ValueError:
        raise SnapMotionError(0, f"Unparseable response: {raw!r}")
    raise SnapMotionError(code)


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

    Args:
        conn: Connection to probe.
        timeout: Per-command timeout in seconds (default 2.0).

    Returns:
        True if both probe commands succeed and return valid responses.
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
    """Scan all available serial ports and return the device path of the first one that identifies as a Snap2Motion controller.

    Tries every enumerated port at each baudrate in order. Returns None if no
    matching port is found. Raises ImportError if pyserial is not installed.

    Because other USB serial devices (flow meters, gauges, ADCs) may be attached
    at the same time, every port is probed with read-only commands before being
    accepted. The function cleans up each candidate connection whether it matches
    or not.

    Args:
        baudrates: Baud rates to try in order (default tries 9600, 115200, etc.).
        probe_timeout: Timeout for each probe command in seconds (default 1.5).

    Returns:
        Device path string if a controller is found, None otherwise.

    Raises:
        ImportError: If pyserial is not installed.
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

    Call this after connect() to catch misidentified ports or misconfigured
    baud rates before issuing any motion commands.

    Args:
        conn: Connection to verify.

    Raises:
        SnapMotionError: If probe fails or no controller is detected.
    """
    if not probe_connection(conn):
        raise SnapMotionError(
            0,
            "No Snap2Motion controller detected on this connection. "
            "Check port, baudrate, and cable.",
        )
