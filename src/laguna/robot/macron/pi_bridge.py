"""Persistent SSH-bridged transport to the Snap2Motion controller.

Deploys gantry_agent.py (see the sibling file, kept standalone/pyserial-only
so it can be SFTP'd to a Pi without installing the laguna package there) and
launches it once as a long-running process over a single SSH exec_command
channel, then exchanges newline-delimited JSON lines with it for the
lifetime of the connection. This avoids a fresh SSH connect+exec per
command — too slow for frequent position polling or streamed motion — the
one-shot-per-call pattern used by laguna.camera.network.CameraArray is fine
for occasional captures but not for this.

gantry_agent.py is the sole owner of the BLC serial port and can run a full
topographic scan on a background thread while remaining responsive to a
stop command — see its module docstring. Because a scan's completion
message arrives asynchronously (long after the request that started it),
this class runs a background reader thread that dispatches every incoming
line either to the pending request it answers, or to a dedicated scan
result queue. See "Async scan protocol" below.

Wire protocol, one JSON object per line:
  PC -> agent:   {"id": N, "cmd": "A1 ACP", "timeout": 5.0}
                 {"op": "ping"}
                 {"op": "close"}
                 {"id": N, "op": "scan_start", "axis": "A1", "end_mm": 500.0,
                  "feed_rate_mm_s": 5.0, "al1342_host": "192.168.1.251",
                  "pdin_port": 2, "output": "/tmp/profile_....csv"}
                 {"op": "scan_stop"}
  agent -> PC:   {"ready": true}                      (once, at startup)
                 {"id": N, "raw": "0 12.000 >"}
                 {"id": N, "error": "...", "code": 600}
                 {"id": N, "scan_started": true, "start_pos_mm": ..., ...}  (ack)
                 {"id": N, "error": "..."}                                  (scan rejected)
                 {"scan_stop_ack": true}

Async scan protocol: a scan's *completion* arrives independently of any
request/response pairing —
  {"scan_done": true, "id": N, "csv_path": ..., "actual_start_mm": ..., ...}
  {"scan_error": "...", "id": N}
These are routed by the reader thread to a dedicated queue, retrieved via
wait_for_scan_result(), not via the per-request dispatch used for everything
else.

Safe-mode gate: every command is checked against SAFE_COMMANDS by
check_safe_mode() BEFORE anything is written to the channel. This is
independent of any other safety layer in this codebase (fences, dry_run,
confirm_cb) and cannot be bypassed by a bug elsewhere — while safe_mode is
True, only read-only query commands can ever reach the wire from this
class. gantry_agent.py keeps an identical, independently-enforced copy of
this table on the Pi side as defense in depth. scan_start is gated the same
way agent-side (checked against "BMT", not on the allowlist) — this class
does not duplicate that check client-side since scan_start isn't a bare
ASCII command string, but the agent will reject it and the rejection
surfaces as a SnapMotionError from start_scan().
"""

from __future__ import annotations

import json
import logging
import queue
import re
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

from .connection import COMM_TIMEOUT_CODE, SnapConnection, SnapMotionError, _parse_response

logger = logging.getLogger(__name__)

AGENT_SCRIPT = Path(__file__).parent / "gantry_agent.py"
REMOTE_AGENT_PATH = "/tmp/laguna_gantry_agent.py"
DEFAULT_SSH_PORT = 22
DEFAULT_TIMEOUT = 5.0
READY_TIMEOUT = 10.0

# mnemonic -> max allowed argument count (0 = bare read only). Checked
# BEFORE any byte reaches the wire whenever safe_mode is True. See module
# docstring — gantry_agent.py enforces an identical table independently.
SAFE_COMMANDS = {
    "WHT": 0, "UHD": 0, "UTP": 1,
    "INB": 1, "ISI": 1, "ALI": 1,
    "ACP": 0, "ENP": 0, "COP": 0, "DEP": 0,
    "SPD": 0, "ACL": 0, "DCL": 0, "NLT": 0, "PLT": 0,
    "MTR": 0, "ENA": 0, "MIF": 0,
    "CAB": 0, "CAP": 0, "CAT": 0, "PFP": 0, "PFV": 0,
}

_PREFIX_RE = re.compile(r"^[AC]\d+$")


def parse_command(cmd: str) -> Tuple[str, int]:
    """Split a formatted ASCII command into (mnemonic, arg_count).

    Handles the optional leading axis/group prefix (A<n>/C<n>) — e.g.
    "A1 SPD 5000" -> ("SPD", 1); "INB 3" -> ("INB", 1); "WHT" -> ("WHT", 0).
    """
    tokens = cmd.split()
    if not tokens:
        raise ValueError("empty command")
    idx = 1 if _PREFIX_RE.match(tokens[0]) else 0
    if idx >= len(tokens):
        raise ValueError(f"command has no mnemonic: {cmd!r}")
    return tokens[idx].upper(), len(tokens) - idx - 1


def check_safe_mode(cmd: str) -> None:
    """Raise SnapMotionError if cmd is not on the query-only allowlist.

    Must be called before any byte is written to the wire while safe_mode
    is active.
    """
    mnemonic, arg_count = parse_command(cmd)
    max_args = SAFE_COMMANDS.get(mnemonic)
    if max_args is None or arg_count > max_args:
        raise SnapMotionError(
            0,
            f"Command {cmd!r} blocked by safe_mode "
            f"(mnemonic={mnemonic!r}, {arg_count} args) — no-motion restriction active",
        )


class SafeModeConnection(SnapConnection):
    """Wraps any SnapConnection with the same client-side query-only allowlist gate.

    PiGantryConnection has this gate built in; RS232Connection and
    EthernetConnection do not (they're deliberately dumb passthroughs — the
    gate belongs to the driver, not the transport). This wrapper lets any
    transport (e.g. RS232Connection pointed at the raw socket_bridge) get
    the same "impossible to send anything but a read-only query while
    safe_mode is True" guarantee, checked here before any byte reaches the
    inner connection. Intended for M4-style read-only hardware verification
    against transports that aren't PiGantryConnection.
    """

    def __init__(self, inner: SnapConnection, safe_mode: bool = True):
        self._inner = inner
        self.safe_mode = safe_mode

    def connect(self) -> None:
        self._inner.connect()

    def disconnect(self) -> None:
        self._inner.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._inner.is_connected

    def send(self, command: str) -> str:
        if self.safe_mode:
            check_safe_mode(command)  # raises before touching the inner connection
        return self._inner.send(command)


class PiGantryConnection(SnapConnection):
    """SSH-bridged persistent transport, talking to gantry_agent.py on a remote Pi.

    Requires paramiko: pip install paramiko
    """

    def __init__(
        self,
        host: str,
        ssh_user: str,
        ssh_key: Optional[str] = None,
        ssh_passphrase: Optional[str] = None,
        remote_serial_device: str = "",
        remote_baud: int = 9600,
        ssh_port: int = DEFAULT_SSH_PORT,
        timeout: float = DEFAULT_TIMEOUT,
        safe_mode: bool = True,
        reconnect_on_failure: bool = True,
        max_reconnect_attempts: int = 3,
    ):
        if not host:
            raise ValueError("host must be a non-empty string")
        if not ssh_user:
            raise ValueError("ssh_user must be a non-empty string")
        if not remote_serial_device:
            raise ValueError(
                "remote_serial_device must be a non-empty string "
                "(e.g. '/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0')"
            )
        self.host = host
        self.ssh_user = ssh_user
        self.ssh_key = ssh_key
        self.ssh_passphrase = ssh_passphrase
        self.remote_serial_device = remote_serial_device
        self.remote_baud = remote_baud
        self.ssh_port = ssh_port
        self.timeout = timeout
        self.safe_mode = safe_mode
        self.reconnect_on_failure = reconnect_on_failure
        self.max_reconnect_attempts = max_reconnect_attempts

        self._client = None
        self._channel = None
        self._lock = threading.Lock()          # serializes send()/start_scan() request/ack cycles
        self._write_lock = threading.Lock()    # guards raw channel.send() calls
        self._next_id = 1
        self._stdout_buf = b""

        self._pending: dict = {}                # request id -> queue.Queue, filled by reader thread
        self._pending_lock = threading.Lock()
        self._scan_result_queue: "queue.Queue[dict]" = queue.Queue()
        self._scan_running = False               # plain bool; atomic under the GIL, same
                                                  # pattern as MqttSubscriber._is_connected
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        try:
            import paramiko  # type: ignore[import]
        except ImportError as exc:
            raise ImportError(
                "paramiko is required for PiGantryConnection: pip install paramiko"
            ) from exc

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        kwargs: dict = {"username": self.ssh_user, "port": self.ssh_port, "timeout": self.timeout}
        if self.ssh_key:
            kwargs["key_filename"] = self.ssh_key
        if self.ssh_passphrase:
            kwargs["passphrase"] = self.ssh_passphrase
        client.connect(self.host, **kwargs)

        sftp = client.open_sftp()
        try:
            sftp.put(str(AGENT_SCRIPT), REMOTE_AGENT_PATH)
        finally:
            sftp.close()

        safe_flag = "" if self.safe_mode else " --allow-motion"
        remote_cmd = (
            f"python3 {REMOTE_AGENT_PATH} "
            f"--port {self.remote_serial_device} --baud {self.remote_baud}{safe_flag}"
        )
        _, stdout, _stderr = client.exec_command(remote_cmd)
        channel = stdout.channel

        self._client = client
        self._channel = channel
        self._stdout_buf = b""
        self._next_id = 1
        self._pending = {}
        self._scan_running = False
        # Drain anything left in the scan result queue from a prior connection
        while True:
            try:
                self._scan_result_queue.get_nowait()
            except queue.Empty:
                break

        self._await_ready()

        self._reader_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        logger.info("Connected to gantry agent on %s via SSH", self.host)

    def _await_ready(self) -> None:
        deadline = time.monotonic() + READY_TIMEOUT
        while True:
            line = self._read_line(deadline)
            if line is None:
                raise SnapMotionError(0, "Timed out waiting for gantry_agent.py to report ready")
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                logger.debug("[agent stdout, pre-ready] %s", line)
                continue
            if msg.get("ready"):
                return
            logger.debug("[agent] %s", msg)

    def disconnect(self) -> None:
        self._reader_stop.set()
        channel = self._channel
        client = self._client
        self._channel = None
        self._client = None
        if channel is not None:
            try:
                channel.send((json.dumps({"op": "close"}) + "\n").encode("ascii"))
            except Exception:
                pass
            try:
                channel.close()
            except Exception:
                pass
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        reader = self._reader_thread
        self._reader_thread = None
        if reader is not None and reader.is_alive():
            reader.join(timeout=2.0)
        # Wake up anything blocked on a pending-request queue — the
        # connection is gone, so no response will ever arrive for them.
        with self._pending_lock:
            pending = list(self._pending.values())
            self._pending.clear()
        for q in pending:
            q.put({"error": "connection closed", "code": 0})
        self._scan_running = False
        logger.info("Disconnected from gantry agent")

    @property
    def is_connected(self) -> bool:
        return (
            self._channel is not None
            and not self._channel.closed
            and not self._channel.exit_status_ready()
        )

    @property
    def is_scan_running(self) -> bool:
        return self._scan_running

    # ------------------------------------------------------------------
    # Background reader — dispatches every incoming line
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set():
            line = self._read_line(None)
            if line is None:
                break
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                logger.debug("[agent stdout] %s", line)
                continue

            if "scan_done" in msg or "scan_error" in msg:
                self._scan_running = False
                self._scan_result_queue.put(msg)
                continue
            if "pong" in msg or "scan_stop_ack" in msg or "scan_running" in msg:
                logger.debug("[agent] %s", msg)
                continue

            mid = msg.get("id")
            if mid is not None:
                with self._pending_lock:
                    q = self._pending.pop(mid, None)
                if q is not None:
                    q.put(msg)
                    continue

            logger.debug("[agent] discarding unmatched message: %s", msg)

    def _write_line(self, payload: str) -> None:
        with self._write_lock:
            self._channel.send((payload + "\n").encode("ascii"))

    # ------------------------------------------------------------------
    # Command exchange (interactive)
    # ------------------------------------------------------------------

    def send(self, command: str) -> str:
        if self.safe_mode:
            check_safe_mode(command)  # raises before anything is written

        if self._scan_running:
            raise SnapMotionError(0, "Cannot send interactive command: scan in progress")

        with self._lock:
            if not self.is_connected:
                if self.reconnect_on_failure:
                    self._reconnect()
                else:
                    raise SnapMotionError(0, "Not connected")

            request_id = self._next_id
            self._next_id += 1
            q: "queue.Queue[dict]" = queue.Queue()
            with self._pending_lock:
                self._pending[request_id] = q
            payload = json.dumps({"id": request_id, "cmd": command, "timeout": self.timeout})
            try:
                self._write_line(payload)
            except Exception as exc:
                with self._pending_lock:
                    self._pending.pop(request_id, None)
                raise SnapMotionError(0, f"Failed to send to agent: {exc}") from exc

            try:
                msg = q.get(timeout=self.timeout)
            except queue.Empty:
                with self._pending_lock:
                    self._pending.pop(request_id, None)
                raise SnapMotionError(
                    COMM_TIMEOUT_CODE, f"Timed out waiting for agent response to {command!r}"
                )

        if "error" in msg:
            raise SnapMotionError(int(msg.get("code", 0)), str(msg["error"]))
        return _parse_response(msg["raw"])

    # ------------------------------------------------------------------
    # Scan control
    # ------------------------------------------------------------------

    def start_scan(
        self, axis: str, end_mm: float, feed_rate_mm_s: float,
        al1342_host: str, pdin_port: int, output: str,
    ) -> dict:
        """Start a topographic scan on the agent. Returns the ack dict
        ({"scan_started": True, "start_pos_mm": ..., "accel_mm_s2": ...,
        "decel_mm_s2": ...}). Raises SnapMotionError if the agent rejects it
        (blocked by its own safe_mode, or a scan is already running) or on
        timeout waiting for the ack. The scan itself then runs on the
        agent's background thread — use wait_for_scan_result() to block for
        completion, and stop_scan() to cancel it early.
        """
        if self._scan_running:
            raise SnapMotionError(0, "Scan already in progress")

        with self._lock:
            if not self.is_connected:
                if self.reconnect_on_failure:
                    self._reconnect()
                else:
                    raise SnapMotionError(0, "Not connected")

            request_id = self._next_id
            self._next_id += 1
            q: "queue.Queue[dict]" = queue.Queue()
            with self._pending_lock:
                self._pending[request_id] = q
            payload = json.dumps({
                "id": request_id, "op": "scan_start",
                "axis": axis, "end_mm": end_mm, "feed_rate_mm_s": feed_rate_mm_s,
                "al1342_host": al1342_host, "pdin_port": pdin_port, "output": output,
            })
            try:
                self._write_line(payload)
            except Exception as exc:
                with self._pending_lock:
                    self._pending.pop(request_id, None)
                raise SnapMotionError(0, f"Failed to send scan_start: {exc}") from exc

            try:
                msg = q.get(timeout=self.timeout)
            except queue.Empty:
                with self._pending_lock:
                    self._pending.pop(request_id, None)
                raise SnapMotionError(COMM_TIMEOUT_CODE, "Timed out waiting for scan_start ack")

        if "error" in msg:
            raise SnapMotionError(0, str(msg["error"]))
        self._scan_running = True
        return msg

    def stop_scan(self) -> None:
        """Request the agent cancel the currently running scan (sends BST
        on the scanning axis). Fire-and-forget — call wait_for_scan_result()
        to observe the scan's actual completion afterward; a stopped scan
        still finishes normally through the same scan_done path (see
        gantry_agent.py), just with fewer samples than a full pass.
        """
        self._write_line(json.dumps({"op": "scan_stop"}))

    def wait_for_scan_result(self, timeout: float) -> dict:
        """Block for the async scan_done/scan_error message. Raises
        SnapMotionError on timeout. Returns the raw dict either way — check
        for "error" in the result to distinguish success from failure."""
        try:
            result = self._scan_result_queue.get(timeout=timeout)
        except queue.Empty:
            raise SnapMotionError(COMM_TIMEOUT_CODE, "Timed out waiting for scan result")
        self._scan_running = False
        return result

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _reconnect(self) -> None:
        for attempt in range(1, self.max_reconnect_attempts + 1):
            try:
                self.disconnect()
                self.connect()
                return
            except Exception as exc:
                logger.warning(
                    "Reconnect attempt %d/%d failed: %s", attempt, self.max_reconnect_attempts, exc
                )
                time.sleep(min(2**attempt, 10))
        raise SnapMotionError(
            0, f"Failed to reconnect to {self.host} after {self.max_reconnect_attempts} attempts"
        )

    def _read_line(self, deadline: Optional[float]) -> Optional[str]:
        """Read one newline-delimited line from the channel's stdout.

        deadline=None means block indefinitely (used by the reader thread's
        main loop, which is the only caller now that reads are centralized
        there — send()/start_scan() get their responses via a queue
        instead). Also drains stderr to the logger while waiting.
        """
        while b"\n" not in self._stdout_buf:
            if deadline is not None and time.monotonic() > deadline:
                return None
            channel = self._channel
            if channel is None:
                return None
            if channel.recv_stderr_ready():
                chunk = channel.recv_stderr(4096)
                for line in chunk.decode("utf-8", errors="replace").splitlines():
                    if line.strip():
                        logger.info("[gantry_agent] %s", line)
            if channel.recv_ready():
                self._stdout_buf += channel.recv(4096)
            elif channel.exit_status_ready():
                return None
            else:
                time.sleep(0.01)
        line, self._stdout_buf = self._stdout_buf.split(b"\n", 1)
        return line.decode("utf-8", errors="replace").strip()
