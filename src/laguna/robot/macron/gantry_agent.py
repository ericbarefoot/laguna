#!/usr/bin/env python3
"""Persistent serial bridge agent for the Snap2Motion OEM-2T controller.

Runs on the Raspberry Pi next to the controller. Deployed and launched over
SSH by PiGantryConnection (see pi_bridge.py on the PC side) — this file is
intentionally standalone, depending only on pyserial (not the laguna
package), so it can be SFTP'd to the Pi and run with a bare
`python3 gantry_agent.py --port <device> --baud <rate>` without installing
anything else there.

Protocol (newline-delimited JSON on stdin/stdout, matching pi_bridge.py):
  stdin  -> {"id": N, "cmd": "A1 ACP", "timeout": 5.0}
            {"op": "ping"}
            {"op": "close"}
  stdout <- {"ready": true}                      (once, at startup)
            {"id": N, "raw": "0 12.000 >"}
            {"id": N, "error": "...", "code": 600}
  stderr <- human-readable progress/diagnostic lines only, never JSON —
            streamed back and logged by the coordinator (same discipline as
            laguna.camera.agent)

Safe-mode gate: this agent keeps its OWN copy of the query-only allowlist
below, deliberately duplicated from pi_bridge.py's SAFE_COMMANDS rather than
imported from it, because this script must run standalone on a host that
does not have the laguna package installed. It is enforced independently of
the PC-side gate — defense in depth, so a bug in the PC-side driver can't
reach the wire even if it somehow bypasses the client-side check. Pass
--allow-motion to disable this; only ever intended for Stage 3 testing,
after the user has explicitly lifted the no-motion restriction.

Every command sent and its response (or error) is appended to an audit log
file next to this script for after-the-fact review.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print(json.dumps({"error": "pyserial not available on this host — pip install pyserial"}))
    sys.exit(1)


# Kept in sync by hand with pi_bridge.py's SAFE_COMMANDS — see module
# docstring for why this can't just be a shared import.
SAFE_COMMANDS = {
    "WHT": 0, "UHD": 0, "UTP": 1,
    "INB": 1, "ISI": 1, "ALI": 1,
    "ACP": 0, "ENP": 0, "COP": 0, "DEP": 0,
    "SPD": 0, "ACL": 0, "DCL": 0, "NLT": 0, "PLT": 0,
    "MTR": 0, "ENA": 0, "MIF": 0,
    "CAB": 0, "CAP": 0, "CAT": 0, "PFP": 0, "PFV": 0,
}

_PREFIX_RE = re.compile(r"^[AC]\d+$")

DEFAULT_LOG_PATH = Path(__file__).parent / "gantry_agent.log"


def _log(msg: str) -> None:
    """Write a timestamped progress line to stderr so the coordinator can stream it."""
    ts = time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() % 1 * 1000):03d}"
    print(f"[agent {ts}] {msg}", file=sys.stderr, flush=True)


def _audit(log_path: Path, entry: dict) -> None:
    entry = dict(entry, ts=time.time())
    try:
        with open(log_path, "a") as f:
            f.write(json.dumps(entry) + "\n")
    except OSError as exc:
        _log(f"WARNING: could not write audit log: {exc}")


def parse_command(cmd: str):
    """Split a formatted ASCII command into (mnemonic, arg_count). See pi_bridge.py."""
    tokens = cmd.split()
    if not tokens:
        raise ValueError("empty command")
    idx = 1 if _PREFIX_RE.match(tokens[0]) else 0
    if idx >= len(tokens):
        raise ValueError(f"command has no mnemonic: {cmd!r}")
    return tokens[idx].upper(), len(tokens) - idx - 1


def check_safe_mode(cmd: str) -> None:
    mnemonic, arg_count = parse_command(cmd)
    max_args = SAFE_COMMANDS.get(mnemonic)
    if max_args is None or arg_count > max_args:
        raise PermissionError(
            f"command {cmd!r} blocked by agent-side safe_mode "
            f"(mnemonic={mnemonic!r}, {arg_count} args)"
        )


class SerialBridge:
    """Owns the serial port for the lifetime of the agent process."""

    def __init__(self, port: str, baud: int, timeout: float = 5.0):
        self._ser = serial.Serial(
            port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=timeout,
        )
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def send(self, cmd: str, timeout: float) -> str:
        """Write cmd (CR-terminated) and read raw bytes up to and including '>'."""
        self._ser.timeout = timeout
        self._ser.reset_input_buffer()
        self._ser.write((cmd + "\r").encode("ascii"))
        buf = b""
        deadline = time.monotonic() + timeout
        while b">" not in buf:
            if time.monotonic() > deadline:
                raise TimeoutError(f"no '>' terminator within {timeout:.1f}s")
            chunk = self._ser.read(1)
            if not chunk:
                raise TimeoutError(f"no '>' terminator within {timeout:.1f}s")
            buf += chunk
        return buf.decode("ascii", errors="replace")


def main() -> None:
    parser = argparse.ArgumentParser(description="Persistent Snap2Motion serial bridge agent")
    parser.add_argument("--port", required=True, help="Serial device path")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument(
        "--allow-motion", action="store_true",
        help="Disable the agent-side safe-mode gate. Stage 3 only — after the "
             "no-motion restriction has been explicitly lifted.",
    )
    parser.add_argument("--log", default=str(DEFAULT_LOG_PATH))
    args = parser.parse_args()

    log_path = Path(args.log)
    safe_mode = not args.allow_motion
    if not safe_mode:
        _log("WARNING: started with --allow-motion — agent-side safe-mode gate is DISABLED")

    _log(f"Opening serial port {args.port} at {args.baud} baud...")
    try:
        bridge = SerialBridge(args.port, args.baud)
    except Exception as exc:
        print(json.dumps({"error": f"failed to open serial port: {exc}"}), flush=True)
        sys.exit(1)
    _log("Serial port open.")

    print(json.dumps({"ready": True}), flush=True)

    for raw_line in sys.stdin:
        raw_line = raw_line.strip()
        if not raw_line:
            continue
        try:
            msg = json.loads(raw_line)
        except json.JSONDecodeError as exc:
            _log(f"Ignoring unparseable stdin line: {raw_line!r} ({exc})")
            continue

        op = msg.get("op")
        if op == "close":
            _log("Received close request — shutting down.")
            break
        if op == "ping":
            print(json.dumps({"pong": True}), flush=True)
            continue

        request_id = msg.get("id")
        cmd = msg.get("cmd")
        timeout = float(msg.get("timeout", 5.0))
        if request_id is None or cmd is None:
            _log(f"Ignoring malformed request: {raw_line!r}")
            continue

        if safe_mode:
            try:
                check_safe_mode(cmd)
            except (PermissionError, ValueError) as exc:
                _log(f"BLOCKED (safe_mode): {cmd!r} — {exc}")
                _audit(log_path, {"id": request_id, "cmd": cmd, "blocked": True, "reason": str(exc)})
                print(json.dumps({"id": request_id, "error": str(exc), "code": 0}), flush=True)
                continue

        try:
            raw = bridge.send(cmd, timeout)
        except TimeoutError as exc:
            _log(f"TIMEOUT: {cmd!r} — {exc}")
            _audit(log_path, {"id": request_id, "cmd": cmd, "error": "timeout"})
            print(json.dumps({"id": request_id, "error": str(exc), "code": 600}), flush=True)
            continue
        except Exception as exc:
            _log(f"SERIAL ERROR: {cmd!r} — {exc}")
            _audit(log_path, {"id": request_id, "cmd": cmd, "error": str(exc)})
            print(json.dumps({"id": request_id, "error": str(exc), "code": 0}), flush=True)
            continue

        _audit(log_path, {"id": request_id, "cmd": cmd, "raw": raw})
        print(json.dumps({"id": request_id, "raw": raw}), flush=True)

    bridge.close()
    _log("Serial port closed. Exiting.")


if __name__ == "__main__":
    main()
