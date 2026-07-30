#!/usr/bin/env python3
"""Persistent serial bridge agent for the Snap2Motion OEM-2T controller.

Runs on the Raspberry Pi next to the controller. Deployed and launched over
SSH by PiGantryConnection (see pi_bridge.py on the PC side) — this file is
intentionally standalone, depending only on pyserial (not the laguna
package), so it can be SFTP'd to the Pi and run with a bare
`python3 gantry_agent.py --port <device> --baud <rate>` without installing
anything else there.

This agent is the SOLE owner of the BLC serial port for its whole lifetime.
Both interactive axis commands AND full topographic scans (with live OD2000
distance data) go through this one process — there is no longer a separate
scan_runner.py that takes turns owning the serial port. All serial access
goes through SerialBridge.send(), which holds a lock for the full write+read
round trip, so the main stdin-reading thread and the background scan-worker
thread (below) can never interleave bytes on the wire even though they run
concurrently. This directly guards against a documented failure mode:
pipelined/interleaved commands previously hung the BLC controller, requiring
a physical power-cycle to recover.

Protocol (newline-delimited JSON on stdin/stdout, matching pi_bridge.py):
  stdin  -> {"id": N, "cmd": "A1 ACP", "timeout": 5.0}
            {"op": "ping"}
            {"op": "close"}
            {"id": N, "op": "scan_start", "axis": "A1", "end_mm": 500.0,
             "feed_rate_mm_s": 5.0, "al1342_host": "192.168.1.251",
             "pdin_port": 2, "output": "/tmp/profile_....csv",
             "sensor": "od2000"}                    ("sensor" optional, defaults to
                                                       "od2000"; the other supported
                                                       value is "wtt12l_powerprox" —
                                                       see _decode_dp4200_wtt12l_pdin)
            {"op": "scan_stop"}
            {"op": "scan_status"}
  stdout <- {"ready": true}                      (once, at startup)
            {"id": N, "raw": "0 12.000 >"}
            {"id": N, "error": "...", "code": 600}
            {"id": N, "scan_started": true, "start_pos_mm": ..., "accel_mm_s2": ...,
             "decel_mm_s2": ...}                  (immediate ack — the scan itself
                                                     then runs on a background thread)
            {"id": N, "error": "..."}              (scan_start rejected: safe_mode,
                                                     already running, bad axis state)
            {"scan_stop_ack": true}
            {"scan_running": true|false}           (reply to scan_status)
            {"scan_done": true, "id": N, "csv_path": ..., "meta_path": ...,
             "actual_start_mm": ..., "actual_end_mm": ..., "samples": N,
             "achieved_rate_hz": ...}               (ASYNC — pushed whenever the
                                                      background scan finishes, not
                                                      in response to any one request)
            {"scan_error": "...", "id": N}          (ASYNC, same as above)
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
after the user has explicitly lifted the no-motion restriction. scan_start
is gated the same way (checked against "BMT", which is not on the allowlist
at all) — a scan cannot move the gantry unless the agent was launched with
--allow-motion, exactly like any other motion command.

Every command sent and its response (or error) is appended to an audit log
file next to this script for after-the-fact review.

OD2000/AL1342 data collection (ported from the retired scan_runner.py,
2026-07-28): the AL1342 has no push mechanism faster than 2 Hz for
continuous process data (only timer[n]-based subscribe, with a documented
and measured 500ms floor) — so scans poll pdin/getdata directly over a
persistent HTTP connection instead, which measured ~380 Hz with zero errors
on hardware. The OD2000 laser is switched on/off around each scan via IODD
index 97/0 ("Sender configuration"): "00" = on, "01" = off — note these are
inverted from what "0/1" might suggest.
"""

import argparse
import csv
import datetime
import http.client
import json
import queue
import re
import sys
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print(json.dumps({"error": "pyserial not available on this host — pip install pyserial"}))
    sys.exit(1)


# Real mm per raw controller (ACP) unit on the linear (X/Y/Z) axes —
# confirmed on hardware 2026-07-28, see docs/GANTRY_UNIT_CALIBRATION.md.
# This is the ONE place scan math converts between the two: incoming
# end_mm/feed_rate_mm_s (real mm, from the scan_start request) are divided
# by this to get the raw ACP values the BMT/SPD hardware commands need;
# start_pos_mm/accel_mm_s2/decel_mm_s2/actual_end_mm (read from the
# controller as raw ACP/ACL/DCL) are multiplied by this before being used
# in dead-reckoning math or reported back — so scan_done/scan_started
# results and the CSV's pos_mm column are real mm end-to-end. If the
# Snap2Motion/DSM project's axis scale is fixed at the source, flip this to
# 1.0 — nothing else in this file needs to change. Kept as a plain
# module-level literal (not imported from laguna.config) for the same
# standalone-deployment reason SAFE_COMMANDS is duplicated rather than
# imported — see module docstring.
MM_PER_ACP_UNIT = 15.0

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
_TOKEN_SPLIT = re.compile(r"[,\s]+")

DEFAULT_LOG_PATH = Path(__file__).parent / "gantry_agent.log"

_stdout_lock = threading.Lock()


def _log(msg: str) -> None:
    """Write a timestamped progress line to stderr so the coordinator can stream it."""
    ts = time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() % 1 * 1000):03d}"
    print(f"[agent {ts}] {msg}", file=sys.stderr, flush=True)


def _emit(obj: dict) -> None:
    """Write one JSON line to stdout, guarded so the main thread and the scan
    worker thread never interleave partial writes."""
    with _stdout_lock:
        print(json.dumps(obj), flush=True)


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


def _parse_blc_response(raw: str) -> str:
    """Parse a raw '>'-terminated BLC response into its success value token.

    Duplicated from laguna.robot.macron.connection._parse_response — this
    agent must remain standalone (no laguna install on the Pi), same reason
    SAFE_COMMANDS is duplicated rather than imported. Keep in sync by hand.

    Response envelope: success is "0 <value> >", error is "<escape_code> >".
    Used only by the scan worker thread, which (unlike the interactive
    command path) needs actual parsed values on the Pi side to do dead-
    reckoning math and check MIF — interactive commands still forward their
    raw text to the PC, which parses it itself via the same logic.
    """
    payload = raw.split(">", 1)[0]
    lines = [line for line in re.split(r"[\r\n]+", payload) if line.strip()]
    if not lines:
        raise ValueError(f"Empty response: {raw!r}")
    last = lines[-1]
    tokens = [t for t in _TOKEN_SPLIT.split(last.strip()) if t]
    if not tokens:
        raise ValueError(f"Empty response: {raw!r}")
    if tokens[0] == "0":
        return tokens[1] if len(tokens) > 1 else "0"
    raise ValueError(f"BLC error response: {raw!r}")


# ---------------------------------------------------------------------------
# OD2000 / WTT12L PowerProx / AL1342 helpers — ported verbatim from the
# retired scan_runner.py (OD2000 decode), plus the WTT12L PowerProx
# analog-via-DP4200 decode added once the WTT12L's own native IO-Link
# process data was found not to validate on this AL1342 — see
# docs/WTT12L_POWERPROX_SETUP.md and laguna.rangefinder's
# decode_dp4200_wtt12l_analog_pdin(), which this mirrors (duplicated here,
# not imported, per this file's standalone-deployment constraint above).
# ---------------------------------------------------------------------------


def _decode_pdin(hex_str: str, pdin_port: int) -> dict:
    """Decode OD2000 7002T15 6-byte PDIN hex string.

    Confirmed on hardware 2026-07-28: big-endian int32 nm (bytes 0-3) decoded
    to 808.28 mm against a physically measured 808.4 mm +/- 0.1 mm reference.
    Byte 4 ("scale") was observed as 247, not 0 as originally assumed —
    unexplained, but unused in this decode (no scale multiplication applied).
    """
    raw = bytes.fromhex(hex_str)
    distance_nm = int.from_bytes(raw[0:4], "big", signed=True)
    return {
        "distance_nm": distance_nm,
        "distance_mm": distance_nm / 1_000_000,
        "scale": raw[4],
        "q1": bool(raw[5] & 0x01),
        "q2": bool(raw[5] & 0x02),
    }


def _decode_dp4200_wtt12l_pdin(hex_str: str, pdin_port: int) -> dict:
    """Decode a WTT12L-A2523 PowerProx reading taken via its analog output,
    digitized by an ifm DP4200 IO-Link analog-input bridge plugged into
    pdin_port in place of the WTT12L's own IO-Link connection (the WTT12L's
    native process data never validated on this AL1342 — see
    docs/WTT12L_POWERPROX_SETUP.md).

    4 bytes (8 hex chars), big-endian, two 16-bit channel fields — channel 1
    (bytes 0-1) confirmed on hardware to be current in uA from the WTT12L's
    Qa analog output; channel 2 (bytes 2-3) confirmed constant regardless of
    target distance (unconnected DP4200 input) and not decoded here.

    Distance conversion assumes the sensor's un-taught default 4-20mA span
    (100mm..1400mm) — unconfirmed against the sensor's actual teach
    parameters, expect more slop than the OD2000's decode. pdin_port is
    accepted only for call-signature symmetry with _decode_pdin (used
    identically as a dict-dispatch target in _poll_pdin_loop/_run_scan) —
    the WTT12L has no per-port-dependent decode step, unlike a real
    multi-port-aware decoder might.
    """
    raw = bytes.fromhex(hex_str)
    channel1_raw = int.from_bytes(raw[0:2], "big", signed=False)
    current_ma = channel1_raw / 1000.0
    near_mm, far_mm = 100.0, 1400.0
    distance_mm = near_mm + (current_ma - 4.0) / 16.0 * (far_mm - near_mm)
    return {
        "current_ma": current_ma,
        "distance_mm": distance_mm,
    }


# sensor name -> decode function, used by scan_start dispatch and
# _poll_pdin_loop's decode_fn parameter. "od2000" stays the default
# everywhere for backward compatibility with clients that don't send a
# "sensor" field at all.
SENSOR_DECODERS = {
    "od2000": _decode_pdin,
    "wtt12l_powerprox": _decode_dp4200_wtt12l_pdin,
}


def _extract_pdin_hex_from_getdata(resp: dict) -> str:
    """Extract the pdin hex string from an AL1342 getdata HTTP response.

    Response shape: {"cid": -1, "data": {"value": "<hex>"}, "code": 200}
    """
    return resp["data"]["value"]


def _set_laser(al1342_host: str, pdin_port: int, on: bool) -> bool:
    """Turn the OD2000 laser on or off via IO-Link acyclic write.

    OD2000 IODD parameter "Sender configuration": index 97 (0x61), subindex 0,
    UInt8. 0 = Sender active (laser on), 1 = Sender not active (laser off).
    Confirmed on hardware 2026-07-28 via the AL1342's iolwriteacyclic service.
    Returns True if the write succeeded (code 200), False otherwise — a
    failure here should not abort the scan, just get logged.
    """
    conn = http.client.HTTPConnection(al1342_host, 80, timeout=5)
    payload = json.dumps({
        "code": "request", "cid": -1,
        "adr": f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/iolwriteacyclic",
        "data": {"index": 97, "subindex": 0, "value": "00" if on else "01"},
    })
    headers = {"Content-Type": "application/json"}
    try:
        conn.request("POST", "/", body=payload, headers=headers)
        resp = conn.getresponse()
        data = json.loads(resp.read())
        ok = data.get("code") == 200
        if not ok:
            _log(f"Laser {'on' if on else 'off'} write returned code {data.get('code')}")
        return ok
    except Exception as exc:
        _log(f"Failed to turn laser {'on' if on else 'off'}: {exc}")
        return False
    finally:
        try:
            conn.close()
        except Exception:
            pass


def _poll_pdin_loop(al1342_host: str, pdin_path: str, out_queue: "queue.Queue[dict]",
                     stop_event: threading.Event, pdin_port: int,
                     decode_fn=_decode_pdin) -> None:
    """Continuously poll pdin/getdata over a persistent HTTP connection.

    Confirmed on hardware: a fresh TCP connection per request (e.g. via
    urllib) chokes the AL1342's embedded HTTP server within a couple seconds
    of tight-loop polling. Reusing one http.client.HTTPConnection avoids
    this entirely and sustained ~380 Hz with zero errors over 5s.

    decode_fn: which sensor's pdin decode to apply — _decode_pdin (OD2000,
    default, matches every caller before the WTT12L PowerProx was added) or
    _decode_dp4200_wtt12l_pdin. Both take (hex_str, pdin_port) and return a
    dict; only the dict's key set differs downstream (see SENSOR_DECODERS).
    """
    payload = json.dumps({"code": "request", "cid": -1, "adr": pdin_path})
    headers = {"Content-Type": "application/json"}
    conn = http.client.HTTPConnection(al1342_host, 80, timeout=2)
    error_count = 0
    while not stop_event.is_set():
        try:
            conn.request("POST", "/", body=payload, headers=headers)
            resp = conn.getresponse()
            data = json.loads(resp.read())
            hex_str = _extract_pdin_hex_from_getdata(data)
            if hex_str:
                wall_time = time.time()
                decoded = decode_fn(hex_str, pdin_port)
                decoded["wall_time"] = wall_time
                out_queue.put(decoded)
        except Exception as exc:
            error_count += 1
            _log(f"PDIN poll error (#{error_count}): {exc}")
            try:
                conn.close()
            except Exception:
                pass
            conn = http.client.HTTPConnection(al1342_host, 80, timeout=2)
    try:
        conn.close()
    except Exception:
        pass
    if error_count:
        _log(f"PDIN poll loop finished with {error_count} transient errors")


# ---------------------------------------------------------------------------
# Serial port ownership
# ---------------------------------------------------------------------------


class SerialBridge:
    """Owns the serial port for the lifetime of the agent process.

    send() holds self._lock for the entire write+read-until('>') round trip.
    This is what makes it safe for the main stdin-reader thread and a scan
    worker thread to share one SerialBridge without ever interleaving bytes
    on the wire — see module docstring.
    """

    def __init__(self, port: str, baud: int, timeout: float = 5.0):
        self._ser = serial.Serial(
            port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=timeout,
        )
        self._ser.reset_input_buffer()
        self._lock = threading.Lock()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def send(self, cmd: str, timeout: float) -> str:
        """Write cmd (CR-terminated) and read raw bytes up to and including '>'."""
        with self._lock:
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


# ---------------------------------------------------------------------------
# Scan lifecycle — one scan at a time, tracked here
# ---------------------------------------------------------------------------


class ScanState:
    """Tracks the currently running scan thread, if any."""

    def __init__(self):
        self._lock = threading.Lock()
        self._thread: "threading.Thread | None" = None
        self._stop_event: "threading.Event | None" = None

    def is_running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def start(self, target, args, kwargs=None) -> bool:
        """Start a new scan thread. Returns False if one is already running.

        stop_event is always appended as the last positional arg (existing
        behavior, unchanged) — kwargs is for anything that needs to be
        keyword-only in target's signature instead (e.g. _run_scan's
        `sensor`, which sits after stop_event and therefore can't be
        positional without giving stop_event a default too).
        """
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._stop_event = threading.Event()
            self._thread = threading.Thread(
                target=target, args=args + (self._stop_event,), kwargs=kwargs or {}, daemon=True
            )
            self._thread.start()
            return True

    def request_stop(self) -> None:
        with self._lock:
            if self._stop_event is not None:
                self._stop_event.set()


def _run_scan(bridge: SerialBridge, request_id, axis: str, end_mm: float, feed_rate_mm_s: float,
              al1342_host: str, pdin_port: int, output: str,
              start_pos_mm: float, accel_mm_s2: float, decel_mm_s2: float,
              log_path: Path, stop_event: threading.Event, *, sensor: str = "od2000") -> None:
    """Background scan worker: runs the move, polls the rangefinder over HTTP, fuses, writes CSV.

    All bridge.send() calls here go through the same lock as interactive
    commands (see SerialBridge), so this thread and the main stdin loop
    never interleave bytes on the wire even though they run concurrently.

    sensor: "od2000" (default) or "wtt12l_powerprox" — selects both the
    pdin decode (SENSOR_DECODERS) and whether laser on/off is attempted.
    "wtt12l_powerprox" skips _set_laser() entirely: what's actually on
    pdin_port in that configuration is a DP4200 analog-input bridge, not
    the WTT12L itself, so an ISDU write there would target the DP4200's
    own (irrelevant) parameter space rather than the sensor's laser — see
    docs/WTT12L_POWERPROX_SETUP.md, "Consequence: no programmatic laser
    control on this path".
    """
    ax = axis
    laser_was_turned_on = False
    t_move_start = t_move_done = None
    records = []

    try:
        # KeyError on an unknown sensor is caught by the except below and
        # reported as a scan_error, same as any other setup failure here.
        decode_fn = SENSOR_DECODERS[sensor]
        control_laser = sensor == "od2000"

        if control_laser:
            _set_laser(al1342_host, pdin_port, on=True)
            laser_was_turned_on = True
        else:
            _log(f"sensor={sensor!r} — skipping laser control (not available via this path)")

        pdin_path = f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/pdin/getdata"
        pdin_samples: "queue.Queue[dict]" = queue.Queue()
        poll_stop = threading.Event()
        poll_thread = threading.Thread(
            target=_poll_pdin_loop,
            args=(al1342_host, pdin_path, pdin_samples, poll_stop, pdin_port, decode_fn),
            daemon=True,
        )

        feed_rate_raw = feed_rate_mm_s / MM_PER_ACP_UNIT
        end_raw = end_mm / MM_PER_ACP_UNIT
        bridge.send(f"{ax} SPD {feed_rate_raw}", timeout=5.0)
        poll_thread.start()
        bridge.send(f"{ax} BMT {end_raw}", timeout=5.0)
        t_move_start = time.time()
        _log(f"Scan move started: {ax} -> {end_mm} mm at {feed_rate_mm_s} mm/s "
             f"(raw: {end_raw:.3f} @ {feed_rate_raw:.3f})")

        while True:
            if stop_event.is_set():
                _log("Scan stop requested — sending BST")
                bridge.send(f"{ax} BST", timeout=5.0)
                break
            mif_raw = bridge.send(f"{ax} MIF", timeout=2.0)
            mif_value = _parse_blc_response(mif_raw)
            # Compare as float, not string — the BLC returns "1.000"/"0.000"
            # for MIF, not bare "1"/"0". A string comparison silently never
            # matches, so this loop would spin forever even after the move
            # physically finishes. Confirmed on hardware 2026-07-28 (same
            # bug independently hit and fixed in ad-hoc test scripts earlier
            # the same day, but not backported here until a real scan hung).
            if float(mif_value) == 1.0:
                break
            time.sleep(0.1)

        t_move_done = time.time()
        poll_stop.set()
        poll_thread.join(timeout=2.0)

        while True:
            try:
                records.append(pdin_samples.get_nowait())
            except queue.Empty:
                break

        actual_end_raw = bridge.send(f"{ax} ACP", timeout=5.0)
        actual_end_mm = float(_parse_blc_response(actual_end_raw)) * MM_PER_ACP_UNIT

    except Exception as exc:
        _log(f"SCAN ERROR: {exc}")
        _audit(log_path, {"id": request_id, "scan_error": str(exc)})
        _emit({"scan_error": str(exc), "id": request_id})
        return
    finally:
        if laser_was_turned_on:
            _set_laser(al1342_host, pdin_port, on=False)

    # ------------------------------------------------------------ fuse
    ramp_t_accel = feed_rate_mm_s / accel_mm_s2 if accel_mm_s2 > 0 else 0.0
    ramp_t_decel = feed_rate_mm_s / decel_mm_s2 if decel_mm_s2 > 0 else 0.0
    t_slew_start = t_move_start + ramp_t_accel
    t_slew_end = t_move_done - ramp_t_decel

    csv_rows = []
    for rec in records:
        t = rec["wall_time"]
        in_ramp = not (t_slew_start <= t <= t_slew_end)
        pos_mm = start_pos_mm + feed_rate_mm_s * (t - t_slew_start)
        wall_iso = datetime.datetime.utcfromtimestamp(t).isoformat() + "Z"
        csv_rows.append({
            "wall_time_unix": t,
            "wall_time_iso": wall_iso,
            "pos_mm": pos_mm,
            "distance_nm": rec.get("distance_nm"),
            "distance_mm": rec["distance_mm"],
            "q1": (int(rec["q1"]) if "q1" in rec else None),
            "q2": (int(rec["q2"]) if "q2" in rec else None),
            "in_ramp": int(in_ramp),
            "current_ma": rec.get("current_ma"),
        })
    csv_rows.sort(key=lambda r: r["wall_time_unix"])

    # ------------------------------------------------------------ write
    # distance_nm/q1/q2 are OD2000-only, current_ma is wtt12l_powerprox-only
    # (see SENSOR_DECODERS) — whichever the current sensor doesn't produce
    # is written as an empty CSV field rather than a missing column, so a
    # single fixed schema works for both.
    fieldnames = ["wall_time_unix", "wall_time_iso", "pos_mm",
                  "distance_nm", "distance_mm", "q1", "q2", "in_ramp", "current_ma"]
    try:
        with open(output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(csv_rows)
    except Exception as exc:
        _log(f"SCAN ERROR: failed to write CSV: {exc}")
        _emit({"scan_error": f"failed to write CSV: {exc}", "id": request_id})
        return

    sidecar_path = output.replace(".csv", "_meta.json")
    duration_s = t_move_done - t_move_start
    achieved_rate = len(records) / duration_s if duration_s > 0 else 0.0
    metadata = {
        "axis": ax,
        "end_mm": end_mm,
        "feed_rate_mm_s": feed_rate_mm_s,
        "actual_start_mm": start_pos_mm,
        "actual_end_mm": actual_end_mm,
        "accel_mm_s2": accel_mm_s2,
        "decel_mm_s2": decel_mm_s2,
        "ramp_t_accel_s": ramp_t_accel,
        "ramp_t_decel_s": ramp_t_decel,
        "t_move_start": t_move_start,
        "t_move_done": t_move_done,
        "duration_s": duration_s,
        "samples": len(records),
        "achieved_rate_hz": achieved_rate,
        "al1342_host": al1342_host,
        "pdin_port": pdin_port,
        "sensor": sensor,
    }
    try:
        with open(sidecar_path, "w") as f:
            json.dump(metadata, f, indent=2)
    except Exception as exc:
        _log(f"Failed to write metadata sidecar: {exc}")

    _audit(log_path, {"id": request_id, "scan_done": True, "csv_path": output, "samples": len(records)})
    _emit({
        "scan_done": True,
        "id": request_id,
        "csv_path": output,
        "meta_path": sidecar_path,
        "actual_start_mm": start_pos_mm,
        "actual_end_mm": actual_end_mm,
        "samples": len(records),
        "achieved_rate_hz": achieved_rate,
    })


def main() -> None:
    parser = argparse.ArgumentParser(description="Persistent Snap2Motion serial bridge agent")
    parser.add_argument("--port", required=True, help="Serial device path")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument(
        "--allow-motion", action="store_true",
        help="Disable the agent-side safe-mode gate. Stage 3 only — after the "
             "no-motion restriction has been explicitly lifted. Also required "
             "for scan_start, which moves the gantry.",
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
        _emit({"error": f"failed to open serial port: {exc}"})
        sys.exit(1)
    _log("Serial port open.")

    scan_state = ScanState()

    _emit({"ready": True})

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
            _emit({"pong": True})
            continue

        if op == "scan_status":
            _emit({"scan_running": scan_state.is_running()})
            continue

        if op == "scan_stop":
            scan_state.request_stop()
            _emit({"scan_stop_ack": True})
            continue

        if op == "scan_start":
            request_id = msg.get("id")
            axis = msg.get("axis")
            end_mm = msg.get("end_mm")
            feed_rate_mm_s = msg.get("feed_rate_mm_s")
            al1342_host = msg.get("al1342_host")
            pdin_port = msg.get("pdin_port")
            output = msg.get("output")
            sensor = msg.get("sensor", "od2000")
            if None in (request_id, axis, end_mm, feed_rate_mm_s, al1342_host, pdin_port, output):
                _log(f"Ignoring malformed scan_start: {raw_line!r}")
                _emit({"id": request_id, "error": "scan_start missing required field(s)"})
                continue

            if sensor not in SENSOR_DECODERS:
                _log(f"Rejecting scan_start: unknown sensor {sensor!r}")
                _emit({"id": request_id,
                       "error": f"unknown sensor {sensor!r} — must be one of {sorted(SENSOR_DECODERS)}"})
                continue

            if safe_mode:
                try:
                    check_safe_mode(f"{axis} BMT {end_mm}")
                except (PermissionError, ValueError) as exc:
                    _log(f"BLOCKED (safe_mode): scan_start — {exc}")
                    _audit(log_path, {"id": request_id, "op": "scan_start", "blocked": True, "reason": str(exc)})
                    _emit({"id": request_id, "error": str(exc)})
                    continue

            if scan_state.is_running():
                _emit({"id": request_id, "error": "scan already in progress"})
                continue

            try:
                start_raw = bridge.send(f"{axis} ACP", timeout=5.0)
                start_pos_mm = float(_parse_blc_response(start_raw)) * MM_PER_ACP_UNIT
                acl_raw = bridge.send(f"{axis} ACL", timeout=5.0)
                accel_mm_s2 = float(_parse_blc_response(acl_raw)) * MM_PER_ACP_UNIT
                dcl_raw = bridge.send(f"{axis} DCL", timeout=5.0)
                decel_mm_s2 = float(_parse_blc_response(dcl_raw)) * MM_PER_ACP_UNIT
            except Exception as exc:
                _log(f"scan_start: failed to query axis state: {exc}")
                _emit({"id": request_id, "error": f"failed to query axis state: {exc}"})
                continue

            started = scan_state.start(
                _run_scan,
                (bridge, request_id, axis, end_mm, feed_rate_mm_s, al1342_host, pdin_port, output,
                 start_pos_mm, accel_mm_s2, decel_mm_s2, log_path),
                kwargs={"sensor": sensor},
            )
            if not started:
                _emit({"id": request_id, "error": "scan already in progress"})
                continue

            _audit(log_path, {"id": request_id, "op": "scan_start", "axis": axis, "end_mm": end_mm, "sensor": sensor})
            _emit({
                "id": request_id,
                "scan_started": True,
                "start_pos_mm": start_pos_mm,
                "accel_mm_s2": accel_mm_s2,
                "decel_mm_s2": decel_mm_s2,
            })
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
                _emit({"id": request_id, "error": str(exc), "code": 0})
                continue

        try:
            raw = bridge.send(cmd, timeout)
        except TimeoutError as exc:
            _log(f"TIMEOUT: {cmd!r} — {exc}")
            _audit(log_path, {"id": request_id, "cmd": cmd, "error": "timeout"})
            _emit({"id": request_id, "error": str(exc), "code": 600})
            continue
        except Exception as exc:
            _log(f"SERIAL ERROR: {cmd!r} — {exc}")
            _audit(log_path, {"id": request_id, "cmd": cmd, "error": str(exc)})
            _emit({"id": request_id, "error": str(exc), "code": 0})
            continue

        _audit(log_path, {"id": request_id, "cmd": cmd, "raw": raw})
        _emit({"id": request_id, "raw": raw})

    scan_state.request_stop()
    bridge.close()
    _log("Serial port closed. Exiting.")


if __name__ == "__main__":
    main()
