#!/usr/bin/env python3
"""Raw-pyserial polling-speed benchmark for the Modusystems OEM-2T gantry controller.

Purpose
-------
Measure the real-world achievable polling rate/latency/reliability for
read-only actual-position (ACP) queries against the live OEM-2T rev D
controller, over the existing lab link:

    this machine --SSH--> oak@red.dyn.ucr.edu (Pi)
                                |
                          serial_bridge.py (raw TCP<->serial passthrough, port 9700)
                                |
                          RS232 (9600 8N1) --> OEM-2T controller --> gantry motors

This is a deliberately minimal, standalone script -- NOT laguna's
SafeModeConnection/MMCCommands stack -- because those add abstraction
overhead (locking, dataclass construction, exception wrapping) that would
distort a timing measurement. It talks straight to `serial.serial_for_url()`
(pyserial), exactly the mechanism laguna's own RS232Connection uses under
the hood for its "socket://host:port" transport (see
src/laguna/robot/macron/connection.py), so the numbers here reflect the same
wire path laguna would actually use in production, just without the extra
Python-side layers.

Safety constraint (must hold for the entire script, not just conceptually)
---------------------------------------------------------------------------
This script may ONLY ever send four exact read-only status queries:
"A1 ACP", "A2 ACP", "A5 ACP", "A6 ACP" (actual position of X, Y, Z, Theta --
axis mapping X=1/Y=2/Z=5/Theta=6 per docs/MACRON_GANTRY.md). No motion, jog,
move, or configuration command is ever constructed or sent. ALLOWED_COMMANDS
below is checked with an assertion immediately before every write() call, as
a defense-in-depth measure mirroring the layered safety model described in
MACRON_GANTRY.md/GANTRY_GUIDE.md, even though the command set here is fixed
by the loop structure and never derived from external input.

The axes are not expected to move (and it's fine/expected if they don't --
we are only checking how fast/reliably we can read position feedback, not
commanding any motion). Baud rate is left at the existing configured value
(9600 8N1) -- we are measuring the real-world ceiling at the current
configuration, not reconfiguring the hardware.

Wire protocol (verified byte-for-byte against
src/laguna/robot/macron/connection.py's docstrings and _parse_response, and
confirmed live against hardware before this benchmark was written):
  - Commands are submitted CR-terminated ("A1 ACP\r"). LF is ignored by the
    firmware; we send bare CR only, matching RS232Connection.send().
  - Responses terminate at a literal ">" prompt character, NOT CRLF.
  - Success envelope: "0 <value> >" (e.g. "0 0.005 >"). Error envelope:
    "<escape_code> >" -- note the absent leading "0". The only reliable way
    to distinguish success from error is checking whether the first
    whitespace/comma-delimited token equals the literal string "0" -- never
    by numeric magnitude (a legitimate position of 700.0 must not be
    misread as error code 700).

Read timeout choice
--------------------
Before writing this script, 160 warm-up queries (40 full 4-axis cycles, run
back-to-back with no pacing delay) were measured against the live hardware
to characterize baseline latency:

    min ~29.6 ms   median ~78.9 ms   p95 ~81.1 ms   max ~81.8 ms

(Responder-node axes A5/Z and A6/Theta consistently ran slower than
commander-node axes A1/X and A2/Y -- expected, since the responder is a
second networked PLC node the commander has to bridge through internally.)

READ_TIMEOUT_S below is set to 0.5 s: ~6x the observed p95/max, so genuine
(if slightly slow) responses are not misclassified as timeouts, while still
being short enough that a single truly stuck read can never stall the whole
benchmark for long -- even a worst-case stage where every single query
times out only costs (4 axes x 0.5 s) = 2 s per cycle, comfortably bounded
within any stage's duration budget (see STAGE_DURATION below). We do NOT
clear the serial input buffer before each query: if a previous response
arrives late (after we've already given up and moved on), its leftover
bytes will still be sitting in the buffer when we send + read the next
command, and will show up as extra/misaligned bytes in that next read --
which is exactly the "garbled" failure signature we want to be able to
detect, not an artifact to hide.

Stage duration logic
---------------------
For each target frequency, stage duration = max(15, min(60, 30/target_hz)):
at least 15 s so even fast stages get a solid sample size, scaled up for
low frequencies so 1-2 Hz stages still collect ~30 cycles, capped at 60 s
so the whole sweep stays practical. Concretely, for the sequence 1, 2, 4,
8, ..., 1024 Hz this evaluates to 30 s at 1 Hz and 15 s for every other
frequency (2 Hz already gives 30/2=15, and every higher frequency clamps to
the same 15 s floor) -- total sweep runtime ~180 s (3 minutes).
"""

from __future__ import annotations

import csv
import statistics
import time
from pathlib import Path

import serial

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

HOST = "red.dyn.ucr.edu"
PORT = 9700
SERIAL_URL = f"socket://{HOST}:{PORT}"
BAUDRATE = 9600  # existing configured value -- deliberately NOT changed
READ_TIMEOUT_S = 0.5  # see "Read timeout choice" in module docstring

# Axis label -> exact command string. This is the ONLY set of commands this
# script is permitted to send (read-only ACP queries), per the safety
# constraint in the module docstring.
AXES = (
    ("X", "A1 ACP"),
    ("Y", "A2 ACP"),
    ("Z", "A5 ACP"),
    ("Theta", "A6 ACP"),
)
ALLOWED_COMMANDS = frozenset(cmd for _, cmd in AXES)

TARGET_HZS = (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024)

# Tolerance factor for "did this cycle keep pace with its target period?".
# A cycle is considered "on pace" if its actual wall-clock duration was no
# more than 1.5x the intended target period. This is deliberately a bit
# looser than 1.0x to absorb ordinary scheduling/GC jitter without treating
# every marginal cycle as a failure to keep pace, while still being tight
# enough to clearly flag the frequencies where the link structurally cannot
# keep up (which, given ~250ms observed for a full 4-axis cycle, is expected
# to be everything above roughly 4 Hz).
PACE_TOLERANCE_FACTOR = 1.5

DATA_DIR = Path(__file__).resolve().parent / "data"
RAW_CSV = DATA_DIR / "raw_polls.csv"
SUMMARY_CSV = DATA_DIR / "summary_by_frequency.csv"

RAW_FIELDNAMES = [
    "target_hz",
    "cycle_index",
    "axis",
    "command",
    "t_send_perf",
    "t_send_epoch",
    "t_recv_perf",
    "t_recv_epoch",
    "latency_s",
    "raw_bytes_repr",
    "raw_len",
    "classification",
    "parsed_value",
    "error_code",
    "cycle_target_period_s",
    "cycle_actual_duration_s",
    "kept_pace",
]

SUMMARY_FIELDNAMES = [
    "target_hz",
    "target_period_s",
    "stage_duration_requested_s",
    "stage_duration_actual_s",
    "cycles_attempted",
    "cycles_fully_successful",
    "cycles_kept_pace",
    "total_axis_polls",
    "successful_axis_polls",
    "timeout_count",
    "parse_error_count",
    "garbled_count",
    "drop_rate",
    "achieved_cycle_rate_hz",
    "mean_latency_s",
    "median_latency_s",
    "p95_latency_s",
    "max_latency_s",
] + [
    f"{stat}_latency_{axis}_s"
    for axis in ("X", "Y", "Z", "Theta")
    for stat in ("mean", "median", "p95", "max")
]


# ---------------------------------------------------------------------------
# Response classification
# ---------------------------------------------------------------------------
#
# Classifications (defined precisely, since this is the crux of the
# reliability measurement):
#   success      -- terminating '>' received; envelope is "0 <value>" and
#                    <value> parses as a float. This is the only case we
#                    count as a real, usable position reading.
#   timeout      -- no terminating '>' byte received within READ_TIMEOUT_S
#                    of sending the command (pyserial's read_until()
#                    returned without finding b'>').
#   parse_error  -- a terminating '>' WAS received (framing looked intact --
#                    exactly one '>', in a plausible position), but the
#                    payload before it did not parse as a valid ACP success
#                    value. This covers two real sub-cases we don't
#                    distinguish further in the top-level classification
#                    (the `error_code` column preserves the distinction):
#                      (a) a genuine firmware error envelope "<code> >"
#                          (first token != "0") -- e.g. a comm-error escape
#                          code:  the framing is fine, but there is no
#                          position value to report;
#                      (b) a "0 ..." envelope where the value token itself
#                          isn't a parseable float.
#   garbled      -- classic sign of a dropped/overlapping response at a
#                    poll rate the link can't sustain: either more than one
#                    '>' shows up in a single read (i.e. a stray/leftover
#                    frame boundary from a previous, late-arriving response
#                    got concatenated with this one), or the payload before
#                    '>' doesn't even tokenize into a plausible 1-2-token
#                    numeric envelope (extra stray bytes, truncated/merged
#                    digits, non-numeric junk).
#
# We deliberately do NOT clear the serial input buffer before each query
# (see module docstring) so that garbling from late-arriving previous
# responses is observable rather than silently swallowed.

import re

_TOKEN_SPLIT = re.compile(r"[,\s]+")
_NUMERIC_TOKEN = re.compile(r"^[-+]?\d+(\.\d+)?$")


def classify_response(raw: bytes):
    """Classify one raw response buffer.

    Returns (classification, parsed_value_or_None, error_code_or_None).
    """
    if b">" not in raw:
        return "timeout", None, None

    prompt_count = raw.count(b">")
    # Payload is everything up to the FIRST '>' -- if a second one shows up
    # anywhere in the buffer, that's a leftover fragment from an overlapping
    # response: garbled, regardless of whether the first part looks clean.
    payload_bytes = raw.split(b">", 1)[0]
    try:
        payload = payload_bytes.decode("ascii", errors="strict")
    except UnicodeDecodeError:
        return "garbled", None, None

    lines = [line for line in re.split(r"[\r\n]+", payload) if line.strip()]
    if not lines:
        # '>' arrived with nothing meaningful before it.
        return "garbled" if prompt_count > 1 else "parse_error", None, None

    last = lines[-1]
    tokens = [t for t in _TOKEN_SPLIT.split(last.strip()) if t]

    if prompt_count > 1:
        return "garbled", None, None

    if not tokens or len(tokens) > 2:
        return "garbled", None, None

    if not all(_NUMERIC_TOKEN.match(t) for t in tokens):
        return "garbled", None, None

    if tokens[0] == "0":
        if len(tokens) == 2:
            try:
                return "success", float(tokens[1]), None
            except ValueError:
                return "parse_error", None, None
        # bare "0" with no value token -- not a position reading.
        return "parse_error", None, None

    # First token isn't "0": either a genuine firmware error code, or noise.
    try:
        code = int(float(tokens[0]))
        return "parse_error", None, code
    except ValueError:
        return "garbled", None, None


# ---------------------------------------------------------------------------
# Benchmark core
# ---------------------------------------------------------------------------


def run_stage(ser: serial.Serial, target_hz: float, raw_rows: list):
    period = 1.0 / target_hz
    duration = max(15.0, min(60.0, 30.0 / target_hz))

    stage_start_perf = time.perf_counter()
    stage_deadline = stage_start_perf + duration

    cycle_index = 0
    cycles_fully_successful = 0
    cycles_kept_pace = 0

    while time.perf_counter() < stage_deadline:
        cycle_start_perf = time.perf_counter()
        cycle_all_ok = True

        for axis_label, cmd in AXES:
            assert cmd in ALLOWED_COMMANDS, f"refusing to send disallowed command: {cmd!r}"

            t_send_perf = time.perf_counter()
            t_send_epoch = time.time()
            ser.write((cmd + "\r").encode("ascii"))
            raw = ser.read_until(b">")
            t_recv_perf = time.perf_counter()
            t_recv_epoch = time.time()

            latency_s = t_recv_perf - t_send_perf
            classification, value, error_code = classify_response(raw)
            if classification != "success":
                cycle_all_ok = False

            raw_rows.append(
                {
                    "target_hz": target_hz,
                    "cycle_index": cycle_index,
                    "axis": axis_label,
                    "command": cmd,
                    "t_send_perf": t_send_perf,
                    "t_send_epoch": t_send_epoch,
                    "t_recv_perf": t_recv_perf,
                    "t_recv_epoch": t_recv_epoch,
                    "latency_s": latency_s,
                    "raw_bytes_repr": repr(raw),
                    "raw_len": len(raw),
                    "classification": classification,
                    "parsed_value": value if value is not None else "",
                    "error_code": error_code if error_code is not None else "",
                    # filled in after we know the cycle's actual duration:
                    "cycle_target_period_s": period,
                    "cycle_actual_duration_s": None,
                    "kept_pace": None,
                }
            )

        cycle_end_perf = time.perf_counter()
        cycle_actual_duration = cycle_end_perf - cycle_start_perf
        kept_pace = cycle_actual_duration <= period * PACE_TOLERANCE_FACTOR

        # Backfill the per-cycle timing columns onto the 4 rows just appended.
        for row in raw_rows[-len(AXES):]:
            row["cycle_actual_duration_s"] = cycle_actual_duration
            row["kept_pace"] = kept_pace

        if cycle_all_ok:
            cycles_fully_successful += 1
        if kept_pace:
            cycles_kept_pace += 1

        cycle_index += 1

        # Pace to the next cycle boundary; if we're already behind (typical
        # once target_hz exceeds what the link can sustain), don't sleep at
        # all -- just proceed immediately, which is exactly what should
        # happen to reveal the achievable ceiling.
        next_cycle_target = stage_start_perf + cycle_index * period
        sleep_for = next_cycle_target - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)

    stage_actual_duration = time.perf_counter() - stage_start_perf
    cycles_attempted = cycle_index

    return {
        "target_hz": target_hz,
        "target_period_s": period,
        "stage_duration_requested_s": duration,
        "stage_duration_actual_s": stage_actual_duration,
        "cycles_attempted": cycles_attempted,
        "cycles_fully_successful": cycles_fully_successful,
        "cycles_kept_pace": cycles_kept_pace,
    }


def _pctile(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    n = len(sorted_vals)
    idx = min(n - 1, max(0, int(round(p * (n - 1)))))
    return sorted_vals[idx]


def summarize_stage(stage_meta: dict, raw_rows: list) -> dict:
    target_hz = stage_meta["target_hz"]
    rows = [r for r in raw_rows if r["target_hz"] == target_hz]

    total_axis_polls = len(rows)
    successful = [r for r in rows if r["classification"] == "success"]
    timeouts = [r for r in rows if r["classification"] == "timeout"]
    parse_errors = [r for r in rows if r["classification"] == "parse_error"]
    garbled = [r for r in rows if r["classification"] == "garbled"]

    successful_axis_polls = len(successful)
    drop_rate = (
        (total_axis_polls - successful_axis_polls) / total_axis_polls
        if total_axis_polls
        else float("nan")
    )

    all_latencies = sorted(r["latency_s"] for r in rows)
    achieved_cycle_rate_hz = (
        stage_meta["cycles_attempted"] / stage_meta["stage_duration_actual_s"]
        if stage_meta["stage_duration_actual_s"] > 0
        else float("nan")
    )

    summary = {
        "target_hz": target_hz,
        "target_period_s": stage_meta["target_period_s"],
        "stage_duration_requested_s": stage_meta["stage_duration_requested_s"],
        "stage_duration_actual_s": stage_meta["stage_duration_actual_s"],
        "cycles_attempted": stage_meta["cycles_attempted"],
        "cycles_fully_successful": stage_meta["cycles_fully_successful"],
        "cycles_kept_pace": stage_meta["cycles_kept_pace"],
        "total_axis_polls": total_axis_polls,
        "successful_axis_polls": successful_axis_polls,
        "timeout_count": len(timeouts),
        "parse_error_count": len(parse_errors),
        "garbled_count": len(garbled),
        "drop_rate": drop_rate,
        "achieved_cycle_rate_hz": achieved_cycle_rate_hz,
        "mean_latency_s": statistics.mean(all_latencies) if all_latencies else float("nan"),
        "median_latency_s": statistics.median(all_latencies) if all_latencies else float("nan"),
        "p95_latency_s": _pctile(all_latencies, 0.95),
        "max_latency_s": max(all_latencies) if all_latencies else float("nan"),
    }

    for axis_label, _ in AXES:
        axis_lats = sorted(r["latency_s"] for r in rows if r["axis"] == axis_label)
        summary[f"mean_latency_{axis_label}_s"] = (
            statistics.mean(axis_lats) if axis_lats else float("nan")
        )
        summary[f"median_latency_{axis_label}_s"] = (
            statistics.median(axis_lats) if axis_lats else float("nan")
        )
        summary[f"p95_latency_{axis_label}_s"] = _pctile(axis_lats, 0.95)
        summary[f"max_latency_{axis_label}_s"] = max(axis_lats) if axis_lats else float("nan")

    return summary


def main():
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Connecting to {SERIAL_URL} at {BAUDRATE} baud (unchanged from existing config)...")
    ser = serial.serial_for_url(
        SERIAL_URL,
        baudrate=BAUDRATE,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=READ_TIMEOUT_S,
    )
    print("Connected.")

    try:
        # Sanity warm-up: one query per axis, must all succeed before we
        # trust the link enough to run the full sweep.
        print("Warm-up queries:")
        for axis_label, cmd in AXES:
            assert cmd in ALLOWED_COMMANDS
            ser.write((cmd + "\r").encode("ascii"))
            raw = ser.read_until(b">")
            classification, value, error_code = classify_response(raw)
            print(f"  {axis_label:6s} {cmd:10s} -> {raw!r}  [{classification}] value={value}")
            if classification != "success":
                raise RuntimeError(
                    f"Warm-up query failed for {cmd!r}: {raw!r} "
                    f"(classification={classification}, error_code={error_code}). "
                    "Aborting before running the full sweep."
                )

        raw_rows: list = []
        summary_rows: list = []

        for target_hz in TARGET_HZS:
            duration = max(15.0, min(60.0, 30.0 / target_hz))
            print(
                f"\n=== Stage: target={target_hz} Hz, period={1.0/target_hz*1000:.3f} ms, "
                f"duration={duration:.1f} s ==="
            )
            stage_meta = run_stage(ser, target_hz, raw_rows)
            summary = summarize_stage(stage_meta, raw_rows)
            summary_rows.append(summary)
            print(
                f"  cycles_attempted={summary['cycles_attempted']} "
                f"fully_successful={summary['cycles_fully_successful']} "
                f"kept_pace={summary['cycles_kept_pace']} "
                f"achieved_rate={summary['achieved_cycle_rate_hz']:.3f} Hz "
                f"drop_rate={summary['drop_rate']*100:.2f}% "
                f"median_latency={summary['median_latency_s']*1000:.2f} ms "
                f"p95_latency={summary['p95_latency_s']*1000:.2f} ms "
                f"timeouts={summary['timeout_count']} "
                f"parse_errors={summary['parse_error_count']} "
                f"garbled={summary['garbled_count']}"
            )

        print(f"\nWriting raw per-poll CSV: {RAW_CSV}")
        with open(RAW_CSV, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=RAW_FIELDNAMES)
            writer.writeheader()
            writer.writerows(raw_rows)

        print(f"Writing summary CSV: {SUMMARY_CSV}")
        with open(SUMMARY_CSV, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDNAMES)
            writer.writeheader()
            writer.writerows(summary_rows)

        print("\nDone.")

    finally:
        ser.close()
        print("Connection closed.")


if __name__ == "__main__":
    main()
