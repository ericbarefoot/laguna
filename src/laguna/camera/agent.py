#!/usr/bin/env python3
"""Camera agent — runs on each Raspberry Pi.

Invoked by the coordinator over SSH:
    python3 camera_agent.py <target_unix_timestamp> [output_dir]

Waits until target_unix_timestamp, captures one image with picamzero,
then writes a single JSON line to stdout with timing metadata.
"""

import importlib
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def _log(msg: str) -> None:
    """Write a timestamped progress line to stderr so the coordinator can stream it."""
    now = time.time()
    utc = time.strftime("%H:%M:%S", time.gmtime(now)) + f".{int(now % 1 * 1000):03d}Z"
    local = time.strftime("%H:%M:%S", time.localtime(now))
    print(f"[agent {utc}/{local}] {msg}", file=sys.stderr, flush=True)


def capture_at_time(target_time: float, output_dir: str = "/tmp/laguna_captures") -> dict:
    _log("Starting — importing picamzero...")
    try:
        picamzero = importlib.import_module("picamzero")
        Camera = picamzero.Camera
    except ImportError:
        _log("ERROR: picamzero not available")
        return {"error": "picamzero not available on this host"}
    _log("picamzero imported OK")

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    now = time.time()
    wait = target_time - now

    if wait < -2.0:
        _log(f"ERROR: target_time is {-wait:.3f}s in the past")
        return {
            "error": f"target_time is {-wait:.3f}s in the past — increase lead_time on coordinator"
        }

    _log(f"Sleeping {wait:.3f}s until target_time...")
    if wait > 0:
        time.sleep(wait)
    _log("Sleep done — initializing Camera()...")

    cam = Camera()
    _log("Camera() initialized — calling take_photo()...")

    utc_str = datetime.fromtimestamp(target_time, tz=timezone.utc).strftime("%Y%m%dT%H%M%S%f")[:-3] + "Z"
    filename = str(out / f"capture_{utc_str}.jpg")
    t_before = time.time()
    cam.take_photo(filename)
    t_after = time.time()
    _log(f"take_photo() returned in {(t_after - t_before)*1000:.0f}ms — writing result")

    return {
        "filename": filename,
        "target_time": target_time,
        "capture_time_start": t_before,
        "capture_time_end": t_after,
        "capture_time_mid": (t_before + t_after) / 2,
        "capture_duration_ms": (t_after - t_before) * 1000,
        "latency_ms": (t_before - target_time) * 1000,
    }


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(json.dumps({"error": "usage: camera_agent.py <target_unix_timestamp> [output_dir]"}))
        sys.exit(1)

    target_time = float(sys.argv[1])
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/laguna_captures"

    result = capture_at_time(target_time, output_dir)
    print(json.dumps(result))
