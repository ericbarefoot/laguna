#!/usr/bin/env python3
"""Camera agent — runs on each Raspberry Pi.

CameraArray copies this file to /tmp/laguna_camera_agent.py and runs it with
a bare `python3`. The Pi does not have the laguna package installed, so this
file must stay standalone and must not import laguna.

Invoked by the coordinator over SSH:
    python3 camera_agent.py capture <target_unix_timestamp> [output_dir] [--controls JSON]
    python3 camera_agent.py calibrate [output_dir] [--warmup-s SECONDS]

``capture`` waits until target_unix_timestamp, takes one image, then writes a
single JSON line to stdout with timing metadata.

* Without ``--controls`` it uses picamzero, i.e. libcamera's automatic
  exposure and white balance (historical behaviour).
* With ``--controls`` it uses picamera2 directly, switches AE/AWB off and
  applies the fixed exposure time, gain and colour gains. If picamera2 is
  missing it returns an error rather than capturing in auto mode: a frame at
  the wrong exposure cannot be retaken.

``calibrate`` lets AE/AWB converge, then reports the values they settled on
(and a preview frame) for an operator to confirm. It is never run on a
schedule.
"""

import argparse
import importlib
import json
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

# Open the camera this long before target_time so the fixed controls have
# taken effect by the moment of capture; the timestamp then reflects the
# shutter, not the camera start-up.
PREOPEN_S = 3.0

# Frames discarded after applying manual controls. Sensor exposure and gain
# land a few frames after set_controls() because of pipeline latency.
SETTLE_FRAMES = 8

# Frames sampled at the end of calibration warm-up.
CALIBRATION_SAMPLES = 8

# Relative spread (max-min)/median across the sampled frames at or below
# which AE/AWB are considered converged.
CONVERGED_SPREAD = 0.05


def utc_local_ts() -> tuple[str, str]:
    """Return (utc_str, local_str) with millisecond precision for log lines.

    Kept in this file on purpose: the script is executed on the Pi, outside
    the laguna package.
    """
    now = time.time()
    utc = time.strftime("%H:%M:%S", time.gmtime(now)) + f".{int(now % 1 * 1000):03d}Z"
    local = time.strftime("%H:%M:%S", time.localtime(now))
    return utc, local


def _log(msg: str) -> None:
    """Write a timestamped progress line to stderr so the coordinator can stream it."""
    utc, local = utc_local_ts()
    print(f"[agent {utc}/{local}] {msg}", file=sys.stderr, flush=True)


def _timestamp_name(target_time: float) -> str:
    """Millisecond UTC stamp used in capture file names."""
    return datetime.fromtimestamp(target_time, tz=timezone.utc).strftime("%Y%m%dT%H%M%S%f")[:-3] + "Z"


def _import_picamera2():
    """Return the ``Picamera2`` class, or None if picamera2 is not installed."""
    try:
        return importlib.import_module("picamera2").Picamera2
    except ImportError:
        return None


def _frame_values(metadata: dict) -> dict:
    """Pick exposure-relevant fields out of a libcamera metadata dict."""
    cg = metadata.get("ColourGains")
    return {
        "exposure_time_us": metadata.get("ExposureTime"),
        "analogue_gain": metadata.get("AnalogueGain"),
        "colour_gains": [float(cg[0]), float(cg[1])] if cg is not None else None,
    }


def _check_controls(controls: dict) -> dict:
    """Validate a controls dict from the coordinator; raise ValueError if unusable."""
    try:
        t = float(controls["exposure_time_us"])
        g = float(controls["analogue_gain"])
        r, b = (float(v) for v in controls["awb_gains"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"bad controls {controls!r}: {exc}") from None
    if not (t >= 1 and g >= 1 and r > 0 and b > 0):
        raise ValueError(f"controls out of range: {controls!r}")
    return {"exposure_time_us": int(round(t)), "analogue_gain": g, "awb_gains": (r, b)}


def _wait_until(target_time: float) -> None:
    """Sleep until the wall-clock time ``target_time``."""
    wait = target_time - time.time()
    if wait > 0:
        time.sleep(wait)


def _capture_locked(target_time: float, output_dir: str, controls: dict) -> dict:
    """Capture one frame with AE/AWB off and fixed controls (picamera2)."""
    Picamera2 = _import_picamera2()
    if Picamera2 is None:
        _log("ERROR: picamera2 not available — refusing to capture in auto mode")
        return {"error": "picamera2 not available on this host; cannot apply fixed exposure"}
    try:
        fixed = _check_controls(controls)
    except ValueError as exc:
        return {"error": str(exc)}

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    wait = target_time - time.time()
    if wait < -2.0:
        _log(f"ERROR: target_time is {-wait:.3f}s in the past")
        return {
            "error": f"target_time is {-wait:.3f}s in the past — increase lead_time on coordinator"
        }

    if wait > PREOPEN_S:
        _log(f"Sleeping {wait - PREOPEN_S:.3f}s before opening camera...")
        time.sleep(wait - PREOPEN_S)

    cam = Picamera2()
    try:
        cam.configure(cam.create_still_configuration())
        cam.set_controls(
            {
                "AeEnable": False,
                "AwbEnable": False,
                "ExposureTime": fixed["exposure_time_us"],
                "AnalogueGain": fixed["analogue_gain"],
                "ColourGains": fixed["awb_gains"],
            }
        )
        cam.start()
        for _ in range(SETTLE_FRAMES):
            cam.capture_metadata()
        _log(f"Controls applied, waiting for target_time (T{target_time - time.time():+.3f}s)")
        _wait_until(target_time)

        filename = str(out / f"capture_{_timestamp_name(target_time)}.jpg")
        t_before = time.time()
        request = cam.capture_request()
        try:
            request.save("main", filename)
            metadata = request.get_metadata()
        finally:
            request.release()
        t_after = time.time()
    finally:
        try:
            cam.stop()
        finally:
            cam.close()
    _log(f"capture done in {(t_after - t_before) * 1000:.0f}ms")

    return {
        "filename": filename,
        "target_time": target_time,
        "capture_time_start": t_before,
        "capture_time_end": t_after,
        "capture_time_mid": (t_before + t_after) / 2,
        "capture_duration_ms": (t_after - t_before) * 1000,
        "latency_ms": (t_before - target_time) * 1000,
        "exposure_mode": "locked",
        **_frame_values(metadata),
    }


def _capture_auto(target_time: float, output_dir: str) -> dict:
    """Capture one frame with libcamera's automatic exposure/white balance (picamzero)."""
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

    wait = target_time - time.time()

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

    filename = str(out / f"capture_{_timestamp_name(target_time)}.jpg")
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
        "exposure_mode": "auto",
    }


def capture_at_time(
    target_time: float,
    output_dir: str = "/tmp/laguna_captures",
    controls: "dict | None" = None,
) -> dict:
    """Capture one image at a specific Unix timestamp on a remote Raspberry Pi.

    Args:
        target_time: Unix timestamp (seconds since epoch) when capture should occur.
        output_dir: Directory to write captured image files (default /tmp/laguna_captures).
        controls: Fixed exposure settings ``{"exposure_time_us", "analogue_gain",
            "awb_gains": [red, blue]}``. When given, AE and AWB are disabled and
            these are applied (picamera2). When None, libcamera's automatic
            exposure and white balance are used (picamzero).

    Returns:
        On success, a dict with filename, target_time, capture_time_start,
        capture_time_end, capture_time_mid, capture_duration_ms, latency_ms,
        exposure_mode ("locked" or "auto") and, for a locked capture, the
        exposure_time_us / analogue_gain / colour_gains the camera reported
        for the frame. On failure, a dict with an "error" key holding a string
        description.
    """
    if controls is not None:
        return _capture_locked(target_time, output_dir, controls)
    return _capture_auto(target_time, output_dir)


def calibrate(output_dir: str = "/tmp/laguna_captures", warmup_s: float = 5.0) -> dict:
    """Let AE/AWB converge and report the values they settled on.

    Args:
        output_dir: Directory for the preview frame.
        warmup_s: Seconds to run with AE/AWB on before sampling.

    Returns:
        A dict with exposure_time_us, analogue_gain, colour_gains (medians over
        the sampled frames), converged (bool), spread (largest relative
        max-min across the sampled frames), samples, and filename of a preview
        JPEG. On failure, a dict with an "error" key.
    """
    Picamera2 = _import_picamera2()
    if Picamera2 is None:
        return {"error": "picamera2 not available on this host; cannot calibrate"}

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cam = Picamera2()
    try:
        cam.configure(cam.create_still_configuration())
        cam.set_controls({"AeEnable": True, "AwbEnable": True})
        cam.start()
        _log(f"Warming up AE/AWB for {warmup_s:.1f}s...")
        deadline = time.time() + warmup_s
        while time.time() < deadline:
            cam.capture_metadata()

        samples = [_frame_values(cam.capture_metadata()) for _ in range(CALIBRATION_SAMPLES)]
        if any(s["exposure_time_us"] is None or s["analogue_gain"] is None
               or s["colour_gains"] is None for s in samples):
            return {"error": "camera did not report ExposureTime/AnalogueGain/ColourGains"}

        filename = str(out / f"calibration_{_timestamp_name(time.time())}.jpg")
        request = cam.capture_request()
        try:
            request.save("main", filename)
        finally:
            request.release()
    finally:
        try:
            cam.stop()
        finally:
            cam.close()

    series = {
        "exposure_time_us": [float(s["exposure_time_us"]) for s in samples],
        "analogue_gain": [float(s["analogue_gain"]) for s in samples],
        "colour_red": [s["colour_gains"][0] for s in samples],
        "colour_blue": [s["colour_gains"][1] for s in samples],
    }
    med = {k: statistics.median(v) for k, v in series.items()}
    spread = max((max(v) - min(v)) / med[k] if med[k] else float("inf") for k, v in series.items())
    return {
        "exposure_time_us": int(round(med["exposure_time_us"])),
        "analogue_gain": med["analogue_gain"],
        "colour_gains": [med["colour_red"], med["colour_blue"]],
        "converged": spread <= CONVERGED_SPREAD,
        "spread": spread,
        "samples": len(samples),
        "filename": filename,
    }


def _parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    p = argparse.ArgumentParser(prog="camera_agent.py")
    sub = p.add_subparsers(dest="action", required=True)
    c = sub.add_parser("capture", help="capture one image at a target time")
    c.add_argument("target_time", type=float, help="Unix timestamp to capture at")
    c.add_argument("output_dir", nargs="?", default="/tmp/laguna_captures")
    c.add_argument("--controls", help="JSON: fixed exposure_time_us, analogue_gain, awb_gains")
    k = sub.add_parser("calibrate", help="report converged AE/AWB values")
    k.add_argument("output_dir", nargs="?", default="/tmp/laguna_captures")
    k.add_argument("--warmup-s", type=float, default=5.0)
    return p


if __name__ == "__main__":
    args = _parser().parse_args()
    if args.action == "calibrate":
        result = calibrate(args.output_dir, args.warmup_s)
    else:
        ctrl = None
        if args.controls is not None:
            try:
                ctrl = json.loads(args.controls)
            except json.JSONDecodeError as exc:
                print(json.dumps({"error": f"--controls is not valid JSON: {exc}"}))
                sys.exit(1)
        result = capture_at_time(args.target_time, args.output_dir, ctrl)
    print(json.dumps(result))
