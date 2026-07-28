#!/usr/bin/env python3
"""Interactive height calibration for the OD2000 or WTT12L PowerProx.

Both rangefinders read distance *down* to whatever's under them, and
neither is mounted perfectly vertical, so raw readings need a linear
(slope + intercept) correction to become real-world z height — see
laguna.rangefinder.calibration for why a 2-parameter line fit handles the
mount-angle error, not just a fixed offset.

Usage — collect a new calibration interactively:

    python scripts/calibrate_rangefinder.py calibrate --device od2000 \\
        --al1342-ip 192.168.1.251 --pdin-port 2 --output od2000_cal.csv

    python scripts/calibrate_rangefinder.py calibrate --device wtt12l_powerprox \\
        --al1342-ip 192.168.1.251 --pdin-port 7 --output wtt12l_cal.csv

You'll be prompted to place a block of known height under the sensor and
enter its height; repeat for as many blocks as you have (3+ recommended —
2 points fit a line trivially with no way to sanity-check it, more points
let you see the residual error per point). Type 'done' when finished,
'undo' to drop the last point if a reading looked bad.

Usage — apply a saved calibration to live readings:

    python scripts/calibrate_rangefinder.py apply --device od2000 \\
        --al1342-ip 192.168.1.251 --pdin-port 2 --input od2000_cal.csv

Prints real-world z height once a second until interrupted (Ctrl-C).

Design note on what "raw_value" means per device:
  - od2000: distance_mm from decode_od2000_pdin() — the OD2000's own
    time-of-flight measurement, already metric and reasonably linear; the
    calibration fit here is mostly correcting for mount angle.
  - wtt12l_powerprox: current_ma from decode_dp4200_wtt12l_analog_pdin() —
    deliberately the *raw current*, not that function's own distance_mm
    (which already bakes in an assumed, unconfirmed 4-20mA=100-1400mm
    span — see docs/WTT12L_POWERPROX_SETUP.md). Fitting directly from
    current to real height in one step avoids compounding two separate
    uncertain linear transforms (assumed current->distance, then
    distance->real height) into one, and should be more accurate.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
import urllib.request
from typing import Callable, List

from laguna.rangefinder import decode_dp4200_wtt12l_analog_pdin, decode_od2000_pdin
from laguna.rangefinder.calibration import CalibrationPoint, LinearCalibration


def _al1342_pdin_hex(al1342_ip: str, pdin_port: int, timeout: float = 5.0) -> str:
    """One on-demand pdin/getdata read against the AL1342. See
    docs/MQTT_AL1342_SETUP.md — the AL1342 has no DNS of its own, address
    it by raw IP, and this is plain HTTP POST, not MQTT."""
    adr = f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/pdin/getdata"
    payload = json.dumps({"code": "request", "cid": -1, "adr": adr}).encode()
    req = urllib.request.Request(
        f"http://{al1342_ip}/", data=payload, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = json.loads(resp.read())
    if body.get("code") != 200:
        raise RuntimeError(
            f"AL1342 returned code {body.get('code')} for {adr} — "
            f"check pdin_port and that the device is connected"
        )
    return body["data"]["value"]


def _read_od2000_raw(al1342_ip: str, pdin_port: int) -> float:
    return decode_od2000_pdin(_al1342_pdin_hex(al1342_ip, pdin_port))["distance_mm"]


def _read_wtt12l_powerprox_raw(al1342_ip: str, pdin_port: int) -> float:
    return decode_dp4200_wtt12l_analog_pdin(_al1342_pdin_hex(al1342_ip, pdin_port))["current_ma"]


DEVICES: dict = {
    "od2000": {
        "read_raw": _read_od2000_raw,
        "raw_units": "mm",
        "default_pdin_port": 2,
    },
    "wtt12l_powerprox": {
        "read_raw": _read_wtt12l_powerprox_raw,
        "raw_units": "mA (via DP4200 bridge)",
        "default_pdin_port": 7,
    },
}


def _sample_raw(
    read_raw: Callable[[str, int], float],
    al1342_ip: str,
    pdin_port: int,
    n_samples: int,
    delay_s: float,
) -> float:
    """Take n_samples readings and return their mean, printing the spread
    so an unstable/misaimed target is obvious before it's baked into the
    fit."""
    samples: List[float] = []
    for _ in range(n_samples):
        samples.append(read_raw(al1342_ip, pdin_port))
        time.sleep(delay_s)
    mean = statistics.mean(samples)
    spread = statistics.stdev(samples) if len(samples) > 1 else 0.0
    print(f"    {n_samples} samples: mean={mean:.4f}, stdev={spread:.4f}")
    if spread > abs(mean) * 0.05 and spread > 1e-6:
        print("    warning: high spread relative to the reading — check the "
              "target is stable and centered under the sensor before continuing")
    return mean


def cmd_calibrate(args: argparse.Namespace) -> None:
    device = DEVICES[args.device]
    read_raw = device["read_raw"]
    pdin_port = args.pdin_port if args.pdin_port is not None else device["default_pdin_port"]

    print(f"Calibrating {args.device} on AL1342 {args.al1342_ip}, port {pdin_port}")
    print(f"Raw units: {device['raw_units']}")
    print()
    print("For each block: place it under the sensor, enter its real-world")
    print("height in mm, and I'll take a burst of readings. Type 'done' when")
    print("finished (3+ points recommended), or 'undo' to drop the last point.")
    print()

    points: List[CalibrationPoint] = []
    while True:
        prompt = f"[{len(points)} points collected] Known height in mm (or 'done'/'undo'): "
        raw_input_str = input(prompt).strip().lower()
        if raw_input_str in ("done", "d", "q", "quit"):
            break
        if raw_input_str in ("undo", "u"):
            if points:
                removed = points.pop()
                print(f"  removed point: height={removed.known_height_mm}, "
                      f"raw={removed.raw_value:.4f}")
            else:
                print("  no points to undo")
            continue
        try:
            known_height_mm = float(raw_input_str)
        except ValueError:
            print("  not a number — enter a height in mm, or 'done'/'undo'")
            continue

        try:
            raw_value = _sample_raw(
                read_raw, args.al1342_ip, pdin_port, args.samples, args.sample_delay
            )
        except Exception as e:
            print(f"  read failed: {e} — point not recorded, try again")
            continue

        points.append(CalibrationPoint(known_height_mm=known_height_mm, raw_value=raw_value))

    if len(points) < 2:
        print(f"Only {len(points)} point(s) collected — need at least 2 to fit a "
              f"calibration. Nothing saved.")
        sys.exit(1)

    cal = LinearCalibration.fit(args.device, points)
    print()
    print(f"Fit: real_height_mm = {cal.slope:.6f} * raw + {cal.intercept:.6f}")
    print(f"r_squared = {cal.r_squared:.6f}")
    print("Residuals (predicted - known, mm):")
    for p, resid in zip(cal.points, cal.residuals_mm()):
        print(f"  height={p.known_height_mm:>8.2f}  raw={p.raw_value:>10.4f}  "
              f"residual={resid:+.2f}")

    if args.output:
        cal.to_csv(args.output)
        print(f"\nSaved calibration to {args.output}")
    else:
        out = input("\nSave calibration to file? (path, or blank to skip): ").strip()
        if out:
            cal.to_csv(out)
            print(f"Saved calibration to {out}")


def cmd_apply(args: argparse.Namespace) -> None:
    device = DEVICES[args.device]
    read_raw = device["read_raw"]
    pdin_port = args.pdin_port if args.pdin_port is not None else device["default_pdin_port"]

    cal = LinearCalibration.from_csv(args.input)
    print(f"Loaded calibration for '{cal.device}': "
          f"real_height_mm = {cal.slope:.6f} * raw + {cal.intercept:.6f} "
          f"(r_squared={cal.r_squared:.4f}, {len(cal.points)} points)")
    if cal.device != args.device:
        print(f"warning: calibration was fitted for '{cal.device}', "
              f"applying it to '{args.device}' readings")

    print(f"Reading {args.device} on AL1342 {args.al1342_ip}, port {pdin_port} "
          f"(Ctrl-C to stop)...")
    try:
        while True:
            raw = read_raw(args.al1342_ip, pdin_port)
            height_mm = cal.apply(raw)
            print(f"raw={raw:.4f}  ->  real_height_mm={height_mm:.2f}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="command", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--device", choices=sorted(DEVICES), required=True)
    common.add_argument("--al1342-ip", default="192.168.1.251")
    common.add_argument("--pdin-port", type=int, default=None,
                         help="defaults per-device: od2000=2, wtt12l_powerprox=7 "
                              "(confirm on your own hardware — see docs/MQTT_AL1342_SETUP.md "
                              "and docs/WTT12L_POWERPROX_SETUP.md)")

    p_cal = subparsers.add_parser("calibrate", parents=[common],
                                   help="interactively collect known-height points and fit a calibration")
    p_cal.add_argument("--output", default=None, help="CSV path to save the fitted calibration to")
    p_cal.add_argument("--samples", type=int, default=10,
                        help="readings averaged per calibration point (default: 10)")
    p_cal.add_argument("--sample-delay", type=float, default=0.1,
                        help="seconds between samples within a point (default: 0.1)")
    p_cal.set_defaults(func=cmd_calibrate)

    p_apply = subparsers.add_parser("apply", parents=[common],
                                     help="load a saved calibration and print live calibrated readings")
    p_apply.add_argument("--input", required=True, help="CSV path of a calibration saved by 'calibrate'")
    p_apply.set_defaults(func=cmd_apply)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
