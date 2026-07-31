#!/usr/bin/env python3
"""Run a line scan (X or Y axis) with real-world-unit output.

Moved out of server-setup/plans/ (was run_scan_x_0_to_20mm.py /
run_scan_x_0_to_100mm.py / run_scan_y_0_to_100mm.py — those three were
near-duplicates, and the Y-axis one still had SCAN_AXIS hardcoded to "A1",
i.e. it silently scanned X). This is one script parametrized by --axis
instead.

--sensor selects which rangefinder feeds the scan: "od2000" (default) or
"wtt12l_powerprox" — see gantry_agent.py's SENSOR_DECODERS and
docs/WTT12L_POWERPROX_SETUP.md. The WTT12L path is via a DP4200
analog-input bridge (its own native IO-Link process data never validated
on this AL1342), so it has no programmatic laser control — see that doc's
"Consequence: no programmatic laser control on this path".

--- Real-world units ---

1. Horizontal (position along the scan axis): gantry_agent.py now applies
   the 15 mm/unit conversion itself (see its MM_PER_ACP_UNIT constant and
   docs/GANTRY_UNIT_CALIBRATION.md), so TopographicProfiler's CSV `pos_mm`
   column and `actual_start_mm`/`actual_end_mm` metadata are already real
   mm — this script no longer needs to convert them itself. `--distance-mm`
   below is passed straight through as real mm.

2. Vertical (sensor reading -> real height): a linear (slope + intercept)
   correction from laguna.rangefinder.calibration, fitted against known
   reference-block heights by scripts/calibrate_rangefinder.py. Pass
   --calibration with that tool's output CSV to get a `real_height_mm`
   column; without it, the output only gets the horizontal fix. Which raw
   column the calibration applies to differs by sensor (see
   SENSOR_RAW_COLUMN below) — od2000's distance_mm is a real physical
   time-of-flight measurement already, so its calibration is mostly
   correcting mount angle; wtt12l_powerprox is calibrated directly against
   current_ma (not the decoder's own distance_mm, which already bakes in
   an unconfirmed assumed current-to-distance span — see
   docs/WTT12L_POWERPROX_SETUP.md) to avoid compounding two uncertain
   linear transforms into one.

Usage:

    python3 scripts/run_line_scan.py --axis X --distance-mm 100 --rate-mm-s 10 \\
        --calibration od2000_cal.csv

    python3 scripts/run_line_scan.py --axis Y --distance-mm 100 --rate-mm-s 10

    python3 scripts/run_line_scan.py --axis X --distance-mm 100 --rate-mm-s 10 \\
        --sensor wtt12l_powerprox --calibration wtt12l_cal.csv

Scans the given real-world distance starting from wherever the axis
currently is. Type 'stop' + Enter at any time to cancel early (see the
threading note in the old scripts' docstrings, preserved below in
run_scan()) — both paths end at the same scan_done completion, just with
fewer samples if stopped.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
from pathlib import Path

import pandas as pd

from laguna.rangefinder.calibration import LinearCalibration
from laguna.robot.macron.controller import GantryController
from laguna.robot.macron.pi_bridge import PiGantryConnection
from laguna.robot.macron.profiler import ProfileResult, TopographicProfiler

# --- Connection settings — adjust for your setup ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "/home/eric/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"
AL1342_HOST = "192.168.1.251"

# --- Axis map — confirmed on hardware 2026-07-28 (docs/MACRON_GANTRY.md) ---
AXES = {"X": "A1", "Y": "A2"}

# --- Sensor defaults: IO-Link port and which raw pdin column the
# calibration fit applies to. od2000 confirmed on port 2, wtt12l_powerprox
# (via its DP4200 bridge) on port 7 — both 2026-07-28, see
# docs/MQTT_AL1342_SETUP.md / docs/WTT12L_POWERPROX_SETUP.md. Re-confirm
# on your own hardware before trusting these — port assignment is a
# physical-wiring fact, not something this script can infer. ---
SENSOR_DEFAULTS = {
    "od2000": {"pdin_port": 2, "raw_column": "distance_mm"},
    "wtt12l_powerprox": {"pdin_port": 7, "raw_column": "current_ma"},
}

# --- Horizontal unit conversion — see module docstring point 1 and
# docs/GANTRY_UNIT_CALIBRATION.md. Used only to interpret this script's own
# direct `ACP` query below (a raw passthrough command, not routed through
# gantry_agent.py's scan protocol, which now converts internally) — keep in
# sync by hand with gantry_agent.py's MM_PER_ACP_UNIT. ---
MM_PER_ACP_UNIT = 15.0

DEFAULT_OUTPUT_DIR = "experiments/scan_output"


def add_real_world_columns(
    df: pd.DataFrame,
    calibration: "LinearCalibration | None",
    raw_column: str = "distance_mm",
) -> pd.DataFrame:
    """Add real_pos_mm (always) and real_height_mm (if a calibration is
    given) to a scan DataFrame. Returns a new DataFrame — does not mutate
    the input.

    `pos_mm` is already real mm as of gantry_agent.py's own
    MM_PER_ACP_UNIT conversion (docs/GANTRY_UNIT_CALIBRATION.md) — this
    just copies it to `real_pos_mm` for a consistent column name across
    calibrated and uncalibrated output.

    raw_column: which CSV column the calibration was fitted against and
    should be applied to — "distance_mm" for od2000, "current_ma" for
    wtt12l_powerprox (see SENSOR_DEFAULTS and the module docstring's
    point 2 for why they differ).

    Kept as a standalone function (not buried in main()) so it's usable
    directly against an already-retrieved CSV, e.g. to re-derive real
    units from an old scan without re-running it:

        df = pd.read_csv("profile_20260728_221009.csv")
        cal = LinearCalibration.from_csv("od2000_cal.csv")
        add_real_world_columns(df, cal, raw_column="distance_mm").to_csv(
            "profile_..._real_units.csv")
    """
    out = df.copy()
    out["real_pos_mm"] = out["pos_mm"]
    if calibration is not None:
        out["real_height_mm"] = calibration.apply(out[raw_column])
    return out


def run_scan(
    conn: PiGantryConnection,
    axis: str,
    distance_mm: float,
    rate_mm_s: float,
    output_dir: str,
    sensor: str,
    pdin_port: int,
) -> ProfileResult:
    """Run one line scan on `axis` (raw "A1"/"A2" form), relative distance
    `distance_mm` (real mm) from wherever the axis currently is.

    TopographicProfiler.scan() (via gantry_agent.py) now speaks real mm
    directly, so only the initial position query needs local unit
    conversion (it's a raw `ACP` passthrough command, not routed through
    the scan protocol) — see module docstring point 1.

    Runs the scan on a background thread while the main thread waits for
    either scan completion or the user typing 'stop' + Enter — NOT "open a
    second terminal" (a second terminal would launch a second
    gantry_agent.py that fails to open the already-held serial port). See
    pi_bridge.py's background reader thread for why this single connection
    stays responsive to profiler.stop() while scan() blocks elsewhere.
    """
    start_raw = float(conn.send(f"{axis} ACP"))
    start_mm = start_raw * MM_PER_ACP_UNIT
    target_mm = start_mm + distance_mm
    print(f"Current position: {start_mm:.3f} mm ({start_raw:.3f} raw units)")
    print(f"Scan target ({distance_mm:+.3f} mm): {target_mm:.3f} mm")

    gantry = GantryController(connection=conn)
    profiler = TopographicProfiler(
        gantry=gantry,
        pi_host=PI_HOST,
        pi_user=PI_USER,
        pi_key=PI_KEY,
        pdin_port=pdin_port,
        al1342_host=AL1342_HOST,
        output_dir=output_dir,
        sensor=sensor,
    )

    print()
    print(f"Starting scan: {axis} -> {target_mm:.3f} mm at {rate_mm_s} mm/s "
          f"(~{abs(distance_mm) / rate_mm_s:.0f}s)... sensor={sensor}, pdin_port={pdin_port}")
    if sensor == "od2000":
        print("(Laser turns on automatically for the scan and off again when it's done.)")
    else:
        print("(No programmatic laser control on this sensor path — see module docstring.)")
    print()
    print(">>> Type 'stop' and press Enter at any time to cancel the scan early. <<<")
    print(">>> Otherwise this just waits for the scan to finish on its own.       <<<")
    print()

    scan_result: dict = {}

    def do_scan():
        try:
            scan_result["value"] = profiler.scan(axis=axis, end_mm=target_mm, feed_rate_mm_s=rate_mm_s)
        except Exception as exc:
            scan_result["error"] = exc

    scan_thread = threading.Thread(target=do_scan, daemon=True)
    scan_thread.start()

    # A dedicated stdin-reading thread, since input() can't be interrupted
    # from another thread — lets the main loop poll both "did the scan
    # finish on its own?" and "did the user type stop?" without ever
    # blocking indefinitely on either one.
    stdin_q: "queue.Queue[str]" = queue.Queue()

    def read_stdin():
        while True:
            try:
                line = input()
            except EOFError:
                break
            stdin_q.put(line)

    threading.Thread(target=read_stdin, daemon=True).start()

    stop_sent = False
    while scan_thread.is_alive():
        try:
            line = stdin_q.get(timeout=0.5)
        except queue.Empty:
            continue
        if line.strip().lower() == "stop" and not stop_sent:
            print("Stop requested — sending stop_scan()...")
            profiler.stop()
            stop_sent = True

    scan_thread.join()

    if "error" in scan_result:
        raise scan_result["error"]
    return scan_result["value"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--axis", choices=sorted(AXES), required=True)
    parser.add_argument("--distance-mm", type=float, required=True,
                         help="relative scan distance in real mm from wherever the axis currently is")
    parser.add_argument("--rate-mm-s", type=float, required=True, help="scan speed in real mm/s")
    parser.add_argument("--sensor", choices=sorted(SENSOR_DEFAULTS), default="od2000")
    parser.add_argument("--pdin-port", type=int, default=None,
                         help="defaults per-sensor: od2000=2, wtt12l_powerprox=7 — "
                              "confirm on your own hardware, see docs/MQTT_AL1342_SETUP.md "
                              "and docs/WTT12L_POWERPROX_SETUP.md")
    parser.add_argument("--calibration", default=None,
                         help="CSV from scripts/calibrate_rangefinder.py --device <same as --sensor> — "
                              "adds a real_height_mm column if given")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--plot", action="store_true", default=True)
    parser.add_argument("--no-plot", dest="plot", action="store_false")
    args = parser.parse_args()

    sensor_defaults = SENSOR_DEFAULTS[args.sensor]
    pdin_port = args.pdin_port if args.pdin_port is not None else sensor_defaults["pdin_port"]
    raw_column = sensor_defaults["raw_column"]

    calibration = None
    if args.calibration:
        calibration = LinearCalibration.from_csv(args.calibration)
        print(f"Loaded calibration '{calibration.device}': "
              f"real_height_mm = {calibration.slope:.6f} * {raw_column} + {calibration.intercept:.6f} "
              f"(r_squared={calibration.r_squared:.4f})")
        if calibration.device != args.sensor:
            print(f"warning: calibration was fitted for '{calibration.device}', "
                  f"applying it to '{args.sensor}' readings")

    axis = AXES[args.axis]
    conn = PiGantryConnection(
        host=PI_HOST,
        ssh_user=PI_USER,
        ssh_key=PI_KEY,
        remote_serial_device=REMOTE_SERIAL_DEVICE,
        remote_baud=9600,
        safe_mode=False,  # required: scan_start is gated like any motion command
    )

    print(f"Connecting to gantry agent on {PI_HOST}...")
    conn.connect()

    try:
        result = run_scan(conn, axis, args.distance_mm, args.rate_mm_s, args.output_dir,
                           args.sensor, pdin_port)

        print()
        print("=== Scan complete ===")
        print(f"Samples collected : {result.metadata['samples']}")
        print(f"Achieved rate     : {result.metadata.get('achieved_rate_hz', 0):.1f} Hz")
        print(f"Start position    : {result.metadata['actual_start_mm']:.3f} mm")
        print(f"End position      : {result.metadata['actual_end_mm']:.3f} mm")
        print(f"Actual distance   : {result.metadata['actual_distance_mm']:.3f} mm")
        print(f"Raw CSV path      : {result.path}")

        if result.df is None:
            print("(no DataFrame loaded — pandas issue retrieving the CSV; "
                  "real-units post-processing skipped)")
            return

        real_df = add_real_world_columns(result.df, calibration, raw_column=raw_column)
        real_path = Path(str(result.path).replace(".csv", "_real_units.csv"))
        real_df.to_csv(real_path, index=False)
        print(f"Real-units CSV    : {real_path}")
        if calibration is None:
            print("  (real_pos_mm only — pass --calibration for real_height_mm too)")

        y_col = "real_height_mm" if calibration is not None else "distance_mm"
        y_label = "Calibrated height (mm)" if calibration is not None else f"{args.sensor} raw distance (mm)"

        if args.plot:
            try:
                import matplotlib.pyplot as plt

                slew = real_df[real_df["in_ramp"] == 0]
                ramp = real_df[real_df["in_ramp"] == 1]

                fig, ax = plt.subplots(figsize=(12, 4))
                ax.plot(slew["real_pos_mm"], slew[y_col], linewidth=0.8, label="slew phase")
                ax.plot(ramp["real_pos_mm"], ramp[y_col], ".", alpha=0.3, markersize=3,
                        label="ramp phase (excluded)")
                ax.set_xlabel(f"Gantry {args.axis} position (mm)")
                ax.set_ylabel(y_label)
                ax.set_title(f"Line scan — {args.axis} axis")
                ax.legend()
                plt.tight_layout()
                plot_path = str(real_path).replace(".csv", ".png")
                plt.savefig(plot_path)
                print(f"Plot saved        : {plot_path}")
                plt.show()
            except ImportError:
                print("(matplotlib not installed — skipping plot; CSVs are still available above)")

    finally:
        conn.disconnect()
        print()
        print("Disconnected.")


if __name__ == "__main__":
    main()
