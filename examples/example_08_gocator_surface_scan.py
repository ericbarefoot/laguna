"""
Example 8: Gocator 2690 3D surface scan over a moving gantry axis

The shortest useful path through the scanner subsystem: configure the
sensor, run one coordinated gantry pass, and save the result.

For the full-control version — active area, subsampling, filters, output
formats, mounting overrides — use `scripts/gocator_scan.py`, which is the
CLI this example was extracted from.

--- How it works ---

There is no encoder in this setup. The sensor builds a 3D surface by
stacking laser-line profiles, and it works out how far apart they are from
the *travel speed we tell it* — so the sensor's travel speed and the
gantry's feed rate must be the same number, or the scan is stretched or
squashed along travel. scan_with_gantry() keeps them in sync for you from
`feed_rate_mm_s`, and also sizes the sensor's capture window from `end_mm`.

The pass is: start a non-blocking move, wait out the acceleration ramp
(`settle_s`), fire the software trigger, receive one surface.

--- Background you need before setting ALLOW_MOTION = True ---

1. Build the GoSdk shared libraries once (the vendor ships none for
   x86_64):
       sudo apt install build-essential
       scripts/build_gosdk.sh

2. LAS/LAZ output needs the optional extra: pip install 'laguna[scanner]'

3. The gantry needs `transport: pi_agent` (the default) and safe_mode
   off. This script derives safe_mode from ALLOW_MOTION below, the same
   pattern as example_05 and example_07 — the config file's value is not
   trusted, so flipping one flag here is the only way to enable motion.

4. scan_with_gantry() drives the axis through its AxisHandle rather than
   gantry.move_to(), because the trigger has to fire *while* the axis is
   moving. That path is still safe_mode-gated, but it does NOT fence-check
   the target the way move_to() does — make sure `END_MM` is inside the
   work envelope.

5. Sensor axes are not gantry axes. The Gocator calls X "across the laser
   line" and Y "travel"; on this rig it is mounted rotated 90 degrees, so
   a gantry move along X arrives as the sensor's Y. Set `gocator.mounting`
   in config to get scans back in gantry coordinates — see
   docs/subsystems/scanner.md.
"""

from laguna import FlumeLab
from laguna.robot.macron import GantryController
from laguna.scanner import GocatorScanner

# --- Scan geometry — adjust for your setup ---
SCAN_AXIS = "X"          # gantry axis to travel along
END_MM = 200.0           # absolute target; also sizes the sensor's capture window
FEED_RATE_MM_S = 20.0    # must be a speed the gantry holds steadily

# --- Motion is off by default. Flip this to True only when you've decided
#     to actually move something. ---
ALLOW_MOTION = False


def main():
    lab = FlumeLab("config/example_config.yaml")

    gocator_config = lab.config.get_value("gocator")
    if not gocator_config:
        print("No 'gocator:' section in the config — nothing to do.")
        return

    scanner = GocatorScanner.from_config(gocator_config)
    lab.add(scanner)

    if not scanner.connect():
        print("Could not connect to the Gocator. Check that the SDK libraries")
        print("are built (scripts/build_gosdk.sh) and the sensor answers at")
        print(f"{gocator_config.get('ip')} — try pinging it.")
        return

    # Read-only: what the sensor currently thinks it's doing.
    print("Sensor status:")
    for key, value in scanner.get_status().items():
        print(f"  {key}: {value}")

    if not ALLOW_MOTION:
        print()
        print("ALLOW_MOTION is False — no configuration written, nothing moved.")
        print("Set ALLOW_MOTION = True at the top of this file to run the scan.")
        lab.disconnect_all()
        return

    # ------------------------------------------------------------------
    # The scan
    # ------------------------------------------------------------------

    gantry_config = lab.config.get("gantry")
    gantry_config["safe_mode"] = False      # this script owns the decision
    gantry = GantryController.from_config(gantry_config)
    lab.add(gantry)

    if not gantry.connect():
        print("Could not connect to the gantry — see the errors above.")
        lab.disconnect_all()
        return

    try:
        print()
        print("Applying the encoderless scan recipe to the sensor...")
        print(f"  {scanner.configure()}")

        # What feed rate makes sense here? Y spacing = feed_rate / frame_rate,
        # and this solves that relation against the sensor's live ceiling.
        rates = scanner.solve_scan_rates(feed_rate_mm_s=FEED_RATE_MM_S)
        print(f"  at {FEED_RATE_MM_S} mm/s -> {rates['frame_rate_hz']:.1f} Hz, "
              f"Y spacing {rates['y_spacing_mm']:.4f} mm "
              f"(X resolution {rates['x_resolution_mm']:.4f} mm)")

        print()
        print(f"Scanning along {SCAN_AXIS} to {END_MM} mm at {FEED_RATE_MM_S} mm/s...")
        scan = scanner.scan_with_gantry(
            gantry,
            axis=SCAN_AXIS,
            end_mm=END_MM,
            feed_rate_mm_s=FEED_RATE_MM_S,
        )

        rows, cols = scan.shape
        print(f"  got a {rows} x {cols} surface, "
              f"{scan.valid_count:,} points with a laser return")
        print(f"  grid rows run along gantry {scan.grid_axes[0]}, "
              f"columns along gantry {scan.grid_axes[1]}")

        written = scanner.save_scan(scan)
        for fmt, path in written.items():
            print(f"  wrote {fmt}: {path}")

        print()
        print("Plot it with:  python scripts/visualize_scan.py")
    finally:
        lab.disconnect_all()


if __name__ == "__main__":
    main()
