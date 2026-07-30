"""Example 08 — Gocator 2690 3D surface scan over a moving gantry axis.

Captures a 3D point cloud by moving the gantry at constant velocity under the
Gocator's laser line and bracketing the pass with a software trigger. There is
no encoder in this setup, so the sensor scales the travel (Y) axis from the
travel speed we configure — which is why the sensor's travel speed and the
gantry's feed rate must be the same number.

Prerequisites:
  1. Build the GoSdk shared libraries (one-time):
         sudo apt install build-essential      # if not already present
         scripts/build_gosdk.sh
  2. A `gocator:` section in your config (see config/example_config.yaml).
  3. The sensor reachable at its configured IP (`ping 192.168.1.10`).

Run:
    python examples/example_08_gocator_surface_scan.py --dry-run     # no motion
    python examples/example_08_gocator_surface_scan.py --axis X --end-mm 400

Safety: the non-dry-run path commands real gantry motion. It uses the per-axis
command path, which bypasses the fence checking that gantry.move_to() performs
— make sure the target is inside the work envelope before running.
"""

from __future__ import annotations

import argparse
import logging

from laguna import FlumeLab
from laguna.robot.macron import GantryController
from laguna.scanner import GocatorScanner

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("example_08")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/example_config.yaml")
    parser.add_argument("--axis", default="X", help="Gantry axis to scan along")
    parser.add_argument("--end-mm", type=float, default=200.0, help="Absolute target, mm")
    parser.add_argument(
        "--feed-rate", type=float, default=20.0, help="Scan speed, mm/s"
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.5,
        help="Delay after commanding motion before triggering, to clear the accel ramp",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Connect and print sensor status, then exit. Read-only: does not "
        "move the gantry and does not write any sensor settings.",
    )
    parser.add_argument(
        "--configure",
        action="store_true",
        help="Apply the scan configuration to the sensor without scanning. "
        "Combine with --dry-run to set up the sensor and stop. Note this "
        "writes travel speed to sensor flash when it changes.",
    )
    args = parser.parse_args()

    lab = FlumeLab(args.config)

    gocator_config = lab.config.get_value("gocator")
    if not gocator_config:
        logger.error(
            "No 'gocator:' section in %s — copy the commented block from "
            "config/example_config.yaml",
            args.config,
        )
        return 1

    scanner = GocatorScanner.from_config(gocator_config)
    lab.add(scanner)

    if not scanner.connect():
        logger.error(
            "Could not connect to the Gocator. Check that:\n"
            "  - the SDK libs are built (scripts/build_gosdk.sh)\n"
            "  - the sensor answers at %s (try: ping %s)",
            gocator_config.get("ip"),
            gocator_config.get("ip"),
        )
        return 1

    try:
        if args.configure or not args.dry_run:
            applied = scanner.configure()
            logger.info("Sensor configured: %s", applied)
        else:
            logger.info(
                "--dry-run without --configure: reading sensor state only, "
                "not writing any settings."
            )

        logger.info("Status: %s", scanner.get_status())

        if args.dry_run:
            logger.info("--dry-run given; not moving the gantry or scanning.")
            return 0

        gantry = GantryController.from_config(lab.config.get("gantry"))
        lab.add(gantry)
        if not gantry.connect():
            logger.error("Could not connect to the gantry.")
            return 1

        logger.info(
            "Scanning along %s to %.1f mm at %.1f mm/s",
            args.axis,
            args.end_mm,
            args.feed_rate,
        )
        scan = scanner.scan_with_gantry(
            gantry,
            axis=args.axis,
            end_mm=args.end_mm,
            feed_rate_mm_s=args.feed_rate,
            settle_s=args.settle_s,
        )

        rows, cols = scan.shape
        total = rows * cols
        logger.info(
            "Surface: %d x %d grid (%d cells), %d valid points (%.1f%% return)",
            rows,
            cols,
            total,
            scan.valid_count,
            100.0 * scan.valid_count / total if total else 0.0,
        )
        logger.info(
            "X spacing %.4f mm, Y spacing %.4f mm",
            scan.metadata.get("x_spacing_mm", float("nan")),
            scan.metadata.get("y_spacing_mm", float("nan")),
        )

        points = scan.to_points()
        if len(points):
            logger.info(
                "Bounds: x [%.2f, %.2f]  y [%.2f, %.2f]  z [%.2f, %.2f] mm",
                points[:, 0].min(), points[:, 0].max(),
                points[:, 1].min(), points[:, 1].max(),
                points[:, 2].min(), points[:, 2].max(),
            )

        written = scanner.save_scan(scan, formats=("npz", "ply", "csv"))
        for fmt, path in written.items():
            logger.info("Wrote %s: %s", fmt, path)

        # Sanity check the encoderless Y scaling: the surface's Y extent should
        # be close to the distance the gantry actually travelled during the
        # triggered window. A large mismatch means travel speed and the real
        # feed rate disagree — see SurfaceScan.rescale_y().
        y_extent = float(scan.y_mm.max() - scan.y_mm.min())
        logger.info(
            "Y extent %.2f mm (fixed_length_mm was %s). If these disagree "
            "badly, check the travel speed / feed rate match.",
            y_extent,
            scan.metadata.get("fixed_length_mm"),
        )
        return 0
    finally:
        lab.disconnect_all()


if __name__ == "__main__":
    raise SystemExit(main())
