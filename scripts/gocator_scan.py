#!/usr/bin/env python3
"""Gocator 2690 surface-scan and sensor-tuning CLI.

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

This is the full-control utility: every sensor knob, plus the coordinated
gantry pass. For a short, readable introduction to the same subsystem see
examples/example_08_gocator_surface_scan.py.

Run:
    # read-only: print sensor state and the live frame-rate ceiling
    python scripts/gocator_scan.py --dry-run

    # tune the active area and see what it buys, without moving anything
    python scripts/gocator_scan.py --dry-run --configure \
        --active-area z=-10,height=200 --x-subsampling 4

    # an actual scan
    python scripts/gocator_scan.py --axis X --end-mm 400 --allow-motion

    # true point cloud (no X resampling), saved as LAZ
    python scripts/gocator_scan.py --axis X --end-mm 400 \
        --point-cloud --formats npz,laz --allow-motion

LAS/LAZ output needs the optional scanner extra: pip install 'laguna[scanner]'

Safety: matches the ALLOW_MOTION pattern in example_05/example_07 — motion is
opt-in, not opt-out. Without --allow-motion this always behaves like
--dry-run, regardless of what the config file's gantry.safe_mode says; this
script derives safe_mode from --allow-motion itself, via set_safe_mode(),
rather than trusting a possibly-stale config value. Pass --allow-motion only
once you've decided to actually move something.

The --allow-motion path also uses the per-axis command path (AxisHandle,
same safe_mode-gated BMT that gantry.move_to() itself uses), which bypasses
the fence checking that gantry.move_to() performs — make sure the target is
inside the work envelope before running.
"""

from __future__ import annotations

import argparse
import logging
from typing import Optional

import numpy as np

from laguna import FlumeLab
from laguna.scanner import FILTER_NAMES
from laguna.scanner.mounting import SENSOR_AXES, SensorMounting

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("gocator_scan")


def parse_active_area(spec: Optional[str]) -> Optional[dict]:
    """Parse ``--active-area z=400,height=200`` into a kwargs dict, in mm.

    Returns None for None, so callers can pass it straight through to
    configure() and get the config-file/sensor fallback.
    """
    if spec is None:
        return None
    fields = ("x", "y", "z", "width", "length", "height")
    out = {}
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        key, sep, value = part.partition("=")
        key = key.strip().lower()
        if not sep or key not in fields:
            raise ValueError(
                f"--active-area: expected FIELD=MM with FIELD in {fields}, got {part!r}"
            )
        try:
            out[key] = float(value)
        except ValueError:
            raise ValueError(f"--active-area: {value!r} is not a number") from None
    if not out:
        raise ValueError("--active-area was given but empty")
    return out


def parse_filters(specs: Optional[list]) -> Optional[dict]:
    """Parse repeated ``--filter name[=mm|=off]`` into a set_filters() dict."""
    if not specs:
        return None
    out: dict = {}
    for spec in specs:
        name, sep, value = spec.partition("=")
        name = name.strip().lower()
        if name not in FILTER_NAMES:
            raise ValueError(
                f"--filter: unknown filter {name!r}; expected one of "
                f"{', '.join(sorted(FILTER_NAMES))}"
            )
        if not sep or value.strip().lower() in ("", "on", "true"):
            out[name] = True
        elif value.strip().lower() in ("off", "false"):
            out[name] = False
        else:
            try:
                out[name] = float(value)
            except ValueError:
                raise ValueError(
                    f"--filter {name}: expected a window in mm, 'on' or 'off', "
                    f"got {value!r}"
                ) from None
    return out


def parse_spacing_interval(spec: Optional[str]) -> Optional[dict]:
    """Parse ``--spacing-interval`` into a set_spacing_interval() dict."""
    if spec is None:
        return None
    spec = spec.strip().lower()
    if spec in ("max_res", "balanced", "max_speed", "custom"):
        return {"type": spec}
    try:
        return {"type": "custom", "value_mm": float(spec)}
    except ValueError:
        raise ValueError(
            f"--spacing-interval: expected max_res/balanced/max_speed or a "
            f"number in mm, got {spec!r}"
        ) from None


def parse_mounting(spec: Optional[str]) -> Optional[dict]:
    """Parse ``--mounting scan_x=-Y,scan_y=+X,scan_z=+Z`` into a config dict."""
    if spec is None:
        return None
    out: dict = {}
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        key, sep, value = part.partition("=")
        key = key.strip().lower()
        if not sep or key not in SENSOR_AXES:
            raise ValueError(
                f"--mounting: expected KEY=AXIS with KEY in {list(SENSOR_AXES)}, "
                f"got {part!r}"
            )
        out[key] = value.strip()
    if not out:
        raise ValueError("--mounting was given but empty")
    # Build it now so a bad/mirroring map is rejected before we connect.
    SensorMounting.from_config(out)
    return out


def force_pi_agent_transport(lab: FlumeLab) -> dict:
    """Force the gantry config's transport to pi_agent before construction.

    transport has to be set before GantryController is built — there's no
    post-construction way to change it, unlike safe_mode (see
    GantryController.set_safe_mode(), called separately after lab.add()).

    Same reasoning as example_05/example_07: pi_agent is the only transport
    that supports topographic scanning and the only one with a working
    default. The retired socket_bridge transport depended on
    serial_bridge.py, a hand-started, unversioned script on the Pi that did
    not survive a reboot — see docs/MACRON_GANTRY.md, "Retired:
    serial_bridge.py". Its code path still exists if configured explicitly,
    but this script shouldn't hand a stale config value through to it.
    """
    gantry_config = lab.config.get("gantry")
    gantry_config["transport"] = "pi_agent"
    return gantry_config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/example_config.yaml")
    parser.add_argument("--axis", default="X", help="Gantry axis to scan along")
    parser.add_argument(
        "--end-mm",
        type=float,
        default=200.0,
        help="Absolute target, mm. Also determines the sensor's "
        "fixed_length_mm surface-generation setting: scan_with_gantry() "
        "derives it from abs(end_mm - the axis's current position) so the "
        "sensor's capture window matches the commanded move, overriding "
        "config/example_config.yaml's gocator.fixed_length_mm rather than "
        "requiring it be kept in sync by hand.",
    )
    parser.add_argument(
        "--feed-rate", type=float, default=20.0, help="Scan speed, mm/s"
    )
    frame_rate_group = parser.add_mutually_exclusive_group()
    frame_rate_group.add_argument(
        "--frame-rate-hz",
        type=float,
        default=None,
        help="Sensor profile trigger rate, Hz. Omit to use the config file's "
        "gocator.frame_rate_hz (or whatever frame-rate mode/rate the sensor "
        "already has, if that's also unset — see --frame-rate-max to "
        "request the sensor's current maximum explicitly instead). Y "
        "spacing = travel_speed / frame_rate — see docs/subsystems/scanner.md "
        "for this unit's measured ceiling; configure() rejects a rate the "
        "sensor can't deliver. Mutually exclusive with --frame-rate-max.",
    )
    frame_rate_group.add_argument(
        "--frame-rate-max",
        action="store_true",
        default=None,
        help="Explicitly (re-)enable max-frame-rate mode and use whatever "
        "rate the sensor reports after flushing, regardless of prior state "
        "— unlike omitting --frame-rate-hz, this works even if a previous "
        "run configured an explicit rate (which leaves max-frame-rate mode "
        "disabled in sensor flash). The achievable max is dynamic — depends "
        "on FOV/exposure/uniform spacing — so this is read back live, not "
        "assumed. Mutually exclusive with --frame-rate-hz.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.5,
        help="Delay after commanding motion before triggering, to clear the accel ramp",
    )
    spacing_group = parser.add_mutually_exclusive_group()
    spacing_group.add_argument(
        "--point-cloud",
        dest="uniform_spacing",
        action="store_false",
        default=None,
        help="Disable the sensor's X resampling, so it returns a true point "
        "cloud (SURFACE_POINT_CLOUD: an explicit x/y/z per point at native, "
        "non-uniform X spacing) instead of a resampled UNIFORM_SURFACE "
        "heightmap. Also raises the achievable frame-rate ceiling. Omit both "
        "this and --uniform-spacing to leave the sensor's current setting.",
    )
    spacing_group.add_argument(
        "--uniform-spacing",
        dest="uniform_spacing",
        action="store_true",
        default=None,
        help="Force the sensor's X resampling ON — a UNIFORM_SURFACE "
        "heightmap. The inverse of --point-cloud.",
    )
    parser.add_argument(
        "--active-area",
        default=None,
        metavar="FIELD=MM,...",
        help="Restrict the sensor's region of interest, e.g. "
        "'z=400,height=200,width=600'. Fields: x,y,z (origin) and "
        "width,length,height (extents), all mm. THE main lever for scan "
        "speed — a smaller area (above all in Z) means fewer camera rows per "
        "profile and so a higher frame-rate ceiling. Anything outside it is "
        "not measured, so leave margin for the tallest feature and any Z "
        "wander. Omit to use the config's active_area, or leave the sensor's "
        "own. Check the result with --dry-run, which prints the live area "
        "and the resulting sensor_frame_rate_max_hz.",
    )
    parser.add_argument(
        "--x-subsampling",
        type=int,
        default=None,
        choices=(1, 2, 4),
        help="X resolution divider: 1 full, 2 half, 4 quarter. The cheapest "
        "big speed win — measured on this 2690, x=2 and x=4 multiply the "
        "frame-rate ceiling by exactly 2.00x and 4.00x, in BOTH uniform and "
        "point-cloud modes, at the cost of X resolution (0.124mm -> 0.248 -> "
        "0.496). Unlike filters, this is NOT restricted to uniform spacing.",
    )
    parser.add_argument(
        "--z-subsampling",
        type=int,
        default=None,
        choices=(1, 2, 4, 8),
        help="Z resolution divider. Measured to have NO effect on frame rate "
        "on this unit (ratio 1.000 across every active-area height), so it "
        "trades Z resolution for nothing — leave it alone without a reason.",
    )
    parser.add_argument(
        "--spacing-interval",
        default=None,
        metavar="TYPE|MM",
        help="X resampling bin size: one of max_res, balanced, max_speed, or "
        "a number in mm (implies custom). UNIFORM SPACING ONLY — rejected "
        "with --point-cloud.",
    )
    parser.add_argument(
        "--filter",
        action="append",
        default=None,
        metavar="NAME[=MM|=off]",
        help="Enable a post-processing filter, repeatable. NAME is one of "
        "x_smoothing, x_median, x_decimation, x_gap_filling, and the y_ "
        "equivalents. 'name' enables with the current window, 'name=1.5' "
        "enables with a 1.5mm window, 'name=off' disables. UNIFORM SPACING "
        "ONLY — these run on the resampled X grid, so they are rejected with "
        "--point-cloud.",
    )
    parser.add_argument(
        "--mounting",
        default=None,
        metavar="scan_x=-Y,scan_y=+X,scan_z=+Z",
        help="How the sensor's own axes sit on the gantry. The sensor calls X "
        "'across the laser' and Y 'travel' — NOT the gantry's axes. On this rig "
        "the sensor is rotated 90 deg, so gantry X motion is the sensor's Y. "
        "Setting this makes scans and every export come back in GANTRY "
        "coordinates. Overrides the config's gocator.mounting. A mirroring map "
        "(bare swap with no sign flip) is rejected.",
    )
    parser.add_argument(
        "--formats",
        default="npz,laz",
        help="Comma-separated output formats (default: npz,laz). npz keeps "
        "the full grid including no-data cells and is the right choice for "
        "reprocessing; laz/las is an ASPRS point cloud (LAZ is by far the "
        "fastest and smallest — ~0.5s/14MB vs csv's ~55s/1.2GB on a 24M-point "
        "scan); ply for CloudCompare/MeshLab; csv only if something "
        "downstream truly needs text.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Connect and print sensor status, then exit. Read-only: does not "
        "move the gantry and does not write any sensor settings.",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="Required to actually move the gantry and scan — same ALLOW_MOTION "
        "idiom as example_05/example_07. Without it, this script always behaves "
        "like --dry-run no matter what other flags are given, and the gantry is "
        "connected with safe_mode=True regardless of the config file's value.",
    )
    parser.add_argument(
        "--configure",
        action="store_true",
        help="Apply the scan configuration to the sensor without scanning. "
        "Combine with --dry-run to set up the sensor and stop. Note this "
        "writes travel speed to sensor flash when it changes.",
    )
    args = parser.parse_args()

    # Validate here rather than at the configure() call site further down:
    # that call is skipped entirely on the dry-run path, so a typo like
    # --active-area bogus=1 would otherwise be accepted in silence.
    try:
        active_area = parse_active_area(args.active_area)
        filters = parse_filters(args.filter)
        mounting = parse_mounting(args.mounting)
        spacing_interval = parse_spacing_interval(args.spacing_interval)
    except ValueError as exc:
        parser.error(str(exc))

    # Fail here rather than after connecting: filters and the spacing
    # interval act on the resampled X grid, which --point-cloud switches off.
    if args.uniform_spacing is False:
        conflicting = [n for n, v in (("--filter", filters),
                                      ("--spacing-interval", spacing_interval)) if v]
        if conflicting:
            parser.error(
                f"{' and '.join(conflicting)} cannot be used with --point-cloud: "
                "these run on the resampled X grid, which point-cloud mode "
                "disables. (--x-subsampling works in both modes.)"
            )

    subsampling = {k: v for k, v in (("x", args.x_subsampling),
                                     ("z", args.z_subsampling)) if v is not None} or None

    lab = FlumeLab(args.config)

    if "gocator" not in lab.config.explicit_sections:
        logger.error(
            "No 'gocator:' section in %s — copy the commented block from "
            "config/example_config.yaml",
            args.config,
        )
        return 1
    gocator_config = lab.config.get("gocator")

    # --mounting overrides the config's, so scans come back in gantry
    # coordinates without editing the config file.
    if mounting is not None:
        gocator_config["mounting"] = mounting

    lab.add("gocator")
    scanner = lab.gocator

    if not scanner.connect():
        logger.error(
            "Could not connect to the Gocator. Check that:\n"
            "  - the SDK libs are built (scripts/build_gosdk.sh)\n"
            "  - the sensor answers at %s (try: ping %s)",
            gocator_config.get("ip"),
            gocator_config.get("ip"),
        )
        return 1

    # --allow-motion is the authoritative gate, same ALLOW_MOTION idiom as
    # example_05/example_07: without it, this always behaves like --dry-run,
    # regardless of --dry-run's own value or what the config file says.
    effective_dry_run = args.dry_run or not args.allow_motion
    if not args.allow_motion and not args.dry_run:
        logger.info(
            "--allow-motion not given — running as if --dry-run. Pass "
            "--allow-motion once you've decided to actually move the gantry."
        )

    try:
        if args.configure or not effective_dry_run:
            applied = scanner.configure(
                frame_rate_hz=args.frame_rate_hz,
                frame_rate_max=args.frame_rate_max,
                uniform_spacing=args.uniform_spacing,
                active_area=active_area,
                subsampling=subsampling,
                spacing_interval=spacing_interval,
                filters=filters,
            )
            logger.info("Sensor configured: %s", applied)
        else:
            logger.info(
                "Dry run without --configure: reading sensor state only, "
                "not writing any settings."
            )

        logger.info("Status: %s", scanner.get_status())

        if effective_dry_run:
            logger.info("Not moving the gantry or scanning.")
            return 0

        gantry_config = force_pi_agent_transport(lab)

        lab.add("gantry")
        gantry = lab.gantry
        # This script owns the motion decision, not the config file — safe
        # to call before connect() (see set_safe_mode()'s docstring).
        gantry.set_safe_mode(not args.allow_motion)
        if not gantry.connect():
            # pi_agent (forced above) SFTPs and launches gantry_agent.py over
            # SSH itself, then owns the serial port for the session — no
            # manual Pi-side step, and no separate bridge process to check.
            # PiGantryConnection.connect() already warns if something else
            # is holding the port (a stale agent from an interrupted
            # session, or a tio terminal) before it gets here, so a failure
            # at this point is almost always SSH reachability. See
            # docs/MACRON_GANTRY.md.
            logger.error(
                "Could not connect to the gantry via pi_agent. Check SSH "
                "reachability of %s as user %r, and any stale-port warning "
                "logged just above. To check by hand: ssh %s@%s 'fuser %s'",
                gantry_config.get("host"),
                gantry_config.get("ssh_user", "oak"),
                gantry_config.get("ssh_user", "oak"),
                gantry_config.get("host"),
                gantry_config.get("remote_serial_device", "<remote_serial_device>"),
            )
            return 1

        # Sanity check, not the primary gate — set_safe_mode(False) was
        # already called from --allow-motion above. Fail fast rather than
        # partway through a move if that didn't take for some reason, since
        # safe_mode gates every motion command.
        if gantry.get_status().get("safe_mode", True):
            logger.error(
                "Gantry safe_mode is enabled — motion commands are blocked, so "
                "the scan pass cannot run, despite --allow-motion. Check "
                "GantryController.connect()/set_safe_mode() and the gantry "
                "config for something overriding safe_mode back to True."
            )
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

        # Read bounds straight off the grids. Flattening to an (N, 3) point
        # array first would build (and immediately discard) hundreds of MB
        # purely to print six numbers; nanmin/nanmax handles both the
        # uniform (1-D x/y) and point-cloud (2-D x/y) layouts.
        if scan.valid_count:
            logger.info(
                "Bounds: x [%.2f, %.2f]  y [%.2f, %.2f]  z [%.2f, %.2f] mm",
                np.nanmin(scan.x_mm), np.nanmax(scan.x_mm),
                np.nanmin(scan.y_mm), np.nanmax(scan.y_mm),
                np.nanmin(scan.z_mm), np.nanmax(scan.z_mm),
            )

        formats = tuple(f.strip() for f in args.formats.split(",") if f.strip())
        written = scanner.save_scan(scan, formats=formats)
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
