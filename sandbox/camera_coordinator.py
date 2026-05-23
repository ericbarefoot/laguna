#!/usr/bin/env python3
"""Camera coordinator — sandbox entry point.

All logic now lives in laguna.camera.network.  This script re-exports
the package classes so that existing sandbox invocations still work, and
provides the same CLI as before.

Usage:
    python3 camera_coordinator.py
    python3 camera_coordinator.py --cameras antares.laguna sirius.laguna
    python3 camera_coordinator.py --lead-time 5.0 --fetch-images --output-dir ./captures
    python3 camera_coordinator.py --ssh-key ~/.ssh/id_rsa --check-sync
"""

import sys
from pathlib import Path

# Allow running directly from the sandbox directory without installing the package.
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from laguna.camera.network import (  # noqa: F401 — re-exported for backwards compatibility
    CameraArray,
    CaptureResult,
    DEFAULT_CAMERAS,
    DEFAULT_LEAD_TIME,
    DEFAULT_SSH_USER,
    _resolve_passphrase,
)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Trigger simultaneous capture on Pi cameras.")
    parser.add_argument("--cameras", nargs="+", default=DEFAULT_CAMERAS)
    parser.add_argument("--ssh-user", default=DEFAULT_SSH_USER)
    parser.add_argument("--ssh-key", default=None, help="Path to SSH private key")
    parser.add_argument("--ssh-passphrase", default=None, help="Passphrase for encrypted SSH key")
    parser.add_argument("--lead-time", type=float, default=DEFAULT_LEAD_TIME)
    parser.add_argument("--fetch-images", action="store_true")
    parser.add_argument("--output-dir", default="./captures")
    parser.add_argument("--check-sync", action="store_true", help="Check NTP clock sync before capture")
    args = parser.parse_args()

    passphrase = _resolve_passphrase(args.ssh_passphrase)

    array = CameraArray(
        hosts=args.cameras,
        ssh_user=args.ssh_user,
        ssh_key=args.ssh_key,
        ssh_passphrase=passphrase,
    )

    if args.check_sync:
        print("Checking clock synchronization...")
        offsets = array.check_clock_sync()
        for host, offset_s in offsets.items():
            print(f"  {host}: offset = {offset_s * 1000:.1f} ms")
        print()

    print(f"Triggering capture on: {args.cameras}")
    print(f"Lead time: {args.lead_time:.1f}s — target = now + {args.lead_time:.1f}s")

    results = array.trigger_capture(lead_time=args.lead_time)
    array.report_simultaneity(results)

    if args.fetch_images:
        output_dir = Path(args.output_dir)
        print(f"Fetching images to {output_dir} ...")
        local = array.fetch_images(results, output_dir)
        for host, path in local.items():
            print(f"  {host} -> {path}")
