"""List attached Canon DSLRs and give each a stable /dev symlink via udev.

Prints every camera gphoto2 can see with its EOS body serial (what goes in
the config's ``dslr_cameras.<name>.serial``), its current gphoto2 port, and
its physical USB path. With ``--config``, cameras are named by matching
their serials against that file's ``dslr_cameras`` section, and the udev
rules file is printed (or installed with ``--install``, as root).

The symlink is only a hint for DslrCameraSubsystem — cameras are always
bound by serial — but it makes ``ls -l /dev/dslr_*`` show at a glance which
camera is plugged in where. It follows the hub port, not the camera, so
rerun this after moving a cable.

Wake the cameras first (half-press the shutter): an asleep T7 drops off USB.

Usage:
    python scripts/setup_dslr_udev.py
    python scripts/setup_dslr_udev.py --config config/my_run.yaml
    sudo "$(which python)" scripts/setup_dslr_udev.py --config config/my_run.yaml --install
"""

import argparse
import subprocess
import sys
from pathlib import Path

import yaml

from laguna.camera.canon import discover_cameras, udev_rule
from laguna.camera.gvfs import release_gphoto_usb

RULES_FILE = Path("/etc/udev/rules.d/80-laguna-dslr.rules")


def main() -> int:
    """Run the CLI."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--config", help="experiment YAML whose dslr_cameras serials name the cameras")
    parser.add_argument("--install", action="store_true",
                        help=f"write {RULES_FILE} and reload udev (needs root)")
    args = parser.parse_args()

    names = {}
    if args.config:
        section = (yaml.safe_load(Path(args.config).read_text()) or {}).get("dslr_cameras") or {}
        names = {str(c.get("serial")): n for n, c in (section.get("cameras") or {}).items()}

    release_gphoto_usb()
    cameras = discover_cameras()
    if not cameras:
        print("No cameras detected. Are they on and awake (half-press the shutter)?")
        return 1

    rules = []
    for cam in cameras:
        name = names.get(str(cam.serial))
        print(f"{cam.model}\n  serial:   {cam.serial}\n  port:     {cam.port}"
              f"\n  usb path: {cam.usb_path}\n  config:   {name or '(not in config)'}")
        if name and cam.usb_path:
            rules.append(udev_rule(f"dslr_{name.lower()}", cam.usb_path))
    missing = set(names.values()) - {names.get(str(c.serial)) for c in cameras}
    if missing:
        print(f"\nConfigured but not attached: {', '.join(sorted(missing))}")
    if not rules:
        return 0

    text = "# Managed by laguna/scripts/setup_dslr_udev.py — do not edit by hand.\n" + "\n".join(rules) + "\n"
    print(f"\n{RULES_FILE}:\n{text}")
    if args.install:
        RULES_FILE.write_text(text)
        subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
        subprocess.run(["udevadm", "trigger", "--subsystem-match=usb"], check=True)
        subprocess.run(["udevadm", "settle"], check=True)
        print(f"Wrote {RULES_FILE} and reloaded udev.")
        subprocess.run("ls -l /dev/dslr_*", shell=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
