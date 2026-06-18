#!/usr/bin/env bash
# toggle-canon-gvfs.sh
#
# Toggles the udev rule that prevents GNOME from auto-mounting Canon cameras.
#
# WHY THIS EXISTS:
#   GNOME's gvfsd-gphoto2 claims Canon cameras over USB as soon as they're
#   plugged in, making them browsable in Nautilus but blocking direct gphoto2
#   access (dualcam-timelapse, laguna). The udev rule sets GVFS_IGNORE on all
#   Canon USB devices so gvfs never claims them.
#
# STATES:
#   BLOCKED  (rule present) — gphoto2/dualcam can claim cameras directly.
#                             Nautilus cannot browse camera storage.
#   ALLOWED  (rule absent)  — GNOME auto-mounts cameras; Nautilus works.
#                             gphoto2 will fail with "Could not claim USB device".
#
# USAGE:
#   sudo ./scripts/toggle-canon-gvfs.sh          # toggle current state
#   sudo ./scripts/toggle-canon-gvfs.sh status   # print current state only

set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-canon-no-gvfs.rules"
RULE_LINE='SUBSYSTEM=="usb", ATTRS{idVendor}=="04a9", ENV{GVFS_IGNORE}="1"'

print_status() {
    if [[ -f "$RULE_FILE" ]]; then
        echo "Canon GVFS:  BLOCKED  (gphoto2 access enabled, Nautilus browsing disabled)"
    else
        echo "Canon GVFS:  ALLOWED  (Nautilus browsing enabled, gphoto2 access blocked)"
    fi
}

if [[ "${1:-}" == "status" ]]; then
    print_status
    exit 0
fi

if [[ $EUID -ne 0 ]]; then
    echo "error: this script must be run as root (use sudo)" >&2
    exit 1
fi

if [[ -f "$RULE_FILE" ]]; then
    echo "Removing Canon GVFS block — re-enabling Nautilus file browsing..."
    rm "$RULE_FILE"
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=usb
    echo "Done. Replug cameras to restore GNOME auto-mount."
    echo ""
    print_status
else
    echo "Installing Canon GVFS block — enabling direct gphoto2 access..."
    echo "$RULE_LINE" > "$RULE_FILE"
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=usb
    echo "Done. Replug cameras — gphoto2 can now claim them directly."
    echo ""
    print_status
fi
