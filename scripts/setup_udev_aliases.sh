#!/bin/sh
# Install udev rules that give the confluence node's serial devices
# memorable /dev symlinks (e.g. /dev/teknic_clearcore) instead of the raw
# /dev/serial/by-id/... names. Matches on the FTDI cable's serial number
# (ID_SERIAL_SHORT) plus, for the shared 4-port hub, its USB interface
# number (ID_USB_INTERFACE_NUM) — that's the same pair of attributes
# already used to build the distinct -if00.../-if01... by-id names, so it
# reliably tells the hub's ports apart. Mapping below was determined by
# probing each port with confluence's device protocols directly (see
# confluence/scripts/probe_serial_ports.py and probe_clearcore.py) —
# rerun that probing if the hub or cables are ever replaced:
#
#   AV0K9L0C            (single-port cable) -> macron gantry bridge
#   FT9DZBFK if00       -> Teknic_ClearCore (weir/flow)
#   FT9DZBFK if01       -> Massa_Ultrasonic (gauge)
#   FT9DZBFK if02       -> Fuji_Frenic_VFD (flow)
#   FT9DZBFK if03       -> unused/spare
#
# Usage: sudo ./setup_udev_aliases.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Run as root (sudo ./setup_udev_aliases.sh)" >&2
    exit 1
fi

RULES_FILE=/etc/udev/rules.d/99-confluence-serial.rules

cat > "$RULES_FILE" <<'EOF'
# Managed by laguna/scripts/setup_udev_aliases.sh — do not edit by hand.
SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="FT9DZBFK", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="teknic_clearcore"
SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="FT9DZBFK", ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="massa_ultrasonic"
SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="FT9DZBFK", ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="fuji_vfd"
SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="AV0K9L0C", SYMLINK+="macron_gantry"
EOF

echo "Wrote $RULES_FILE"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm settle  # trigger only queues events; wait for udevd to finish processing them

echo "Reloaded udev rules. Resulting aliases:"
ls -l /dev/teknic_clearcore /dev/massa_ultrasonic /dev/fuji_vfd /dev/macron_gantry 2>&1
