"""Release USB cameras from GNOME gvfs before libgphoto2 access.

Two helpers are provided and are typically called together:

    release_gphoto_usb()  # kill gvfsd-gphoto2 so it can't re-claim
    reset_usb_cameras()   # USB bus-reset each camera to clear stale PTP sessions
"""

from __future__ import annotations

import fcntl
import glob
import logging
import os
import signal
import subprocess
import time

logger = logging.getLogger(__name__)

GVFS_GPHOTO2_PROCESS = "gvfsd-gphoto2"
USBDEVFS_RESET = 0x5514          # _IO('U', 20) — Linux USB device reset ioctl
CANON_VENDOR_ID = "04a9"         # Canon Inc.


def release_gphoto_usb() -> int:
    """Stop gvfsd-gphoto2 processes owned by the current user.

    GNOME's gvfs helper often claims Canon cameras and blocks gphoto2 with
    "Could not claim the USB device". Returns the number of processes stopped.
    """
    uid = os.getuid()
    try:
        result = subprocess.run(
            ["pgrep", "-u", str(uid), "-f", GVFS_GPHOTO2_PROCESS],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        logger.warning("pgrep not found; skipping gvfs release")
        return 0

    pids = [pid.strip() for pid in result.stdout.splitlines() if pid.strip()]
    if not pids:
        logger.debug("No %s processes found for uid %s", GVFS_GPHOTO2_PROCESS, uid)
        return 0

    stopped = 0
    for pid in pids:
        try:
            os.kill(int(pid), signal.SIGTERM)
            stopped += 1
            logger.info("Stopped %s (pid %s)", GVFS_GPHOTO2_PROCESS, pid)
        except ProcessLookupError:
            pass
        except PermissionError as exc:
            logger.warning("Could not stop pid %s: %s", pid, exc)

    return stopped


def reset_usb_cameras(vendor_id: str = CANON_VENDOR_ID, settle_s: float = 5.0) -> list[str]:
    """Send a USB bus-reset to every camera from the given vendor.

    This is the heavier second-level recovery: it clears stale PTP sessions
    that survive a gvfs release (e.g. from a previous crashed connection).
    After reset the camera re-enumerates and gets a new device number, so
    callers should re-run gphoto2 auto-detect to find the new ports.

    Args:
        vendor_id:  USB vendor ID to match (default: Canon ``04a9``).
        settle_s:   Seconds to wait after reset for re-enumeration.

    Returns:
        List of device paths that were successfully reset.
    """
    reset_devs: list[str] = []

    id_paths = glob.glob(f"/sys/bus/usb/devices/*/idVendor")
    for id_path in id_paths:
        try:
            with open(id_path) as f:
                if f.read().strip() != vendor_id:
                    continue
        except OSError:
            continue

        # Resolve the sysfs device path to the /dev/bus/usb/BBB/DDD node
        device_dir = os.path.dirname(id_path)
        try:
            with open(os.path.join(device_dir, "busnum")) as f:
                bus = f.read().strip().zfill(3)
            with open(os.path.join(device_dir, "devnum")) as f:
                dev = f.read().strip().zfill(3)
        except OSError:
            continue

        dev_path = f"/dev/bus/usb/{bus}/{dev}"
        try:
            fd = os.open(dev_path, os.O_WRONLY)
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            os.close(fd)
            reset_devs.append(dev_path)
            logger.info("USB reset sent to %s", dev_path)
        except PermissionError as exc:
            logger.warning("Could not reset %s (permission): %s", dev_path, exc)
        except OSError as exc:
            logger.warning("Could not reset %s: %s", dev_path, exc)

    if reset_devs:
        logger.info("Waiting %.1fs for camera re-enumeration...", settle_s)
        time.sleep(settle_s)

    return reset_devs


def detect_camera_ports() -> list[str]:
    """Return gphoto2 USB port strings for all currently detected cameras.

    Example return value: ['usb:001,057', 'usb:001,058']

    Call this after reset_usb_cameras() to get the freshly assigned ports
    before reconnecting.
    """
    try:
        import gphoto2 as gp
    except ImportError:
        logger.error("gphoto2 Python package not available")
        return []

    cameras = list(gp.Camera.autodetect())
    ports = [port for _name, port in cameras]
    logger.info("Detected camera ports: %s", ports)
    return ports
