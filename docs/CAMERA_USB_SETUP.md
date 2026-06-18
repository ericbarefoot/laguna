# Camera USB Setup

## The Conflict: GNOME vs. gphoto2

When a Canon camera is plugged into this machine, GNOME's virtual filesystem
daemon (`gvfsd-gphoto2`) claims the USB device automatically so Nautilus can
browse camera storage as a mounted drive. This is convenient for file browsing
but **blocks any direct gphoto2 access** — dualcam-timelapse, laguna, and the
`gphoto2` CLI will all fail with:

```
[-53] Could not claim the USB device
```

The udev rule at `/etc/udev/rules.d/99-canon-no-gvfs.rules` resolves this
conflict by telling the kernel to flag Canon USB devices with `GVFS_IGNORE`,
so GNOME never attempts to auto-mount them.

## Current States

| State | Rule file present? | gphoto2 works? | Nautilus browsing works? |
|---|---|---|---|
| **BLOCKED** (lab default) | Yes | ✓ | ✗ |
| **ALLOWED** (file browsing) | No | ✗ | ✓ |

Check the current state at any time:

```bash
./scripts/toggle-canon-gvfs.sh status
```

## Toggling

The toggle script switches between states and triggers udev immediately.
A camera replug is needed for the change to take effect on already-connected
cameras.

```bash
# Switch from BLOCKED → ALLOWED (re-enable Nautilus browsing)
sudo ./scripts/toggle-canon-gvfs.sh

# Switch from ALLOWED → BLOCKED (re-enable gphoto2 direct access)
sudo ./scripts/toggle-canon-gvfs.sh
```

> **After toggling:** unplug and replug the cameras (or power-cycle them) so
> the new udev environment is applied to the already-connected devices.

## Installing the Rule for the First Time

If the rule is not yet present and you want to enable direct gphoto2 access:

```bash
sudo ./scripts/toggle-canon-gvfs.sh   # installs rule if absent
```

To verify:

```bash
cat /etc/udev/rules.d/99-canon-no-gvfs.rules
# SUBSYSTEM=="usb", ATTRS{idVendor}=="04a9", ENV{GVFS_IGNORE}="1"
```

## Removing the Rule Permanently

```bash
sudo rm /etc/udev/rules.d/99-canon-no-gvfs.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

Then replug cameras. GNOME will auto-mount them again.

## Why the Lab Default Is BLOCKED

The experiment scripts (dualcam-timelapse, laguna's DslrCameraSubsystem) use
libgphoto2 to trigger the shutter, download images, and apply exposure
settings. This requires exclusive USB access. GNOME auto-mount and gphoto2
cannot share the device simultaneously.

If you need to copy files off a camera with Nautilus temporarily, toggle to
ALLOWED, copy the files, then toggle back to BLOCKED before running any
experiment scripts.

## Camera USB Port Discovery

After any replug or power cycle, camera USB device numbers change. Find current
ports with:

```python
import gphoto2 as gp
print(list(gp.Camera.autodetect()))
# [('Canon EOS 1500D', 'usb:001,057'), ('Canon EOS 1500D', 'usb:001,058')]
```

Update `experiment/2026-06-17/cameras.yaml` with the new port values, or run
`dslr_test.py` which handles port re-detection automatically after a USB reset.
