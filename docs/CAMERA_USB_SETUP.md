# Camera USB Setup

## The Conflict: GNOME vs. gphoto2

When a Canon camera is plugged into this machine, GNOME's virtual filesystem
daemon (`gvfsd-gphoto2`) claims the USB device automatically so Nautilus can
browse camera storage as a mounted drive. This is convenient for file browsing
but **blocks any direct gphoto2 access** — laguna and the
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

Laguna's DslrCameraSubsystem uses
libgphoto2 to trigger the shutter, download images, and apply exposure
settings. This requires exclusive USB access. GNOME auto-mount and gphoto2
cannot share the device simultaneously.

If you need to copy files off a camera with Nautilus temporarily, toggle to
ALLOWED, copy the files, then toggle back to BLOCKED before running any
experiment scripts.

## Camera Identity and Port Discovery

A camera's gphoto2 port (`usb:001,057`) contains the kernel's device
number, which changes every time the camera re-enumerates: a power cycle,
a replug, or a hiccup in the USB extender. Laguna never stores ports.
It binds each camera by its EOS body serial, which it reads over PTP on
every connect and resume. See `laguna.camera.canon`.

List the attached cameras and their serials:

```bash
python scripts/setup_dslr_udev.py
```

Wake the cameras first by half-pressing the shutter. A T7 with auto
power-off enabled drops off USB entirely, and laguna's pre-flight refuses
to connect until auto power-off is disabled in the camera menu.

### Optional: stable `/dev/dslr_<name>` symlinks

The T7's USB serial descriptor is empty, so udev can't tell two bodies
apart. It can only match the **physical hub port** (`KERNELS=="1-9.1"`).
With `--config`, the script names each camera by matching its serial
against the config's `dslr_cameras` section, and writes
`/etc/udev/rules.d/80-laguna-dslr.rules`:

```bash
sudo "$(which python)" scripts/setup_dslr_udev.py --config config/my_run.yaml --install
```

Then set `device: /dev/dslr_<name>` on each camera in the config. Laguna
tries that port first but still checks the serial, and logs a warning if
the camera turns up somewhere else (for example, after a cable was moved).
Rerun the script after recabling. Install the rules on **laguna**, the
acquisition PC the cameras plug into.
