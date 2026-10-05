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

## Lab setup on laguna

Two Canon EOS Rebel T7 bodies (gphoto2 reports them as "EOS 1500D", the
same camera's name outside North America) connect to **laguna** through
the USB extender. Its far end is a 4-port hub on the PC's root port 9.

| Name | EOS serial | Hub port (`KERNELS`) | Symlink | SD card |
|---|---|---|---|---|
| Nakdong | `852078018710` | `1-9.1` | `/dev/dslr_nakdong` | none |
| Hangang | `922079026084` | `1-9.2` | `/dev/dslr_hangang` | yes |

The hub ports are **physical**: if a cable moves to a different hub
socket, the symlink follows the socket, not the camera. Laguna still binds
by serial, so a moved cable only produces a warning, but rerun step 3
below so the symlinks are accurate again.

### One-time install

Run these on laguna, from the repo root, with the `flumelab` env active and
both cameras switched on and awake:

1. **Stop GNOME from grabbing the cameras:**
   ```bash
   sudo ./scripts/toggle-canon-gvfs.sh status
   ```
   If that reports ALLOWED, switch it to BLOCKED:
   ```bash
   sudo ./scripts/toggle-canon-gvfs.sh
   ```
   Laguna also releases gvfs each time it connects, so this step only
   saves the camera a PTP session reset.
2. **Check what's attached:**
   ```bash
   python scripts/setup_dslr_udev.py --config <your run config>
   ```
   Each camera should list the serial and hub port from the table above,
   with its config name next to it.
3. **Install the symlinks:**
   ```bash
   sudo "$(which python)" scripts/setup_dslr_udev.py --config <your run config> --install
   ```
   This writes `/etc/udev/rules.d/80-laguna-dslr.rules`:
   ```
   # Managed by laguna/scripts/setup_dslr_udev.py — do not edit by hand.
   SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="04a9", KERNELS=="1-9.2", SYMLINK+="dslr_hangang"
   SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="04a9", KERNELS=="1-9.1", SYMLINK+="dslr_nakdong"
   ```
   It then reloads udev and runs `ls -l /dev/dslr_*`.
4. **Check the symlinks:**
   ```bash
   ls -l /dev/dslr_*
   ```
   Each one should point at a `bus/usb/001/NNN` node. The `NNN` changes
   every time the camera re-enumerates; the symlink name doesn't.

To remove the symlinks, delete `/etc/udev/rules.d/80-laguna-dslr.rules`
and run `sudo udevadm control --reload-rules`.

### Config

```yaml
dslr_cameras:
  interval_s: 20
  capture_target: ram        # Nakdong has no card; one value for ALL cameras (see below)
  cameras:
    Nakdong:
      serial: "852078018710"
      device: /dev/dslr_nakdong
      imageformat: RAW + L
      exposure: {iso: "1600", aperture: "5.6", shutter: "1/60"}
      output_dir: ./captures/Nakdong
    Hangang:
      serial: "922079026084"
      device: /dev/dslr_hangang
      imageformat: RAW + L
      exposure: {iso: "800", aperture: "5.6", shutter: "1/30"}
      output_dir: ./captures/Hangang
```

The exposures are just what each body had set when it was tested; set
them for the experiment. With both cameras in RAM mode, a RAW + L
capture-and-download takes about 3.2 s per trigger (about 42 MB per
camera), measured on 2026-10-05.

### Camera menu settings

Pre-flight refuses to connect if the first three aren't set. gphoto2 can't
change them; a person has to, on each camera:

- **Mode dial on M.** In any other mode the camera ignores some of the
  exposure it is sent.
- **Lens switch on MF**, so autofocus can't shift focus between frames.
- **Auto power-off: Disable.** An idle T7 otherwise drops off USB
  entirely.
- **Time zone: the same on every camera.** Pre-flight sets each camera's
  clock from the PC. After that, both lab cameras wrote EXIF times in UTC,
  which matches the UTC timestamps on the Pi camera logs. Laguna's own
  filenames use the PC's clock either way.

### Gotchas

- **The capture target is one setting for the whole computer.**
  libgphoto2 stores it once in `~/.config/gphoto/settings`
  (`ptp2=capturetarget=...`) and applies it to whichever camera is about
  to shoot. With mixed targets, the last camera's pre-flight decides for
  all of them, and a cardless camera told `card` hangs for 90 s per
  capture. That's why `capture_target` can only be set for the whole
  `dslr_cameras` section.
- **No card space means a 90 s hang, not an error.** Pre-flight refuses
  with fewer than 10 free shots, on the card or in RAM.
