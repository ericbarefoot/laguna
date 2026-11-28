# Camera

Three distinct camera paths, unified behind one `CameraManager` façade:
locally-attached cameras (OpenCV), a synchronized array of networked
Raspberry Pi cameras (SSH-triggered), and DSLRs via a wrapped external
package. Lives at `src/laguna/camera/`.

## The three paths

| Class | `subsystem_name` | What it talks to | Where |
|---|---|---|---|
| `LocalCamera` | — (used via `CameraManager`) | OpenCV, `cv2.VideoCapture` | `camera/local.py` |
| `CameraArray` | `pi_cameras` | Raspberry Pis over SSH/paramiko | `camera/network.py` |
| `DslrCameraSubsystem` | `dslr_cameras` | Any number of Canon EOS DSLRs via `gphoto2`, bound by body serial | `camera/dslr.py`, `camera/canon.py` |

`CameraArray` and `DslrCameraSubsystem` each register with `FlumeLab`
directly via their own `subsystem_name` (`pi_cameras` / `dslr_cameras`).
`CameraManager` is a separate, higher-level façade for when you want one
object managing several local + networked cameras together (its own config
takes a `cameras:` list, not the `pi_cameras:`/`dslr_cameras:` top-level
keys `setup_run()` uses — see below).

## Networked Pi array — synchronized capture

`CameraArray.trigger_capture()` SSHes into every configured Pi concurrently
(one thread per host) and fires the shutter with a **lead time** — each Pi
is told to capture `lead_time` seconds in the future, not immediately — so
that clock/network jitter on the SSH round-trip doesn't desynchronize the
shots. Default lead time: `DEFAULT_LEAD_TIME` in `network.py`.

After capture, `CaptureResult.capture_time_mid_pc` timestamps let you
measure how tight the sync actually was. `assess_spread()` labels the
result:

| Spread | Label |
|---|---|
| < 20 ms | `EXCELLENT` |
| < 50 ms | `GOOD` |
| < 200 ms | `MARGINAL` |
| ≥ 200 ms | `POOR` — likely an NTP sync problem between Pis |

```python
from laguna.camera.network import CameraArray

array = CameraArray(hosts=["antares.laguna", "sirius.laguna"], ssh_user="pi")
array.connect()
results = array.trigger_capture(lead_time=5.0)
fetched = array.fetch_images(results, output_dir="./captures")  # SFTP pull
```

`check_clock_sync()` and `check_connectivity()` are worth running once
before a real experiment — they exist specifically because SSH+NTP drift is
the main failure mode here, not the capture itself.

## DSLR — Canon EOS over gphoto2

`laguna.camera.canon` drives the cameras directly. It started as a vendored
copy of Minsik's (@yukms)
[`dualcam-timelapse`](https://github.com/yukms/dualcam-timelapse) (issue
#60), so there is no longer a separate repo to clone. Install the optional
extra with `pip install -e ".[dslr]"`. It only runs on Linux, and
[Camera USB setup](../CAMERA_USB_SETUP.md) explains how to keep GVFS from
grabbing the cameras.

How it behaves:

- **Cameras are bound by body serial, never by port.** A gphoto2 port
  (`usb:001,005`) changes every time the camera re-enumerates, and the T7
  reports an empty USB serial, so the serial read from the camera itself is
  the only reliable identity. `connect()` and `resume()` open each attached
  camera, check its serial, and bind it under its configured name. Port
  order never matters, so two cameras can't silently swap output folders.
- **Pre-flight refuses rather than warns.** A camera won't connect unless
  its mode dial is on **M**, its lens is on **MF**, and auto power-off is
  disabled; an asleep T7 drops off USB. Exposure, image format and
  capture target (the memory card) are written to the camera and read
  back. Each camera's clock is set from the PC so EXIF times can be
  trusted.
- **Connect is all-or-nothing.** If one configured camera is missing,
  none of them connect.
- **Every capture is verified, and card copies are a rolling backup.**
  Each file (both halves of a RAW+JPEG pair) downloads to a `.part` file,
  is checked against the card copy's size and file signature, and only
  then gets renamed. Copies on the card are deleted oldest-first, and only
  once free space drops below `card_reserve_shots` and the download has
  been verified. A download that fails is retried once (the shutter has
  already fired, so this costs no timing).
- **Cardless bodies:** `capture_target: ram` sends shots to the
  camera's RAM instead, and each shot is released from RAM after download.
  There is no second copy, so a failed download is a lost frame, which
  still pauses the lab. **`capture_target` applies to every camera.**
  libgphoto2 keeps one capture target for the whole computer (in
  `~/.config/gphoto/settings`), not one per camera, so setting it per
  camera would let one camera overwrite another's. On hardware, that sent
  a cardless body to "card", and each of its captures hung for 90 s.
  Either every camera has a card, or all of them use `ram`.
- **A failed capture pauses the lab.** `capture_all()` never retries,
  because a missed frame can't be retaken later. The runner escalates any
  failure with `lab.escalate()`, and the `capture_failed` event names any
  card files you can still recover. `resume()` re-binds by serial, so you
  can power-cycle or recable a camera while the lab is paused.
- **Filenames are unique:** `<camera>_t<runtime>_<wall time to the ms>.jpg`.
  When run directories are enabled, cameras with no `output_dir` of their
  own write to `<run dir>/dslr_cameras/<camera>/`. A camera with an
  explicit `output_dir` keeps it.

```python
lab.add("dslr_cameras")                  # from the config's dslr_cameras: section
lab.connect_all()                        # binds by serial, runs pre-flight
records = lab.dslr_cameras.capture_all() # {name: CaptureRecord(files, card_files, error)}
```

To find each camera's serial, and optionally give it a `/dev/dslr_<name>`
symlink for its hub port, run:

```bash
python scripts/setup_dslr_udev.py --config config/my_run.yaml
```

## Fixed exposure and white balance

By default the Pi agent uses libcamera's **automatic** exposure (AE) and white
balance (AWB). Because the camera is opened fresh for each capture, both
algorithms restart from scratch every frame and have only a few frames to
settle, so brightness and colour vary through a run (and between Pis). For
anything quantitative, fix them.

```yaml
pi_cameras:
  hosts: [antares.laguna, sirius.laguna]
  exposure:
    mode: calibrated                              # auto (default) | manual | calibrated
    calibration_file: ./pi_camera_calibration.json
```

| Mode | Behaviour |
|---|---|
| `auto` | AE/AWB on. Historical behaviour and the default. A warning is logged at run start. |
| `manual` | Fixed `exposure_time_us`, `analogue_gain`, `awb_gains: [red, blue]` from the config; `per_host:` overrides individual values for one Pi. |
| `calibrated` | Per-host values from `calibration_file`, produced by the procedure below. |

### Calibrating at the start of an experiment

Do this under the lighting the run will use — lamps on and warmed up, water
at the working level.

```bash
laguna-picam --config config/my_config.yaml calibrate
```

Each Pi runs AE/AWB for `warmup_s` seconds (default 5), samples several frames
and reports the median exposure time, gain and colour gains. A host whose
frames still disagree by more than 5 % is marked `NOT CONVERGED` — wait for
the scene to settle and run it again. A preview JPEG per camera is downloaded
next to the calibration file (`<name>_previews/`); look at them, then confirm
at the prompt. The values are written only if **every** camera calibrated and
you confirm, so a calibration never covers some of the cameras. An existing
file is not replaced unless you pass `--overwrite`.

The file is plain JSON (`format`, `hosts: {host: {exposure_time_us,
analogue_gain, awb_gains}}`); edit a value by hand if you want a different
exposure from the one auto picked. Keep it with the run's data.

Then start the run with `mode: calibrated`. Every capture applies those
values with AE/AWB **off**.

### Guarantees

- A locked mode never degrades to auto. A missing, unreadable or incomplete
  calibration file raises when the config is read, before the run starts. If
  the Pi has no `picamera2`, the capture fails (and is reported as a missed
  capture) rather than producing an auto-exposed frame.
- The Pi opens the camera ~3 s before the target time so the fixed values have
  taken effect; timing metadata still describes the actual capture.
- The agent returns what the camera *reported* for each frame. The event log's
  `capture` row carries `exposure_mode`, `exposure_time_us`, `analogue_gain` and
  `colour_gains`; a frame that differs from what was commanded (beyond sensor
  quantisation, 5 %) also logs an `exposure_mismatch` warning. `exposure_settings`
  at run start records the mode and values in force.
- In `auto` mode with the legacy path, per-frame values are not available.

### Previewing with the same settings

```bash
laguna-picam --config config/my_config.yaml snap 1 --locked -o shot.jpg
```

`--locked` applies the config's `exposure` for that camera to `snap`/`stream`
(or give `--shutter-us`, `--gain` and `--awbgains R,B` together).

Not covered by simulation: `simulate=True` exercises the config, the
calibration file and the event log, but it cannot tell whether the chosen
values suit the real lighting — only a calibration on the real rig can.

## Remote view and snapshot (`laguna-picam`)

For a human looking at a Pi camera from another machine, with commands going
client → laguna → pi and image bytes coming back pi → laguna → client over the
SSH pipes (no ports opened, nothing to configure on the Pi beyond `rpicam-*`
or `libcamera-*`, which Pi OS ships).

On laguna, `pip install -e .` provides `laguna-picam`:

```bash
laguna-picam --config config/my_config.yaml list
laguna-picam --config config/my_config.yaml snap 1 -o shot.jpg   # 1 = first pi_cameras.hosts entry
laguna-picam --config config/my_config.yaml stream 1 | ffplay -f mjpeg -i -
```

On the client, `scripts/picam-remote.sh` does the first hop and saves/plays
locally (needs `ffplay` or `mpv` for `view`):

```bash
export LAGUNA_SSH_HOST=laguna
export LAGUNA_PICAM=~/miniforge3/envs/flumelab/bin/laguna-picam   # non-interactive ssh has no conda
export LAGUNA_CONFIG=~/mysoftware/laguna/config/my_config.yaml
alias picam='/path/to/laguna/scripts/picam-remote.sh'

picam list
picam snap 1                 # saves ./<host>_<utc>.jpg on the client
picam view 1 --framerate 10  # live MJPEG window
```

Requirements and limits:

- laguna → Pi login must be non-interactive (`BatchMode`): use a key without a
  passphrase, or load it into an agent on laguna.
- The Pi camera is **exclusive**. A stream left open when a scheduled capture
  fires makes that capture fail, and a missed frame cannot be re-taken. Close
  the view before a scheduled run, or between its capture times.
- This is for looking, not data collection: nothing is logged to the event log
  and no timing metadata is recorded.

## Config, via `setup_run()`

`src/laguna/experiment/runner.py`'s `setup_run()` looks for two independent
top-level YAML sections — a config can have either, both, or neither:

```yaml
pi_cameras:
  hosts: [antares.laguna, sirius.laguna]
  ssh_user: ucrs
  ssh_key: ~/.ssh/id_rsa
  output_dir: ./captures/pi
  interval_s: 60          # or trigger_at: [30, 90] or use_schedule: true

dslr_cameras:
  trigger_at: [0, 30, 60]
  capture_target: card    # or ram, for a body with no SD card
  card_reserve_shots: 500
  cameras:
    Hangang:
      serial: "852078018710"
      imageformat: RAW + L
      exposure: {iso: "800", aperture: "5.6", shutter: "1/125"}
      output_dir: ./captures/Hangang
```

With `use_schedule: true`, the schedule CSV's `pi_cameras` / `dslr_cameras`
columns (if present) gate which scheduled time points actually fire a
capture — see [Schedule](schedule.md). If the CSV has no such column,
capture fires at every `time_s` row.

## Log format and file naming

Camera agents run on distributed Raspberry Pi nodes that may be in different
timezones. All timestamps are anchored to UTC so that logs and filenames from
multiple hosts can be compared directly.

**Log lines** (written by `agent.py` to stderr, streamed back to the coordinator):

```
[agent 14:23:45.123Z/10:23:45] Sleeping 4.997s until target_time...
```

The format is `HH:MM:SS.mmmZ` (UTC) followed by `/HH:MM:SS` (local time on that
Pi). `utc_local_ts()` in `camera/agent.py` produces both strings. The helper
lives in the agent script because that file is copied to the Pi and run with
plain `python3`, and the Pi does not have the `laguna` package installed.

**Image filenames** use ISO 8601 UTC derived from the scheduled `target_time`,
not the wall-clock time of the actual shutter:

```
capture_20240115T142345123Z.jpg   # YYYYMMDDTHHMMSSfffZ
```

Using `target_time` (rather than capture-wall-clock) means filenames are
consistent across the array even when individual Pis capture a few milliseconds
early or late.

## Further reading

- [API reference](../reference/camera.md) — generated from docstrings.
- [Camera USB setup](../CAMERA_USB_SETUP.md) — udev rules, GVFS-vs-gphoto2 USB conflicts.
- [Experiment runner](experiment.md) — how `pi_cameras:`/`dslr_cameras:` config drives scheduling.
- `src/laguna/camera/` — `local.py`, `network.py`, `dslr.py`.
