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
| `DslrCameraSubsystem` | `dslr_cameras` | Canon DSLRs via `gphoto2`, wrapping the external [`dualcam-timelapse`](https://github.com/yukms/dualcam-timelapse) package | `camera/dslr.py` |

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

## DSLR — wraps an external package

`DslrCameraSubsystem` does not talk to `gphoto2` directly; it wraps
Minsik's `dualcam-timelapse` package, which is **not pip-installable** — it
must be cloned separately and pointed to via `dualcam_path`. This is a
Linux-only path (gphoto2/libgphoto2), and per [Camera USB
setup](../CAMERA_USB_SETUP.md), GVFS auto-mounting the camera as a media
device will fight with `gphoto2` for the USB connection unless the udev
rules there are applied.

```python
from laguna.camera import DslrCameraSubsystem

dslr = DslrCameraSubsystem(
    config_path="/path/to/dualcam-timelapse/config/cameras.yaml",
    dualcam_path="/path/to/dualcam-timelapse",
)
lab.add(dslr)
lab.connect_all()
dslr.capture_all()
```

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
  config_path: /path/to/dualcam-timelapse/config/cameras.yaml
  dualcam_path: /path/to/dualcam-timelapse
  trigger_at: [0, 30, 60]
```

With `use_schedule: true`, the schedule CSV's `pi_cameras` / `dslr_cameras`
columns (if present) gate which scheduled time points actually fire a
capture — see [Schedule](schedule.md). If the CSV has no such column,
capture fires at every `time_s` row.

## Further reading

- [API reference](../reference/camera.md) — generated from docstrings.
- [Camera USB setup](../CAMERA_USB_SETUP.md) — udev rules, GVFS-vs-gphoto2 USB conflicts.
- [Experiment runner](experiment.md) — how `pi_cameras:`/`dslr_cameras:` config drives scheduling.
- `src/laguna/camera/` — `local.py`, `network.py`, `dslr.py`.
