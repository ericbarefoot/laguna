# Scanner

LMI Gocator 2690 line-laser 3D surface scanner. Lives in
`src/laguna/scanner/` — `gosdk.py` (ctypes binding), `gocator.py`
(subsystem), `pointcloud.py` (surface → point cloud + export).

Distinct from [Rangefinder](rangefinder.md): the OD2000/WTT12L are
single-point distance sensors reached through a Pi-side IO-Link master over
MQTT/HTTP. The Gocator is a line-laser 3D scanner on the laguna PC's own
Ethernet network — no Pi, no IO-Link, no MQTT, no deployed scripts.

---

## Hardware chain

```
Gocator 2690 ──Gigabit Ethernet──> laguna PC
                                      ↓
                            libGoSdk.so (ctypes)
                                      ↓
                            GocatorScanner (laguna)
```

Two independent TCP connections, both opened by `connect()`: control on
port 3190 and data on 3196. The sensor's web UI is on port 80 — useful for
eyeballing settings and running alignment.

| Item | Value |
|------|-------|
| IP | `192.168.1.10` |
| Sensor ID | 188089 |
| Model | Gocator 2690 (2600 series) |
| Scan rate (datasheet) | 900–10000 Hz (high end needs reduced FOV + uniform spacing off) |
| **Scan rate (measured, this unit)** | **max 443.127 Hz at stock FOV/exposure** |
| Points per profile | 3700 |
| X resolution | 124–550 µm |
| Field of view (X) | 385–2000 mm |
| Measurement range (Z) | 1550 mm, clearance 325 mm |
| Z repeatability | 12 µm |

Don't plan around the datasheet's 10 kHz headline: read live on 2026-07-30,
this unit's `GoSetup_FrameRateLimitMax` was **443.127 Hz** at its configured
FOV/exposure/uniform-spacing. `configure()` validates `frame_rate_hz` against
that live limit and raises rather than letting an over-range value through.

Settings read off the sensor the same day (its state before laguna touched
anything): Surface mode, Time trigger, Fixed-Length generation, start trigger
`SEQUENTIAL`, fixed length 500 mm, travel speed 100 mm/s, exposure 434.9 µs,
uniform spacing on, max-frame-rate on. Note `configure()` changes the start
trigger from `SEQUENTIAL` to `SOFTWARE` — that's the one deliberate departure
from how the sensor was left after manual web-UI testing.

---

## One-time setup: build the SDK

The vendor SDK ships prebuilt shared libraries only for `linux_arm64` (the
sensor's own CPU) and Windows — `lib/linux_x64/` is **empty**. Since the
laguna PC is x86_64, `libkApi.so` and `libGoSdk.so` must be built first:

```bash
sudo apt install build-essential
```

```bash
scripts/build_gosdk.sh
```

The script drives the vendor's own `kApi-Linux_X64.mk` and
`GoSdk-Linux_X64.mk` makefiles and leaves the libraries in
`$GO_SDK/lib/linux_x64/`. Point laguna at them with the `sdk_lib_dir` config
key, or `$LAGUNA_GOSDK_LIB_DIR`; with neither set,
`laguna.scanner.gosdk.find_lib_dir()` checks
`~/Downloads/14400-6.5.2.5_SOFTWARE_GO_SDK/GO_SDK/lib/linux_x64`, `/opt/GO_SDK`,
and `/usr/local/GO_SDK`.

The SDK tree itself is **not** vendored into this repo (it's ~170 MB).

---

## How encoderless scanning works

This is the part worth understanding before trusting a scan, because a
silently wrong answer is possible here.

The Gocator builds a 3D surface by stacking successive laser-line profiles
along the direction of relative motion (Y). Normally an encoder tells it how
far the target moved between profiles. **We have no encoder.** Instead:

1. **Trigger source = Time** — profiles fire on the sensor's internal clock
   at a fixed frame rate. The sensor has no idea how far anything moved.
2. **Travel speed** (mm/s) is configured out of band, and the sensor uses it
   to convert elapsed time into Y distance:
   `Y spacing = travel_speed / frame_rate`.
3. **Surface generation = Fixed Length** with a **Software** start trigger,
   so one gantry pass produces exactly one surface.

The consequence: **Y accuracy is entirely dependent on the configured travel
speed matching the gantry's actual velocity.** Nothing measures or corrects
this. If the gantry runs at 19 mm/s while the sensor is told 20 mm/s, every
Y coordinate is 5% wrong and the scan is stretched along travel. So:

- Fire the trigger *after* the axis is off its acceleration ramp
  (`settle_s`), and keep the pass within constant-velocity travel.
- Re-configure travel speed whenever the feed rate changes —
  `scan_with_gantry()` does this for you from `feed_rate_mm_s`.
- If you later measure the true velocity, `SurfaceScan.rescale_y()` fixes
  the travel axis without re-scanning.

`travel_speed` maps to `GoTransform_SetSpeed()` in the SDK, and to
**Manage > Motion and Alignment > Speed** in the web UI. It writes to sensor
**flash**, so `configure()` only pushes it when the value actually changes.

---

## Quick start

```python
from laguna import FlumeLab
from laguna.robot.macron import GantryController
from laguna.scanner import GocatorScanner

lab = FlumeLab("config/example_config.yaml")

scanner = GocatorScanner.from_config(lab.config.get("gocator"))
gantry = GantryController.from_config(lab.config.get("gantry"))
lab.add(scanner).add(gantry)
lab.connect_all()

# Coordinated pass: configures travel speed from feed_rate_mm_s, starts a
# non-blocking move, waits out the accel ramp, triggers, receives the surface.
scan = lab.gocator.scan_with_gantry(
    gantry, axis="X", end_mm=400.0, feed_rate_mm_s=20.0, settle_s=0.5
)

print(scan.shape)                 # (rows, cols) = (Y samples, X samples)
print(scan.valid_count)           # cells with an actual laser return
points = scan.to_points()         # (N, 3) XYZ in mm, invalid dropped

lab.gocator.save_scan(scan, formats=("npz", "ply"))
lab.disconnect_all()
```

`scan_with_gantry()` drives the axis through its `AxisHandle`
(`gantry.axis("X")`) rather than `gantry.move_to()`, because the trigger must
fire *while* the axis is mid-move and `move_to()` blocks until the move
finishes. `AxisHandle.begin_move_to()` is non-blocking and still enforces the
gantry's `safe_mode` gate — the raw `gantry.cmd` path does not, on the
socket_bridge/ethernet/rs232 transports.

It does **not** fence-check the target the way `move_to()` does, so validate
your destination is inside the work envelope.

### Manual lifecycle

When you want to drive motion yourself (or the target moves independently):

```python
scanner.configure(travel_speed_mm_s=20.0, frame_rate_hz=500, fixed_length_mm=200.0)
scanner.start()          # enable data channel + begin acquisition
# ... get the target moving at constant velocity ...
scanner.trigger()        # software start trigger — one fixed-length surface
scan = scanner.receive_surface(timeout_s=15.0)
scanner.stop()
```

`scan()` wraps configure/start/trigger/receive/stop for the case where motion
is already underway.

---

## Output data

The sensor returns a **dense grid**, not an unordered cloud — rows along Y
(travel), columns along X (across the laser line). Two message flavours are
handled:

| Message type | Contents | `SurfaceScan.is_uniform` |
|---|---|---|
| `UNIFORM_SURFACE` (8) | Z only per cell; X/Y implied by column/row index × resolution | `True` |
| `SURFACE_POINT_CLOUD` (28) | full x/y/z raw triple per cell (un-resampled) | `False` |

Scaling, applied by `pointcloud.py`:

```
value_mm = offset_um / 1000 + resolution_nm / 1e6 * raw_count
```

Resolutions are nanometres, offsets micrometres, grid values 16-bit signed
counts. `0x8000` (-32768) means "no data" (occlusion, no return) and becomes
`NaN` in `z_mm` — `to_points()` drops those by default.

### Export formats

| Format | Method | Use for |
|---|---|---|
| `.npz` | `save_npz()` | Reprocessing — preserves the grid *and* NaNs |
| `.ply` | `save_ply()` | CloudCompare / MeshLab viewing |
| `.csv` | `save_csv()` | `x_mm,y_mm,z_mm` text, spreadsheet-friendly |

`save_scan(scan, formats=(...))` writes several at once under `output_dir`
with a UTC-timestamped name.

---

## Configuration

```yaml
gocator:
  ip: 192.168.1.10
  travel_speed_mm_s: 20.0     # MUST match the gantry feed rate
  frame_rate_hz: 500          # omit for the sensor's max rate
  fixed_length_mm: 200.0      # surface length along travel
  exposure_us: null           # omit to leave the sensor's setting
  uniform_spacing: null       # true/false to force X resampling; omit to leave as-is
  output_dir: ./data/scans
  sdk_lib_dir: null           # omit to auto-discover
```

Picking a frame rate: `Y spacing = travel_speed / frame_rate`. At 20 mm/s and
400 Hz that's 0.05 mm between profiles — far finer than the 0.124–0.55 mm X
resolution, so X is usually the limiting axis. Lower the frame rate (or raise
the feed rate) to trade Y density for scan time. Keep the value under the
sensor's live ceiling (443 Hz measured here, see above); `configure()` will
reject anything over it.

---

## Testing without hardware

`tests/test_gocator_scanner.py` replaces the ctypes layer with a fake that
records calls and serves synthetic surfaces, so the full scan lifecycle,
the recipe constants, and the raw-count scaling are all testable with no
sensor and no built SDK:

```bash
python -m pytest tests/test_gocator_scanner.py -q
```

---

## Reference

- `docs/reference/gocator/GOCATOR_SDK_NOTES.md` — GoSdk C API: ports, enums,
  message structs, control flow, threading/buffer gotchas
- `docs/reference/gocator/GOCATOR_CONCEPTS.md` — manual-derived concepts:
  Profile vs Surface, encoderless triggering, alignment, 2690 specs
- `examples/example_08_gocator_surface_scan.py` — runnable end-to-end scan
  with a `--dry-run` mode

---

## Verified against hardware (2026-07-30)

The read path is confirmed working end-to-end from Python against the real
sensor — SDK loads, all bound symbols resolve, discovery finds the unit,
`GoSensor_Connect` succeeds, and every settings accessor returns sane values
(the table above was read this way). Clean disconnect too.

**Discovery needs UDP broadcast** (port 3220). `GoSystem_FindSensorByIpAddress`
searches the *discovered* list, so it returns `kERROR_NOT_FOUND` (-999) on a
network that blocks broadcast even when the sensor answers ICMP fine. If you
see -999, check broadcast reachability before suspecting the IP.

## Open items

- **Verify `GoSensor_Trigger()` is the right software-start call.** The recipe
  was confirmed on hardware via the web UI's start-scan button; firing it from
  the SDK is implemented but untested. No SDK sample calls `_Trigger()`, so
  this is the highest-risk assumption in the module. If it doesn't work,
  check what the web UI actually sends.
- **Nothing has yet *written* to the sensor from laguna** — the hardware
  verification above was strictly read-only, so `configure()`'s write path
  (including the `GoTransform_SetSpeed` flash write and the `SEQUENTIAL` →
  `SOFTWARE` start-trigger change) is still unexercised.
- **Confirm which message type the sensor emits** with this configuration
  (`UNIFORM_SURFACE` vs `SURFACE_POINT_CLOUD`). Both paths are implemented;
  only one will exercise on hardware.
- **Validate Y scaling end-to-end**: scan an object of known length along
  travel and check the surface's Y extent matches. This is the real test of
  the encoderless assumption.
- **Measure achievable frame rate** at the FOV/exposure we actually use,
  rather than trusting the datasheet range.
- **Tune `settle_s`** against the gantry's real acceleration ramp — if the
  leading edge of scans looks compressed along Y, it's too short.
- Consider whether `GoSystem_SetDataCapacity` needs raising for long scans
  (`data_capacity_bytes` config key is wired up but unused by default).
