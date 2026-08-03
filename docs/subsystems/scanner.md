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
FOV/exposure/uniform-spacing.

**The ceiling is dynamic, and that's a trap.** With max-frame-rate mode
enabled it read 443.127 Hz; after `configure()` disabled that mode and set an
explicit rate, the same accessor read **221.563 Hz** — exactly half, with
exposure unchanged. So a rate can pass validation at write time and still
leave the sensor holding a rate it cannot deliver.

This matters because `Y spacing = travel_speed / frame_rate`. A sensor
quietly running slower than commanded produces a scan that is *distorted
along travel*, not one that obviously fails. `configure()` therefore
re-checks after flushing: it raises if the configured rate exceeds the
post-flush ceiling, and if the sensor reports a different rate than
requested, it believes the sensor and uses that value for bookkeeping.

Practical guidance: **keep `frame_rate_hz` at or below ~220 Hz** on this unit
*at stock settings* — that figure is a property of the current active area
and exposure, not of the sensor, and shrinking the active area (see below)
raises it substantially. Or pass `frame_rate_max=True` (`configure(frame_rate_max=True)`,
or `example_08 --frame-rate-max`) to explicitly run at the sensor's own
maximum instead of guessing a number. Prefer this over just omitting
`frame_rate_hz` — omitting it only leaves whatever frame-rate mode/rate the
sensor already has untouched, and a *previous* `configure()` call that set
an explicit rate disables max-frame-rate mode in sensor flash, so omission
afterward does **not** get you back to max. `frame_rate_max=True`
(re-)enables that mode outright and reads back the achieved rate after
flushing, since the ceiling is dynamic — see `get_status()["sensor_frame_rate_max_hz"]`
after configuring, not before, either way.

### Raising the ceiling: the active area

The frame-rate ceiling is not fixed — it is mostly a consequence of how much
of the sensor's field of view it has to read out. The **active area** (region
of interest) is the biggest lever:

```python
scanner.get_active_area()          # current values + the sensor's live limits
scanner.set_active_area(z=400, height=200, width=600)
scanner.configure(active_area={"z": 400, "height": 200})
```

```yaml
gocator:
  active_area: {z: 400, height: 200, width: 600}
```

```bash
python examples/example_08_gocator_surface_scan.py --dry-run \
    --active-area z=400,height=200,width=600
```

Six fields, all mm: `x`, `y`, `z` (the origin) and `width`, `length`,
`height` (the extents from it). Set any subset; the rest are left alone.
**Cutting `height` (the Z/range extent) buys the most** — fewer camera rows
per profile is precisely what raises the rate — with `width` (X) next.

The trade-off is unforgiving in one direction: anything outside the active
area is simply not measured. Leave margin for the tallest feature you care
about and for any Z wander in the gantry, or you will silently clip the top
of the scan rather than get an error.

`configure()` applies the active area **before** the frame rate, because the
ceiling the rate is validated against depends on it — validating first would
check a stale number. Values are checked against the sensor's own live
`*LimitMin`/`*LimitMax` (model- and configuration-dependent, so read them
rather than trusting the datasheet), and all fields are validated before any
are written, so a bad value can't leave a half-applied ROI silently clipping
the scan.

Practical loop: `--dry-run` prints both the live active area and the
resulting `sensor_frame_rate_max_hz`, so shrink, check the ceiling, repeat.

SDK: `GoSetup_[Set]ActiveArea{X,Y,Z,Width,Length,Height}` plus
`*LimitMin`/`*LimitMax` — 24 entry points, each taking a `GoRole`
(`GO_ROLE_MAIN`), all `k64f` mm, introduced in firmware 4.0.10.27. Verified
present in this unit's built `libGoSdk.so`.

### Measured: what actually buys frame rate (2026-08-02)

Swept on this unit with `z=-10`, full X width, max-frame-rate mode held on,
reading `GoSetup_FrameRateLimitMax` at each step. Ceiling in Hz:

| active-area height (mm) | uniform spacing | point cloud |
|---|---|---|
| 10 | 603.0 | 201.0 |
| 100 | 559.9 | 186.6 |
| 200 | 509.8 | 169.9 |
| 400 | 411.4 | 137.1 |
| 600 | 312.9 | 104.3 |
| ≥800 | 221.7 | 73.9 |

Four findings, all extremely consistent:

1. **Point-cloud mode costs exactly 3.000×**, at every height tested. That
   ratio never budged.
2. **X subsampling is exactly linear**: `x=2` → 2.000×, `x=4` → 3.996×. It
   works in **both** modes, and it is the cheapest large win available.
3. **Z subsampling does nothing for rate** — ratio 1.000 at every height and
   both modes. It costs Z resolution and buys no speed; leave it at 1.
4. **Active-area height saturates at ~800 mm.** Above that the ceiling is
   pinned at 221.7 Hz; below it the gain is real but sublinear — 800→10 mm
   is only 2.72×, and 400→100 mm just 1.36×.

**The most important consequence: you cannot buy back the point-cloud
penalty with the active area.** The best point-cloud figure measured (201 Hz,
height 10 mm) is still below the *worst* uniform-spacing figure (221.7 Hz at
height 1400 mm). If you need a true point cloud, budget for ~⅓ the rate and
recover it with `--x-subsampling` instead.

Practical combinations at 20 mm/s travel:

| config | ceiling | Y spacing |
|---|---|---|
| uniform, x=1, height 400 | 411 Hz | 0.049 mm |
| uniform, x=4, height 100 | 2236 Hz | 0.0089 mm |
| point cloud, x=1, height 100 | 187 Hz | 0.107 mm |

Y spacing is already far finer than the 0.124 mm native X resolution in every
case, so the sensible use of extra rate is usually a **higher feed rate**
(shorter scans), not finer Y.

### Subsampling, spacing interval, and filters

```python
scanner.get_subsampling()      # {'x': 1, 'x_options': [1,2,4], 'z': 2, ...}
scanner.set_subsampling(x=4)   # works in BOTH modes
scanner.set_spacing_interval(type="balanced")   # uniform spacing only
scanner.get_filters()          # per-filter available/enabled/window/limits
scanner.set_filters(x_smoothing=1.5, y_median=True, x_gap_filling=False)
```

Filter values: `False` disables, `True` enables keeping the current window,
a number enables and sets the window in mm. The eight filters are
`x_smoothing`, `x_median`, `x_decimation`, `x_gap_filling` and the `y_`
equivalents (`laguna.scanner.FILTER_NAMES`).

**Filters and the spacing interval require uniform spacing.** They act on
the resampled X grid, which point-cloud mode doesn't produce — the sensor
reports them unavailable (`GoSetup_*Used` reads false, verified to flip
0→1 exactly with uniform spacing) and would silently ignore the writes. So
laguna raises `UniformSpacingRequiredError` instead, and `example_08`
rejects `--filter`/`--spacing-interval` alongside `--point-cloud` before it
even connects. `configure()` checks against the value *that call* applies,
so enabling uniform spacing and setting filters together works, while
disabling it and setting filters together is refused without writing
anything.

**Subsampling is deliberately not gated** — it is an acquisition-level
divider, confirmed working in both modes.

CLI: `--x-subsampling {1,2,4}`, `--z-subsampling {1,2,4,8}`,
`--spacing-interval max_res|balanced|max_speed|<mm>`, and repeatable
`--filter NAME[=MM|=off]`.

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

The consequence: **travel-axis accuracy is entirely dependent on the
configured travel speed matching the gantry's actual velocity.** Nothing
measures or corrects this. If the gantry runs at 19 mm/s while the sensor is
told 20 mm/s, every coordinate along travel is 5% wrong and the scan is
stretched. ("Travel" is the sensor's Y, which with the mounting above is
gantry **X** — see "Sensor axes are not gantry axes".) So:

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

# Coordinated pass: configures travel speed from feed_rate_mm_s, derives
# fixed_length_mm from end_mm and the axis's current position (overriding
# gocator.fixed_length_mm in the config — see "Configuration" below), starts
# a non-blocking move, waits out the accel ramp, triggers, receives the
# surface.
scan = lab.gocator.scan_with_gantry(
    gantry, axis="X", end_mm=400.0, feed_rate_mm_s=20.0, settle_s=0.5
)

print(scan.shape)                 # (rows, cols) = (Y samples, X samples)
print(scan.valid_count)           # cells with an actual laser return
points = scan.to_points()         # (N, 3) XYZ in mm, invalid dropped

lab.gocator.save_scan(scan, formats=("npz", "ply"))
lab.disconnect_all()
```

### Gantry prerequisites

Two things must be true before a scan pass can move anything:

- **Transport.** `pi_agent` — the only supported transport, and the only one
  `example_08` will configure (it forces `transport: pi_agent` itself,
  regardless of what the config file says). It launches `gantry_agent.py`
  over SSH itself, so there's no manual Pi-side step and nothing else to
  start first. The retired `socket_bridge` transport depended on
  `serial_bridge.py`, a hand-started script on the Pi that never survived a
  reboot and could not do topographic scanning at all — see
  `docs/MACRON_GANTRY.md`, "Retired: serial_bridge.py".
- **`safe_mode: false`.** Every motion command is gated by it, so a scan
  cannot run with it on. `example_08` owns this itself — it derives
  `safe_mode` from its own `--allow-motion` flag (same `ALLOW_MOTION` idiom
  as `example_05`/`example_07`) rather than trusting the config file's
  value. Without `--allow-motion`, the script always behaves like
  `--dry-run`.

`scan_with_gantry()` drives the axis through its `AxisHandle`
(`gantry.axis("X")`) rather than `gantry.move_to()`, because the trigger must
fire *while* the axis is mid-move and `move_to()` blocks until the move
finishes. `AxisHandle.begin_move_to()` is non-blocking and still enforces the
gantry's `safe_mode` gate — the raw `gantry.cmd` path does not, on the
ethernet/rs232 transports.

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

## Sensor axes are not gantry axes

**The single most confusing thing about this subsystem.** The Gocator names
its axes from its own optics, not from the machine it is bolted to:

| sensor axis | means |
|---|---|
| X | across the laser line (the ~2 m fan) |
| Y | along travel — whichever way the target moves |
| Z | range (height / standoff) |

On this rig the sensor is mounted **rotated 90° about Z**. So a gantry move
along **X** produces the sensor's **Y** axis, and the laser fan lies along
gantry **Y**. Confirmed from two scans on 2026-08-02: `axis="X"` for 200 mm
and 300 mm gave sensor-Y spans of 199.8 mm and 299.9 mm, while sensor X held
a constant 2003 mm — the active-area width, nothing the gantry did.

Left untranslated, that means `move_to(X=...)` yields a picture whose *Y*
axis is the motion. Hence:

```yaml
gocator:
  mounting:
    scan_x: -Y      # sensor X (across the laser) -> gantry -Y
    scan_y: +X      # sensor Y (travel)           -> gantry +X
    scan_z: +Z      # sensor Z (range)            -> gantry +Z
```

With a mounting set, `to_points()` and **every export** come back in gantry
coordinates — a point's X is the same X you would command with
`move_to(X=...)`. The raw sensor frame stays reachable via
`to_points(frame="sensor")`, and `scan.grid_axes` reports which gantry axes
the grid's `(rows, cols)` run along (`("X", "Y")` here). Saved `.npz` files
record both `mounting` and `grid_axes` in their metadata, so
`visualize_scan.py` labels its axes correctly and a future reader never has
to guess which frame a file is in.

The default is the **identity map**, so nothing changes until you configure
a mounting. CLI override: `--mounting scan_x=-Y,scan_y=+X,scan_z=+Z`.

> **Verify the sign before trusting it.** The axis *pairing* is confirmed
> from data; whether sensor +X points to gantry +Y or −Y is not, and cannot
> be determined from the scans alone. Scan something asymmetric and check the
> result isn't rotated 180°. Getting it wrong rotates the data in-plane; it
> cannot mirror it, because **a mirroring map is rejected outright** — a bare
> X↔Y swap has determinant −1 and would reflect real geometry, so
> `SensorMounting` requires a proper rotation (flip exactly one sign).

## Matching feed rate to frame rate

`solve_scan_rates()` ties together the three quantities locked by
`y_spacing = feed_rate / frame_rate`. Give any two, get the third; give
fewer and the sensor fills in the rest:

```python
scanner.solve_scan_rates()                              # fastest isotropic feed
scanner.solve_scan_rates(feed_rate_mm_s=20)             # -> frame rate + Y spacing
scanner.solve_scan_rates(y_spacing_mm=0.05)             # -> feed rate at the ceiling
scanner.solve_scan_rates(feed_rate_mm_s=10, frame_rate_hz=100)   # -> Y spacing
```

With no arguments it returns the **fastest feed rate that still samples
travel at least as finely as across the laser** (Y spacing ≤ X resolution),
using the sensor's live frame-rate ceiling. That default exists because Y
spacing already comes out far finer than X resolution in every configuration
measured here — surplus frame rate is better spent on shorter scans than on
Y detail X can't match.

The result reports `travel_axis`: the **gantry** axis that feed rate applies
to (`X` with the mounting above, not `Y`). It also returns the live
`frame_rate_max_hz`, `aspect_ratio` (y_spacing / x_resolution) and
`isotropic`, and rejects a frame rate above the live ceiling with a pointer
to the knobs that raise it.

## Output data

The sensor returns a **dense grid**, not an unordered cloud — rows along Y
(travel), columns along X (across the laser line). Two message flavours are
handled:

| Message type | Contents | `SurfaceScan.is_uniform` | Emitted when |
|---|---|---|---|
| `UNIFORM_SURFACE` (8) | Z only per cell; X/Y implied by column/row index × resolution | `True` | `uniform_spacing: true` |
| `SURFACE_POINT_CLOUD` (28) | full x/y/z raw triple per cell (un-resampled) | `False` | `uniform_spacing: false` |

### Resampled heightmap vs. true point cloud

Which one you get is the `uniform_spacing` setting, and it is a real choice,
not a formatting detail. Internally the sensor always builds "a random 3D
point cloud where each individual point is an (X,Y,Z) coordinate triplet."
With uniform spacing **enabled** it then resamples that onto even X bins —
"the resampling divides the X-Y plane into fixed size square bins... points
that fall into the same bin are combined into a single Z value" — and
transmits Z only, because "the X positions can be reconstructed through the
array index at the receiving end." That is a **resampled heightmap**, and it
discards the native sample positions.

With uniform spacing **disabled**, no resampling happens and the sensor sends
an explicit (x, y, z) per point at its native, non-uniform X spacing — a
**true point cloud**. Disabling it also raises the achievable frame rate (LMI
lists "uniform spacing disabled" as part of the 2600-series high-speed
recipe), so it interacts with `frame_rate_max`.

Set it in config (`uniform_spacing: false`), per call
(`configure(uniform_spacing=False)`), or from the CLI
(`example_08 --point-cloud` / `--uniform-spacing`). Both message paths are
implemented and both land in the same `SurfaceScan`, distinguished by
`is_uniform`; note that for a point cloud `x_mm`/`y_mm` are full 2-D
per-cell arrays rather than 1-D axis vectors.

Source: [Uniform Data and Point Cloud Data](https://am.lmi3d.com/manuals/gopxl/gopxl-1.1/LMILaserLineProfiler/Content/TheoryOfOperation/Profile_RangeOutput/ResampledAndUniformSpacingProfile.htm),
plus `docs/reference/gocator/GOCATOR_CONCEPTS.md` §4.

Scaling, applied by `pointcloud.py`:

```
value_mm = offset_um / 1000 + resolution_nm / 1e6 * raw_count
```

Resolutions are nanometres, offsets micrometres, grid values 16-bit signed
counts. `0x8000` (-32768) means "no data" (occlusion, no return) and becomes
`NaN` in `z_mm` — `to_points()` drops those by default.

### Export formats

Measured on a real 2100x16154 scan (23.8M valid points), writing one format
at a time:

| Format | Method | Time | Size | Use for |
|---|---|---|---|---|
| `.laz` | `save_las()` | **0.5 s** | **14 MB** | **Default point-cloud export.** CloudCompare, PDAL, QGIS, laspy |
| `.npz` | `save_npz()` | 2.4 s | 30 MB | Reprocessing — the only format preserving the grid *and* no-data cells |
| `.ply` | `save_ply()` | 1.2 s | 286 MB | CloudCompare / MeshLab, when LAS isn't an option |
| `.las` | `save_las()` | 2.0 s | 476 MB | Uncompressed ASPRS, for tools without a LAZ backend |
| `.csv` | `save_csv()` | 55 s | 1.2 GB | Only when something downstream truly needs text |

`save_scan(scan, formats=("npz", "laz"))` — the default — writes several at
once under `output_dir` with a UTC-timestamped name, building the flattened
point array **once** and sharing it across every point-cloud format rather
than rebuilding it per format.

**CSV is ~100x slower and ~85x larger than LAZ for identical data.** It was
in `example_08`'s default set and dominated save time; the default is now
`npz,laz`, overridable with `--formats`.

LAS/LAZ stores coordinates as scaled int32 with a per-file scale/offset,
which fits this sensor well (its native output is already 16-bit counts plus
a resolution/offset). laguna writes **millimetres** with a 1e-4 mm quantum —
5e-5 mm round-trip error, ~240x finer than the 2690's 12 µm Z repeatability.
LAS has no unit field short of a CRS, and these scans aren't georeferenced,
so consumers must treat the values as mm.

LAS/LAZ needs an optional dependency:

```bash
pip install 'laguna[scanner]'
```

---

## Configuration

```yaml
gocator:
  ip: 192.168.1.10
  travel_speed_mm_s: 20.0     # MUST match the gantry feed rate
  frame_rate_hz: 500          # mutually exclusive with frame_rate_max below
  # frame_rate_max: true      # use the sensor's current max instead — see note below
  fixed_length_mm: 200.0      # fallback only — see note below
  exposure_us: null           # omit to leave the sensor's setting
  uniform_spacing: null       # true/false to force X resampling; omit to leave as-is
  output_dir: ./data/scans
  sdk_lib_dir: null           # omit to auto-discover
```

`fixed_length_mm` here is only a fallback. `scan_with_gantry()` derives the
real value itself from `end_mm` and the axis's position at call time
(`abs(end_mm - current)`), so the sensor's capture window matches the actual
commanded move rather than depending on this config value being kept in sync
by hand — a stale `fixed_length_mm` used to mean the sensor could stop
generating the surface well before (or long after) the gantry's move
actually finished. The config value is only used when the axis's position
can't be read at all, or when a caller passes `fixed_length_mm=` explicitly
to `scan_with_gantry()` (e.g. to deliberately scan only part of a longer
traverse — a mismatch against the derived distance is logged as a warning,
not rejected). `example_08`'s `--end-mm` flows through this same path.

Picking a frame rate: `Y spacing = travel_speed / frame_rate`. At 20 mm/s and
400 Hz that's 0.05 mm between profiles — far finer than the 0.124–0.55 mm X
resolution, so X is usually the limiting axis. Lower the frame rate (or raise
the feed rate) to trade Y density for scan time. Keep the value under the
sensor's live ceiling (443 Hz measured here, see above); `configure()` will
reject anything over it. `example_08 --frame-rate-hz` sets this from the
CLI, mutually exclusive with `--frame-rate-max` (see above) — omit both to
use the config file's `frame_rate_hz`/`frame_rate_max`, or whatever
frame-rate mode/rate the sensor already has if neither is set.

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
- `examples/example_08_gocator_surface_scan.py` — runnable end-to-end scan;
  read-only by default, requires `--allow-motion` to move the gantry

---

## Verified against hardware (2026-07-30)

Confirmed working from Python against the real sensor: SDK loads, all bound
symbols resolve, discovery finds the unit, connect/disconnect are clean, every
settings accessor returns sane values, and **`configure()`'s write path
applies the full recipe and persists it** — read back afterwards as
`surface` / `time` / `fixed_length` / `software`, with travel speed, fixed
length and frame rate all landing as set.

Still unverified: `GoSensor_Trigger()` and everything downstream of it (the
receive path, message-type handling, point-cloud conversion) — nothing has
produced an actual surface yet.

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
- **Why the frame-rate ceiling halves** when max-frame-rate mode is disabled
  is not understood — only that it does, reproducibly, with exposure
  unchanged. If high Y density matters, this is worth pinning down (try
  varying exposure, FOV, and uniform spacing and watching
  `sensor_frame_rate_max_hz`).
- **Travel speed writes to flash on every change.** `configure()` skips the
  write when the value is unchanged, but scanning at many different feed
  rates means many flash writes. Fine at experiment cadence; worth knowing
  before scripting a sweep over hundreds of speeds.
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
