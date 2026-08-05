# Scanner

LMI Gocator 2690 line-laser 3D surface scanner. Lives in
`src/laguna/scanner/` — `gosdk.py` (ctypes binding), `gocator.py`
(acquisition lifecycle), `settings.py` (configuration surface),
`pointcloud.py` (surface -> point cloud + export), `mounting.py`
(sensor-to-gantry axes).

Distinct from [Rangefinder](rangefinder.md): the OD2000/WTT12L are
single-point distance sensors reached through a Pi-side IO-Link master over
MQTT/HTTP. The Gocator is a line-laser 3D scanner on the laguna PC's own
Ethernet network — no Pi, no IO-Link, no MQTT, no deployed scripts.

**First time here?** Start with [Scanner setup and tuning](scanner-setup.md)
— you need the SDK built and a `gocator:` config section before any of this
runs. That page also holds the config reference and the frame-rate tuning
results.

Two ways in:

- `scripts/gocator_scan.py` — the full-control CLI: every sensor knob plus a
  coordinated gantry pass. Read-only unless you pass `--allow-motion`.
- `examples/example_08_gocator_surface_scan.py` — a short, readable version
  of the same thing.

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

lab = FlumeLab("config/example_config.yaml")

lab.add("gocator").add("gantry")
lab.connect_all()

# Coordinated pass: configures travel speed from feed_rate_mm_s, derives
# fixed_length_mm from end_mm and the axis's current position (overriding
# gocator.fixed_length_mm in the config — see scanner-setup.md), starts
# a non-blocking move, waits out the accel ramp, triggers, receives the
# surface.
scan = lab.gocator.scan_with_gantry(
    lab.gantry, axis="X", end_mm=400.0, feed_rate_mm_s=20.0, settle_s=0.5
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
  `scripts/gocator_scan.py` will configure (it forces `transport: pi_agent` itself,
  regardless of what the config file says). It launches `gantry_agent.py`
  over SSH itself, so there's no manual Pi-side step and nothing else to
  start first. The retired `socket_bridge` transport depended on
  `serial_bridge.py`, a hand-started script on the Pi that never survived a
  reboot and could not do topographic scanning at all — see
  `docs/MACRON_GANTRY.md`, "Retired: serial_bridge.py".
- **`safe_mode: false`.** Every motion command is gated by it, so a scan
  cannot run with it on. `scripts/gocator_scan.py` owns this itself — it derives
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
(`gocator_scan.py --point-cloud` / `--uniform-spacing`). Both message paths are
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
in `gocator_scan.py`'s default set and dominated save time; the default is now
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
- `scripts/gocator_scan.py` — full-control CLI: every sensor knob plus the
  coordinated gantry pass. Read-only by default; needs `--allow-motion` to move.
- `examples/example_08_gocator_surface_scan.py` — the short, readable version
  of the same thing, for learning the subsystem

---

