# Plotting saved scans and profiles with `plot_acquisition()`

`laguna.viz.plot_acquisition()` gives a four-panel sanity check (XY/XZ/YZ
footprint plus a Z heatmap or height-vs-travel plot) for a Gocator scan or a
rangefinder transect. It's most useful *after* a run, in a fresh Python
session or notebook with no hardware attached at all — reload what was
saved to disk, orient it, and look at it. This walks through that path for
both instrument types, plus today's `Tile.stitch()` output. Every snippet
below was actually run while writing this page.

Needs the `viz` extra: `pip install 'laguna[viz]'`.

## The one thing that trips people up

`plot_acquisition()` only understands **experiment-frame** data. A raw
scan or profile fresh off the sensor is in sensor/gantry-local coordinates
— it has to go through `laguna.frames.orient_scan()` (Gocator) or
`laguna.robot.macron.profiler.orient_profile()` (rangefinder) first. Skip
that step and the plot still renders — `plot_acquisition()` won't refuse —
but it logs a warning and the landmark overlays won't line up with the
data, since they're real-world flume coordinates and the data isn't yet.
Both orient functions need to know where the gantry was when the pass
happened, which is why they ask for the pass's `gantry_start`/`gantry_axis`
— present automatically if the scan was saved through the normal
`FlumeLab`/`SurveyRunner` path (see below for where it lives on disk).

## Getting a `FrameRegistry` without connecting to anything

Orienting data needs the lab's frame configuration, not the lab itself —
build a `FrameRegistry` straight from the config file:

```python
from laguna.config import Config
from laguna.frames import FrameRegistry

cfg = Config("config/config_scan.yaml")
frames = FrameRegistry.from_config(cfg.config_dict.get("frames"))
```

No `connect_all()`, no hardware, works offline. If you already have a live
or `simulate=True` `FlumeLab` in the same session, `lab.frames` is the
same thing — skip rebuilding it.

## A saved Gocator scan

Only `.npz` round-trips the metadata `orient_scan()` needs (`gantry_axis`,
`gantry_start`, the mounting) — LAZ/PLY/CSV flatten that away on save. If a
pass only wrote LAZ, there's nothing left to reorient; save `.npz`
alongside whatever export format you actually want (`gocator.scan.formats`
in config, or `formats=` on `save_scan()`).

```python
from laguna.scanner.pointcloud import SurfaceScan
from laguna.frames import orient_scan
from laguna.viz import plot_acquisition, Landmark

raw = SurfaceScan.from_npz("data/scans/scan_20260812_120000_000.npz")
oriented = orient_scan(raw, instrument="gocator", frames=frames)

fig, axes = plot_acquisition(
    oriented,
    landmarks=[Landmark("flume wall", 0, 3000, 0, 1200, -200, 200)],
    title="scan_20260812_120000_000",
)
fig.savefig("scan_check.png")
```

`axes` is the `[[xy, xz], [yz, data]]` 2x2 array if you want to tweak
limits or add anything after the fact.

## A stitched `Tile` survey

`Tile.stitch()`'s merged `SurfaceScan` (see today's addition to
`laguna.survey`) is already in experiment coordinates — every source pass
went through `orient_scan()` internally before being concatenated — so it
plots directly, no separate orient step:

```python
from laguna.viz import plot_acquisition

merged = tile.stitch(done, runner.results, lab.frames)   # done, runner.results from run(keep_results=True)
fig, axes = plot_acquisition(merged, title="stitched tile")
```

A flat `(1, N)` grid like this used to break the heatmap panel's
downsampling specifically — it split the budget across a row stride locked
at 1 (only one row) and a 1200-point column stride, capping the whole
panel near ~1200 plotted points regardless of how large `N` actually was.
Fixed; see the next section for how downsampling works now.

## Speeding up plotting on a big scan

Every panel — the three XY/XZ/YZ footprint scatters and the data panel —
downsamples independently (a deterministic stride, not random) to a shared
per-panel budget. Two knobs control it, and only one takes effect at a
time:

```python
# A quick, cheap look at a huge stitched scan — ~1% of each panel's points.
plot_acquisition(merged, sample_fraction=0.01)

# Or cap each panel at an absolute count instead (the default is 480,000).
plot_acquisition(merged, max_points=50_000)
```

`sample_fraction` (a proportion, `(0, 1]`) takes precedence over
`max_points` when both are given. Neither changes what's *in* the returned
`SurfaceScan`/DataFrame — only what gets drawn — so re-plotting the same
`merged` object with a different `sample_fraction` costs nothing beyond
the second plot call.

Two more knobs, both cosmetic: `cmap` (default `"viridis"`) recolors the
data panel's heatmap/point-cloud scatter — any matplotlib colormap name;
`figsize` (default `(11, 9)`) sizes a newly created figure — ignored if you
pass your own `fig=`.

## A saved rangefinder profile

Rangefinder passes (OD2000/WTT12L) write a CSV plus a `_meta.json` sidecar
next to it (same stem, `.csv` → `_meta.json`) carrying `gantry_axis`/
`gantry_start` — load that sidecar into `ProfileResult.metadata` before
orienting:

```python
import json
from pathlib import Path
from laguna.robot.macron.profiler import ProfileResult, orient_profile
from laguna.viz import plot_acquisition

csv_path = Path("data/scans/profile_od2000_20260812_120000_000.csv")
meta_path = csv_path.with_name(csv_path.stem + "_meta.json")
result = ProfileResult(path=csv_path, metadata=json.loads(meta_path.read_text()))

oriented_df = orient_profile(
    result,
    instrument="od2000",
    frames=frames,
    config=None,   # pass lab.config.get("od2000") for a calibrated height_mm column
)
fig, axes = plot_acquisition(oriented_df, title=csv_path.stem)
```

Without a calibration, `orient_profile()` logs a warning and
`experiment_z_mm` reflects only the constant mount offset, not a real
height — pass `config=lab.config.get("od2000")` (or `calibration=`
directly) if a `calibration_file` is set up, and the data panel switches
from "no calibration" to real calibrated height automatically.

Scans made before `orient_profile()`'s `gantry_start`/`gantry_axis`
sidecar persistence landed won't have a usable `_meta.json` — pass
`axis=`/`gantry_start=` to `orient_profile()` explicitly for those.

## Landmarks are visual only

`Landmark` boxes (flume walls, a known obstacle) are experiment-coordinate
reference rectangles drawn on each XY/XZ/YZ panel — plain visual aids, not
connected to `laguna.robot.macron.fences` in any way. Measure/eyeball their
bounds directly; there's no automatic derivation from fence config.

## See also

- [`laguna.frames.orient_scan`](../reference/frames.md) /
  [`laguna.robot.macron.profiler.orient_profile`](../reference/rangefinder.md)
  — full docstrings, including the travel-direction handling
  `orient_scan()` needs `gantry_start`/`gantry_end` for.
- [Visualization API reference](../reference/viz.md) — generated signatures
  for `plot_acquisition()`/`plot_trajectory()`/`Landmark`.
- `scripts/visualize_scan.py` — a lower-level, dependency-free quick-look
  tool that reads a Gocator `.npz` directly without going through
  `orient_scan()` (sensor-frame axes, not experiment-frame) — useful for a
  fast "did the laser even fire" check before bothering with frames at all.
