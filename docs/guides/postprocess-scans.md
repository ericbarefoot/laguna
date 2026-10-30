# Post-processing saved scans into DEMs

`laguna.postprocess` turns a folder of saved scans (`.laz`, e.g. from
`SurfaceScan.save_las`) into ground-surface DEMs that all share one grid,
so any two can be differenced cell by cell. It runs offline, with no
hardware attached. The point-cloud work is done by
[LAStools](https://rapidlasso.de/lastools/); Python handles the settings,
the region shapes, scheduling and the final masking.

Each scan goes through these steps:

1. **Experiment frame** (optional): move the points from the scan's own
   coordinates into the experiment frame with a rigid transform.
2. **ROI**: keep the ROI rectangle and an optional z window (`las2las`).
3. **Include/exclude shapes**: keep only points inside the include
   polygons, if there are any, and drop points inside any exclude shape.
4. **Denoise** (`lasnoise`): remove isolated points.
5. **Ground filter** (`lasground_new`): keep the ground surface and drop
   debris and scan artefacts sitting on it.
6. **DEM** (`las2dem`): grid on the ROI's fixed grid, then set the cells in
   excluded areas to nodata.

Denoising runs *before* ground filtering on purpose. The ground filter
seeds its surface from the lowest points, so stray low returns left in the
data become ground and drag the surface down around them. On the first tray
dataset, running the ground filter first left pits up to 18 mm deep.
Exclusion zones are applied before both steps, so an obstruction that
distorts the scan can't distort the surface around it either.

## Setup

```bash
pip install -e '.[postprocess]'
```

LAStools isn't on PyPI. Download the Linux binaries from rapidlasso and put
the `bin/` folder on `PATH`, or point `LASTOOLS_BIN` at it. The binaries
have a `64` suffix (`las2las64`). `lasnoise`, `lasground_new` and `las2dem`
need a license beyond a few million points per file, so copy
`lastoolslicense.txt` into the same `bin/` folder. Without it these tools
add noise or refuse to run, and the pipeline reports the scan as failed.

## Settings

Settings live in a YAML file of their own. This is not the rig config,
because how you process an experiment's scans usually changes after the
scans are taken. Start from `examples/example_postprocess.yaml`; it
documents every key. All lengths are in mm, as written by `save_las`.

### Experiment frame

A saved `.laz` is in scanner coordinates: the travel axis is centred on the
pass rather than on the flume. Set `frame` to a rigid transform that maps
those coordinates into the experiment frame. It uses the same keys as
`frames.experiment` in the rig config (`rotation_deg` about Z, then
`translation` or `origin`, or a 4x4 `matrix`):

```yaml
frame: {rotation_deg: 0, origin: [320, -480, 20]}   # this scan point becomes (0, 0, 0)
```

The frame is applied first, so `roi`, `z_range`, the shapes and the DEM grid
are all in experiment coordinates. If you change `frame` you have to redraw
the shapes as well, and every scan is reprocessed. Leave `frame` unset to
work in scanner coordinates.

`lasnoise` and `lasground_new` grid the points from the coordinate origin.
A frame that shifts by a multiple of `noise.step_xy` and `ground.step` (for
example 30 mm with the defaults) gives exactly the DEM you would get in
scanner coordinates. Any other shift can change the result by a fraction of
a millimetre in a few percent of cells.

## Drawing the ROI and exclusion zones

`examples/example_16_postprocess_roi_editor.ipynb` plots a scan with
`ShapeEditor` and lets you draw the ROI rectangle and the include/exclude
polygons, rectangles and circles on it. **Preview DEM** runs the whole
pipeline on that one scan, so you can check a shape's effect straight
away. **Save config** writes the YAML. The editor needs `ipympl`
(`%matplotlib widget`).

## Running

From the shell:

```bash
laguna-postprocess run  ~/data/run_042 -c ~/data/run_042/postprocess.yaml
laguna-postprocess run  ~/data/run_042 -c ~/data/run_042/postprocess.yaml -f   # redo everything
laguna-postprocess status ~/data/run_042 -c ~/data/run_042/postprocess.yaml    # what is stale
```

From Python:

```python
from laguna.postprocess import Circle, load_postprocess_config, run

cfg = load_postprocess_config("postprocess.yaml")
cfg.exclude.append(Circle(x=600, y=0, r=20, label="bolt"))
results = run(["~/data/run_042"], cfg)        # list[ScanResult], one per scan
```

A scan is skipped when its outputs came from the same input file and the
same settings. Shape labels, `output_dir` and `cores` don't count as
settings. One bad scan is reported as `status="failed"` and doesn't stop
the rest of the batch.

## Outputs

Everything goes under `output_dir`, which is relative to the YAML file:

| Path | Contents |
|---|---|
| `dem/<scan>.tif` | float32 DEM, `dem.step` cells, nodata = `dem.nodata` |
| `clip/<scan>.laz` | points that fed the DEM (class 2 = ground) |
| `meta/<scan>.json` | settings used, input fingerprint, point counts |
| `logs/<scan>.log` | LAStools output |
| `manifest.csv` | one row per processed scan, with point counts after each stage |

`dem_nodata_frac` is the share of empty cells in the area you kept, so it
leaves out the excluded areas. Empty cells come from gaps wider than
`dem.kill` between scan lines; raise `kill` to fill more of them.
