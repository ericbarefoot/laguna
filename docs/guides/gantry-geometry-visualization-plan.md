# Plan: visualizing the gantry's physical geometry

Status: **not started** — this is a design sketch, not a commitment. Written
after discovering that `laguna.viz`'s fence overlay (checked against the
gantry's *commanded* point) doesn't represent where the machine's metal
actually is: the Z axis carries an arm on the order of a meter long, so a
fence configured at `z: [360, 1000]` isn't "the exclusion zone is between
360mm and 1000mm of real height" — it's a boundary on the *commanded* Z
coordinate, empirically placed by whoever configured it so that the whole
arm (commanded point plus everything hanging below/around it) stays clear
of the HVAC unit. The fence geometry is a proxy for a swept volume, not the
volume itself.

## Why this is a bigger project than it looks

1. **No physical dimensions exist anywhere in the codebase today.**
   `config/*.yaml`'s `axes:` blocks carry `name`/`index`/wiring only (see
   `config_scan.yaml:64-75`) — no carriage envelope, no arm length, no
   mounted-instrument offset beyond the `frames:` translation (which places
   a single *point* — the measurement point — not a volume). Building this
   means introducing an entirely new kind of config data.

2. **Travel limits (PLT/NLT) are live-only, not config.** As covered
   elsewhere in `laguna.viz`'s docstring: axis soft limits only exist as
   queryable hardware state (`AxisState.positive_limit`/`negative_limit`,
   `commands.py`), read via the `PLT`/`NLT` ASCII commands. A geometry
   visualization that wants to draw the *full rail extent* (not just
   wherever the carriage happens to be right now) needs these, and they
   aren't available offline today.

3. **This borders on a safety question, not just a visualization one.**
   `TrajectoryChecker` (`fences.py`) currently checks only the commanded
   point against fence geometry. If the real goal is "does any part of the
   machine enter this zone," the *correct* fix is arguably in fence-checking
   itself, not just in a plotting utility — but that's squarely inside
   `fences.py`, which the project's own guidance (`CLAUDE.md`) flags as
   requiring explicit sign-off before touching: it's the thing that keeps
   real motion from ramming into a real HVAC unit. This plan deliberately
   scopes itself to **visualization only** (informational, non-gating) and
   treats any change to actual exclusion-checking as a separate, later,
   carefully-reviewed decision — not something to fold in here.

## What a first version would need

**A static geometry model**, even a rough one — not CAD-accurate, just
enough bounding boxes to answer "does this look like it clears the HVAC":

- **Carriage envelope**: a fixed-size box (X/Y/Z extent) centered on the
  commanded X/Y point, representing the physical carriage body.
- **Z-arm extent**: how far the mechanism actually reaches below (and
  possibly above) the commanded Z point — this is the dimension that
  matters most for the motivating scenario, and the cheapest to model
  first (a single length, not a full 3D shape).
- **Mounted instrument offset**: already partially captured by
  `frames.instruments.<name>` (a point, via `InstrumentFrame.offset`) —
  would need widening to a small volume (the instrument's own physical
  footprint) if the instrument itself is the thing that could strike
  something.
- **Rail extents**: PLT/NLT per axis, to draw the full travel envelope
  rather than just the current/planned position.

**Where this data would live**: a new `gantry_geometry:` (or similar)
config block, sibling to `frames:` — e.g.:

```yaml
gantry_geometry:
  carriage_envelope_mm: {x: 200, y: 200, z: 100}   # box, centered on commanded X/Y/Z
  z_arm_length_mm: 1000                             # how far below commanded Z the arm reaches
  # rail extents: either mirrored from a one-time PLT/NLT read (with a
  # warning if the live value ever disagrees), or entered by hand
  rail_limits:
    X: {min: 0, max: 1800}
    Y: {min: 0, max: 1000}
```

Whether rail limits should be *sourced from* a live PLT/NLT read (so config
can't silently drift from hardware truth) or entered independently is an
open question — mirroring live state into config read-only, with a
loud mismatch warning, seems safer than letting two sources of truth exist
unreconciled.

## Rendering approach — incremental

**Phase 1 (cheapest, answers the motivating question directly):** extend
`laguna.viz`'s existing 2D XY/XZ/YZ layout. At a given commanded point,
draw the Z-arm as a fixed-height vertical column (commanded Z down to
`commanded Z - z_arm_length_mm`) on the XZ/YZ panels, using the same
`Rectangle` patch style already used for `Landmark`. No 3D rendering, no
carriage box yet — just "if I command this, does the arm's swept column
clear this obstacle."

**Phase 2:** add the carriage envelope as a box around the commanded X/Y,
and instrument footprints as small boxes at their `frames.instruments`
offsets. Still the same 2D panel layout — three 2D projections of several
boxes is tractable without a 3D library.

**Phase 3 (bigger, likely a separate decision to even pursue):** true 3D
rendering (`mpl_toolkits.mplot3d` or a proper 3D library) showing the whole
assembly posed along a `plot_trajectory()` path or animated through a
`Survey`'s passes — this is the "see the robot move through the flume"
version, and where most of the real effort lives (pose composition per
waypoint, likely wants its own module rather than living inside
`laguna.viz`).

## Relationship to `plot_trajectory()`

`laguna.viz.plot_trajectory()` (added alongside this plan) is scoped to
drawing **planned paths** (a `Survey`'s `Pass` list, already in experiment
coordinates) plus `Landmark` boxes — no swept-volume/arm geometry yet. It's
the natural place to eventually plug Phase 1/2 geometry in once it exists:
draw the arm's swept column at each pass's start/end, not just a bare line.
Direct G-code text preview (rather than a `Survey`'s already-resolved
`Pass` list) is also out of scope for now — G-code coordinates are gantry
frame, not experiment frame, and resolving that reintroduces the same
kind of frame-transform question this plan exists because of.

## Suggested next step, if picked up

Phase 1 only: add `z_arm_length_mm` (a single number) to config, read it in
`plot_trajectory()`/`plot_acquisition()` optionally, and draw the swept
column. Small, testable, and directly answers "would the arm have hit the
HVAC" without committing to the config schema or rendering approach a full
carriage+rail model would need.
