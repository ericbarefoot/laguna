# Reference frames

Bring every instrument's data into one coordinate system, and name a *place*
instead of a robot position. Lives in `src/laguna/frames.py`, reachable as
`lab.frames`.

---

## The problem

Each instrument sees the world from where it is bolted. The Gocator's laser
line, the OD2000's dot and the WTT12L's dot sit at three different places on
the carriage, and the Gocator's own axes are rotated relative to the gantry's
(see [Scanner](scanner.md), "Sensor axes are not gantry axes"). Without a
common frame:

- "the same spot" means three different gantry positions,
- output coordinates from different instruments aren't comparable,
- and everything is expressed relative to the gantry's home, which is rarely
  where you want an experiment's origin.

---

## Two transforms, deliberately separate

**Mount** — per instrument, fixed by the hardware. Where that instrument's
measurement point sits relative to the gantry's *commanded* point, and how
its axes are turned. Measure once when the rig is built or an instrument
moves.

**Experiment frame** — per project, chosen by you. A rigid transform from
gantry coordinates to whatever origin and orientation the experiment wants,
typically a flume corner so every coordinate comes out positive. Changing it
re-labels all output without touching any mount.

They compose in one direction:

```
experiment_point = experiment_from_gantry @ translate(gantry_position)
                                         @ mount_i @ sensor_reading
```

and invert for positioning, which is what makes "put the other sensor here"
work.

Everything is millimetres. **Every transform must be rigid** — rotation plus
translation. A scale or shear is rejected, because it would silently distort
real geometry and nothing here has a legitimate use for one. Mirroring is
rejected for the same reason it is in `scanner.mounting`: a reflection cannot
describe a physical mounting.

---

## Configuration

```yaml
frames:
  experiment:                     # gantry -> experiment
    translation: [500, 300, 0]    # origin at a flume corner
    rotation_deg: 0               # about Z, optional
  instruments:                    # measured once, when the rig is built
    gocator:
      translation: [0, 0, -325]   # translation ONLY — see the warning below
    od2000:
      translation: [52.0, -18.0, 0]
    wtt12l:
      translation: [52.0, 31.0, 0]
```

Omit the section entirely for identity transforms and zero offsets — a rig
without it behaves exactly as it did before frames existed. An instrument
with no entry is assumed to measure at the gantry's commanded point.

Per-instrument keys: `translation`, `rotation_deg`, `axes` (a sensor axis map,
same form as `gocator.mounting`), `matrix` (an explicit 4×4), and
`reference_point` for a sensor whose measurement point isn't at its own frame
origin. `rotation` is applied before `translation`, so a translation always
reads in the target frame — "move the origin over there".

> **Don't put the Gocator's axis map in both places.** `SurfaceScan` already
> rotates its points into gantry orientation using `gocator.mounting`. If
> `frames.instruments.gocator` also carries `axes` or `rotation_deg`, the
> data would be turned twice. `orient_scan()` detects that combination and
> raises rather than returning quietly-wrong geometry — keep the rotation in
> `gocator.mounting` and give the frame a translation only.

---

## Naming a place instead of a robot position

```python
lab.place("od2000", [100, 200, 0])     # OD2000's dot on the target
lab.place("wtt12l", [100, 200, 0])     # WTT12L's dot on the SAME physical spot
```

Same experiment coordinate, different gantry commands — differing by exactly
the two mounts' offset. This is what lets you re-run a transect with a second
instrument by naming the transect rather than recomputing robot positions.
`lab.place()` leaves Theta untouched, since it is outside the Cartesian frame
model.

`move_to()` still works exactly as before for commanding the robot directly.

Related helpers:

```python
lab.frames.gantry_target_for("od2000", [100, 200, 0])   # what place() computes
lab.frames.retarget("od2000", "wtt12l", gantry_position)  # swap instruments in place
lab.frames.to_experiment("od2000", reading, gantry_position)
lab.frames.gantry_to_experiment(points)
lab.frames.experiment_to_gantry(points)
lab.frames.describe()                                   # offsets, for logs/status
```

`retarget()` is the direct form of "re-scan that with the other sensor" when
you have the original gantry position rather than an experiment coordinate.
It is independent of the experiment frame — purely the difference of the two
mounts.

---

## Placing a Gocator surface

A surface's own coordinates are relative to where the pass began: the travel
axis runs about −length/2 … +length/2, not from the gantry position. Placing
one therefore needs the pass's starting gantry position, which
`scan_with_gantry()` already records:

```python
from laguna.frames import orient_scan

oriented = orient_scan(scan, frames=lab.frames)                            # SurfaceScan, experiment mm
oriented = orient_scan(scan, frames=lab.frames, gantry_start=[700, 0, 0])  # explicit start
oriented = orient_scan(scan, frames=lab.frames, output="scan.laz")        # also write a file
```

`orient_scan()` returns a new `SurfaceScan` — the transform can rotate
(experiment `rotation_deg`, or an instrument `axes` map), which a uniform
grid's compact `x_mm`/`y_mm` centre arrays can't represent once every cell's
X/Y no longer lines up with its row/column, so the result is always per-cell
(`is_uniform=False`) with `mounting` reset to identity: the transform is
already baked into the stored coordinates. The original `scan` is untouched.
Same shape as `laguna.robot.macron.profiler.orient_profile()`, the
rangefinder equivalent (named differently on purpose — same-named imports
from two modules forced an `as` alias every time) — raw object in, same
type out, optional `output=` file.

**Travel direction matters, separately from mounting.** The Gocator is
encoderless — its own Y is just acquisition order (first frame captured to
last), not tied to any real-world direction. `orient_scan()` anchors the
first-acquired point to the pass's real starting position and orients
everything else by the recorded `gantry_start_mm -> gantry_end_mm` direction
for *that* pass. This is independent of `gocator.mounting`'s rotation, which
is a fixed rig constant and doesn't vary by pass — see
[scanner.md](scanner.md), "Y is acquisition order, not a lab-frame
direction," for the mechanism and what happens if you place scans some
other way.

---

## FYI: the Gocator has a source-level mirror setting (skeleton — WIP)

> **Status: not yet fully characterized.** Filed here as a placeholder while
> this gets pinned down properly, with screenshots from the device's web UI.
> Don't treat the specifics below as settled — the short version is: this
> setting exists, it does what it sounds like, and getting it wrong looks
> exactly like a `gocator.mounting` sign error, which cost real time to
> untangle (2026-08-11/12) before the actual cause was found.

**The setting.** Somewhere in the Gocator's own web UI (exact location TBD —
screenshots go here) there's a toggle — something like Normal/Reverse —
that flips the sensor's own data readout horizontally *at the source*,
before any of it reaches `laguna`. This is a genuine data-level mirror
(reversed CCD/readout order), not an axis relabeling.

<!-- TODO: screenshot(s) of the setting in the Gocator web UI here -->
<!-- TODO: confirm the exact menu path — earlier guess (Manage > Layout /
     "Layout Types") was wrong; that page is GoLayout/GoOrientation, for
     multi-sensor buddy systems, unrelated to this. -->

**Why this matters for `gocator.mounting`.** `SensorMounting` (see
[Scanner](scanner.md) and `laguna/scanner/mounting.py`) only ever accepts a
proper rotation (determinant +1) — it deliberately refuses to encode a
mirror, because a rigid, physically-mounted sensor can only ever be
*rotated* relative to the gantry, never reflected (see that module's
docstring for the geometric argument). That's correct — **provided the raw
data reaching `laguna` is already a faithful, non-mirrored view of
reality**. If the device's own setting is mirroring the data at the source,
no choice of `mounting` can fix it: a rotation cannot undo a reflection.
The fix has to happen at the source (this device setting), not by trying to
smuggle a mirror into `mounting` (which the code correctly won't allow).

**Open questions, still unresolved as of 2026-08-11/12:**

- Exact UI location and label of the setting.
- Whether "Normal" or "Reverse" is the non-mirrored state — this flipped
  more than once while diagnosing it, and the device is currently set to
  "Normal." Needs a clean, repeatable check (e.g. scan a known-asymmetric
  object, confirm the geometry — not just "looks plausible") rather than
  going by feel.
- Whether this is a per-sensor, per-session, or persistent (flash-saved)
  setting — i.e. whether it can silently reset and re-break this.
- Once settled: record the confirmed state here **and** as a comment next
  to `gocator.mounting` in config, so a future sign-chasing session doesn't
  repeat this one — check the device setting *first*, before touching
  `mounting`'s signs.

---

## Working out the offsets

The offsets are physical measurements, and the numbers above are
placeholders. Two practical approaches:

1. **Direct measurement.** With the gantry at a known position, measure from
   the commanded point to each instrument's laser dot / line centre.
2. **Common-target calibration.** Put a distinctive small target in the work
   area. For each instrument, jog until it reads the target, and record the
   gantry position. The differences between those positions are exactly the
   differences between mounts, which is what `retarget()` needs; anchoring
   one instrument by direct measurement then fixes all of them.

Verify with the round trip — ask for a target, move, and confirm the reading
lands where you asked:

```python
target = [100.0, 200.0, 0.0]
lab.place("od2000", target)
# ... take a reading; it should correspond to `target` in experiment coords
```
