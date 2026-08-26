# Gantry config parameter reference

A catalog of confirmed, hardware-measured `gantry:` config values —
`mm_per_unit` calibration, home-switch polarity, soft limits, and known
fences — so a new config file (a new basin, a remount, a rebuilt rig)
starts from what's already been measured instead of rediscovering it.

**This is a reference, not a template to copy blindly.** Soft limits and
fences describe a specific physical setup at a specific point in time —
gantry position on its rails, mounting height, what's actually in the
work envelope. If the rig moved, was remounted, or you're setting up a
different basin, treat every number below as a starting guess to
re-verify, not a known-good value. See [Driving the gantry
safely](guides/gantry-motion.md) for how to measure these safely from
scratch (jog cautiously, watch the limits, confirm before trusting).

## How to use this page

1. Copy the relevant block below into your config file's `gantry:`
   section.
2. Re-verify anything that depends on physical position (soft limits,
   fences) before trusting it — see each section's "confirmed" date and
   what it was confirmed against.
3. Add a new dated entry here, in the same format, whenever you confirm a
   new value on real hardware — that's the whole point of this page:
   don't let this knowledge live only in a config comment or someone's
   memory.

## Axis scale — `mm_per_unit`

Raw controller (ACP) units don't map 1:1 to real mm — `mm_per_unit`
converts. See
[`docs/archive/GANTRY_UNIT_CALIBRATION.md`](archive/GANTRY_UNIT_CALIBRATION.md)
for the measurement method.

| Scope | Value | Confirmed | Notes |
|---|---|---|---|
| Shared default (`mm_per_unit:`, top-level) | `15.0` | 2026-07-28 | Originally measured uniform across X/Y/Z. |
| Z override (`axes: - {name: Z, mm_per_unit: ...}`) | `13.5` | 2026-08-10 | The "uniform across axes" finding above didn't hold up under a second measurement — Z specifically needed its own override. **Don't assume X/Y are still exactly 15.0 either** without re-checking; only Z has been re-measured since the original finding. |

```yaml
mm_per_acp_unit: 15.0   # shared default
axes:
  - {name: Z, index: 5, mm_per_unit: 13.5}   # Z-specific override
```

This is flagged `TEMPORARY` in both shipped config files — see the
comment above `mm_per_acp_unit:` in `config/example_config.yaml` for the
plan to retire it once the DSM project's axis scale is fixed at the
source.

## Home-switch polarity

| Parameter | Value | Confirmed | Notes |
|---|---|---|---|
| `home_trip_on_high` | `false` | 2026-08-25 | Normally-closed wiring — switch reads LOW when tripped. Confirmed for X/Y/Z's home switches on this hardware. |
| `home_switch` | `home` (default) | — | Jogs toward `INB 1/3/5`. `limit` jogs toward `INB 2/4/6` instead — see [Motion control layers & guards](MOTION_CONTROL_LAYERS.md). |

```yaml
axes:
  - {name: X, index: 1, home_switch: home, home_trip_on_high: false}
```

No reason to expect this to change across basins/remounts (it's about
switch wiring, not gantry position) — but re-confirm if the controller or
its wiring is ever touched. See
`examples/example_09_gantry_home_axis_test.py` for a per-axis homing test
script with a Ctrl-C safety stop, and
`examples/example_08_gantry_io_verify.py` for a live INB toggle check.

## Soft limits — `NLT`/`PLT`

Per-axis travel bounds, written to the controller on `connect()` with
`safe_mode=False` — see `GantryController._apply_soft_limits()` and the soft-limits note in
[MACRON_GANTRY.md's Digital IO
section](MACRON_GANTRY.md#digital-io-read-this-before-touching-brakes-or-limit-switches).
**Position-dependent**: tied to
wherever the gantry's raw zero currently is, which changes if the rig is
re-homed against a different reference or physically remounted.

| Setup | X (mm) | Y (mm) | Z (mm) | Confirmed | Source |
|---|---|---|---|---|---|
| `config/example_config.yaml` | −5 → 2130 | −5 → 1200 | 0 → 520 | 2026-08-25 | shipped config |
| `config/config_scan.yaml` (Basin A) | −5 → 2130 | −5 → 1200 | 0 → 470 | 2026-08-25 | shipped config + interactive session notes ("zmax is 470 zmin is 0") |

**Z disagrees between the two (520 vs. 470) and that hasn't been
reconciled** — don't copy one over the other without re-measuring which
is actually correct for your setup; they may simply be two different
physical configurations (different Z-axis mounting/reach) rather than one
being wrong.

```yaml
axes:
  - {name: X, index: 1, soft_negative_limit_mm: -5, soft_positive_limit_mm: 2130}
  - {name: Y, index: 2, soft_negative_limit_mm: -5, soft_positive_limit_mm: 1200}
  - {name: Z, index: 5, soft_negative_limit_mm: 0,  soft_positive_limit_mm: 470}   # or 520 — see above
```

## Fences — known exclusion zones

Pure-Python keepout boxes, checked before any move reaches the wire — see
`laguna.robot.macron.fences.TrajectoryChecker` and [Driving the gantry
safely](guides/gantry-motion.md). Same position-dependence caveat as soft
limits: a fence describes a real obstacle's location relative to the
gantry's current coordinate origin.

| Name | Basin | Box (x / y / z, mm) | What it is | Status |
|---|---|---|---|---|
| `hvac` | C | x:[650,1400] y:[115,900] z:[360,1000] | HVAC register in that quadrant | Commented out in both shipped configs — re-enable once re-verified against the basin currently in use. |
| `sirius` | A | x:[1080,1430] y:[430,850] z:[295,500] | Air intake at the south end of Basin A, above the robot arm's normal reach | Added to `config/config_scan.yaml` 2026-08-25, **not yet committed** — still being validated interactively. |

```yaml
fences:
  # - {type: box, name: hvac, x: [650, 1400], y: [115, 900], z: [360, 1000]}
  - {type: box, name: sirius, x: [1080, 1430], y: [430, 850], z: [295, 500]}
```

Swap `box` for `laguna.robot.macron.fences.CylinderFence` if the obstacle
has a circular footprint (a post) instead of a rectangular one — see
`examples/example_07_flumelab_gantry_scan.py`'s fence-demo section.

## Open items

- Reconcile the Z soft-limit disagreement (520 vs. 470) above.
- Confirm/commit the `sirius` fence once validated.
- Re-check X/Y's `mm_per_unit` (still assumed 15.0 from the original 2026-07-28 measurement — only Z has been independently re-confirmed since).
