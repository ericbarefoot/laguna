# Finding: Gantry ACP Units Are Not Millimeters

**Status as of 2026-07-28: confirmed on hardware, not yet fixed at the
source. A stopgap conversion constant was patched into
`examples/example_05_gantry_single_axis.py` only — nowhere else in the
codebase accounts for this yet.**

---

## The finding

Every part of this codebase that talks to the Macron gantry (`commands.py`,
`gantry_agent.py`, `TopographicProfiler`, the `fences`/`homing` config
sections, all narrative docs) assumes **1 `ACP` unit = 1 mm**. That
assumption is wrong on the physical machine.

Confirmed by ruler-measured manual moves on 2026-07-28:

| Axis | Commanded move (ACP units) | Measured physical travel | Ratio |
|------|----------------------------|---------------------------|-------|
| Z    | 10 units (20 → 10)         | 150 mm (15 cm)            | 15 mm/unit |
| X    | 20 units                   | 300 mm (30 cm)             | 15 mm/unit |
| Y    | 20 units                   | 300 mm (30 cm)             | 15 mm/unit |

All three axes give the **identical** 15 mm/unit ratio — this is not
measurement noise, it's a single consistent scale factor, almost certainly
from one shared axis-configuration parameter (electronic gearing ratio,
counts-per-mm, or ballscrew-pitch entry) applied uniformly across X/Y/Z in
the Snap2Motion/DSM project.

Theta (rotary) is unaffected by this finding — it was already known to have
non-mm units (~10.365 ACP units per revolution, measured separately via
limit-switch-to-limit-switch rotation on the same date — see
`docs/MACRON_GANTRY.md`).

## Why this matters

Every "mm" value used in this codebase before 2026-07-28's calibration
finding was actually 15x smaller in real distance than reported:

- All manual axis moves performed earlier in the 2026-07-28 session
  (X 100→98, Y 75→74, Z 24→23, the "2mm"/"1mm" moves) were real moves of
  30mm/15mm/15mm respectively.
- `server-setup/plans/run_scan_x_0_to_20mm.py`'s "20mm" line scan would
  have actually commanded a 300mm traverse.
- **Safety-relevant:** `config/example_config.yaml`'s `fences:` section is
  currently empty (`fences: []`), so this hasn't caused an active problem
  yet — but the moment fence bounds are populated in what look like real
  mm (e.g. the commented-out example `x: [0, 500]`), they will silently
  permit 15x more physical travel than intended unless this is accounted
  for. Do not populate fences without resolving this first.
- `homing.standoff_mm`/`speed_mm_s` in the same config section have the
  same latent problem once homing is actually wired up.

## Recommended fix path

**Primary recommendation: fix it at the source, in Snap2Motion.** The
15 mm/unit ratio is almost certainly a misconfigured axis scale parameter
(counts-per-unit / electronic gearing / ballscrew pitch) in the DSM
project itself. If that gets corrected there so the controller's own `ACP`
native unit becomes real mm, every layer of this codebase (driver, agent,
profiler, fences, docs) becomes correct automatically, with no software
compensation needed anywhere. This is almost certainly less total work
than the alternative, and removes an entire class of future bugs (someone
adding a new position-reading code path without knowing about the
conversion).

**Fallback: software conversion layer**, if reconfiguring Snap2Motion isn't
possible or desired. If chosen, the conversion must live at the layer that
already knows a given ASCII response/argument *means* a position — not in
raw command passthrough (`PiGantryConnection.send()`/`conn.send()` doesn't
know whether a given command's numeric argument is a position, an I/O
index, or something else; converting there would be guessing at semantics
it doesn't have). Concretely, that means:

1. `src/laguna/robot/macron/commands.py` — `MMCCommands`'s
   position-reading/-writing methods (`get_actual_position`,
   whatever wraps `SPD`/`BMT`, etc.)
2. `src/laguna/robot/macron/gantry_agent.py` — `_run_scan()`'s BLC
   round-trip values (`start_pos_mm`/`accel_mm_s2`/`decel_mm_s2`/
   `actual_end_mm` — currently raw ACP/ACL/DCL values used directly) AND
   the dead-reckoning position math (`pos_mm = start_pos_mm + feed_rate_mm_s
   * (t - t_slew_start)`, currently computed entirely in raw units)
3. A shared constant needs to exist in **both** places, kept in sync by
   hand (same pattern already used for `SAFE_COMMANDS`, duplicated between
   `pi_bridge.py` and `gantry_agent.py` because the agent must stay
   standalone/no-laguna-import on the Pi)
4. Tests for both layers need updating to assert on real mm rather than
   raw units
5. `TopographicProfiler`'s CSV output (`pos_mm`/`distance_mm` columns) and
   any narrative docs referencing scan distances need re-verification once
   the layer below them is fixed — they may already be "correct" in
   appearance (just labeled `pos_mm` while actually holding raw units) and
   need re-checking, not just re-scaling
6. `examples/example_05_gantry_single_axis.py`'s stopgap conversion
   (`MM_PER_ACP_UNIT`, `units_to_mm`/`mm_to_units`) can be deleted once the
   driver layer handles this correctly — it exists only because this
   finding surfaced faster than the proper fix could be scoped

## What's already patched (stopgap, not the real fix)

- `examples/example_05_gantry_single_axis.py`: added `MM_PER_ACP_UNIT =
  15.0`, `units_to_mm()`/`mm_to_units()`, and applied them in
  `get_position()`/`move_axis()` for X/Y/Z only (`get_position_raw_units()`
  added for Theta / raw-unit access). This makes the example's own public
  API speak real mm, but nothing else in the codebase is aware of the
  conversion.

## What's NOT yet touched (still assumes 1 unit = 1 mm)

- `src/laguna/robot/macron/commands.py` (`MMCCommands`)
- `src/laguna/robot/macron/gantry_agent.py` (`_run_scan`'s BLC values and
  dead-reckoning math)
- `src/laguna/robot/macron/profiler.py` (`TopographicProfiler`)
- `server-setup/plans/run_scan_x_0_to_20mm.py`
- `config/example_config.yaml`'s `fences`/`homing` sections (currently
  empty/unpopulated, so not yet actively wrong, but will be the moment
  they're filled in)
- All narrative docs describing distances/speeds in mm
  (`docs/MACRON_GANTRY.md`, `docs/RANGEFINDER_PROFILING.md`,
  `docs/subsystems/rangefinder.md`)

## Next steps for whoever picks this up

1. Check whether the Snap2Motion/DSM project's axis configuration can be
   corrected directly (preferred — see "Recommended fix path" above).
   Compare the configured counts-per-mm/gearing ratio against the expected
   value; 15x off suggests either a decimal/unit entry error or a
   deliberate-but-undocumented gearing choice worth asking the vendor
   contact or checking the `.dsm` project files.
2. If a source fix isn't feasible, implement the software conversion layer
   per the "Fallback" section above, working outward from `commands.py`
   and `gantry_agent.py` (the layers other things build on) before fixing
   examples/docs/fences.
3. Either way, re-verify the 15 mm/unit ratio with a second, independent
   measurement pass before committing to it as final — the 2026-07-28
   measurement was ruler-measured and consistent across three axes, which
   is a strong signal, but a second confirmation (ideally with a more
   precise instrument, or a longer travel distance to reduce relative
   measurement error) would be worth doing before this becomes load-bearing
   in safety-relevant code (fences).
