# Finding: Gantry ACP Units Are Not Millimeters

**Status as of 2026-07-29: confirmed on hardware, still not fixed at the
source — but the software workaround described below has now landed as a
single toggle in two places, replacing the scattered per-script stopgaps.
Flip both to `1.0` if the Snap2Motion/DSM project's axis scale is ever
corrected at the source (see "Recommended fix path"); nothing else needs
to change.**

**The two toggle locations:**

1. `config/example_config.yaml` → `gantry.mm_per_acp_unit` (read by
   `GantryController.from_config()`, applied inside `MMCCommands` — every
   position/velocity-reading or -writing method on linear axes goes
   through this one choke point; Theta is exempt, see below).
2. `src/laguna/robot/macron/gantry_agent.py` → module constant
   `MM_PER_ACP_UNIT` (kept as a separate literal, not imported from
   config, because this file must run standalone on the Pi with no laguna
   install — same reason `SAFE_COMMANDS` is duplicated there rather than
   shared).

`GantryController.move_to()` (vector or `X=`/`Y=`/`Z=`/`Theta=` keyword
forms) and `FlumeLab.move_to()`/`acquire_scan()` all speak real mm now —
no caller outside `commands.py`/`gantry_agent.py` needs to know about the
15x ratio.

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
native unit becomes real mm, flip both toggles above to `1.0` — every
layer of this codebase (driver, agent, profiler, fences, docs) becomes
correct automatically, with no other software change needed anywhere.

**Software conversion layer (now implemented)** — the conversion lives at
the layer that already knows a given ASCII response/argument *means* a
position, not in raw command passthrough (`PiGantryConnection.send()`/
`conn.send()` doesn't know whether a given command's numeric argument is a
position, an I/O index, or something else). Concretely:

1. `src/laguna/robot/macron/commands.py` — `MMCCommands` takes
   `mm_per_unit`/`coordinate_offset_mm`/`group_axes` and applies them in
   every position/velocity-reading or -writing method (`get_actual_position`,
   `move_to`, `set_speed`, the `group_*` methods, etc.) for linear (X/Y/Z)
   axes only — Theta is exempt (separate, already-correct rotary
   conversion). `GantryController.from_config()` wires this up from
   `gantry.mm_per_acp_unit`/`gantry.coordinate_offset` in config.
2. `src/laguna/robot/macron/gantry_agent.py` — `_run_scan()`'s BLC
   round-trip values (`start_pos_mm`/`accel_mm_s2`/`decel_mm_s2`/
   `actual_end_mm`) and the dead-reckoning position math are converted via
   its own `MM_PER_ACP_UNIT` constant, so `scan_started`/`scan_done`
   results and the CSV's `pos_mm` column are real mm end-to-end — callers
   (`TopographicProfiler.scan()`, `FlumeLab.acquire_scan()`) pass/receive
   real mm and no longer need their own conversion.
3. The constant is duplicated by hand between the two places above (same
   pattern already used for `SAFE_COMMANDS`, duplicated between
   `pi_bridge.py` and `gantry_agent.py`, because the agent must stay
   standalone/no-laguna-import on the Pi) — keep them in sync.
4. `examples/example_05_gantry_single_axis.py` and
   `scripts/run_line_scan.py` had their own local `MM_PER_ACP_UNIT`
   stopgaps deleted now that the driver/agent layers handle this
   correctly; they call `GantryController.move_to()` /
   `TopographicProfiler.scan()` directly with real mm.

## What still assumes raw units, or needs care

- `config/example_config.yaml`'s `fences`/`homing` sections are populated
  in real mm now that `MMCCommands` converts before checking them — but
  they were still empty/unpopulated as of this writing, so this is
  untested in practice. Re-verify with a real fence once one is defined.
- `run_line_scan.py` still queries `ACP` directly via a raw
  `conn.send()` passthrough (not through `MMCCommands`) for its initial
  position read, so it keeps its own small `MM_PER_ACP_UNIT` for that one
  call — kept in sync by hand with `gantry_agent.py`'s copy.
- Narrative docs describing distances/speeds in mm
  (`docs/MACRON_GANTRY.md`, `docs/RANGEFINDER_PROFILING.md`,
  `docs/subsystems/rangefinder.md`) were written before this fix landed —
  worth a re-read, though their examples were already expressed in
  intended real mm rather than raw units.

## Next steps for whoever picks this up

1. Check whether the Snap2Motion/DSM project's axis configuration can be
   corrected directly (preferred — see "Recommended fix path" above).
   Compare the configured counts-per-mm/gearing ratio against the expected
   value; 15x off suggests either a decimal/unit entry error or a
   deliberate-but-undocumented gearing choice worth asking the vendor
   contact or checking the `.dsm` project files. If corrected, flip both
   toggles to `1.0` and delete this doc's now-obsolete conversion notes.
2. Re-verify the 15 mm/unit ratio with a second, independent measurement
   pass before it becomes load-bearing in safety-relevant code (fences) —
   the 2026-07-28 measurement was ruler-measured and consistent across
   three axes, which is a strong signal, but a second confirmation
   (ideally with a more precise instrument, or a longer travel distance to
   reduce relative measurement error) is still worth doing.
