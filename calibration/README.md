# Calibration notebooks

Procedures that get re-run when the rig changes, kept here (not under `examples/`) because they are tools,
not demonstrations.

## `gocator_alignment_and_seam.ipynb`

Realign the Gocator after a remount and verify the tile-scan seam. Run it with the gantry clear: it moves the
rig, behind `ALLOW_MOTION` and an explicit enable cell, with a STOP cell and sentinel files (`PAUSE`, `ESTOP`)
always available.

| § | What | Moves? |
|---|---|---|
| 0 | Parameters, helpers, connect in safe mode; reuse saved results | no |
| 1a | Stationary profile: is the bed inside the active area? | no |
| 1c | WTT12L: find the block centre in gantry mm | yes |
| 1d | Three Gocator passes over the block | yes |
| 1e-1g | Mounting check, solve the translation, reload the config | no |
| 2a | Plan the tile from a region of interest; dry-run against soft limits and fences | no |
| 2b | Run A (old behaviour) vs run B (ramp lead-in and lead-out) | yes |
| 2c | Seam analysis: forward vs reverse offset along travel | no |
| 2e | Calibrate `gocator.trigger_delay_s` at your scan speed (5 repeats) | yes |

Every result is written to `data/scans/seam_test_<time>/` as it appears (`AlignmentStore`), so a later run can
skip a scan with `LOAD_FROM` / `B_W_KNOWN` (set in the parameters cell). A block position saved before the
2026-10-08 reverse-scan position fix is wrong and the store warns when it loads one.

The trigger delay depends on scan speed and axis acceleration. Calibrate at the speed you will scan at.
