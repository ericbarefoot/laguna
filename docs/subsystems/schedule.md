# Schedule

Loads a tabular time-elevation (and time-flowrate, time-valve-state)
schedule from CSV or Excel and turns it into callables you can evaluate at
any experiment-runtime second. Lives at `src/laguna/schedule/`, a single
class: `ExperimentSchedule`.

## Why it exists

Actuator subsystems (`weir`, `flow`) don't read the CSV themselves — the
[experiment runner](experiment.md) loads it once into an `ExperimentSchedule`
and uses its interpolators to compute a setpoint at each scheduled tick.
`ExperimentSchedule` itself has no dependency on `FlumeLab` or any
subsystem; it's a standalone table-to-function utility.

**Hard dependency**: importing `laguna.schedule` requires `numpy`, `scipy`,
and `pandas` — the module raises `ImportError` with an explicit install
hint at import time if any are missing, rather than failing later with a
confusing `NameError`.

## Quick start

```python
from laguna.schedule import ExperimentSchedule

schedule = ExperimentSchedule.from_csv("examples/example_schedule.csv")

schedule.weir_elevation(90.0)   # -> interpolated mm at t=90s (cubic spline)
schedule.pump_flow(90.0)        # -> interpolated L/min at t=90s (linear)
schedule.qin_open(90.0)         # -> True/False at t=90s (step — holds last value)
```

`from_excel(path, sheet=0)` works the same way for `.xlsx` files.
`from_dataframe(df)` accepts a DataFrame you've already loaded/built
yourself (e.g. constructed programmatically rather than from a file).

## Required and recognized columns

Only `time_s` is required — `from_csv`/`from_excel`/`from_dataframe` raise
`ValueError` if it's missing. Every other column is optional; interpolators
are only built for columns that are actually present in the DataFrame.

| Column | Default interpolation | Accessor property |
|---|---|---|
| `time_s` | — (the x-axis) | — |
| `weir_elevation_mm` | `spline` (cubic) | `.weir_elevation` |
| `pump_flow_lpm` | `linear` | `.pump_flow` |
| `qin_open` | `step` | `.qin_open` |
| `qaux_open` | `step` | `.qaux_open` |

Accessing a property for a column that wasn't in the DataFrame raises
`AttributeError` with a message naming the missing column — check
`"col" in df.columns` yourself first if you need to branch on availability
(the [experiment runner](experiment.md) does exactly this before deciding
whether to register a scheduled action for `weir`/`flow`).

Two more columns are meaningful to the [experiment runner](experiment.md)
but are **not** part of `INTERP_DEFAULTS` and have no interpolator or
property here: `pi_cameras` and `dslr_cameras`. Those are read directly off
the raw DataFrame (`exp_schedule._df`) as a truthy row filter — "only fire
the camera trigger at `time_s` rows where this column is truthy" — rather
than interpolated, since a boolean-ish trigger column doesn't make sense to
interpolate between rows.

## Interpolation modes

Pass a custom `interpolation={"col": "mode"}` dict to any constructor to
override a default, or to add a mode for a column not in
`INTERP_DEFAULTS`. Three modes are implemented:

- **`spline`** — `scipy.interpolate.CubicSpline` over all rows. Smooth
  motion between keyframes; can overshoot past the local min/max between
  two widely-spaced points, which matters for something like weir
  elevation where an overshoot means real (if brief) over-travel.
- **`linear`** — `numpy.interp`. No overshoot, but has a slope
  discontinuity at each keyframe.
- **`step`** — holds the value from the most recent `time_s <= t`
  (`np.searchsorted(..., side="right") - 1`, clamped to valid range).
  Appropriate for anything that's inherently discrete (valve open/closed),
  not appropriate for anything you want to move smoothly.

All three interpolators extrapolate flatly/mathematically outside the
table's time range rather than raising — `CubicSpline` and `np.interp`
will both return values for `t` before the first or after the last row
(clamped for `np.interp`, polynomial extrapolation for `CubicSpline`,
which can diverge badly outside the fitted range). Keep your schedule's
last row at or beyond your intended experiment duration.

## Example schedule

`examples/example_schedule.csv`:

```csv
time_s,weir_elevation_mm,pump_flow_lpm,qin_open,qaux_open
0,100,0.0,1,0
60,90,0.0,1,0
120,80,0.0,1,0
180,70,0.0,1,0
240,60,0.0,1,0
300,50,0.0,1,0
```

## Further reading

- [API reference](../reference/schedule.md) — generated from docstrings.
- [Experiment runner](experiment.md) — how a loaded `ExperimentSchedule` drives `weir`/`flow`/camera scheduling via `schedule_action()`.
- [Weir](weir.md), [Flow](flow.md) — the subsystems that consume `weir_elevation_mm`/`pump_flow_lpm`/`qin_open`/`qaux_open`.
