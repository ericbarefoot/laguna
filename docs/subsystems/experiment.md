# Experiment runner

`src/laguna/experiment/runner.py` is the glue between a YAML config file, an
optional schedule CSV, and a running `FlumeLab`. It's what
`experiments/weir_gauge_camera_experiment.py` (and any future experiment
script) calls into rather than hand-wiring subsystems itself.

## What `setup_run()` actually does

1. Loads `lab_config` YAML and instantiates a `FlumeLab`.
2. For each of `gauge`, `weir`, `flow`, `pi_cameras`, `dslr_cameras`: if that
   top-level key is present in the YAML, instantiate the matching subsystem
   and `lab.add()` it. **Absent key → subsystem is simply not created** —
   this is how the framework stays opt-in; a config with only a `weir:`
   section runs weir-only with no camera/gauge code touched at all.
3. Connects every registered subsystem, logging which succeeded/failed
   (failures are non-fatal — the run continues without that subsystem).
4. Registers one scheduled action per subsystem, in one of three mutually
   exclusive modes chosen by which key is present in that subsystem's
   config section:

   | Key | Behavior |
   |---|---|
   | `interval_s: N` | Fire every `N` runtime seconds |
   | `trigger_at: [t1, t2, ...]` | Fire once at each listed runtime second |
   | `use_schedule: true` | Fire at every time point in the loaded schedule CSV (see [Schedule](schedule.md)) |

   `_validate_trigger_config()` raises `ValueError` at setup time if a
   section specifies more than one of these — fail fast rather than
   silently picking one.

5. Returns the configured (but not yet running) `FlumeLab`. Call
   `lab.start(duration)` yourself, or use `run_blocking()` (below) for the
   common CLI case.

## Actuators vs. sensors under `use_schedule`

The scheduling logic in `_register_action()` treats "does this action need
a *value* from the schedule, or just a *timestamp*?" as the key branch:

- **Actuators** (`weir`, `flow`) pass an `action_factory(t_s) -> Callable`
  instead of a fixed `action`. At every schedule time point, the factory
  builds a closure that reads the interpolated target value at that
  instant (`exp_schedule.weir_elevation(t_s)`, etc.) and issues the move —
  so the actual setpoint is resolved lazily, at fire time, not once at
  registration time.
- **Sensors/cameras** (`gauge`, `pi_cameras`, `dslr_cameras`) pass a fixed
  `action`. If `use_schedule` is set and the CSV has a matching column
  named after the config key (e.g. a `pi_cameras` column of 1/0), the
  action only fires on truthy rows; with no such column, it fires at every
  time point.

If `weir`/`flow` request `use_schedule: true` but the loaded CSV lacks the
required column (`weir_elevation_mm` / any of `pump_flow_lpm`, `qin_open`,
`qaux_open`), `setup_run()` doesn't fail — it logs a warning and falls back
to read-only status polling for that subsystem instead of guessing a
setpoint.

## Quick start

```python
from laguna.experiment.runner import setup_run, run_blocking

lab = setup_run(
    lab_config="config/my_experiment.yaml",
    schedule="my_schedule.csv",   # omit if nothing uses use_schedule
)
run_blocking(lab, duration=1800)   # 30 minutes, blocks until done
```

Or drive it yourself without `run_blocking()`'s signal handling:

```python
lab = setup_run("config/my_experiment.yaml")
thread = lab.start(duration=1800)
thread.join()
lab.disconnect_all()
```

## `run_blocking()` — pause/resume without dropping hardware connections

Built for interactive CLI use. It writes `.experiment.pid` (removed on
exit) and installs signal handlers so a run can be paused and resumed from
another terminal *without* re-establishing serial/SSH connections to
hardware:

| Signal | Effect |
|---|---|
| `Ctrl+C` (first) | Pause — `lab.stop()`, hardware stays connected |
| `Ctrl+C` (second, while paused) | Stop and disconnect, exit |
| `kill -USR1 <pid>` | Pause, from another terminal |
| `kill -USR2 <pid>` | Resume — `lab.resume()`, continues from elapsed time, not from zero |
| `kill <pid>` (`SIGTERM`) | Stop and disconnect |

`tail -f experiment_events.csv` from another terminal gives a live view of
the [event log](timing.md) while a run is in progress.

## Further reading

- [API reference](../reference/experiment.md) — generated from docstrings.
- [Timing](timing.md) — the `Scheduler`/`EventLog` this module drives.
- [Schedule](schedule.md) — the CSV format behind `use_schedule: true`.
- [Weir](weir.md), [Flow](flow.md), [Gauge](gauge.md), [Camera](camera.md) — the subsystems this wires together.
- `src/laguna/experiment/runner.py` — the module itself.
