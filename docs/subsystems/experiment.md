# Experiment runner

`src/laguna/experiment/runner.py` is the glue between a YAML config file, an
optional schedule CSV, and a running `FlumeLab`. It's what
`experiments/weir_gauge_camera_experiment.py` (and any future experiment
script) calls into rather than hand-wiring subsystems itself.

## What `setup_run()` actually does

1. Loads `lab_config` YAML and instantiates a `FlumeLab`.
2. Calls `lab.add_all()` — builds and registers every subsystem whose
   section was explicitly present in the YAML, via
   `laguna.registry.SUBSYSTEM_REGISTRY` (gantry, weir, gauge, flow,
   gocator, od2000, wtt12l, pi_cameras, dslr_cameras). **Absent key →
   subsystem is simply not created** — this is how the framework stays
   opt-in; a config with only a `weir:` section runs weir-only with no
   camera/gauge code touched at all. See the
   [FlumeLab setup guide](../FLUMELAB_SETUP_GUIDE.md) for the full
   `add()`/`add_all()`/registry picture.
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
   silently picking one. `gauge`/`weir`'s periodic status-polling actions
   also respect an opt-in `log_as_event: true` key — see the
   [setup guide's logging section](../FLUMELAB_SETUP_GUIDE.md#two-tier-logging)
   for why a poll doesn't reach the archival event log by default.

5. Returns the configured (but not yet running) `FlumeLab`. Call
   `lab.start(duration)` yourself, or use `run_blocking()` (below) for the
   common CLI case.

## Actuators vs. sensors under `use_schedule`

The scheduling logic in `schedule_action()` treats "does this action need
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

## User-defined scheduled actions

`setup_run()` only wires up the built-in per-subsystem actions (gauge
polling, weir/flow setpoints, camera captures, one fixed Gocator
transect). Anything more involved — a tiled survey, a repeated WTT12L
transect, a custom action spanning several subsystems — isn't something
you add to the codebase; you register it yourself, in your own run
script, using the same `schedule_action()` building block every built-in
action already goes through:

```python
from laguna.experiment import schedule_action, setup_run, run_blocking
from laguna.survey import Tile, SurveyRunner

lab = setup_run("config/my_experiment.yaml")

def tiled_scan():
    active_area = lab.gocator.get_active_area()
    tile = Tile(
        origin=[0, 0, 0], length_mm=1000, width_mm=600,
        swath_mm=active_area["width_mm"], instrument="gocator", speed=20.0,
    )
    SurveyRunner(lab, tile).run()

schedule_action(
    lab, lab.config.get("tiled_scan"), subsystem="gocator", name="tiled_scan",
    action=tiled_scan,
)

run_blocking(lab, duration=3600)
```

with a matching config section — any of `interval_s`/`trigger_at`/
`use_schedule` works, same as a built-in subsystem:

```yaml
tiled_scan:
  interval_s: 900   # a full tile pass every 15 minutes
```

`schedule_action()` validates that section the same way it validates
`gauge:`/`weir:`/etc. — set more than one of those three keys and it
raises `ValueError`, whether the section belongs to a real subsystem or
not. The action itself is a plain zero-arg closure: it can read
`lab.gocator`/`lab.gantry`/any other connected subsystem, run a whole
`SurveyRunner` pass, and take as long as it needs — the scheduler doesn't
care what's inside, only when to fire it.

**Overlap is handled for you, and it's a pause, not a skip.**
`Scheduler._fire()` spawns a fresh daemon thread on every due firing with
no awareness of whether the previous firing is still running — so a
tiled scan that takes longer than its own `interval_s` would otherwise
run concurrently with itself. `schedule_action()` guards against this
automatically: if a new firing starts before the previous one for the
same action finished, it calls `lab.escalate(...)` (pausing the whole
lab) instead of running it, skipping it, or letting them race. This is
deliberate, not a conservative default to override — per this project's
priority ordering, missed/duplicated data is worse than a pause, and an
interval dense enough to trigger this means the *schedule* doesn't match
how long the action actually takes, which is worth fixing at the
experiment-design level (widen the interval, or speed up the action),
not working around at runtime.

**That guard is self-only by default — two *different* actions never
block each other unless you opt them in.** A weir setpoint and a camera
capture run fully concurrently with no coordination at all out of the
box, since each `schedule_action()` call gets its own private lock. Pass
the same `exclusive_with` tag to two (or more) calls to widen that into a
shared exclusion group — e.g. a tiled Gocator scan and a camera capture
that must not fire while the gantry is mid-scan:

```python
schedule_action(lab, lab.config.get("tiled_scan"), "gocator", "tiled_scan",
                 action=tiled_scan, exclusive_with="gantry_busy")
schedule_action(lab, lab.config.get("pi_cameras"), "pi_cameras", "capture",
                 action=capture, exclusive_with="gantry_busy")
```

Both still can't overlap *themselves* either way — `exclusive_with` only
adds cross-action exclusion, it never removes the self-exclusion above.
A capture that fires while the tagged scan is mid-flight escalates
exactly like a self-overlap would, naming the tag in the pause reason.

One more thing worth building into a real hook like this, following the
pattern `runner.py`'s own `_scan_gocator()` closure uses internally: pass
a `CheckpointStore` to `SurveyRunner` so an interrupted tile resumes
instead of restarting — see `laguna.timing.checkpoint.CheckpointStore`
and `SurveyRunner`'s own docstring. (A failed *pass* inside a survey is a
separate concern from an *overlapping firing* — `SurveyRunner.run()`
re-raises on a pass failure, which propagates out of the closure and
still needs handling; see `example_14` for one way to do that.)

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

- [FlumeLab setup & logging guide](../FLUMELAB_SETUP_GUIDE.md) — `add()`/
  `add_all()`/the registry, the two-tier event/operational log model, and
  `simulate=True` rehearsal mode, all with worked examples.
- [API reference](../reference/experiment.md) — generated from docstrings.
- [Timing](timing.md) — the `Scheduler`/`EventLog` this module drives.
- [Schedule](schedule.md) — the CSV format behind `use_schedule: true`.
- [Weir](weir.md), [Flow](flow.md), [Gauge](gauge.md), [Camera](camera.md) — the subsystems this wires together.
- `src/laguna/experiment/runner.py` — the module itself.
