# FlumeLab setup & logging guide

Everything on this page landed together: a config-driven way to build
`FlumeLab` subsystems, and a two-tier logging model (a terse archival
event log vs. a detailed operational log) that subsystems participate in
automatically. If you've used an older version of `laguna` and remember
hand-building each subsystem and wiring `lab.event_log.log(...)` calls
into your own script, this replaces that.

## Quick start

```python
from laguna import FlumeLab

lab = FlumeLab("config/my_experiment.yaml")
lab.add_all()          # builds + registers every subsystem present in the YAML
lab.connect_all()

with lab.experiment() as clock:
    lab.scheduler.run(duration=1800)
```

`add_all()` looks at which top-level sections were actually present in
your YAML (`Config.explicit_sections` — not `Config`'s built-in defaults,
which exist for every section whether you asked for it or not), looks
each one up in `laguna.registry.SUBSYSTEM_REGISTRY`, and calls its
`from_config()` for you. Omit a section from your config file and that
subsystem is never built — no code path touches it.

## Building subsystems: three ways

```python
from laguna import FlumeLab
from laguna.weir import SaflWeirController

lab = FlumeLab("config/my_experiment.yaml")

# 1. Everything present in the config file
lab.add_all()

# 2. One subsystem by registry name — looks up SUBSYSTEM_REGISTRY,
#    pulls that section's config, calls from_config() for you
lab.add("gantry")
lab.add("gantry").add("od2000").add("wtt12l")   # chainable

# 3. A pre-built instance — still works exactly as before, for anything
#    not in the registry (e.g. CameraManager) or built with custom args
lab.add(SaflWeirController({"port": "/dev/ttyUSB0"}))
```

All three register under `subsystem.subsystem_name` and become reachable
as `lab.<subsystem_name>` — `lab.gantry`, `lab.weir`, `lab.od2000`, etc.

### `SUBSYSTEM_REGISTRY`

```python
from laguna.registry import SUBSYSTEM_REGISTRY

print(list(SUBSYSTEM_REGISTRY))
# ['gantry', 'weir', 'gauge', 'flow', 'gocator',
#  'od2000', 'wtt12l', 'pi_cameras', 'dslr_cameras']
```

`mqtt` is deliberately not in this list even though `MqttSubscriber` has a
`from_config()` — each rangefinder already builds its own private
`MqttSubscriber` from the shared `mqtt:` config section, so a standalone
`lab.mqtt` would just be a second, unused connection to the same broker.
Build one directly if you actually need one:
`lab.add(MqttSubscriber.from_config(lab.config))`.

## Writing `from_config()`

Every registry entry's `from_config()` takes the lab's whole `Config`
object, not just its own section — this is what lets a subsystem reach
sibling config or the config file's own path when it needs to:

```python
class SaflWeirController(WeirController, SubsystemLogging):
    @classmethod
    def from_config(cls, config: "Config") -> "SaflWeirController":
        return cls(config.get("weir"))
```

```python
class RangefinderSubsystem:
    @classmethod
    def from_config(cls, config: "Config") -> "RangefinderSubsystem":
        # Reaches into the *sibling* 'mqtt:' section to build its own
        # private MqttSubscriber — every rangefinder gets its own client
        # ID, so several can share one broker without fighting.
        section = config.get(cls.subsystem_name)
        mqtt_subscriber = MqttSubscriber(config.get("mqtt"))
        return cls(section, mqtt_subscriber)
```

```python
class DslrCameraSubsystem(SubsystemLogging):
    @classmethod
    def from_config(cls, config: "Config") -> "DslrCameraSubsystem":
        # config.config_file resolves output_dir paths relative to
        # wherever the experiment YAML actually lives on disk.
        dslr_cfg = config.get("dslr_cameras")
        main_dir = Path(config.config_file).resolve().parent
        ...
```

See [Adding a New Subsystem](CONTRIBUTING.md#adding-a-new-subsystem) for
the full contract (`connect()`/`disconnect()`/`get_status()`/`stop()`,
`subsystem_name`, registry entry, tests).

## Two-tier logging

Two separate channels, for two separate audiences:

| | Event log | Operational log |
|---|---|---|
| **What** | `lab.event_log` — a CSV (`laguna.timing.EventLog`) | Standard Python `logging`, under the `laguna.*` hierarchy |
| **Audience** | Ships alongside published data as metadata | You, troubleshooting a run |
| **Contents** | State-changing actions & milestones only: `weir moved to 300mm`, `scan completed`, `inflow on` | Connections firing, broad commands (`move_to() started/completed`), and — at `DEBUG` — every low-level step underneath |
| **Verbosity knob** | `event_log_verbosity` (per subsystem, in config) | `log_level` (per subsystem, in config) or `FlumeLab(debug=True)` (everything at once) |

### Event log: what lands there by default

```python
lab.weir.go_to_elevation(300.0)     # -> archival row: weir / go_to_elevation / target_mm=300.00
lab.flow.qin = True                 # -> archival row: flow / qin / state=True
lab.dslr_cameras.capture_all()      # -> one archival row per camera: capture / file=...
```

Passive reads stay **out** of the archival log by default — a reading
measures the experiment's state without changing it, so it's not a "step
taken":

```python
lab.gauge.read_mm()   # operational log only, e.g.:
# 2026-08-05 12:03:41 - laguna.gauge.sensor - INFO - read_mm: elevation_mm=182.40
```

If a scheduled poll is infrequent enough that it *is* a milestone worth
archiving (an hourly summary reading, say), opt it in explicitly in
config — this is a human decision, not something inferred from the
interval:

```yaml
gauge:
  port: /dev/ttyUSB2
  interval_s: 3600
  log_as_event: true    # this poll IS a milestone -> also written to the event log
```

### `lab.log_note()` — add your own entry

The event log is meant to be readable years later next to the data it
describes. Add a note any time — pausing/stopping first isn't required:

```python
lab.log_note("operator restarted the pump after a fault at the inlet valve")

# Point a note at a specific prior row — every log_event()/log_note() call
# returns its own event_id
scan_id = lab.event_log.log(lab.clock.elapsed(), "gocator", "scan", result="error: timeout")
lab.log_note("sensor cable was loose, reseated and rerunning", refers_to=scan_id)
```

```csv
event_id,wall_time_iso,wall_time_unix,runtime_s,subsystem,event_type,result,notes,refers_to
41,2026-08-05T18:02:11+00:00,...,812.400,gocator,scan,error: timeout,,
42,2026-08-05T18:03:05+00:00,...,866.100,operator,note,ok,sensor cable was loose...,41
```

### Operational log: connections and broad motion, not every segment

```python
lab.gantry.move_to([100.0, 50.0, 10.0, 0.0])
```

At the default `INFO` level, that's exactly two lines — regardless of
whether the move is a straight line or a tessellated arc expanding into
dozens of G-code segments underneath:

```
2026-08-05 12:04:02 - laguna.robot.macron.controller - INFO - move_to([100.0, 50.0, 10.0, 0.0]) — started
2026-08-05 12:04:03 - laguna.robot.macron.controller - INFO - move_to([100.0, 50.0, 10.0, 0.0]) — completed
```

Turn on `DEBUG` (per subsystem, or globally — next section) to see the
segments themselves:

```
2026-08-05 12:04:02 - laguna.robot.macron.gcode - DEBUG - C1 SPD 10; C1 BMT 6.88427 -25.3186
2026-08-05 12:04:02 - laguna.robot.macron.gcode - DEBUG - C1 SPD 10; C1 BMT 4.81533 -21.4090
... (70 more lines for one tessellated arc)
```

### Persisting the operational log to disk

Set `timing.run_dir` and it's written automatically, alongside anything
else the run produces:

```yaml
timing:
  run_dir: ./runs
```

```
runs/20260805T120400Z-9f3a/laguna.log
```

No `run_dir` configured → terminal only, exactly like before this system
existed.

### `FlumeLab(debug=True)` — one switch, not N config edits

```python
lab = FlumeLab("config/my_experiment.yaml", debug=True)
```

Sets every `laguna.*` logger to `DEBUG` at once — including subsystems
that have their own `log_level: INFO` in config, so you don't have to go
edit five sections to fully debug a multi-subsystem issue. Third-party
libraries (`paramiko`, etc.) stay pinned to `WARNING` regardless —
"debug my code," not "debug every dependency's own chatter."

## Rehearsal mode (`simulate=True`)

```python
lab = FlumeLab("config/my_experiment.yaml", simulate=True)
lab.add_all()
lab.connect_all()   # succeeds — nothing on the network
```

`simulate=True` rehearses the **script and plan**, not physical
feasibility. Every subsystem in `laguna.registry.SUBSYSTEM_REGISTRY` has a
simulated backend today — gantry, gocator, weir, flow, gauge,
`pi_cameras`, `dslr_cameras`, `od2000`, `wtt12l`:

- **Commands succeed and log exactly as they would for real** — this is
  what actually proves a schedule fires the right thing at the right
  time. `lab.weir.go_to_elevation(300.0)` still writes its archival
  `go_to_elevation` row; `lab.dslr_cameras.capture_all()` still logs a
  `capture` row per camera.
- **Readings come back `NaN`** (or `None` for non-numeric status fields)
  — never a fabricated, physically-plausible number:

  ```python
  import math

  lab.weir.get_elevation()          # nan
  lab.gauge.read_mm()               # nan
  lab.flow.get_status()["vfd_state"]  # None
  math.isnan(lab.weir.get_elevation())  # True
  ```

  `get_flowrate()` is the one exception worth knowing about: it echoes
  the setpoint *you* last commanded via `set_flowrate()` rather than
  reading anything back from a driver, so it's a real number, not NaN —
  it was never fabricated data to begin with.

- **Fence checking still runs for real.** A rehearsed move that would
  violate a fence still raises `FenceViolation` — physical-limit checking
  is `fences.py`'s job, real in every mode, and a rehearsal that let
  fence-violating scripts through would be worse than no rehearsal.
- **The rangefinders (`od2000`/`wtt12l`) rehearse too** — `connect()`
  succeeds with no MQTT broker or AL1342 needed, `activate()`/
  `deactivate()` skip the real IO-Link HTTP write, and every reading
  (`get_distance_mm()`, `read_mm()`, `get_status()`'s numeric fields) comes
  back `NaN`. Every entry in `laguna.registry.SUBSYSTEM_REGISTRY` has a
  simulated path today — `_NO_SIMULATED_BACKEND` is empty, kept only as
  the fail-closed guard for whatever gets added next without one.
- **A simulated Gocator scan is a small, fixed-size synthetic surface**
  (~80,000 cells, well under a megabyte) — it does not scale up with
  `fixed_length_mm`/frame rate the way a real scan would, so a rehearsal
  with scans on a tight schedule does not accumulate large files on disk.

### Rehearsal-specific event log safeguards

A rehearsal's archival record can never collide with a real run's:

```python
lab = FlumeLab("config/my_experiment.yaml", simulate=True)
# timing.event_log: experiment_events.csv  (from your YAML)
# -> actually written to:    experiment_events_simulated.csv
```

The `_simulated` suffix is applied even if you set `timing.event_log`
explicitly — the same config file is often reused for both a real run and
its rehearsal, and the filename split guards against exactly that. A
`flume_lab` / `simulate_mode` row is also written to the event log at
construction, so the file says what it is even if it ends up copied or
merged somewhere else later:

```csv
event_id,wall_time_iso,...,subsystem,event_type,result,notes,refers_to
1,2026-08-05T12:00:00+00:00,...,flume_lab,simulate_mode,ok,rehearsal — no hardware contacted; speed_factor=1.0,
```

### Giving a new subsystem a rehearsal path

See [Adding a New Subsystem, item 7](CONTRIBUTING.md#adding-a-new-subsystem)
— a `simulated: true` config key your `connect()` checks, an in-memory
stand-in that succeeds on commands and returns `NaN`/`None` on reads, and
an entry in `laguna.simulation`'s `_SIMULATED_SECTIONS`.

## Putting it together

A full config-driven rehearsal, exercising weir, flow, gauge, and a
camera trigger together, with an hourly gauge summary opted into the
archival log:

```yaml
# experiment_config.yaml
timing:
  event_log: ./experiment_events.csv
  run_dir: ./runs

weir:
  port: /dev/ttyUSB0
  use_schedule: true

flow:
  vfd_port: /dev/ttyUSB1
  use_schedule: true

gauge:
  port: /dev/ttyUSB2
  interval_s: 3600
  log_as_event: true

pi_cameras:
  hosts: [pi1.local, pi2.local]
  interval_s: 300
```

```python
from laguna.experiment import setup_run, run_blocking

lab = setup_run(
    "experiment_config.yaml",
    schedule="my_schedule.csv",
    simulate=True,     # drop to False once the rehearsal looks right
    debug=True,        # see every command while you're checking it over
)
run_blocking(lab, duration=3600)
```

Read `runs/<run_id>/laguna.log` afterward for the full blow-by-blow, or
`experiment_events_simulated.csv` for the terse narrative — weir/flow
setpoint changes, camera captures, the hourly gauge reading, and nothing
else.

## Further reading

- [Adding a New Subsystem](CONTRIBUTING.md#adding-a-new-subsystem) — the
  full contract for `from_config()`, the registry, and logging.
- [Experiment runner](subsystems/experiment.md) — `setup_run()`/
  `run_blocking()` in more depth, and the scheduling keys
  (`interval_s`/`trigger_at`/`use_schedule`).
- [Timing](subsystems/timing.md) — `EventLog`/`Scheduler` themselves.
- `src/laguna/subsystem_logging.py` — the module docstring is the
  canonical source for the two-tier reasoning above.
- `src/laguna/simulation.py` — same, for what rehearsal mode does and
  does not catch.
