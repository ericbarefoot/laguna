# Quick Reference

## Installation

```bash
git clone <repo-url>
cd laguna
mamba env create -f environment.yml   # or: conda env create -f environment.yml
mamba activate flumelab               # or: conda activate flumelab
```

Verify it worked:

```bash
python -c "from laguna import FlumeLab; print('OK')"
```

Hardware-specific extras (DSLR/`gphoto2` support, S3/SFTP storage, etc.) are
covered on the relevant [subsystem page](subsystems/camera.md) — most of
them are Linux-only and not required just to explore the framework.

## Your First Experiment

Everything (serial ports, hosts, capture intervals, gantry axes) lives in
one YAML file. Start from the checked-in example and adjust ports for your
hardware:

```bash
cp config/example_config.yaml config/my_experiment.yaml
# edit ports/hosts; comment out sections for hardware you don't have
```

Run `experiments/weir_gauge_camera_experiment.py`, which wires up whatever
subsystems have a section in the config (via `laguna.experiment.setup_run`)
and registers their scheduled actions:

```bash
# Foreground, blocking, 30 seconds, monitoring only (no --schedule ⇒ no motion):
python experiments/weir_gauge_camera_experiment.py \
    --lab-config config/my_experiment.yaml \
    --duration 30
```

Check `experiment_events.csv` (path set by `timing.event_log` in the
config) for one row per scheduled action. See [Architecture § Event
log](ARCHITECTURE.md#event-log) for the column format.

To make an actuator (weir, flow, gantry) follow a time-elevation schedule,
add `use_schedule: true` to its config section and pass `--schedule`:

```bash
python experiments/weir_gauge_camera_experiment.py \
    --lab-config config/my_experiment.yaml \
    --schedule examples/example_schedule.csv \
    --duration 300
```

While a blocking run is going, control it from another terminal (PID is
printed at startup, and written to `.experiment.pid`):

```bash
kill -USR1 <pid>   # pause (hardware stays connected)
kill -USR2 <pid>   # resume
kill <pid>         # SIGTERM — stop and disconnect
tail -f experiment_events.csv   # live status
```

Ctrl+C once pauses the same way; twice stops and disconnects.

### Interactive mode (REPL / notebook)

For ad hoc control instead of a fixed `--duration`, use
`main_interactive()` — it sets everything up but starts nothing until you
call `lab.start()`:

```python
from experiments.weir_gauge_camera_experiment import main_interactive

lab = main_interactive(lab_config="config/my_experiment.yaml")
lab.start(3600)              # run for up to an hour
lab.get_system_status()      # query all subsystems
lab.stop()                   # pause (hardware stays connected)
lab.resume()                 # resume for the remaining time
lab.disconnect_all()         # clean shutdown when done
```

## Building Your Own Experiment Script

For anything beyond the weir/gauge/camera/gantry combination,
`setup_run()` is optional — you can assemble a `FlumeLab` by hand the same
way every subsystem's own doc page does:

```python
from laguna import FlumeLab
from laguna.config import Config
from laguna.weir import SaflWeirController
from laguna.gauge import SaflWaterLevelSensor

config = Config(config_file="config/my_experiment.yaml")
lab = FlumeLab(config_file="config/my_experiment.yaml")
lab.add(SaflWeirController(config.get("weir")))
lab.add(SaflWaterLevelSensor(config.get("gauge")))
lab.connect_all()

lab.scheduler.repeat(every=5, action=lambda: lab.gauge.read_mm_smoothed(),
                      subsystem="gauge", name="log_level")
lab.start(300)                # non-blocking; returns the scheduler thread
```

Every subsystem follows the same shape: constructor takes a config dict,
`connect()`/`disconnect()`, a `get_status()` for `lab.get_system_status()`,
and (for actuators) `stop()` for `lab.stop()`/`emergency_stop()` to call.
See [Architecture](ARCHITECTURE.md) for the full module breakdown and the
non-blocking scheduler/threading model, and the [subsystem
pages](subsystems/camera.md) for per-subsystem detail.

## Common Tasks

```python
lab.get_system_status()      # {"timing": {...}, "weir": {...}, "gauge": {...}, ...}
lab.emergency_stop()         # stop + disconnect every registered subsystem
lab.stop()                   # pause only (resumable)
lab.resume()                 # resume for the remaining configured duration
lab.resume(60)               # resume for 60 more experiment-seconds instead
```

Per-subsystem access after `lab.add(...)` — the subsystem is available as
`lab.<subsystem_name>`:

```python
lab.weir.set_elevation(200.0)        # mm, non-blocking
lab.weir.wait_for_move()
lab.gauge.read_mm_smoothed()         # FIFO moving average
lab.flow.set_flowrate(20.0)          # L/min
lab.flow.qin = True                  # open solenoid
```

## Running Tests

```bash
pytest                       # all tests
pytest tests/test_macron_controller.py   # one file
pytest --cov                 # coverage report
```

## Development Commands

```bash
black src/ tests/ examples/  # format
isort src/ tests/ examples/  # sort imports
mypy src/laguna              # type check
```

See [Contributing](CONTRIBUTING.md) for the full dev workflow.

## File Structure Quick Map

```
laguna/
├── pyproject.toml              ← package config & dependencies
├── config/example_config.yaml  ← canonical example: every subsystem section
├── src/laguna/
│   ├── core.py                 ← FlumeLab orchestrator
│   ├── config.py                ← Config: defaults + YAML loading
│   ├── experiment/runner.py    ← setup_run() / run_blocking(): config-driven wiring
│   ├── schedule/                ← ExperimentSchedule: CSV → spline-interpolated setpoints
│   ├── timing/                  ← ExperimentClock, Scheduler, EventLog, CheckpointStore
│   ├── weir/, flow/, gauge/    ← SAFL hydraulics hardware (weir motor, pump/VFD, level sensor)
│   ├── camera/                  ← DSLR (dslr.py) and Pi-networked (network.py) cameras
│   ├── robot/macron/            ← Modusystems OEM-2T gantry driver (current, hardware-verified)
│   ├── robot/__init__.py        ← older RobotController scaffold (Modbus/ASCII stub, unrelated to macron)
│   ├── data/                    ← data aggregation/export
│   └── storage/                 ← remote storage (S3/SFTP/local)
├── experiments/                 ← runnable experiment scripts
├── tests/                       ← unit tests
├── config/                      ← config templates
└── docs/                        ← this site (see mkdocs.yml at repo root)
```

## Common Errors

**`ModuleNotFoundError: No module named 'laguna'`**
Confirm you activated the `flumelab` conda env (`mamba activate flumelab`)
and that `environment.yml` was applied (`which python` should point into
`.../envs/flumelab`).

**Serial port permission denied (Linux/macOS)**
```bash
sudo chmod 666 /dev/ttyUSB0
# or, more permanently:
sudo usermod -a -G dialout $USER   # re-login required
```

**Weir/gauge/flow don't respond**
Check the port exists (`ls -la /dev/ttyUSB*`) and matches the config file;
call `lab.weir.get_status()` / `lab.gauge.get_status()` to see the last
known state. See the relevant [subsystem page](subsystems/weir.md) for
hardware-specific troubleshooting.

**Camera / gantry issues**
See [Camera USB setup](CAMERA_USB_SETUP.md), the [subsystem
pages](subsystems/camera.md), and the [Gantry usage guide](GANTRY_GUIDE.md)
— those own the hardware-specific troubleshooting for their subsystems.

## Documentation Map

- [Architecture](ARCHITECTURE.md) — module map, design patterns, and the
  non-blocking scheduler/threading model
- [Subsystem guides](subsystems/camera.md) — one page per subsystem
- [Gantry usage guide](GANTRY_GUIDE.md) / [technical reference](MACRON_GANTRY.md)
- [Contributing](CONTRIBUTING.md) — dev setup and how to add a subsystem
- [API reference](reference/index.md) — generated from docstrings
