# Weir

Elevation control for the flume's tailgate weir: a stepper-driven gate that
sets the downstream water-surface elevation. Lives at `src/laguna/weir/`
and is a thin wrapper around the `safl_ocean_hardware` package's Teknic
ClearCore motor driver — it does not talk to the stepper directly.

## Why it exists

`weir` is one of the opt-in subsystems `FlumeLab` coordinates (alongside
`gauge`, `flow`, cameras, and the [gantry](../GANTRY_GUIDE.md)). It follows
the same shape as every other subsystem here: a `subsystem_name` class
attribute (`"weir"`), `connect()`/`disconnect()`/`get_status()`, and
registration via `lab.add(weir)` → `lab.weir`.

`WeirController` (`src/laguna/weir/controller.py`) is an ABC; the only
concrete implementation today is `SaflWeirController`, which requires the
`safl_ocean_hardware` package to be installed. If it isn't, `connect()`
logs a warning and returns `False` rather than raising — the subsystem is
safe to register even on a machine with no hardware attached, it just
won't connect.

## Quick start

```python
from laguna.config import Config
from laguna.weir import SaflWeirController

config = Config(config_file="config/example_config.yaml")
weir = SaflWeirController(config.get("weir"))

weir.connect()
weir.home()                    # find home switch, apply home_offset_mm
weir.set_elevation(150.0)      # mm, absolute
weir.wait_for_move(timeout=30.0)
print(weir.get_status())
# {'is_connected': True, 'elevation_mm': 150.0, 'motor': {...raw ClearCore status...}}
weir.disconnect()
```

Register it with `FlumeLab` like any other subsystem:

```python
from laguna.core import FlumeLab

lab = FlumeLab("config/example_config.yaml")
lab.add(weir)          # -> lab.weir
lab.connect_all()
```

## Units

Positions and velocities are passed through in **mm** and **mm/s** with no
scaling applied in this codebase — the ClearCore firmware itself does the
steps-per-mm conversion (configured on its SD card, not in `laguna`). Note
that `Config._get_defaults()` still carries a legacy `steps_per_mm` default
under the top-level `robot:` section from an earlier, more generic
configuration shape; `SaflWeirController` does not read or use it.

## API surface

| Method | What it does |
|---|---|
| `connect()` / `disconnect()` | Open/close the serial connection to the ClearCore |
| `home()` | Run the ClearCore's homing routine, offset by `home_offset_mm` |
| `set_elevation(mm)` | Issue an absolute-position move |
| `go_to_elevation(mm)` | Issue an absolute-position move, returns a bool (used by the scheduled-motion path — see below) |
| `set_velocity(mm_per_sec)` / `get_velocity()` | Set/read the move speed applied to the *next* move |
| `enable()` / `disable()` | Enable/disable the motor drive (disable to reposition by hand) |
| `wait_for_move(timeout=30.0)` | Block until the current move's HLFB signal indicates completion |
| `clear_faults()` | Clear ClearCore fault state |
| `stop()` | Stop motion immediately |
| `get_status()` | `{"is_connected", "elevation_mm", "motor": {...}}` — `motor` is the raw dict from the ClearCore's status poll (keys seen in the wild via `runner.py`: `Enabled`, `MotorInFault`, `StepsActive`, `VelSetPoint`, `position`) |

**Honesty note**: `set_elevation()` and `go_to_elevation()` both end up
calling into `safl_ocean_hardware`'s `TeknicMotor` (`set_absolute_position()`
and `move_to_position()` respectively). That package lives outside this
repo, so the exact blocking/non-blocking distinction between the two calls
isn't verifiable from `laguna`'s source alone — treat `go_to_elevation()`
(used by the scheduler, see below) as the one to reach for in experiment
code, and use `wait_for_move()` if you need to block until motion finishes.

## Config

```yaml
weir:
  port: /dev/ttyACM0          # or /dev/ttyUSB1 — Teknic ClearCore motor controller
  baudrate: 9600
  home_offset_mm: 0.0
  # Scheduling (choose one):
  interval_s: 20               # poll status every 20 s (read-only, no motor motion)
  # use_schedule: true         # follow schedule CSV (calls go_to_elevation)
```

`Config._get_defaults()` has matching defaults under `weir:` (`port`,
`baudrate`, `steps_per_mm` — unused, see above — and `home_offset_mm`).

## Scheduling it via `setup_run()`

`src/laguna/experiment/runner.py`'s `setup_run()` reads the `weir:` section
and registers exactly one scheduled action, chosen by which key is present:

- **`interval_s: N`** — read-only status poll every `N` seconds
  (`weir.get_status()`), logged to the event log. No motion is issued.
- **`use_schedule: true`** — requires a schedule CSV with a
  `weir_elevation_mm` column (see [schedule](schedule.md)). At every
  `time_s` row, the interpolated target elevation is sent via
  `weir.go_to_elevation(target_mm)`. If the CSV has no such column,
  `setup_run()` falls back to read-only status polling and logs a warning
  instead of failing outright.

As of a recent hotfix, the scheduled action calls `weir.connect()` again
before every read or move — real hardware here has been observed to drop
its connection between polls, so each scheduled tick defensively
reconnects rather than assuming the connection from `connect_all()` is
still good.

## Further reading

- [API reference](../reference/weir.md) — generated from docstrings.
- [Experiment runner](experiment.md) — how `weir:` config drives scheduling.
- [Schedule](schedule.md) — the CSV format and `weir_elevation_mm` spline interpolation.
- `src/laguna/weir/controller.py` — the driver itself.
