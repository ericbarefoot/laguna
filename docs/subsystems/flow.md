# Flow

Pump flow-rate control plus the two inflow solenoid valves. Lives at
`src/laguna/flow/`, wrapping a Fuji VFD (variable-frequency drive) pump
controller and a Teknic ClearCore's digital IO — both via
`safl_ocean_hardware`, not implemented in this repo.

## Why it exists

`flow` is one of the opt-in subsystems `FlumeLab` coordinates, following
the same shape as `weir`/`gauge`: `subsystem_name = "flow"`,
`connect()`/`disconnect()`/`get_status()`, registered via `lab.add(flow)`
→ `lab.flow`.

`FlowController` (`src/laguna/flow/controller.py`) is an ABC; the only
concrete implementation is `SaflFlowController`. If `safl_ocean_hardware`
isn't installed, `connect()` logs a warning and returns `False`.

## Quick start

```python
from laguna.config import Config
from laguna.flow import SaflFlowController

config = Config(config_file="config/example_config.yaml")
flow = SaflFlowController(config.get("flow"))

flow.connect()
flow.qin = True             # open the main inflow solenoid
flow.qaux = False            # keep the auxiliary line closed
flow.set_flowrate(20.0)      # L/min
flow.start()                 # start the pump at that setpoint
print(flow.get_status())
# {'is_connected': True, 'flowrate_lpm': 20.0, 'qin_open': True, 'qaux_open': False,
#  'vfd_state': ..., 'vfd_estop': ..., 'vfd_setpoint_hz': ...}
flow.stop()
flow.disconnect()
```

## Two physically separate things under one subsystem

`SaflFlowController` owns two independent hardware connections:

- **A Fuji VFD** (`self._vfd`, on `vfd_port`/`vfd_slave_id`) — sets pump
  speed by frequency and starts/stops it.
- **A Teknic ClearCore** (`self._motor`, on `motor_port`/`motor_baudrate`)
  — used here purely for its digital IO pins to drive the `qin`/`qaux`
  solenoid valves (`motor.set_io(0, state)` / `motor.set_io(1, state)`),
  *not* for any motion. It happens to be the same model of controller the
  weir subsystem uses for its stepper.

**Known limitation, called out directly in the source docstring**: in
production, this `TeknicMotor` instance should be the *same* one
`SaflWeirController` uses (the example config even comments
`motor_port: /dev/ttyUSB1  # shared serial port with weir`), but
`SaflFlowController` currently opens its own independent connection
instead of sharing one. If `weir` and `flow` are both configured against
the same serial port, that means two separate connections to the same
physical device — the source comment flags this explicitly as a
"self-contained first-draft" shortcut, not a deliberate design.

## The `C0`/`C1`/`C2` calibration coefficients

`Config._get_defaults()`'s `flow:` section defines three floats:

```python
"C0": 4.902,
"C1": 58.49,
"C2": 0.08956,
```

These are the coefficients of a quadratic pump curve that converts a
requested flow rate (L/min) into the VFD drive frequency (Hz) that
actually produces it:

```
Hz = C2 * Q^2 + C1 * Q + C0        # Q in L/min
```

`set_flowrate(lpm)` doesn't do this arithmetic itself — it hands `lpm` and
all three coefficients straight to
`safl_ocean_hardware`'s `vfd.set_freq_from_flowrate(lpm, C0, C1, C2)`,
which applies the formula and writes the resulting frequency to the drive.
The three constants are a **per-pump calibration**, not a physical
universal — they come from fitting a quadratic to that specific pump's
measured flow-vs-frequency curve, so a different pump (or the same pump
after maintenance/re-calibration) would need different values. The
defaults above and in `config/example_config.yaml` are this lab's current
calibration; don't reuse them for a different installation without
re-deriving the fit.

## Config

```yaml
flow:
  vfd_port: /dev/ttyUSB3
  vfd_slave_id: 1
  motor_port: /dev/ttyUSB1   # shared serial port with weir (see limitation above)
  motor_baudrate: 9600
  C0: 4.902                  # flowrate-to-frequency calibration: Hz = C2*Q^2 + C1*Q + C0
  C1: 58.49
  C2: 0.08956
  # Scheduling (choose one):
  # interval_s: 20            # poll status every 20 s (read-only)
  # use_schedule: true        # follow schedule CSV (pump_flow_lpm, qin_open, qaux_open columns)
```

The `flow:` section is commented out by default in
`config/example_config.yaml` — omit it entirely to disable the subsystem
if you don't have a pump/VFD attached.

## Scheduling it via `setup_run()`

Same three-way choice as every other subsystem, via `_register_action()`
in `src/laguna/experiment/runner.py`:

- **`interval_s: N`** — read-only `flow.get_status()` poll, logged to the
  event log.
- **`use_schedule: true`** — requires a schedule CSV with at least one of
  `pump_flow_lpm`, `qin_open`, `qaux_open` (see [schedule](schedule.md)).
  At every `time_s` row: the flow rate is set (linear interpolation), the
  pump is started if the interpolated rate is `> 0` else stopped, and both
  valve booleans are applied (step interpolation — they're not
  meaningfully interpolatable between rows). Missing columns default to
  `0.0`/`False` rather than erroring.

## Further reading

- [API reference](../reference/flow.md) — generated from docstrings.
- [Experiment runner](experiment.md) — how `flow:` config drives scheduling.
- [Schedule](schedule.md) — the CSV format and interpolation modes.
- `src/laguna/flow/controller.py` — the driver itself.
