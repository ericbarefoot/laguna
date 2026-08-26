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

## Flow-rate calibration: `calibration_file` vs. `C0`/`C1`/`C2`

`set_flowrate(lpm)` converts a requested flow rate (L/min) into the VFD
drive frequency (Hz) that actually produces it — the pump has no native
notion of L/min, only a frequency setpoint. That conversion needs a
per-pump/plumbing calibration; there are two ways to provide one.

**`calibration_file` (preferred)** — a CSV written by
`laguna.flow.calibration.PumpCalibration.to_csv()`, fitting a quadratic
curve `discharge_lpm = f(freq_hz)` against real measured points (command
a known frequency, measure the actual discharge — bucket and stopwatch, a
flow meter, whatever's available), then inverting that fit numerically to
answer "what Hz gives me this many L/min." See
[the API reference](../reference/flow.md#calibration) for
`PumpCalibrationPoint`/`PumpCalibration.fit()`/`hz_for_lpm()`, and
`config/example_pump_calibration.csv` for a worked example file (fit from
made-up but plausible data — replace with real measurements from your own
pump before trusting it).

```python
from laguna.flow.calibration import PumpCalibration, PumpCalibrationPoint

points = [
    PumpCalibrationPoint(freq_hz=10.0, discharge_lpm=1.1),
    PumpCalibrationPoint(freq_hz=20.0, discharge_lpm=2.4),
    PumpCalibrationPoint(freq_hz=30.0, discharge_lpm=3.9),
    # ... at least 3 points; more, spread across the range you'll
    # actually use, gives a more trustworthy fit.
]
cal = PumpCalibration.fit("my pump - main inlet", points, hz_max=60.0)
print(f"r_squared: {cal.r_squared:.4f}")   # check the fit quality
cal.to_csv("config/my_pump_calibration.csv")
```

```yaml
flow:
  calibration_file: config/my_pump_calibration.csv
```

**`C0`/`C1`/`C2` (legacy)** — only used when `calibration_file` is unset.
Three floats plugged directly into `Hz = C2*Q^2 + C1*Q + C0` inside
`set_flowrate()` itself, with no separate fit-and-save step. **The
defaults shipped in `Config._get_defaults()`/`config/example_config.yaml`
(`C0=4.902, C1=58.49, C2=0.08956`) were found to be wrong on first live
hardware test** — they compute a 1 L/min request to 63.48 Hz, clamped to
the VFD's 60 Hz max, i.e. essentially full speed for what was meant to be
a low test rate. Don't rely on these numbers for any real pump without
re-deriving them (or, better, switching to a `calibration_file`) — a
single triple of coefficients also can't represent a curve that isn't a
clean quadratic the way a real fit-from-data calibration can.

## Config

```yaml
flow:
  vfd_port: /dev/ttyUSB3
  vfd_slave_id: 1
  motor_port: /dev/ttyUSB1   # shared serial port with weir (see limitation above)
  motor_baudrate: 9600
  calibration_file: config/my_pump_calibration.csv  # preferred — see "Flow-rate calibration" above
  C0: 4.902                  # legacy fallback, only used without calibration_file — see above
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
