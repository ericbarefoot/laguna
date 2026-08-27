# Flow

Pump flow-rate control plus the two inflow solenoid valves. Lives at
`src/laguna/flow/`. Both the Fuji VFD pump drive and the Teknic ClearCore
that switches the qin/qaux solenoids are wired to the confluence node on
red.lab, which exposes each over its own MQTT interface — this subsystem
is an MQTT client, not a direct-serial driver; see
`src/laguna/flow/controller.py`.

## Why it exists

`flow` is one of the opt-in subsystems `FlumeLab` coordinates, following
the same shape as `weir`/`gauge`: `subsystem_name = "flow"`,
`connect()`/`disconnect()`/`get_status()`, registered via `lab.add(flow)`
→ `lab.flow`.

`FlowController` (`src/laguna/flow/controller.py`) is an ABC; the only
concrete implementation is `SaflFlowController`. Build it via
`SaflFlowController.from_config()`, which derives its MQTT topics from
the shared `mqtt.node_name` — `connect()` fails (returns `False`) rather
than raising if the confluence node can't be reached.

## Quick start

```python
from laguna.config import Config
from laguna.flow import SaflFlowController

config = Config(config_file="config/example_config.yaml")
flow = SaflFlowController.from_config(config)

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

`SaflFlowController` talks to two independent confluence interfaces over
one shared `MqttSubscriber` (a single MQTT client avoids the
duplicate-client-ID reconnect fight documented in
`laguna.mqtt.subscriber.MqttSubscriber`):

- **The Fuji VFD** — confluence's `Fuji_Frenic_VFD` interface
  (`vfd_topic_status`/`vfd_topic_commands`/`vfd_topic_replies`) — sets
  pump speed by frequency and starts/stops it via request/reply commands
  (`set_setpoint_hz`, `start_motor`, `stop_motor`, `clear_faults`).
- **The shared ClearCore's `flow_valve` axis** — confluence's
  `Teknic_ClearCore` interface (`valve_topic_status`/
  `valve_topic_commands`/`valve_topic_replies`) — drives the `qin`/`qaux`
  solenoid digital outputs (`set_io` on channels 0/1), *not* any motion.
  It is the same physical ClearCore the weir gate axis lives on (one
  controller, two axes), published/commanded as its own confluence
  interface/topic set so the two subsystems don't step on each other.

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
# weir/gauge/flow are MQTT clients of the confluence node on red.lab, not
# direct USB serial — their topics default to `{mqtt.node_name}/...`,
# so the one thing you actually need to set is mqtt.node_name matching
# red.lab's confluence_config.json "Node Name".
# mqtt:
#   node_name: "UCRS Confluence Node 1"

flow:
  calibration_file: config/my_pump_calibration.csv  # preferred — see "Flow-rate calibration" above
  C0: 4.902                  # legacy fallback, only used without calibration_file — see above
  C1: 58.49
  C2: 0.08956
  # command_timeout_s: 5.0    # seconds to wait for a confluence reply before failing
  # A topic_* key overrides the mqtt.node_name-derived default for a single
  # subsystem if ever needed, e.g.:
  # vfd_topic_status: "SAFL Confluence Node 1/Fuji_Frenic_VFD"
  # vfd_topic_commands: "SAFL Confluence Node 1/Fuji_Frenic_VFD/commands"
  # vfd_topic_replies: "SAFL Confluence Node 1/Fuji_Frenic_VFD/replies"
  # valve_topic_status: "SAFL Confluence Node 1/flow_valve"
  # valve_topic_commands: "SAFL Confluence Node 1/flow_valve/commands"
  # valve_topic_replies: "SAFL Confluence Node 1/flow_valve/replies"
  # Scheduling (choose one):
  # interval_s: 20            # poll status every 20 s (read-only)
  # use_schedule: true        # follow schedule CSV (pump_flow_lpm, qin_open, qaux_open columns)
```

The `flow:` section is commented out by default in
`config/example_config.yaml` — omit it entirely to disable the subsystem
if you don't have a pump/VFD attached. Build with
`SaflFlowController.from_config(config)`, not the constructor directly, so
the topic defaults get derived from `mqtt.node_name`.

## Scheduling it via `setup_run()`

Same three-way choice as every other subsystem, via `schedule_action()`
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
