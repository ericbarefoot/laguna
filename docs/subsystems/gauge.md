# Gauge

Water-surface elevation sensing via a Massa ultrasonic distance sensor.
Lives at `src/laguna/gauge/`, and like `weir`, it's a thin wrapper around
`safl_ocean_hardware` — the actual serial protocol to the Massa unit is
implemented there, not in this repo.

## Why it exists

`gauge` is a purely read-only subsystem: it never actuates anything, it
just tells you where the water surface currently is. It follows the usual
subsystem contract — `subsystem_name = "gauge"`, `connect()`/
`disconnect()`/`get_status()` — and registers via `lab.add(gauge)` →
`lab.gauge`.

`WaterLevelSensor` (`src/laguna/gauge/sensor.py`) is an ABC; the only
concrete implementation is `SaflWaterLevelSensor`, backed by
`safl_ocean_hardware.massa.MassaSensor`. If that package isn't installed,
`connect()` logs a warning and returns `False` rather than raising.

## Quick start

```python
from laguna.config import Config
from laguna.gauge import SaflWaterLevelSensor

config = Config(config_file="config/example_config.yaml")
gauge = SaflWaterLevelSensor(config.get("gauge"))

gauge.connect()
elevation_mm = gauge.read_mm()             # instantaneous reading
smoothed_mm = gauge.read_mm_smoothed()     # sensor's own moving average
print(gauge.get_status())
# {'is_connected': True, 'elevation_mm': 87.5, 'temperature_c': 21.0, 'signal_strength': ...}
gauge.disconnect()
```

## How the elevation number is derived

The Massa sensor is mounted above the water and measures **downward
distance** to the surface, in centimeters. `laguna` converts that to an
elevation in millimeters with:

```
elevation_mm = offset_mm - distance_cm * 10.0
```

As the water rises, the measured distance shrinks, so elevation increases —
the `-` sign is intentional. `offset_mm` is a per-installation calibration
constant (sensor mounting height above some reference datum, typically the
flume bed or a fixed benchmark) that you set in config; there's no
auto-calibration.

`read_mm()` returns a single instantaneous reading. `read_mm_smoothed()`
instead reads the sensor's own onboard FIFO moving average
(`dist_cm_array_moving_avg`, exposed by `safl_ocean_hardware`) and applies
the same conversion — use this one for logging if the raw signal is noisy;
it returns `float("nan")` if the sensor hasn't populated that buffer yet.

## Config

```yaml
gauge:
  port: /dev/ttyUSB2          # Massa M-5000 ultrasonic sensor
  sensor_ids: [0]             # supports multiple heads on one Massa bus
  offset_mm: 0.0
  interval_s: 5                # read every 5 s
```

`Config._get_defaults()` mirrors this (`port`, `sensor_ids`, `offset_mm`).
An optional `offsets` key (plural, per-sensor-id offsets) is also read by
`SaflWaterLevelSensor.__init__` but has no corresponding entry in
`Config._get_defaults()` or `config/example_config.yaml` — it exists for
multi-head setups but isn't demonstrated anywhere in this repo.

## Scheduling it via `setup_run()`

Because the gauge has nothing to set — only to read — `setup_run()` only
ever wires it up as a periodic status poll: `interval_s` (or `trigger_at`)
fires `gauge.read_mm()` and logs `elevation_mm=<value>` to the event log.
`use_schedule: true` is structurally accepted by the same generic
`schedule_action()` helper used for every subsystem, but there's no
schedule-CSV column that maps to a gauge setpoint, so in practice you'd
only reach for `interval_s`.

As with `weir`, a recent hotfix makes the scheduled read call
`gauge.connect()` again before every `read_mm()` — real hardware has been
observed to drop its serial connection between polls.

## Troubleshooting (from field notes)

- **Garbage or `NaN` readings**: check the physical connection
  (`/dev/ttyUSB2` by default), confirm `offset_mm` matches your actual
  flume geometry, and make sure the sensor has power — a disconnected or
  unpowered Massa unit tends to fail quietly rather than raise.
- **`read_mm_smoothed()` returns `NaN`**: the onboard moving-average buffer
  hasn't filled yet; read a few times before relying on it, or fall back to
  `read_mm()`.

## Further reading

- [API reference](../reference/gauge.md) — generated from docstrings.
- [Experiment runner](experiment.md) — how `gauge:` config drives polling.
- `src/laguna/gauge/sensor.py` — the driver itself.
