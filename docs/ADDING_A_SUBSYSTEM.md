# Adding a new subsystem

The mechanical checklist (base contract, registry entry, tests, docs) lives
in [Contributing](CONTRIBUTING.md#adding-a-new-subsystem) — this page is
about the design decisions that checklist doesn't cover: how a new sensor
or actuator should be shaped depending on where it lives and what it does,
how to give it a rehearsal path, and two full worked examples.

## Two questions decide the shape

Every subsystem answers two independent questions. Answer both before
writing any code — they determine which existing subsystem is the closest
precedent to copy from.

**1. Does it move with the gantry, or sit in one place?**

| | Meaning | Precedent |
|---|---|---|
| **Static** | Fixed location for the whole experiment | `weir`, `flow`, `gauge` |
| **Gantry-mounted** | Carried by the gantry; where it measures depends on gantry position | `od2000`, `wtt12l` (rangefinders) |

**2. Does it change the experiment's state, or only observe it?**

| | Meaning | Precedent |
|---|---|---|
| **Sensor** | Reads/observes only — a reading doesn't change anything | `gauge`, `od2000`, `wtt12l` |
| **Actuator** | Commands change a real boundary condition | `weir`, `flow` |

These combine into four cases. The first three all have real precedent in
this codebase; the fourth doesn't yet — if you're building one, treat it
as "gantry-mounted sensor" (placement) plus "actuator" (safety verbs,
archival logging) combined, and expect to make judgment calls neither
worked example below covers exactly.

| | Sensor | Actuator |
|---|---|---|
| **Static** | `gauge` — [worked example below](#worked-example-1-static-sensor) | `weir`, `flow` — [what's different](#what-changes-for-an-actuator) |
| **Gantry-mounted** | `od2000`/`wtt12l` — [worked example below](#worked-example-2-gantry-mounted-sensor) | No precedent yet — combine both sections above |

## Worked example 1: static sensor

A fictional static pressure transducer at the flume bed — gathers data
only, never moves, no precedent to reference beyond `gauge` itself. This
is the simplest case and the one to copy from if you're unsure.

```python
# src/laguna/pressure/sensor.py
"""Static pressure transducer subsystem."""

from typing import TYPE_CHECKING, Any, Dict, Optional
import logging

from ..subsystem_logging import SubsystemLogging

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)

try:
    from safl_ocean_hardware.pressure import PressureTransducer as _PressureTransducer
except ImportError:
    _PressureTransducer = None


class SaflPressureSensor(SubsystemLogging):
    """Bed-mounted pressure transducer, fixed for the whole experiment."""

    subsystem_name = "pressure"

    def __init__(self, config: Dict[str, Any]):
        self._port = config.get("port", "/dev/ttyUSB3")
        self.log_level = config.get("log_level", "INFO")
        self.event_log_verbosity = config.get("event_log_verbosity", "INFO")
        self._simulated = config.get("simulated", False)
        self._sensor = None
        self._is_connected = False

    @classmethod
    def from_config(cls, config: "Config") -> "SaflPressureSensor":
        return cls(config.get("pressure"))

    def connect(self) -> bool:
        if self._simulated:
            self._is_connected = True
            return True
        if _PressureTransducer is None:
            logger.warning("safl_ocean_hardware is not installed; SaflPressureSensor cannot connect")
            return False
        try:
            self._sensor = _PressureTransducer(self._port)
            self._is_connected = self._sensor.connect()
            return self._is_connected
        except Exception as e:
            logger.error(f"Failed to connect pressure sensor: {e}")
            return False

    def disconnect(self) -> None:
        if self._sensor and self._is_connected:
            self._sensor.disconnect()
        self._is_connected = False

    def read_kpa(self) -> float:
        """A reading — not a state change, so this logs to the operational
        log only (see SaflWaterLevelSensor.read_mm() for the precedent),
        not the archival event log."""
        if self._simulated:
            value = float("nan")
        else:
            value = self._sensor.read_kpa()
        logger.info("read_kpa: %.2f", value)
        return value

    def get_status(self) -> Dict[str, Any]:
        return {"is_connected": self._is_connected}
```

That's the whole subsystem. Register it (`src/laguna/registry.py`):

```python
from .pressure.sensor import SaflPressureSensor

SUBSYSTEM_REGISTRY: Dict[str, Type] = {
    "gantry": GantryController,     # ...existing entries...
    "pressure": SaflPressureSensor,
}
```

Give it a simulated backend a config author can reach:

```python
for section in ("weir", "flow", "gauge", "pi_cameras", "dslr_cameras",
                "od2000", "wtt12l", "pressure"):   # add it here
    if section in out:
        sub = dict(out[section])
        sub["simulated"] = True
        out[section] = sub
```

(`src/laguna/simulation.py`'s `simulate_config()` — see
`_SIMULATED_SECTIONS`.) No `SimulatedPressureTransducer` class was even
needed here — `self._simulated` short-circuits inside `connect()`/
`read_kpa()` directly, same as `SaflWaterLevelSensor`. Build a small
in-memory stand-in class in `laguna/simulation.py` only if the real driver
object has enough methods that inlining the branches gets unwieldy — see
`SimulatedTeknicMotor`/`SimulatedVFD` for when that was worth it (weir and
flow share one).

And use it:

```python
lab = FlumeLab("config.yaml")
lab.add("pressure")
lab.pressure.connect()
lab.pressure.read_kpa()
```

## Worked example 2: gantry-mounted sensor

A fictional turbidity probe, carried by the gantry — where it measures
depends on where the gantry is. **The subsystem class itself is
unchanged** from a static sensor; nothing about being gantry-mounted
requires the class to know about the gantry. What's different is entirely
at the config/calling-code level:

```python
# src/laguna/turbidity/sensor.py — identical shape to SaflPressureSensor
# above; subsystem_name = "turbidity", read_ntu() instead of read_kpa().
# Omitted here since it's the same pattern — copy worked example 1.
```

**1. Add a mounting offset under `frames:`** — where the probe measures
relative to the gantry's commanded point (measure this once on the real
rig; see [Reference frames](subsystems/frames.md)):

```yaml
frames:
  instruments:
    turbidity:
      translation: [30.0, -15.0, 0]   # probe's sensing point vs. gantry's commanded point
```

**2. Use `lab.place()`, not `lab.move_to()`, before reading** — `place()`
looks up the `turbidity` frame entry and works out what gantry position
puts the probe over the point you actually want measured:

```python
lab.add("gantry").add("turbidity")
lab.connect_all()

lab.place("turbidity", [100.0, 200.0, 0])   # moves the gantry so the PROBE is at (100,200,0)
lab.turbidity.read_ntu()
```

An instrument with no `frames:` entry is treated as measuring exactly at
the gantry's commanded point — `place()` degrades to `move_to()` — so
step 1 is easy to skip while developing and add once you've measured the
real offset.

**3. Optional: scheduled scanning while the gantry moves.** `od2000`/
`wtt12l` also support `lab.acquire_scan("od2000", start=..., end=...)` —
continuous reading *while* the gantry moves along a transect, not just a
single reading at a stationary point. That's `TopographicProfiler`
(`laguna.robot.macron.profiler`), built specifically for the rangefinders'
MQTT streaming path — treat it as an advanced pattern to study directly
in that module if your sensor needs the same, not something every
gantry-mounted sensor should implement.

## What changes for an actuator

Compare `SaflWeirController`/`SaflFlowController` against the sensor
examples above — three real differences, no new example needed:

1. **Safety verbs are real, not no-ops.** `pause()`/`resume()`/`stop()`/
   `estop()` must actually quiesce the hardware (see `laguna.safety`'s
   module docstring for the vocabulary) — a sensor's default no-op verbs
   are fine because there's nothing to quiesce; an actuator changing a
   real boundary condition is exactly what these exist for.
2. **State-changing calls log to the archival event log**, not just the
   operational log — inherit `SubsystemLogging` and call
   `self.log_event("go_to_elevation", target_mm=mm)` at the point a
   command is actually issued (see `SaflWeirController.go_to_elevation()`).
   A pure reading never does this (see
   [`laguna.subsystem_logging`'s module docstring](FLUMELAB_SETUP_GUIDE.md#two-tier-logging)
   for the full reasoning) — but a command that changes what the
   experiment is doing always does.
3. **The simulated backend must not silently "succeed" a read that was
   never really taken.** Commands (`go_to_elevation`, `set_flowrate`)
   still succeed and log normally under `simulated: True` — that's what
   proves the schedule fires correctly. Readings *of the actuator's own
   state* (`get_elevation()`, `get_flowrate()`'s driver-backed fields)
   still come back `NaN`, same rule as any sensor. The one exception
   worth knowing: `get_flowrate()` returns the setpoint the caller itself
   commanded, not a driver reading — echoing back known data isn't
   fabricating anything, so it stays a real number.

## Checklist

The rest — `subsystem_name`, `get_status()`, tests, registry entry,
`docs/subsystems/` page — is exactly [Contributing's Adding a New
Subsystem checklist](CONTRIBUTING.md#adding-a-new-subsystem). Nothing
above changes it; this page is about which precedent to copy from and why.

## Further reading

- [Contributing — Adding a New Subsystem](CONTRIBUTING.md#adding-a-new-subsystem) —
  the mechanical checklist.
- [FlumeLab setup & logging guide](FLUMELAB_SETUP_GUIDE.md) — the registry,
  `from_config()`, and the two-tier logging model in depth.
- [Reference frames](subsystems/frames.md) — `frames:` config,
  `InstrumentFrame`, the mounting-offset math behind `lab.place()`.
- `src/laguna/simulation.py` — the module docstring covers what a
  rehearsal does and does not catch.
