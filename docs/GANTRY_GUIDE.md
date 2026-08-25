# Gantry Usage Guide

How to talk to the Modusystems OEM-2T gantry from `laguna`, and how it fits
alongside the weir, gauge, and camera subsystems under `FlumeLab`'s
scheduler. For deep protocol/wiring detail (exact vendor findings, IO
channel decode, safety-model rationale) see [`MACRON_GANTRY.md`](MACRON_GANTRY.md) —
this guide is the "how do I use it" companion to that technical reference.

## Topology

```
laguna (this PC) --SSH--> red.dyn.ucr.edu (Pi, "oak" account)
                               |
                          gantry_agent.py (sole owner of the serial port)
                               |
                          RS232 --> OEM-2T rev D controller --> gantry motors
```

The controller sits too far from the PC for direct serial, so a Raspberry
Pi next to it bridges the connection. Transports implement the same
`SnapConnection` interface (`src/laguna/robot/macron/connection.py`). The
default is `PiGantryConnection` — a persistent SSH+JSON agent that launches
and owns `gantry_agent.py` on the Pi, and the only transport that supports
topographic scanning. The former default, `RS232Connection` over the Pi's
raw TCP passthrough (`serial_bridge.py`, the `socket_bridge` transport), is
retired — see `docs/MACRON_GANTRY.md`, "Retired: `serial_bridge.py`".

## Where it sits in the software stack

```
┌───────────────────────────────────────────────────────────────┐
│                    FlumeLab (core.py)                         │
│   clock (ExperimentClock) · scheduler (Scheduler) · event_log │
└───────┬──────────┬──────────┬──────────┬──────────┬───────────┘
        │          │          │          │          │
        ▼          ▼          ▼          ▼          ▼
   ┌────────┐ ┌────────┐ ┌─────────┐ ┌────────┐ ┌──────────┐
   │ gantry │ │  weir  │ │  gauge  │ │  flow  │ │ cameras  │
   │(macron)│ │        │ │         │ │        │ │          │
   └────────┘ └────────┘ └─────────┘ └────────┘ └──────────┘
```

`GantryController` (`src/laguna/robot/macron/controller.py`) follows the
same subsystem contract as every other module here — its own docstring
cites `laguna.core.FlumeLab.add` and `laguna.weir.SaflWeirController` as the
pattern it mirrors: a `subsystem_name` attribute, plus
`connect()`/`disconnect()`/`get_status()`/`stop()`. It differs from
weir/gauge in one way: it exposes an explicit `from_config()` classmethod
rather than taking a plain config dict directly in `__init__`, because it
has more to assemble (transport, axes, IO map, homing, fences).

## Quick start

```python
from laguna.config import Config
from laguna.robot.macron import GantryController

config = Config(config_file="config/example_config.yaml")
gantry = GantryController.from_config(config)

gantry.connect()
print(gantry.get_status())
# {'subsystem': 'gantry', 'is_connected': True, 'safe_mode': True,
#  'positions': {'X': 0.005, 'Y': 0.005, 'Z': 0.005, 'Theta': 0.025}}
gantry.disconnect()
```

Register it with `FlumeLab` exactly like any other subsystem:

```python
from laguna.core import FlumeLab

lab = FlumeLab("config/example_config.yaml")
lab.add("gantry")        # -> lab.gantry
lab.connect_all()
lab.get_system_status()  # {'timing': {...}, 'gantry': {...}}
```

`get_system_status()` always includes a `"timing"` key even with zero
subsystems registered — `FlumeLab.__init__` creates `self.clock =
ExperimentClock()` unconditionally, and `get_system_status()` reads
`self.clock.now()` before it ever loops over `self._subsystems`. That's not
a gantry-specific quirk; every subsystem's status sits alongside it in the
same dict, keyed by `subsystem_name`.

## Protocol primer

Verified two independent ways (vendor's shipped Pascal interpreter source,
and the user's own hardware-tested Pi code) — see `MACRON_GANTRY.md` for
the full derivation. The short version:

- **Syntax**: `A<n> CMD [params]` (single axis), `C<n> CMD [params]`
  (coordinated group, must `INI` first), `CMD [params]` (global, no prefix).
- **Framing**: commands terminate on CR; responses terminate at a literal
  `>` (not CRLF).
- **Envelope**: success = `"0 <value> >"`, error = `"<code> >"` — parsed by
  checking the first token equals `"0"`, never by numeric magnitude.

## Axis model

8 firmware axis slots split across two PLC nodes — a local "commander" and
a second, networked "responder":

| Slot | Axis  | Node      | Notes                                    |
|------|-------|-----------|-------------------------------------------|
| 1    | X     | commander | |
| 2    | Y     | commander | electromagnetic brake |
| 3    | —     | commander | encoder-only, not exposed as a motion axis |
| 4    | —     | commander | encoder-only, not exposed as a motion axis |
| 5    | Z     | responder | electromagnetic brake |
| 6    | Theta | responder | rotary — no soft position limits |
| 7    | —     | responder | encoder-only, not exposed as a motion axis |
| 8    | —     | responder | encoder-only, not exposed as a motion axis |

`from laguna.robot.macron import X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS, ALL_AXES`
gives you these as ready-made `Axis` objects; `ALL_AXES` is the 4-tuple of
real, commandable axes.

## Command reference

**No-motion (read-only queries — always safe, allowlisted in `SAFE_COMMANDS`):**

| Command      | Meaning |
|--------------|---------|
| `WHT`        | WatchdogHasTripped |
| `UHD`        | UserHasDisabled |
| `UTP <n>`    | is user task *n* present/running |
| `ALI <n>`    | read analog input *n* |
| `INB <n>`    | read native digital input bit *n* |
| `ISI <n>`    | read isolated (IsoIO) input *n* (fails here — board not installed) |
| `ACP`        | actual/stepper position |
| `ENP`        | encoder position |
| `COP`/`DEP`  | commanded / destination position |
| `SPD`/`ACL`/`DCL` | current speed / accel / decel |
| `NLT`/`PLT`  | soft negative/positive travel limits |
| `MTR`/`ENA`  | is motor on / is axis enabled |
| `MIF`        | move-is-finished |
| `CAB`/`CAP`/`CAT` | capture bit / latched capture position / has-tripped |
| `PFP`/`PFV`  | profile phase / profile velocity |

**Motion / write (blocked whenever `safe_mode=True`, which is the default everywhere):**

| Command | Meaning |
|---|---|
| `BMT`/`BMB` | non-blocking absolute/relative move |
| `MVT`/`MVB` | blocking absolute/relative move |
| `JOG`       | continuous velocity jog |
| `AMT`/`AMB`/`ARC` | queue a waypoint/arc into the group curve buffer |
| `BST`/`ABT`/`STP` | controlled decel stop / emergency abort / immediate stop |
| `INI`/`CLR` | initialize a coordinated group / clear its curve buffer |
| `SOB`/`ISO` | set a native/isolated digital output — **this is how brakes engage/disengage** |
| `MTR`/`ENA` *(with an argument)* | enable/disable the motor drive or axis |
| `SPD`/`ACL`/`DCL`/`NLT`/`PLT` *(with an argument)* | change motion parameters or soft limits |
| `ACP`/`ENP` *(with an argument)* | zero/offset the position registers |
| `SCS`/`SCT`/`AIC` | configure and arm the hardware capture-latch (not currently used by homing — see `homing.py`) |

`MMCCommands` (`src/laguna/robot/macron/commands.py`) wraps almost all of
these in typed Python methods (`get_actual_position()`, `begin_move_to()`,
`disengage_brake()`, etc.) — read its docstrings for the full mapping.

## Safety model

Four independent layers, all active by default:

```
 caller
   │
   ▼
┌─────────────────────────┐   only mnemonics in SAFE_COMMANDS reach here;
│ 1. SAFE_COMMANDS         │   checked BEFORE any byte hits the wire, on
│    allowlist gate        │   both the PC side and (independently) the Pi
└──────────┬───────────────┘   side — a bug in one can't bypass the other
           ▼
┌─────────────────────────┐   GCodeExecutor.execute() only accepts a
│ 2. Fence check            │  CheckedTrajectory produced by its own plan()
│    (fences.py)            │  call — structurally impossible to skip
└──────────┬───────────────┘
           ▼
┌─────────────────────────┐
│ 3. dry_run                │  logs intended commands, sends nothing
└──────────┬───────────────┘
           ▼
┌─────────────────────────┐
│ 4. confirm_cb              │ per-motion-segment human confirmation hook
└──────────┬───────────────┘
           ▼
        hardware
```

**Update, 2026-07-28: motion has since been verified on real hardware** —
moves, brake engage/disengage, STOP, and a full topographic scan all ran
successfully (see `docs/MACRON_GANTRY.md`, `docs/RANGEFINDER_PROFILING.md`).
The safe-mode staging model below is still the right way to approach any
*new* motion path (a fresh axis, a new script) before trusting it, even
though the basic move/stop/scan path is now confirmed working.

## G-code (dry-run example)

`GantryController.from_config()` already builds a `GCodeExecutor` wired up
with the controller's own fences and homing procedure — access it via
`gantry.gcode` (its underlying `MMCCommands` instance is `gantry.cmd`):

```python
trajectory = gantry.gcode.plan("G1 X10 Y20 F600")   # parses + fence-checks, sends nothing yet
gantry.gcode.execute(trajectory)
```

**Careful**: `GantryController.from_config()` does *not* set `dry_run=True`
on the `GCodeExecutor` it builds — `gantry.gcode.execute()` will actually
try to send motion commands. What stops it under safe mode is the
connection-layer `SAFE_COMMANDS` gate (layer 1 above): `execute()` sends
`BMT`/`MVT`/etc., which `SafeModeConnection`/`PiGantryConnection` will
reject with a `SnapMotionError` while `safe_mode=True`. If you want a
guaranteed-silent dry run regardless of transport settings, build your own
`GCodeExecutor` with `dry_run=True` explicitly, as below — don't rely on
`gantry.gcode` alone for that.

To build a standalone executor yourself (e.g. for a quick one-off dry run
outside a full `GantryController`):

```python
from laguna.robot.macron.gcode import GCodeExecutor
from laguna.robot.macron.fences import FenceRegistry, TrajectoryChecker

checker = TrajectoryChecker(FenceRegistry())
executor = GCodeExecutor(gantry.cmd, checker, dry_run=True)  # dry_run: logs only, sends nothing

trajectory = executor.plan("G1 X10 Y20 F600")
executor.execute(trajectory)
```

## Scheduling it alongside weir / gauge

`weir` and `gauge` are polled today via `src/laguna/experiment/runner.py`'s
`setup_run()`, using the same `_register_action()` helper for every
subsystem — it supports three mutually-exclusive scheduling modes read
straight from each YAML section:

```yaml
gauge:
  interval_s: 5        # fire every 5 runtime seconds
weir:
  use_schedule: true    # fire at schedule-CSV time_s rows instead
```

`_register_action()` dispatches on whichever key is present:

```python
if use_sched:
    for t in times:
        lab.scheduler.at(float(t), make(float(t)), subsystem=subsystem, name=name)
elif "interval_s" in cfg:
    lab.scheduler.repeat(every=cfg["interval_s"], action=action, subsystem=subsystem, name=name)
elif "trigger_at" in cfg:
    for t in cfg["trigger_at"]:
        lab.scheduler.at(float(t), action, subsystem=subsystem, name=name)
```

`setup_run()` builds and registers every subsystem whose section is
present in the YAML — including `gantry:` — via `lab.add_all()`
(`laguna.core.FlumeLab.add_all`), which looks each section name up in
`laguna.registry.SUBSYSTEM_REGISTRY` and calls its `from_config()` for
you. No manual `lab.add(gantry)` needed; `lab.gantry` is populated by the
time `setup_run()` returns.

The gantry still has no scheduled action of its own the way `gauge`/`weir`
do — it only ever moves as part of a scan (see `laguna.scanner.gocator`'s
`_scan_gocator` closure in `setup_run()`), so there's no built-in periodic
status-logging equivalent to `_log_gauge`/`_log_weir_status`. Add one the
same way if you want it:

```python
# alongside _log_gauge/_log_weir_status in setup_run()
def _log_gantry_status():
    try:
        status = lab.gantry.get_status()
        logger.info("Gantry positions: %s", status.get("positions"))
        lab.event_log.log(lab.clock.elapsed(), "gantry", "get_status", str(status))
    except Exception as e:
        logger.error("Gantry status query failed: %s", e)
        raise

if "gantry" in lab._subsystems:
    _register_action(lab, lab.config.get("gantry"), "gantry", "log_status",
                     action=_log_gantry_status)
```

### Full example sketch

```python
from laguna.experiment import setup_run, run_blocking

lab = setup_run("config/example_config.yaml", schedule="my_schedule.csv")
# lab.gantry is already connected and registered — nothing more to add.

lab.scheduler.repeat(every=10, action=lambda: lab.event_log.log(
    lab.clock.elapsed(), "gantry", "get_status", str(lab.gantry.get_status())
), subsystem="gantry", name="log_status")

run_blocking(lab, duration=300)  # runs weir/gauge/gantry/cameras together
```

## Reference: how weir/gauge map config to their Python API

Useful to compare against, since the gantry deliberately follows the same
shape:

| | weir / gauge | gantry |
|---|---|---|
| Factory | `SaflWeirController(cfg)` — constructor *is* the factory | `GantryController.from_config(config)` — explicit classmethod, takes the whole `Config` |
| Registration | `lab.add("weir")` → `lab.weir` | `lab.add("gantry")` → `lab.gantry` |
| Status | `get_status()` → `{"is_connected": ..., "elevation_mm": ..., ...}` | `get_status()` → `{"subsystem": "gantry", "is_connected": ..., "safe_mode": ..., "positions": {...}}` |
| Scheduling | `interval_s` (status polling) or `use_schedule` (CSV-driven motion, e.g. `weir.go_to_elevation()`) | no built-in scheduled action — it only moves as part of a scan; add a `_log_gantry_status`-style closure yourself if you want periodic polling, same `_register_action()` mechanism |

## Further reading

- [`MACRON_GANTRY.md`](MACRON_GANTRY.md) — full technical reference: protocol derivation, IO channel decode, file-by-file breakdown, current hardware-verification status.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — overall `laguna` module map (note: some of it predates the current `FlumeLab` API — cross-check against `src/laguna/core.py` for anything load-bearing).
- [`QUICKREF.md`](QUICKREF.md) — quick command reference for the rest of the lab's subsystems.
- `src/laguna/robot/macron/` — the driver itself; every public method is documented inline.
- `tests/test_macron_*.py` — runnable, offline examples of every command this guide describes (via `FakeSnapConnection`), useful as copy-paste starting points.
