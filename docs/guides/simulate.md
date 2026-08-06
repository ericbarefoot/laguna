# Rehearsing safely with `simulate=True`

A flume run is expensive to set up and impossible to repeat identically. A
schedule with a typo, a survey that overruns the gap between events, or a
scan spec missing a key are all mistakes you'd rather find at a desk than
after the water is up. `FlumeLab(..., simulate=True)` runs your *actual*
script — same scheduler, same clock, same event log, same fence checks —
against simulated subsystems instead of hardware, so those mistakes surface
before anything is connected to a wire.

This is not optional polish — it's the recommended first step for any new
or modified experiment script, and the safest way to explore the API.

## What's real and what's not

| Real | Simulated |
|---|---|
| Scheduler, clock, event log, run manifest | Every subsystem in `laguna.registry.SUBSYSTEM_REGISTRY` (gantry, gocator, weir, flow, gauge, cameras, rangefinders) |
| Frame transforms, survey planner, safety verbs | Sensor readings — always `NaN`, never a fabricated plausible number |
| Fence checking (a rehearsed move that violates a fence still raises) | Gantry moves complete instantly; a Gocator scan returns a small fixed-size synthetic surface |
| Timing — a 300 s run takes 300 s of wall-clock time unless you set `speed_factor` | Camera captures — placeholder filenames, no files written |

Rehearsal proves your schedule and plan are well-formed and fire in the
right order. It **cannot** catch physical mistakes — a mounting sign error,
an infeasible feed rate, a target outside the real work envelope — because
none of those exist in simulation. It's a stand-in for the *script*, not
the apparatus.

## Minimal example

Take the [Getting started](getting-started.md) script and change one
argument:

```python
from laguna import FlumeLab
from laguna.weir import SaflWeirController
from laguna.flow import SaflFlowController

lab = FlumeLab("config/example_config.yaml", simulate=True)   # <-- only change

lab.add(SaflWeirController(lab.config.get("weir")))
lab.add(SaflFlowController(lab.config.get("flow")))

assert lab.connect_all()      # succeeds; nothing is on the network

with lab.experiment() as clock:
    lab.weir.home()
    lab.weir.set_elevation(150.0)
    lab.flow.start()
    lab.flow.set_flowrate(20.0)
    clock.wait_until(2.0)
    lab.flow.stop()

lab.disconnect_all()
```

Running this prints a `SIMULATION MODE` warning at startup, then executes
identically to the real thing — every log line, every event, every fence
check — with zero hardware involved. This exact script has been run as
part of writing this guide.

## Scheduled experiments rehearse too

The same switch works on a scheduled, multi-subsystem experiment:

```python
from laguna import FlumeLab, CheckpointStore

lab = FlumeLab("config/example_config.yaml", simulate=True)
lab.add("weir")

store = CheckpointStore("./checkpoint.json", resume=False)

def poll():
    return lab.weir.get_status()

lab.scheduler.repeat(every=1, action=poll, subsystem="weir", name="status_poll")

with lab.experiment() as clock:
    lab.scheduler.run(duration=4)

lab.disconnect_all()
```

Every scheduled action still fires on schedule, in real wall-clock time,
against the simulated weir — this is what actually proves a schedule is
well-formed rather than merely syntactically valid.

## Speeding up a rehearsal

Pass `speed_factor` to compress wall-clock time for a long rehearsal —
only valid alongside `simulate=True`, since accelerating a run driving real
hardware isn't supported:

```python
lab = FlumeLab("config/example_config.yaml", simulate=True, speed_factor=10.0)
```

## Once a rehearsal passes

Passing rehearsal means the script is structurally sound — it does not
mean it's safe to run against real hardware unreviewed. Re-check config
values (ports, IPs, calibration constants), and see
[Driving the gantry safely](gantry-motion.md) before flipping any
`safe_mode`/`ALLOW_MOTION` flag to actually move something.
