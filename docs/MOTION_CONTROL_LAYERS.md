# Motion control layers & guards

Every way of putting the gantry in motion, and exactly which guards each one
passes through. The design rule is short:

> **Every move is fence-checked, except homing (no reference frame yet) and
> the two explicitly named `*_unfenced` calls.** Everything else — safe_mode,
> the halt latch, the motion arbiter — applies to *every* path, unfenced ones
> included.

If you find a way to move the gantry that isn't in the table below, that's a
bug.

## The public motion entry points

| Call | Fence-checked? | `safe_mode` | Halt latch | Arbiter | Blocks? | Position resynced after? |
|---|---|---|---|---|---|---|
| `lab.move_to(...)` / `lab.gantry.move_to(...)` | **Yes** — X/Y/Z as a ribbon (see below); Theta not (#62) | Yes | Yes | Yes | **No** — returns a `MoveHandle` | Yes |
| `lab.place(instrument, point)` | **Yes** (it's `move_to()`) | Yes | Yes | Yes | **No** | Yes |
| `lab.acquire_scan(...)` (Pi-agent rangefinder pass) | **Yes** — reposition via `move_to()`, the pass as a straight segment | Yes, client **and** agent side | Yes | Yes | Yes (it's a whole acquisition) | Yes |
| `lab.gocator.scan_with_gantry(...)` / `.acquire(...)` | **Yes** — the pass as a straight segment (`begin_scan_move`) | Yes | Yes | Yes | Yes | Yes |
| `SurveyRunner.run()` | **Yes** — every reposition and every pass | Yes | Yes | Yes, across each whole pass | Yes | Yes |
| `lab.gantry.gcode.plan()` / `.execute()` | **Yes** — the one fenced path underneath all of the above | Transport only¹ | Only if you pass a `guard` | **No** | Yes | Yes (its own cache) |
| `lab.gantry.home()` / `.home_axis()` / `.locate_limit_switch()` | **No — by design**: until homing finishes there's no frame for fences to mean anything in | Yes | Yes | Yes | **No** — `MoveHandle`; `.result` has the position | Yes |
| `lab.gantry.move_to_unfenced(axis, position)` | **No — by name**. For locating fences, or recovering an axis a (stale) fence won't let `move_to()` touch. Logged at WARNING | Yes | Yes | Yes | **No** | Yes |
| `lab.gantry.jog_unfenced(axis, speed)` | **No — by name**. Open-ended, so nothing *can* check it in advance. `speed=0` stops it | Yes (to start) | Yes (to start) | Refuses if held | Starts and returns | On `jog_unfenced(axis, 0)` |

¹ `gcode` is the engine the gated entry points drive; calling it directly
skips the controller's client-side safe_mode/halt/arbiter checks. Use
`move_to()`.

**Not motion entry points:** `lab.gantry.cmd` (`MMCCommands`) keeps every
read, setting and *stop* public, but its motion-starting primitives
(`_begin_move_to`, `_begin_move_by`, `_jog`, `_group_begin_move_to`, …) are
private. The only callers are the layers in the table above.

The per-axis handles (`lab.gantry.x`, `.y`, …) likewise have reads,
speed/ramp settings, brakes, switches and stops, but no motion methods.

## Non-blocking moves

`move_to()`, `place()`, `home*()`, `locate_limit_switch()` and
`move_to_unfenced()` do every check that can refuse the move — validation,
a live position read, the fence check, safe_mode, the halt latch, "motion in
progress" — **on your thread, before returning**. A refusal raises right
there, with nothing sent. Only then does the traverse start, on a background
thread, and you get a `MoveHandle` back:

```python
h = lab.move_to(X=1200, speed=50)   # returns as soon as the move has started
lab.pause()                          # works immediately — no Ctrl-C needed
h.wait()                             # raises MotionHalted: the pause cancelled it

lab.move_to(X=0).wait()              # scripts: .wait() to sequence
pos = lab.gantry.home_axis("X").wait().result
```

- A second motion call while one is running raises `MotionBusyError` at
  once — it doesn't queue.
- A `MoveHandle` refuses to be used as a bool (`if gantry.home():` raises),
  because `move_to()`/`home()` used to return `True`/`False` and a handle is
  always truthy.
- An error on the background thread is logged at ERROR the moment it
  happens, and re-raised by `.wait()`.
- Library code that already holds the arbiter (a survey pass, `acquire_scan`)
  gets the move run inline instead — a background thread would wait forever
  on the hold its caller is sitting in.

## Guard by guard

### Fences — `TrajectoryChecker`

Pure Python, exact (analytic, not sampled — a fence thinner than the old
0.5 mm step, or a short chord through a post, is caught). NaN/inf
coordinates raise `ValueError` instead of passing.

Two swept shapes:

- **Ribbon** (`check_ribbon`) for `move_to()`/G-code moves: X/Y follow their
  straight line (one coordinated group), but Z runs as an independent leg on
  the other PLC node, so Z may be anywhere in its start–end range at any
  point along the line. The old check tested only the two "elbow" corner
  paths, which missed the straight diagonal itself.
- **Segment** (`check_segment`) for single-axis scan passes, which really
  are straight lines.

Checked from the **live** position, read just before planning — never from a
cached one.

### `safe_mode`

Checked client-side by `GantryController` for every entry point above,
whatever the transport — so `rs232`/`ethernet`, which have no gate of their
own, are covered too. On `pi_agent`, `PiGantryConnection` and
`gantry_agent.py` each enforce the `SAFE_COMMANDS` allowlist independently
as well.

**Stop-class commands always pass the gate:** `BST`, `ABT`, `STP` (bare),
and `SOB <n> 0` (output off — on this machine, engaging a brake). They also
skip the "scan in progress" refusal and the request lock, and never trigger
a reconnect, so a halt can't be blocked or delayed by any of those.
`MTR 0` is deliberately *not* stop-class: cutting a motor with its brake
released drops Z.

`set_safe_mode(True)` stops and brakes *before* closing the gate.
`set_safe_mode(False)` turns motors on, releases brakes, and writes the
configured soft limits (NLT/PLT) — whichever way motion got enabled.

### Halt latch — `halt.py`

`pause()` / `stop()` / `estop()` latch a halt:

| Tier | Cleared by | Effect on a move in flight |
|---|---|---|
| pause | `resume()` | cancelled — its next leg is never issued |
| stop | `rearm()`, or a fresh `connect()` (new run) | cancelled |
| estop | `rearm()` only | cancelled |

While latched, every entry point above raises `MotionHalted`. A halt during
a Pi-agent scan also sends `scan_stop`; the partial profile is kept and the
pass raises, so it gets re-run rather than counted.

**`rearm()` never re-enables motion.** It clears the latch and leaves the
gantry in `safe_mode`; motion needs a separate, explicit
`set_safe_mode(False)`.

### Motion arbiter — `motion_arbiter.py`

One re-entrant lock over a whole motion *operation*. Every entry point in
the table takes it; interactive calls refuse immediately if it's held
(`MotionBusyError`), scheduled scans wait up to its timeout. Safety verbs
never take it — stopping is never queued behind a move.

### Position cache

`GCodeExecutor` plans from a cached position. Every entry point resyncs it
from hardware before planning and after finishing, however it ends — so
there's nothing to call by hand any more. (`resync_position()` remains for
anything that moved the gantry from outside this process.)

## Which call do I want?

| I want to... | Use |
|---|---|
| Move to a coordinate | `lab.move_to(...)` — add `.wait()` in a script |
| Put an instrument's measuring point somewhere | `lab.place(instrument, point)` |
| Stop *now*, resumably | `lab.pause()` — then `lab.resume()` |
| Find where a fence should go | `lab.gantry.move_to_unfenced(axis, pos)` or `jog_unfenced(axis, speed)` |
| Home | `lab.gantry.home()` / `home_axis(axis)` |
| Declare position without moving | `lab.gantry.set_position(...)` |
| Run a G-code program | `lab.gantry.gcode.plan()` then `.execute(trajectory)` — but prefer `move_to()`, which adds the gates |

## See also

- [`docs/guides/gantry-motion.md`](guides/gantry-motion.md) — the worked
  walkthrough.
- [`src/laguna/safety.py`](https://github.com/ericbarefoot/laguna/blob/develop/src/laguna/safety.py)
  — the `pause`/`resume`/`stop`/`estop` vocabulary across every subsystem.
- [`src/laguna/robot/macron/halt.py`](https://github.com/ericbarefoot/laguna/blob/develop/src/laguna/robot/macron/halt.py)
  — why the latch takes a lock, not just a counter.
