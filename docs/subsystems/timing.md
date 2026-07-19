# Timing

The shared timing backbone every other subsystem sits on top of: an
`ExperimentClock`, a `Scheduler`, an append-only `EventLog`, and a
`CheckpointStore` for crash recovery. Lives at `src/laguna/timing/`.
Unlike `weir`/`gauge`/`flow`/cameras, this isn't opt-in — `FlumeLab.__init__`
creates a clock, event log, and scheduler unconditionally, before any
hardware subsystem is registered (see [`GANTRY_GUIDE.md`](../GANTRY_GUIDE.md)
for the concrete consequence: `get_system_status()` always has a
`"timing"` key, even with zero subsystems attached).

## `ExperimentClock`

Tracks two timelines simultaneously: wall-clock time and *experiment
runtime*, which freezes during `pause()` and picks back up on `resume()`.
This is the distinction between "how long has this been running in the
real world" and "how much time has the experiment actually spent
collecting data."

```python
from laguna.timing import ExperimentClock

clock = ExperimentClock()
clock.start()                # T=0
clock.wait_until(30)         # blocks (short-polls at 10ms) until 30s of runtime elapsed
clock.pause()
...
clock.resume()
clock.elapsed()               # runtime seconds, excluding paused time
clock.now()                    # (wall_time, runtime_s) as one atomic read
```

`stop()` freezes the clock for good — `elapsed()` after `stop()` returns
the final runtime value rather than continuing to advance. Calling
`start()` again on an already-running clock raises `RuntimeError`; call
`stop()` first.

## `Scheduler`

Fires registered actions at specific experiment-runtime instants, without
blocking the caller. Two registration styles:

```python
from laguna.timing import Scheduler

scheduler = Scheduler(clock, event_log)
scheduler.repeat(every=5, action=gauge.read_mm, subsystem="gauge", name="poll")
scheduler.at(runtime_s=60, action=cameras.trigger_capture, subsystem="camera", name="shot")
scheduler.run(duration=300)     # BLOCKS the calling thread for 5 experiment-minutes
```

`run()` polls every 50ms (`Scheduler._POLL`) to check which actions are
due. **Each due action fires in its own daemon thread** — actions never
block the scheduler loop or each other, but that also means two
overlapping firings of the same recurring action are possible if the
action itself takes longer than its `every` interval; nothing here
serializes repeated firings of one action against itself.

If an action raises, the scheduler logs a warning and writes an
`error: <exception>` row to the event log — it does not crash the
scheduler loop or propagate the exception anywhere the caller can catch it
synchronously. If you need to know an action failed, watch the event log
or wrap the action's own body in a try/except that does something more
visible (this is exactly what `experiment/runner.py`'s closures do — see
[experiment runner](experiment.md)).

`run_async(duration)` runs `run()` in a background daemon thread instead
and returns the `Thread` immediately, freeing the caller (REPL, notebook,
signal handler) for interactive control. `stop()` interrupts a running
`run()`/`run_async()` call by setting an internal event and **pausing**
(not stopping) the clock, so runtime is preserved and a later `run()` call
can resume where it left off.

## `EventLog`

Thread-safe, append-only CSV writer — one row per `log()` call, flushed to
disk immediately (no buffering). Columns:

```
wall_time_iso, wall_time_unix, runtime_s, subsystem, event_type, result, notes
```

The header is written only if the file doesn't already exist, so a
resumed experiment (same path, `resume=True` on `FlumeLab.experiment()`)
appends to the same log rather than overwriting it. Every subsystem's
scheduled actions in `experiment/runner.py` write rows here, and
`FlumeLab.print_summary()` parses this same file back to build its
end-of-experiment report — the event log is both the write path and the
read path for "what happened during this run."

## `CheckpointStore`

Persists a set of completed, integer-identified events to a JSON file, so
a crashed and restarted experiment can skip work it already did:

```python
from laguna.timing import CheckpointStore

store = CheckpointStore("experiment_checkpoint.json", resume=True)
for i, t in enumerate(capture_times):
    if store.is_complete(i):
        continue
    clock.wait_until(t)
    cameras.trigger_capture()
    store.mark_complete(i, runtime_s=clock.elapsed(), wall_time=clock.wall_time())
```

Writes are atomic (`.tmp` file then `os.replace()`), so a crash mid-write
can't corrupt the checkpoint. With `resume=False` (the default), any
pre-existing checkpoint file at that path is deleted on construction —
`CheckpointStore` assumes a fresh start unless told otherwise. Event IDs
are caller-assigned arbitrary integers (typically a sequence index); there's
no built-in notion of "phase" beyond the optional freeform `name` field.

`FlumeLab.experiment()` creates a `CheckpointStore` automatically from
`timing.checkpoint_file` in config (default `./experiment_checkpoint.json`),
but — as of this writing — doesn't itself use it for anything beyond
instantiation; using it to skip completed work (as in the snippet above) is
left to the calling code, which is exactly the pattern
`examples/example_04_scheduled_experiment.py` demonstrates.

## Config

```yaml
timing:
  checkpoint_file: ./experiment_checkpoint.json
  event_log: ./experiment_events.csv
```

Both default the same way in `Config._get_defaults()` if the `timing:`
section (or individual keys) are omitted.

## Further reading

- [API reference](../reference/timing.md) — generated from docstrings.
- [Experiment runner](experiment.md) — how the scheduler and event log get driven from YAML.
- [`GANTRY_GUIDE.md`](../GANTRY_GUIDE.md) — worked example of `get_system_status()` reading the clock unconditionally.
- `src/laguna/timing/` — the four modules themselves (`clock.py`, `scheduler.py`, `event_log.py`, `checkpoint.py`).
