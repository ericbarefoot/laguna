# Roadmap: co-scripting the survey instruments with the rest of FlumeLab

> Status: **all five workstreams implemented**, each as its own PR in a
> stack off `develop`. Written 2026-08-03 alongside the Gocator integration
> work. None has been validated against hardware yet.

## Why

laguna has **two disjoint halves**.

The *environmental* half — gauge, weir, flow, cameras — is fully config-driven.
`experiment/runner.py::setup_run()` reads `interval_s`/`trigger_at`/`use_schedule`,
registers scheduled actions, writes to the event log, and runs unattended via
`run_blocking()` with signal-based pause/resume.

The *survey* half — gantry, OD2000, WTT12L, Gocator — participates in **none** of
it. It is hand-scripted in `examples/` and `scripts/run_line_scan.py`:

- `setup_run()` has a hardcoded five-section tuple (`experiment/runner.py:149`);
  `gocator` and `gantry` are absent. `docs/GANTRY_GUIDE.md:323` documents this as a
  known gap for the gantry; nothing documented it for the scanner.
- The Gocator has **no zero-arg "acquire now"** — `scan_with_gantry()` requires four
  arguments including a gantry handle, so it cannot be handed to
  `Scheduler.repeat(action=...)`.
- Scans and transects **never touch the event log**, so the only cross-subsystem
  index does not know they happened. There is no run ID, no run directory, and the
  runtime↔wall-clock mapping dies with the process.

**Goal:** a Gocator scan should be schedulable from a config file exactly like a
camera capture, its output correlatable with everything else in the run, and the
whole rig should have one coherent, externally-triggerable safety vocabulary.

## Two latent defects to fix along the way

1. **`Scheduler._fire()` (`timing/scheduler.py:148`) dispatches every action in its
   own daemon thread with no coordination.** Latent today only because nothing
   schedulable moves the gantry — the moment scans become schedulable, two threads
   command motion concurrently.

2. **`FlumeLab.emergency_stop()` (`core.py:520-529`) calls `stop()` unguarded in a
   loop.** `GocatorScanner.stop()` begins with `_require_connected()`
   (`scanner/gocator.py:924`), which **raises when disconnected** — aborting the loop
   so every subsystem registered after it never gets stopped. `weir.stop()` and
   `flow.stop()` have the same raising shape.

---

## Workstream 0 — Safety vocabulary (foundational, do first)

Today `stop()` means four different severities:

| Subsystem | `stop()` does | Severity |
|---|---|---|
| `GantryController` (`controller.py:367`) | zero-decel abort, brakes engaged, motors disabled — docstring calls it the emergency-stop path | **hard e-stop** |
| `SaflWeirController` (`weir/controller.py:360`) | halt in-progress move | moderate |
| `SaflFlowController` (`flow/controller.py:309`) | **stops the pump** | drains the experiment's hydraulic state |
| `GocatorScanner` (`gocator.py:1262`) | stops the SDK data channel | benign |
| gauge, rangefinders, pi cameras | *no `stop()` at all* | — |

And `FlumeLab.stop()` (`core.py:320`) is really a *pause* — it logs
`experiment_pause`, halts the scheduler, and hardcodes `weir.stop()`.

### Settled semantics

| Decision | Choice |
|---|---|
| Vocabulary | Three tiers: `pause()` / `stop()` / `estop()`, **identical on every subsystem** |
| API stability | Deliberately sacrificed — `GantryController.stop()` changed from hard abort to clean stop |
| Discarded data | Written to the **event log**, not just the Python log |
| Trigger tiers | All three pollable, not just estop — so a health check can `touch PAUSE` |
| Pause & hydraulics | Quiesce **everything**, pump included |
| Estop & hydraulics | Kill everything — pump off, both valves closed |
| Mid-scan pause/estop | Abort immediately, **discard** the partial surface, log it |
| Pause & the clock | The experiment clock pauses too |
| Re-arm after estop | `lab.rearm()` normally, full reconnect as fallback |

- **`pause()`** — temporary and resumable. Gantry `soft_stop()` (BST decel ramp,
  brakes and motors untouched), pump ramped down via normal VFD stop, acquisition
  stopped, everything stays *connected*. `resume()` restores the flow setpoint.
  **The experiment clock pauses too**, so runtime means "time under experimental
  conditions" and a schedule-CSV row at t=600 fires 600s of real experiment time
  in however long the pause lasted. `ExperimentClock` already supports this; the
  consequence is that runtime and wall clock diverge, which is exactly why
  Workstream 2's `run.json` has to record the piecewise mapping.
- **`stop()`** — end the run cleanly. Quiesce as for pause, then disconnect.
- **`estop()`** — screeching halt. Gantry hard abort (ABT + brakes + motors off),
  pump off, **both valves closed**, acquisition stopped. Requires explicit re-arm.

Rationale for discarding a partial surface: `Y spacing = travel_speed / frame_rate`
assumes constant velocity, so a surface captured across a decelerating pass has a
distorted travel axis. It would be quietly wrong data rather than useful data.

**`GantryController.stop()` changed meaning**, as the API-stability row above says.
It used to be the zero-decel abort with motors disabled; that behaviour moved to
`estop()` (`controller.py:711`), which now issues its own hard `shutdown()` rather
than delegating to `stop()`. `stop()` (`controller.py:367`) is the tier below —
`soft_stop()` (ramped deceleration, brakes/motors untouched) followed by parking
the Y/Z brakes, safe to disconnect from. `pause()` (`controller.py:693`) calls
`soft_stop()` directly, without the brake-park. Existing scripts calling
`gantry.stop()` expecting the old hard-abort behaviour will need updating —
that is the "deliberately sacrificed" API stability tradeoff.

### Work

**New:** `src/laguna/safety.py` — a `SafetyState` enum
(`RUNNING`/`PAUSED`/`STOPPED`/`ESTOPPED`) and a `Quiescible` protocol documenting the
three verbs.

**`src/laguna/core.py`:**
- Add `FlumeLab.pause()` / `resume()` / `stop()` / `estop()`. Today's `stop()`
  (`core.py:320`) becomes `pause()`, with a thin deprecated alias so existing scripts
  keep working.
- Every subsystem loop must be **individually guarded** (`try/except` per subsystem,
  log and continue) — this is defect 2 above, and it applies to all four verbs.
- Order on `estop`: motion first (gantry), then hydraulics, then acquisition.
- Log every transition via the existing `EventLog.log()` (`timing/event_log.py:42`).

**Per subsystem** — add `pause()`/`resume()`/`estop()`, delegating to what exists:

| Subsystem | `pause()` | `estop()` |
|---|---|---|
| `GantryController` | `soft_stop()` | `stop()` |
| `SaflFlowController` | remember setpoint, `stop()` pump | pump off + both valves closed |
| `SaflWeirController` | `stop()` (halt move) | `stop()` |
| `GocatorScanner` | abort + discard in-flight scan | same, plus **never raise** |
| cameras / gauge / rangefinders | cease scheduled activity | same |

`GocatorScanner` needs a safety path that does **not** call `_require_connected()`.

### Estop triggers

`src/laguna/safety.py`, designed as a **pluggable list of trigger sources**:

1. **`SentinelFileTrigger`** — background thread polling for e.g. `./ESTOP` (~100 ms).
   Works regardless of which thread is blocked.
2. **Physical button** — wired to write the ESTOP sentinel file. *This needs no new
   software infrastructure*: it reuses trigger 1 entirely, so the button is a wiring
   task, not a code task.
3. **`DigitalInputTrigger`** (optional, later) — poll a spare gantry PLC digital input
   (see `IOMap` in `robot/macron/commands.py`) or an AL1342 IO-Link port, for a path
   that does not depend on a filesystem write succeeding.
4. Also poll the VFD's existing **hardware** `e_stop` flag (`flow/controller.py:416`,
   read-only) and propagate it — the pump drive already has a real e-stop circuit.

**A signal-based trigger is deliberately not the primary path.** Python delivers
signals only in the main thread between bytecodes, so a signal cannot interrupt a
blocking serial read or SDK call — the halt would be deferred, not immediate.

---

## Workstream 1 — Schedulable surveys + motion arbiter

> **Implemented.** `src/laguna/robot/motion_arbiter.py`,
> `GocatorScanner.acquire()`, and `gantry`/`gocator` sections in
> `experiment/runner.py::setup_run()`.

**Motion arbiter** — new `src/laguna/robot/motion_arbiter.py`. A re-entrant lock with
a timeout and a descriptive error naming the current holder. Acquired by every
gantry-consuming operation: `GantryController.move_to`, `FlumeLab.place`,
`FlumeLab.acquire_scan`, `GocatorScanner.scan_with_gantry`. **Prerequisite for
everything else here** — without it, two scheduled actions command motion at once.

**Zero-arg acquire verbs.** Give the Gocator a scheduler-compatible entry point:
`GocatorScanner.acquire(gantry=None, **defaults)`, closing over a configured scan
spec. Mirrors `RangefinderSubsystem.read_mm()` (`rangefinder/subsystem.py:167`) and
`CameraManager.trigger_capture()` (`camera/manager.py:127`).

**`experiment/runner.py`:**
- Extend the section tuple (`runner.py:149`) and instantiation chain
  (`runner.py:172-205`) to cover `gantry`, `gocator`.
- Add action closures beside the existing `_capture_pi`/`_log_gauge` ones
  (`runner.py:229-346`), each writing an event-log row.
- **Reuse `_register_action()` as-is** (`runner.py:47`) — already subsystem-agnostic,
  handling `interval_s`/`trigger_at`/`use_schedule` plus the schedule-CSV column
  filter. This is the key existing utility; do not reimplement it.

**Config:** scheduling keys plus a scan spec in the `gocator:` section, and a
`gocator` boolean column in the schedule CSV (same idiom as `pi_cameras`).

**`od2000`/`wtt12l` scheduling is deliberately out of scope here.** The stated goal
was a Gocator scan being schedulable exactly like a camera capture; the rangefinder
line-scan path (`FlumeLab.acquire_scan()`) is a separate, synchronous entry point,
not driven through `setup_run()`'s section/schedule machinery. Adding `od2000`/
`wtt12l` sections would need their own scan-spec shape (a rangefinder pass has no
Gocator-style `uniform_spacing`/filters to configure) and is a candidate for a
follow-up workstream, not a silent gap in this one.

---

## Workstream 2 — Run context & correlation

> **Implemented.** `src/laguna/run_context.py`, clock pause/resume observers,
> and run stamping in scan metadata.

**New:** `src/laguna/run_context.py`.

- A **run ID** (UTC timestamp + short random suffix) minted by
  `FlumeLab.experiment()`/`start()`.
- A **run directory** — all outputs beneath it, so a run is one self-contained
  artifact. Keep per-subsystem `output_dir` overrides working.
- **Persist the runtime↔wall mapping** to a `run.json` manifest, including pause
  intervals. `ExperimentClock` holds `_start_wall`/`_pause_offset` privately and they
  die with the process, so after a pause the mapping is piecewise and cannot be
  reconstructed from a single start time.
- Stamp `run_id` + `runtime_s` into `SurfaceScan.metadata` and the profiler's
  `_meta.json`; write an event-log row for every scan/transect, with the output path.
- Fix second-resolution filename collisions in `save_scan()` (`gocator.py:1626`) and
  `profiler.py:115` — two outputs in the same second overwrite silently.

**Done, narrowly:** `laguna.data.DataProcessor.save_data()` (`data/processor.py:89`)
used to return `True` while writing nothing. It now raises `NotImplementedError`
instead — reporting success for a silent no-op was an active hazard. Actually
implementing data streaming/packaging (HDF5/CSV export, compression) is still out
of scope here and remains a future project.

---

## Workstream 3 — Survey / raster planner

> **Implemented.** `src/laguna/survey.py`.

**New:** `src/laguna/survey.py`. Nothing multi-pass exists anywhere in the repo today.

- `RasterSurvey` — tile a region with multiple Gocator passes, accounting for FOV
  width and a configurable overlap; emits a pass list.
- `RepeatTransect` — re-run a line on a cadence, optionally with a different
  instrument.
- Expressed in **experiment-frame coordinates**, building on `src/laguna/frames.py`:
  `FrameRegistry.gantry_target_for()`, `retarget()`, `place_scan()`.
- Use `GocatorScanner.solve_scan_rates()` to pick feed rate per pass, and
  `CheckpointStore` (`timing/checkpoint.py`) so a long survey resumes after
  interruption.
- Stitching multiple placed surfaces is the natural follow-on; keep it out of scope
  unless it falls out cheaply.

**Landed in this PR but two things are explicitly still open, not silently
dropped:**
- `solve_scan_rates()` is never called from `survey.py` — `Pass.feed_rate_mm_s`
  is either the survey's fixed rate or `None` (falling back to the scanner's own
  configured rate for `acquire()`). Picking a rate automatically per pass is a
  real feature, not a one-line wiring job — needs its own follow-up.
- `CheckpointStore.mark_complete(p.index, ...)` has no geometry fingerprint. If a
  survey's YAML changes between runs (a different `origin`/`width_mm`/`swath_mm`),
  a stale checkpoint would resume against pass indices that now mean something
  else, with nothing to detect the mismatch. Needs a hash of the survey's
  geometry fields stored alongside each completed index and checked on resume.

---

## Workstream 4 — Offline rehearsal

> **Implemented.** `src/laguna/simulation.py` and
> `src/laguna/scanner/simulation.py`, behind `FlumeLab(simulate=True,
> speed_factor=...)` and `setup_run(simulate=True, speed_factor=...)`. Only
> gantry/gocator have a simulated backend — see `simulate_config()`'s
> docstring for what that does and does not cover.

A `simulate=True` flag on `FlumeLab`/`setup_run()` swapping in fake transports, so a
whole experiment script — schedule, survey plan, timing — can be validated with no
hardware.

**What actually landed differs from the original plan in two ways, both
narrower than described below:**
- `SimulatedSnapConnection` is a fresh, purpose-built model of the OEM-2T
  protocol (including group→member axis distribution for `C1 BMT`), not a
  promotion of the test suite's `FakeSnapConnection`
  (`tests/macron_fixtures.py`) — that fixture is a scripted lookup table,
  fine for pinning specific command/response pairs in a unit test but not
  for driving an actual rehearsal.
- `GCodeExecutor(dry_run=True)` (`robot/macron/gcode.py:529`) was never
  surfaced through `GantryController`, config, or `FlumeLab` — the
  simulated-transport approach above replaced it rather than building on
  it, so `dry_run` remains dead code today.

Highest value per line of code for a rig where a bad schedule costs a flume day.

---

## Sequencing

1. Workstream 0 — safety vocabulary. Foundational; also fixes both latent defects.
2. Workstream 1 — motion arbiter, then schedulable surveys.
3. Workstream 2 — run context.
4. Workstreams 3 and 4 — independent of each other; either order.

## Verification

- **Unit:** `python -m pytest -q`. New coverage for the three verbs per subsystem
  (including that one failing subsystem does not abort the loop), arbiter contention,
  `_register_action` wiring for the new sections, run-ID propagation into metadata,
  and raster tiling geometry. In the `flumelab` conda env use `-o addopts=""` — that
  env lacks `pytest-cov`.
- **Offline:** once Workstream 4 lands, run a full multi-subsystem experiment with
  `simulate=True` and assert the event log shows the expected interleaving.
- **Hardware, staged, read-only first:**
  1. Estop triggers with the gantry in `safe_mode: true` — confirm the sentinel file
     fires and the state machine reaches `ESTOPPED`.
  2. `pause()`/`resume()` with hydraulics only, no motion — confirm the flow setpoint
     is restored.
  3. A scheduled Gocator scan in a short run — confirm the event log shows it and the
     output lands in the run directory with a matching `run_id`.
  4. `pause()` mid-scan — confirm prompt abort, discarded surface, logged row.
- **Correlation check:** after a run, load `run.json` + `experiment_events.csv` and
  confirm a scan's wall-clock timestamp maps to the right experiment runtime *across
  a pause*.

## Re-arming after an estop

Two paths, both supported:

- **`lab.rearm()`** for the normal case — re-enables motors, releases brakes, and
  returns state to `RUNNING` without dropping any connection, so recovery is fast.
  It **refuses while a trigger is still asserted** (the ESTOP sentinel file still
  present, the VFD's hardware `e_stop` flag still set), so the rig cannot be re-armed
  back into a live emergency.
- **A full disconnect/reconnect** as the documented fallback, for when the controller
  is in a state `rearm()` cannot clear — a PLC that needed a power cycle, say.
  `connect()` already performs motor-on then brake-release in the correct order.
  Note this loses the gantry's position reference unless the #23 checkpoint is
  restored with `restore_last_position()`.

## When to start

**Blocked on PR #28 merging to `develop`.** These workstreams branch from a clean
`develop` rather than from the scanner branch: W0 and W2 both rewrite `core.py`,
W1 and W4 both touch `experiment/runner.py`, and W0 and W1 both touch
`scanner/gocator.py`, so branching them off an unmerged parent would put #28's
commits in every diff and guarantee conflicts between them.

Once #28 lands, each workstream gets its own branch off `develop` and its own draft
PR, in the sequencing order above.
