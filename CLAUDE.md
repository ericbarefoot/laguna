# Laguna — Guidance for Claude

Control software for robotic systems in hydraulic flume experiments. `FlumeLab`
coordinates opt-in hardware subsystems (gantry, weir, gauge, flow, cameras,
Gocator scanner) under a shared clock, scheduler, and event log. Real
apparatus, real motors, and irreplaceable experimental data are on the other
end of this code — the priorities below follow from that, in order.

## Priority hierarchy

When a change touches more than one of these, the higher one wins.

1. **Safety.** All motion must be quickly stoppable. Motion must never be
   commandable from a stop condition. Motion is always explicitly
   commanded via a script or REPL call, so a human triggers "go" and can
   verify safety immediately beforehand — never auto-triggered by a
   schedule, health check, or side effect.
2. **Data preservation.** The point of the apparatus is the data it
   collects, and that data is perishable — a missed scan or dropped frame
   can't be redone by rerunning the script. Bugs or gaps that cause missed
   data collection are stop-and-fix events, not follow-up tickets, and any
   recovery mechanism should include a way to re-acquire what was missed
   (see `CheckpointStore` in `src/laguna/timing/checkpoint.py` and the
   runtime↔wall timeline in `src/laguna/timing/clock.py` for the existing
   patterns).
3. **Graceful shutdown and easy recovery.** When something does go wrong
   (power loss, camera malfunction, sensor dropout), the system should
   quiesce safely, recovery should be simple, and logging should describe
   *what led to* any pause/stop clearly enough that a resume — or a human
   reading the event log later — can pick up correctly. See `safety.py`'s
   docstring for the pause/stop/estop vocabulary and why verbs return notes
   about discarded data.

## Motion safe mode — always on by default

Default to **`safe_mode=True`** for any gantry/motion code path. Never
write code, examples, or scripts that command real motion without the user
explicitly and separately enabling it (see the `ALLOW_MOTION` /
`safe_mode` pattern in `examples/example_07_flumelab_gantry_scan.py`).

If you are ever unsure whether a change could cause unintended or
unreviewed motion — stop and ask rather than guessing. This includes:
generating example/demo scripts, writing test fixtures that touch
`GantryController` directly instead of a mock, or refactors that touch how
`safe_mode` is read or gated.

**Always ask before proceeding, don't just default to safe mode and go,**
when a change touches any of:
- `src/laguna/safety.py`, `src/laguna/robot/macron/fences.py`, or
  `motion_arbiter.py` — the safety-verb vocabulary and exclusion-zone
  enforcement.
- Anything that changes default motion limits, fence geometry, or config
  defaults affecting them (`config.py` `_get_defaults()`,
  `config/example_config.yaml`).
- Deleting or overwriting recorded experiment data — `data/`,
  `experiment_events.csv`, checkpoint files, or any run manifest.

## API stability

Pre-1.0.0, favor a consistent API across the whole project over preserving
existing call signatures. Breaking changes are fine — and have already
happened deliberately (e.g. `GantryController.stop()` changed meaning
in `safety.py`'s pause/resume/stop/estop unification; see that module's
docstring for the reasoning). Don't add compatibility shims, deprecated
aliases, or `# removed` comments to soften a breaking change — just make
the change and update call sites.

## Module organization

Substantive code belongs in descriptively named modules, not
`__init__.py` — `__init__.py` re-exports and wires things together, it
doesn't implement them. New subsystems should follow the shape documented
in [`docs/CONTRIBUTING.md`](docs/CONTRIBUTING.md#adding-a-new-subsystem)
(config dict / `from_config()`, `connect()`/`disconnect()`,
`get_status()`, `stop()`, a `subsystem_name` attribute) rather than a new
pattern — deviations make `FlumeLab.add()` and the runner's config wiring
harder to reason about uniformly.

## Comments and docstrings

Docstrings are required (Google convention — enforced by ruff's `D` rules,
see `pyproject.toml`). Beyond that:
- Don't write comments that restate what the code does — names and
  docstrings should already make that clear.
- Do write a comment when there's a non-obvious *why*: a hardware
  quirk, a workaround for a specific controller/SDK bug, an invariant a
  future reader could easily violate. `safety.py` and `simulation.py` are
  good examples of this in practice — read their module docstrings before
  writing similar code.

## Testing

- New hardware-facing code ships with tests against a mocked connection,
  not real hardware — follow the existing per-subsystem pattern (e.g.
  `tests/macron_fixtures.py` for the gantry, the scripted serial transport
  and fake GoSdk described in `simulation.py` for full-experiment
  rehearsal).
- Safety-critical paths — `pause()`/`stop()`/`estop()` on every subsystem,
  fence checking (`TrajectoryChecker`/`CheckedTrajectory`), and `safe_mode`
  gating — need explicit test coverage before merging, not just coverage
  as a byproduct of testing the happy path.
- Run `pytest` (or a targeted file) before considering motion-adjacent or
  safety-adjacent work done; `pytest --cov=src/laguna` for coverage.

## Simulation / rehearsal mode

`FlumeLab(..., simulate=True)` rehearses a whole experiment with no
hardware attached — the scheduler, clock, event log, and safety verbs are
all real, only the wire is faked. Prefer extending this rehearsal path
over hand-rolling one-off mocks when a change needs end-to-end exercise;
see `src/laguna/simulation.py`'s module docstring for what is and isn't
simulated, and note its limits (it cannot catch a mounting sign error, an
infeasible feed rate, or a target outside the work envelope) so it isn't
mistaken for a substitute for a real hardware check before a run.
