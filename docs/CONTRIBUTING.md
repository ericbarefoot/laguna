# Contributing

Thanks for your interest in contributing to Laguna.

## Development Setup

```bash
git clone <repo-url>
cd laguna
mamba env create -f environment.yml   # or: conda env create -f environment.yml
mamba activate flumelab               # or: conda activate flumelab
```

`environment.yml` installs the package editable with every extra
(`dev`, `scanner`, `docs`, `storage`) via pip inside the conda env — the
package's actual dependency versions stay pinned in `pyproject.toml`, not
duplicated here, so there's one source of truth. See `make docs-serve` /
`make docs-build` and [the index page](index.md#viewing-these-docs-as-a-site)
for working on this documentation site.

## Running Tests

```bash
pytest                        # all tests
pytest --cov=src/laguna       # with coverage report
pytest tests/test_macron_controller.py   # a specific file
```

## Code Style

This project uses Black for formatting, isort for import sorting, and mypy
for type checking:

```bash
black src/ tests/ examples/
isort src/ tests/ examples/
mypy src/laguna
```

## Making Changes

1. Create a feature branch: `git checkout -b feature/your-feature-name`
2. Make your changes and add tests
3. Run tests and check code style
4. Commit with a clear message
5. Push and open a pull request

## Releasing / Version Bumps

`pyproject.toml`'s `version` field is the single source of truth — it isn't
duplicated as a separate hardcoded string anywhere. `laguna.__version__` reads
it via `importlib.metadata`, which reflects whatever was true when the package
was last installed (`pip install -e .`), not a live read of the file — after
bumping, reinstall if your current environment needs `__version__` to reflect
it immediately. As you make notable changes, add a bullet under the
`## [Unreleased]` heading in `CHANGELOG.md`. When it's time to cut a version:

```bash
python scripts/bump_version.py patch   # or: minor, major, or an explicit X.Y.Z
```

This bumps `pyproject.toml`, rotates `CHANGELOG.md`'s `[Unreleased]` section
into a dated release entry, and creates a local commit + annotated `vX.Y.Z`
tag. It refuses to run against a dirty working tree, and never pushes —
pushing the branch and tag is a separate, explicit step.

## Adding a New Subsystem

This is the mechanical checklist. For the design decisions it doesn't
cover — static vs. gantry-mounted, sensor vs. actuator, and two full
worked examples — see [Adding a New Subsystem
(guide)](ADDING_A_SUBSYSTEM.md).

`FlumeLab` is opt-in: subsystems are constructed independently and
attached with `lab.add(subsystem)` — or built for you from config via
`lab.add("name")` / `lab.add_all()`, see below — rather than hardcoded
into `core.py`. Every existing subsystem (`weir`, `gauge`, `flow`,
`camera`, `robot/macron`) follows the same shape, so a new one should too:

1. Create `src/laguna/<subsystem_name>/` with a main class that:
   - takes a config `dict` in `__init__` (its own section's shape only —
     see `Config._get_defaults()`'s existing sections for the convention)
   - exposes `from_config(cls, config: Config) -> Self`, a classmethod
     taking the lab's whole `Config` object rather than just its own
     section. For most subsystems this is just
     `return cls(config.get("<subsystem_name>"))`; subsystems that need to
     build their own dependencies pull whatever else they need the same
     way — e.g. `RangefinderSubsystem.from_config()`
     (`src/laguna/rangefinder/subsystem.py`) also reads the shared
     `mqtt:` section to build its own `MqttSubscriber`, and
     `DslrCameraSubsystem.from_config()` (`src/laguna/camera/dslr.py`)
     reads `config.config_file` to resolve paths relative to the
     experiment YAML.
   - implements `connect()` / `disconnect()`
   - implements `get_status()` — used by `lab.get_system_status()` and
     `lab.print_summary()`
   - implements `stop()` if it's an actuator — used by `lab.stop()` and
     `lab.emergency_stop()`
   - sets a `subsystem_name` class or instance attribute — this is the key
     `lab.add()` registers it under, and the attribute name it becomes
     accessible as (`lab.<subsystem_name>`)
2. Add its defaults to `Config._get_defaults()` in `src/laguna/config.py`
   if it should have built-in defaults, and document its section in
   `config/example_config.yaml`.
3. Add an entry to `SUBSYSTEM_REGISTRY` in `src/laguna/registry.py`
   (config-section name -> class) so `lab.add("<subsystem_name>")` and
   `lab.add_all()` can find it. `add_all()` builds every subsystem whose
   section was explicitly present in the loaded YAML (tracked in
   `Config.explicit_sections`) — this is what
   `laguna.experiment.runner.setup_run()` calls, so a registry entry is
   usually all a new subsystem needs to become wireable from a single
   experiment config file. If it needs scheduling (`interval_s` /
   `trigger_at` / `use_schedule`), add that to `setup_run()` following the
   pattern of the existing closures (`_log_gauge`, `_make_update_weir`,
   etc.) and the shared `schedule_action()` helper.
4. Logging follows a two-tier model — see `laguna.subsystem_logging`'s
   module docstring for the full reasoning:
   - **Archival event log** (`lab.event_log`, ships alongside published
     data as metadata): state-changing actions and milestones only —
     "weir moved to 300mm", "scan completed", "inflow on". Inherit
     `SubsystemLogging`, set `self.log_level` / `self.event_log_verbosity`
     from config in `__init__` (both default `"INFO"`), and call
     `self.log_event("action_name", **fields)` at the points that command
     real state changes. `lab.add()` wires `attach_event_log()`
     automatically for any subsystem exposing it — no extra step needed
     beyond inheriting the mixin.
   - **Operational log** (plain `logging.getLogger(__name__)`, INFO by
     default): connections firing, broad commands — one line per call, not
     one per low-level step underneath (see `GantryController.move_to()`
     for the pattern: one "started"/"completed" pair regardless of how
     many G-code segments a move tessellates into). Passive reads/status
     polls (a sensor reading, a `get_status()` snapshot) belong here, not
     the archival log — they measure the experiment's state without
     changing it. `laguna.experiment.runner.setup_run()`'s scheduled
     status-polling closures (`_log_gauge`, `_log_weir_status`) respect an
     opt-in `log_as_event: true` config key for the rare case an
     infrequent poll IS a milestone worth archiving — follow that pattern
     rather than defaulting a new polling action to archival.
   - Low-level detail underneath a broad operational-log line (individual
     G-code segments, etc.) uses `logger.debug(...)`, surfaced only via
     `FlumeLab(debug=True)` or a subsystem's own `log_level: DEBUG` config
     — never gated behind `dry_run` the way it used to be, since that
     meant real runs logged nothing at all at that granularity.
5. Add tests in `tests/test_<subsystem_name>.py` (mock the hardware
   connection; see `tests/macron_fixtures.py` for an example of a shared
   fixture module for a subsystem with several test files).
6. Document it: add a page under `docs/subsystems/` (and list it in
   `mkdocs.yml`'s `nav:`), and add a `::: laguna.<subsystem_name>` block to
   the relevant page under `docs/reference/` so its public API is picked
   up by mkdocstrings automatically.
7. If practical, give it a rehearsal path: a `simulated: true` config key
   your `connect()` (and any other hardware-touching method) checks to
   build/use an in-memory stand-in instead of the real driver — see
   `laguna.simulation`'s `SimulatedTeknicMotor`/`SimulatedVFD`/
   `SimulatedMassaSensor` for the pattern, or `CameraArray`/
   `DslrCameraSubsystem` for a subsystem with no separate driver object to
   swap. Commands should succeed and log normally (that's what actually
   proves a schedule fires correctly); readings should come back `NaN` (or
   `None` for non-numeric fields) rather than a fabricated value — a
   rehearsal checks that the script and plan are well-formed, not physical
   feasibility. Wire it into `simulate_config()` in `laguna/simulation.py`
   (add the section to `_SIMULATED_SECTIONS`, inject `simulated: True`)
   once it exists; until then it belongs in `_NO_SIMULATED_BACKEND` so
   `simulate=True` drops it rather than silently touching real hardware.

## Logging Reference

Quick summary of `## Adding a New Subsystem`'s item 4, for when you're not
adding a subsystem but just want to know where something gets logged or
how to turn up verbosity:

- `lab.event_log` (CSV, `laguna.timing.EventLog`) — the archival record.
  Terse by design; every row is `event_id, wall_time_iso, wall_time_unix,
  runtime_s, subsystem, event_type, result, notes, refers_to`. Add a
  free-text entry yourself with `lab.log_note("what happened", refers_to=
  <event_id>)` — useful when a run has to be paused for a reason no
  automated log line captures, or to annotate a failure after the fact.
  `refers_to` is optional; every `log_event()`/`log_note()` call returns
  its own `event_id` so a later note can point back at it.
- Operational log — standard Python logging under the `laguna` logger
  hierarchy, INFO by default. Persisted to `<run_dir>/<run_id>/laguna.log`
  automatically when `timing.run_dir` is configured; terminal-only
  otherwise.
- `FlumeLab(debug=True)` sets every `laguna.*` logger to DEBUG at once
  (third-party libraries like paramiko stay pinned to WARNING regardless —
  "debug my code", not "debug every dependency") — turn this on rather
  than editing `log_level` under each config section individually when
  troubleshooting.

## Reporting Issues

When reporting bugs, please include:
- Python version and operating system
- Minimal code to reproduce the issue
- Full error messages and stack traces
