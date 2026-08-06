# Changelog

All notable changes to this project are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

Add entries under `[Unreleased]` as you work; `scripts/bump_version.py` moves
them into a dated release section when you cut a version.

## [Unreleased]

### Added
- **`simulate=True` now rehearses every subsystem in
  `laguna.registry.SUBSYSTEM_REGISTRY`** — weir, flow, gauge, both camera
  subsystems, and both AL1342 rangefinders (`od2000`/`wtt12l`), not just
  gantry/gocator. Each builds a simulated driver
  (`SimulatedTeknicMotor`, `SimulatedVFD`, `SimulatedMassaSensor` in
  `laguna.simulation`; `pi_cameras`/`dslr_cameras`/`od2000`/`wtt12l` check a
  `simulated` flag directly, skipping the real MQTT broker/AL1342 HTTP
  path) — commands succeed and log exactly as they would against real
  hardware, so a rehearsal actually proves a schedule's weir moves, flow
  changes, camera triggers, and rangefinder activate/read calls all fire in
  the right order. Readings come back `NaN` (or `None` for non-numeric
  status fields) rather than a fabricated physically-plausible value — a
  rehearsal checks that the script and plan are well-formed and execute as
  scheduled, not physical feasibility (fence checking still is, and still
  runs for real). `_NO_SIMULATED_BACKEND` is empty today; kept as the
  fail-closed guard for whatever gets added to the registry next without a
  simulated path yet. A simulated Gocator scan is a small fixed-size
  synthetic surface (~80,000 cells, well under a megabyte) regardless of
  what the real scan config asks for, so a rehearsal with frequent scans
  does not accumulate large files.
- A rehearsal's event log can no longer land in the same file as a real
  run's: `simulate=True` suffixes the event-log filename with `_simulated`
  (even if `timing.event_log` was set explicitly, since the same config is
  often reused for both), and writes an explicit `flume_lab`/`simulate_mode`
  row at construction as a second, row-level safeguard.
- `lab.add("name")` / `lab.add_all()` — build and register subsystems
  straight from config via `laguna.registry.SUBSYSTEM_REGISTRY`, instead of
  constructing and `lab.add()`-ing each one by hand.
- `Config.explicit_sections` / `Config.config_file` — track which config
  sections were literally present in the loaded YAML (as opposed to
  `_get_defaults()`'s unconditional defaults) and where the file lives.
- **Two-tier logging.** `lab.event_log` (CSV) stays the terse archival
  record that ships alongside published data — state-changing actions and
  milestones only. Everything else (connections firing, broad motion
  commands, low-level detail) goes to a new operational log tier: standard
  Python logging, INFO by default, persisted to
  `<run_dir>/<run_id>/laguna.log` when a run directory is configured.
  `laguna.subsystem_logging.SubsystemLogging` — opt-in mixin giving a
  subsystem `self.log_event(...)` for the archival tier, auto-wired to the
  run's event log/clock by `lab.add()`. Adopted by weir, flow, pi_cameras,
  and dslr_cameras for their state-changing actions (`go_to_elevation`,
  `set_flowrate`/`start`/`stop`/`qin`/`qaux`, camera captures) — passive
  reads (gauge's `read_mm`, status polls) deliberately stay out of the
  archival log by default; see `log_as_event` below.
- `GantryController.move_to()`/`home()` now log to the operational log —
  one "started"/"completed" line per call regardless of how many G-code
  segments a move tessellates into (an arc can expand into dozens; those
  now log at DEBUG via `gcode.py`, no longer gated behind `dry_run`, which
  meant real runs logged nothing at that granularity at all before this).
- `EventLog` gained `event_id` (monotonic, continues correctly across a
  resumed run) and `refers_to` columns — **breaking CSV schema change**.
  `log()` now returns the new row's `event_id`.
- `FlumeLab.log_note(text, refers_to=None)` — add a free-text entry to the
  archival event log yourself, optionally pointing at a specific prior
  `event_id` (e.g. explaining why a run was paused, or annotating a
  failure after the fact).
- `FlumeLab(debug=True)` — sets every `laguna.*` logger to DEBUG at once
  (third-party libraries like paramiko stay pinned to WARNING), instead of
  editing `log_level` under each subsystem's config section individually.
- `log_as_event: true` — opt-in config key for `laguna.experiment.runner`'s
  scheduled status-polling closures (`_log_gauge`, `_log_weir_status`),
  for the rare case an infrequent poll (e.g. hourly) IS a milestone worth
  archiving. Defaults to `false`.

### Changed
- **Breaking:** Minimum supported Python bumped from 3.9 to **3.14**
  (`requires-python`, `ruff`/`black`/`mypy` target versions all updated to
  match). The repo's `.venv` is currently on 3.13.13 and will need
  recreating against a 3.14 interpreter; the `flumelab` conda env is
  already on 3.14.6.
- **Breaking:** `GantryController.from_config()` and
  `GocatorScanner.from_config()` now take the lab's whole `Config` object
  instead of just their own section dict, matching every other
  subsystem's new `from_config(config: Config)` contract — lets a
  subsystem pull sibling sections (e.g. rangefinders reading the shared
  `mqtt:` section to build their own `MqttSubscriber`) or `config_file`
  (DSLR cameras, resolving paths relative to the experiment YAML)
  uniformly. Update any direct `from_config(some_dict)` call site to pass
  the `Config` instead.
- `laguna.experiment.runner.setup_run()` now builds subsystems via
  `lab.add_all()` instead of re-parsing the YAML and instantiating each
  subsystem by hand — the same config-driven opt-in, with no behavior
  change to what gets built or how it's scheduled.
- `DslrCameraSubsystem.from_dict(config, main_yaml_path)` renamed to
  `from_config(config: Config)`.

### Fixed
- `mqtt` is no longer a standalone `SUBSYSTEM_REGISTRY` entry — it was
  reachable via `add_all()`/`simulate=True` even though nothing reads a
  standalone `lab.mqtt` (each rangefinder already builds its own private
  `MqttSubscriber`), and `laguna.simulation`'s drop-list didn't cover it,
  so a config with a top-level `mqtt:` section (e.g. one written just to
  override `broker_host`) could open a real broker connection during what
  was supposed to be a hardware-free rehearsal.
- `CameraArray.from_config()` defaulted `hosts` to the class's own
  `DEFAULT_CAMERAS` (real named lab cameras) instead of `[]` when a
  `pi_cameras:` section had no explicit `hosts:` key — silently targeted
  real hardware instead of no-op.
- `DslrCameraSubsystem.from_config()` now raises a clear `ValueError`
  instead of an opaque `TypeError` when `config.config_file` is `None`.

## [0.1.0] - 2026-08-03

Formalizes semantic versioning for a codebase that had already grown to cover:

- `FlumeLab` core: opt-in subsystem registration, shared clock, scheduler,
  and event log.
- Gantry motion control (Macron driver) with safety verbs, fence checking,
  and Pi-bridge transport.
- Weir, flow, and gauge subsystems for SAFL hydraulic hardware.
- DSLR and Pi camera capture subsystems.
- MQTT-based rangefinder ingestion (OD2000, WTT12L) and calibration.
- Gocator 3D scanner integration and survey planning.
- Experiment scheduling from CSV/Excel, checkpointing, and a runner CLI.
- Full-experiment simulation/rehearsal mode with no hardware attached.
