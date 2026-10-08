# Changelog

All notable changes to this project are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

Add entries under `[Unreleased]` as you work; `scripts/bump_version.py` moves
them into a dated release section when you cut a version.

## [Unreleased]

### Changed
- **Alignment and calibration runs are written to `calibration/results/`, not `data/scans/`.** The alignment
  notebook's run folder (`seam_test_<time>`: block position, passes, corners, solution, trigger-delay summary) now
  comes from `AlignmentStore.new_run()`, and `AlignmentStore.latest()` searches there by default. Runs already in
  `data/scans/` are left where they are: pass `LOAD_FROM` the old folder to reuse one.

### Fixed
- `examples/example_14_scheduled_tiled_scan_hook.py` reused one checkpoint across firings, so every firing after
  the first was a silent no-op; it now makes one per firing.
- **Reverse WTT12L scans were mislabelled.** The Pi agent computed every sample's `pos_mm` as if the axis moved in
  the positive direction, so any scan toward lower positions had its positions mirrored about its start point. An
  out-and-back average (the alignment notebook's block position) then landed near the reverse scan's start instead
  of the block, wrong by about half a scan length per axis (about 166 mm and 106 mm in the saved runs), and so was
  the Gocator translation solved from it. The agent now takes the direction from start versus end and includes the
  acceleration ramp distance. `laguna.robot.macron.profiler.recompute_positions()` rebuilds `pos_mm` for saved scans
  from their raw timestamps and sidecar (the file is left untouched).
- **`orient_scan` mirrored passes whenever `scan_y` was negative.** It multiplied the gantry's start-to-end direction
  by the mounting's sign on the travel axis, putting a forward pass behind its start and a reverse pass beyond it.
  The direction along travel now comes from start versus end alone.
- **`SurveyRunner` offset a tile by one swath in a rotated experiment frame.** The sensor edge placed on a pass's
  near edge was chosen from the gantry-axis mounting matrix although the tile steps along an experiment axis; it now
  goes through the frame's rotation. A rotated frame also scanned along the wrong gantry axis at 90/270 degrees: the
  runner now maps each experiment axis to the gantry axis that moves along it (`FrameRegistry.gantry_axis_for` /
  `experiment_axis_for`) and refuses a rotation that is not a multiple of 90 degrees before anything moves.
- `SurfaceScan.save_npz` wrote numpy scalars as `np.float64(...)` in its metadata, which `from_npz` could not read
  back, so such scans could not be reloaded. `save_npz` now writes plain values and `from_npz` reads the old files.

### Added
- **Surveys can be scheduled from config.** A `surveys:` section (`laguna.survey_config`) takes `kind: tile` or
  `traverse` entries, the `Tile`/`Traverse` constructor arguments plus `interval_s`/`trigger_at` and
  `max_scan_speed_mm_s`; `swath_mm: auto` reads the live active area at each firing. Unknown keys are an error and
  `setup_run()` validates everything before anything connects or moves. Each firing plans afresh against a new, kept
  checkpoint under `<run dir>/surveys/`; a firing interrupted by a pause is not resumed automatically. Motion still
  goes through `safe_mode`, fences and the motion arbiter.
- **Survey checkpoints carry a geometry fingerprint.** `Survey.fingerprint()` hashes what each pass measures
  (instrument, travel axis, measuring start and end, swath alignment; not speeds, labels or the ramp start).
  `SurveyRunner` stamps it into the checkpoint's new `meta` and, at construction, refuses a checkpoint with completed
  passes for different geometry, or recorded without a fingerprint, with `SurveyCheckpointMismatch`. Nothing is
  deleted: `restart=True` moves the old file aside via `CheckpointStore.clear()`. Older checkpoints still load.
- **A survey pass with no `scan_speed` takes one from the scanner:** its configured `scan.feed_rate_mm_s`
  (`GocatorScanner.configured_feed_rate_mm_s`), else `solve_scan_rates()` capped at
  `SurveyRunner(max_scan_speed_mm_s=...)`, which is required for the automatic choice. An explicit speed is never
  overridden, and instruments without a solver (rangefinders) still fail loudly.
- **`SurveyRunner.run(place_results=True)`** fills `runner.placed` with each pass's result in experiment
  coordinates (`orient_scan` / `orient_profile`). The raw result is kept; a placement failure is logged to the
  event log (`survey_place`) and never aborts the survey.
- **`surveys:` entries can take a region of interest.** A `kind: tile` entry with `roi: {x_mm, y_mm, z_mm}` is planned
  by `Tile.from_roi()` (passes and overlap worked out from the swath and `min_overlap`), optionally along a
  `gantry_axis`. `origin`, `length_mm`, `width_mm`, `overlap` and `step_axis` come from the region and are refused alongside it.
- **`laguna-picam` CLI and `scripts/picam-remote.sh`** — snapshot or live-view
  a Pi camera from a remote client, relayed client → laguna → pi over SSH
  pipes (no ports opened). See `docs/subsystems/camera.md`. The Pi camera is
  exclusive, so don't leave a view open when a scheduled capture is due.
- **DSLR control is now part of laguna** (`laguna.camera.canon`, issue
  #60). It is vendored from Minsik's (@yukms)
  [dualcam-timelapse](https://github.com/yukms/dualcam-timelapse), which no
  longer needs to be cloned separately. Install it with the new `dslr`
  extra (`pip install -e ".[dslr]"`). Supports any number of cameras.
  - Each camera is bound by its EOS body serial on every connect and
    `resume()`, so cameras can't swap names when their USB ports change.
  - Pre-flight refuses to connect unless the camera is on M, the lens is on
    MF, and auto power-off is disabled. Every setting written is read back.
  - Every download is verified against the card copy. Card files are
    deleted (oldest first, verified files only) once free space drops below
    `card_reserve_shots`. A failed download is retried once.
  - `capture_target: ram` supports bodies with no SD card. The verified
    download is then the only copy. It applies to every camera, because
    libgphoto2 holds one capture target for the whole computer; a
    per-camera value is an error.
  - A failed capture escalates to a lab-wide pause.
  - `scripts/setup_dslr_udev.py` lists cameras by serial and installs
    `/dev/dslr_<name>` symlinks.
  - Pre-flight sets each camera's clock from the PC (`syncdatetime`), so
    EXIF times are correct. On the lab T7s they come out in UTC.
- **Tile scan lead-out.** A `Tile` with `accel_mm_s2` now sends each pass one ramp past its swath end
  (`Pass.overrun_end`), mirroring the lead-in, so the swath is covered entirely at constant speed and the slowdown
  happens outside it; `scan_with_gantry(capture_end_mm=...)` ends the capture at the swath end. The stop is
  fence-checked like any other end point, and it extends commanded travel by `v^2 / 2a` per pass.
- **`gocator.trigger_delay_s`** (default `0.0`, so nothing changes until calibrated), added to the wait before a
  gantry pass's trigger to cover the command-to-motion latency. `laguna.scanner.trigger_delay` fits it from
  forward/reverse block offsets, including `summarize_trigger_delay()` for repeats at one speed, and
  `laguna.viz.plot_trigger_delay()` plots them. Calibrate at the speed you will scan at.
- `Tile.from_roi()`: plan a tile from experiment-frame ROI bounds, with the swath read from the live active area;
  passes and overlap are solved (fewest passes keeping `min_overlap`, spread evenly), a region no wider than one
  swath is a single centred pass, and `gantry_axis=` fixes the gantry axis scanned on whatever the frame's rotation.
- `frames.experiment.origin`, an alternative to `translation` that names the gantry point which is the experiment
  origin, so changing `rotation_deg` does not move it.
- `laguna.viz.plot_survey_plan()`: top-down plan with origins, soft limits, sensor paths, swaths and, given a lab,
  the carriage path and the footprint the live mounting and active area will image.
- `laguna.scanner.block_finder.find_block()`: a local-bed block detector for uneven beds.
  `laguna.alignment_store.AlignmentStore` saves and reloads the intermediate results of an alignment run, so a rerun
  can skip a scan. `GocatorScanner.get_alignment()` reads the sensor's alignment transform.
- `calibration/gocator_alignment_and_seam.ipynb` (alignment, tile planning, seam analysis, trigger-delay
  calibration), moved here from `examples/` because it is a procedure that gets re-run.
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
- **Breaking: new `dslr_cameras` config schema.** Each camera now needs
  `serial` and `exposure: {iso, aperture, shutter}`. Optional keys are
  `imageformat`, `device`, `capture_target` and `card_reserve_shots`. `dualcam_path`,
  `config_path` and `port` are gone, and laguna no longer writes detected
  ports back into the experiment YAML.
- **Breaking:** `DslrCameraSubsystem.capture_all()` now returns
  `{name: CaptureRecord}` instead of `{name: Path | None}`.
  `DslrCameraSubsystem` now implements `pause`/`resume`/`stop`/`estop`.
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
