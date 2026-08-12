# API reference

These pages are generated directly from the docstrings in `src/laguna/` —
they will always match the code, because they render from the code. If
something here looks wrong, the fix is a docstring edit, not a page edit.

The narrative guides (Architecture, Gantry usage guide, subsystem articles)
explain *why* and *how to use* a subsystem. These reference pages are the
exhaustive *what* — every public class, method, and parameter.

| Section | Covers |
|---|---|
| [Core & config](core.md) | `FlumeLab` orchestrator, `Config` loading, subsystem registry, logging |
| [Safety](safety.md) | The pause/stop/estop/resume vocabulary every subsystem shares |
| [Macron gantry driver](macron.md) | Connection, commands, G-code, fences, homing, controller, motion arbiter, position store, profiler, Pi bridge |
| [Camera](camera.md) | `CameraManager`, DSLR, networked Pi array, local capture, Pi-side agent, gvfs recovery |
| [Weir](weir.md) | Tailgate elevation control |
| [Gauge](gauge.md) | Ultrasonic water-level sensing |
| [Rangefinder](rangefinder.md) | OD2000 / WTT12L distance sensors via AL1342, MQTT subscriber |
| [Scanner](scanner.md) | Gocator 2690 3D laser scanner, simulation stand-in |
| [Flow](flow.md) | Pump / solenoid flow control |
| [Reference frames](frames.md) | `FrameRegistry` — one coordinate system for every instrument |
| [Survey](survey.md) | Multi-pass coverage: `Tile`, `Traverse` |
| [Visualization](viz.md) | `plot_acquisition()`, `plot_trajectory()` — quick-look plots for scans/profiles/planned passes |
| [Schedule](schedule.md) | Time-elevation schedule loading and interpolation |
| [Timing](timing.md) | Experiment clock, scheduler, checkpointing, event log |
| [Experiment runner](experiment.md) | YAML-driven experiment setup and blocking run, run context |
| [Simulation](simulation.md) | `simulate=True` full-experiment rehearsal with no hardware |
| [Data & storage](data_storage.md) | Post-processing and remote storage backends |
