# API reference

These pages are generated directly from the docstrings in `src/laguna/` —
they will always match the code, because they render from the code. If
something here looks wrong, the fix is a docstring edit, not a page edit.

The narrative guides (Architecture, Gantry usage guide, subsystem articles)
explain *why* and *how to use* a subsystem. These reference pages are the
exhaustive *what* — every public class, method, and parameter.

| Section | Covers |
|---|---|
| [Core & config](core.md) | `FlumeLab` orchestrator, `Config` loading |
| [Macron gantry driver](macron.md) | Connection, commands, G-code, fences, homing, controller, Pi bridge |
| [Camera](camera.md) | `CameraManager`, DSLR, networked Pi array, local capture |
| [Weir](weir.md) | Tailgate elevation control |
| [Gauge](gauge.md) | Ultrasonic water-level sensing |
| [Flow](flow.md) | Pump / solenoid flow control |
| [Schedule](schedule.md) | Time-elevation schedule loading and interpolation |
| [Timing](timing.md) | Experiment clock, scheduler, checkpointing, event log |
| [Experiment runner](experiment.md) | YAML-driven experiment setup and blocking run |
