# Core & config

`FlumeLab` is the top-level orchestrator: an opt-in registry that subsystems
(camera, robot, weir, gauge, flow) attach themselves to by their
`subsystem_name` attribute. `Config` loads and validates the YAML that
drives it.

::: laguna.core

::: laguna.config

## Subsystem registry

::: laguna.registry

## Subsystem logging

Two-tier logging (event log vs. operational log) shared by every subsystem.

::: laguna.subsystem_logging
