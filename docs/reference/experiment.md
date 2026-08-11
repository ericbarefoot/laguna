# Experiment runner

YAML-driven glue that wires subsystems and a schedule together into a
blocking experiment run.

::: laguna.experiment.runner

## Run context

Ties one experiment run's outputs (data directory, run ID, manifest)
together — used by both the runner above and `FlumeLab` directly.

::: laguna.run_context
