# Laguna

Control software for robotic systems in hydraulic flume experiments.

Laguna coordinates a set of opt-in hardware subsystems — weir elevation,
water-level gauge, pump/flow, DSLR and Pi cameras, and the Modusystems
gantry — under a shared clock, scheduler, and event log, via a central
`FlumeLab` orchestrator.

## Documentation

This README is intentionally minimal. The real documentation lives in
[`docs/`](docs/index.md) as an MkDocs + Material site — architecture,
per-subsystem guides, the gantry driver reference, and an API reference
generated from docstrings.

```bash
make docs-install   # pip install -e ".[docs]"
make docs-serve      # http://127.0.0.1:8000, live-reloads as you edit docs/*.md
```

## Install

```bash
git clone <repo-url>
cd laguna
python3 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -e ".[dev]"
```

## Run Your First Experiment

```bash
cp config/example_config.yaml config/my_experiment.yaml
# edit ports/hosts for your hardware, then:
python experiments/weir_gauge_camera_experiment.py \
    --lab-config config/my_experiment.yaml --duration 30
```

See the [Quick reference](docs/QUICKREF.md) for interactive/REPL control,
scheduling actuators against a CSV, and troubleshooting.

## License

MIT License — see [LICENSE](LICENSE) for details.
