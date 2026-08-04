# Contributing

Thanks for your interest in contributing to Laguna.

## Development Setup

```bash
git clone <repo-url>
cd laguna
python3 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -e ".[dev]"
```

`.[docs]` additionally installs MkDocs/Material/mkdocstrings if you're
working on this documentation site — see `make docs-serve` /
`make docs-build` and [the index page](index.md#viewing-these-docs-as-a-site).

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

`pyproject.toml`'s `version` field is the single source of truth —
`laguna.__version__` reads it dynamically via `importlib.metadata`, it isn't a
separate hardcoded string. As you make notable changes, add a bullet under the
`## [Unreleased]` heading in `CHANGELOG.md`. When it's time to cut a version:

```bash
python scripts/bump_version.py patch   # or: minor, major, or an explicit X.Y.Z
```

This bumps `pyproject.toml`, rotates `CHANGELOG.md`'s `[Unreleased]` section
into a dated release entry, and creates a local commit + annotated `vX.Y.Z`
tag. It refuses to run against a dirty working tree, and never pushes —
pushing the branch and tag is a separate, explicit step.

## Adding a New Subsystem

`FlumeLab` is opt-in: subsystems are constructed independently and
attached with `lab.add(subsystem)` rather than hardcoded into `core.py`.
Every existing subsystem (`weir`, `gauge`, `flow`, `camera`,
`robot/macron`) follows the same shape, so a new one should too:

1. Create `src/laguna/<subsystem_name>/` with a main class that:
   - takes a config `dict` (or, if it has more to assemble, exposes a
     `from_config()` classmethod — see `GantryController` in
     `src/laguna/robot/macron/controller.py` for that pattern)
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
3. If it should be wireable from a single experiment config file, add a
   section to `laguna.experiment.runner.setup_run()` following the
   pattern of the existing `if "gauge" in cfg:` / `if "weir" in cfg:`
   blocks — including scheduling via `interval_s` / `trigger_at` /
   `use_schedule` through the shared `_register_action()` helper.
4. Add tests in `tests/test_<subsystem_name>.py` (mock the hardware
   connection; see `tests/macron_fixtures.py` for an example of a shared
   fixture module for a subsystem with several test files).
5. Document it: add a page under `docs/subsystems/` (and list it in
   `mkdocs.yml`'s `nav:`), and add a `::: laguna.<subsystem_name>` block to
   the relevant page under `docs/reference/` so its public API is picked
   up by mkdocstrings automatically.

## Reporting Issues

When reporting bugs, please include:
- Python version and operating system
- Minimal code to reproduce the issue
- Full error messages and stack traces
