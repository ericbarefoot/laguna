# Setting up continuous integration on GitHub

Laguna has no CI today: a regression is only caught if someone runs `pytest` locally on Python 3.14.
This page is the plan for adding it. It is a one-time job of about an hour, and nothing here needs the
hardware, because the suite runs entirely against fakes and `simulate=True`.

## What CI should check

| Check | Command | Notes |
|-------|---------|-------|
| Tests | `pytest` | About 25 s. Coverage is on by default (`addopts` in `pyproject.toml`). |
| Lint | `ruff check src` | Passes today. **Lint `src` only**: `ruff check .` reports over 2,000 errors in tests, examples and notebooks, which would fail every run. Tidy those first if you want to widen it. |
| Docs build (optional) | `mkdocs build --strict` | Needs the `docs` extra. Catches broken links and nav entries. |

Hardware checks (rig, Pi agent, real sensors) cannot run on a GitHub runner. They stay in notebooks, such as
those under `examples/` and `calibration/`.

## 1. Add the workflow

Create `.github/workflows/ci.yml`:

```yaml
name: ci

on:
  pull_request:
  push:
    branches: [develop]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.14"        # pyproject requires-python is >=3.14
          cache: pip
      - name: Install
        run: pip install -e ".[dev,scanner,viz,dslr]"
      - name: Lint
        run: ruff check src
      - name: Test
        run: pytest
```

Notes:

- `dslr` is Linux-only; its wheel bundles libgphoto2, so no system packages are needed. If the install step
  fails on an extra, drop that extra and check which tests actually need it.
- `viz` brings matplotlib, which `tests/test_viz.py` needs. `scanner` brings `laspy` for LAZ export.
- The conda environment (`environment.yml`) is not used in CI. pip with the extras is enough and faster.
- Commit the workflow on a branch and open a PR: the `pull_request` trigger runs CI on that PR itself, so you
  see it work before it lands.

## 2. Make it block merges

A workflow that only reports does not stop a bad merge. In the repository on GitHub:

1. **Settings, Branches, Add branch protection rule** for `develop` (and `main`).
2. Tick **Require a pull request before merging**.
3. Tick **Require status checks to pass before merging**, then search for and select `test` (the job name).
   The check only appears in that list after it has run once, so do step 1 first.
4. Optionally tick **Require branches to be up to date before merging**, so a PR is tested against the
   current `develop` and not the `develop` it branched from.

Stacked PRs deserve care: a PR whose base is another PR's branch is not checked by a rule on `develop`.
Retarget it to `develop` first (`gh pr edit <n> --base develop`), as with #69, which merged into a branch
that had already merged and so never reached `develop`.

## 3. Later additions

- **Docs job:** a second job running `pip install -e ".[docs]"` and `mkdocs build --strict`.
- **Python matrix:** the project requires 3.14, so keep a single version until it supports more.
- **Coverage:** upload `htmlcov` as an artifact, or report to a service such as Codecov.
- **Scheduled run:** add `schedule: - cron: "0 6 * * 1"` under `on:` to catch dependency drift on a weekly basis.
- **Dependabot:** add `.github/dependabot.yml` for the `pip` and `github-actions` ecosystems.

## Running the same checks locally

Without activating the environment (this works in non-interactive shells):

```bash
~/miniforge3/envs/flumelab/bin/python -m pytest -q --no-cov
ruff check src
```

`laguna` is installed editable in that environment, so no install step is needed. From a git worktree, prefix
the test command with `PYTHONPATH=src` so it tests the worktree's code and not the main checkout's.
