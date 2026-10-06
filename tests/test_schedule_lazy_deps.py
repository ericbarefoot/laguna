"""``laguna.schedule`` must import without scipy/pandas (issue #11).

numpy is not blocked: ``import laguna`` itself needs it (``laguna.frames``), so
it is a hard requirement of the package regardless of this module.
"""

import subprocess
import sys
import textwrap


def _run(snippet: str) -> subprocess.CompletedProcess:
    """Run `snippet` in a fresh interpreter where scipy and pandas can't be imported."""
    blocker = textwrap.dedent(
        """
        import sys
        for name in ("scipy", "scipy.interpolate", "pandas"):
            sys.modules[name] = None  # makes `import name` raise ImportError
        """
    )
    return subprocess.run(
        [sys.executable, "-c", blocker + textwrap.dedent(snippet)],
        capture_output=True,
        text=True,
    )


def test_import_succeeds_without_scipy_pandas():
    result = _run("from laguna.schedule import ExperimentSchedule, REQUIRED_COLUMNS")
    assert result.returncode == 0, result.stderr


def test_use_without_scipy_pandas_explains_what_to_install():
    result = _run(
        """
        from laguna.schedule import ExperimentSchedule
        try:
            ExperimentSchedule.from_csv("nope.csv")
        except ImportError as exc:
            print(exc)
            raise SystemExit(0)
        raise SystemExit(1)
        """
    )
    assert result.returncode == 0, result.stderr
    assert "pip install numpy scipy pandas" in result.stdout


def test_schedule_still_works_with_deps_present():
    import pandas as pd

    from laguna.schedule import ExperimentSchedule

    sched = ExperimentSchedule.from_dataframe(
        pd.DataFrame({"time_s": [0.0, 10.0], "pump_flow_lpm": [0.0, 10.0], "qin_open": [0, 1]})
    )
    assert sched.pump_flow(5.0) == 5.0
    assert sched.qin_open(9.0) == 0
