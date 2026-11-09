"""The Pi camera agent is copied to the Pi and run outside the laguna install."""

import sys
from pathlib import Path

AGENT = Path(__file__).resolve().parents[1] / "src" / "laguna" / "camera" / "agent.py"


def test_agent_imports_without_laguna_on_the_path():
    code = (
        "import runpy, sys\n"
        "sys.path = [p for p in sys.path if 'laguna' not in p]\n"
        f"runpy.run_path({str(AGENT)!r})\n"
    )
    proc = __import__("subprocess").run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    assert "No module named 'laguna'" not in proc.stderr
