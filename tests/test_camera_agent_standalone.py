"""The Pi camera agent is copied alone to the Pi, where laguna isn't installed."""

import ast
import sys
from pathlib import Path

import laguna.camera.agent as agent_module

AGENT = Path(agent_module.__file__)


def test_agent_imports_only_the_standard_library():
    tree = ast.parse(AGENT.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert node.level == 0, "relative import breaks single-file deploy"
            imported.add((node.module or "").split(".")[0])
    assert "laguna" not in imported
    assert imported <= set(sys.stdlib_module_names)


def test_inlined_timestamp_matches_the_package_helper():
    from laguna.camera._log import utc_local_ts

    got, want = agent_module.utc_local_ts(), utc_local_ts()
    assert len(got[0]) == len(want[0]) and got[0].endswith("Z")
    assert len(got[1]) == len(want[1])
