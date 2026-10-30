"""Locate and run LAStools command-line tools.

LAStools ships Linux binaries with a ``64`` suffix (``las2las64``). The
licensed tools (``lasnoise``, ``lasground_new``, ``las2dem``) still exit 0
when the license check fails, and they degrade or refuse beyond a few
million points. Their output is therefore scanned for errors as well as
the exit code.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import TextIO


class LastoolsError(RuntimeError):
    """A LAStools tool is missing, failed, or reported a license failure."""


def lastools_bin() -> Path:
    """Return the directory holding the LAStools binaries.

    Checked in order: ``$LASTOOLS_BIN``, the folder of ``las2las64`` on
    ``PATH``, then ``~/lastools/bin``.
    """
    if env := os.environ.get("LASTOOLS_BIN"):
        return Path(env)
    if found := shutil.which("las2las64"):
        return Path(found).parent
    return Path.home() / "lastools" / "bin"


def run_tool(tool: str, *args: object, log: TextIO | None = None) -> None:
    """Run one LAStools tool.

    Args:
        tool: Tool name without the ``64`` suffix, e.g. ``"las2las"``.
        *args: Command-line arguments; converted with ``str()``.
        log: Open text file that receives the command line and its output.

    Raises:
        LastoolsError: If the binary is missing, exits non-zero, or prints
            an error (including a license failure).
    """
    exe = lastools_bin() / f"{tool}64"
    if not exe.exists():
        raise LastoolsError(f"{exe} not found; set LASTOOLS_BIN")
    cmd = [str(exe), *map(str, args)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    output = proc.stdout + proc.stderr
    if log is not None:
        log.write(f"$ {' '.join(cmd)}\n{output}\n")
    if proc.returncode != 0 or "license failure" in output or "ERROR:" in output:
        raise LastoolsError(f"{tool} failed:\n{output.strip()}")
