#!/usr/bin/env python3
"""Bump laguna's version: pyproject.toml, CHANGELOG.md, commit, and tag.

Usage:
    python scripts/bump_version.py major|minor|patch
    python scripts/bump_version.py 1.2.3

Requires a clean working tree (bump commits should be atomic). Creates a
local commit and an annotated git tag `vX.Y.Z` — does not push either; that
remains an explicit, separate step.
"""

import datetime
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PYPROJECT = REPO_ROOT / "pyproject.toml"
CHANGELOG = REPO_ROOT / "CHANGELOG.md"

VERSION_RE = re.compile(r'^version = "(\d+)\.(\d+)\.(\d+)"$', re.MULTILINE)
UNRELEASED_RE = re.compile(r"^## \[Unreleased\]$", re.MULTILINE)


def current_version() -> tuple[int, int, int]:
    match = VERSION_RE.search(PYPROJECT.read_text())
    if not match:
        raise SystemExit(f"could not find a version line in {PYPROJECT}")
    return tuple(int(part) for part in match.groups())


def next_version(current: tuple[int, int, int], bump: str) -> str:
    major, minor, patch = current
    if bump == "major":
        return f"{major + 1}.0.0"
    if bump == "minor":
        return f"{major}.{minor + 1}.0"
    if bump == "patch":
        return f"{major}.{minor}.{patch + 1}"
    if re.fullmatch(r"\d+\.\d+\.\d+", bump):
        return bump
    raise SystemExit(f"expected major|minor|patch or X.Y.Z, got {bump!r}")


def ensure_clean_tree() -> None:
    result = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    if result.stdout.strip():
        raise SystemExit(
            "working tree is not clean — commit or stash changes before bumping"
        )


def update_pyproject(new_version: str) -> None:
    text = PYPROJECT.read_text()
    updated, count = VERSION_RE.subn(f'version = "{new_version}"', text, count=1)
    if count != 1:
        raise SystemExit(f"expected exactly one version line in {PYPROJECT}")
    PYPROJECT.write_text(updated)


def update_changelog(new_version: str) -> None:
    text = CHANGELOG.read_text()
    if not UNRELEASED_RE.search(text):
        raise SystemExit(f"could not find '## [Unreleased]' heading in {CHANGELOG}")
    today = datetime.date.today().isoformat()
    replacement = f"## [Unreleased]\n\n## [{new_version}] - {today}"
    updated = UNRELEASED_RE.sub(replacement, text, count=1)
    CHANGELOG.write_text(updated)


def commit_and_tag(new_version: str) -> None:
    subprocess.run(
        ["git", "add", str(PYPROJECT), str(CHANGELOG)], cwd=REPO_ROOT, check=True
    )
    subprocess.run(
        ["git", "commit", "-m", f"chore: bump version to {new_version}"],
        cwd=REPO_ROOT,
        check=True,
    )
    subprocess.run(
        ["git", "tag", "-a", f"v{new_version}", "-m", f"v{new_version}"],
        cwd=REPO_ROOT,
        check=True,
    )


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    ensure_clean_tree()
    current = current_version()
    new_version = next_version(current, sys.argv[1])
    update_pyproject(new_version)
    update_changelog(new_version)
    commit_and_tag(new_version)
    print(f"bumped {'.'.join(map(str, current))} -> {new_version}, tagged v{new_version}")
    print("nothing was pushed — push the branch and tag explicitly when ready")


if __name__ == "__main__":
    main()
