"""Shared pytest configuration for the laguna test suite."""

import pytest

from laguna.robot.macron import commands as _macron_commands


@pytest.fixture(autouse=True)
def _no_real_move_polling_sleeps(monkeypatch):
    """Collapse the sparse move-completion polling delays for offline tests.

    commands.poll_until_move_finished deliberately sleeps through most of a
    move's predicted duration and then polls every SPARSE_POLL_INTERVAL_S
    (0.5s) — that pacing is the point on real hardware (see that function's
    module note), but against FakeSnapConnection there is no physical move
    to wait for, and honoring it would add seconds of pure sleeping to the
    suite. Zeroing both knobs keeps the polling *logic* under test while
    removing the wall-clock cost.
    """
    monkeypatch.setattr(_macron_commands, "SPARSE_POLL_INTERVAL_S", 0.0)
    monkeypatch.setattr(_macron_commands, "PREDICTED_SLEEP_FRACTION", 0.0)
