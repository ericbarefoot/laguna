"""Test suite for core FlumeLab orchestrator."""

import pytest
from laguna import FlumeLab


class FakeSubsystem:
    subsystem_name = "fake"

    def connect(self):
        return True

    def disconnect(self):
        pass

    def get_status(self):
        return {"ok": True}


class TestFlumeLab:
    @pytest.fixture
    def lab(self):
        return FlumeLab()

    def test_initialization(self, lab):
        assert not lab.is_running
        assert lab._subsystems == {}

    def test_add_subsystem(self, lab):
        sub = FakeSubsystem()
        lab.add(sub)
        assert lab.fake is sub
        assert "fake" in lab._subsystems

    def test_add_returns_self_for_chaining(self, lab):
        sub = FakeSubsystem()
        result = lab.add(sub)
        assert result is lab

    def test_add_requires_subsystem_name(self, lab):
        with pytest.raises(ValueError):
            lab.add(object())

    def test_get_system_status_contains_only_timing_with_no_subsystems(self, lab):
        status = lab.get_system_status()
        assert list(status.keys()) == ["timing"]

    def test_get_system_status_includes_registered_subsystems(self, lab):
        lab.add(FakeSubsystem())
        status = lab.get_system_status()
        assert "fake" in status
        assert status["fake"] == {"ok": True}
