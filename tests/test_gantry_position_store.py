"""Tests for GantryPositionStore — the last-known-position checkpoint file
used by GantryController._persist_position()/restore_last_position()."""

import json

import pytest

from laguna.robot.macron.position_store import GantryPositionStore


class TestGantryPositionStore:
    def test_load_missing_file_returns_none(self, tmp_path):
        store = GantryPositionStore(str(tmp_path / "missing.json"))
        assert store.load() is None

    def test_save_then_load_round_trips_positions(self, tmp_path):
        path = tmp_path / "gantry_position.json"
        store = GantryPositionStore(str(path))
        store.save({"X": 12.3, "Y": 45.6, "Z": 0.0, "Theta": 6.0})

        data = store.load()
        assert data["positions"] == {"X": 12.3, "Y": 45.6, "Z": 0.0, "Theta": 6.0}
        assert "wall_time" in data

    def test_save_creates_parent_directories(self, tmp_path):
        path = tmp_path / "nested" / "dir" / "gantry_position.json"
        store = GantryPositionStore(str(path))
        store.save({"X": 1.0})
        assert path.exists()

    def test_save_is_atomic_no_tmp_file_left_behind(self, tmp_path):
        path = tmp_path / "gantry_position.json"
        store = GantryPositionStore(str(path))
        store.save({"X": 1.0})
        assert not path.with_suffix(".tmp").exists()

    def test_save_overwrites_previous_value(self, tmp_path):
        path = tmp_path / "gantry_position.json"
        store = GantryPositionStore(str(path))
        store.save({"X": 1.0})
        store.save({"X": 2.0})
        assert store.load()["positions"] == {"X": 2.0}

    def test_load_malformed_json_returns_none(self, tmp_path):
        path = tmp_path / "gantry_position.json"
        path.write_text("{not valid json")
        store = GantryPositionStore(str(path))
        assert store.load() is None

    def test_load_missing_positions_key_returns_none(self, tmp_path):
        path = tmp_path / "gantry_position.json"
        path.write_text(json.dumps({"wall_time": 123.0}))
        store = GantryPositionStore(str(path))
        assert store.load() is None
