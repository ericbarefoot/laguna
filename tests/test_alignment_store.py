"""Saving and reloading an alignment run's intermediate results."""

import numpy as np
import pytest

from laguna.alignment_store import CALIBRATION_RESULTS_DIR, PASS_NAMES, AlignmentStore
from laguna.scanner.pointcloud import SurfaceScan


def make_scan(start=303.5):
    return SurfaceScan(
        z_mm=np.array([[1.0, 2.0], [np.nan, 4.0]]),
        x_mm=np.array([0.0, 1.0]), y_mm=np.array([0.0, 2.0]),
        metadata={"gantry_axis": "X", "gantry_start_mm": np.float64(start), "gantry_end_mm": start + 600.0},
        is_uniform=True,
    )


class TestBlock:
    def test_block_round_trips_with_numpy_values(self, tmp_path):
        store = AlignmentStore(tmp_path / "run")
        store.save_block(np.array([1200.3, 671.5]), {"X": {"center": np.float64(1200.3)}}, (196.85, 98.4))
        np.testing.assert_allclose(AlignmentStore(tmp_path / "run").load_block(), [1200.3, 671.5])

    def test_loading_from_an_empty_or_missing_folder_returns_none_and_creates_nothing(self, tmp_path):
        store = AlignmentStore(tmp_path / "nope")
        assert store.load_block() is None
        assert store.load_passes() == {}
        assert store.load_corners() == {}
        assert store.load_solution() is None
        assert not (tmp_path / "nope").exists()

    def test_falls_back_to_the_block_inside_an_older_solution_file(self, tmp_path):
        store = AlignmentStore(tmp_path)
        store.save_solution([1200.0, 670.0], {"scan_x": "+Y", "scan_y": "-X", "scan_z": "+Z"}, [597, 312, 0])
        np.testing.assert_allclose(store.load_block(), [1200.0, 670.0])


class TestPreFixBlockPositions:
    def test_a_block_saved_now_is_marked_and_loads_quietly(self, tmp_path, caplog):
        store = AlignmentStore(tmp_path)
        store.save_block([1.0, 2.0])
        with caplog.at_level("WARNING"):
            store.load_block()
        assert "reverse-scan position fix" not in caplog.text

    def test_an_old_solution_file_warns(self, tmp_path, caplog):
        store = AlignmentStore(tmp_path)
        store.save_solution([1200.0, 670.0], {"scan_x": "+Y"}, [0, 0, 0])
        with caplog.at_level("WARNING"):
            store.load_block()
        assert "reverse-scan position fix" in caplog.text

    def test_a_block_file_without_the_marker_warns(self, tmp_path, caplog):
        (tmp_path / "block_location.json").write_text('{"block_center_gantry_mm": [1.0, 2.0]}')
        with caplog.at_level("WARNING"):
            AlignmentStore(tmp_path).load_block()
        assert "reverse-scan position fix" in caplog.text


class TestPasses:
    def test_passes_round_trip_including_numpy_metadata(self, tmp_path):
        store = AlignmentStore(tmp_path)
        for i, name in enumerate(PASS_NAMES):
            store.save_pass(name, make_scan(300.0 + i))
        loaded = AlignmentStore(tmp_path).load_passes()
        assert list(loaded) == list(PASS_NAMES)
        assert loaded["P2"].metadata["gantry_start_mm"] == pytest.approx(301.0)

    def test_missing_passes_are_left_out(self, tmp_path):
        store = AlignmentStore(tmp_path)
        store.save_pass("P1", make_scan())
        assert list(store.load_passes()) == ["P1"]

    def test_rejects_an_unknown_pass_name(self, tmp_path):
        with pytest.raises(ValueError, match="pass name"):
            AlignmentStore(tmp_path).save_pass("P9", make_scan())


class TestCornersAndSolution:
    def test_corners_round_trip_as_tuples(self, tmp_path):
        store = AlignmentStore(tmp_path)
        store.save_corners({"P1": np.array([1.0, 2.0, 3.0, 4.0])})
        assert store.load_corners() == {"P1": (1.0, 2.0, 3.0, 4.0)}

    def test_solution_round_trips(self, tmp_path):
        store = AlignmentStore(tmp_path)
        store.save_solution([1.0, 2.0], {"scan_x": "+Y"}, np.array([5.0, 6.0, 0.0]))
        sol = store.load_solution()
        assert sol["translation"] == [5.0, 6.0, 0.0] and sol["mounting"] == {"scan_x": "+Y"}


class TestLatestAndProvenance:
    def test_latest_picks_the_newest_run_that_has_the_result(self, tmp_path):
        AlignmentStore(tmp_path / "seam_test_20261007_100000").save_block([1.0, 1.0])
        AlignmentStore(tmp_path / "seam_test_20261007_120000").save_block([2.0, 2.0])
        AlignmentStore(tmp_path / "seam_test_20261007_130000")        # newest, but empty
        latest = AlignmentStore.latest(tmp_path, having="block")
        np.testing.assert_allclose(latest.load_block(), [2.0, 2.0])

    def test_latest_passes_needs_all_three(self, tmp_path):
        partial = AlignmentStore(tmp_path / "seam_test_2")
        partial.save_pass("P1", make_scan())
        full = AlignmentStore(tmp_path / "seam_test_1")
        for name in PASS_NAMES:
            full.save_pass(name, make_scan())
        assert AlignmentStore.latest(tmp_path, having="passes").directory == full.directory

    def test_latest_returns_none_when_nothing_matches(self, tmp_path):
        assert AlignmentStore.latest(tmp_path, having="solution") is None

    def test_latest_rejects_an_unknown_kind(self, tmp_path):
        with pytest.raises(ValueError, match="having"):
            AlignmentStore.latest(tmp_path, having="bogus")

    def test_provenance_is_recorded(self, tmp_path):
        new, old = AlignmentStore(tmp_path / "new"), AlignmentStore(tmp_path / "old")
        path = new.note_loaded_from(old, ["block", "passes"])
        assert "old" in path.read_text() and "passes" in path.read_text()


class TestNewRun:
    def test_defaults_to_the_calibration_folder_not_data(self):
        store = AlignmentStore.new_run()
        assert store.directory.parent == CALIBRATION_RESULTS_DIR
        assert "data" not in store.directory.parts
        assert store.directory.name.startswith("seam_test_")

    def test_creates_nothing_until_the_first_write(self, tmp_path):
        store = AlignmentStore.new_run(tmp_path / "results")
        assert not (tmp_path / "results").exists()
        store.save_block([1.0, 2.0])
        assert store.directory.is_dir() and store.directory.parent == tmp_path / "results"

    def test_latest_finds_a_new_run_in_the_same_root(self, tmp_path):
        store = AlignmentStore.new_run(tmp_path)
        store.save_block([3.0, 4.0])
        assert AlignmentStore.latest(tmp_path).directory == store.directory

    def test_latest_searches_the_calibration_folder_by_default(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        AlignmentStore.new_run().save_block([5.0, 6.0])
        assert AlignmentStore.latest() is not None
