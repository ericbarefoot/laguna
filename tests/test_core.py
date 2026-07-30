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


class FakeAxis:
    def __init__(self, name, index):
        self.name = name
        self.index = index

    def token(self):
        return f"A{self.index}"


class FakeGantryCmd:
    def __init__(self, positions):
        self.positions = dict(positions)

    def get_actual_position(self, axis):
        return self.positions[axis.name]


class FakeGantry:
    """Test double for GantryController — just enough for FlumeLab.move_to()/acquire_scan()."""

    subsystem_name = "gantry"

    def __init__(self, positions=None):
        self._axes = (FakeAxis("X", 1), FakeAxis("Y", 2), FakeAxis("Z", 5), FakeAxis("Theta", 6))
        self.cmd = FakeGantryCmd(positions or {"X": 0.0, "Y": 0.0, "Z": 0.0, "Theta": 0.0})
        self.move_to_calls = []

    def move_to(self, vector=None, **axes):
        self.move_to_calls.append((vector, axes))
        return True

    def connect(self):
        return True

    def disconnect(self):
        pass

    def get_status(self):
        return {}


class TestMoveTo:
    @pytest.fixture
    def lab(self):
        return FlumeLab()

    def test_requires_gantry_subsystem(self, lab):
        with pytest.raises(RuntimeError):
            lab.move_to([100, 0, 2, 0])

    def test_delegates_vector_to_gantry(self, lab):
        gantry = FakeGantry()
        lab.add(gantry)
        lab.move_to([100, 0, 2, 0])
        assert gantry.move_to_calls == [([100, 0, 2, 0], {})]

    def test_delegates_keywords_to_gantry(self, lab):
        gantry = FakeGantry()
        lab.add(gantry)
        lab.move_to(X=100, Z=5)
        assert gantry.move_to_calls == [(None, {"X": 100, "Z": 5})]


class TestAcquireScan:
    @pytest.fixture
    def lab(self):
        lab = FlumeLab()
        lab.config.config_dict["od2000"] = {"al1342_host": "192.168.1.251", "pdin_port": 2}
        return lab

    def test_requires_gantry_subsystem(self, lab):
        with pytest.raises(RuntimeError):
            lab.acquire_scan("od2000", end=[100, 0, 0, 0], feed_rate_mm_s=5.0)

    def test_requires_end(self, lab):
        lab.add(FakeGantry())
        with pytest.raises(ValueError):
            lab.acquire_scan("od2000", feed_rate_mm_s=5.0)

    def test_requires_feed_rate(self, lab):
        lab.add(FakeGantry())
        with pytest.raises(ValueError):
            lab.acquire_scan("od2000", end=[100, 0, 0, 0])

    def test_rejects_multi_axis_difference(self, lab):
        lab.add(FakeGantry())
        with pytest.raises(ValueError):
            lab.acquire_scan(
                "od2000", start=[0, 0, 0, 0], end=[100, 50, 0, 0], feed_rate_mm_s=5.0
            )

    def test_infers_scan_axis_and_moves_to_start_first(self, lab, monkeypatch, tmp_path):
        gantry = FakeGantry()
        lab.add(gantry)

        scan_calls = []

        class FakeProfiler:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

            def scan(self, axis, end_mm, feed_rate_mm_s):
                scan_calls.append((axis, end_mm, feed_rate_mm_s))
                from laguna.robot.macron.profiler import ProfileResult

                csv_path = tmp_path / "profile_20260101_000000.csv"
                csv_path.write_text("wall_time_unix,pos_mm\n")
                return ProfileResult(path=csv_path, metadata={"samples": 0}, df=None)

        monkeypatch.setattr("laguna.robot.macron.profiler.TopographicProfiler", FakeProfiler)

        result = lab.acquire_scan(
            "od2000", start=[0, 0, 2, 0], end=[100, 0, 2, 0], feed_rate_mm_s=5.0
        )

        assert gantry.move_to_calls == [([0, 0, 2, 0], {})]
        assert scan_calls == [("A1", 100, 5.0)]  # X axis inferred, token A1
        assert result.metadata == {"samples": 0}

    def test_output_path_renames_result_csv(self, lab, monkeypatch, tmp_path):
        gantry = FakeGantry()
        lab.add(gantry)

        class FakeProfiler:
            def __init__(self, **kwargs):
                pass

            def scan(self, axis, end_mm, feed_rate_mm_s):
                from laguna.robot.macron.profiler import ProfileResult

                csv_path = tmp_path / "raw" / "profile_20260101_000000.csv"
                csv_path.parent.mkdir(parents=True, exist_ok=True)
                csv_path.write_text("wall_time_unix,pos_mm\n")
                meta_path = tmp_path / "raw" / "profile_20260101_000000_meta.json"
                meta_path.write_text("{}")
                return ProfileResult(path=csv_path, metadata={}, df=None)

        monkeypatch.setattr("laguna.robot.macron.profiler.TopographicProfiler", FakeProfiler)

        desired_output = tmp_path / "named" / "myscan.csv"
        result = lab.acquire_scan(
            "od2000", end=[100, 0, 0, 0], feed_rate_mm_s=5.0, output=str(desired_output)
        )

        assert result.path == desired_output
        assert desired_output.exists()
        assert (tmp_path / "named" / "myscan_meta.json").exists()
