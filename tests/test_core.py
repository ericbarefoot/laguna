"""Test suite for core FlumeLab orchestrator."""

import time

import pytest
from laguna import FlumeLab
from laguna.frames import FrameRegistry


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

    def test_getattr_falls_back_to_registered_subsystems(self, lab):
        sub = FakeSubsystem()
        lab.add(sub)
        delattr(lab, "fake")

        assert lab.fake is sub

    def test_add_returns_self_for_chaining(self, lab):
        sub = FakeSubsystem()
        result = lab.add(sub)
        assert result is lab

    def test_add_requires_subsystem_name(self, lab):
        with pytest.raises(ValueError):
            lab.add(object())

    def test_getattr_returns_registered_subsystem(self, lab):
        sub = FakeSubsystem()
        lab.add(sub)
        assert lab.__getattr__("fake") is sub

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
        from laguna.robot.motion_arbiter import MotionArbiter

        self._axes = (FakeAxis("X", 1), FakeAxis("Y", 2), FakeAxis("Z", 5), FakeAxis("Theta", 6))
        self.cmd = FakeGantryCmd(positions or {"X": 0.0, "Y": 0.0, "Z": 0.0, "Theta": 0.0})
        self.move_to_calls = []
        self.arbiter = MotionArbiter(timeout_s=0.2)

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


class TestPlace:
    """lab.place() names a measurement location instead of a robot position,
    compensating for where each instrument is mounted."""

    def _lab(self):
        lab = FlumeLab()
        lab.frames = FrameRegistry.from_config(
            {
                "experiment": {"translation": [500.0, 300.0, 0.0]},
                "instruments": {
                    "od2000": {"translation": [52.0, -18.0, 0.0]},
                    "wtt12l": {"translation": [52.0, 31.0, 0.0]},
                },
            }
        )
        gantry = FakeGantry()
        lab.add(gantry)
        return lab, gantry

    def test_compensates_for_the_instrument_offset(self):
        lab, gantry = self._lab()
        lab.place("od2000", [100.0, 200.0, 0.0])
        _, axes = gantry.move_to_calls[0]
        # experiment -> gantry is -500/-300, then subtract the mount offset
        assert axes["X"] == pytest.approx(-452.0)
        assert axes["Y"] == pytest.approx(-82.0)

    def test_same_target_gives_different_commands_per_instrument(self):
        lab, gantry = self._lab()
        lab.place("od2000", [100.0, 200.0, 0.0])
        lab.place("wtt12l", [100.0, 200.0, 0.0])
        first, second = gantry.move_to_calls
        assert first[1]["Y"] != second[1]["Y"]
        assert first[1]["Y"] - second[1]["Y"] == pytest.approx(49.0)

    def test_leaves_theta_untouched(self):
        """Theta is outside the Cartesian frame model, so place() must use
        the keyword form rather than a full vector."""
        lab, gantry = self._lab()
        lab.place("od2000", [0.0, 0.0, 0.0])
        vector, axes = gantry.move_to_calls[0]
        assert vector is None
        assert "Theta" not in axes

    def test_unconfigured_instrument_reduces_to_move_to(self):
        lab = FlumeLab()
        gantry = FakeGantry()
        lab.add(gantry)
        lab.place("mystery", [1.0, 2.0, 3.0])
        _, axes = gantry.move_to_calls[0]
        assert (axes["X"], axes["Y"], axes["Z"]) == (1.0, 2.0, 3.0)

    def test_requires_gantry_subsystem(self):
        with pytest.raises(RuntimeError):
            FlumeLab().place("od2000", [0, 0, 0])


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

    def test_holds_the_motion_arbiter_across_the_scan(self, lab, monkeypatch, tmp_path):
        """A scheduled Gocator scan holds the same arbiter via
        scan_with_gantry() — acquire_scan() must too, or the two could
        command the gantry at the same time."""
        gantry = FakeGantry()
        lab.add(gantry)
        held_during_scan = []

        class FakeProfiler:
            def __init__(self, **kwargs):
                pass

            def scan(self, axis, end_mm, feed_rate_mm_s):
                held_during_scan.append(gantry.arbiter.is_held)
                from laguna.robot.macron.profiler import ProfileResult

                csv_path = tmp_path / "profile_20260101_000000.csv"
                csv_path.write_text("wall_time_unix,pos_mm\n")
                return ProfileResult(path=csv_path, metadata={}, df=None)

        monkeypatch.setattr("laguna.robot.macron.profiler.TopographicProfiler", FakeProfiler)

        lab.acquire_scan("od2000", end=[100, 0, 0, 0], feed_rate_mm_s=5.0)

        assert held_during_scan == [True]
        assert gantry.arbiter.is_held is False, "arbiter must be released afterward"

    def test_refuses_to_start_while_the_gantry_is_already_held(self, lab, monkeypatch):
        """A concurrently-held arbiter (e.g. a scheduled Gocator scan already
        in flight, on another thread) must make acquire_scan() fail loudly
        rather than let both callers command the gantry. MotionArbiter is a
        re-entrant lock, so contention only shows up across threads — the
        same thread re-entering its own hold is fine by design."""
        import threading

        from laguna.robot.motion_arbiter import MotionBusyError

        gantry = FakeGantry()
        lab.add(gantry)
        monkeypatch.setattr(
            "laguna.robot.macron.profiler.TopographicProfiler", lambda **kwargs: None
        )

        released = threading.Event()

        def hold_from_another_thread():
            with gantry.arbiter.hold("someone else's scan"):
                released.wait(timeout=5.0)

        holder = threading.Thread(target=hold_from_another_thread)
        holder.start()
        try:
            deadline = time.time() + 5.0
            while gantry.arbiter.holder != "someone else's scan" and time.time() < deadline:
                time.sleep(0.005)
            assert gantry.arbiter.holder == "someone else's scan", "holder thread never acquired"
            with pytest.raises(MotionBusyError):
                lab.acquire_scan("od2000", end=[100, 0, 0, 0], feed_rate_mm_s=5.0)
        finally:
            released.set()
            holder.join(timeout=5.0)
