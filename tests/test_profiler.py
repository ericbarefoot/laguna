"""Tests for TopographicProfiler and ProfileResult.

All offline — no real SSH/SFTP/paramiko or gantry_agent.py involved. The
scan itself is exercised through a fake gantry.connection exposing
start_scan()/wait_for_scan_result()/stop_scan() (matching the real
PiGantryConnection API — see pi_bridge.py); only the CSV/metadata retrieval
step still touches paramiko, and that's faked the same way profiler tests
always have. Each test documents one assumption about the orchestration; a
failure on hardware reveals which step in the sequence went wrong.
"""

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from laguna.robot.macron.connection import SnapMotionError
from laguna.robot.macron.profiler import ProfileResult, TopographicProfiler


# ---------------------------------------------------------------------------
# ProfileResult dataclass
# ---------------------------------------------------------------------------


class TestProfileResult:
    def test_result_stores_path_and_metadata(self, tmp_path):
        p = tmp_path / "profile.csv"
        p.write_text("a,b\n1,2\n")
        result = ProfileResult(path=p, metadata={"samples": 100})
        assert result.path == p
        assert result.metadata["samples"] == 100

    def test_result_df_defaults_none(self, tmp_path):
        p = tmp_path / "profile.csv"
        result = ProfileResult(path=p, metadata={})
        assert result.df is None


# ---------------------------------------------------------------------------
# TopographicProfiler construction
# ---------------------------------------------------------------------------


class TestProfilerConstruction:
    def _make(self, **overrides):
        kwargs = dict(
            gantry=MagicMock(),
            pi_host="red.lab",
            pi_user="oak",
            al1342_host="192.168.1.251",
        )
        kwargs.update(overrides)
        return TopographicProfiler(**kwargs)

    def test_requires_al1342_host(self):
        """al1342_host is required — empty string should raise ValueError.
        On hardware: the AL1342 has no DNS of its own, so this must be a
        raw IP (e.g. '192.168.1.251'), not a hostname like 'al1342.lab'.
        """
        with pytest.raises(ValueError, match="al1342_host"):
            TopographicProfiler(
                gantry=MagicMock(),
                pi_host="red.lab",
                pi_user="oak",
                al1342_host="",
            )

    def test_stores_config(self):
        p = self._make(pdin_port=3, al1342_host="192.168.1.251", output_dir="/data")
        assert p._pdin_port == 3
        assert p._al1342_host == "192.168.1.251"
        assert p._output_dir == Path("/data")

    def test_sensor_defaults_to_od2000(self):
        p = self._make()
        assert p._sensor == "od2000"

    def test_sensor_stored_when_given(self):
        p = self._make(sensor="wtt12l_powerprox")
        assert p._sensor == "wtt12l_powerprox"


# ---------------------------------------------------------------------------
# Fake gantry connection + SFTP infrastructure for scan() tests
# ---------------------------------------------------------------------------


def _make_ack(**overrides):
    ack = {"scan_started": True, "start_pos_mm": 100.0, "accel_mm_s2": 10.0, "decel_mm_s2": 10.0}
    ack.update(overrides)
    return ack


def _make_result(**overrides):
    result = {
        "scan_done": True,
        "csv_path": "/tmp/profile_test.csv",
        "meta_path": "/tmp/profile_test_meta.json",
        "actual_start_mm": 100.1,
        "actual_end_mm": 500.5,
        "samples": 850,
        "achieved_rate_hz": 300.0,
    }
    result.update(overrides)
    return result


class FakeGantryConnection:
    """Fake for PiGantryConnection's scan control API (pi_bridge.py)."""

    def __init__(self, ack=None, result=None, raise_on_start=None):
        self.start_scan_calls = []
        self.wait_calls = []
        self.stop_scan_calls = 0
        self._ack = ack if ack is not None else _make_ack()
        self._result = result if result is not None else _make_result()
        self._raise_on_start = raise_on_start

    def start_scan(self, axis, end_mm, feed_rate_mm_s, al1342_host, pdin_port, output,
                    sensor="od2000"):
        self.start_scan_calls.append(
            {"axis": axis, "end_mm": end_mm, "feed_rate_mm_s": feed_rate_mm_s,
             "al1342_host": al1342_host, "pdin_port": pdin_port, "output": output,
             "sensor": sensor}
        )
        if self._raise_on_start is not None:
            raise self._raise_on_start
        return self._ack

    def wait_for_scan_result(self, timeout):
        self.wait_calls.append(timeout)
        return self._result

    def stop_scan(self):
        self.stop_scan_calls += 1


class FakeSftp:
    """Simulates paramiko SFTP that creates a local stub CSV on get()."""

    def __init__(self):
        self.gets = []

    def get(self, remote, local):
        self.gets.append((remote, local))
        Path(local).write_text(
            "wall_time_unix,wall_time_iso,pos_mm,distance_nm,distance_mm,q1,q2,in_ramp\n"
            "1000.0,2026-01-01T00:00:00Z,100.0,200000000,200.0,0,0,0\n"
        )

    def close(self):
        pass


class FakeSSHClient:
    """Simulates paramiko.SSHClient for the SFTP-only retrieval step."""

    def __init__(self):
        self.sftp = FakeSftp()
        self.connected = False
        self.closed = False

    def set_missing_host_key_policy(self, policy):
        pass

    def connect(self, host, **kwargs):
        self.connected = True

    def open_sftp(self):
        return self.sftp

    def close(self):
        self.closed = True


def _make_profiler(gantry_connection=None, output_dir="/tmp", sensor="od2000"):
    if gantry_connection is None:
        gantry_connection = FakeGantryConnection()
    gantry = MagicMock()
    gantry.connection = gantry_connection
    profiler = TopographicProfiler(
        gantry=gantry,
        pi_host="red.lab",
        pi_user="oak",
        pdin_port=2,
        al1342_host="192.168.1.251",
        output_dir=output_dir,
        sensor=sensor,
    )
    return profiler, gantry_connection


# ---------------------------------------------------------------------------
# scan() orchestration tests
# ---------------------------------------------------------------------------


class TestProfilerScan:
    def _run_scan(self, tmp_path, gantry_connection=None):
        profiler, conn = _make_profiler(gantry_connection, output_dir=str(tmp_path))
        fake_client = FakeSSHClient()
        with patch("paramiko.SSHClient", return_value=fake_client):
            result = profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)
        return result, conn, fake_client

    def test_happy_path_returns_profile_result(self, tmp_path):
        result, conn, _ = self._run_scan(tmp_path)
        assert isinstance(result, ProfileResult)
        assert result.metadata["scan_done"] is True

    def test_start_scan_called_with_correct_args(self, tmp_path):
        """On hardware: if any of these are wrong, the scan runs against
        the wrong axis/target/AL1342/port, or the CSV lands somewhere the
        retrieval step won't find it.
        """
        _, conn, _ = self._run_scan(tmp_path)
        assert len(conn.start_scan_calls) == 1
        call = conn.start_scan_calls[0]
        assert call["axis"] == "A1"
        assert call["end_mm"] == 500.0
        assert call["feed_rate_mm_s"] == 5.0
        assert call["al1342_host"] == "192.168.1.251"
        assert call["pdin_port"] == 2
        assert call["output"].startswith("/tmp/profile_") and call["output"].endswith(".csv")
        assert call["sensor"] == "od2000"

    def test_sensor_passed_through_to_start_scan(self, tmp_path):
        profiler, conn = _make_profiler(output_dir=str(tmp_path), sensor="wtt12l_powerprox")
        fake_client = FakeSSHClient()
        with patch("paramiko.SSHClient", return_value=fake_client):
            profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)
        assert conn.start_scan_calls[0]["sensor"] == "wtt12l_powerprox"

    def test_wait_timeout_scales_with_distance_and_feed_rate(self, tmp_path):
        """move_timeout = distance/feed_rate + 30s buffer. On hardware: if
        this is too short, a long/slow scan gets killed client-side before
        the agent actually finishes — check this formula before blaming
        the agent for a spurious timeout.
        """
        _, conn, _ = self._run_scan(tmp_path)
        # start_pos_mm=100 (ack), end_mm=500, feed_rate=5 -> distance=400 -> 400/5 + 30 = 110
        assert conn.wait_calls == [110.0]

    def test_scan_error_raises_runtime_error(self, tmp_path):
        """On hardware: the error message explains what failed inside the
        agent (serial timeout, AL1342 unreachable, CSV write failure, etc).
        """
        conn = FakeGantryConnection(result={"scan_error": "serial timeout", "id": 1})
        with pytest.raises(RuntimeError, match="scan error"):
            self._run_scan(tmp_path, gantry_connection=conn)

    def test_start_scan_rejection_propagates(self, tmp_path):
        """If the agent rejects scan_start (blocked by safe_mode, or a scan
        already running there), that must surface as-is to the caller, not
        get swallowed or turned into a generic error.
        """
        conn = FakeGantryConnection(raise_on_start=SnapMotionError(0, "scan already in progress"))
        with pytest.raises(SnapMotionError, match="scan already in progress"):
            self._run_scan(tmp_path, gantry_connection=conn)

    def test_result_metadata_includes_actual_distance(self, tmp_path):
        """metadata['actual_distance_mm'] = |actual_end - actual_start|.
        On hardware: compare this against the commanded distance to
        validate the open-loop assumption. A discrepancy > ~2 mm suggests
        stepper slippage, belt stretch, or commanded distance error.
        """
        conn = FakeGantryConnection(result=_make_result(actual_start_mm=100.0, actual_end_mm=498.5))
        result, _, _ = self._run_scan(tmp_path, gantry_connection=conn)
        assert abs(result.metadata["actual_distance_mm"] - 398.5) < 0.01

    def test_result_df_loaded_from_csv(self, tmp_path):
        result, _, _ = self._run_scan(tmp_path)
        assert result.df is not None
        assert "pos_mm" in result.df.columns
        assert "distance_mm" in result.df.columns
        assert "in_ramp" in result.df.columns

    def test_csv_retrieved_to_output_dir(self, tmp_path):
        result, _, fake_client = self._run_scan(tmp_path)
        assert result.path.parent == tmp_path
        assert result.path.exists()
        assert len(fake_client.sftp.gets) >= 1

    def test_metadata_includes_scan_params(self, tmp_path):
        result, _, _ = self._run_scan(tmp_path)
        assert result.metadata["axis"] == "A1"
        assert result.metadata["end_mm"] == 500.0
        assert result.metadata["feed_rate_mm_s"] == 5.0

    def test_holds_the_gantry_arbiter_for_the_whole_traverse(self, tmp_path):
        """The transport already serialises individual commands, but nothing
        stopped another thread's move_to()/scan_with_gantry() from landing
        in between start_scan() and wait_for_scan_result(). scan() must hold
        the same arbiter GantryController.move_to() and
        GocatorScanner.scan_with_gantry() use — see laguna.robot.motion_arbiter.
        """
        from laguna.robot.motion_arbiter import MotionArbiter

        class RealishGantry:
            def __init__(self, connection):
                self.connection = connection
                self.arbiter = MotionArbiter()

        conn = FakeGantryConnection()
        gantry = RealishGantry(conn)
        held_during = {}

        def start_scan_spy(*args, **kwargs):
            held_during["start"] = gantry.arbiter.is_held
            return FakeGantryConnection.start_scan(conn, *args, **kwargs)

        conn.start_scan = start_scan_spy

        profiler = TopographicProfiler(
            gantry=gantry, pi_host="red.lab", pi_user="oak",
            pdin_port=2, al1342_host="192.168.1.251", output_dir=str(tmp_path),
        )
        fake_client = FakeSSHClient()
        with patch("paramiko.SSHClient", return_value=fake_client):
            profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)

        assert held_during["start"] is True
        assert gantry.arbiter.is_held is False, "must release once the scan completes"

    def test_two_scans_in_the_same_second_do_not_collide(self, tmp_path):
        """Auto-generated names used to have only second resolution — two
        scans saved within the same wall-clock second silently overwrote
        each other. Exercised for real, not just checked for a '%f' in the
        format string.

        Resolution is milliseconds, not microseconds, so a short sleep
        guarantees the two runs land in different milliseconds — without it
        this is a race against the boundary they need to cross.
        """
        import time

        first, _, _ = self._run_scan(tmp_path)
        time.sleep(0.005)
        second, _, _ = self._run_scan(tmp_path)
        assert first.path != second.path
        assert first.path.exists() and second.path.exists()


class TestProfilerStop:
    def test_stop_calls_stop_scan(self):
        profiler, conn = _make_profiler()
        profiler.stop()
        assert conn.stop_scan_calls == 1
