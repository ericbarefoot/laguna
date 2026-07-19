"""Tests for TopographicProfiler and ProfileResult.

All offline — no real SSH/SFTP/paramiko involved. The SSH session and SFTP
transfers are replaced with fake objects that simulate the scan_runner.py
wire protocol. Each test documents one assumption about the orchestration;
a failure on hardware reveals which step in the sequence went wrong.
"""

import csv
import json
import time
from pathlib import Path
from typing import List
from unittest.mock import MagicMock, patch

import pytest

from laguna.robot.macron.profiler import (
    REMOTE_SCAN_RUNNER,
    SCAN_RUNNER_SCRIPT,
    ProfileResult,
    TopographicProfiler,
)


# ---------------------------------------------------------------------------
# Sanity checks on the script paths (catch deployment regressions)
# ---------------------------------------------------------------------------


class TestScriptPaths:
    def test_scan_runner_script_exists(self):
        """scan_runner.py must exist at its computed path so SFTP can deploy it.
        If this test fails, the file was moved without updating SCAN_RUNNER_SCRIPT.
        """
        assert SCAN_RUNNER_SCRIPT.exists(), (
            f"scan_runner.py not found at {SCAN_RUNNER_SCRIPT}; "
            "update SCAN_RUNNER_SCRIPT in profiler.py if the file was moved."
        )

    def test_scan_runner_script_is_python(self):
        assert SCAN_RUNNER_SCRIPT.suffix == ".py"

    def test_remote_path_is_absolute(self):
        assert REMOTE_SCAN_RUNNER.startswith("/")

    def test_scan_runner_declares_main(self):
        """scan_runner.py must have a main() entry point and __main__ guard."""
        text = SCAN_RUNNER_SCRIPT.read_text()
        assert "def main()" in text, "scan_runner.py must define main()"
        assert '__name__ == "__main__"' in text, "scan_runner.py must have __main__ guard"

    def test_scan_runner_emits_ready(self):
        """scan_runner.py must emit {ready: true} in its wire protocol."""
        text = SCAN_RUNNER_SCRIPT.read_text()
        assert '"ready"' in text or "'ready'" in text

    def test_scan_runner_emits_done(self):
        """scan_runner.py must emit {done: true} in its wire protocol."""
        text = SCAN_RUNNER_SCRIPT.read_text()
        assert '"done"' in text or "'done'" in text


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
            serial_device="/dev/ttyUSB0",
        )
        kwargs.update(overrides)
        return TopographicProfiler(**kwargs)

    def test_requires_serial_device(self):
        """serial_device is required — empty string should raise ValueError.
        On hardware: supply the full /dev/serial/by-id/... path to avoid
        ambiguity if the USB adapter re-enumerates as a different ttyUSBN.
        """
        with pytest.raises(ValueError, match="serial_device"):
            TopographicProfiler(
                gantry=MagicMock(),
                pi_host="red.lab",
                pi_user="oak",
                serial_device="",
            )

    def test_stores_config(self):
        p = self._make(pdin_port=3, od2000_topic="laguna/od2000", output_dir="/data")
        assert p._pdin_port == 3
        assert p._od2000_topic == "laguna/od2000"
        assert p._output_dir == Path("/data")


# ---------------------------------------------------------------------------
# Fake SSH + SFTP infrastructure for scan() tests
# ---------------------------------------------------------------------------


def _make_ready_msg(**overrides):
    msg = {"ready": True, "start_pos_mm": 100.0, "accel_mm_s2": 10.0, "decel_mm_s2": 10.0}
    msg.update(overrides)
    return msg


def _make_done_msg(**overrides):
    msg = {
        "done": True,
        "csv_path": "/tmp/profile_test.csv",
        "meta_path": "/tmp/profile_test_meta.json",
        "actual_start_mm": 100.1,
        "actual_end_mm": 500.5,
        "samples": 850,
        "achieved_rate_hz": 10.0,
    }
    msg.update(overrides)
    return msg


class FakeChannel:
    """Simulates a paramiko exec_command stdout channel."""

    def __init__(self, messages: List[dict]):
        self._queue = [(json.dumps(m) + "\n").encode("utf-8") for m in messages]
        self._buf = b""
        self._stderr = []

    def recv_ready(self):
        return bool(self._queue)

    def recv(self, n):
        if self._queue:
            chunk = self._queue.pop(0)
            self._buf += chunk
        data = self._buf[:n]
        self._buf = self._buf[n:]
        return data

    def recv_stderr_ready(self):
        return bool(self._stderr)

    def recv_stderr(self, n):
        return self._stderr.pop(0) if self._stderr else b""

    def exit_status_ready(self):
        return not self._queue and not self._buf


class FakeChannelHanging:
    """A channel that never produces data and never reports exit.

    Used for timeout tests: the loop keeps spinning until the deadline fires.
    """

    def recv_ready(self): return False
    def recv(self, n): return b""
    def recv_stderr_ready(self): return False
    def recv_stderr(self, n): return b""
    def exit_status_ready(self): return False


class FakeSftp:
    """Simulates paramiko SFTP that creates a local stub CSV on get()."""

    def __init__(self, tmp_path):
        self._tmp = tmp_path
        self.puts = []
        self.gets = []

    def put(self, local, remote):
        self.puts.append((local, remote))

    def get(self, remote, local):
        self.gets.append((remote, local))
        # Write a minimal stub CSV so pd.read_csv works
        Path(local).write_text(
            "wall_time_unix,wall_time_iso,pos_mm,distance_nm,distance_mm,q1,q2,in_ramp\n"
            "1000.0,2026-01-01T00:00:00Z,100.0,200000000,200.0,0,0,0\n"
        )

    def close(self):
        pass


class FakeSSHClient:
    """Simulates paramiko.SSHClient with scripted exec_command responses."""

    def __init__(self, messages, tmp_path):
        self._messages = messages
        self._tmp = tmp_path
        self.sftp = FakeSftp(tmp_path)
        self.connected = False
        self.closed = False

    def set_missing_host_key_policy(self, policy):
        pass

    def connect(self, host, **kwargs):
        self.connected = True

    def open_sftp(self):
        return self.sftp

    def exec_command(self, cmd):
        chan = FakeChannel(self._messages)
        stdin_mock = MagicMock()
        stdout_mock = MagicMock()
        stdout_mock.channel = chan
        return stdin_mock, stdout_mock, MagicMock()

    def close(self):
        self.closed = True


def _make_profiler(gantry, tmp_path, messages):
    """Build a TopographicProfiler with a fake SSH client injected."""
    profiler = TopographicProfiler(
        gantry=gantry,
        pi_host="red.lab",
        pi_user="oak",
        serial_device="/dev/ttyUSB0",
        pdin_port=1,
        output_dir=str(tmp_path),
    )
    fake_client = FakeSSHClient(messages, tmp_path)
    return profiler, fake_client


# ---------------------------------------------------------------------------
# scan() orchestration tests
# ---------------------------------------------------------------------------


class TestProfilerScan:
    def _run_scan(self, tmp_path, messages, gantry=None):
        if gantry is None:
            gantry = MagicMock()
            gantry.connection.disconnect = MagicMock()
            gantry.connection.connect = MagicMock()

        profiler, fake_client = _make_profiler(gantry, tmp_path, messages)

        # scan() imports paramiko locally, so patch the class in the real module.
        # pandas is a core dep and is installed; no need to mock it.
        with patch("paramiko.SSHClient", return_value=fake_client):
            return profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)

    def test_happy_path_returns_profile_result(self, tmp_path):
        """Full scan() with ready → done produces a ProfileResult."""
        result = self._run_scan(tmp_path, [_make_ready_msg(), _make_done_msg()])
        assert isinstance(result, ProfileResult)
        assert result.metadata["done"] is True

    def test_gantry_disconnected_before_scan(self, tmp_path):
        """gantry.connection.disconnect() must be called before deploying scan_runner.
        On hardware: if this isn't called, scan_runner.py will fail to open
        the serial port (busy, held by gantry_agent.py).
        """
        gantry = MagicMock()
        gantry.connection.disconnect = MagicMock()
        gantry.connection.connect = MagicMock()
        self._run_scan(tmp_path, [_make_ready_msg(), _make_done_msg()], gantry=gantry)
        gantry.connection.disconnect.assert_called_once()

    def test_gantry_reconnected_after_scan(self, tmp_path):
        """gantry.connection.connect() must be called after scan completes.
        On hardware: if the gantry is unresponsive after a scan, check that
        connect() succeeded — it redeploys gantry_agent.py.
        """
        gantry = MagicMock()
        gantry.connection.disconnect = MagicMock()
        gantry.connection.connect = MagicMock()
        self._run_scan(tmp_path, [_make_ready_msg(), _make_done_msg()], gantry=gantry)
        gantry.connection.connect.assert_called_once()

    def test_scan_runner_is_sftp_deployed(self, tmp_path):
        """scan_runner.py is SFTP'd to the Pi before exec_command."""
        profiler, fake_client = _make_profiler(
            MagicMock(), tmp_path, [_make_ready_msg(), _make_done_msg()]
        )
        with patch("paramiko.SSHClient", return_value=fake_client):
            profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)

        remote_paths = [r for _, r in fake_client.sftp.puts]
        assert REMOTE_SCAN_RUNNER in remote_paths

    def test_result_metadata_includes_actual_distance(self, tmp_path):
        """metadata['actual_distance_mm'] = |actual_end - actual_start|.
        On hardware: compare this against the commanded distance to validate
        the open-loop assumption. A discrepancy > ~2 mm suggests stepper
        slippage, belt stretch, or commanded distance error.
        """
        done = _make_done_msg(actual_start_mm=100.0, actual_end_mm=498.5)
        result = self._run_scan(tmp_path, [_make_ready_msg(), done])
        assert abs(result.metadata["actual_distance_mm"] - 398.5) < 0.01

    def test_result_df_loaded_from_csv(self, tmp_path):
        """ProfileResult.df is a pandas DataFrame loaded from the retrieved CSV."""
        result = self._run_scan(tmp_path, [_make_ready_msg(), _make_done_msg()])
        assert result.df is not None
        assert "pos_mm" in result.df.columns
        assert "distance_mm" in result.df.columns
        assert "in_ramp" in result.df.columns

    def test_error_from_scan_runner_raises(self, tmp_path):
        """If scan_runner.py emits {error: '...'}, scan() raises RuntimeError.
        On hardware: error message will explain what failed (serial open, MQTT,
        BLC command, etc.).
        """
        error_msg = {"error": "Failed to open serial port /dev/ttyUSB0: [Errno 13] Permission denied"}
        with pytest.raises(RuntimeError, match="scan_runner error"):
            self._run_scan(tmp_path, [error_msg])

    def test_scan_runner_cli_includes_required_args(self, tmp_path):
        """The exec_command string must include all required scan_runner.py args.
        If scan_runner.py silently uses wrong defaults because an arg was omitted,
        the resulting profile will be silently wrong.
        """
        profiler, fake_client = _make_profiler(
            MagicMock(), tmp_path, [_make_ready_msg(), _make_done_msg()]
        )
        executed_cmds = []
        original_exec = fake_client.exec_command

        def capture_exec(cmd):
            executed_cmds.append(cmd)
            return original_exec(cmd)

        fake_client.exec_command = capture_exec

        with patch("paramiko.SSHClient", return_value=fake_client):
            profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)

        assert executed_cmds, "exec_command was never called"
        cmd = executed_cmds[0]
        assert "--serial-port" in cmd
        assert "--axis" in cmd and "A1" in cmd
        assert "--end-mm" in cmd and "500" in cmd
        assert "--feed-rate-mm-s" in cmd and "5" in cmd
        assert "--pdin-port" in cmd
        assert "--output" in cmd

    def test_gantry_reconnected_even_if_scan_fails(self, tmp_path):
        """Gantry must reconnect even if scan_runner reports an error.
        On hardware: if the gantry is dead after a failed scan, this check
        identifies whether reconnect() is being skipped on the error path.
        """
        gantry = MagicMock()
        gantry.connection.disconnect = MagicMock()
        gantry.connection.connect = MagicMock()
        error_msg = {"error": "simulated hardware failure"}
        with pytest.raises(RuntimeError):
            self._run_scan(tmp_path, [error_msg], gantry=gantry)
        gantry.connection.connect.assert_called_once()

    def test_csv_retrieved_to_output_dir(self, tmp_path):
        """The result CSV must be retrieved from the Pi and saved to output_dir."""
        result = self._run_scan(tmp_path, [_make_ready_msg(), _make_done_msg()])
        assert result.path.parent == tmp_path
        assert result.path.exists()

    def test_metadata_includes_scan_params(self, tmp_path):
        """metadata must include axis, end_mm, and feed_rate_mm_s from the call."""
        result = self._run_scan(tmp_path, [_make_ready_msg(), _make_done_msg()])
        assert result.metadata["axis"] == "A1"
        assert result.metadata["end_mm"] == 500.0
        assert result.metadata["feed_rate_mm_s"] == 5.0


# ---------------------------------------------------------------------------
# _read_json_line — timeout and protocol edge cases
# ---------------------------------------------------------------------------


class TestReadJsonLine:
    def _make_profiler(self):
        return TopographicProfiler(
            gantry=MagicMock(),
            pi_host="red.lab",
            pi_user="oak",
            serial_device="/dev/ttyUSB0",
        )

    def test_reads_ready_message(self):
        profiler = self._make_profiler()
        chan = FakeChannel([_make_ready_msg()])
        msg = profiler._read_json_line(chan, timeout=2.0, label="ready")
        assert msg["ready"] is True

    def test_reads_done_message(self):
        profiler = self._make_profiler()
        chan = FakeChannel([_make_done_msg()])
        msg = profiler._read_json_line(chan, timeout=2.0, label="done")
        assert msg["done"] is True

    def test_skips_non_json_lines(self):
        """Diagnostic output (stderr-style text) on stdout is skipped until
        a valid JSON line arrives.
        On hardware: if scan_runner.py prints anything to stdout before
        the {ready} message (e.g. debug logging), this test confirms the
        profiler skips it rather than crashing.
        """
        profiler = self._make_profiler()
        # Inject a non-JSON line before the real message
        chan = FakeChannel([_make_ready_msg()])
        chan._queue.insert(0, b"DEBUG: opened serial port\n")
        msg = profiler._read_json_line(chan, timeout=2.0, label="ready")
        assert msg.get("ready") is True

    def test_timeout_raises_runtime_error(self):
        """If scan_runner.py never sends {ready}, RuntimeError is raised after timeout.
        On hardware: check scan_runner.py stderr for the actual failure reason
        (serial port, MQTT broker, BLC command error).
        """
        profiler = self._make_profiler()
        # FakeChannelHanging never exits and never produces data → hits deadline
        chan = FakeChannelHanging()
        with pytest.raises(RuntimeError, match="Timed out"):
            profiler._read_json_line(chan, timeout=0.05, label="ready")

    def test_error_message_propagates(self):
        """{error: '...'} from scan_runner.py is returned as-is (not raised here).
        The caller (scan()) decides to raise RuntimeError.
        """
        profiler = self._make_profiler()
        error_msg = {"error": "Failed to open serial port"}
        chan = FakeChannel([error_msg])
        msg = profiler._read_json_line(chan, timeout=2.0, label="done")
        assert "error" in msg
