"""Tests for the laguna-picam CLI — ssh is mocked; no Pi is contacted."""

import subprocess

import pytest

from laguna.camera import picam

JPEG = b"\xff\xd8\xff\xe0fakejpegdata"


def _cp(stdout=b"", returncode=0, stderr=b""):
    return subprocess.CompletedProcess(["ssh"], returncode, stdout, stderr)


def test_resolve_host_index_and_literal():
    hosts = ["pi1.local", "pi2.local"]
    assert picam.resolve_host("2", hosts) == "pi2.local"
    assert picam.resolve_host("other.local", hosts) == "other.local"
    with pytest.raises(SystemExit):
        picam.resolve_host("3", hosts)


def test_build_ssh_command_is_batchmode_and_expands_key():
    cmd = picam.build_ssh_command("pi1", "ucrs", "~/.ssh/id", "echo hi")
    assert "BatchMode=yes" in cmd
    assert cmd[cmd.index("-i") + 1].endswith("/.ssh/id")
    assert not cmd[cmd.index("-i") + 1].startswith("~")
    assert cmd[-2:] == ["ucrs@pi1", "echo hi"]


def test_remote_tool_falls_back_to_libcamera():
    line = picam._remote_tool("still", ["-o", "-"])
    assert "rpicam" in line and "libcamera" in line and "-still" in line


def test_snap_writes_file(monkeypatch, tmp_path):
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _cp(JPEG))
    out = tmp_path / "a.jpg"
    assert picam.snap(["ssh"], str(out)) == 0
    assert out.read_bytes() == JPEG


def test_snap_rejects_non_jpeg_and_writes_nothing(monkeypatch, tmp_path):
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _cp(b"ERROR: busy", 0))
    out = tmp_path / "a.jpg"
    assert picam.snap(["ssh"], str(out)) != 0
    assert not out.exists()


def test_snap_ssh_failure(monkeypatch, tmp_path):
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _cp(b"", 255, b"denied"))
    assert picam.snap(["ssh"], str(tmp_path / "a.jpg")) == 255


def test_stream_refuses_terminal(monkeypatch):
    called = []
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: called.append(1))
    monkeypatch.setattr("sys.stdout.isatty", lambda: True, raising=False)
    assert picam.stream(["ssh"]) == 2
    assert not called


def test_main_snap_uses_config_user_and_index(monkeypatch, tmp_path):
    cfg = tmp_path / "c.yaml"
    cfg.write_text("pi_cameras:\n  hosts: [a.local, b.local]\n  ssh_user: bob\n")
    seen = {}

    def fake_run(cmd, **k):
        seen["cmd"] = cmd
        return _cp(JPEG)

    monkeypatch.setattr(subprocess, "run", fake_run)
    out = tmp_path / "x.jpg"
    assert picam.main(["--config", str(cfg), "snap", "2", "-o", str(out)]) == 0
    assert "bob@b.local" in seen["cmd"]
    assert out.read_bytes() == JPEG
