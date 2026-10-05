"""``laguna-picam`` — one-off still capture and live stream from a Pi camera.

Meant to be run ON the laguna PC, usually by a remote client over SSH::

    client --ssh--> laguna --ssh--> pi        (commands)
    client <-------- laguna <------ pi        (JPEG / MJPEG bytes, via the pipes)

No port is opened anywhere: the image bytes ride the SSH sessions' stdout.
``scripts/picam-remote.sh`` is the client-side wrapper that does the first hop.

This is deliberately separate from ``CameraArray``: it is for a human looking
at the apparatus, not for data collection. It never starts motion, and it
shares the Pi camera with the scheduled capture agent — see ``_EXCLUSIVE_NOTE``.
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

# The Pi's libcamera pipeline admits one client at a time. A live view held
# open when a scheduled capture fires makes that capture fail, and a missed
# frame cannot be re-taken, so say so every time rather than only in the docs.
_EXCLUSIVE_NOTE = (
    "note: the Pi camera is exclusive — do not leave a stream open when a "
    "scheduled capture is due, or that capture will fail."
)

DEFAULT_SSH_USER = "ucrs"
_JPEG_MAGIC = b"\xff\xd8"


def load_pi_settings(config_path: Optional[str]) -> dict:
    """Read the ``pi_cameras:`` section of a laguna config, if one is given.

    Args:
        config_path: YAML config path, or None to use built-in defaults.

    Returns:
        Dict with ``hosts`` (list), ``ssh_user`` and ``ssh_key`` (may be None).
    """
    if not config_path:
        return {"hosts": [], "ssh_user": DEFAULT_SSH_USER, "ssh_key": None}
    from ..config import Config

    pi_cfg = Config(config_path).get("pi_cameras") or {}
    return {
        "hosts": list(pi_cfg.get("hosts", [])),
        "ssh_user": pi_cfg.get("ssh_user", DEFAULT_SSH_USER),
        "ssh_key": pi_cfg.get("ssh_key"),
    }


def resolve_host(name: str, hosts: Sequence[str]) -> str:
    """Map a 1-based index into the configured hosts, or pass a hostname through.

    Args:
        name: ``"2"`` for the second configured host, or a literal hostname/IP.
        hosts: Configured host list.

    Returns:
        The hostname to connect to.

    Raises:
        SystemExit: If an index is out of range.
    """
    if name.isdigit():
        idx = int(name)
        if not 1 <= idx <= len(hosts):
            raise SystemExit(f"error: no configured camera #{idx} (have {len(hosts)})")
        return hosts[idx - 1]
    return name


def _remote_tool(tool: str, args: Sequence[str]) -> str:
    """Build a remote shell line using ``rpicam-*`` or the older ``libcamera-*``.

    Pi OS Bookworm renamed libcamera-still/-vid to rpicam-*; fleets mix both.
    """
    return (
        "T=rpicam; command -v rpicam-still >/dev/null 2>&1 || T=libcamera; "
        f"exec ${{T}}-{tool} " + " ".join(shlex.quote(a) for a in args)
    )


def build_ssh_command(host: str, user: str, ssh_key: Optional[str], remote: str) -> list[str]:
    """Build the laguna → pi ssh invocation.

    ``BatchMode`` makes a missing/locked key fail fast instead of prompting on
    a terminal whose stdout is carrying binary image data.
    """
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]
    if ssh_key:
        cmd += ["-i", os.path.expanduser(ssh_key)]
    return cmd + [f"{user}@{host}", remote]


def still_args(width: Optional[int], height: Optional[int], quality: int) -> list[str]:
    """Arguments for a single JPEG written to the remote's stdout."""
    args = ["-n", "-t", "500", "-e", "jpg", "-q", str(quality)]
    if width and height:
        args += ["--width", str(width), "--height", str(height)]
    return args + ["-o", "-"]


def stream_args(width: int, height: int, framerate: int, quality: int) -> list[str]:
    """Arguments for an endless MJPEG stream written to the remote's stdout."""
    return [
        "-n", "-t", "0", "-v", "0", "--codec", "mjpeg",
        "--width", str(width), "--height", str(height),
        "--framerate", str(framerate), "-q", str(quality), "-o", "-",
    ]  # fmt: skip


def default_snap_path(host: str) -> Path:
    """Timestamped file name in the current directory."""
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path(f"{host}_{ts}.jpg")


def snap(cmd: list[str], out: str) -> int:
    """Run the ssh command, validate that a JPEG came back, write it to ``out``.

    Args:
        cmd: Full ssh command line.
        out: Destination path, or ``"-"`` for stdout.

    Returns:
        Process exit code (0 on success).
    """
    proc = subprocess.run(cmd, capture_output=True, timeout=60)
    if proc.returncode != 0 or not proc.stdout.startswith(_JPEG_MAGIC):
        sys.stderr.write(proc.stderr.decode(errors="replace"))
        print(
            f"error: capture failed (ssh exit {proc.returncode}, "
            f"{len(proc.stdout)} bytes). Camera busy, or laguna → pi key needs a passphrase?",
            file=sys.stderr,
        )
        return proc.returncode or 1
    if out == "-":
        sys.stdout.buffer.write(proc.stdout)
        sys.stdout.buffer.flush()
    else:
        Path(out).write_bytes(proc.stdout)
        print(f"saved {out} ({len(proc.stdout)} bytes)", file=sys.stderr)
    return 0


def stream(cmd: list[str]) -> int:
    """Pass the remote MJPEG stream straight through to this process's stdout."""
    if sys.stdout.isatty():
        print("error: refusing to write a binary stream to a terminal; pipe it "
              "to a player (e.g. ... | ffplay -f mjpeg -i -)", file=sys.stderr)  # fmt: skip
        return 2
    print(_EXCLUSIVE_NOTE, file=sys.stderr)
    try:
        return subprocess.run(cmd, stdin=subprocess.DEVNULL).returncode
    except KeyboardInterrupt:
        return 0


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="laguna-picam", description=__doc__.split("\n\n")[0])
    p.add_argument("--config", default=os.environ.get("LAGUNA_CONFIG"),
                   help="laguna YAML config (default: $LAGUNA_CONFIG)")  # fmt: skip
    p.add_argument("--user", help="ssh user on the Pi (default: from config, else ucrs)")
    p.add_argument("--key", help="ssh private key for laguna → pi (default: from config)")
    sub = p.add_subparsers(dest="action", required=True)

    sub.add_parser("list", help="show configured cameras")

    s = sub.add_parser("snap", help="capture one JPEG")
    s.add_argument("camera", help="hostname, or 1-based index into pi_cameras.hosts")
    s.add_argument("-o", "--output", help="file to write; '-' for stdout (default: ./<host>_<utc>.jpg)")
    s.add_argument("--width", type=int)
    s.add_argument("--height", type=int)
    s.add_argument("--quality", type=int, default=95)

    v = sub.add_parser("stream", help="write a live MJPEG stream to stdout (pipe to a player)")
    v.add_argument("camera")
    v.add_argument("--width", type=int, default=1280)
    v.add_argument("--height", type=int, default=720)
    v.add_argument("--framerate", type=int, default=15)
    v.add_argument("--quality", type=int, default=60)
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point (``laguna-picam``)."""
    args = _parser().parse_args(argv)
    settings = load_pi_settings(args.config)

    if args.action == "list":
        for i, h in enumerate(settings["hosts"], 1):
            print(f"{i}\t{h}")
        if not settings["hosts"]:
            print("no pi_cameras.hosts configured (pass --config or set $LAGUNA_CONFIG)", file=sys.stderr)
        return 0

    host = resolve_host(args.camera, settings["hosts"])
    user = args.user or settings["ssh_user"]
    key = args.key or settings["ssh_key"]

    if args.action == "snap":
        remote = _remote_tool("still", still_args(args.width, args.height, args.quality))
        out = args.output or str(default_snap_path(host))
        return snap(build_ssh_command(host, user, key, remote), out)

    remote = _remote_tool("vid", stream_args(args.width, args.height, args.framerate, args.quality))
    return stream(build_ssh_command(host, user, key, remote))


if __name__ == "__main__":
    sys.exit(main())
