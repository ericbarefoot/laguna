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
from typing import Dict, Optional, Sequence

from .exposure import ExposureControls, ExposureSettings, save_calibration

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
        Dict with ``hosts`` (list), ``ssh_user`` and ``ssh_key`` (may be None),
        and ``exposure`` (the raw ``pi_cameras.exposure:`` mapping, unparsed so
        ``calibrate`` can read a config whose calibration file does not exist yet).
    """
    if not config_path:
        return {"hosts": [], "ssh_user": DEFAULT_SSH_USER, "ssh_key": None, "exposure": {}}
    from ..config import Config

    pi_cfg = Config(config_path).get("pi_cameras") or {}
    return {
        "hosts": list(pi_cfg.get("hosts", [])),
        "ssh_user": pi_cfg.get("ssh_user", DEFAULT_SSH_USER),
        "ssh_key": pi_cfg.get("ssh_key"),
        "exposure": dict(pi_cfg.get("exposure") or {}),
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


def exposure_args(controls: Optional[ExposureControls]) -> list[str]:
    """``rpicam-*`` flags that fix exposure, gain and white balance.

    Setting ``--shutter`` and ``--gain`` turns AE off and ``--awbgains`` turns
    AWB off, matching what the scheduled capture agent does.
    """
    if controls is None:
        return []
    return [
        "--shutter", str(int(controls.exposure_time_us)),
        "--gain", f"{controls.analogue_gain:g}",
        "--awbgains", f"{controls.awb_gains[0]:g},{controls.awb_gains[1]:g}",
    ]  # fmt: skip


def still_args(
    width: Optional[int],
    height: Optional[int],
    quality: int,
    controls: Optional[ExposureControls] = None,
) -> list[str]:
    """Arguments for a single JPEG written to the remote's stdout."""
    args = ["-n", "-t", "500", "-e", "jpg", "-q", str(quality)]
    if width and height:
        args += ["--width", str(width), "--height", str(height)]
    return args + exposure_args(controls) + ["-o", "-"]


def stream_args(
    width: int,
    height: int,
    framerate: int,
    quality: int,
    controls: Optional[ExposureControls] = None,
) -> list[str]:
    """Arguments for an endless MJPEG stream written to the remote's stdout."""
    return [
        "-n", "-t", "0", "-v", "0", "--codec", "mjpeg",
        "--width", str(width), "--height", str(height),
        "--framerate", str(framerate), "-q", str(quality),
        *exposure_args(controls), "-o", "-",
    ]  # fmt: skip


def resolve_controls(args: argparse.Namespace, settings: dict, host: str) -> Optional[ExposureControls]:
    """Work out the fixed controls (if any) a ``snap``/``stream`` should use.

    ``--locked`` applies the config's ``pi_cameras.exposure`` for ``host``, so a
    preview shows what a scheduled capture will produce. ``--shutter-us`` /
    ``--gain`` / ``--awbgains`` give explicit values instead.

    Raises:
        SystemExit: On inconsistent flags or a config with no controls for ``host``.
    """
    explicit = [args.shutter_us, args.gain, args.awbgains]
    if args.locked and any(v is not None for v in explicit):
        raise SystemExit("error: use either --locked or --shutter-us/--gain/--awbgains, not both")
    if args.locked:
        try:
            exposure = ExposureSettings.from_config(settings["exposure"])
            controls = exposure.controls_for(host)
        except (ValueError, KeyError) as exc:
            raise SystemExit(f"error: {exc}") from None
        if controls is None:
            raise SystemExit("error: --locked but pi_cameras.exposure.mode is 'auto'")
        return controls
    if all(v is None for v in explicit):
        return None
    if any(v is None for v in explicit):
        raise SystemExit("error: --shutter-us, --gain and --awbgains must be given together")
    try:
        red, blue = (float(v) for v in args.awbgains.split(","))
        return ExposureControls(int(args.shutter_us), float(args.gain), (red, blue))
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from None


def calibrate(settings: dict, args: argparse.Namespace, user: str, key: Optional[str]) -> int:
    """Run the operator calibration: converge AE/AWB on every Pi, confirm, save.

    Nothing is written unless every camera calibrated and the operator
    confirms, so a calibration file never covers only some of the cameras. An
    existing file is left untouched unless ``--overwrite`` is given.

    Returns:
        Process exit code (0 if a calibration was saved).
    """
    from .network import CameraArray

    exposure_cfg = settings["exposure"]
    path = args.output or exposure_cfg.get("calibration_file")
    if not path:
        print("error: no calibration file: set pi_cameras.exposure.calibration_file "
              "or pass --output", file=sys.stderr)  # fmt: skip
        return 2
    path = Path(os.path.expanduser(path))
    if path.exists() and not args.overwrite:
        print(f"error: {path} already exists; pass --overwrite to replace it", file=sys.stderr)
        return 2
    if not settings["hosts"]:
        print("error: no pi_cameras.hosts configured", file=sys.stderr)
        return 2

    array = CameraArray(hosts=settings["hosts"], ssh_user=user, ssh_key=key)
    warmup = args.warmup_s if args.warmup_s is not None else float(exposure_cfg.get("warmup_s", 5.0))
    print(f"Calibrating {len(array.hosts)} camera(s): AE/AWB warm-up {warmup:g}s each.")
    print("Lighting and scene must be as they will be during the run.")
    results = array.calibrate(warmup_s=warmup)
    preview_dir = path.parent / f"{path.stem}_previews"
    array.fetch_calibration_previews(results, preview_dir)

    ok = True
    print()
    for host in array.hosts:
        r = results[host]
        if not r.success:
            ok = False
            print(f"  {host}: FAILED — {r.error}")
            continue
        c = r.controls
        note = "" if r.converged else f"  NOT CONVERGED (spread {r.spread:.1%}) — scene still changing?"
        print(f"  {host}: exposure={c.exposure_time_us}us gain={c.analogue_gain:.3f} "
              f"awb=({c.awb_gains[0]:.3f}, {c.awb_gains[1]:.3f}){note}")  # fmt: skip
        if r.local_preview:
            print(f"      preview: {r.local_preview}")
    if not ok:
        print("\nNot saved: a calibration must cover every camera.", file=sys.stderr)
        return 1

    if not args.yes:
        if not sys.stdin.isatty():
            print("\nNot saved: confirmation needed (run interactively, or pass --yes).",
                  file=sys.stderr)  # fmt: skip
            return 1
        reply = input(f"\nCheck the previews, then save these to {path}? [y/N] ")
        if reply.strip().lower() not in ("y", "yes"):
            print("Not saved.")
            return 1

    notes: Dict[str, object] = {
        "converged": {h: results[h].converged for h in array.hosts},
        "warmup_s": warmup,
    }
    save_calibration(path, {h: results[h].controls for h in array.hosts},
                     overwrite=args.overwrite, notes=notes)  # fmt: skip
    print(f"Saved {path}")
    print("Set pi_cameras.exposure: {mode: calibrated, calibration_file: ...} to apply it.")
    return 0


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
    _add_exposure_flags(s)

    v = sub.add_parser("stream", help="write a live MJPEG stream to stdout (pipe to a player)")
    v.add_argument("camera")
    v.add_argument("--width", type=int, default=1280)
    v.add_argument("--height", type=int, default=720)
    v.add_argument("--framerate", type=int, default=15)
    v.add_argument("--quality", type=int, default=60)
    _add_exposure_flags(v)

    c = sub.add_parser(
        "calibrate",
        help="converge AE/AWB on every Pi, confirm, and save a calibration file",
    )
    c.add_argument("--output", help="calibration file (default: pi_cameras.exposure.calibration_file)")
    c.add_argument("--warmup-s", type=float, help="AE/AWB warm-up seconds (default: from config, else 5)")
    c.add_argument("--overwrite", action="store_true", help="replace an existing calibration file")
    c.add_argument("-y", "--yes", action="store_true", help="save without the interactive confirmation")
    return p


def _add_exposure_flags(sp: argparse.ArgumentParser) -> None:
    """Add the fixed-exposure flags shared by ``snap`` and ``stream``."""
    sp.add_argument("--locked", action="store_true",
                    help="use pi_cameras.exposure from the config for this camera")  # fmt: skip
    sp.add_argument("--shutter-us", type=int, help="fixed exposure time (microseconds)")
    sp.add_argument("--gain", type=float, help="fixed analogue gain")
    sp.add_argument("--awbgains", help="fixed white-balance gains: RED,BLUE")


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

    user = args.user or settings["ssh_user"]
    key = args.key or settings["ssh_key"]

    if args.action == "calibrate":
        return calibrate(settings, args, user, key)

    host = resolve_host(args.camera, settings["hosts"])
    controls = resolve_controls(args, settings, host)

    if args.action == "snap":
        remote = _remote_tool("still", still_args(args.width, args.height, args.quality, controls))
        out = args.output or str(default_snap_path(host))
        return snap(build_ssh_command(host, user, key, remote), out)

    remote = _remote_tool(
        "vid", stream_args(args.width, args.height, args.framerate, args.quality, controls)
    )
    return stream(build_ssh_command(host, user, key, remote))


if __name__ == "__main__":
    sys.exit(main())
