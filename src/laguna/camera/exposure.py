"""Fixed exposure / white balance for the Pi camera array.

Left alone, libcamera runs auto-exposure (AE) and auto white balance (AWB) on
every frame. The scheduled agent opens the camera fresh for each capture, so
the algorithms restart from scratch every time and only get a few frames to
settle: two shots under identical light can differ, and so can two Pis. That
makes frame-to-frame comparison unreliable, and a frame captured at the wrong
exposure cannot be retaken.

This module holds the coordinator-side half of the fix:

* :class:`ExposureControls` — one camera's fixed exposure time, gain and
  white-balance gains.
* :class:`ExposureSettings` — the ``pi_cameras.exposure:`` config section,
  resolved to "which controls does this host get?".
* :func:`save_calibration` / :func:`load_calibration` — the calibration file
  written by ``laguna-picam calibrate`` and read back at the start of a run.

Modes (``pi_cameras.exposure.mode``):

``auto``
    Historical behaviour: AE and AWB on. The default, so existing configs are
    unchanged.
``manual``
    Fixed values written in the config (global, with per-host overrides).
``calibrated``
    Per-host values read from a calibration file that an operator produced and
    confirmed under the real flume lighting before the run.

A locked mode never silently degrades to auto. A missing or incomplete
calibration file raises while the config is read — before the run starts —
and the agent refuses to capture if it cannot apply the controls.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

MODES = ("auto", "manual", "calibrated")
CALIBRATION_FORMAT = 1

# Sanity bounds, not hardware limits: they catch a typo (seconds for
# microseconds, a swapped red/blue pair) before it ruins a run's frames.
MAX_EXPOSURE_US = 10_000_000
MAX_GAIN = 64.0
MAX_COLOUR_GAIN = 32.0

#: Relative difference between requested and reported values that raises an
#: ``exposure_mismatch`` warning. The sensor quantises exposure to whole
#: sensor lines and gain to its register steps, so exact equality is never
#: expected.
MISMATCH_TOLERANCE = 0.05


@dataclass(frozen=True)
class ExposureControls:
    """Fixed exposure settings for one camera.

    Attributes:
        exposure_time_us: Shutter time in microseconds.
        analogue_gain: Sensor analogue gain (1.0 = no gain).
        awb_gains: ``(red, blue)`` white-balance gains.
    """

    exposure_time_us: int
    analogue_gain: float
    awb_gains: Tuple[float, float]

    def __post_init__(self) -> None:
        """Validate ranges, so a bad value fails at config time, not mid-run."""
        t, g = self.exposure_time_us, self.analogue_gain
        if isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t):
            raise ValueError(f"exposure_time_us must be a number, got {t!r}")
        if not 1 <= t <= MAX_EXPOSURE_US:
            raise ValueError(f"exposure_time_us must be 1..{MAX_EXPOSURE_US}, got {t}")
        if isinstance(g, bool) or not isinstance(g, (int, float)) or not math.isfinite(g):
            raise ValueError(f"analogue_gain must be a number, got {g!r}")
        if not 1.0 <= g <= MAX_GAIN:
            raise ValueError(f"analogue_gain must be 1..{MAX_GAIN}, got {g}")
        try:
            r, b = self.awb_gains
        except (TypeError, ValueError):
            raise ValueError(f"awb_gains must be [red, blue], got {self.awb_gains!r}") from None
        for name, v in (("red", r), ("blue", b)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"awb_gains {name} must be a number, got {v!r}")
            if not 0 < v <= MAX_COLOUR_GAIN:
                raise ValueError(f"awb_gains {name} must be in (0, {MAX_COLOUR_GAIN}], got {v}")

    @classmethod
    def from_dict(cls, d: Mapping[str, Any], where: str = "exposure") -> "ExposureControls":
        """Build from a config / calibration-file mapping.

        Args:
            d: Mapping with ``exposure_time_us``, ``analogue_gain``, ``awb_gains``.
            where: Config path used in error messages.

        Returns:
            The validated controls.

        Raises:
            ValueError: If a key is missing or a value is out of range.
        """
        missing = [k for k in ("exposure_time_us", "analogue_gain", "awb_gains") if k not in d]
        if missing:
            raise ValueError(f"{where}: missing {', '.join(missing)}")
        awb = d["awb_gains"]
        try:
            awb_t = (float(awb[0]), float(awb[1]))
            if len(awb) != 2:
                raise ValueError
        except (TypeError, ValueError, IndexError):
            raise ValueError(f"{where}: awb_gains must be [red, blue], got {awb!r}") from None
        try:
            return cls(
                exposure_time_us=int(round(d["exposure_time_us"])),
                analogue_gain=float(d["analogue_gain"]),
                awb_gains=awb_t,
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{where}: {exc}") from None

    def to_dict(self) -> Dict[str, Any]:
        """JSON-able form, also the payload handed to the Pi agent."""
        return {
            "exposure_time_us": int(self.exposure_time_us),
            "analogue_gain": float(self.analogue_gain),
            "awb_gains": [float(self.awb_gains[0]), float(self.awb_gains[1])],
        }


def compare_controls(
    expected: ExposureControls,
    exposure_time_us: Optional[float],
    analogue_gain: Optional[float],
    colour_gains: Optional[Iterable[float]],
    tolerance: float = MISMATCH_TOLERANCE,
) -> Optional[str]:
    """Describe how a frame's reported values differ from what was requested.

    Args:
        expected: The controls that were commanded.
        exposure_time_us: Exposure the camera reported for the frame.
        analogue_gain: Gain the camera reported.
        colour_gains: ``(red, blue)`` gains the camera reported.
        tolerance: Allowed relative difference.

    Returns:
        A one-line description of the differences, or ``None`` if the frame
        matches. Missing metadata counts as a difference: a locked capture
        whose settings cannot be confirmed is not silently trusted.
    """
    problems = []

    def _check(name: str, want: float, got: Optional[float]) -> None:
        if got is None or not math.isfinite(got):
            problems.append(f"{name} not reported (wanted {want:g})")
        elif abs(got - want) > tolerance * abs(want):
            problems.append(f"{name} {got:g} != {want:g}")

    _check("exposure_time_us", expected.exposure_time_us, exposure_time_us)
    _check("analogue_gain", expected.analogue_gain, analogue_gain)
    got_cg = list(colour_gains) if colour_gains is not None else [None, None]
    if len(got_cg) != 2:
        got_cg = [None, None]
    _check("colour_gain_red", expected.awb_gains[0], got_cg[0])
    _check("colour_gain_blue", expected.awb_gains[1], got_cg[1])
    return "; ".join(problems) if problems else None


@dataclass
class ExposureSettings:
    """Resolved ``pi_cameras.exposure:`` section.

    Attributes:
        mode: One of :data:`MODES`.
        manual: Global fixed controls (``manual`` mode).
        per_host: Per-host overrides of ``manual`` (``manual`` mode).
        calibrated: Per-host controls read from the calibration file
            (``calibrated`` mode).
        calibration_file: Path of the calibration file, if one is configured.
        warmup_s: Seconds the agent lets AE/AWB converge during calibration.
    """

    mode: str = "auto"
    manual: Optional[ExposureControls] = None
    per_host: Dict[str, ExposureControls] = field(default_factory=dict)
    calibrated: Dict[str, ExposureControls] = field(default_factory=dict)
    calibration_file: Optional[Path] = None
    warmup_s: float = 5.0

    @classmethod
    def from_config(cls, cfg: Optional[Mapping[str, Any]]) -> "ExposureSettings":
        """Parse the ``pi_cameras.exposure:`` mapping.

        Args:
            cfg: The section (``None`` or empty means ``auto``).

        Returns:
            The settings. In ``calibrated`` mode the calibration file is read
            here, so a missing or corrupt file stops the run from starting.

        Raises:
            ValueError: On an unknown mode, a bad value, or a missing /
                invalid calibration file.
        """
        cfg = dict(cfg or {})
        mode = cfg.get("mode", "auto")
        if mode not in MODES:
            raise ValueError(f"pi_cameras.exposure.mode must be one of {MODES}, got {mode!r}")
        warmup_s = float(cfg.get("warmup_s", 5.0))
        if not 1.0 <= warmup_s <= 120.0:
            raise ValueError(f"pi_cameras.exposure.warmup_s must be 1..120, got {warmup_s}")
        cal_file = cfg.get("calibration_file")
        settings = cls(
            mode=mode,
            calibration_file=Path(os.path.expanduser(cal_file)) if cal_file else None,
            warmup_s=warmup_s,
        )
        if mode == "manual":
            keys = ("exposure_time_us", "analogue_gain", "awb_gains")
            if any(k in cfg for k in keys):
                settings.manual = ExposureControls.from_dict(cfg, "pi_cameras.exposure")
            for host, hcfg in (cfg.get("per_host") or {}).items():
                merged = {k: cfg[k] for k in keys if k in cfg}
                merged.update(hcfg or {})
                settings.per_host[host] = ExposureControls.from_dict(
                    merged, f"pi_cameras.exposure.per_host.{host}"
                )
            if settings.manual is None and not settings.per_host:
                raise ValueError(
                    "pi_cameras.exposure.mode is 'manual' but no exposure_time_us / "
                    "analogue_gain / awb_gains are set"
                )
        elif mode == "calibrated":
            if settings.calibration_file is None:
                raise ValueError(
                    "pi_cameras.exposure.mode is 'calibrated' but calibration_file is not set"
                )
            settings.calibrated = load_calibration(settings.calibration_file)
        return settings

    @property
    def locked(self) -> bool:
        """True when AE/AWB are disabled and fixed values are applied."""
        return self.mode != "auto"

    def controls_for(self, host: str) -> Optional[ExposureControls]:
        """Return the fixed controls for ``host``, or ``None`` in ``auto`` mode.

        Raises:
            KeyError: In a locked mode, if ``host`` has no controls — a locked
                mode never falls back to auto for one camera.
        """
        if self.mode == "auto":
            return None
        if self.mode == "manual":
            controls = self.per_host.get(host, self.manual)
        else:
            controls = self.calibrated.get(host)
        if controls is None:
            raise KeyError(f"no {self.mode} exposure controls for host {host!r}")
        return controls

    def validate_hosts(self, hosts: Iterable[str]) -> None:
        """Check every host has controls, so a gap is found before the run.

        Raises:
            ValueError: Naming each host with no controls.
        """
        if self.mode == "auto":
            return
        missing = []
        for host in hosts:
            try:
                self.controls_for(host)
            except KeyError:
                missing.append(host)
        if missing:
            where = f" in {self.calibration_file}" if self.mode == "calibrated" else ""
            raise ValueError(
                f"pi_cameras.exposure mode '{self.mode}': no controls{where} for "
                f"{', '.join(missing)}"
            )


def save_calibration(
    path: os.PathLike | str,
    controls_by_host: Mapping[str, ExposureControls],
    *,
    overwrite: bool = False,
    notes: Optional[Mapping[str, Any]] = None,
) -> Path:
    """Write a calibration file.

    A calibration file is part of a run's record, so an existing one is never
    replaced unless ``overwrite`` is set.

    Args:
        path: Destination file.
        controls_by_host: Confirmed controls, keyed by hostname.
        overwrite: Replace an existing file.
        notes: Extra context stored under ``"notes"`` (e.g. ``converged`` flags).

    Returns:
        The path written.

    Raises:
        FileExistsError: If ``path`` exists and ``overwrite`` is false.
        ValueError: If ``controls_by_host`` is empty.
    """
    path = Path(path)
    if not controls_by_host:
        raise ValueError("refusing to write an empty calibration")
    if path.exists() and not overwrite:
        raise FileExistsError(f"{path} already exists; pass overwrite=True to replace it")
    doc = {
        "format": CALIBRATION_FORMAT,
        "created_wall": time.time(),
        "hosts": {h: c.to_dict() for h, c in controls_by_host.items()},
    }
    if notes:
        doc["notes"] = dict(notes)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Write-then-rename so a crash never leaves half a calibration where a
    # run would later read it.
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(doc, f, indent=2)
            f.write("\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
    return path


def load_calibration(path: os.PathLike | str) -> Dict[str, ExposureControls]:
    """Read a calibration file written by :func:`save_calibration`.

    Args:
        path: Calibration file.

    Returns:
        Controls keyed by hostname.

    Raises:
        ValueError: If the file is missing, unreadable, an unknown format, or
            holds an invalid entry. The message says how to produce one.
    """
    path = Path(path)
    hint = "run `laguna-picam --config <config> calibrate` first"
    if not path.is_file():
        raise ValueError(f"pi camera calibration file {path} not found ({hint})")
    try:
        doc = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"pi camera calibration file {path} unreadable: {exc}") from None
    if not isinstance(doc, dict) or doc.get("format") != CALIBRATION_FORMAT:
        raise ValueError(f"{path}: unsupported calibration format ({hint})")
    hosts = doc.get("hosts")
    if not isinstance(hosts, dict) or not hosts:
        raise ValueError(f"{path}: no hosts in calibration ({hint})")
    return {
        host: ExposureControls.from_dict(entry, f"{path}[{host}]") for host, entry in hosts.items()
    }
