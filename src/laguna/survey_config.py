"""Build :class:`~laguna.survey.Survey` plans from a ``surveys:`` config section.

``setup_run()`` makes a single Gocator scan schedulable from a config file
exactly like a camera capture. This does the same for multi-pass surveys, so
"tile the bed every 30 minutes" is a few lines of YAML rather than a
hand-written scheduled closure::

    surveys:
      bed_tile:
        kind: tile
        instrument: gocator
        origin: [0, 0, 0]
        length_mm: 1000
        width_mm: 600
        swath_mm: auto          # or a number; auto reads the live active area
        scan_speed: 20
        interval_s: 1800        # or trigger_at: [0, 900, 1800]

Everything but the scheduling keys is the planner's own constructor arguments
(see :class:`~laguna.survey.Tile` / :class:`~laguna.survey.Traverse`), so
there is one vocabulary, not a config dialect. Unknown keys are an error: a
typo'd ``scan_speed`` silently falling back to a default speed is exactly the
kind of mistake a config-driven plan must not allow.

A scheduled survey moves the gantry, so it is still gated by the gantry's
``safe_mode`` and fence checking like any other motion; starting the run script
is the human "go". Nothing here bypasses either.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, Optional, Tuple

from .survey import Survey, Tile, Traverse

#: Keys that schedule or tune a survey rather than describe its geometry.
SCHEDULING_KEYS = ("interval_s", "trigger_at", "use_schedule")
RUNNER_KEYS = ("max_scan_speed_mm_s",)

_KINDS = {"tile": Tile, "traverse": Traverse}


def build_survey(name: str, spec: Dict[str, Any], lab: Optional[Any] = None) -> Tuple[Survey, Dict[str, Any]]:
    """Construct the planner described by one ``surveys:`` entry.

    Args:
        name: The survey's key, used only in error messages.
        spec: Its config dict (see the module docstring).
        lab: A lab whose instrument can supply ``swath_mm: auto`` live. When
            omitted — validating at setup, before anything is connected — a
            stand-in swath equal to ``width_mm`` is used just to check the
            geometry; the real plan is rebuilt with `lab` each firing.

    Returns:
        ``(survey, runner_options)`` — the planner, and the SurveyRunner
        keyword options found in `spec` (``max_scan_speed_mm_s``).

    Raises:
        ValueError: On a missing or unknown ``kind``, an unknown key, a
            missing required field, ``use_schedule`` (not supported for
            surveys), or whatever the planner's own validation rejects.
    """
    if not isinstance(spec, dict):
        raise ValueError(f"[surveys.{name}] must be a mapping, got {type(spec).__name__}")
    spec = dict(spec)
    kind = spec.pop("kind", None)
    if kind not in _KINDS:
        raise ValueError(f"[surveys.{name}] 'kind' must be one of {sorted(_KINDS)}, got {kind!r}")
    if spec.get("use_schedule"):
        raise ValueError(
            f"[surveys.{name}] 'use_schedule' isn't supported for surveys — "
            "use interval_s or trigger_at"
        )

    runner_options = {k: spec.pop(k) for k in RUNNER_KEYS if k in spec}
    for key in SCHEDULING_KEYS:
        spec.pop(key, None)

    cls = _KINDS[kind]
    allowed = {f.name for f in dataclasses.fields(cls)}
    unknown = sorted(set(spec) - allowed)
    if unknown:
        raise ValueError(
            f"[surveys.{name}] unknown key(s) {unknown} for kind {kind!r}. "
            f"Allowed: {sorted(allowed | set(RUNNER_KEYS) | set(SCHEDULING_KEYS) | {'kind'})}"
        )

    if kind == "tile" and spec.get("swath_mm") == "auto":
        spec["swath_mm"] = _auto_swath(name, spec, lab)
    try:
        return cls(**spec), runner_options
    except TypeError as exc:                      # a required field is missing
        raise ValueError(f"[surveys.{name}] {exc}") from exc


def _auto_swath(name: str, spec: Dict[str, Any], lab: Optional[Any]) -> float:
    """Resolve ``swath_mm: auto`` from the instrument's live active area."""
    if lab is None:
        if "width_mm" not in spec:
            raise ValueError(f"[surveys.{name}] 'width_mm' is required")
        return float(spec["width_mm"])               # stand-in, geometry check only
    instrument = spec.get("instrument", "gocator")
    scanner = getattr(lab, instrument, None)
    if scanner is None or not hasattr(scanner, "get_active_area"):
        raise ValueError(
            f"[surveys.{name}] swath_mm: auto needs {instrument!r} to report an active "
            "area — set swath_mm to a number instead"
        )
    return float(scanner.get_active_area()["width_mm"])


def survey_instruments(survey: Survey) -> Tuple[str, ...]:
    """The distinct instruments a survey's passes use, in first-use order."""
    seen: Dict[str, None] = {}
    for p in survey.passes():
        seen.setdefault(p.instrument)
    return tuple(seen)
