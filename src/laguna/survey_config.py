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

A tile can instead be given as a region of interest, and the planner works out
how many passes it needs and how much they overlap (see
:meth:`~laguna.survey.Tile.from_roi`)::

    surveys:
      bed_tile:
        kind: tile
        roi: {x_mm: [100, 700], y_mm: [0, 2400], z_mm: 50}
        swath_mm: auto          # or a number
        min_overlap: 0.1        # smallest overlap between neighbouring swaths
        gantry_axis: X          # optional: scan along this *gantry* axis
        scan_speed: 20
        interval_s: 1800

With ``roi`` the geometry keys (``origin``, ``length_mm``, ``width_mm``,
``overlap``, ``step_axis``) come from the region and giving them is an error.

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

#: Tile arguments a ``roi:`` entry derives itself, so spelling them out is a contradiction.
_ROI_DERIVED_KEYS = ("origin", "length_mm", "width_mm", "overlap", "step_axis")
_ROI_KEYS = ("x_mm", "y_mm", "z_mm")


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
    if "roi" in spec:
        if kind != "tile":
            raise ValueError(f"[surveys.{name}] 'roi' is only valid for kind 'tile'")
        return _tile_from_roi(name, spec, lab), runner_options
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


def _tile_from_roi(name: str, spec: Dict[str, Any], lab: Optional[Any]) -> Tile:
    """Build a Tile from a ``roi:`` entry via :meth:`Tile.from_roi`.

    Args:
        name: The survey's key, for error messages.
        spec: The entry with scheduling/runner keys already removed.
        lab: As for :func:`build_survey`. Without one, a stand-in swath and
            axis are used purely to check the region's geometry.

    Raises:
        ValueError: On a malformed or incomplete ``roi``, a geometry key that
            ``roi`` already determines, an unknown key, or whatever
            ``Tile.from_roi`` rejects.
    """
    spec = dict(spec)
    roi = spec.pop("roi")
    if not isinstance(roi, dict) or set(roi) != set(_ROI_KEYS):
        raise ValueError(
            f"[surveys.{name}] 'roi' must be a mapping with exactly the keys {list(_ROI_KEYS)}"
        )
    contradictory = sorted(set(spec) & set(_ROI_DERIVED_KEYS))
    if contradictory:
        raise ValueError(
            f"[surveys.{name}] {contradictory} come from 'roi' — remove them, or drop 'roi' "
            "to give the tile geometry by hand"
        )
    tile_fields = {f.name for f in dataclasses.fields(Tile)} - set(_ROI_DERIVED_KEYS)
    planner_keys = {"min_overlap", "gantry_axis", "center_single_pass"}
    unknown = sorted(set(spec) - tile_fields - planner_keys)
    if unknown:
        raise ValueError(
            f"[surveys.{name}] unknown key(s) {unknown} for a tile with 'roi'. "
            f"Allowed: {sorted(tile_fields | planner_keys | set(RUNNER_KEYS) | set(SCHEDULING_KEYS) | {'kind', 'roi'})}"
        )

    x_mm, y_mm = roi["x_mm"], roi["y_mm"]
    swath = spec.pop("swath_mm", None)
    if swath == "auto":
        swath = None
    if lab is None:
        # Setup-time check, before anything is connected: the real swath and
        # frames are only available per firing. Any positive stand-in
        # validates the region itself.
        if swath is None:
            swath = max(abs(float(hi) - float(lo)) for lo, hi in (x_mm, y_mm)) or 1.0
        if spec.get("gantry_axis") is not None and "axis" not in spec:
            spec.pop("gantry_axis")
            spec["axis"] = "X"
    try:
        return Tile.from_roi(x_mm, y_mm, roi["z_mm"], swath_mm=swath, lab=lab, **spec)
    except (TypeError, ValueError) as exc:
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
