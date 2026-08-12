"""Quick-look plots for scans, profiles, and planned trajectories.

Two entry points, both experiment-frame only, both returning ``(fig, axes)``:

:func:`plot_acquisition` — dispatches on input type (a Gocator
:class:`~laguna.scanner.pointcloud.SurfaceScan`, a rangefinder
:class:`~laguna.robot.macron.profiler.ProfileResult`, or the DataFrame
:func:`~laguna.robot.macron.profiler.orient_profile` returns) and produces a
four-panel figure: XY/XZ/YZ projections of the data, plus a panel of the
data itself (a Z heatmap for a scan, height vs. travel distance for a
profile).

:func:`plot_trajectory` — the same XY/XZ/YZ layout, but for a **planned**
:class:`~laguna.survey.Survey`/list of :class:`~laguna.survey.Pass` instead
of acquired data — a start/end line per pass, for previewing motion before
running it.

**Landmarks, not fences.** Both accept :class:`Landmark` boxes for visual
reference (flume walls, a known obstacle) — plain experiment-coordinate
boxes with no relation to
:class:`~laguna.robot.macron.fences.BoxFence`/``TrajectoryChecker``. Fence
geometry is defined relative to the gantry's *commanded* point, not the
physical extent of the arm/carriage that could actually strike something —
transforming it correctly would need the machine's physical geometry (arm
length, carriage envelope), which nothing in this codebase models yet. See
``docs/guides/gantry-geometry-visualization-plan.md`` for that. Give a
Landmark's bounds directly in experiment coordinates instead — e.g.
measured/eyeballed flume walls, a physical obstacle's footprint.

**Experiment frame only.** `SurfaceScan`/profile data needs to already be
oriented (:func:`~laguna.frames.orient_scan` /
:func:`~laguna.robot.macron.profiler.orient_profile`); `Survey`/`Pass` data
already is, by construction (see ``laguna.survey``'s module docstring).

Optional dependency — install with ``pip install 'laguna[viz]'``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Iterable, Optional, Sequence, Tuple, Union

import numpy as np

if TYPE_CHECKING:
    import pandas as pd
    from matplotlib.figure import Figure

    from .robot.macron.profiler import ProfileResult
    from .scanner.pointcloud import SurfaceScan
    from .survey import Pass, Survey

logger = logging.getLogger(__name__)

#: (row index, col index, row axis name, col axis name) for each projection.
_PLANES: dict = {
    "xy": (0, 1, "X", "Y"),
    "xz": (0, 2, "X", "Z"),
    "yz": (1, 2, "Y", "Z"),
}


@dataclass
class Landmark:
    """A labeled box, in experiment coordinates, for visual reference only.

    Not a safety fence and not checked against anything — see this module's
    docstring for why fence geometry (gantry-frame, relative to the
    commanded point) can't be dropped in directly. Give a Landmark's bounds
    as you'd actually measure or eyeball them in the flume.

    Attributes:
        name: Label, for a future legend — not currently rendered.
        x_min, x_max, y_min, y_max, z_min, z_max: Box bounds, experiment mm.
    """

    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


def plot_acquisition(
    data: Union["SurfaceScan", "ProfileResult", "pd.DataFrame"],
    *,
    landmarks: Optional[Sequence[Landmark]] = None,
    fig: Optional["Figure"] = None,
    title: Optional[str] = None,
) -> Tuple["Figure", Any]:
    """Plot a scan or profile's XY/XZ/YZ footprint against landmarks, plus its data.

    Args:
        data: A SurfaceScan (from ``frames.orient_scan()`` — a raw,
            un-oriented one plots but logs a warning, since its coordinates
            aren't experiment-frame and the landmark overlay would be
            meaningless), a ProfileResult, or a DataFrame — the latter two
            need ``experiment_x_mm``/``y_mm``/``z_mm`` columns, i.e. already
            run through ``orient_profile()``.
        landmarks: Optional boxes to draw on each projection — see
            :class:`Landmark`.
        fig: Existing Figure to plot into (its own ``subplots(2, 2)`` — any
            existing content is replaced). Omit to create a new one.
        title: Figure title. Defaults to a summary of `data`.

    Returns:
        ``(fig, axes)`` — `fig` is `fig` if given, otherwise newly created;
        `axes` is the 2x2 array of Axes (``[[xy, xz], [yz, data]]``) for
        adjusting limits, titles, or anything else after the fact.

    Raises:
        ImportError: If matplotlib isn't installed.
        TypeError: If `data` isn't a recognized type.
        ValueError: If a DataFrame/ProfileResult lacks the experiment_x_mm/
            y_mm/z_mm columns ``orient_profile()`` produces.
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise ImportError(
            "laguna.viz.plot_acquisition() needs matplotlib, an optional "
            "dependency: pip install 'laguna[viz]'"
        ) from e

    points, panel = _extract(data)

    if fig is None:
        fig = plt.figure(figsize=(11, 9))
    axes = fig.subplots(2, 2)
    ax_xy, ax_xz = axes[0]
    ax_yz, ax_data = axes[1]

    _plot_plane(ax_xy, points, landmarks, "xy")
    _plot_plane(ax_xz, points, landmarks, "xz")
    _plot_plane(ax_yz, points, landmarks, "yz")
    panel(ax_data)

    fig.suptitle(title or _default_title(data, points))
    fig.tight_layout()
    return fig, axes


def plot_trajectory(
    data: Union["Survey", Iterable["Pass"]],
    *,
    landmarks: Optional[Sequence[Landmark]] = None,
    fig: Optional["Figure"] = None,
    title: Optional[str] = None,
) -> Tuple["Figure", Any]:
    """Preview a planned Survey/Pass list's XY/XZ/YZ motion, against landmarks.

    Skeleton: draws each pass as a start->end line, colored by order, plus
    a text summary panel (index, instrument, axis, length). Doesn't yet draw
    the machine's own physical geometry (carriage/arm swept volume) — see
    ``docs/guides/gantry-geometry-visualization-plan.md`` — nor accept raw
    G-code text (gantry-frame, not experiment-frame; out of scope for the
    same reason fences are, see this module's docstring).

    Args:
        data: A Survey, or any iterable of Pass (both already
            experiment-frame — see ``laguna.survey``'s module docstring).
        landmarks: Optional boxes to draw on each projection — see
            :class:`Landmark`.
        fig: Existing Figure to plot into. Omit to create a new one.
        title: Figure title. Defaults to a pass-count summary.

    Returns:
        ``(fig, axes)`` — see :func:`plot_acquisition`.

    Raises:
        ImportError: If matplotlib isn't installed.
        ValueError: If `data` has no passes.
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib import colormaps
    except ImportError as e:
        raise ImportError(
            "laguna.viz.plot_trajectory() needs matplotlib, an optional "
            "dependency: pip install 'laguna[viz]'"
        ) from e

    passes = list(data)
    if not passes:
        raise ValueError("plot_trajectory(): no passes to plot")

    if fig is None:
        fig = plt.figure(figsize=(11, 9))
    axes = fig.subplots(2, 2)
    ax_xy, ax_xz = axes[0]
    ax_yz, ax_data = axes[1]

    colors = colormaps["viridis"](np.linspace(0, 1, len(passes)))
    _plot_trajectory_plane(ax_xy, passes, colors, landmarks, "xy")
    _plot_trajectory_plane(ax_xz, passes, colors, landmarks, "xz")
    _plot_trajectory_plane(ax_yz, passes, colors, landmarks, "yz")
    _plot_trajectory_panel(ax_data, passes)

    fig.suptitle(title or f"planned trajectory — {len(passes)} passes")
    fig.tight_layout()
    return fig, axes


# ---------------------------------------------------------------------------
# Per-input-type extraction (plot_acquisition)
# ---------------------------------------------------------------------------


def _extract(data: Any):
    """Return ((N, 3) experiment-frame points, panel(ax) -> None)."""
    from .robot.macron.profiler import ProfileResult
    from .scanner.pointcloud import SurfaceScan

    if isinstance(data, SurfaceScan):
        if not data.mounting.is_identity:
            logger.warning(
                "plot_acquisition(): this SurfaceScan's mounting is not "
                "identity — it doesn't look like it's been run through "
                "frames.orient_scan() yet, so its coordinates are not "
                "experiment-frame and the landmark overlay will not line up "
                "with the data"
            )
        points = data.to_points(drop_invalid=True, dtype=np.float64)
        return points, lambda ax: _plot_surface_panel(ax, data)

    if isinstance(data, ProfileResult):
        return _extract_profile(data.df)

    try:
        import pandas as pd
    except ImportError:
        pd = None
    if pd is not None and isinstance(data, pd.DataFrame):
        return _extract_profile(data)

    raise TypeError(
        f"plot_acquisition() doesn't know how to handle {type(data)!r} — "
        "expected a SurfaceScan, ProfileResult, or DataFrame"
    )


def _extract_profile(df: Optional["pd.DataFrame"]):
    if df is None:
        raise ValueError("plot_acquisition(): no DataFrame available on this ProfileResult")
    required = ("experiment_x_mm", "experiment_y_mm", "experiment_z_mm")
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(
            f"plot_acquisition() needs {missing} — run this through "
            "laguna.robot.macron.profiler.orient_profile() first"
        )
    points = df[list(required)].to_numpy(dtype=float)
    return points, lambda ax: _plot_profile_panel(ax, df)


# ---------------------------------------------------------------------------
# XY/XZ/YZ projections
# ---------------------------------------------------------------------------


def _plot_plane(
    ax: Any,
    points: np.ndarray,
    landmarks: Optional[Sequence[Landmark]],
    plane: str,
) -> None:
    i, j, label_i, label_j = _PLANES[plane]
    ax.scatter(points[:, i], points[:, j], s=2, alpha=0.4, color="tab:blue", label="data")
    _draw_landmarks(ax, landmarks, plane)
    ax.set_xlabel(f"{label_i} (mm)")
    ax.set_ylabel(f"{label_j} (mm)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)


def _plot_trajectory_plane(
    ax: Any,
    passes: Sequence["Pass"],
    colors: np.ndarray,
    landmarks: Optional[Sequence[Landmark]],
    plane: str,
) -> None:
    i, j, label_i, label_j = _PLANES[plane]
    for p, color in zip(passes, colors):
        start, end = p.start, p.end
        ax.plot(
            [start[i], end[i]], [start[j], end[j]],
            color=color, linewidth=1.5, marker="o", markersize=3,
        )
    _draw_landmarks(ax, landmarks, plane)
    ax.set_xlabel(f"{label_i} (mm)")
    ax.set_ylabel(f"{label_j} (mm)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)


def _draw_landmarks(ax: Any, landmarks: Optional[Sequence[Landmark]], plane: str) -> None:
    if not landmarks:
        return
    from matplotlib.patches import Rectangle

    _, _, label_i, label_j = _PLANES[plane]
    bounds_by_axis = {
        "X": lambda lm: (lm.x_min, lm.x_max),
        "Y": lambda lm: (lm.y_min, lm.y_max),
        "Z": lambda lm: (lm.z_min, lm.z_max),
    }
    for lm in landmarks:
        lo_i, hi_i = bounds_by_axis[label_i](lm)
        lo_j, hi_j = bounds_by_axis[label_j](lm)
        ax.add_patch(Rectangle(
            (lo_i, lo_j), hi_i - lo_i, hi_j - lo_j,
            fill=False, edgecolor="darkorange", linestyle="--", linewidth=1.5,
            label="landmark",
        ))


# ---------------------------------------------------------------------------
# Data panels
# ---------------------------------------------------------------------------


def _plot_surface_panel(ax: Any, scan: "SurfaceScan") -> None:
    """Z heatmap, downsampled — same method as scripts/visualize_scan.py.

    ``x_mm``/``y_mm`` 1D (uniform surface) renders via ``imshow`` with a
    flat extent; 2D (point cloud — including any scan run through
    ``orient_scan()``, which always downgrades to per-cell storage) renders
    as a real scatter of each cell's own (x, y), since imshow's extent
    assumes uniform column spacing that a point cloud doesn't have — see
    that script's ``plot_scan()`` docstring for the full rationale.
    """
    z, x, y = scan.z_mm, scan.x_mm, scan.y_mm
    non_uniform_xy = x.ndim == 2
    row_stride = max(1, z.shape[0] // 400)
    col_stride = max(1, z.shape[1] // 1200)

    zd = z[::row_stride, ::col_stride]
    if non_uniform_xy:
        xd = x[::row_stride, ::col_stride]
        yd = y[::row_stride, ::col_stride]
    else:
        xd = x[::col_stride]
        yd = y[::row_stride]

    valid = ~np.isnan(z)
    valid_frac = valid.sum() / z.size if z.size else 0.0
    zvalid = z[valid]
    if zvalid.size == 0:
        ax.text(0.5, 0.5, "no valid points", ha="center", va="center", transform=ax.transAxes)
        return
    vmin, vmax = np.nanpercentile(z, 1), np.nanpercentile(z, 99)

    if non_uniform_xy:
        # A point can be a valid Z return but still carry a NaN x/y (or the
        # reverse) — plot only where all three are finite.
        valid_d = ~np.isnan(zd) & ~np.isnan(xd) & ~np.isnan(yd)
        im = ax.scatter(
            xd[valid_d], yd[valid_d], c=zd[valid_d], s=1, marker=".",
            cmap="viridis", vmin=vmin, vmax=vmax,
        )
        ax.set_aspect("equal")
        ax.set_facecolor("black")
    else:
        im = ax.imshow(
            zd, aspect="equal", origin="lower",
            extent=[np.nanmin(xd), np.nanmax(xd), np.nanmin(yd), np.nanmax(yd)],
            cmap="viridis", vmin=vmin, vmax=vmax,
        )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(f"Z heatmap (downsampled) — {valid_frac * 100:.1f}% valid returns")
    ax.figure.colorbar(im, ax=ax, label="Z (mm)")


def _plot_profile_panel(ax: Any, df: "pd.DataFrame") -> None:
    if "height_mm" in df.columns:
        y_col, y_label = "height_mm", "calibrated height (mm)"
    elif "pos_mm" in df.columns:
        # No calibration applied — still show something rather than nothing.
        y_col, y_label = "experiment_z_mm", "Z, mount offset only (mm) — no calibration"
    else:
        ax.text(0.5, 0.5, "no data columns found", ha="center", va="center", transform=ax.transAxes)
        return
    x = df["pos_mm"] if "pos_mm" in df.columns else range(len(df))
    ax.plot(x, df[y_col], color="tab:blue", linewidth=1)
    ax.set_xlabel("travel position (mm)" if "pos_mm" in df.columns else "sample")
    ax.set_ylabel(y_label)
    ax.grid(True, alpha=0.3)


def _plot_trajectory_panel(ax: Any, passes: Sequence["Pass"]) -> None:
    """Text summary table — index, instrument, axis, length, scan speed."""
    ax.axis("off")
    rows = [
        [str(p.index), p.instrument, p.axis, f"{p.length_mm:.0f}", f"{p.scan_speed or '-'}"]
        for p in passes
    ]
    table = ax.table(
        cellText=rows,
        colLabels=["#", "instrument", "axis", "length (mm)", "scan speed"],
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    ax.set_title(f"{len(passes)} passes")


# ---------------------------------------------------------------------------


def _default_title(data: Any, points: np.ndarray) -> str:
    kind = type(data).__name__
    return f"{kind} — {len(points)} points"
