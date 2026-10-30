"""Quick-look rasters and plots for raw scans, DEMs and region shapes."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rasterio
from matplotlib import patches
from matplotlib.axes import Axes

from .config import PostprocessConfig
from .orient import oriented_points
from .regions import Circle, Polygon, Rect, Shape


def scan_grid(
    scan: str | Path,
    cfg: PostprocessConfig | None = None,
    cell: float = 2.0,
    zclip: tuple[float, float] = (0.5, 99.5),
) -> tuple[np.ndarray, tuple]:
    """Grid a whole scan to mean z per cell, for plotting.

    Args:
        scan: The scan's ``.laz``, ``.las`` or ``.npz``.
        cfg: If given, the points are oriented per ``cfg.orient`` first, so
            the plot is in the coordinates the ROI and shapes use.
        cell: Cell size, mm.
        zclip: Percentiles outside which points are ignored, so stray returns
            don't set the colour scale.

    Returns:
        ``(z, extent)``: a 2-D array (NaN = empty, row 0 = top) and
        ``(xmin, xmax, ymin, ymax)`` for ``imshow``.
    """
    x, y, z = oriented_points(scan, cfg).T
    lo, hi = np.percentile(z, zclip)
    keep = (z >= lo) & (z <= hi)
    x, y, z = x[keep], y[keep], z[keep]
    xmin, ymin = np.floor(x.min() / cell) * cell, np.floor(y.min() / cell) * cell
    ncols = int((x.max() - xmin) // cell) + 1
    nrows = int((y.max() - ymin) // cell) + 1
    col = ((x - xmin) // cell).astype(int)
    row = (nrows - 1) - ((y - ymin) // cell).astype(int)
    idx = row * ncols + col
    total = np.bincount(idx, weights=z, minlength=nrows * ncols)
    count = np.bincount(idx, minlength=nrows * ncols)
    with np.errstate(invalid="ignore", divide="ignore"):
        grid = (total / count).reshape(nrows, ncols)
    return grid, (xmin, xmin + ncols * cell, ymin, ymin + nrows * cell)


def read_dem(path: str | Path) -> tuple[np.ndarray, tuple]:
    """Read a DEM; returns ``(z, extent)`` with nodata as NaN."""
    with rasterio.open(path) as ds:
        z = ds.read(1).astype(float)
        if ds.nodata is not None:
            z[z == ds.nodata] = np.nan
        b = ds.bounds
    return z, (b.left, b.right, b.bottom, b.top)


def hillshade(
    z: np.ndarray,
    cell: float = 1.0,
    azimuth: float = 315,
    altitude: float = 45,
    exaggeration: float = 3.0,
) -> np.ndarray:
    """Return a 0-1 hillshade of ``z``; NaN cells stay NaN."""
    gy, gx = np.gradient(np.nan_to_num(z, nan=np.nanmedian(z)) * exaggeration, cell)
    slope = np.pi / 2 - np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    az, alt = np.radians(360 - azimuth + 90), np.radians(altitude)
    hs = np.sin(alt) * np.sin(slope) + np.cos(alt) * np.cos(slope) * np.cos(az - aspect)
    hs[np.isnan(z)] = np.nan
    return hs


def shape_patch(shape: Shape, **kw) -> patches.Patch:
    """Return a matplotlib patch for a region shape."""
    if isinstance(shape, Rect):
        return patches.Rectangle(
            (shape.xmin, shape.ymin), shape.xmax - shape.xmin, shape.ymax - shape.ymin, **kw
        )
    if isinstance(shape, Circle):
        return patches.Circle((shape.x, shape.y), shape.r, **kw)
    if isinstance(shape, Polygon):
        return patches.Polygon(np.asarray(shape.points), closed=True, **kw)
    raise TypeError(f"not a region shape: {shape!r}")


def draw_shapes(ax, cfg: PostprocessConfig) -> list:
    """Draw the ROI (gold), include shapes (green) and exclude shapes (red)."""
    artists = [ax.add_patch(shape_patch(cfg.roi, fill=False, ec="gold", lw=2, ls="--"))]
    for s in cfg.include:
        artists.append(ax.add_patch(shape_patch(s, fill=False, ec="lime", lw=1.5)))
    for s in cfg.exclude:
        artists.append(ax.add_patch(shape_patch(s, fc=(1, 0, 0, 0.25), ec="red", lw=1.5)))
    return artists


def show_dem(
    path: str | Path,
    ax=None,
    cfg: PostprocessConfig | None = None,
    vmin: float | None = None,
    vmax: float | None = None,
    shade: bool = True,
) -> Axes:
    """Plot a DEM, optionally hillshaded and with ``cfg``'s shapes on top.

    Returns:
        The axes plotted on.
    """
    import matplotlib.pyplot as plt

    if ax is None:
        _, ax = plt.subplots(figsize=(5, 8))
    z, ext = read_dem(path)
    im = ax.imshow(z, extent=ext, cmap="terrain", vmin=vmin, vmax=vmax, interpolation="nearest")
    if shade:
        ax.imshow(hillshade(z), extent=ext, cmap="gray", alpha=0.35, interpolation="nearest")
    if cfg is not None:
        draw_shapes(ax, cfg)
    ax.set_aspect("equal")
    ax.set_title(Path(path).stem, fontsize=8)
    ax.figure.colorbar(im, ax=ax, label="z (mm)", shrink=0.7)
    return ax
