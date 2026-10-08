"""Find a raised block on an uneven bed in an oriented point cloud.

Used by the Gocator alignment run, where a known block of a known size sits
on a surface that is not flat: a test area can have a raised ledge, a tilt, or
two plateaus tens of mm apart. A global "N mm above the bed" threshold, with
the bed taken as a low percentile of all heights, then lands inside the
higher plateau and flags a third of the scan as "block" (the failure this
module exists to avoid).

Instead the bed is estimated *locally*: a large-window median of the heights
around each spot, which ignores a block much smaller than the window. Cells
standing `min_height_mm` above their local bed are grouped into connected
blobs, and the blob whose footprint best matches the expected block is the
answer, so scattered high returns and larger raised features are passed over.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Sequence

import numpy as np

try:  # scipy is a core dependency of the scanner stack; guard only for clarity of the error
    from scipy import ndimage
except ImportError as exc:  # pragma: no cover
    raise ImportError("laguna.scanner.block_finder needs scipy") from exc

#: Cells the block's footprint may span relative to the expected area before a
#: blob is rejected as "not the block" (a wall, a ledge, a speck).
_AREA_TOLERANCE = (0.25, 4.0)


def find_block(
    points: np.ndarray,
    *,
    expected_size_mm: Optional[Sequence[float]] = None,
    min_height_mm: float = 20.0,
    cell_mm: float = 5.0,
    background_mm: float = 500.0,
    roi_mask: Optional[np.ndarray] = None,
    edge_percentile: float = 0.5,
) -> Optional[Dict[str, Any]]:
    """Locate the block in an (N, >=3) point cloud.

    Args:
        points: ``[x, y, z]`` rows, mm, in a frame where Z is height.
        expected_size_mm: ``(a, b)`` footprint of the block, in either
            orientation. Used to pick the right blob and to reject ones that
            are far too big or too small. Without it the largest blob wins.
        min_height_mm: How far above its local bed a cell must stand. Keep it
            well under the block's real height.
        cell_mm: Grid cell for blob finding. Opening/closing runs over about
            two cells, so features thinner than ~2 cells are discarded and
            holes up to that size (laser dropout) are filled.
        background_mm: Window of the local-bed median. Several times the
            block's largest side, so the block can't pull its own bed up.
        roi_mask: Optional ``(N,)`` bool mask; only blobs overlapping it are
            considered. The bed is still estimated from every point.
        edge_percentile: Footprint edges are this percentile (and its mirror)
            of the chosen blob's points, which ignores stray returns.

    Returns:
        A dict with ``center`` and ``size`` (``(2,)`` X/Y), ``lo``/``hi``
        (footprint corners), ``bed_z`` (local bed under the block), ``top_z``
        (median block top) and ``n`` (points in the blob), or None if no blob
        qualifies.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) < 100:
        return None
    keep = np.isfinite(pts[:, :3]).all(axis=1)
    pts = pts[keep]
    if roi_mask is not None:
        roi_mask = np.asarray(roi_mask, dtype=bool)[keep]
    if len(pts) < 100:
        return None

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    x0, y0 = x.min(), y.min()

    # Local bed on a coarse grid (cheap median), looked up per fine cell.
    bg_cell = max(cell_mm, 20.0)
    bx, by = ((x - x0) // bg_cell).astype(int), ((y - y0) // bg_cell).astype(int)
    coarse_sum = np.zeros((bx.max() + 1, by.max() + 1))
    coarse_n = np.zeros_like(coarse_sum)
    np.add.at(coarse_sum, (bx, by), z)
    np.add.at(coarse_n, (bx, by), 1.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        coarse = coarse_sum / coarse_n
    coarse[coarse_n == 0] = np.nanmedian(coarse)
    window = max(3, int(round(background_mm / bg_cell)) | 1)
    bed_coarse = ndimage.median_filter(coarse, size=window, mode="nearest")

    fx, fy = ((x - x0) // cell_mm).astype(int), ((y - y0) // cell_mm).astype(int)
    shape = (fx.max() + 1, fy.max() + 1)
    fine_sum = np.zeros(shape)
    fine_n = np.zeros(shape)
    np.add.at(fine_sum, (fx, fy), z)
    np.add.at(fine_n, (fx, fy), 1.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        fine = fine_sum / fine_n
    ci = np.minimum((np.arange(shape[0]) * cell_mm // bg_cell).astype(int), bed_coarse.shape[0] - 1)
    cj = np.minimum((np.arange(shape[1]) * cell_mm // bg_cell).astype(int), bed_coarse.shape[1] - 1)
    bed = bed_coarse[np.ix_(ci, cj)]

    high = (fine_n > 0) & ((fine - bed) > min_height_mm)
    if roi_mask is not None:
        roi_cells = np.zeros(shape, bool)
        roi_cells[fx[roi_mask], fy[roi_mask]] = True
        high &= ndimage.binary_dilation(roi_cells, iterations=1)
    high = ndimage.binary_opening(ndimage.binary_closing(high, iterations=2), iterations=2)
    labels, count = ndimage.label(high)
    if count == 0:
        return None

    areas = ndimage.sum(high, labels, range(1, count + 1)) * cell_mm**2
    if expected_size_mm is not None:
        target = float(expected_size_mm[0]) * float(expected_size_mm[1])
        ratio = areas / target
        ok = (ratio >= _AREA_TOLERANCE[0]) & (ratio <= _AREA_TOLERANCE[1])
        if not ok.any():
            return None
        pick = int(np.argmin(np.where(ok, np.abs(np.log(np.maximum(ratio, 1e-9))), np.inf)))
    else:
        pick = int(np.argmax(areas))
    chosen = labels[fx, fy] == pick + 1
    if roi_mask is not None:
        chosen &= roi_mask
    blob = pts[chosen]
    if len(blob) < 50:
        return None

    lo = np.percentile(blob[:, :2], edge_percentile, axis=0)
    hi = np.percentile(blob[:, :2], 100.0 - edge_percentile, axis=0)
    cells = labels == pick + 1
    return {
        "center": (lo + hi) / 2, "size": hi - lo, "lo": lo, "hi": hi,
        "bed_z": float(np.median(bed[cells])), "top_z": float(np.median(blob[:, 2])),
        "n": int(len(blob)),
    }
