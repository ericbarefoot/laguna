#!/usr/bin/env python3
"""Quick-look visualization for a Gocator surface scan .npz file.

Renders a 2x2 figure: a downsampled Z heatmap, the valid-return mask (where
the sensor got an actual triangulated return vs. 0x8000/no-data), a histogram
of valid Z values, and a single raw Y-row profile — enough to eyeball whether
a scan looks like a real surface (laser actually fired) or like noise/no
signal (flat near-zero baseline, a hard FOV-shaped mask constant across every
row, isolated edge artifacts).

Usage:
    python scripts/visualize_scan.py                          # most recent scan in data/scans/
    python scripts/visualize_scan.py data/scans/scan_....npz
    python scripts/visualize_scan.py --output /tmp/scan.png

Metadata note: GocatorScanner.save_npz() stores the scan's metadata dict via
repr(), not JSON (see laguna.scanner.pointcloud.SurfaceScan.save_npz) — this
script parses it with ast.literal_eval() to match, not json.loads().
"""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
from typing import Optional

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_SCAN_DIR = Path("data/scans")


def find_latest_scan(scan_dir: Path = DEFAULT_SCAN_DIR) -> Path:
    """Return the most recently modified .npz under `scan_dir`.

    Raises:
        FileNotFoundError: If `scan_dir` has no .npz files.
    """
    candidates = sorted(scan_dir.glob("*.npz"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No .npz scans found under {scan_dir}")
    return candidates[-1]


def load_scan(path: Path):
    """Return (z_mm, x_mm, y_mm, metadata_dict) from a GocatorScanner .npz."""
    d = np.load(path, allow_pickle=True)
    z = d["z_mm"]
    x = d["x_mm"]
    y = d["y_mm"]
    meta = ast.literal_eval(str(d["metadata"].item())) if "metadata" in d.files else {}
    return z, x, y, meta


def axis_labels(meta: dict) -> tuple:
    """Axis labels for (columns, rows), naming the gantry axis where known.

    The stored grid is in the sensor's frame: columns run across the laser
    line, rows along travel. Those are NOT generally the gantry's X and Y —
    with the sensor mounted rotated 90 degrees, gantry X motion produces the
    grid's *rows*. Scans saved with a mounting record `grid_axes`, so label
    from that rather than letting a reader assume column == gantry X.
    """
    grid = meta.get("grid_axes")
    if grid and len(grid) == 2:
        rows_axis, cols_axis = grid[0], grid[1]
        return (
            f"gantry {cols_axis} (mm, across laser line)",
            f"gantry {rows_axis} (mm, travel)",
        )
    return (
        "sensor X (mm, across laser line)",
        "sensor Y (mm, travel)",
    )


def plot_scan(z: np.ndarray, x: np.ndarray, y: np.ndarray, meta: dict, title: str):
    """Build the 2x2 diagnostic figure. Returns the Figure.

    ``x``/``y`` are 1D (uniform surface, resampled onto an even grid) or 2D
    (point cloud, native per-column sensor spacing — see
    laguna.scanner.pointcloud's module docstring). Point-cloud X spacing is
    NOT uniform across a row (confirmed on real scans in data/scans/:
    consecutive-column spacing ranges ~1.1-4.6mm within a single row, not a
    single constant) — imshow()'s `extent=` draws its whole array across one
    flat [xmin, xmax] span assuming every column is the same width, so a
    non-uniform-X point cloud rendered that way is geometrically warped
    regardless of how correct the *metadata*'s frame_rate/travel_speed are
    (those only govern Y spacing, which is genuinely uniform — the X
    warping is real and independent of them). Rendered as a real scatter of
    each cell's own (x, y) instead — correct for non-uniform spacing, and
    tolerates the NaN x/y that invalid-return cells carry in this mode
    (pcolormesh doesn't: it requires finite mesh coordinates everywhere,
    even for masked/invalid *values*).
    """
    col_label, row_label = axis_labels(meta)
    row_stride = max(1, z.shape[0] // 400)
    col_stride = max(1, z.shape[1] // 1200)
    non_uniform_xy = x.ndim == 2

    zd = z[::row_stride, ::col_stride]
    if non_uniform_xy:
        xd = x[::row_stride, ::col_stride]
        yd = y[::row_stride, ::col_stride]
    else:
        xd = x[::col_stride]
        yd = y[::row_stride]

    valid = ~np.isnan(z)
    valid_frac = valid.sum() / z.size
    zvalid = z[valid]

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))

    vmin = np.nanpercentile(z, 1) if zvalid.size else 0
    vmax = np.nanpercentile(z, 99) if zvalid.size else 1

    ax = axes[0, 0]
    if non_uniform_xy:
        # A point can be a valid Z return but still carry a NaN x/y (or the
        # reverse) — scatter only where all three are finite.
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
    ax.set_xlabel(col_label)
    ax.set_ylabel(row_label)
    ax.set_title(f"Z heatmap (downsampled) — {valid_frac * 100:.1f}% valid returns")
    plt.colorbar(im, ax=ax, label="Z (mm)")

    ax = axes[0, 1]
    if zvalid.size:
        ax.hist(zvalid, bins=200, color="steelblue")
        ax.set_title(
            f"Z distribution (valid only, n={zvalid.size:,})\n"
            f"mean={zvalid.mean():.2f} std={zvalid.std():.2f}"
        )
    else:
        ax.set_title("Z distribution — no valid returns")
    ax.set_xlabel("Z (mm)")
    ax.set_ylabel("count")

    ax = axes[1, 0]
    if non_uniform_xy:
        # Invalid-return cells generally have no known (x, y) either, so
        # there's no meaningful spatial "black" region to show for them —
        # just plot where valid returns actually landed.
        ax.scatter(xd[valid_d], yd[valid_d], c="white", s=1, marker=".")
        ax.set_aspect("equal")
        ax.set_facecolor("black")
    else:
        valid_d = valid[::row_stride, ::col_stride]
        ax.imshow(valid_d, aspect="equal", origin="lower",
                  extent=[np.nanmin(xd), np.nanmax(xd), np.nanmin(yd), np.nanmax(yd)], cmap="gray")
    ax.set_xlabel(col_label)
    ax.set_ylabel(row_label)
    ax.set_title("Valid-return mask (white = got a return, black = 0x8000/no data)")

    # mid_row = z.shape[0] // 2
    # ax = axes[1, 1]
    # ax.plot(x, z[mid_row], lw=0.5)
    # ax.set_xlabel(col_label)
    # ax.set_ylabel("Z (mm, height)")
    # ax.set_title(f"Single profile across the laser line (row {mid_row}, {y[mid_row]:.2f}mm along travel)")

    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    return fig


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "scan", nargs="?", default=None,
        help="Path to a scan .npz file. Omit to use the most recently modified "
        "file under data/scans/ (or --scan-dir).",
    )
    parser.add_argument(
        "--scan-dir", type=Path, default=DEFAULT_SCAN_DIR,
        help=f"Directory to search for the latest scan when `scan` is omitted (default: {DEFAULT_SCAN_DIR})",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Output PNG path. Defaults to <scan stem>_viz.png next to the input file.",
    )
    parser.add_argument("--dpi", type=int, default=130)
    args = parser.parse_args()

    scan_path: Optional[Path] = Path(args.scan) if args.scan else find_latest_scan(args.scan_dir)
    if not scan_path.exists():
        parser.error(f"No such file: {scan_path}")

    z, x, y, meta = load_scan(scan_path)

    title = (
        f"{scan_path.stem} — {z.shape[0]}x{z.shape[1]} grid, "
        f"axis={meta.get('gantry_axis', '?')} "
        f"{meta.get('gantry_start_mm', float('nan')):.1f}->{meta.get('gantry_end_mm', float('nan')):.1f}mm "
        f"@ {meta.get('gantry_feed_rate_mm_s', float('nan'))}mm/s, "
        f"frame_rate={meta.get('frame_rate_hz', '?')}Hz"
    )
    fig = plot_scan(z, x, y, meta, title)

    out = args.output or scan_path.with_name(scan_path.stem + "_viz.png")
    fig.savefig(out, dpi=args.dpi)
    print(f"Saved {out}")

    valid = ~np.isnan(z)
    print(f"{valid.sum() / z.size * 100:.1f}% valid returns "
          f"({valid.sum():,} / {z.size:,} cells)")
    if valid.any():
        print(f"Z: min={np.nanmin(z):.3f} max={np.nanmax(z):.3f} "
              f"median={np.nanmedian(z):.3f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
