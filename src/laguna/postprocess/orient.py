"""Find a scan's files and put its points in the coordinates the config uses.

A scan is named by its file stem. It may be saved as ``.laz``/``.las``
(points only), as ``.npz`` (the full grid plus metadata: gantry position,
travel direction, mounting), or as both side by side, which is what
``GocatorScanner.save_scan`` writes by default. Which file is read depends
on :attr:`~laguna.postprocess.config.PostprocessConfig.orient`:

- ``metadata`` reads the ``.npz``; nothing else has the gantry position.
- ``scanner`` and ``frame`` read the ``.laz`` when there is one and fall
  back to the ``.npz``. Both give the same points, and the same DEM, since
  ``save_scan`` writes the LAZ from the same grid.
"""

from __future__ import annotations

from pathlib import Path

import laspy
import numpy as np

from ..frames import orient_scan
from ..scanner.pointcloud import SurfaceScan
from .config import PostprocessConfig

POINT_SUFFIXES = (".laz", ".las")
SCAN_SUFFIXES = POINT_SUFFIXES + (".npz",)


def scan_files(scan: str | Path) -> dict[str, Path | None]:
    """Return the scan's ``"points"`` (``.laz``/``.las``) and ``"npz"`` files, None if absent."""
    base = Path(scan).with_suffix("")
    points = next((p for p in (base.with_suffix(s) for s in POINT_SUFFIXES) if p.exists()), None)
    npz = base.with_suffix(".npz")
    return {"points": points, "npz": npz if npz.exists() else None}


def source_file(scan: str | Path, cfg: PostprocessConfig | None = None) -> Path:
    """Return the file :func:`oriented_points` reads for this scan (``cfg=None``: as saved).

    Raises:
        FileNotFoundError: If ``orient: metadata`` is set and the scan has no
            ``.npz``, or the scan has no files at all.
    """
    files = scan_files(scan)
    if cfg is not None and cfg.orient == "metadata":
        if files["npz"] is None:
            raise FileNotFoundError(
                f"orient: metadata needs {Path(scan).with_suffix('.npz').name}, which "
                "holds the gantry position; only the point file was found"
            )
        return files["npz"]
    src = files["points"] or files["npz"]
    if src is None:
        raise FileNotFoundError(f"no .laz, .las or .npz for {Path(scan).with_suffix('')}")
    return src


def oriented_points(scan: str | Path, cfg: PostprocessConfig | None = None) -> np.ndarray:
    """Read the scan's valid points as ``(N, 3)`` float64, oriented per ``cfg.orient``.

    With ``cfg=None`` the points come back in the coordinates they were saved in.
    """
    src = source_file(scan, cfg)
    if src.suffix == ".npz":
        surface = SurfaceScan.from_npz(src)
        if cfg is not None and cfg.orient == "metadata":
            surface = orient_scan(surface, frames=cfg.frames)
        # float32, as save_scan uses for the .laz: the ground filter is sensitive
        # enough that float64 points move ~6% of DEM cells by up to 1 mm.
        xyz = surface.to_points(drop_invalid=True, dtype=np.float32).astype(np.float64)
    else:
        las = laspy.read(src)
        xyz = np.column_stack([las.x, las.y, las.z])
    if cfg is None or cfg.transform.is_identity:
        return xyz
    return cfg.transform.apply(xyz)


def write_points(xyz: np.ndarray, path: Path) -> None:
    """Write points as LAS 1.4, the way ``SurfaceScan.save_las`` does (1e-4 mm scale)."""
    header = laspy.LasHeader(point_format=0, version="1.4")
    header.scales = np.array([1e-4] * 3)
    header.offsets = np.floor(xyz.min(axis=0)) if len(xyz) else np.zeros(3)
    las = laspy.LasData(header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.write(path)


def prepare_points(scan: str | Path, cfg: PostprocessConfig, tmp: Path) -> tuple[Path, int]:
    """Return a point file in config coordinates for LAStools, and its point count.

    The scan's own ``.laz`` is used as it is when no change is needed;
    otherwise the oriented points are written to ``tmp/oriented.laz``.
    """
    src = source_file(scan, cfg)
    if src.suffix in POINT_SUFFIXES and cfg.orient == "scanner":
        with laspy.open(src) as f:
            return src, f.header.point_count
    xyz = oriented_points(scan, cfg)
    write_points(xyz, tmp / "oriented.laz")
    return tmp / "oriented.laz", len(xyz)
