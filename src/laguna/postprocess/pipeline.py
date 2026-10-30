"""Turn saved scans into ground-surface DEMs on a fixed grid.

Per scan: transform into the experiment frame (when ``cfg.frame`` is set),
then ROI rectangle and z window (``las2las``), then include/exclude shapes, then ``lasnoise``, then ``lasground_new``, then ``las2dem``, then
DEM cells in excluded areas set to nodata.

Order matters. Noise is removed *before* ground filtering because
``lasground_new`` seeds its surface from the lowest points. Stray low
returns left in the data get taken as ground and pull the surface down
around them. On the first gantry dataset, ground-then-denoise left pits up
to 18 mm deep and ground-only left pits up to 111 mm deep; denoise-first
left none. Exclusion zones (e.g. an obstruction that distorts the scan
around it) are applied before both, so they can't influence the surface
next to them.

Outputs under ``cfg.out_path`` for each scan:

- ``dem/<scan>.tif``: float32 DEM, nodata = ``cfg.dem.nodata``
- ``clip/<scan>.laz``: points that fed the DEM (class 2 = ground)
- ``meta/<scan>.json``: config, input fingerprint and point counts
- ``logs/<scan>.log``: LAStools output
- one row appended to ``manifest.csv``

Files are built in a temp dir and renamed into place, so an interrupted
run never leaves a half-written output. A scan is skipped when its outputs
came from the same input (size + mtime) and the same
:meth:`~laguna.postprocess.config.PostprocessConfig.processing_hash`.
"""

from __future__ import annotations

import csv
import fcntl
import json
import logging
import os
import shutil
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Iterable

import laspy
import numpy as np
import rasterio

from ..frames import AffineTransform
from .config import PostprocessConfig
from .lastools import run_tool
from .regions import keep_mask

logger = logging.getLogger(__name__)


@dataclass
class ScanResult:
    """Outcome of processing one scan.

    Attributes:
        scan: Scan file stem.
        status: ``"ok"``, ``"skipped"`` (already up to date) or ``"failed"``.
        n_input: Points in the input file.
        n_roi: Points left after the ROI rectangle and z window.
        n_shapes: Points left after the include/exclude shapes.
        n_denoised: Points left after denoising.
        n_ground: Points classified as ground (None if ground filtering is off).
        dem_nodata_frac: Share of empty DEM cells within the kept area
            (excluded areas are not counted).
        error: Error message when ``status == "failed"``.
    """

    scan: str
    status: str
    n_input: int | None = None
    n_roi: int | None = None
    n_shapes: int | None = None
    n_denoised: int | None = None
    n_ground: int | None = None
    dem_nodata_frac: float | None = None
    error: str | None = None


MANIFEST_FIELDS = [
    "scan",
    "processed_at",
    "config_hash",
    "n_input",
    "n_roi",
    "n_shapes",
    "n_denoised",
    "n_ground",
    "dem_nodata_frac",
]


def output_paths(scan: str | Path, cfg: PostprocessConfig) -> dict[str, Path]:
    """Return the ``clip``/``dem``/``meta``/``log`` output paths for a scan."""
    b, out = Path(scan).stem, cfg.out_path
    return {
        "clip": out / "clip" / f"{b}.laz",
        "dem": out / "dem" / f"{b}.tif",
        "meta": out / "meta" / f"{b}.json",
        "log": out / "logs" / f"{b}.log",
    }


def _fingerprint(path: Path) -> dict:
    st = path.stat()
    return {"size": st.st_size, "mtime_ns": st.st_mtime_ns}


def is_current(scan: str | Path, cfg: PostprocessConfig) -> bool:
    """Return True if the scan's outputs match its file and ``cfg``."""
    scan = Path(scan)
    p = output_paths(scan, cfg)
    if not (p["dem"].exists() and p["meta"].exists()):
        return False
    meta = json.loads(p["meta"].read_text())
    return meta.get("config_hash") == cfg.processing_hash() and meta.get("input") == _fingerprint(
        scan
    )


def _count(path: Path) -> int:
    with laspy.open(path) as f:
        return f.header.point_count


def _orient(src: Path, dst: Path, transform: AffineTransform) -> None:
    las = laspy.read(src)
    xyz = transform.apply(np.column_stack([las.x, las.y, las.z]))
    if len(xyz):
        # Re-centre the offsets so the moved coordinates still fit LAS's int32 range.
        las.change_scaling(offsets=np.floor(xyz.min(axis=0)))
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.write(dst)


def _apply_shapes(src: Path, dst: Path, cfg: PostprocessConfig) -> None:
    las = laspy.read(src)
    keep = keep_mask(np.asarray(las.x), np.asarray(las.y), cfg.roi, cfg.include, cfg.exclude)
    las.points = las.points[keep]
    las.write(dst)


def _mask_dem(src: Path, dst: Path, cfg: PostprocessConfig) -> float:
    """Blank cells outside the kept area; return the nodata share inside it."""
    with rasterio.open(src) as ds:
        z = ds.read(1).astype("float32")
        profile = ds.profile
        t = ds.transform
    rows, cols = np.indices(z.shape)
    x = t.c + (cols + 0.5) * t.a
    y = t.f + (rows + 0.5) * t.e
    kept = keep_mask(x.ravel(), y.ravel(), cfg.roi, cfg.include, cfg.exclude).reshape(z.shape)
    nodata = cfg.dem.nodata
    z[~kept] = nodata
    profile.update(
        dtype="float32",
        nodata=nodata,
        compress="deflate",
        predictor=3,
        tiled=True,
        blockxsize=256,
        blockysize=256,
    )
    with rasterio.open(dst, "w", **profile) as out:
        out.write(z, 1)
    n_kept = kept.sum()
    return float(((z == nodata) & kept).sum() / n_kept) if n_kept else 1.0


def _append_manifest(out: Path, row: dict) -> None:
    with open(out / "manifest.csv", "a", newline="") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        w = csv.DictWriter(f, fieldnames=MANIFEST_FIELDS)
        if f.tell() == 0:
            w.writeheader()
        w.writerow(row)


def process_scan(scan: str | Path, cfg: PostprocessConfig, force: bool = False) -> ScanResult:
    """Process one scan into a DEM.

    Errors are caught and returned as ``status="failed"`` rather than
    raised, so one bad scan doesn't stop a batch.

    Args:
        scan: ``.laz``/``.las`` file.
        cfg: Processing settings.
        force: Reprocess even if the outputs are already current.

    Returns:
        The result, with point counts after each stage.
    """
    scan = Path(scan).resolve()
    if not force and is_current(scan, cfg):
        return ScanResult(scan.stem, "skipped")

    out = cfg.out_path
    p = output_paths(scan, cfg)
    for d in ("clip", "dem", "meta", "logs", "tmp"):
        (out / d).mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix=f"{scan.stem}.", dir=out / "tmp"))
    res = ScanResult(scan.stem, "ok")
    try:
        with open(tmp / "log", "w") as log:
            src = scan
            if not cfg.transform.is_identity:
                _orient(scan, tmp / "oriented.laz", cfg.transform)
                src = tmp / "oriented.laz"
            r = cfg.roi
            zargs = ["-keep_z", *cfg.z_range] if cfg.z_range else []
            run_tool(
                "las2las",
                "-i",
                src,
                "-keep_xy",
                r.xmin,
                r.ymin,
                r.xmax,
                r.ymax,
                *zargs,
                "-o",
                tmp / "roi.laz",
                log=log,
            )
            res.n_input, res.n_roi = _count(scan), _count(tmp / "roi.laz")
            cur = tmp / "roi.laz"

            if cfg.include or cfg.exclude:
                _apply_shapes(cur, tmp / "shapes.laz", cfg)
                cur = tmp / "shapes.laz"
            res.n_shapes = _count(cur)

            if cfg.noise.enabled:
                n = cfg.noise
                run_tool(
                    "lasnoise",
                    "-i",
                    cur,
                    "-step_xy",
                    n.step_xy,
                    "-step_z",
                    n.step_z,
                    "-isolated",
                    n.isolated,
                    "-remove_noise",
                    "-o",
                    tmp / "clean.laz",
                    log=log,
                )
                cur = tmp / "clean.laz"
            res.n_denoised = _count(cur)

            demargs = []
            if cfg.ground.enabled:
                g = cfg.ground
                run_tool(
                    "lasground_new",
                    "-i",
                    cur,
                    "-step",
                    g.step,
                    "-offset",
                    g.offset,
                    "-spike",
                    g.spike,
                    "-bulge",
                    g.bulge,
                    "-o",
                    tmp / "ground.laz",
                    log=log,
                )
                cur = tmp / "ground.laz"
                res.n_ground = int((laspy.read(cur).classification == 2).sum())
                demargs = ["-keep_class", 2]

            nrows, ncols = cfg.grid_shape
            run_tool(
                "las2dem",
                "-i",
                cur,
                *demargs,
                "-step",
                cfg.dem.step,
                "-ll",
                r.xmin,
                r.ymin,
                "-ncols",
                ncols,
                "-nrows",
                nrows,
                "-kill",
                cfg.dem.kill,
                "-nodata",
                cfg.dem.nodata,
                "-o",
                tmp / "dem_raw.tif",
                log=log,
            )
            res.dem_nodata_frac = round(_mask_dem(tmp / "dem_raw.tif", tmp / "dem.tif", cfg), 4)

        meta = {
            "config_hash": cfg.processing_hash(),
            "input": _fingerprint(scan),
            "processed_at": datetime.now().isoformat(timespec="seconds"),
            "config": cfg.to_dict(),
            "result": asdict(res),
        }
        (tmp / "meta.json").write_text(json.dumps(meta, indent=1))

        os.replace(cur, p["clip"])
        os.replace(tmp / "dem.tif", p["dem"])
        os.replace(tmp / "log", p["log"])
        os.replace(tmp / "meta.json", p["meta"])
        _append_manifest(
            out,
            {
                "scan": res.scan,
                "processed_at": meta["processed_at"],
                "config_hash": meta["config_hash"],
                **{k: getattr(res, k) for k in MANIFEST_FIELDS[3:]},
            },
        )
    except Exception as e:  # noqa: BLE001 - one bad scan must not stop the batch
        res.status, res.error = "failed", f"{type(e).__name__}: {e}"
        if (tmp / "log").exists():
            shutil.copy(tmp / "log", p["log"])
        logger.error("post-processing %s failed: %s", scan.name, res.error)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return res


def find_scans(inputs: Iterable[str | Path]) -> list[Path]:
    """Expand files and directories (not recursive) to a sorted list of ``.laz``/``.las``."""
    scans = []
    for i in map(Path, inputs):
        if i.is_dir():
            scans += [p for p in i.iterdir() if p.suffix.lower() in (".laz", ".las")]
        else:
            scans.append(i)
    return sorted({p.resolve() for p in scans})


def run(
    inputs: Iterable[str | Path],
    cfg: PostprocessConfig,
    force: bool = False,
    progress: Callable[[ScanResult], None] | None = None,
) -> list[ScanResult]:
    """Process every scan in ``inputs`` in parallel.

    Args:
        inputs: Scan files and/or directories of scans.
        cfg: Processing settings; ``cfg.cores`` scans run at once.
        force: Reprocess scans whose outputs are already current.
        progress: Called with each result as it finishes.

    Returns:
        One result per scan, sorted by name.
    """
    scans = find_scans(inputs)
    logger.info("post-processing %d scans into %s", len(scans), cfg.out_path)
    results = []
    # Threads, not processes: the heavy lifting happens in LAStools
    # subprocesses. Forked workers deadlock if the caller has already decoded
    # LAZ (lazrs keeps a thread pool), and spawned workers would need an
    # `if __name__ == "__main__"` guard in every calling script.
    with ThreadPoolExecutor(max_workers=cfg.cores) as pool:
        for f in as_completed([pool.submit(process_scan, s, cfg, force) for s in scans]):
            res = f.result()
            results.append(res)
            if progress:
                progress(res)
    counts = {s: sum(r.status == s for r in results) for s in ("ok", "skipped", "failed")}
    logger.info(
        "post-processing done: %(ok)d processed, %(skipped)d up to date, " "%(failed)d failed",
        counts,
    )
    return sorted(results, key=lambda r: r.scan)
