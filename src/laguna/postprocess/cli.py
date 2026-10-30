"""``laguna-postprocess`` command: batch-process saved scans into DEMs."""

from __future__ import annotations

import argparse
import logging
import sys

from .config import load_postprocess_config
from .pipeline import ScanResult, find_scans, is_current, run


def _fmt(r: ScanResult) -> str:
    if r.status == "failed":
        return f"FAILED {r.scan}: {r.error}"
    if r.status == "skipped":
        return f"skip   {r.scan}"
    return (
        f"ok     {r.scan}  {r.n_input} -> {r.n_roi} -> {r.n_shapes} -> {r.n_denoised} -> "
        f"{r.n_ground} ground, DEM nodata {r.dem_nodata_frac:.1%}"
    )


def main(argv: list[str] | None = None) -> int:
    """Run the command line; returns the exit code (1 if any scan failed)."""
    ap = argparse.ArgumentParser(
        prog="laguna-postprocess",
        description="ROI clip, denoise, ground-filter and grid saved scans into DEMs.",
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, help_ in [
        ("run", "process new or changed scans"),
        ("status", "list scans whose outputs are out of date"),
    ]:
        p = sub.add_parser(name, help=help_)
        p.add_argument("inputs", nargs="+", help="scan files or directories")
        p.add_argument("-c", "--config", required=True, help="post-processing YAML")
        if name == "run":
            p.add_argument("-f", "--force", action="store_true", help="reprocess everything")
            p.add_argument("-j", "--cores", type=int, help="override the config's cores")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    cfg = load_postprocess_config(a.config)
    if a.cmd == "status":
        stale = [s for s in find_scans(a.inputs) if not is_current(s, cfg)]
        print("\n".join(s.name for s in stale) or "all scans up to date")
        return 0

    if a.cores:
        cfg.cores = a.cores
    results = run(a.inputs, cfg, force=a.force, progress=lambda r: print(_fmt(r), flush=True))
    failed = sum(r.status == "failed" for r in results)
    done = sum(r.status == "ok" for r in results)
    print(
        f"{done} processed, {len(results) - done - failed} up to date, {failed} failed; "
        f"outputs in {cfg.out_path}"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
