"""Pipeline tests that run without a LAStools licence, against a stand-in.

The stand-in tools (one script installed under each LAStools name) do the
simplest thing that keeps the pipeline's contract: clip, copy, label ground,
grid. They record what each tool was given so tests can check stage order
and what reached each stage. They say nothing about LAStools' own output.
"""

import json
import os
import stat
import sys
import threading

import numpy as np
import pytest

laspy = pytest.importorskip("laspy")
pytest.importorskip("lazrs")
rasterio = pytest.importorskip("rasterio")

from laguna.postprocess import PostprocessConfig, Rect, Circle, find_scans, process_scan  # noqa: E402
from laguna.postprocess.lastools import LastoolsError, run_tool  # noqa: E402
from laguna.postprocess import pipeline  # noqa: E402
from laguna.postprocess.orient import scan_files, write_points  # noqa: E402

SHIM = r'''#!{python}
import json, os, sys
import laspy, numpy as np
tool = os.path.basename(sys.argv[0]).removesuffix("64")
a = sys.argv[1:]
def arg(flag, n=1):
    i = a.index(flag)
    return a[i + 1 : i + 1 + n] if n > 1 else a[i + 1]
las = laspy.read(arg("-i"))
with open(os.environ["SHIM_LOG"], "a") as f:
    f.write(json.dumps({{"tool": tool, "n": len(las.x)}}) + "\n")
if os.environ.get("SHIM_FAIL") == tool:
    print(os.environ.get("SHIM_MSG", "WARNING: license"))
if os.environ.get("SHIM_EMPTY") == tool:
    las.points = las.points[:0]
if tool == "las2las":
    xmin, ymin, xmax, ymax = map(float, arg("-keep_xy", 4))
    keep = (las.x >= xmin) & (las.x <= xmax) & (las.y >= ymin) & (las.y <= ymax)
    las.points = las.points[keep]
if tool == "lasground_new":
    las.classification = np.full(len(las.x), 2, dtype=np.uint8)
if tool == "las2dem":
    import rasterio
    from rasterio.transform import from_origin
    step = float(arg("-step")); x0, y0 = map(float, arg("-ll", 2))
    ncols, nrows = int(arg("-ncols")), int(arg("-nrows"))
    z = np.full((nrows, ncols), float(arg("-nodata")), dtype="float32")
    if len(las.x):
        c = np.clip(((np.asarray(las.x) - x0) / step).astype(int), 0, ncols - 1)
        r = np.clip(nrows - 1 - ((np.asarray(las.y) - y0) / step).astype(int), 0, nrows - 1)
        z[r, c] = np.asarray(las.z)
    with rasterio.open(arg("-o"), "w", driver="GTiff", height=nrows, width=ncols, count=1,
                       dtype="float32", transform=from_origin(x0, y0 + nrows * step, step, step),
                       nodata=float(arg("-nodata"))) as ds:
        ds.write(z, 1)
else:
    las.write(arg("-o"))
'''


@pytest.fixture
def fake_lastools(tmp_path, monkeypatch):
    bin_dir = tmp_path / "lastools_bin"
    bin_dir.mkdir()
    script = SHIM.format(python=sys.executable)
    for tool in ("las2las", "lasnoise", "lasground_new", "las2dem"):
        exe = bin_dir / f"{tool}64"
        exe.write_text(script)
        exe.chmod(exe.stat().st_mode | stat.S_IXUSR)
    log = tmp_path / "shim.log"
    monkeypatch.setenv("LASTOOLS_BIN", str(bin_dir))
    monkeypatch.setenv("SHIM_LOG", str(log))
    return lambda: [json.loads(line) for line in log.read_text().splitlines()]


def _scan(directory, name="scan_a", n=400, seed=0):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(1, 49, size=(n, 2))
    xyz = np.column_stack([xy, rng.uniform(10, 12, n)])
    path = directory / f"{name}.laz"
    write_points(xyz, path)
    return path, xyz


def _cfg(tmp_path, **kw):
    cfg = PostprocessConfig(roi=Rect(0, 0, 50, 50), z_range=(0, 100), cores=2,
                            output_dir=str(tmp_path / "out"), **kw)
    cfg.dem.step = 1
    return cfg


def test_stage_order_and_exclusion_before_noise_and_ground(tmp_path, fake_lastools):
    path, xyz = _scan(tmp_path)
    zone = Circle(25, 25, 10)
    cfg = _cfg(tmp_path, exclude=[zone])
    inside = int(zone.contains(xyz[:, 0], xyz[:, 1]).sum())
    assert inside > 0

    res = process_scan(path, cfg)
    assert res.status == "ok", res.error
    calls = fake_lastools()
    assert [c["tool"] for c in calls] == ["las2las", "lasnoise", "lasground_new", "las2dem"]
    by_tool = {c["tool"]: c["n"] for c in calls}
    # Excluded points are gone before the noise and ground filters see them.
    assert by_tool["lasnoise"] == len(xyz) - inside
    assert by_tool["lasground_new"] == len(xyz) - inside
    assert (cfg.out_path / "clip" / "scan_a.laz").exists()
    assert (cfg.out_path / "dem" / "scan_a.tif").exists()


def test_is_current_handles_corrupt_meta_and_missing_clip(tmp_path, fake_lastools):
    path, _ = _scan(tmp_path)
    cfg = _cfg(tmp_path)
    assert process_scan(path, cfg).status == "ok"
    assert process_scan(path, cfg).status == "skipped"

    (cfg.out_path / "clip" / "scan_a.laz").unlink()
    assert process_scan(path, cfg).status == "ok"  # redone, not "up to date"

    (cfg.out_path / "meta" / "scan_a.json").write_text('{"config_ha')
    assert pipeline.is_current(path, cfg) is False
    assert process_scan(path, cfg).status == "ok"  # the batch carries on


@pytest.mark.parametrize("tool", ["lasnoise", "lasground_new"])
def test_a_stage_that_leaves_no_points_fails_the_scan(tmp_path, fake_lastools, monkeypatch, tool):
    path, _ = _scan(tmp_path)
    cfg = _cfg(tmp_path)
    monkeypatch.setenv("SHIM_EMPTY", tool)
    res = process_scan(path, cfg)
    assert res.status == "failed" and "no points left" in res.error
    assert not (cfg.out_path / "dem" / "scan_a.tif").exists()


def test_license_warning_fails_the_scan(tmp_path, fake_lastools, monkeypatch):
    path, _ = _scan(tmp_path)
    cfg = _cfg(tmp_path)
    monkeypatch.setenv("SHIM_FAIL", "lasground_new")
    monkeypatch.setenv("SHIM_MSG", "WARNING: unlicensed version limits output to 1000000 points")
    res = process_scan(path, cfg)
    assert res.status == "failed" and "license warning" in res.error


def test_run_tool_ignores_unrelated_warnings(tmp_path, fake_lastools, monkeypatch):
    path, _ = _scan(tmp_path)
    monkeypatch.setenv("SHIM_FAIL", "las2las")
    monkeypatch.setenv("SHIM_MSG", "WARNING: 3 points outside the bounding box")
    run_tool("las2las", "-i", path, "-keep_xy", 0, 0, 50, 50, "-o", tmp_path / "o.laz")
    with pytest.raises(LastoolsError):
        monkeypatch.setenv("SHIM_MSG", "WARNING: license check skipped, 5 points kept")
        run_tool("las2las", "-i", path, "-keep_xy", 0, 0, 50, 50, "-o", tmp_path / "o.laz")


def test_failure_keeps_the_previous_good_log(tmp_path, fake_lastools, monkeypatch):
    path, _ = _scan(tmp_path)
    cfg = _cfg(tmp_path)
    assert process_scan(path, cfg).status == "ok"
    good = (cfg.out_path / "logs" / "scan_a.log").read_text()
    monkeypatch.setenv("SHIM_EMPTY", "lasnoise")
    assert process_scan(path, cfg, force=True).status == "failed"
    assert (cfg.out_path / "logs" / "scan_a.log").read_text() == good
    assert (cfg.out_path / "logs" / "scan_a.failed.log").exists()
    assert (cfg.out_path / "dem" / "scan_a.tif").exists()  # earlier result untouched


def test_manifest_has_one_header_under_concurrent_writers(tmp_path):
    out = tmp_path
    fields = pipeline.MANIFEST_FIELDS
    barrier = threading.Barrier(8)

    def writer(i):
        barrier.wait()
        pipeline._append_manifest(out, {k: i for k in fields})

    for _ in range(25):
        (out / "manifest.csv").unlink(missing_ok=True)
        threads = [threading.Thread(target=writer, args=(i,)) for i in range(8)]
        [t.start() for t in threads]
        [t.join() for t in threads]
        lines = (out / "manifest.csv").read_text().splitlines()
        assert sum(line.startswith("scan,") for line in lines) == 1
        assert len(lines) == 9


def test_dotted_scan_names_are_kept_apart(tmp_path):
    for name in ("pass_0.5m", "pass_0", "scanA"):
        _scan(tmp_path, name)
    found = [p.name for p in find_scans([tmp_path])]
    assert found == ["pass_0.5m.laz", "pass_0.laz", "scanA.laz"]
    assert scan_files(tmp_path / "pass_0.5m.laz")["points"].name == "pass_0.5m.laz"
    assert scan_files(tmp_path / "pass_0.5m")["points"].name == "pass_0.5m.laz"


def test_run_sweeps_only_stale_tmp_dirs(tmp_path):
    tmp = tmp_path / "tmp"
    old, fresh = tmp / "old.x", tmp / "fresh.x"
    old.mkdir(parents=True), fresh.mkdir()
    ancient = 1_000_000_000
    os.utime(old, (ancient, ancient))
    pipeline._sweep_stale_tmp(tmp)
    assert not old.exists() and fresh.exists()
