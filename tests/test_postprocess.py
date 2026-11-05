"""Tests for laguna.postprocess: regions, config, and the LAStools pipeline.

The pipeline tests build a small synthetic scan (a flat bed with a mound,
a block of debris, and a few low outliers) and need LAStools installed;
they are skipped otherwise.
"""

import copy

import shutil

import numpy as np
import pytest
import yaml

laspy = pytest.importorskip("laspy")
pytest.importorskip("lazrs")
pytest.importorskip("rasterio")

from laguna.frames import FrameRegistry, orient_scan  # noqa: E402
from laguna.postprocess import (  # noqa: E402
    Circle,
    Polygon,
    PostprocessConfig,
    Rect,
    find_scans,
    load_postprocess_config,
    oriented_points,
    process_scan,
    run,
    save_postprocess_config,
)
from laguna.postprocess.orient import write_points  # noqa: E402
from laguna.scanner.pointcloud import SensorMounting, SurfaceScan  # noqa: E402
from laguna.postprocess.cli import main as cli_main  # noqa: E402
from laguna.postprocess.lastools import lastools_bin  # noqa: E402
from laguna.postprocess.preview import read_dem  # noqa: E402
from laguna.postprocess.regions import keep_mask, shape_from_dict  # noqa: E402

BED_Z = 30.0
MOUND = Circle(50, 50, 25)


class TestRegions:
    """Shape containment and serialisation."""

    def test_contains(self):
        """Each shape reports inside/outside correctly, edges included."""
        x = np.array([0.0, 5.0, 20.0])
        y = np.array([0.0, 5.0, 20.0])
        assert Rect(-1, -1, 10, 10).contains(x, y).tolist() == [True, True, False]
        assert Circle(0, 0, 8).contains(x, y).tolist() == [True, True, False]
        tri = Polygon([(-1, -1), (10, -1), (-1, 10)])
        assert tri.contains(x, y).tolist() == [True, False, False]

    def test_invalid_shapes_rejected(self):
        """Degenerate shapes and unknown types raise ValueError."""
        with pytest.raises(ValueError):
            Rect(1, 0, 0, 1)
        with pytest.raises(ValueError):
            Circle(0, 0, 0)
        with pytest.raises(ValueError):
            Polygon([(0, 0), (1, 1)])
        with pytest.raises(ValueError):
            shape_from_dict({"type": "hexagon"})

    def test_keep_mask_include_and_exclude(self):
        """Include limits to its union; exclude carves out of it."""
        x, y = np.meshgrid(np.arange(10) + 0.5, np.arange(10) + 0.5)
        x, y = x.ravel(), y.ravel()
        roi = Rect(0, 0, 10, 10)
        assert keep_mask(x, y, roi, [], []).all()
        keep = keep_mask(x, y, roi, [Rect(0, 0, 5, 10)], [Rect(0, 0, 5, 2)])
        assert keep.sum() == 5 * 8
        assert not keep[x > 5].any()


class TestConfig:
    """YAML round trip and the reprocessing hash."""

    def test_roundtrip(self, tmp_path):
        """Saving then loading gives back the same config, with paths resolved."""
        cfg = PostprocessConfig(
            roi=Rect(0, 0, 10, 20),
            exclude=[Circle(1, 2, 3, label="a")],
            include=[Polygon([(0, 0), (1, 0), (0, 1)])],
            z_range=(1, 2),
            orient="frame",
            frame={"rotation_deg": 90, "origin": [5, 0, 0]},
        )
        path = tmp_path / "pp.yaml"
        save_postprocess_config(cfg, path)
        back = load_postprocess_config(path)
        assert back.to_dict() == cfg.to_dict()
        assert back.out_path == (tmp_path / "postprocessed").resolve()
        assert back.grid_shape == (20, 10)
        assert back.transform.apply([5, 0, 0]) == pytest.approx([0, 0, 0])

    def test_orient_settings_validated(self, tmp_path):
        """Each orient mode's missing or invalid settings fail at load, not mid-run."""
        roi = {"xmin": 0, "ymin": 0, "xmax": 1, "ymax": 1}
        bad = [
            {"orient": "sideways"},
            {"orient": "frame"},
            {"orient": "frame", "frame": {"scale": 2}},
            {"orient": "frame", "frame": {"matrix": np.diag([2, 1, 1, 1]).tolist()}},
            {"orient": "metadata"},
            {"orient": "metadata", "rig_config": "missing.yaml"},
        ]
        for extra in bad:
            path = tmp_path / "pp.yaml"
            path.write_text(yaml.safe_dump({"roi": roi, **extra}))
            with pytest.raises((ValueError, FileNotFoundError)):
                load_postprocess_config(path)

    def test_unused_orient_settings_are_ignored(self, tmp_path):
        """Both frame and rig_config can be set; only the active one counts."""
        cfg = PostprocessConfig(roi=Rect(0, 0, 1, 1), frame={"translation": [1, 0, 0]})
        cfg.validate()
        assert cfg.transform.is_identity
        assert cfg.processing_hash() == PostprocessConfig(roi=Rect(0, 0, 1, 1)).processing_hash()
        cfg.orient = "frame"
        assert cfg.transform.apply([0, 0, 0]) == pytest.approx([1, 0, 0])

    def test_hash_ignores_labels_output_and_cores(self):
        """Only settings that change outputs change the hash."""
        cfg = PostprocessConfig(roi=Rect(0, 0, 10, 20), exclude=[Circle(1, 2, 3, label="a")])
        h = cfg.processing_hash()
        other = copy.deepcopy(cfg)
        other.exclude[0].label, other.cores, other.output_dir = "renamed", 1, "elsewhere"
        assert other.processing_hash() == h
        other.exclude[0].r = 4
        assert other.processing_hash() != h


needs_lastools = pytest.mark.skipif(
    not (lastools_bin() / "lasground_new64").exists(), reason="LAStools not installed"
)


@pytest.fixture(scope="module")
def synthetic_scan(tmp_path_factory):
    """Write a 100 x 100 mm scan: flat bed, a 20 mm mound, debris and low outliers.

    The low outliers are 3 x 3 clusters: lasground_new's spike filter already
    drops single low points, but a cluster this size pulls the ground surface
    down unless lasnoise has removed it first.

    Written the way ``SurfaceScan.save_las`` writes scans (LAS 1.4, point
    format 0, 1e-4 mm scale).
    """
    rng = np.random.default_rng(0)
    xs, ys = np.meshgrid(np.arange(0, 100, 0.5), np.arange(0, 100, 0.5))
    x, y = xs.ravel(), ys.ravel()
    r2 = (x - MOUND.x) ** 2 + (y - MOUND.y) ** 2
    z = BED_Z + 20 * np.exp(-r2 / (2 * 10**2)) + rng.normal(0, 0.05, x.size)
    debris = (np.abs(x - 15) < 2) & (np.abs(y - 85) < 2)
    z[debris] += 15
    for cx, cy in [(20, 20), (80, 20), (80, 80)]:
        z[(np.abs(x - cx) < 0.75) & (np.abs(y - cy) < 0.75)] -= 25

    header = laspy.LasHeader(point_format=0, version="1.4")
    header.scales = np.array([1e-4] * 3)
    header.offsets = np.array([0.0, 0.0, 0.0])
    las = laspy.LasData(header)
    las.x, las.y, las.z = x, y, z
    path = tmp_path_factory.mktemp("scans") / "scan_synthetic.laz"
    las.write(path)
    return path


def _cfg(tmp_path, **kw):
    cfg = PostprocessConfig(
        roi=Rect(0, 0, 100, 100), z_range=(0, 100), cores=2, output_dir=str(tmp_path / "out"), **kw
    )
    cfg.ground.step = 5
    cfg.dem.kill = 5
    return cfg


@needs_lastools
class TestPipeline:
    """End-to-end runs through LAStools on the synthetic scan."""

    def test_ground_surface_drops_debris_and_outliers(self, synthetic_scan, tmp_path):
        """The DEM keeps the mound but neither the debris nor pits from low outliers."""
        cfg = _cfg(tmp_path)
        res = process_scan(synthetic_scan, cfg)
        assert res.status == "ok", res.error
        assert res.n_input >= res.n_roi >= res.n_denoised > res.n_ground > 0

        z, (xmin, xmax, ymin, ymax) = read_dem(cfg.out_path / "dem" / "scan_synthetic.tif")
        assert z.shape == cfg.grid_shape and (xmin, ymax) == (0, 100)
        assert np.nanmax(z) == pytest.approx(BED_Z + 20, abs=1)
        assert np.nanmin(z) > BED_Z - 1
        debris = z[100 - 87 : 100 - 83, 13:17]
        assert np.all(np.isnan(debris) | (debris < BED_Z + 2))

    def test_denoise_runs_before_ground(self, synthetic_scan, tmp_path):
        """Without denoising, the low clusters become pits in the ground surface."""
        cfg = _cfg(tmp_path)
        cfg.noise.enabled = False
        assert process_scan(synthetic_scan, cfg).status == "ok"
        z, _ = read_dem(cfg.out_path / "dem" / "scan_synthetic.tif")
        assert np.nanmin(z) < BED_Z - 10

    def test_exclusion_zone_is_blank(self, synthetic_scan, tmp_path):
        """Cells inside an exclude shape are nodata, not interpolated."""
        exc = Circle(50, 50, 10)
        cfg = _cfg(tmp_path, exclude=[exc])
        assert process_scan(synthetic_scan, cfg).status == "ok"
        z, (xmin, _, _, ymax) = read_dem(cfg.out_path / "dem" / "scan_synthetic.tif")
        rows, cols = np.indices(z.shape)
        inside = exc.contains(xmin + cols + 0.5, ymax - rows - 0.5)
        assert np.isnan(z[inside]).all()
        assert np.isfinite(z[~inside]).mean() > 0.9

    def test_frame_is_applied_before_roi(self, synthetic_scan, tmp_path):
        """Points are moved into the frame, and the DEM is the same surface turned."""
        plain = _cfg(tmp_path / "plain")
        turned = _cfg(
            tmp_path / "turned",
            orient="frame",
            frame={"rotation_deg": 90, "translation": [100, 0, 5]},
        )
        assert process_scan(synthetic_scan, plain).status == "ok"
        assert process_scan(synthetic_scan, turned).status == "ok"

        # (x, y, z) -> (100 - y, x, z + 5): the debris block at (15, 85) lands at (15, 15).
        las = laspy.read(turned.out_path / "clip" / "scan_synthetic.laz")
        x, y, z = np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)
        debris = (z > BED_Z + 15) & ~MOUND.contains(100 - y, x)
        assert debris.sum() > 10
        assert (x[debris].mean(), y[debris].mean()) == pytest.approx((15, 15), abs=1)

        z0, _ = read_dem(plain.out_path / "dem" / "scan_synthetic.tif")
        z1, _ = read_dem(turned.out_path / "dem" / "scan_synthetic.tif")
        diff = z1 - (np.rot90(z0) + 5)
        assert np.isfinite(diff).mean() > 0.9
        assert np.nanmedian(np.abs(diff)) < 0.05

    def test_rerun_skips_until_settings_change(self, synthetic_scan, tmp_path):
        """A current scan is skipped; changing a setting reprocesses it."""
        cfg = _cfg(tmp_path)
        assert [r.status for r in run([synthetic_scan], cfg)] == ["ok"]
        assert [r.status for r in run([synthetic_scan.parent], cfg)] == ["skipped"]
        cfg.ground.enabled = False
        res = run([synthetic_scan], cfg)[0]
        assert res.status == "ok" and res.n_ground is None
        lines = (cfg.out_path / "manifest.csv").read_text().splitlines()
        assert len(lines) == 3 and lines[0].startswith("scan,")

    def test_failure_is_reported_not_raised(self, tmp_path):
        """A bad input comes back as status 'failed' and leaves no outputs."""
        bad = tmp_path / "broken.laz"
        bad.write_bytes(b"not a laz file")
        cfg = _cfg(tmp_path)
        res = process_scan(bad, cfg)
        assert res.status == "failed" and res.error
        assert not (cfg.out_path / "dem" / "broken.tif").exists()
        assert not any((cfg.out_path / "tmp").iterdir())

    def test_cli(self, synthetic_scan, tmp_path, capsys):
        """``laguna-postprocess run`` then ``status`` on the same config."""
        cfg = _cfg(tmp_path)
        cfg.output_dir = "out"
        path = tmp_path / "pp.yaml"
        save_postprocess_config(cfg, path)
        assert cli_main(["run", str(synthetic_scan), "-c", str(path)]) == 0
        assert "1 processed" in capsys.readouterr().out
        assert cli_main(["status", str(synthetic_scan), "-c", str(path)]) == 0
        assert "all scans up to date" in capsys.readouterr().out


#: Mounting, pass and frames shaped like the first gantry dataset: sensor Y runs
#: along -X (a reverse pass), and the experiment frame is turned a quarter turn.
MOUNTING = {"scan_x": "+Y", "scan_y": "-X", "scan_z": "+Z"}
RIG_FRAMES = {
    "experiment": {"origin": [1400, 300, 0], "rotation_deg": -90},
    "instruments": {"gocator": {"translation": [50, 20, 0]}},
}


@pytest.fixture(scope="module")
def paired_scan(tmp_path_factory):
    """Save a scan the way ``GocatorScanner.save_scan`` does: ``.npz`` grid plus ``.laz``."""
    xs = np.arange(0, 100, 0.5)
    ys = np.arange(0, 100, 0.5)
    gx, gy = np.meshgrid(xs, ys)
    z = BED_Z + 20 * np.exp(-((gx - 30) ** 2 + (gy - 60) ** 2) / (2 * 10**2))
    surface = SurfaceScan(
        z_mm=z,
        x_mm=xs,
        y_mm=ys,
        metadata={
            "gantry_axis": "X",
            "gantry_start": [1500.0, 200.0, 0.0],
            "gantry_start_mm": 1500.0,
            "gantry_end_mm": 1400.0,
        },
        is_uniform=True,
        mounting=SensorMounting.from_config(MOUNTING),
    )
    d = tmp_path_factory.mktemp("paired")
    surface.save_npz(d / "scan_paired.npz")
    surface.save_las(d / "scan_paired.laz")
    (d / "rig.yaml").write_text(yaml.safe_dump({"frames": RIG_FRAMES}))
    return d / "scan_paired.laz"


def _metadata_cfg(paired_scan, out):
    """A metadata-oriented config whose ROI covers the scan in experiment coordinates."""
    xyz = orient_scan(
        SurfaceScan.from_npz(paired_scan.with_suffix(".npz")),
        frames=FrameRegistry.from_config(RIG_FRAMES),
    ).to_points()
    lo, hi = np.floor(xyz.min(axis=0)), np.ceil(xyz.max(axis=0))
    cfg = PostprocessConfig(
        roi=Rect(lo[0], lo[1], hi[0], hi[1]),
        orient="metadata",
        rig_config=str(paired_scan.parent / "rig.yaml"),
        cores=2,
        output_dir=str(out),
    )
    cfg.dem.kill = 5
    return cfg, xyz


class TestScanFiles:
    """LAZ only, NPZ only, or both."""

    def test_pairs_are_listed_once(self, paired_scan, tmp_path):
        """A .laz/.npz pair is one scan; a lone .npz is a scan too."""
        assert find_scans([paired_scan.parent]) == [paired_scan]
        shutil.copy(paired_scan.with_suffix(".npz"), tmp_path / "lone.npz")
        assert find_scans([tmp_path, paired_scan.with_suffix(".npz")]) == sorted(
            [paired_scan, tmp_path / "lone.npz"]
        )

    def test_npz_and_laz_give_the_same_points(self, paired_scan):
        """In scanner coordinates the .npz grid and the .laz are the same cloud."""
        cfg = PostprocessConfig(roi=Rect(0, 0, 1, 1))
        from_laz = oriented_points(paired_scan, cfg)
        npz_only = paired_scan.parent / "npz_only"
        npz_only.mkdir(exist_ok=True)
        shutil.copy(paired_scan.with_suffix(".npz"), npz_only / "scan_paired.npz")
        from_npz = oriented_points(npz_only / "scan_paired.npz", cfg)
        np.testing.assert_allclose(from_npz, from_laz, atol=1e-3)

    def test_metadata_orients_with_the_rig_frames(self, paired_scan):
        """orient: metadata gives exactly what orient_scan gives."""
        cfg, expected = _metadata_cfg(paired_scan, "unused")
        np.testing.assert_allclose(oriented_points(paired_scan, cfg), expected)


@needs_lastools
class TestOrientedPipeline:
    """The orient modes through the whole pipeline."""

    def test_metadata_route(self, paired_scan, tmp_path):
        """The DEM matches one built from orient_scan's points in scanner mode."""
        cfg, xyz = _metadata_cfg(paired_scan, tmp_path / "meta")
        res = process_scan(paired_scan, cfg)
        assert res.status == "ok", res.error
        assert res.n_input == len(xyz)

        laz = tmp_path / "ref" / "scan_paired.laz"
        laz.parent.mkdir()
        write_points(xyz, laz)
        ref = PostprocessConfig(roi=cfg.roi, cores=2, output_dir=str(tmp_path / "ref_out"))
        ref.dem.kill = 5
        assert process_scan(laz, ref).status == "ok"
        z, _ = read_dem(cfg.out_path / "dem" / "scan_paired.tif")
        z_ref, _ = read_dem(ref.out_path / "dem" / "scan_paired.tif")
        np.testing.assert_array_equal(z, z_ref)
        assert np.isfinite(z).mean() > 0.9

    def test_metadata_needs_the_npz(self, paired_scan, tmp_path):
        """A LAZ-only scan fails under orient: metadata, naming the missing file."""
        cfg, _ = _metadata_cfg(paired_scan, tmp_path / "out")
        shutil.copy(paired_scan, tmp_path / "laz_only.laz")
        res = process_scan(tmp_path / "laz_only.laz", cfg)
        assert res.status == "failed" and "laz_only.npz" in res.error

    def test_cli_orient_override(self, paired_scan, tmp_path, capsys):
        """--orient switches routes without editing the config."""
        cfg, _ = _metadata_cfg(paired_scan, "out")
        cfg.orient = "scanner"
        path = tmp_path / "pp.yaml"
        save_postprocess_config(cfg, path)
        assert cli_main(["run", str(paired_scan), "-c", str(path), "--orient", "metadata"]) == 0
        assert "1 processed" in capsys.readouterr().out
        assert cli_main(["status", str(paired_scan), "-c", str(path)]) == 0
        assert "scan_paired" in capsys.readouterr().out
