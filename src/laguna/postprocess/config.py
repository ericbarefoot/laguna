"""Post-processing settings, stored as a standalone YAML file.

This is separate from the rig's :class:`laguna.config.Config`. One file
describes how to process one experiment's scans, and it usually changes
after the scans are taken. All lengths are in mm, as written by
``SurfaceScan.save_las``. Every setting except ``frame`` is in the
coordinates ``frame`` maps into (the scan's own if it is not set).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from ..frames import AffineTransform
from .regions import Rect, Shape, shape_from_dict


@dataclass
class NoiseParams:
    """``lasnoise`` settings.

    A point is removed when fewer than ``isolated`` points share the 3x3x3
    block of ``step_xy`` x ``step_xy`` x ``step_z`` cells around it.
    """

    enabled: bool = True
    step_xy: float = 3
    step_z: float = 3
    isolated: int = 8


@dataclass
class GroundParams:
    """``lasground_new`` settings.

    ``step`` is the initial ground grid size. ``offset`` is how far above the
    ground TIN a point can sit and still count as ground. ``spike`` removes
    up and down spikes larger than it. ``bulge`` lets sloped triangles bulge
    outward by this much before nearby points stop counting as ground.
    """

    enabled: bool = True
    step: float = 10
    offset: float = 0.5
    spike: float = 1
    bulge: float = 1


@dataclass
class DemParams:
    """``las2dem`` settings.

    ``step`` is the cell size. ``kill`` is the longest TIN edge to
    interpolate across; wider gaps stay nodata.
    """

    step: float = 1
    kill: float = 25
    nodata: float = -9999


@dataclass
class PostprocessConfig:
    """Everything that decides how a scan becomes a DEM.

    Attributes:
        frame: Rigid transform from the scan's own coordinates to the
            experiment frame, applied before anything else. Same keys as
            ``frames.experiment`` in the rig config (see
            :meth:`laguna.frames.AffineTransform.from_config`):
            ``rotation_deg`` about Z, then ``translation`` or ``origin``
            ([x, y, z] mm), or a 4x4 ``matrix``. None keeps the scan's
            coordinates.
        roi: Area to keep. It also fixes the DEM grid (origin = upper-left
            corner, size from ``dem.step``), so every DEM made with one config
            can be differenced cell by cell.
        include: If not empty, only points inside at least one of these are kept.
        exclude: Points inside any of these are dropped before denoising and
            ground filtering, and the matching DEM cells are set to nodata.
        z_range: Optional ``(zmin, zmax)`` window applied with the ROI.
        noise: Denoise settings.
        ground: Ground-filter settings. When enabled, the DEM uses ground
            points (class 2) only.
        dem: DEM settings.
        output_dir: Where outputs go; relative paths resolve against
            ``base_dir``.
        cores: Scans processed in parallel.
        base_dir: Directory a relative ``output_dir`` is resolved against.
            :func:`load_postprocess_config` sets it to the YAML file's folder.
            Not saved.
    """

    roi: Rect
    frame: dict | None = None
    include: list[Shape] = field(default_factory=list)
    exclude: list[Shape] = field(default_factory=list)
    z_range: tuple[float, float] | None = None
    noise: NoiseParams = field(default_factory=NoiseParams)
    ground: GroundParams = field(default_factory=GroundParams)
    dem: DemParams = field(default_factory=DemParams)
    output_dir: str = "postprocessed"
    cores: int = 8
    base_dir: str = field(default=".", repr=False, compare=False)

    @property
    def out_path(self) -> Path:
        """Absolute output directory."""
        return (Path(self.base_dir) / self.output_dir).resolve()

    @property
    def transform(self) -> AffineTransform:
        """``frame`` as a transform (identity when ``frame`` is None)."""
        return AffineTransform.from_config(self.frame)

    @property
    def grid_shape(self) -> tuple[int, int]:
        """``(nrows, ncols)`` of the DEM grid."""
        s = self.dem.step
        return (
            round((self.roi.ymax - self.roi.ymin) / s),
            round((self.roi.xmax - self.roi.xmin) / s),
        )

    def to_dict(self) -> dict:
        """Return the YAML-ready form (``base_dir`` excluded)."""
        return {
            "frame": self.frame,
            "roi": self.roi.to_dict(),
            "include": [s.to_dict() for s in self.include],
            "exclude": [s.to_dict() for s in self.exclude],
            "z_range": list(self.z_range) if self.z_range else None,
            "noise": asdict(self.noise),
            "ground": asdict(self.ground),
            "dem": asdict(self.dem),
            "output_dir": self.output_dir,
            "cores": self.cores,
        }

    @classmethod
    def from_dict(cls, d: dict) -> PostprocessConfig:
        """Build from the :meth:`to_dict` form; missing sections take defaults.

        Raises:
            ValueError: If ``roi`` is not a rectangle, or ``frame`` is not a
                valid rigid transform.
        """
        roi = shape_from_dict({"type": "rect", **d["roi"]})
        if not isinstance(roi, Rect):
            raise ValueError("roi must be a rectangle")
        frame = d.get("frame") or None
        AffineTransform.from_config(frame)
        return cls(
            roi=roi,
            frame=frame,
            include=[shape_from_dict(s) for s in d.get("include") or []],
            exclude=[shape_from_dict(s) for s in d.get("exclude") or []],
            z_range=tuple(d["z_range"]) if d.get("z_range") else None,
            noise=NoiseParams(**d.get("noise", {})),
            ground=GroundParams(**d.get("ground", {})),
            dem=DemParams(**d.get("dem", {})),
            output_dir=d.get("output_dir", "postprocessed"),
            cores=d.get("cores", 8),
        )

    def processing_hash(self) -> str:
        """Return a short hash of every setting that affects outputs.

        A scan is reprocessed when this changes. Shape labels, ``output_dir``
        and ``cores`` are left out, since they don't change the result.
        """
        d = self.to_dict()
        for k in ("output_dir", "cores"):
            d.pop(k)
        for s in d["include"] + d["exclude"] + [d["roi"]]:
            s.pop("label", None)
        return hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()[:16]


def load_postprocess_config(path: str | Path) -> PostprocessConfig:
    """Read a config YAML; a relative ``output_dir`` resolves against its folder."""
    path = Path(path)
    cfg = PostprocessConfig.from_dict(yaml.safe_load(path.read_text()))
    cfg.base_dir = str(path.parent.resolve())
    return cfg


def save_postprocess_config(cfg: PostprocessConfig, path: str | Path) -> None:
    """Write ``cfg`` as YAML. Comments in an existing file are not kept."""
    Path(path).write_text(yaml.safe_dump(cfg.to_dict(), sort_keys=False, default_flow_style=None))
