"""Affine reference frames: put every instrument's data in one coordinate system.

Each instrument sees the world from where it is bolted. The Gocator's laser
line, the OD2000's dot and the WTT12L's dot are at three different places on
the carriage, and the Gocator's own axes are rotated relative to the gantry's
(see :mod:`laguna.scanner.mounting`). Without a common frame, "the same spot"
means three different gantry positions and three incomparable coordinates.

Two kinds of transform, deliberately kept separate
--------------------------------------------------
**Mount** (per instrument, fixed by the hardware): where that instrument's
measurement point sits relative to the gantry's commanded origin, and how its
axes are turned. Measured once when the rig is built or an instrument moves.

**Experiment frame** (per project, chosen by you): a rigid transform from
gantry coordinates to whatever origin and orientation the experiment wants —
typically a corner of the flume, so every coordinate comes out positive.
Changing it re-labels all output without touching any mount.

They compose in one direction::

    experiment_point = experiment_from_gantry @ translate(gantry_position)
                                             @ mount_i @ sensor_reading

and invert for positioning — "put the OD2000's dot at experiment (x, y, z),
what do I command?" — which is what makes it possible to re-scan a transect
with a *different* instrument by naming the transect, not the robot position.

Everything is millimetres, and every transform is rigid (rotation plus
translation). Non-rigid matrices are rejected: a scale or shear would
silently distort real geometry, and nothing here has a legitimate use for one.

Config::

    frames:
      experiment:                     # gantry -> experiment
        translation: [500, 300, 0]    # move the origin to a flume corner
        rotation_deg: 0               # about Z, optional
      instruments:
        gocator:
          axes: {scan_x: -Y, scan_y: +X, scan_z: +Z}   # see scanner.mounting
          translation: [0, 0, -325]                    # standoff below the carriage
        od2000:
          translation: [52.0, -18.0, 0]
        wtt12l:
          translation: [52.0, 31.0, 0]
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Iterable, List, Optional, Sequence

import numpy as np

logger = logging.getLogger(__name__)

#: Tolerance for calling a matrix rigid — generous enough for values written
#: by hand in a config file, tight enough to catch a real scale/shear.
_RIGID_TOL = 1e-6


class AffineTransform:
    """A rigid 4x4 homogeneous transform, mapping points from one frame to another.

    Read a transform's name as *target*_from_*source*: ``apply()`` takes
    points in the source frame and returns them in the target frame.

    Args:
        matrix: 4x4 homogeneous matrix. Defaults to the identity.

    Raises:
        ValueError: If the matrix isn't 4x4, isn't a proper rigid transform
            (rotation must be orthonormal with determinant +1), or has a
            bottom row other than ``[0, 0, 0, 1]``.
    """

    def __init__(self, matrix: Optional[np.ndarray] = None) -> None:
        m = np.eye(4) if matrix is None else np.asarray(matrix, dtype=float)
        if m.shape != (4, 4):
            raise ValueError(f"transform matrix must be 4x4, got {m.shape}")
        if not np.allclose(m[3], [0.0, 0.0, 0.0, 1.0], atol=_RIGID_TOL):
            raise ValueError(
                f"transform matrix bottom row must be [0, 0, 0, 1], got {m[3]}"
            )
        rot = m[:3, :3]
        if not np.allclose(rot @ rot.T, np.eye(3), atol=_RIGID_TOL):
            raise ValueError(
                "transform rotation is not orthonormal, so it scales or shears "
                "the data. Only rigid transforms (rotation + translation) are "
                "allowed here — a scale would silently distort real geometry."
            )
        det = float(np.linalg.det(rot))
        if det < 0:
            raise ValueError(
                f"transform mirrors the data (rotation determinant {det:+.3f}). "
                "A reflection cannot describe a physical mounting or a change "
                "of origin — check for a swapped pair of axes without a "
                "compensating sign flip."
            )
        self.matrix = m

    # -- constructors ---------------------------------------------------

    @classmethod
    def identity(cls) -> "AffineTransform":
        """The do-nothing transform."""
        return cls()

    @classmethod
    def from_translation(cls, xyz: Sequence[float]) -> "AffineTransform":
        """Pure offset, in mm."""
        vec = np.asarray(xyz, dtype=float).ravel()
        if vec.size != 3:
            raise ValueError(f"translation must have 3 values, got {vec.size}")
        m = np.eye(4)
        m[:3, 3] = vec
        return cls(m)

    @classmethod
    def from_rotation_z(cls, degrees: float) -> "AffineTransform":
        """Rotation about Z — the usual case for a gantry-mounted instrument."""
        c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
        m = np.eye(4)
        m[:2, :2] = [[c, -s], [s, c]]
        return cls(m)

    @classmethod
    def from_axis_map(cls, **axes: str) -> "AffineTransform":
        """Build from a sensor axis map, e.g. ``scan_x="-Y", scan_y="+X"``.

        Shares :class:`laguna.scanner.mounting.SensorMounting`'s parsing and
        its rejection of mirroring maps, so the Gocator's 90-degree mounting
        is expressed the same way in both places.
        """
        from .scanner.mounting import SensorMounting

        mounting = SensorMounting.from_config(axes)
        m = np.eye(4)
        m[:3, :3] = mounting.matrix
        return cls(m)

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]]) -> "AffineTransform":
        """Build from a config block.

        Recognised keys, applied in this order — rotation first, then the
        offset, so ``translation`` is always read in the *target* frame and
        means what a reader expects ("move the origin over there"):

          - ``matrix``: an explicit 4x4, overriding everything else.
          - ``axes``: a sensor axis map (see :meth:`from_axis_map`).
          - ``rotation_deg``: rotation about Z, degrees.
          - ``translation``: [x, y, z] offset in mm.
        """
        if not config:
            return cls.identity()
        unknown = set(config) - {"matrix", "axes", "rotation_deg", "translation"}
        if unknown:
            raise ValueError(
                f"unknown transform key(s) {sorted(unknown)}; expected any of "
                "matrix, axes, rotation_deg, translation"
            )
        if "matrix" in config:
            return cls(np.asarray(config["matrix"], dtype=float))

        transform = cls.identity()
        if config.get("axes"):
            transform = cls.from_axis_map(**config["axes"])
        if config.get("rotation_deg"):
            transform = cls.from_rotation_z(float(config["rotation_deg"])) @ transform
        if config.get("translation") is not None:
            transform = cls.from_translation(config["translation"]) @ transform
        return transform

    # -- algebra --------------------------------------------------------

    def __matmul__(self, other: "AffineTransform") -> "AffineTransform":
        """Compose: ``(a @ b).apply(p)`` == ``a.apply(b.apply(p))``."""
        return AffineTransform(self.matrix @ other.matrix)

    def inverse(self) -> "AffineTransform":
        """The reverse mapping — target frame back to source frame."""
        rot = self.matrix[:3, :3]
        inv = np.eye(4)
        inv[:3, :3] = rot.T
        inv[:3, 3] = -rot.T @ self.matrix[:3, 3]
        return AffineTransform(inv)

    @property
    def translation(self) -> np.ndarray:
        """The offset component, mm."""
        return self.matrix[:3, 3].copy()

    @property
    def is_identity(self) -> bool:
        """True if this transform leaves points untouched."""
        return bool(np.allclose(self.matrix, np.eye(4), atol=_RIGID_TOL))

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Map a point ``(3,)`` or an ``(N, 3)`` array into the target frame.

        Applies the rotation and translation in place on a copy, without
        building an (N, 4) homogeneous array — these clouds run to tens of
        millions of points.
        """
        pts = np.asarray(points, dtype=float)
        single = pts.ndim == 1
        if single:
            pts = pts.reshape(1, 3)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"points must be (3,) or (N, 3), got {pts.shape}")
        out = pts @ self.matrix[:3, :3].T
        out += self.matrix[:3, 3]
        return out[0] if single else out

    def to_dict(self) -> Dict[str, Any]:
        """Round-trippable config form."""
        return {"matrix": self.matrix.tolist()}

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        t = self.translation
        return f"AffineTransform(translation=[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}])"


class InstrumentFrame:
    """Where one instrument measures, relative to the gantry's commanded point.

    Args:
        name: Instrument key, matching its config/subsystem name.
        mount: Transform from the instrument's own frame to the gantry frame,
            with the gantry at its origin.
        reference_point: The point *in the instrument's own frame* that
            "where it measures" refers to — the laser dot for a rangefinder.
            Defaults to that frame's origin.
    """

    def __init__(
        self,
        name: str,
        mount: Optional[AffineTransform] = None,
        reference_point: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.name = name
        self.mount = mount or AffineTransform.identity()
        self.reference_point = np.asarray(reference_point, dtype=float).ravel()
        if self.reference_point.size != 3:
            raise ValueError(
                f"{name}: reference_point must have 3 values, got "
                f"{self.reference_point.size}"
            )

    @property
    def offset(self) -> np.ndarray:
        """Return the gantry-frame offset from the commanded point to where it measures."""
        return self.mount.apply(self.reference_point)

    @classmethod
    def from_config(cls, name: str, config: Optional[Dict[str, Any]]) -> "InstrumentFrame":
        """Build from one ``frames.instruments.<name>`` config block."""
        config = dict(config or {})
        reference_point = config.pop("reference_point", (0.0, 0.0, 0.0))
        return cls(
            name=name,
            mount=AffineTransform.from_config(config),
            reference_point=reference_point,
        )

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        o = self.offset
        return f"InstrumentFrame({self.name!r}, offset=[{o[0]:.3f}, {o[1]:.3f}, {o[2]:.3f}])"


class FrameRegistry:
    """Every instrument's mount plus the experiment frame, in one place.

    Build from the ``frames:`` config section via :meth:`from_config`, or
    reach it as ``lab.frames`` once a :class:`~laguna.core.FlumeLab` is
    constructed.
    """

    def __init__(
        self,
        experiment_from_gantry: Optional[AffineTransform] = None,
        instruments: Optional[Dict[str, InstrumentFrame]] = None,
    ) -> None:
        self.experiment_from_gantry = experiment_from_gantry or AffineTransform.identity()
        self.instruments: Dict[str, InstrumentFrame] = dict(instruments or {})

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]]) -> "FrameRegistry":
        """Build from the ``frames:`` config section (None -> all identity)."""
        config = config or {}
        unknown = set(config) - {"experiment", "instruments"}
        if unknown:
            raise ValueError(
                f"unknown frames key(s) {sorted(unknown)}; expected 'experiment' "
                "and/or 'instruments'"
            )
        instruments = {
            name: InstrumentFrame.from_config(name, spec)
            for name, spec in (config.get("instruments") or {}).items()
        }
        return cls(
            experiment_from_gantry=AffineTransform.from_config(config.get("experiment")),
            instruments=instruments,
        )

    # ------------------------------------------------------------------

    def add(self, frame: InstrumentFrame) -> "FrameRegistry":
        """Register an instrument's mount. Returns self, so calls chain."""
        self.instruments[frame.name] = frame
        return self

    def frame_for(self, instrument: str) -> InstrumentFrame:
        """Look up an instrument, defaulting to a zero-offset frame.

        An unconfigured instrument is treated as measuring exactly at the
        gantry's commanded point — the honest default, and it keeps a rig
        with no ``frames:`` section behaving as it always did. The lookup is
        logged at debug level so a silently-missing offset can be traced.
        """
        frame = self.instruments.get(instrument)
        if frame is None:
            logger.debug(
                "No frame configured for %r; assuming it measures at the "
                "gantry's commanded point (zero offset)",
                instrument,
            )
            return InstrumentFrame(instrument)
        return frame

    # -- forward: observations -> experiment frame ----------------------

    def to_experiment(
        self,
        instrument: str,
        points: np.ndarray,
        gantry_position: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> np.ndarray:
        """Map an instrument's readings into the experiment frame.

        Args:
            instrument: Instrument key, e.g. ``"gocator"``.
            points: ``(3,)`` or ``(N, 3)`` in the *instrument's own* frame.
            gantry_position: Where the gantry was when the reading was taken,
                in gantry coordinates.

        Returns:
            The same shape, in experiment coordinates.
        """
        frame = self.frame_for(instrument)
        gantry_points = frame.mount.apply(points) + np.asarray(
            gantry_position, dtype=float
        )
        return self.experiment_from_gantry.apply(gantry_points)

    def gantry_to_experiment(self, points: np.ndarray) -> np.ndarray:
        """Map points already in gantry coordinates into the experiment frame."""
        return self.experiment_from_gantry.apply(points)

    def experiment_to_gantry(self, points: np.ndarray) -> np.ndarray:
        """Map experiment coordinates back into gantry coordinates."""
        return self.experiment_from_gantry.inverse().apply(points)

    # -- inverse: where do I send the robot? ----------------------------

    def gantry_target_for(
        self,
        instrument: str,
        experiment_point: Sequence[float],
    ) -> np.ndarray:
        """Gantry position that puts `instrument`'s measurement point on a target.

        This is what lets you name a *place* instead of a robot position:
        ask for the OD2000 at experiment (x, y, z), scan, then ask for the
        WTT12L at the same (x, y, z) and get a different gantry command that
        lands its dot in the same physical spot.

        Args:
            instrument: Instrument key.
            experiment_point: Desired measurement location, experiment frame.

        Returns:
            ``(3,)`` gantry position to command, e.g. via
            ``lab.move_to(list(target))``.
        """
        target_gantry = self.experiment_to_gantry(
            np.asarray(experiment_point, dtype=float)
        )
        return target_gantry - self.frame_for(instrument).offset

    def retarget(
        self,
        from_instrument: str,
        to_instrument: str,
        gantry_position: Sequence[float],
    ) -> np.ndarray:
        """Gantry position putting `to_instrument` where `from_instrument` was.

        The direct form of "re-scan that transect with the other sensor",
        when you have the original gantry position rather than an experiment
        coordinate. Independent of the experiment frame — it is purely the
        difference of the two mounts.
        """
        delta = (
            self.frame_for(from_instrument).offset
            - self.frame_for(to_instrument).offset
        )
        return np.asarray(gantry_position, dtype=float) + delta

    def place_scan(
        self,
        scan: Any,
        instrument: str = "gocator",
        gantry_start: Optional[Sequence[float]] = None,
        dtype: Any = np.float32,
    ) -> np.ndarray:
        """Flatten a scan into experiment-frame points.

        Takes a :class:`~laguna.scanner.pointcloud.SurfaceScan`.
        A surface's own coordinates are relative to where the pass began —
        the travel axis runs from roughly -length/2 to +length/2, not from
        the gantry position. Placing it therefore needs the pass's starting
        gantry position, which ``scan_with_gantry()`` already records in
        ``scan.metadata["gantry_start_mm"]`` (with the axis in
        ``gantry_axis``); pass `gantry_start` explicitly to override.

        Args:
            scan: The scan to place.
            instrument: Which instrument frame to use.
            gantry_start: Full [x, y, z] gantry position at the start of the
                pass. Reconstructed from the scan's metadata when omitted.
            dtype: Output dtype for the flattened points.

        Returns:
            (N, 3) experiment-frame points, mm.

        Raises:
            ValueError: If no starting position is given and the metadata
                doesn't carry one.
        """
        if gantry_start is None:
            axis = scan.metadata.get("gantry_axis")
            start = scan.metadata.get("gantry_start_mm")
            if axis is None or start is None:
                raise ValueError(
                    "place_scan() needs the gantry position where the pass "
                    "began; the scan's metadata has no 'gantry_axis'/"
                    "'gantry_start_mm' (only scan_with_gantry() records them), "
                    "so pass gantry_start=[x, y, z] explicitly."
                )
            origin = {"X": 0, "Y": 1, "Z": 2}
            if axis not in origin:
                raise ValueError(
                    f"scan metadata names gantry axis {axis!r}, which is not "
                    "one of X/Y/Z — pass gantry_start explicitly."
                )
            gantry_start = [0.0, 0.0, 0.0]
            gantry_start[origin[axis]] = float(start)

        # The scan's points are already gantry-*oriented*: SurfaceScan applies
        # its own mounting rotation in to_points(). So this instrument frame
        # must supply the translation only — a rotation here as well would
        # apply the sensor's turn twice, silently.
        frame = self.frame_for(instrument)
        scan_rotated = not getattr(scan, "mounting", None) or not scan.mounting.is_identity
        frame_rotates = not np.allclose(frame.mount.matrix[:3, :3], np.eye(3), atol=_RIGID_TOL)
        if scan_rotated and frame_rotates:
            raise ValueError(
                f"the scan is already rotated into gantry orientation by its "
                f"own mounting, and frames.instruments.{instrument} also "
                "specifies a rotation ('axes'/'rotation_deg') — applying both "
                "would turn the data twice. Keep the axis map in one place: "
                f"gocator.mounting for the scan, and give "
                f"frames.instruments.{instrument} only a translation."
            )

        points = scan.to_points(dtype=dtype)
        offset = frame.offset + np.asarray(gantry_start, dtype=float)
        return self.experiment_from_gantry.apply(points + offset).astype(
            dtype, copy=False
        )

    def describe(self) -> Dict[str, Any]:
        """Human-readable summary, for logs and get_status()."""
        return {
            "experiment_translation_mm": self.experiment_from_gantry.translation.tolist(),
            "experiment_is_identity": self.experiment_from_gantry.is_identity,
            "instruments": {
                name: frame.offset.tolist()
                for name, frame in sorted(self.instruments.items())
            },
        }


__all__ = [
    "AffineTransform",
    "InstrumentFrame",
    "FrameRegistry",
]
