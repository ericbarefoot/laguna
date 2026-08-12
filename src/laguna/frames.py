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
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Iterable, List, Optional, Sequence, Union

import numpy as np

if TYPE_CHECKING:
    from .scanner.pointcloud import SurfaceScan

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
        """Initialize an affine transform.

        Args:
            matrix: 4x4 homogeneous matrix. Defaults to identity.

        Raises:
            ValueError: If matrix is not 4x4, not rigid, or invalid.
        """
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
        """Return the identity transform.

        Returns:
            Identity transform that leaves points unchanged.
        """
        return cls()

    @classmethod
    def from_translation(cls, xyz: Sequence[float]) -> "AffineTransform":
        """Create a pure translation transform.

        Args:
            xyz: [x, y, z] offset in mm.

        Returns:
            Translation transform.

        Raises:
            ValueError: If xyz does not have 3 values.
        """
        vec = np.asarray(xyz, dtype=float).ravel()
        if vec.size != 3:
            raise ValueError(f"translation must have 3 values, got {vec.size}")
        m = np.eye(4)
        m[:3, 3] = vec
        return cls(m)

    @classmethod
    def from_rotation_z(cls, degrees: float) -> "AffineTransform":
        """Create a rotation about Z.

        Args:
            degrees: Rotation angle in degrees.

        Returns:
            Rotation transform about Z axis.
        """
        c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
        m = np.eye(4)
        m[:2, :2] = [[c, -s], [s, c]]
        return cls(m)

    @classmethod
    def from_axis_map(cls, **axes: str) -> "AffineTransform":
        """Build from a sensor axis map.

        Args:
            **axes: Axis map in the form e.g. ``scan_x="-Y", scan_y="+X"``.
                Shares parsing with laguna.scanner.mounting.SensorMounting.

        Returns:
            Transform with the specified axis mapping.

        Raises:
            ValueError: If the axis map is invalid or mirrored.
        """
        from .scanner.mounting import SensorMounting

        mounting = SensorMounting.from_config(axes)
        m = np.eye(4)
        m[:3, :3] = mounting.matrix
        return cls(m)

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]]) -> "AffineTransform":
        """Build from a config dictionary.

        Args:
            config: Config block with recognized keys applied in this order:
                matrix (4x4, overrides all), axes (sensor map), rotation_deg
                (about Z), translation ([x, y, z] in mm). Rotation is applied
                first, then translation, so translation is always read in the
                target frame.

        Returns:
            Transform from the config.

        Raises:
            ValueError: If config contains unknown keys or invalid values.
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
        """Compose two transforms.

        Returns:
            Composed transform such that (a @ b).apply(p) == a.apply(b.apply(p)).
        """
        return AffineTransform(self.matrix @ other.matrix)

    def inverse(self) -> "AffineTransform":
        """Compute the reverse mapping.

        Returns:
            Transform mapping from target frame back to source frame.
        """
        rot = self.matrix[:3, :3]
        inv = np.eye(4)
        inv[:3, :3] = rot.T
        inv[:3, 3] = -rot.T @ self.matrix[:3, 3]
        return AffineTransform(inv)

    @property
    def translation(self) -> np.ndarray:
        """The offset component in mm.

        Returns:
            [x, y, z] translation vector.
        """
        return self.matrix[:3, 3].copy()

    @property
    def is_identity(self) -> bool:
        """Check if this is the identity transform.

        Returns:
            True if this transform leaves points untouched.
        """
        return bool(np.allclose(self.matrix, np.eye(4), atol=_RIGID_TOL))

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Map points into the target frame.

        Args:
            points: (3,) single point or (N, 3) array of points.

        Returns:
            Transformed point(s) in the target frame, same shape as input.

        Raises:
            ValueError: If points is not (3,) or (N, 3).
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
        """Export to round-trippable config form.

        Returns:
            Dict with 'matrix' key containing the 4x4 matrix as a nested list.
        """
        return {"matrix": self.matrix.tolist()}

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        """Return a string representation of the transform.

        Returns:
            Debugging representation showing translation.
        """
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
        """Initialize an instrument frame.

        Args:
            name: Instrument key matching its config/subsystem name.
            mount: Transform from instrument frame to gantry frame.
                Defaults to identity.
            reference_point: The point in the instrument's frame that
                represents "where it measures" (e.g., laser dot). Defaults
                to origin (0, 0, 0).

        Raises:
            ValueError: If reference_point does not have 3 values.
        """
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
        """Get the gantry-frame offset to where this instrument measures.

        Returns:
            [x, y, z] offset from the gantry's commanded point to the
            measurement point.
        """
        return self.mount.apply(self.reference_point)

    @classmethod
    def from_config(cls, name: str, config: Optional[Dict[str, Any]]) -> "InstrumentFrame":
        """Build from a frames.instruments config block.

        Args:
            name: Instrument key.
            config: Config dict (see AffineTransform.from_config()).

        Returns:
            InstrumentFrame with the specified mount and reference point.
        """
        config = dict(config or {})
        reference_point = config.pop("reference_point", (0.0, 0.0, 0.0))
        return cls(
            name=name,
            mount=AffineTransform.from_config(config),
            reference_point=reference_point,
        )

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        """Return a string representation of the instrument frame.

        Returns:
            Debugging representation showing name and offset.
        """
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
        """Initialize a frame registry.

        Args:
            experiment_from_gantry: Transform from gantry to experiment frame.
                Defaults to identity.
            instruments: Dict mapping instrument keys to InstrumentFrame objects.
                Defaults to empty.
        """
        self.experiment_from_gantry = experiment_from_gantry or AffineTransform.identity()
        self.instruments: Dict[str, InstrumentFrame] = dict(instruments or {})

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]]) -> "FrameRegistry":
        """Build from the ``frames:`` config section.

        Args:
            config: Config dict with optional 'experiment' and 'instruments'
                keys. Missing or None yields identity transforms.

        Returns:
            FrameRegistry with experiment frame and all instrument frames.

        Raises:
            ValueError: If config contains unknown keys.
        """
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
        """Register an instrument's mount.

        Args:
            frame: InstrumentFrame to register.

        Returns:
            self, for method chaining.
        """
        self.instruments[frame.name] = frame
        return self

    def frame_for(self, instrument: str) -> InstrumentFrame:
        """Look up an instrument, defaulting to a zero-offset frame.

        An unconfigured instrument is treated as measuring at the gantry's
        commanded point (the default), keeping unconfigured rigs behaving as
        before.

        Args:
            instrument: Instrument key.

        Returns:
            InstrumentFrame for the instrument, or a zero-offset frame if
            unconfigured.
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
        """Map points from gantry coordinates to experiment frame.

        Args:
            points: (3,) or (N, 3) points in gantry coordinates.

        Returns:
            Points in experiment frame, same shape as input.
        """
        return self.experiment_from_gantry.apply(points)

    def experiment_to_gantry(self, points: np.ndarray) -> np.ndarray:
        """Map points from experiment frame to gantry coordinates.

        Args:
            points: (3,) or (N, 3) points in experiment coordinates.

        Returns:
            Points in gantry coordinates, same shape as input.
        """
        return self.experiment_from_gantry.inverse().apply(points)

    # -- inverse: where do I send the robot? ----------------------------

    def gantry_target_for(
        self,
        instrument: str,
        experiment_point: Sequence[float],
        reference_point: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """Gantry position that puts `instrument`'s measurement point on a target.

        This is what lets you name a *place* instead of a robot position:
        ask for the OD2000 at experiment (x, y, z), scan, then ask for the
        WTT12L at the same (x, y, z) and get a different gantry command that
        lands its dot in the same physical spot.

        Args:
            instrument: Instrument key.
            experiment_point: Desired measurement location, experiment frame.
            reference_point: Override the instrument's configured
                reference point, in its own frame, for a one-off target
                other than its usual measurement point — e.g. a Gocator
                swath edge rather than its centerline (see
                ``laguna.survey.SurveyRunner``). Defaults to the
                instrument's configured reference point.

        Returns:
            ``(3,)`` gantry position to command, e.g. via
            ``lab.move_to(list(target))``.
        """
        target_gantry = self.experiment_to_gantry(
            np.asarray(experiment_point, dtype=float)
        )
        frame = self.frame_for(instrument)
        offset = (
            frame.mount.apply(np.asarray(reference_point, dtype=float))
            if reference_point is not None
            else frame.offset
        )
        return target_gantry - offset

    def experiment_point_for(
        self,
        instrument: str,
        gantry_position: Sequence[float],
    ) -> np.ndarray:
        """Experiment-frame point `instrument` is measuring at a commanded gantry position.

        Inverse of :meth:`gantry_target_for`: that method answers "what do I
        command to put this instrument on this experiment point"; this
        answers "given wherever the gantry actually is, what experiment
        point is this instrument measuring right now." Useful for
        translating a paused/aborted pass's last commanded position back
        into experiment coordinates, or for logging what a pass actually
        covered rather than what it was asked to.

        Args:
            instrument: Instrument key.
            gantry_position: Commanded gantry position, gantry frame.

        Returns:
            ``(3,)`` point in experiment coordinates.
        """
        measured_gantry = np.asarray(gantry_position, dtype=float) + self.frame_for(instrument).offset
        return self.gantry_to_experiment(measured_gantry)

    def retarget(
        self,
        from_instrument: str,
        to_instrument: str,
        gantry_position: Sequence[float],
    ) -> np.ndarray:
        """Compute gantry position to put to_instrument where from_instrument was.

        Direct form of "re-scan with the other sensor" when you have the
        original gantry position. Independent of the experiment frame.

        Args:
            from_instrument: Source instrument key.
            to_instrument: Target instrument key.
            gantry_position: [x, y, z, ...] original gantry position.

        Returns:
            [x, y, z, ...] gantry position for the target instrument.
        """
        delta = (
            self.frame_for(from_instrument).offset
            - self.frame_for(to_instrument).offset
        )
        return np.asarray(gantry_position, dtype=float) + delta

    def describe(self) -> Dict[str, Any]:
        """Human-readable summary for logs and get_status().

        Returns:
            Dict with experiment frame translation and instrument offsets.
        """
        return {
            "experiment_translation_mm": self.experiment_from_gantry.translation.tolist(),
            "experiment_is_identity": self.experiment_from_gantry.is_identity,
            "instruments": {
                name: frame.offset.tolist()
                for name, frame in sorted(self.instruments.items())
            },
        }


def orient_scan(
    scan: "SurfaceScan",
    *,
    instrument: str = "gocator",
    frames: FrameRegistry,
    gantry_start: Optional[Sequence[float]] = None,
    dtype: Any = np.float32,
    output: Optional[Union[str, Path]] = None,
) -> "SurfaceScan":
    """Return a copy of `scan` with every point already in experiment coordinates.

    Explicit and separate from acquisition, mirroring
    :func:`~laguna.robot.macron.profiler.orient_profile` for the rangefinder
    transect case — same purpose (place raw sensor data in the shared
    experiment frame), same call shape (raw object in, same type out, an
    optional file `output`), different sensor geometry underneath.

    A surface's own coordinates are relative to where the pass began — the
    travel axis runs from roughly -length/2 to +length/2, not from the
    gantry position. Orienting it therefore needs the pass's starting
    gantry position, which ``scan_with_gantry()`` already records in
    ``scan.metadata["gantry_start_mm"]`` (with the axis in
    ``gantry_axis``); pass `gantry_start` explicitly to override.

    **Travel direction.** The sensor is fully encoderless (software/time
    triggered — see ``scan_with_gantry()``), so its own Y is just
    acquisition order: row 0 is whatever was captured first, the last row
    whatever was captured last, always centred symmetrically around 0
    regardless of which real-world direction the gantry actually moved.
    This anchors the *first-acquired* point to the pass's real starting
    position and orients everything else by the recorded
    ``gantry_start_mm -> gantry_end_mm`` direction (needs both — see
    ``scan_with_gantry()``, which records them together). Skipping that
    orientation step — i.e. just adding the starting position as a flat
    offset — silently mirrors the travel axis for any pass that travels in
    the negative direction along its axis, since acquisition order no
    longer matches increasing position; see ``docs/subsystems/scanner.md``,
    "Sensor axes are not gantry axes."

    **Returned shape.** The transform can rotate (the experiment frame's
    ``rotation_deg``, or an instrument mount's ``axes``), which a uniform
    grid's separate 1D `x_mm`/`y_mm` centre arrays can't represent once
    every cell's X/Y no longer lines up with its row/column. The returned
    scan is therefore always per-cell (``is_uniform=False``), same physical
    row/column layout as `scan`, with `mounting` reset to identity — the
    transform is now baked into the coordinates themselves, so
    ``to_points()``/``grid_axes``/``gantry_*_mm`` on the *result* no longer
    mean "apply the mount," just "read the stored values." Every cell is
    kept (not just valid ones), so ``save_npz()`` on the result still
    reflects the original grid, NaNs included.

    Args:
        scan: The scan to orient.
        instrument: Which instrument frame to use.
        frames: The lab's FrameRegistry.
        gantry_start: Full [x, y, z] gantry position at the start of the
            pass. Reconstructed from the scan's metadata when omitted.
        dtype: Output dtype for the transformed grid.
        output: Optional path to also write the oriented scan to — format
            inferred from the suffix (.csv/.las/.laz/.ply/.npz).

    Returns:
        A new SurfaceScan, in experiment coordinates.

    Raises:
        ValueError: If no starting position is given and the metadata
            doesn't carry one, or `output`'s suffix isn't recognized.
    """
    from .scanner.pointcloud import SensorMounting, SurfaceScan

    origin = {"X": 0, "Y": 1, "Z": 2}
    axis = scan.metadata.get("gantry_axis")
    travel_col = origin.get(axis)

    if gantry_start is None:
        full_start = scan.metadata.get("gantry_start")
        if full_start is not None:
            # The two static axes' real position, not just the travel axis
            # — see scan_with_gantry()'s metadata. Falls back below only for
            # scans made before this was recorded.
            gantry_start = list(full_start)
        else:
            start = scan.metadata.get("gantry_start_mm")
            if axis is None or start is None:
                raise ValueError(
                    "orient_scan() needs the gantry position where the pass "
                    "began; the scan's metadata has no 'gantry_start' (or, "
                    "for older scans, 'gantry_axis'/'gantry_start_mm' — only "
                    "scan_with_gantry() records these), so pass "
                    "gantry_start=[x, y, z] explicitly."
                )
            if travel_col is None:
                raise ValueError(
                    f"scan metadata names gantry axis {axis!r}, which is not "
                    "one of X/Y/Z — pass gantry_start explicitly."
                )
            # Static axes assumed 0 here, since only the travel axis's real
            # position was ever recorded for scans this old — pass
            # gantry_start explicitly for a scan where that assumption is
            # wrong (it very often is).
            gantry_start = [0.0, 0.0, 0.0]
            gantry_start[travel_col] = float(start)
    else:
        gantry_start = list(gantry_start)

    # The scan's points are already gantry-*oriented*: SurfaceScan applies
    # its own mounting rotation in to_points(). So this instrument frame
    # must supply the translation only — a rotation here as well would
    # apply the sensor's turn twice, silently.
    frame = frames.frame_for(instrument)
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

    # Two flattenings: valid-only to find the anchor (same "first acquired"
    # semantics orient_scan() always used), full-grid (NaNs kept) to build
    # the returned scan without silently dropping cells.
    valid_points = scan.to_points(drop_invalid=True, dtype=np.float64)
    full_points = scan.to_points(drop_invalid=False, dtype=np.float64)

    if travel_col is not None and full_points.shape[0] > 0:
        end = scan.metadata.get("gantry_end_mm")
        start_for_sign = scan.metadata.get("gantry_start_mm")
        if end is not None and start_for_sign is not None:
            travel_sign = 1.0 if float(end) >= float(start_for_sign) else -1.0
        else:
            travel_sign = 1.0
            logger.warning(
                "orient_scan(): scan metadata has 'gantry_axis' but not "
                "both 'gantry_start_mm'/'gantry_end_mm', so the direction "
                "this pass actually traveled can't be determined — "
                "assuming positive. Geometry will be mirrored along %s "
                "if that assumption is wrong.", axis,
            )
        # anchor is the first-ACQUIRED point, i.e. from the valid-only
        # flattening (drop_invalid=True walks cells in acquisition order —
        # see the original note this replaced). Falls back to the full
        # grid's own first cell only if literally every cell is invalid,
        # since there's nothing else to anchor to then.
        if valid_points.shape[0] > 0:
            anchor = float(valid_points[0, travel_col])
        else:
            anchor = float(full_points[0, travel_col])
        full_points[:, travel_col] = (
            float(gantry_start[travel_col])
            + travel_sign * (full_points[:, travel_col] - anchor)
        )
        # This axis is now fully resolved above — the flat offset below
        # must not add gantry_start[travel_col] a second time.
        gantry_start[travel_col] = 0.0

    offset = frame.offset + np.asarray(gantry_start, dtype=float)
    transformed = frames.experiment_from_gantry.apply(full_points + offset)

    rows, cols = scan.z_mm.shape
    grid = transformed.reshape(rows, cols, 3).astype(dtype, copy=False)
    oriented = SurfaceScan(
        x_mm=grid[:, :, 0],
        y_mm=grid[:, :, 1],
        z_mm=grid[:, :, 2],
        metadata=dict(scan.metadata),
        is_uniform=False,
        mounting=SensorMounting(),
    )

    if output is not None:
        output_path = Path(output)
        suffix = output_path.suffix.lower()
        writers = {
            ".csv": oriented.save_csv,
            ".las": oriented.save_las,
            ".laz": oriented.save_las,
            ".ply": oriented.save_ply,
            ".npz": lambda p: oriented.save_npz(p),
        }
        writer = writers.get(suffix)
        if writer is None:
            raise ValueError(
                f"orient_scan(): unknown output format {suffix!r} "
                f"(expected one of {sorted(writers)})"
            )
        writer(output_path)

    return oriented


__all__ = [
    "AffineTransform",
    "InstrumentFrame",
    "FrameRegistry",
    "orient_scan",
]
