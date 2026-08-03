"""Surface scan container plus point-cloud conversion and export.

A Gocator surface arrives as a **dense 2-D grid**, not an unordered cloud —
rows run along sensor Y (travel), columns along sensor X (across the laser
line). Those are the *sensor's* axis names and need not match the gantry's:
see :mod:`laguna.scanner.mounting`, and use ``SurfaceScan.grid_axes`` to see
which gantry axes the rows and columns actually correspond to. Both message
flavours carry the same raw-count-to-mm scaling:

    value_mm = offset_um / 1000 + resolution_nm / 1e6 * raw_count

Resolutions are nanometres (``k32u``), offsets micrometres (``k32s``), and
the raw grid values are 16-bit signed counts where ``0x8000`` (-32768) marks
an invalid/missing point. Confirmed against
``samples/C/ReceiveSurface/src/ReceiveSurface.c``.

For ``GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE`` only Z is transmitted per cell —
X and Y are implied by column/row index. For
``GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD`` every cell carries its own
x/y/z raw triple, which is what an un-resampled surface needs.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from .mounting import SensorMounting

NM_TO_MM = 1e-6
UM_TO_MM = 1e-3

#: Raw 16-bit sentinel for "no data" — see module docstring.
INVALID_RANGE_16BIT = -32768


@dataclass
class SurfaceScan:
    """One Gocator surface, in engineering units.

    **Frames.** The stored ``x_mm``/``y_mm``/``z_mm`` arrays are in the
    *sensor's* frame, where X runs across the laser line and Y along travel.
    Those are the sensor's own names for its optics and generally do **not**
    line up with the gantry's axes — on this rig a gantry move along X shows
    up as sensor Y. Everything user-facing therefore reports the **gantry**
    frame instead: :meth:`to_points`, every ``save_*`` method, and the
    ``gantry_*`` properties. See :mod:`laguna.scanner.mounting`.

    Attributes:
        z_mm: (rows, cols) float array of heights in mm, with invalid points
            as NaN. Rows run along sensor Y (travel), columns along sensor X.
        x_mm: (cols,) column centre positions in mm, for a uniform surface;
            or (rows, cols) per-cell X values for an un-resampled point cloud.
        y_mm: (rows,) row positions in mm, or (rows, cols) for a point cloud.
        metadata: Acquisition context — message type, resolutions/offsets,
            configured travel speed, frame rate, timestamps, etc.
        is_uniform: True when built from a UNIFORM_SURFACE message (implied
            X/Y), False for a SURFACE_POINT_CLOUD message (explicit per-cell
            X/Y).
        mounting: How the sensor sits on the gantry. Defaults to the identity
            map, i.e. gantry frame == sensor frame.
    """

    z_mm: np.ndarray
    x_mm: np.ndarray
    y_mm: np.ndarray
    metadata: Dict[str, Any] = field(default_factory=dict)
    is_uniform: bool = True
    mounting: SensorMounting = field(default_factory=SensorMounting)

    # ------------------------------------------------------------------
    # Gantry frame
    # ------------------------------------------------------------------

    @property
    def grid_axes(self) -> tuple:
        """``(rows, cols)`` as *gantry* axis names, e.g. ``("X", "Y")``.

        The single fact a reader needs to avoid the sensor/gantry axis
        mix-up: which way the grid's rows and columns actually run on the
        machine. Rows are always the travel direction as acquired.
        """
        return self.mounting.grid_axes()

    @property
    def gantry_travel_mm(self) -> np.ndarray:
        """Positions along the *travel* axis, signed into the gantry frame.

        Travel is sensor Y — the direction the gantry actually moved.
        """
        return self.y_mm * self.mounting.signs[1] if self.mounting.signs[1] < 0 else self.y_mm

    @property
    def gantry_across_mm(self) -> np.ndarray:
        """Positions across the laser line, signed into the gantry frame."""
        return self.x_mm * self.mounting.signs[0] if self.mounting.signs[0] < 0 else self.x_mm

    @property
    def gantry_height_mm(self) -> np.ndarray:
        """Height grid, signed into the gantry frame."""
        return self.z_mm * self.mounting.signs[2] if self.mounting.signs[2] < 0 else self.z_mm

    # ------------------------------------------------------------------
    # Derived views
    # ------------------------------------------------------------------

    @property
    def shape(self) -> tuple:
        """(rows, cols) of the height grid."""
        return tuple(self.z_mm.shape)

    @property
    def valid_count(self) -> int:
        """Number of cells holding a real measurement (non-NaN)."""
        return int(np.count_nonzero(~np.isnan(self.z_mm)))

    def to_points(
        self,
        drop_invalid: bool = True,
        dtype: Any = np.float64,
        frame: str = "gantry",
    ) -> np.ndarray:
        """Flatten to an (N, 3) XYZ point cloud in mm, **in gantry frame**.

        For a uniform surface this indexes ``x_mm``/``y_mm`` by the valid
        cells' column/row rather than building full meshgrid arrays and
        masking afterwards. On a 2100x16154 scan that is ~1.7x faster and
        cuts peak memory from ~2.2 GB to ~0.9 GB (at ``dtype=np.float32``),
        which matters because these grids run to hundreds of MB — the old
        path materialised two full-size coordinate grids plus a full-size
        (N, 3) array before dropping ~30% of it. Output is identical.

        Columns are gantry [X, Y, Z] by default, so a point's X is the same
        X you would command with ``move_to(X=...)``. The mounting transform
        is a signed axis permutation, applied by choosing which column each
        sensor component is written into — so it costs nothing over the
        untransformed path.

        Args:
            drop_invalid: Drop points whose Z is NaN (no laser return).
                Keep them only if you specifically want a full-grid array.
            dtype: Output dtype. ``np.float32`` halves memory and is still
                far finer than the sensor's ~12 µm Z repeatability (float32
                resolves ~6e-5 mm at 1000 mm); ``np.float64`` is the
                default only to preserve the previous behaviour exactly.
            frame: ``"gantry"`` (default) or ``"sensor"`` for the raw,
                untransformed sensor axes.

        Returns:
            (N, 3) array of [x, y, z] in mm, in the requested frame.

        Raises:
            ValueError: On an unknown `frame`.
        """
        if frame not in ("gantry", "sensor"):
            raise ValueError(f"frame must be 'gantry' or 'sensor', got {frame!r}")

        # Which output column each sensor axis lands in, and its sign.
        if frame == "sensor" or self.mounting.is_identity:
            cols_for = (0, 1, 2)
            signs = (1.0, 1.0, 1.0)
        else:
            cols_for = self.mounting.axis_index
            signs = self.mounting.signs

        if not drop_invalid:
            if self.is_uniform:
                xx, yy = np.meshgrid(self.x_mm, self.y_mm)
            else:
                xx, yy = self.x_mm, self.y_mm
            components = (xx.ravel(), yy.ravel(), self.z_mm.ravel())
            points = np.empty((components[0].size, 3), dtype=dtype)
        else:
            valid = ~np.isnan(self.z_mm)
            rows, cols = np.nonzero(valid)
            if self.is_uniform:
                components = (self.x_mm[cols], self.y_mm[rows], self.z_mm[valid])
            else:
                components = (self.x_mm[valid], self.y_mm[valid], self.z_mm[valid])
            points = np.empty((rows.size, 3), dtype=dtype)

        for sensor_i, (out_col, sign) in enumerate(zip(cols_for, signs)):
            points[:, out_col] = components[sensor_i]
            if sign < 0:
                points[:, out_col] *= -1
        return points

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def save_csv(
        self,
        path: str | Path,
        drop_invalid: bool = True,
        points: Optional[np.ndarray] = None,
    ) -> Path:
        """Write the point cloud as ``x_mm,y_mm,z_mm`` CSV.

        Slow and enormous for full-resolution scans — ~55 s and ~1.2 GB for
        a 24M-point scan, versus ~0.5 s and 14 MB for :meth:`save_las` with
        compression. Prefer LAZ (or npz for reprocessing) unless something
        downstream genuinely needs text.

        Args:
            path: Destination .csv path.
            drop_invalid: Drop points with no laser return.
            points: Precomputed ``to_points()`` output, to avoid recomputing
                it when writing several formats — see
                ``GocatorScanner.save_scan``.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if points is None:
            points = self.to_points(drop_invalid=drop_invalid)
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x_mm", "y_mm", "z_mm"])
            writer.writerows(points)
        return path

    def save_las(
        self,
        path: str | Path,
        compressed: Optional[bool] = None,
        scale_mm: float = 1e-4,
        points: Optional[np.ndarray] = None,
    ) -> Path:
        """Write the point cloud as an ASPRS LAS/LAZ file.

        The best general-purpose export here: LAS stores coordinates as
        scaled 32-bit integers with a per-file scale/offset, which suits
        this sensor exactly (its native output is already 16-bit counts
        with a resolution/offset). Measured on a 23.8M-point scan —
        LAZ 0.5 s / 14 MB, LAS 2.0 s / 476 MB, versus CSV 55 s / 1.2 GB and
        PLY 1.2 s / 286 MB. Reads directly into CloudCompare, PDAL, QGIS,
        and laspy.

        Coordinates are written in **millimetres**, matching every other
        export here. LAS records units in its header only via a CRS, and
        these scans have no georeferencing, so consumers must treat the
        values as mm — noted in the header's ``generating_software``.

        Args:
            path: Destination path. ``.laz`` selects compression unless
                `compressed` says otherwise.
            compressed: Force LAZ (True) or plain LAS (False). Default None
                infers from the suffix. LAZ needs a backend —
                ``pip install 'laspy[lazrs]'``.
            scale_mm: LAS coordinate quantum, mm. The default 1e-4 mm
                (0.1 µm) gives ~5e-5 mm round-trip error, well under the
                2690's 12 µm Z repeatability, while keeping the int32
                coordinate range at +-214 m.
            points: Precomputed ``to_points()`` output — see
                ``GocatorScanner.save_scan``.

        Raises:
            ImportError: If laspy isn't installed. It is an optional
                dependency: ``pip install 'laguna[scanner]'``.
        """
        try:
            import laspy
        except ImportError as e:  # pragma: no cover - depends on env
            raise ImportError(
                "save_las() needs laspy, an optional dependency: "
                "pip install 'laguna[scanner]'  (or \"laspy[lazrs]\" for LAZ)"
            ) from e

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if compressed is None:
            compressed = path.suffix.lower() == ".laz"
        if points is None:
            points = self.to_points(drop_invalid=True, dtype=np.float32)

        header = laspy.LasHeader(point_format=0, version="1.4")
        header.scales = np.array([scale_mm] * 3)
        # Offsets must make every scaled coordinate fit in int32. Flooring
        # the minimum keeps the stored integers small and positive-ish; an
        # empty cloud has no minimum, so fall back to the origin.
        if len(points):
            header.offsets = np.floor(points.min(axis=0)).astype(np.float64)
        else:
            header.offsets = np.zeros(3)
        # LAS caps this field at 32 bytes; laspy truncates (with a warning)
        # anything longer, so keep it short enough to survive intact.
        header.generating_software = "laguna.scanner; units=mm"

        las = laspy.LasData(header)
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]
        las.write(str(path), do_compress=compressed)
        return path

    def save_ply(
        self,
        path: str | Path,
        binary: bool = True,
        points: Optional[np.ndarray] = None,
    ) -> Path:
        """Write the point cloud as a PLY file (CloudCompare/MeshLab readable).

        Args:
            path: Destination .ply path.
            binary: Write binary little-endian (much smaller/faster). ASCII
                is easier to eyeball for small test scans.
            points: Precomputed ``to_points()`` output — see
                ``GocatorScanner.save_scan``.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if points is None:
            points = self.to_points(drop_invalid=True, dtype=np.float32)
        points = points.astype(np.float32, copy=False)

        fmt = "binary_little_endian 1.0" if binary else "ascii 1.0"
        header = (
            "ply\n"
            f"format {fmt}\n"
            f"comment generated by laguna.scanner (Gocator)\n"
            f"element vertex {len(points)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        )
        with open(path, "wb") as f:
            f.write(header.encode("ascii"))
            if binary:
                f.write(points.tobytes())
            else:
                for x, y, z in points:
                    f.write(f"{x} {y} {z}\n".encode("ascii"))
        return path

    def save_npz(self, path: str | Path) -> Path:
        """Write the full grid (including NaNs) plus metadata as .npz.

        Preserves the grid structure that CSV/PLY flatten away — the right
        choice for reprocessing (e.g. re-deriving Y from a corrected velocity).
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        metadata = dict(self.metadata)
        metadata.setdefault("mounting", self.mounting.to_dict())
        metadata.setdefault("grid_axes", list(self.grid_axes))
        np.savez_compressed(
            path,
            z_mm=self.z_mm,
            x_mm=self.x_mm,
            y_mm=self.y_mm,
            is_uniform=self.is_uniform,
            metadata=np.array([repr(metadata)], dtype=object),
        )
        return path

    def rescale_y(self, actual_speed_mm_s: float) -> "SurfaceScan":
        """Return a copy with Y rescaled for a corrected travel speed.

        Encoderless Y spacing is only as good as the travel speed the sensor
        was told (see docs/reference/gocator/GOCATOR_CONCEPTS.md §2c). If the
        gantry's true velocity is later measured to differ, this rescales the
        travel axis without a re-scan.

        Args:
            actual_speed_mm_s: Measured true velocity during the pass.

        Raises:
            ValueError: If the scan's metadata has no configured travel speed
                to scale relative to, or either speed is non-positive.
        """
        configured = self.metadata.get("travel_speed_mm_s")
        if not configured or configured <= 0 or actual_speed_mm_s <= 0:
            raise ValueError(
                "rescale_y() needs a positive 'travel_speed_mm_s' in metadata "
                f"and a positive actual speed; got configured={configured!r}, "
                f"actual={actual_speed_mm_s!r}"
            )
        factor = actual_speed_mm_s / configured
        metadata = dict(self.metadata)
        metadata.update(
            {
                "travel_speed_mm_s": actual_speed_mm_s,
                "y_rescaled_from_mm_s": configured,
                "y_rescale_factor": factor,
            }
        )
        return SurfaceScan(
            z_mm=self.z_mm.copy(),
            x_mm=self.x_mm.copy(),
            y_mm=self.y_mm * factor,
            metadata=metadata,
            is_uniform=self.is_uniform,
            mounting=self.mounting,
        )


# ----------------------------------------------------------------------
# Message -> SurfaceScan conversion
# ----------------------------------------------------------------------


def _scale(raw: np.ndarray, resolution_nm: int, offset_um: int) -> np.ndarray:
    """Apply Gocator's offset/resolution scaling, mapping invalids to NaN."""
    out = raw.astype(np.float64)
    out[raw == INVALID_RANGE_16BIT] = np.nan
    return offset_um * UM_TO_MM + resolution_nm * NM_TO_MM * out


def uniform_surface_to_scan(
    lib,
    msg,
    metadata: Optional[Dict[str, Any]] = None,
    mounting: Optional[SensorMounting] = None,
) -> SurfaceScan:
    """Convert a ``GoUniformSurfaceMsg`` handle into a :class:`SurfaceScan`.

    Args:
        lib: A :class:`laguna.scanner.gosdk.GoSdkLib`.
        msg: The message handle from ``GoDataSet_At``.
        metadata: Extra context to merge into the result's metadata.
    """
    import ctypes

    go = lib.go
    rows = int(go.GoUniformSurfaceMsg_Length(msg))
    cols = int(go.GoUniformSurfaceMsg_Width(msg))

    x_res = int(go.GoUniformSurfaceMsg_XResolution(msg))
    y_res = int(go.GoUniformSurfaceMsg_YResolution(msg))
    z_res = int(go.GoUniformSurfaceMsg_ZResolution(msg))
    x_off = int(go.GoUniformSurfaceMsg_XOffset(msg))
    y_off = int(go.GoUniformSurfaceMsg_YOffset(msg))
    z_off = int(go.GoUniformSurfaceMsg_ZOffset(msg))

    raw = np.empty((rows, cols), dtype=np.int16)
    for row in range(rows):
        row_ptr = go.GoUniformSurfaceMsg_RowAt(msg, row)
        if not row_ptr:
            # Defensive: a NULL row would otherwise fault inside numpy. Treat
            # the whole row as "no data" rather than losing the scan.
            raw[row, :] = INVALID_RANGE_16BIT
            continue
        # Copy immediately — the buffer belongs to the message, which the
        # caller destroys via GoDestroy once the dataset is released.
        raw[row, :] = np.ctypeslib.as_array(
            ctypes.cast(row_ptr, ctypes.POINTER(ctypes.c_int16)), shape=(cols,)
        )

    z_mm = _scale(raw, z_res, z_off)
    x_mm = x_off * UM_TO_MM + x_res * NM_TO_MM * np.arange(cols, dtype=np.float64)
    y_mm = y_off * UM_TO_MM + y_res * NM_TO_MM * np.arange(rows, dtype=np.float64)

    meta: Dict[str, Any] = {
        "message_type": "uniform_surface",
        "rows": rows,
        "cols": cols,
        "x_resolution_nm": x_res,
        "y_resolution_nm": y_res,
        "z_resolution_nm": z_res,
        "x_offset_um": x_off,
        "y_offset_um": y_off,
        "z_offset_um": z_off,
        "x_spacing_mm": x_res * NM_TO_MM,
        "y_spacing_mm": y_res * NM_TO_MM,
    }
    if metadata:
        meta.update(metadata)

    return SurfaceScan(
        z_mm=z_mm, x_mm=x_mm, y_mm=y_mm, metadata=meta, is_uniform=True,
        mounting=mounting or SensorMounting(),
    )


def surface_point_cloud_to_scan(
    lib,
    msg,
    metadata: Optional[Dict[str, Any]] = None,
    mounting: Optional[SensorMounting] = None,
) -> SurfaceScan:
    """Convert a ``GoSurfacePointCloudMsg`` handle into a :class:`SurfaceScan`.

    Unlike the uniform surface, each cell carries its own x/y/z raw triple
    (an un-resampled surface), so X and Y come back as full 2-D arrays.
    """
    import ctypes

    from .gosdk import kPoint3d16s

    go = lib.go
    rows = int(go.GoSurfacePointCloudMsg_Length(msg))
    cols = int(go.GoSurfacePointCloudMsg_Width(msg))

    x_res = int(go.GoSurfacePointCloudMsg_XResolution(msg))
    y_res = int(go.GoSurfacePointCloudMsg_YResolution(msg))
    z_res = int(go.GoSurfacePointCloudMsg_ZResolution(msg))
    x_off = int(go.GoSurfacePointCloudMsg_XOffset(msg))
    y_off = int(go.GoSurfacePointCloudMsg_YOffset(msg))
    z_off = int(go.GoSurfacePointCloudMsg_ZOffset(msg))

    raw_x = np.empty((rows, cols), dtype=np.int16)
    raw_y = np.empty((rows, cols), dtype=np.int16)
    raw_z = np.empty((rows, cols), dtype=np.int16)

    # kPoint3d16s is three contiguous int16s, so a row of them views cleanly
    # as an (cols, 3) int16 array.
    for row in range(rows):
        row_ptr = go.GoSurfacePointCloudMsg_RowAt(msg, row)
        if not row_ptr:
            raw_x[row, :] = raw_y[row, :] = raw_z[row, :] = INVALID_RANGE_16BIT
            continue
        flat = np.ctypeslib.as_array(
            ctypes.cast(row_ptr, ctypes.POINTER(ctypes.c_int16)), shape=(cols * 3,)
        ).reshape(cols, 3)
        raw_x[row, :] = flat[:, 0]
        raw_y[row, :] = flat[:, 1]
        raw_z[row, :] = flat[:, 2]

    z_mm = _scale(raw_z, z_res, z_off)
    x_mm = _scale(raw_x, x_res, x_off)
    y_mm = _scale(raw_y, y_res, y_off)

    meta: Dict[str, Any] = {
        "message_type": "surface_point_cloud",
        "rows": rows,
        "cols": cols,
        "x_resolution_nm": x_res,
        "y_resolution_nm": y_res,
        "z_resolution_nm": z_res,
        "x_offset_um": x_off,
        "y_offset_um": y_off,
        "z_offset_um": z_off,
    }
    if metadata:
        meta.update(metadata)

    return SurfaceScan(
        z_mm=z_mm, x_mm=x_mm, y_mm=y_mm, metadata=meta, is_uniform=False,
        mounting=mounting or SensorMounting(),
    )


# Keep kPoint3d16s importable from this module for callers that only touch
# the conversion layer.
__all__ = [
    "SurfaceScan",
    "SensorMounting",
    "uniform_surface_to_scan",
    "surface_point_cloud_to_scan",
    "INVALID_RANGE_16BIT",
]
