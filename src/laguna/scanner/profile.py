"""Single-profile container plus message-to-array conversion.

A Gocator profile is one instantaneous exposure: a single 2-D line (X across
the laser line, Z height), with no travel axis and therefore none of
:mod:`laguna.scanner.pointcloud`'s 2-D-grid/travel-axis concerns
(``grid_axes``, ``gantry_travel_mm``, ``rescale_y``, ...) — this is why
:class:`GocatorProfile` is its own small container rather than a
``SurfaceScan`` with one row. It still carries the sensor's
:class:`~laguna.scanner.mounting.SensorMounting`, though: a rotated mount
turns sensor X/Z into different gantry axes even for a stationary exposure,
so ``to_points()`` needs it to report gantry-frame coordinates correctly —
see :func:`laguna.frames.orient_gocator_profile` for going one step further,
into experiment coordinates.

Both message flavours use the same raw-count-to-mm scaling as surfaces (see
:func:`laguna.scanner.pointcloud._scale`, reused here):

    value_mm = offset_um / 1000 + resolution_nm / 1e6 * raw_count

For ``GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE`` only Z is transmitted per
sample — X is implied by index. For
``GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD`` every point carries its own
raw (x, y) pair (y = height), at native non-uniform X spacing.
"""

from __future__ import annotations

import csv
import logging
from ctypes import c_void_p
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Optional

import numpy as np

from .mounting import SensorMounting
from .pointcloud import INVALID_RANGE_16BIT, NM_TO_MM, UM_TO_MM, _scale

if TYPE_CHECKING:
    from .gosdk import GoSdkLib

logger = logging.getLogger(__name__)


@dataclass
class GocatorProfile:
    """One Gocator profile: a single X-Z line from one exposure.

    Attributes:
        z_mm: (n,) float array of heights in mm, invalid points as NaN.
        x_mm: (n,) lateral positions in mm — evenly spaced (implied by
            index) for a resampled profile, or explicit per-point for a raw
            one. See ``is_uniform``.
        metadata: Acquisition context — message type, resolutions/offsets,
            timestamps, configured exposure, etc.
        is_uniform: True when built from a UNIFORM_PROFILE message (implied
            X), False for a PROFILE_POINT_CLOUD message (explicit per-point
            X, native non-uniform spacing).
        mounting: How the sensor sits on the gantry. Defaults to the identity
            map, i.e. gantry frame == sensor frame. Rotation only — there's
            no travel-axis sign to carry, unlike SurfaceScan's mounting.
    """

    z_mm: np.ndarray
    x_mm: np.ndarray
    metadata: Dict[str, Any] = field(default_factory=dict)
    is_uniform: bool = True
    mounting: SensorMounting = field(default_factory=SensorMounting)

    @property
    def valid_count(self) -> int:
        """Number of samples holding a real measurement (non-NaN)."""
        return int(np.count_nonzero(~np.isnan(self.z_mm)))

    def to_points(
        self,
        drop_invalid: bool = True,
        dtype: Any = np.float64,
        frame: str = "gantry",
    ) -> np.ndarray:
        """Flatten to an (N, 3) [x, y, z] array in mm.

        The sensor's own Y (travel) is always 0 here — a profile is one
        exposure, nothing moved — but after the mounting rotation, sensor X
        and Z can still land in any of the 3 gantry columns (e.g. this rig's
        90-degree mount puts sensor X in gantry Y), so the output is a full
        (N, 3) array even though only 2 sensor axes ever carry data.

        Args:
            drop_invalid: Drop points whose Z is NaN (no laser return).
            dtype: Output dtype.
            frame: ``"gantry"`` (default) or ``"sensor"`` for the raw,
                untransformed sensor axes.

        Returns:
            (N, 3) array of [x, y, z] in mm, in the requested frame.

        Raises:
            ValueError: On an unknown `frame`.
        """
        if frame not in ("gantry", "sensor"):
            raise ValueError(f"frame must be 'gantry' or 'sensor', got {frame!r}")

        if drop_invalid:
            valid = ~np.isnan(self.z_mm)
            x, z = self.x_mm[valid], self.z_mm[valid]
        else:
            x, z = self.x_mm, self.z_mm

        sensor_points = np.zeros((z.size, 3), dtype=np.float64)
        sensor_points[:, 0] = x
        sensor_points[:, 2] = z

        if frame == "sensor" or self.mounting.is_identity:
            points = sensor_points
        else:
            points = self.mounting.apply_to_points(sensor_points)
        return points.astype(dtype, copy=False)

    def save_csv(
        self,
        path: str | Path,
        drop_invalid: bool = True,
        points: Optional[np.ndarray] = None,
    ) -> Path:
        """Write the profile as ``x_mm,y_mm,z_mm`` CSV, gantry frame.

        Args:
            path: Destination .csv path.
            drop_invalid: Drop points with no laser return.
            points: Precomputed ``to_points()`` output, to avoid recomputing
                it when writing several formats.
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

    def save_npz(self, path: str | Path) -> Path:
        """Write the full profile (including NaNs) plus metadata as .npz.

        Load a saved file back with :meth:`from_npz`.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        metadata = dict(self.metadata)
        metadata.setdefault("mounting", self.mounting.to_dict())
        np.savez_compressed(
            path,
            z_mm=self.z_mm,
            x_mm=self.x_mm,
            is_uniform=self.is_uniform,
            metadata=np.array([repr(metadata)], dtype=object),
        )
        return path

    @classmethod
    def from_npz(cls, path: str | Path) -> "GocatorProfile":
        """Reconstruct a GocatorProfile saved by :meth:`save_npz`.

        Args:
            path: A file written by ``save_npz()``.

        Returns:
            The reconstructed GocatorProfile.
        """
        import ast

        path = Path(path)
        with np.load(path, allow_pickle=True) as data:
            metadata = ast.literal_eval(str(data["metadata"][0]))
            mounting_dict = metadata.pop("mounting", None)
            return cls(
                z_mm=data["z_mm"],
                x_mm=data["x_mm"],
                metadata=metadata,
                is_uniform=bool(data["is_uniform"]),
                mounting=SensorMounting.from_config(mounting_dict),
            )


# ----------------------------------------------------------------------
# Message -> GocatorProfile conversion
# ----------------------------------------------------------------------


def uniform_profile_to_scan(
    lib: "GoSdkLib",
    msg: c_void_p,
    metadata: Optional[Dict[str, Any]] = None,
    mounting: Optional[SensorMounting] = None,
) -> GocatorProfile:
    """Convert a ``GoUniformProfileMsg`` handle into a :class:`GocatorProfile`.

    Args:
        lib: A :class:`laguna.scanner.gosdk.GoSdkLib`.
        msg: The message handle from ``GoDataSet_At``.
        metadata: Extra context to merge into the result's metadata.
        mounting: Sensor-to-gantry axis transform. Defaults to identity.
    """
    import ctypes

    go = lib.go
    width = int(go.GoUniformProfileMsg_Width(msg))
    x_res = int(go.GoUniformProfileMsg_XResolution(msg))
    z_res = int(go.GoUniformProfileMsg_ZResolution(msg))
    x_off = int(go.GoUniformProfileMsg_XOffset(msg))
    z_off = int(go.GoUniformProfileMsg_ZOffset(msg))

    # A message can in principle batch several profiles (multi-line/
    # multiplexed sensors); the 2690 emits one per exposure. Take the first
    # and log if that assumption doesn't hold, rather than silently
    # discarding the rest.
    count = int(go.GoUniformProfileMsg_Count(msg))
    if count != 1:
        logger.warning(
            "GoUniformProfileMsg reports %d profiles in one message; "
            "using only the first (index 0).",
            count,
        )

    row_ptr = go.GoUniformProfileMsg_At(msg, 0)
    if not row_ptr:
        raw = np.full(width, INVALID_RANGE_16BIT, dtype=np.int16)
    else:
        raw = np.ctypeslib.as_array(
            ctypes.cast(row_ptr, ctypes.POINTER(ctypes.c_int16)), shape=(width,)
        ).copy()  # copy immediately — buffer belongs to the message

    z_mm = _scale(raw, z_res, z_off)
    x_mm = x_off * UM_TO_MM + x_res * NM_TO_MM * np.arange(width, dtype=np.float64)

    meta: Dict[str, Any] = {
        "message_type": "uniform_profile",
        "width": width,
        "x_resolution_nm": x_res,
        "z_resolution_nm": z_res,
        "x_offset_um": x_off,
        "z_offset_um": z_off,
        "x_spacing_mm": x_res * NM_TO_MM,
    }
    if metadata:
        meta.update(metadata)

    return GocatorProfile(
        z_mm=z_mm, x_mm=x_mm, metadata=meta, is_uniform=True,
        mounting=mounting or SensorMounting(),
    )


def profile_point_cloud_to_scan(
    lib: "GoSdkLib",
    msg: c_void_p,
    metadata: Optional[Dict[str, Any]] = None,
    mounting: Optional[SensorMounting] = None,
) -> GocatorProfile:
    """Convert a ``GoProfilePointCloudMsg`` handle into a :class:`GocatorProfile`.

    Unlike the resampled profile, each point carries its own raw (x, y)
    pair (y = height) at native, non-uniform X spacing.

    Args:
        lib: A :class:`laguna.scanner.gosdk.GoSdkLib`.
        msg: The message handle from ``GoDataSet_At``.
        metadata: Extra context to merge into the result's metadata.
        mounting: Sensor-to-gantry axis transform. Defaults to identity.
    """
    import ctypes

    go = lib.go
    width = int(go.GoProfilePointCloudMsg_Width(msg))
    x_res = int(go.GoProfilePointCloudMsg_XResolution(msg))
    z_res = int(go.GoProfilePointCloudMsg_ZResolution(msg))
    x_off = int(go.GoProfilePointCloudMsg_XOffset(msg))
    z_off = int(go.GoProfilePointCloudMsg_ZOffset(msg))

    # See uniform_profile_to_scan()'s matching comment — same "batches
    # several profiles, 2690 emits one" caveat.
    count = int(go.GoProfilePointCloudMsg_Count(msg))
    if count != 1:
        logger.warning(
            "GoProfilePointCloudMsg reports %d profiles in one message; "
            "using only the first (index 0).",
            count,
        )

    row_ptr = go.GoProfilePointCloudMsg_At(msg, 0)
    if not row_ptr:
        raw_x = np.full(width, INVALID_RANGE_16BIT, dtype=np.int16)
        raw_y = np.full(width, INVALID_RANGE_16BIT, dtype=np.int16)
    else:
        # Each row entry is a kPoint16s {x, y} pair — two contiguous int16s
        # — so the row views cleanly as a (width, 2) int16 array, same
        # trick surface_point_cloud_to_scan() uses for kPoint3d16s rows.
        flat = np.ctypeslib.as_array(
            ctypes.cast(row_ptr, ctypes.POINTER(ctypes.c_int16)),
            shape=(width * 2,),
        ).reshape(width, 2).copy()  # copy immediately — buffer belongs to the message
        raw_x = flat[:, 0]
        raw_y = flat[:, 1]

    x_mm = _scale(raw_x, x_res, x_off)
    z_mm = _scale(raw_y, z_res, z_off)

    meta: Dict[str, Any] = {
        "message_type": "profile_point_cloud",
        "width": width,
        "x_resolution_nm": x_res,
        "z_resolution_nm": z_res,
        "x_offset_um": x_off,
        "z_offset_um": z_off,
    }
    if metadata:
        meta.update(metadata)

    return GocatorProfile(
        z_mm=z_mm, x_mm=x_mm, metadata=meta, is_uniform=False,
        mounting=mounting or SensorMounting(),
    )


__all__ = [
    "GocatorProfile",
    "uniform_profile_to_scan",
    "profile_point_cloud_to_scan",
]
