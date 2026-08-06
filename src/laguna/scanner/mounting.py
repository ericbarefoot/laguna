"""Mapping between the Gocator's own axes and the gantry's axes.

**Read this before interpreting any scan coordinate.**

The sensor names its axes from its own optics, not from the machine it is
bolted to:

  - sensor **X** runs *across the laser line* (the ~2 m fan)
  - sensor **Y** runs along *travel* — whichever way the target moves
  - sensor **Z** is *range* (height / standoff)

On this rig the sensor is mounted rotated 90 degrees about Z, so a gantry
move along **X** produces the sensor's **Y** axis, and the laser fan lies
along gantry **Y**. Confirmed from two scans on 2026-08-02: commanding
``axis="X"`` for 200 mm and 300 mm produced sensor-Y spans of 199.8 mm and
299.9 mm, while sensor X held a constant 2003 mm — the active-area width, not
anything the gantry did.

That is exactly the trap this module exists to close: without a transform,
``move_to(X=...)`` yields a picture whose *Y* axis is the motion, and every
downstream reader has to remember the swap.

Handedness matters, and is easy to get wrong
--------------------------------------------
A bare X<->Y swap is **not** a rotation — its matrix determinant is -1, so it
*mirrors* the data. Real geometry then comes back as its own reflection,
which is both wrong and hard to spot. A physically achievable mounting is
always a proper rotation (determinant +1), so one axis must also flip sign.
:class:`SensorMounting` rejects mirroring maps outright rather than letting
a silent reflection through.

Config form::

    gocator:
      mounting:
        scan_x: -Y      # sensor X (across the laser) -> gantry -Y
        scan_y: +X      # sensor Y (travel)           -> gantry +X
        scan_z: +Z      # sensor Z (range)            -> gantry +Z

The default is the identity map (sensor frame == gantry frame), so behaviour
is unchanged unless a mounting is configured.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import numpy as np

#: Gantry axis order used for the (N, 3) point columns and for `axis_index`.
GANTRY_AXES = ("X", "Y", "Z")

#: Sensor axis keys, in the order they appear in a config mapping.
SENSOR_AXES = ("scan_x", "scan_y", "scan_z")


def _parse_target(spec: Any, key: str) -> Tuple[int, float]:
    """Parse ``"+Y"`` / ``"-x"`` / ``"Y"`` into (gantry axis index, sign)."""
    if not isinstance(spec, str):
        raise ValueError(
            f"mounting.{key}={spec!r} must be a string like '+Y', '-X' or 'Z'"
        )
    text = spec.strip().upper()
    sign = 1.0
    if text.startswith(("+", "-")):
        sign = -1.0 if text[0] == "-" else 1.0
        text = text[1:]
    if text not in GANTRY_AXES:
        raise ValueError(
            f"mounting.{key}={spec!r} must name a gantry axis "
            f"({', '.join(GANTRY_AXES)}), optionally signed"
        )
    return GANTRY_AXES.index(text), sign


class SensorMounting:
    """How the sensor's X/Y/Z sit in the gantry's X/Y/Z.

    Each axis maps to a signed gantry axis, e.g. ``"-Y"``. Defaults form the
    identity map. See ``__init__`` for the per-axis parameters.

    Raises:
        ValueError: If two sensor axes map onto the same gantry axis, or if
            the result mirrors rather than rotates (see the module
            docstring — a mirrored map cannot describe a real mounting).
    """

    def __init__(
        self,
        scan_x: str = "+X",
        scan_y: str = "+Y",
        scan_z: str = "+Z",
    ) -> None:
        """Initialize sensor-to-gantry axis mapping.

        Args:
            scan_x: Sensor X axis (across laser) target in gantry coords.
            scan_y: Sensor Y axis (along travel) target in gantry coords.
            scan_z: Sensor Z axis (range/standoff) target in gantry coords.
        """
        self.spec: Dict[str, str] = {
            "scan_x": scan_x,
            "scan_y": scan_y,
            "scan_z": scan_z,
        }
        targets = [_parse_target(v, k) for k, v in self.spec.items()]
        #: gantry column index for each sensor axis, in scan_x/y/z order
        self.axis_index: Tuple[int, int, int] = tuple(t[0] for t in targets)
        #: sign applied to each sensor axis
        self.signs: Tuple[float, float, float] = tuple(t[1] for t in targets)

        if len(set(self.axis_index)) != 3:
            used = [GANTRY_AXES[i] for i in self.axis_index]
            raise ValueError(
                f"mounting maps two sensor axes onto the same gantry axis "
                f"({used}); each of {GANTRY_AXES} must be used exactly once"
            )

        matrix = np.zeros((3, 3))
        for sensor_i, (gantry_i, sign) in enumerate(zip(self.axis_index, self.signs)):
            matrix[gantry_i, sensor_i] = sign
        self.matrix = matrix

        det = float(np.linalg.det(matrix))
        if det < 0:
            raise ValueError(
                f"mounting {self.spec} mirrors the data (determinant {det:+.0f}) "
                "rather than rotating it. A bare axis swap does this: swapping "
                "two axes flips handedness, so real geometry would come back "
                "reflected. A physical mounting is always a proper rotation — "
                "flip the sign of exactly one axis (e.g. use '-Y' instead of "
                "'+Y') to fix it."
            )

    # ------------------------------------------------------------------

    @property
    def is_identity(self) -> bool:
        """True if sensor and gantry frames coincide."""
        return self.axis_index == (0, 1, 2) and self.signs == (1.0, 1.0, 1.0)

    def gantry_axis_of(self, sensor_axis: str) -> str:
        """Signed gantry axis a sensor axis maps to, e.g. ``"-Y"``."""
        i = SENSOR_AXES.index(sensor_axis)
        sign = "-" if self.signs[i] < 0 else "+"
        return f"{sign}{GANTRY_AXES[self.axis_index[i]]}"

    def to_dict(self) -> Dict[str, str]:
        """Round-trippable form, for scan metadata."""
        return dict(self.spec)

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]]) -> "SensorMounting":
        """Build from a ``gocator.mounting`` config block (None -> identity)."""
        if not config:
            return cls()
        unknown = [k for k in config if k not in SENSOR_AXES]
        if unknown:
            raise ValueError(
                f"unknown mounting key(s) {unknown}; expected any of "
                f"{list(SENSOR_AXES)}"
            )
        return cls(**{k: config[k] for k in SENSOR_AXES if k in config})

    # ------------------------------------------------------------------

    def apply_to_points(self, sensor_points: np.ndarray) -> np.ndarray:
        """Map an (N, 3) sensor-frame point array into gantry frame."""
        if self.is_identity:
            return sensor_points
        out = np.empty_like(sensor_points)
        for sensor_i, (gantry_i, sign) in enumerate(zip(self.axis_index, self.signs)):
            out[:, gantry_i] = sensor_points[:, sensor_i]
            if sign < 0:
                out[:, gantry_i] *= -1
        return out

    def grid_axes(self) -> Tuple[str, str]:
        """Which gantry axes the scan grid's (rows, cols) index.

        Rows run along sensor Y (travel) and columns along sensor X (across
        the laser) — that is how the sensor delivers a surface, and laguna
        keeps it that way. This reports what those become in gantry terms,
        so a reader never has to re-derive it: with the 90-degree mounting
        above it returns ``("X", "Y")``, i.e. rows step along gantry X.
        """
        return (
            GANTRY_AXES[self.axis_index[1]],   # rows  <- sensor Y
            GANTRY_AXES[self.axis_index[0]],   # cols  <- sensor X
        )
