"""Region shapes for post-processing, in the scan's own coordinates (mm).

Every shape can test which points lie inside it, so one definition drops
points before ground filtering and also blanks the matching DEM cells
afterwards. That way an excluded area is never filled in by interpolation.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from matplotlib.path import Path


@dataclass
class Rect:
    """Axis-aligned rectangle.

    Attributes:
        xmin: Left edge, mm.
        ymin: Bottom edge, mm.
        xmax: Right edge, mm.
        ymax: Top edge, mm.
        label: Free-text name, ignored by processing.
    """

    xmin: float
    ymin: float
    xmax: float
    ymax: float
    label: str = ""

    def __post_init__(self):
        """Coerce edges to float and reject empty or inverted rectangles."""
        self.xmin, self.ymin = float(self.xmin), float(self.ymin)
        self.xmax, self.ymax = float(self.xmax), float(self.ymax)
        if self.xmin >= self.xmax or self.ymin >= self.ymax:
            raise ValueError(f"degenerate rectangle: {self}")

    def contains(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Return a boolean mask of the points inside (edges included)."""
        return (x >= self.xmin) & (x <= self.xmax) & (y >= self.ymin) & (y <= self.ymax)

    def to_dict(self) -> dict:
        """Return the YAML-ready form, tagged ``type: rect``."""
        d = {
            "type": "rect",
            "xmin": self.xmin,
            "ymin": self.ymin,
            "xmax": self.xmax,
            "ymax": self.ymax,
        }
        if self.label:
            d["label"] = self.label
        return d


@dataclass
class Circle:
    """Circle given by centre and radius.

    Attributes:
        x: Centre x, mm.
        y: Centre y, mm.
        r: Radius, mm.
        label: Free-text name, ignored by processing.
    """

    x: float
    y: float
    r: float
    label: str = ""

    def __post_init__(self):
        """Coerce to float and reject non-positive radii."""
        self.x, self.y, self.r = float(self.x), float(self.y), float(self.r)
        if self.r <= 0:
            raise ValueError(f"circle radius must be positive: {self}")

    def contains(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Return a boolean mask of the points inside (edge included)."""
        return (x - self.x) ** 2 + (y - self.y) ** 2 <= self.r**2

    def to_dict(self) -> dict:
        """Return the YAML-ready form, tagged ``type: circle``."""
        d = {"type": "circle", "x": self.x, "y": self.y, "r": self.r}
        if self.label:
            d["label"] = self.label
        return d


@dataclass
class Polygon:
    """Simple polygon; the closing edge back to the first vertex is implied.

    Attributes:
        points: ``(x, y)`` vertices, mm.
        label: Free-text name, ignored by processing.
    """

    points: list[tuple[float, float]] = field(default_factory=list)
    label: str = ""

    def __post_init__(self):
        """Coerce vertices to float tuples and require at least three."""
        self.points = [(float(px), float(py)) for px, py in self.points]
        if len(self.points) < 3:
            raise ValueError("polygon needs at least 3 vertices")

    def contains(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Return a boolean mask of the points inside."""
        return Path(np.asarray(self.points)).contains_points(np.column_stack([x, y]))

    def to_dict(self) -> dict:
        """Return the YAML-ready form, tagged ``type: polygon``."""
        d = {"type": "polygon", "points": [[round(px, 3), round(py, 3)] for px, py in self.points]}
        if self.label:
            d["label"] = self.label
        return d


Shape = Rect | Circle | Polygon

_SHAPES = {"rect": Rect, "circle": Circle, "polygon": Polygon}


def shape_from_dict(d: dict) -> Shape:
    """Build a shape from its ``to_dict()`` form.

    Args:
        d: Mapping with a ``type`` key (``rect``, ``circle`` or ``polygon``)
            plus that shape's fields.

    Returns:
        The shape.

    Raises:
        ValueError: If ``type`` is missing or unknown.
    """
    d = dict(d)
    kind = d.pop("type", None)
    if kind not in _SHAPES:
        raise ValueError(f"unknown shape type {kind!r}; expected one of {sorted(_SHAPES)}")
    return _SHAPES[kind](**d)


def keep_mask(
    x: np.ndarray, y: np.ndarray, roi: Rect, include: list[Shape], exclude: list[Shape]
) -> np.ndarray:
    """Return which points survive the ROI, include and exclude shapes.

    A point is kept if it is inside the ROI, inside at least one include
    shape (when there are any), and outside every exclude shape.
    """
    keep = roi.contains(x, y)
    if include:
        inc = np.zeros_like(keep)
        for s in include:
            inc |= s.contains(x, y)
        keep &= inc
    for s in exclude:
        keep &= ~s.contains(x, y)
    return keep
