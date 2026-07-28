"""Linear calibration for rangefinder devices (OD2000, WTT12L PowerProx).

Both rangefinders are mounted close to vertical but not perfectly plumb, so
a raw reading needs a linear (slope + intercept) transform to real-world z
height, not just a fixed offset — the slope absorbs the mount angle (over
the sensor's actual working range, off-vertical mounting scales the raw
reading roughly linearly relative to true height) and the intercept
absorbs any sensor zero-offset. A single 2-parameter fit against a series
of known-height reference blocks captures both at once, which is simpler
and more robust than trying to measure the mount angle directly and
correct for it separately.

This module is device-agnostic — it only knows about (raw_value,
known_height_mm) pairs, a line fit through them, and CSV persistence.
Device-specific *reading* (what "raw_value" means for the OD2000 vs. the
WTT12L PowerProx, and how to pull it off the AL1342) lives in
scripts/calibrate_rangefinder.py, not here, so this stays testable without
network access.
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Sequence, Union

import numpy as np


@dataclass
class CalibrationPoint:
    """One (known real-world height, raw sensor reading) pair."""

    known_height_mm: float
    raw_value: float


@dataclass
class LinearCalibration:
    """A fitted real_height_mm = slope * raw_value + intercept transform.

    Args:
        device: Free-form device identifier, e.g. "od2000" or
            "wtt12l_powerprox" — stored for reference, not interpreted by
            this module.
        slope: Fitted slope.
        intercept: Fitted intercept.
        r_squared: Coefficient of determination of the fit against
            `points`. 1.0 for an exact fit (also the trivial case with
            exactly 2 points); lower values mean the raw/height
            relationship isn't very linear, or the readings were noisy.
        points: The calibration points the fit was computed from.
        created_at: ISO-ish local timestamp string, set automatically by
            fit().
    """

    device: str
    slope: float
    intercept: float
    r_squared: float
    points: List[CalibrationPoint] = field(default_factory=list)
    created_at: str = ""

    def apply(self, raw_value: float) -> float:
        """Transform a raw sensor reading to real-world z height (mm)."""
        return self.slope * raw_value + self.intercept

    def residuals_mm(self) -> List[float]:
        """Per-point (predicted - known) error in mm, for sanity-checking
        fit quality beyond the single r_squared number."""
        return [self.apply(p.raw_value) - p.known_height_mm for p in self.points]

    @classmethod
    def fit(cls, device: str, points: Sequence[CalibrationPoint]) -> "LinearCalibration":
        """Least-squares fit a line through the given calibration points.

        Raises:
            ValueError: If fewer than 2 points are given (a line needs at
                least 2 points; r_squared is trivially 1.0 with exactly 2
                and isn't a meaningful quality signal until you have 3+).
        """
        if len(points) < 2:
            raise ValueError(
                f"Need at least 2 calibration points to fit a line, got {len(points)}"
            )
        raw = np.array([p.raw_value for p in points], dtype=float)
        known = np.array([p.known_height_mm for p in points], dtype=float)
        slope, intercept = np.polyfit(raw, known, 1)
        predicted = slope * raw + intercept
        ss_res = float(np.sum((known - predicted) ** 2))
        ss_tot = float(np.sum((known - np.mean(known)) ** 2))
        r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
        return cls(
            device=device,
            slope=float(slope),
            intercept=float(intercept),
            r_squared=r_squared,
            points=list(points),
            created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    def to_csv(self, path: Union[str, Path]) -> None:
        """Save this calibration (fit parameters + source points) to CSV.

        The file has a small metadata block, a blank line, then the
        calibration points — see from_csv() for the exact format this
        round-trips through.
        """
        path = Path(path)
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["# laguna rangefinder calibration"])
            writer.writerow(["device", self.device])
            writer.writerow(["slope", repr(self.slope)])
            writer.writerow(["intercept", repr(self.intercept)])
            writer.writerow(["r_squared", repr(self.r_squared)])
            writer.writerow(["created_at", self.created_at])
            writer.writerow([])
            writer.writerow(["known_height_mm", "raw_value"])
            for p in self.points:
                writer.writerow([repr(p.known_height_mm), repr(p.raw_value)])

    @classmethod
    def from_csv(cls, path: Union[str, Path]) -> "LinearCalibration":
        """Load a calibration previously saved with to_csv().

        Raises:
            ValueError: If required metadata (slope, intercept) is missing
                — i.e. the file isn't one to_csv() wrote.
        """
        path = Path(path)
        with path.open("r", newline="") as f:
            rows = [row for row in csv.reader(f)]

        i = 1 if rows and rows[0] and rows[0][0].startswith("#") else 0
        meta = {}
        while i < len(rows) and rows[i]:
            key, *rest = rows[i]
            meta[key] = rest[0] if rest else ""
            i += 1
        i += 1  # skip the blank separator line
        if i < len(rows) and rows[i][:1] == ["known_height_mm"]:
            i += 1  # skip the points header row

        if "slope" not in meta or "intercept" not in meta:
            raise ValueError(f"{path} does not look like a calibration file written by to_csv()")

        points = [
            CalibrationPoint(known_height_mm=float(row[0]), raw_value=float(row[1]))
            for row in rows[i:]
            if row
        ]
        return cls(
            device=meta.get("device", "unknown"),
            slope=float(meta["slope"]),
            intercept=float(meta["intercept"]),
            r_squared=float(meta["r_squared"]) if meta.get("r_squared") else float("nan"),
            points=points,
            created_at=meta.get("created_at", ""),
        )
