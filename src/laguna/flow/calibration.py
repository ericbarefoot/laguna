"""Pump discharge calibration (VFD frequency -> measured flow rate).

`set_flowrate(lpm)` needs to convert a desired discharge rate (L/min)
into a VFD drive frequency (Hz) that actually produces it. The relation
is pump/plumbing-specific and has to be measured, not assumed — see
`docs/reference/pump_calibration.md` for the full story on why the
hardcoded `C0`/`C1`/`C2` quadratic in `FlowController.__init__` turned
out to be wrong on first live test (a 1 L/min request computed to 63 Hz,
clamped to the VFD's 60 Hz max).

A calibration run commands a series of known frequencies and measures
the resulting discharge at each (bucket-and-stopwatch, a flow meter,
whatever's available) — frequency is the precisely-controlled
(independent) variable, discharge is the noisy measured (dependent) one,
so the fit direction here is discharge_lpm = f(freq_hz), fit against
those points. Going the other way (given a target discharge, what
frequency to command) inverts that same fitted curve numerically rather
than fitting a second, backwards regression — fitting x as a function of
y when y is the one with measurement noise gives a worse fit than fitting
y as a function of x and inverting it.

This module is device-agnostic — it only knows about (freq_hz,
discharge_lpm) pairs, a quadratic fit through them, and CSV persistence.
Mirrors the shape of `laguna.rangefinder.calibration.LinearCalibration`
(same fit/apply/to_csv/from_csv pattern) rather than inventing a new one.
"""

from __future__ import annotations

import ast
import csv
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Union

import numpy as np


@dataclass
class PumpCalibrationPoint:
    """A single (commanded frequency, measured discharge) calibration pair.

    Attributes:
        freq_hz: VFD drive frequency commanded for this point.
        discharge_lpm: Measured flow rate at that frequency, in L/min.
    """

    freq_hz: float
    discharge_lpm: float


@dataclass
class PumpCalibration:
    """A fitted quadratic curve mapping VFD frequency to discharge rate.

    Represents discharge_lpm = a*freq_hz^2 + b*freq_hz + c for a specific
    pump/plumbing configuration, fitted against measured calibration
    points. `hz_max` bounds the physically valid frequency range (the
    VFD's own limit, and the range the fit was actually measured over —
    extrapolating well beyond it is not reliable).

    Attributes:
        device: Identifier for the pump/plumbing configuration calibrated
            (e.g. "flume pump - main inlet, 2026 configuration").
        coeffs: [a, b, c] quadratic coefficients, highest power first
            (same order numpy.polyfit/numpy.roots use).
        hz_max: Maximum valid drive frequency (VFD hardware limit).
        r_squared: Coefficient of determination (1.0 = perfect fit).
        points: Calibration points used to fit the curve.
        created_at: ISO-format timestamp when the calibration was created.
    """

    device: str
    coeffs: List[float]
    hz_max: float = 60.0
    r_squared: float = float("nan")
    points: List[PumpCalibrationPoint] = field(default_factory=list)
    created_at: str = ""

    def lpm_for_hz(self, freq_hz: float) -> float:
        """Predict discharge (L/min) for a given drive frequency (Hz).

        This is the fit's native direction — a direct polynomial
        evaluation, no inversion involved.
        """
        a, b, c = self.coeffs
        return a * freq_hz**2 + b * freq_hz + c

    def hz_for_lpm(self, discharge_lpm: float) -> float:
        """Find the drive frequency (Hz) that produces a target discharge (L/min).

        Inverts the fitted curve numerically (solving
        a*hz^2 + b*hz + (c - discharge_lpm) = 0 for hz) rather than
        fitting a second regression in the opposite direction — see this
        module's docstring for why. Clamps the result to [0, hz_max].

        Raises:
            ValueError: If no real root falls within [0, hz_max] — the
                requested discharge isn't achievable within the
                calibrated (and physically valid) frequency range.
        """
        a, b, c = self.coeffs
        roots = np.roots([a, b, c - discharge_lpm])
        valid = sorted(
            r.real for r in roots if abs(r.imag) < 1e-6 and 0.0 <= r.real <= self.hz_max
        )
        if not valid:
            raise ValueError(
                f"No valid frequency in [0, {self.hz_max}] Hz produces "
                f"{discharge_lpm} L/min under this calibration ({self.device!r})"
            )
        return valid[0]

    def residuals_lpm(self) -> List[float]:
        """Compute per-point fit residuals (predicted - measured) in L/min.

        Useful for assessing fit quality beyond r_squared alone — e.g. a
        residual pattern that isn't randomly scattered suggests the
        underlying relationship isn't actually quadratic over this range.
        """
        return [self.lpm_for_hz(p.freq_hz) - p.discharge_lpm for p in self.points]

    @classmethod
    def fit(
        cls,
        device: str,
        points: Sequence[PumpCalibrationPoint],
        hz_max: float = 60.0,
    ) -> "PumpCalibration":
        """Least-squares fit a quadratic curve through the given calibration points.

        Args:
            device: Identifier for the resulting calibration.
            points: At least 3 (freq_hz, discharge_lpm) pairs — a
                quadratic fit is underdetermined with fewer.
            hz_max: VFD hardware frequency limit (used later by
                hz_for_lpm() to reject unreachable targets).

        Returns:
            Fitted PumpCalibration with coeffs, r_squared, and points.

        Raises:
            ValueError: If fewer than 3 points are given.
        """
        if len(points) < 3:
            raise ValueError(
                f"Need at least 3 calibration points to fit a quadratic, got {len(points)}"
            )
        hz = np.array([p.freq_hz for p in points], dtype=float)
        lpm = np.array([p.discharge_lpm for p in points], dtype=float)
        coeffs = np.polyfit(hz, lpm, 2)
        predicted = np.polyval(coeffs, hz)
        ss_res = float(np.sum((lpm - predicted) ** 2))
        ss_tot = float(np.sum((lpm - np.mean(lpm)) ** 2))
        r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
        return cls(
            device=device,
            coeffs=[float(c) for c in coeffs],
            hz_max=hz_max,
            r_squared=r_squared,
            points=list(points),
            created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    def to_csv(self, path: Union[str, Path]) -> None:
        """Save calibration (parameters and points) to a CSV file.

        Args:
            path: File path to write (metadata, blank line, then points).
        """
        path = Path(path)
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["# laguna pump calibration"])
            writer.writerow(["device", self.device])
            writer.writerow(["coeffs", repr(self.coeffs)])
            writer.writerow(["hz_max", repr(self.hz_max)])
            writer.writerow(["r_squared", repr(self.r_squared)])
            writer.writerow(["created_at", self.created_at])
            writer.writerow([])
            writer.writerow(["freq_hz", "discharge_lpm"])
            for p in self.points:
                writer.writerow([repr(p.freq_hz), repr(p.discharge_lpm)])

    @classmethod
    def from_csv(cls, path: Union[str, Path]) -> "PumpCalibration":
        """Load a calibration from a CSV file written by to_csv().

        Args:
            path: File path to read.

        Returns:
            PumpCalibration instance with metadata and points.

        Raises:
            ValueError: If required metadata (coeffs) is missing.
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
        if i < len(rows) and rows[i][:1] == ["freq_hz"]:
            i += 1  # skip the points header row

        if "coeffs" not in meta:
            raise ValueError(f"{path} does not look like a calibration file written by to_csv()")

        points = [
            PumpCalibrationPoint(freq_hz=float(row[0]), discharge_lpm=float(row[1]))
            for row in rows[i:]
            if row
        ]
        return cls(
            device=meta.get("device", "unknown"),
            coeffs=[float(c) for c in ast.literal_eval(meta["coeffs"])],
            hz_max=float(meta["hz_max"]) if meta.get("hz_max") else 60.0,
            r_squared=float(meta["r_squared"]) if meta.get("r_squared") else float("nan"),
            points=points,
            created_at=meta.get("created_at", ""),
        )
