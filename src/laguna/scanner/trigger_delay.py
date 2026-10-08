"""Measure and calibrate the trigger delay of a gantry-coordinated scan.

``GocatorScanner.scan_with_gantry()`` starts the move, waits for the axis to
come off its acceleration ramp, then fires the software trigger. That wait is
computed from the axis kinematics and assumes the axis starts moving the
instant the move is commanded. It doesn't: there is a latency between the
command and the motion (serial link, controller, jerk-limited ramp). The
trigger therefore fires that much early, the surface is anchored to a position
the axis hasn't reached, and each scan is displaced along its own travel
direction by ``speed * latency``.

A forward and a reverse pass over the same block are displaced in *opposite*
directions, so the block appears shifted between them by
``reverse - forward = -2 * speed * latency``. That difference is the thing to
measure; this module turns it into a delay to add before the trigger
(``gocator.trigger_delay_s``). A latency is a time, so the shift grows with
speed; fitting several speeds separates it from any constant offset in mm
(a ramp-distance error, say), which a delay cannot fix.

**Calibrate at the speed you will scan at.** On this rig the implied delay
fell from about 74 ms at 50 mm/s to 59 ms at 150 mm/s, and a single
measurement scatters by 1-2 mm (several ms of delay). The robust procedure is
therefore a handful of repeats at the planned speed
(:func:`summarize_trigger_delay`): the mean is the delay to configure and the
scatter says how well a single delay can do. Several speeds
(:func:`fit_trigger_delay`) are for checking the model, not for choosing a
delay to use at one speed.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Sequence

import numpy as np

from .block_finder import find_block


def block_travel_offset(
    forward_points: np.ndarray,
    reverse_points: np.ndarray,
    travel_axis: int,
    expected_size_mm: Optional[Sequence[float]] = None,
    **find_kwargs: Any,
) -> Optional[float]:
    """Where the block sits along travel in the reverse pass minus the forward pass.

    Args:
        forward_points: Oriented ``(N, 3)`` points of the pass that travelled
            toward increasing coordinate along `travel_axis`.
        reverse_points: The pass over the same block travelling the other way.
        travel_axis: Column (0 = X, 1 = Y) the passes travel along, in the
            frame the points are in.
        expected_size_mm: Block footprint, see
            :func:`~laguna.scanner.block_finder.find_block`.
        **find_kwargs: Passed to ``find_block``.

    Returns:
        ``centre_reverse - centre_forward`` along the travel axis, mm, or None
        if the block isn't found in both.
    """
    fwd = find_block(forward_points, expected_size_mm=expected_size_mm, **find_kwargs)
    rev = find_block(reverse_points, expected_size_mm=expected_size_mm, **find_kwargs)
    if fwd is None or rev is None:
        return None
    return float(rev["center"][travel_axis] - fwd["center"][travel_axis])


def fit_trigger_delay(
    speeds_mm_s: Sequence[float],
    offsets_mm: Sequence[float],
) -> Dict[str, Any]:
    """Solve the trigger delay from reverse-minus-forward block offsets.

    Each measurement is the offset ``d`` (see :func:`block_travel_offset`) at a
    scan speed ``v``. The model is ``d = -2 * v * delay + constant``.

    - One speed: the constant is assumed zero and ``delay = -d / (2 v)``.
    - Two or more: a least-squares line through ``(v, d)``; the slope gives
      the delay and the intercept is reported separately, as the part a delay
      cannot remove.

    Args:
        speeds_mm_s: Scan speeds, mm/s, all positive.
        offsets_mm: The reverse-minus-forward offset measured at each, mm.

    Returns:
        Dict with ``delay_s`` (signed; positive means the trigger fires early
        and should be delayed, negative means it fires late), ``constant_mm``
        (the fitted intercept, 0.0 for a single speed), ``rms_residual_mm``,
        ``n`` and ``per_speed_delay_s`` (each measurement's own estimate).

    Raises:
        ValueError: If the inputs differ in length, are empty, or contain a
            non-positive speed.
    """
    v = np.asarray(speeds_mm_s, dtype=float)
    d = np.asarray(offsets_mm, dtype=float)
    if v.shape != d.shape or v.ndim != 1 or v.size == 0:
        raise ValueError("speeds_mm_s and offsets_mm must be equal-length, non-empty 1-D sequences")
    if np.any(v <= 0):
        raise ValueError("speeds must be positive")
    per_speed = -d / (2.0 * v)
    if v.size == 1 or np.allclose(v, v[0]):
        delay, constant = float(np.mean(per_speed)), 0.0
    else:
        slope, constant = np.polyfit(v, d, 1)
        delay, constant = float(-slope / 2.0), float(constant)
    residual = d - (-2.0 * delay * v + constant)
    return {
        "delay_s": delay,
        "constant_mm": constant,
        "rms_residual_mm": float(np.sqrt(np.mean(residual**2))),
        "n": int(v.size),
        "per_speed_delay_s": per_speed.tolist(),
    }


def summarize_trigger_delay(speed_mm_s: float, offsets_mm: Sequence[float]) -> Dict[str, Any]:
    """Mean and scatter of the trigger delay from repeated measurements at one speed.

    Args:
        speed_mm_s: The scan speed every measurement was made at, mm/s.
        offsets_mm: Reverse-minus-forward block offsets from repeated
            forward/reverse pairs at that speed, mm.

    Returns:
        Dict with ``delay_s`` (the mean implied delay, the value to configure),
        ``std_s`` (sample standard deviation, NaN for a single measurement),
        ``sem_s`` (standard error of the mean), ``ci95_s`` (half-width of the
        95% confidence interval of the mean, Student's t), ``n``,
        ``samples_s`` (each measurement's own delay), ``seam_sigma_mm`` (the
        1-sigma seam one pair would still show with the mean delay applied,
        ``2 * speed * std``) and ``mean_offset_mm`` (the raw mean offset).

    Raises:
        ValueError: If the speed isn't positive or there are no measurements.
    """
    v = float(speed_mm_s)
    d = np.asarray(offsets_mm, dtype=float)
    d = d[np.isfinite(d)]
    if v <= 0:
        raise ValueError("speed must be positive")
    if d.size == 0:
        raise ValueError("need at least one measurement")
    samples = -d / (2.0 * v)
    n = int(samples.size)
    std = float(np.std(samples, ddof=1)) if n > 1 else float("nan")
    sem = std / np.sqrt(n) if n > 1 else float("nan")
    if n > 1:
        from scipy import stats

        ci95 = float(stats.t.ppf(0.975, n - 1) * sem)
    else:
        ci95 = float("nan")
    return {
        "delay_s": float(np.mean(samples)), "std_s": std, "sem_s": float(sem), "ci95_s": ci95,
        "n": n, "samples_s": samples.tolist(), "seam_sigma_mm": float(2.0 * v * std) if n > 1 else float("nan"),
        "mean_offset_mm": float(np.mean(d)),
    }
