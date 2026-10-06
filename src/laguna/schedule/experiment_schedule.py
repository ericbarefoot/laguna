"""Experiment schedule loading and interpolation."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Callable, Dict, Optional, Tuple, Union

if TYPE_CHECKING:
    import numpy as np
    import pandas as pd


def _require_deps() -> Tuple["np", "pd", Callable]:
    """Import numpy, pandas and scipy's CubicSpline, or explain what to install.

    Deferred from module import time so that importing ``laguna.schedule``
    (which ``laguna.experiment.runner`` does unconditionally) succeeds in a
    lightweight environment, such as a Raspberry Pi camera node, that never
    builds a schedule. The failure surfaces only when a schedule is used.
    numpy is deferred along with the rest for uniformity, but ``import
    laguna`` itself already requires it (``laguna.frames``), so in practice
    this guards the heavier scipy and pandas.

    Returns:
        ``(numpy, pandas, CubicSpline)``.

    Raises:
        ImportError: If any of numpy, scipy or pandas is missing.
    """
    try:
        import numpy as np
        import pandas as pd
        from scipy.interpolate import CubicSpline
    except ImportError as exc:
        raise ImportError(
            "ExperimentSchedule requires numpy, scipy, and pandas. "
            "Install them with: pip install numpy scipy pandas"
        ) from exc
    return np, pd, CubicSpline


REQUIRED_COLUMNS = ["time_s"]


class ExperimentSchedule:
    """Loads a tabular experiment schedule and builds callable time functions.

    Each recognized column in the source table (weir elevation, pump flow,
    valve states) becomes an independent function of experiment time,
    interpolated according to INTERP_DEFAULTS (overridable per column via
    the interpolation argument). Access interpolators via properties:
    weir_elevation, pump_flow, qin_open, qaux_open.
    """

    INTERP_DEFAULTS = {
        "weir_elevation_mm": "spline",
        "pump_flow_lpm": "linear",
        "qin_open": "step",
        "qaux_open": "step",
    }

    def __init__(self, df: pd.DataFrame, interpolation: Optional[Dict[str, str]] = None):
        """Build interpolators from an already-loaded schedule dataframe.

        Prefer from_csv()/from_excel()/from_dataframe() over calling this
        directly — they validate required columns first.

        Args:
            df: Schedule table; must contain a 'time_s' column. Any of
                'weir_elevation_mm', 'pump_flow_lpm', 'qin_open',
                'qaux_open' that are present get an interpolator built for
                them; columns not in `INTERP_DEFAULTS` are ignored.
            interpolation: Per-column overrides for interpolation mode
                ('spline', 'linear', or 'step'), merged on top of
                `INTERP_DEFAULTS`.
        """
        _require_deps()
        interp_modes = {**self.INTERP_DEFAULTS, **(interpolation or {})}
        times = df["time_s"].to_numpy(dtype=float)
        self._validate(df, times, [col for col in interp_modes if col in df.columns])

        self._df = df
        self._interpolators: Dict[str, Callable] = {}
        for col, mode in interp_modes.items():
            if col in df.columns:
                values = df[col].to_numpy()
                if mode == "spline":
                    self._interpolators[col] = self._build_spline(times, values.astype(float))
                elif mode == "linear":
                    self._interpolators[col] = self._build_linear(times, values.astype(float))
                elif mode == "step":
                    self._interpolators[col] = self._build_step(times, values)

    @classmethod
    def from_csv(cls, path: Union[str, Path], interpolation: Optional[Dict[str, str]] = None) -> "ExperimentSchedule":
        """Load a schedule from a CSV file.

        Args:
            path: Path to a CSV file with at least a 'time_s' column.
            interpolation: Per-column interpolation mode overrides; see
                __init__.

        Returns:
            A new ExperimentSchedule.

        Raises:
            ValueError: If the file is missing required columns.
        """
        pd = _require_deps()[1]
        df = pd.read_csv(path)
        return cls.from_dataframe(df, interpolation=interpolation)

    @classmethod
    def from_excel(cls, path: Union[str, Path], sheet: Union[int, str] = 0, interpolation: Optional[Dict[str, str]] = None) -> "ExperimentSchedule":
        """Load a schedule from an Excel workbook.

        Args:
            path: Path to an .xlsx/.xls file.
            sheet: Sheet name or zero-based index to read (default 0).
            interpolation: Per-column interpolation mode overrides; see
                __init__.

        Returns:
            A new ExperimentSchedule.

        Raises:
            ValueError: If the sheet is missing required columns.
        """
        pd = _require_deps()[1]
        df = pd.read_excel(path, sheet_name=sheet)
        return cls.from_dataframe(df, interpolation=interpolation)

    @classmethod
    def from_dataframe(cls, df: pd.DataFrame, interpolation: Optional[Dict[str, str]] = None) -> "ExperimentSchedule":
        """Build a schedule from an already-loaded dataframe, validating required columns first.

        Args:
            df: Schedule table; must contain every column in
                `REQUIRED_COLUMNS` ('time_s').
            interpolation: Per-column interpolation mode overrides; see
                __init__.

        Returns:
            A new ExperimentSchedule.

        Raises:
            ValueError: If `df` is missing any of `REQUIRED_COLUMNS`.
        """
        missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
        if missing:
            raise ValueError(
                f"ExperimentSchedule dataframe is missing required columns: {missing}. "
                f"Required: {REQUIRED_COLUMNS}"
            )
        return cls(df, interpolation=interpolation)

    @property
    def weir_elevation(self) -> Callable[[float], float]:
        """Interpolator function for weir elevation (mm) by experiment time (s).

        Returns:
            Callable that takes time in seconds and returns elevation in mm.

        Raises:
            AttributeError: If weir_elevation_mm column not in source table.
        """
        return self._get_interpolator("weir_elevation_mm")

    @property
    def pump_flow(self) -> Callable[[float], float]:
        """Interpolator function for pump flow rate (L/min) by experiment time (s).

        Returns:
            Callable that takes time in seconds and returns flow in L/min.

        Raises:
            AttributeError: If pump_flow_lpm column not in source table.
        """
        return self._get_interpolator("pump_flow_lpm")

    @property
    def qin_open(self) -> Callable[[float], bool]:
        """Interpolator function for inlet valve state by experiment time (s).

        Returns:
            Callable that takes time in seconds and returns valve state.

        Raises:
            AttributeError: If qin_open column not in source table.
        """
        return self._get_interpolator("qin_open")

    @property
    def qaux_open(self) -> Callable[[float], bool]:
        """Interpolator function for auxiliary valve state by experiment time (s).

        Returns:
            Callable that takes time in seconds and returns valve state.

        Raises:
            AttributeError: If qaux_open column not in source table.
        """
        return self._get_interpolator("qaux_open")

    def _get_interpolator(self, col: str) -> Callable:
        """Retrieve interpolator for given column name.

        Args:
            col: Column name to look up.

        Returns:
            Callable interpolator for the column.

        Raises:
            AttributeError: If column was not present in source table.
        """
        if col not in self._interpolators:
            raise AttributeError(
                f"No interpolator for column '{col}'. "
                f"Ensure the schedule dataframe contains this column."
            )
        return self._interpolators[col]

    @staticmethod
    def _validate(df: pd.DataFrame, times: np.ndarray, columns: list) -> None:
        """Refuse a schedule the interpolators would silently misread.

        np.interp and np.searchsorted assume sorted, unique times and give
        wrong answers — not errors — otherwise: an out-of-order row once
        turned a 10 L/min setpoint into 15. A blank cell becomes NaN, which
        reached set_flowrate() as-is and, in a valve column, read as True
        and opened the valve.

        Raises:
            ValueError: Naming the offending rows.
        """
        np = _require_deps()[0]
        bad = np.flatnonzero(~np.isfinite(times))
        if bad.size:
            raise ValueError(f"time_s must be finite; check rows {bad.tolist()}")
        if times.size > 1:
            out_of_order = np.flatnonzero(np.diff(times) <= 0)
            if out_of_order.size:
                raise ValueError(
                    "time_s must be strictly increasing (no repeats); check rows "
                    f"{(out_of_order + 1).tolist()}"
                )
        for col in columns:
            missing = np.flatnonzero(df[col].isna().to_numpy())
            if missing.size:
                raise ValueError(f"Column {col!r} has blank cells at rows {missing.tolist()}")

    @staticmethod
    def _build_spline(times: np.ndarray, values: np.ndarray) -> Callable[[float], float]:
        """Build cubic spline interpolator through (times, values).

        Args:
            times: Time array (seconds).
            values: Value array at corresponding times.

        Returns:
            Callable that evaluates spline at arbitrary time points.
        """
        cubic_spline = _require_deps()[2]
        cs = cubic_spline(times, values)
        return lambda t: float(cs(t))

    @staticmethod
    def _build_linear(times: np.ndarray, values: np.ndarray) -> Callable[[float], float]:
        """Build piecewise-linear interpolator through (times, values).

        Args:
            times: Time array (seconds).
            values: Value array at corresponding times.

        Returns:
            Callable that linearly interpolates at arbitrary time points.
                Values before/after endpoints are clamped (not extrapolated).
        """
        np = _require_deps()[0]
        return lambda t: float(np.interp(t, times, values))

    @staticmethod
    def _build_step(times: np.ndarray, values: np.ndarray) -> Callable:
        """Build zero-order-hold (step) interpolator through (times, values).

        Args:
            times: Time array (seconds).
            values: Value array at corresponding times.

        Returns:
            Callable that returns value most recently set at or before time t.
        """
        np = _require_deps()[0]

        def step_interp(t):
            idx = np.searchsorted(times, t, side="right") - 1
            idx = max(0, min(idx, len(values) - 1))
            return values[idx]
        return step_interp
