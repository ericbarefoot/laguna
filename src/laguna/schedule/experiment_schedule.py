"""Experiment schedule loading and interpolation."""

from typing import Callable, Dict, Optional, Union
from pathlib import Path

try:
    import numpy as np
    from scipy.interpolate import CubicSpline
    import pandas as pd
except ImportError as _e:
    raise ImportError(
        "laguna.schedule requires numpy, scipy, and pandas. "
        "Install them with: pip install numpy scipy pandas"
    ) from _e

REQUIRED_COLUMNS = ["time_s"]


class ExperimentSchedule:
    """Loads a tabular experiment schedule and builds callable time functions.

    Each recognized column in the source table (weir elevation, pump flow,
    valve states) becomes an independent function of experiment time,
    interpolated according to `INTERP_DEFAULTS` (overridable per column via
    the `interpolation` argument). These functions are what the scheduler
    calls to determine setpoints at arbitrary times between the table's
    explicit rows — see the `weir_elevation`/`pump_flow`/`qin_open`/
    `qaux_open` properties.

    Example::

        schedule = ExperimentSchedule.from_csv("run1.csv")
        target_mm = schedule.weir_elevation(t=42.5)
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
        interp_modes = {**self.INTERP_DEFAULTS, **(interpolation or {})}
        times = df["time_s"].to_numpy(dtype=float)

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
        """Callable mapping experiment time (s) to target weir elevation (mm).

        Raises:
            AttributeError: If the schedule table had no
                'weir_elevation_mm' column.
        """
        return self._get_interpolator("weir_elevation_mm")

    @property
    def pump_flow(self) -> Callable[[float], float]:
        """Callable mapping experiment time (s) to target pump flow rate (L/min).

        Raises:
            AttributeError: If the schedule table had no 'pump_flow_lpm'
                column.
        """
        return self._get_interpolator("pump_flow_lpm")

    @property
    def qin_open(self) -> Callable[[float], bool]:
        """Callable mapping experiment time (s) to target inlet valve state.

        Raises:
            AttributeError: If the schedule table had no 'qin_open' column.
        """
        return self._get_interpolator("qin_open")

    @property
    def qaux_open(self) -> Callable[[float], bool]:
        """Callable mapping experiment time (s) to target auxiliary valve state.

        Raises:
            AttributeError: If the schedule table had no 'qaux_open' column.
        """
        return self._get_interpolator("qaux_open")

    def _get_interpolator(self, col: str) -> Callable:
        """Look up the interpolator built for `col`, or raise if it wasn't present in the source table."""
        if col not in self._interpolators:
            raise AttributeError(
                f"No interpolator for column '{col}'. "
                f"Ensure the schedule dataframe contains this column."
            )
        return self._interpolators[col]

    @staticmethod
    def _build_spline(times: np.ndarray, values: np.ndarray) -> Callable[[float], float]:
        """Build a natural cubic spline interpolator through (times, values).

        Used for weir elevation by default — motion should be smooth
        (continuous velocity/acceleration) rather than piecewise-linear.
        """
        cs = CubicSpline(times, values)
        return lambda t: float(cs(t))

    @staticmethod
    def _build_linear(times: np.ndarray, values: np.ndarray) -> Callable[[float], float]:
        """Build a piecewise-linear interpolator through (times, values).

        Used for pump flow by default. Values before the first or after the
        last timestamp are clamped to the nearest endpoint (numpy.interp
        semantics), not extrapolated.
        """
        return lambda t: float(np.interp(t, times, values))

    @staticmethod
    def _build_step(times: np.ndarray, values: np.ndarray) -> Callable:
        """Build a zero-order-hold (step) interpolator through (times, values).

        Used by default for the boolean valve columns (qin_open,
        qaux_open), where "50% open" is meaningless — the value at time
        `t` is whatever it was most recently set to at or before `t`.
        Times before the first row hold the first row's value.
        """
        def step_interp(t):
            idx = np.searchsorted(times, t, side="right") - 1
            idx = max(0, min(idx, len(values) - 1))
            return values[idx]
        return step_interp
