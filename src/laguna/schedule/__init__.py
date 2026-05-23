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

REQUIRED_COLUMNS = ["time_s", "weir_elevation_mm", "pump_flow_lpm", "qin_open", "qaux_open"]


class ExperimentSchedule:
    """Loads a tabular experiment schedule and builds callable time functions."""

    INTERP_DEFAULTS = {
        "weir_elevation_mm": "spline",
        "pump_flow_lpm": "linear",
        "qin_open": "step",
        "qaux_open": "step",
    }

    def __init__(self, df: pd.DataFrame, interpolation: Optional[Dict[str, str]] = None):
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
        df = pd.read_csv(path)
        return cls.from_dataframe(df, interpolation=interpolation)

    @classmethod
    def from_excel(cls, path: Union[str, Path], sheet: Union[int, str] = 0, interpolation: Optional[Dict[str, str]] = None) -> "ExperimentSchedule":
        df = pd.read_excel(path, sheet_name=sheet)
        return cls.from_dataframe(df, interpolation=interpolation)

    @classmethod
    def from_dataframe(cls, df: pd.DataFrame, interpolation: Optional[Dict[str, str]] = None) -> "ExperimentSchedule":
        missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
        if missing:
            raise ValueError(
                f"ExperimentSchedule dataframe is missing required columns: {missing}. "
                f"Required: {REQUIRED_COLUMNS}"
            )
        return cls(df, interpolation=interpolation)

    @property
    def weir_elevation(self) -> Callable[[float], float]:
        return self._get_interpolator("weir_elevation_mm")

    @property
    def pump_flow(self) -> Callable[[float], float]:
        return self._get_interpolator("pump_flow_lpm")

    @property
    def qin_open(self) -> Callable[[float], bool]:
        return self._get_interpolator("qin_open")

    @property
    def qaux_open(self) -> Callable[[float], bool]:
        return self._get_interpolator("qaux_open")

    def _get_interpolator(self, col: str) -> Callable:
        if col not in self._interpolators:
            raise AttributeError(
                f"No interpolator for column '{col}'. "
                f"Ensure the schedule dataframe contains this column."
            )
        return self._interpolators[col]

    @staticmethod
    def _build_spline(times: np.ndarray, values: np.ndarray) -> Callable[[float], float]:
        cs = CubicSpline(times, values)
        return lambda t: float(cs(t))

    @staticmethod
    def _build_linear(times: np.ndarray, values: np.ndarray) -> Callable[[float], float]:
        return lambda t: float(np.interp(t, times, values))

    @staticmethod
    def _build_step(times: np.ndarray, values: np.ndarray) -> Callable:
        def step_interp(t):
            idx = np.searchsorted(times, t, side="right") - 1
            idx = max(0, min(idx, len(values) - 1))
            return values[idx]
        return step_interp
