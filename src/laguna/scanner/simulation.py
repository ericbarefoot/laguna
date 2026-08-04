"""A GoSdk stand-in that returns synthetic surfaces.

Lets a whole experiment be rehearsed without the vendor SDK built or the
sensor on the network — see :mod:`laguna.simulation`.

The surfaces have the right shape, dtype and scaling, and carry the same
no-data sentinel, so everything downstream is genuinely exercised: the
raw-count conversion, NaN handling, the mounting transform, point
flattening, and every export format. What they do not have is real geometry
— the returned heightmap is a smooth synthetic ramp, not a bed.
"""

from __future__ import annotations

import ctypes
import logging
from typing import Any, Dict, Optional

import numpy as np

from . import gosdk as _g

logger = logging.getLogger(__name__)

#: Shape of the synthetic surface. Small enough to keep a rehearsal quick,
#: large enough that downsampling and export paths behave realistically.
SIM_ROWS = 200
SIM_COLS = 400


class _SimGo:
    """Stands in for the libGoSdk CDLL object."""

    def __init__(self) -> None:
        self.travel_speed = 20.0
        self.frame_rate = 200.0
        self.frame_rate_limit_max = 443.127
        self.frame_rate_limit_min = 0.001
        self.fixed_length = 200.0
        self.length_limit_min, self.length_limit_max = 1.0, 5000.0
        self.scan_mode = _g.GO_MODE_SURFACE
        self.trigger_source = _g.GO_TRIGGER_TIME
        self.generation_type = _g.GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH
        self.start_trigger = _g.GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE
        self.uniform_spacing = True
        self.x_subsampling = 1
        self.z_subsampling = 1
        self.spacing_interval = 0.124
        self.spacing_interval_type = 0
        self.active_area = {
            "x": -1000.0, "y": 0.0, "z": -10.0,
            "width": 2000.0, "length": 200.0, "height": 300.0,
        }
        self._grid = self._make_surface()

    @staticmethod
    def _make_surface() -> np.ndarray:
        """A smooth ramp with a few invalid cells, as raw 16-bit counts."""
        rows = np.linspace(-2000, 2000, SIM_ROWS, dtype=np.float64)
        cols = np.linspace(-1000, 1000, SIM_COLS, dtype=np.float64)
        grid = (rows[:, None] * 0.1 + cols[None, :] * 0.05).astype(np.int16)
        # A patch of no-return, so NaN handling is genuinely exercised.
        grid[10:20, 10:30] = -32768
        return grid

    # -- everything else answers plausibly ------------------------------

    def __getattr__(self, name: str):
        def stub(*args):
            return _g.kOK

        return stub

    # handles
    def GoSensor_Setup(self, sensor):
        return 1

    def GoSensor_Transform(self, sensor):
        return 1

    def GoSetup_SurfaceGeneration(self, setup):
        return 1

    # getters the subsystem reads back
    def GoTransform_Speed(self, t):
        return self.travel_speed

    def GoSetup_FrameRate(self, s):
        return self.frame_rate

    def GoSetup_FrameRateLimitMax(self, s):
        return self.frame_rate_limit_max

    def GoSetup_FrameRateLimitMin(self, s):
        return self.frame_rate_limit_min

    def GoSetup_ScanMode(self, s):
        return self.scan_mode

    def GoSetup_TriggerSource(self, s):
        return self.trigger_source

    def GoSurfaceGeneration_GenerationType(self, s):
        return self.generation_type

    def GoSurfaceGenerationFixedLength_StartTrigger(self, s):
        return self.start_trigger

    def GoSurfaceGenerationFixedLength_Length(self, s):
        return self.fixed_length

    def GoSurfaceGenerationFixedLength_LengthLimitMin(self, s):
        return self.length_limit_min

    def GoSurfaceGenerationFixedLength_LengthLimitMax(self, s):
        return self.length_limit_max

    def GoSetup_UniformSpacingEnabled(self, s):
        return _g.kTRUE if self.uniform_spacing else _g.kFALSE

    def GoSetup_SpacingInterval(self, s, role):
        return self.spacing_interval

    def GoSetup_SpacingIntervalLimitMin(self, s, role):
        return 0.123

    def GoSetup_SpacingIntervalLimitMax(self, s, role):
        return 1.1

    def GoSetup_SpacingIntervalSystemValue(self, s, role):
        return 0.261

    def GoSetup_SpacingIntervalType(self, s, role):
        return self.spacing_interval_type

    def GoSetup_SpacingIntervalUsed(self, s, role):
        return _g.kFALSE

    def GoSetup_XSubsampling(self, s, role):
        return self.x_subsampling

    def GoSetup_ZSubsampling(self, s, role):
        return self.z_subsampling

    def GoSetup_XSubsamplingOptionCount(self, s, role):
        return 3

    def GoSetup_ZSubsamplingOptionCount(self, s, role):
        return 4

    def GoSetup_XSubsamplingOptionAt(self, s, role, i):
        return [1, 2, 4][int(getattr(i, "value", i))]

    def GoSetup_ZSubsamplingOptionAt(self, s, role, i):
        return [1, 2, 4, 8][int(getattr(i, "value", i))]

    def GoSetup_XSubsamplingSystemValue(self, s, role):
        return 1

    def GoSetup_ZSubsamplingSystemValue(self, s, role):
        return 1

    def GoSetup_XSubsamplingUsed(self, s, role):
        return _g.kTRUE

    def GoSetup_ZSubsamplingUsed(self, s, role):
        return _g.kFALSE

    # setters that matter for readback
    def GoSetup_SetFrameRate(self, s, rate):
        self.frame_rate = float(getattr(rate, "value", rate))
        return _g.kOK

    def GoTransform_SetSpeed(self, t, v):
        self.travel_speed = float(getattr(v, "value", v))
        return _g.kOK

    def GoSurfaceGenerationFixedLength_SetLength(self, s, v):
        self.fixed_length = float(getattr(v, "value", v))
        return _g.kOK

    def GoSetup_EnableUniformSpacing(self, s, v):
        self.uniform_spacing = bool(getattr(v, "value", v))
        return _g.kOK

    def GoSetup_SetXSubsampling(self, s, role, v):
        self.x_subsampling = int(getattr(v, "value", v))
        return _g.kOK

    def GoSetup_SetZSubsampling(self, s, role, v):
        self.z_subsampling = int(getattr(v, "value", v))
        return _g.kOK

    # active area
    def __getattribute__(self, name: str):
        if name.startswith("GoSetup_") and "ActiveArea" in name:
            return object.__getattribute__(self, "_active_area_accessor")(name)
        return object.__getattribute__(self, name)

    def _active_area_accessor(self, name: str):
        import re

        m = re.fullmatch(
            r"GoSetup_(Set)?ActiveArea(X|Y|Z|Width|Length|Height)(LimitMin|LimitMax)?", name
        )
        if not m:
            return lambda *a: _g.kOK
        setter, field, limit = m.group(1), m.group(2).lower(), m.group(3)
        area = object.__getattribute__(self, "active_area")
        if setter:
            def _set(setup, role, value):
                area[field] = float(getattr(value, "value", value))
                return _g.kOK

            return _set

        def _get(setup, role):
            if limit == "LimitMin":
                return -2000.0 if field in ("x", "y", "z") else 0.0
            if limit == "LimitMax":
                return 2000.0
            return area[field]

        return _get

    # data channel
    def GoSystem_ReceiveData(self, system, dataset_ptr, timeout):
        dataset_ptr._obj.value = 1
        return _g.kOK

    def GoDataSet_Count(self, dataset):
        return 1

    def GoDataSet_At(self, dataset, i):
        return 1

    def GoDataMsg_Type(self, msg):
        return _g.GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE

    def GoUniformSurfaceMsg_Length(self, msg):
        return SIM_ROWS

    def GoUniformSurfaceMsg_Width(self, msg):
        return SIM_COLS

    def GoUniformSurfaceMsg_XResolution(self, msg):
        return 124000

    def GoUniformSurfaceMsg_YResolution(self, msg):
        return 250000

    def GoUniformSurfaceMsg_ZResolution(self, msg):
        return 24300

    def GoUniformSurfaceMsg_XOffset(self, msg):
        return -1000000

    def GoUniformSurfaceMsg_YOffset(self, msg):
        return -100000

    def GoUniformSurfaceMsg_ZOffset(self, msg):
        return 0

    def GoUniformSurfaceMsg_RowAt(self, msg, row):
        grid = object.__getattribute__(self, "_grid")
        data = grid[int(getattr(row, "value", row))]
        return ctypes.cast(
            (ctypes.c_int16 * len(data))(*data.tolist()),
            ctypes.POINTER(ctypes.c_int16),
        )

    def GoDestroy(self, obj):
        return _g.kOK


class SimulatedGoSdkLib:
    """Stands in for :class:`laguna.scanner.gosdk.GoSdkLib`."""

    def __init__(self, lib_dir: Optional[str] = None) -> None:
        self.lib_dir = "<simulated>"
        self.go = _SimGo()

    def call(self, name: str, *args) -> None:
        getattr(self.go, name)(*args)

    def handle(self, name: str, *args) -> int:
        return getattr(self.go, name)(*args)

    def parse_ip(self, ip: str) -> Any:
        return _g.kIpAddress() if hasattr(_g, "kIpAddress") else ctypes.c_int(0)


__all__ = ["SimulatedGoSdkLib", "SIM_ROWS", "SIM_COLS"]
