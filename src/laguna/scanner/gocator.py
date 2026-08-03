"""LMI Gocator 2690 line-profile scanner subsystem.

Encoderless operation: the gantry moves the sensor at a constant commanded
velocity and we bracket the pass with a software trigger. The sensor does its
own surface generation and returns a correctly-Y-scaled 3D surface, using the
travel speed we configure via ``GoTransform_SetSpeed``.

The configuration recipe (confirmed on hardware 2026-07-30, first via the
sensor's web UI):

1. Travel speed (mm/s) = the gantry's commanded feed rate.
2. Trigger source = ``GO_TRIGGER_TIME`` — profiles fire on the internal clock.
3. Surface generation type = ``FIXED_LENGTH``, sized to the travel distance.
4. Fixed-length start trigger = ``SOFTWARE``.
5. Start gantry motion, wait for constant velocity, fire the software
   trigger, then receive one surface per pass.

Unlike the OD2000/WTT12L rangefinders (which reach the lab network through a
Pi-side AL1342 IO-Link master), the Gocator is a plain Ethernet device on the
laguna PC's own network — no SSH, no deployed scripts, no MQTT.

See ``docs/subsystems/scanner.md`` for usage and
``docs/reference/gocator/`` for SDK/manual reference notes.
"""

from __future__ import annotations

import ctypes
import logging
import time
from ctypes import byref
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from . import gosdk as _g
from .gosdk import GoSdkError, GoSdkLib, GoSdkTimeout
from .mounting import SensorMounting
from .pointcloud import (
    SurfaceScan,
    surface_point_cloud_to_scan,
    uniform_surface_to_scan,
)

logger = logging.getLogger(__name__)

#: Default poll timeout for GoSystem_ReceiveData, in seconds. The SDK samples
#: use 20 s; a scan pass is bounded by the gantry move, so callers normally
#: pass an explicit timeout derived from distance/feed rate.
DEFAULT_RECEIVE_TIMEOUT_S = 20.0

# Enum value -> label, so get_status() reads as words rather than magic ints.
_SCAN_MODES = {0: "video", 1: "range", 2: "profile", 3: "surface"}
_TRIGGER_SOURCES = {0: "time", 1: "encoder", 2: "input", 3: "software"}
_GENERATION_TYPES = {
    0: "continuous",
    1: "fixed_length",
    2: "variable_length",
    3: "rotational",
}
_START_TRIGGERS = {0: "sequential", 1: "digital", 2: "software"}


_SPACING_INTERVAL_TYPES = {
    _g.GO_SPACING_INTERVAL_TYPE_MAX_RES: "max_res",
    _g.GO_SPACING_INTERVAL_TYPE_BALANCED: "balanced",
    _g.GO_SPACING_INTERVAL_TYPE_MAX_SPEED: "max_speed",
    _g.GO_SPACING_INTERVAL_TYPE_CUSTOM: "custom",
}
_SPACING_INTERVAL_TYPE_VALUES = {v: k for k, v in _SPACING_INTERVAL_TYPES.items()}

#: Post-processing filters, as ``python_name -> SDK infix``. All operate on
#: the resampled grid, so all require uniform spacing — see
#: GocatorScanner.set_filters().
_FILTERS = {
    "x_smoothing": "XSmoothing",
    "x_median": "XMedian",
    "x_decimation": "XDecimation",
    "x_gap_filling": "XGapFilling",
    "y_smoothing": "YSmoothing",
    "y_median": "YMedian",
    "y_decimation": "YDecimation",
    "y_gap_filling": "YGapFilling",
}

#: Public tuple of filter names accepted by GocatorScanner.set_filters().
FILTER_NAMES = tuple(_FILTERS)


class UniformSpacingRequiredError(RuntimeError):
    """A uniform-spacing-only feature was requested in point-cloud mode.

    Filters and the spacing interval act on the resampled X grid, which
    only exists when uniform spacing is enabled. With it off the sensor
    emits an un-resampled SURFACE_POINT_CLOUD and reports these settings as
    unavailable (``GoSetup_*Used`` reads false), so writing them would
    silently do nothing. Measured on hardware 2026-08-02: every filter's
    ``Used`` flag flips 0 -> 1 exactly with uniform spacing.

    Subsampling is deliberately *not* covered by this — it is a sensor-level
    acquisition divider that works in both modes (confirmed: x2/x4 scaling
    with uniform spacing both on and off).
    """


def _name(mapping: Dict[int, str], value: Any) -> str:
    """Label an SDK enum value, falling back to the raw number if unknown."""
    key = int(value)
    return mapping.get(key, f"unknown({key})")


class GocatorScanner:
    """Gocator 2690 surface scanner, shaped as a FlumeLab subsystem.

    Attaches to ``FlumeLab`` like any other subsystem (``lab.add(scanner)`` →
    ``lab.gocator``), exposing the same ``connect()``/``disconnect()``/
    ``get_status()`` trio as ``laguna.weir`` and the rangefinders.

    Args:
        config: Dict with keys:
            ip: Sensor IP address (e.g. "192.168.1.10").
            travel_speed_mm_s: Assumed constant gantry velocity, mm/s. This
                is what scales the Y axis — it must match the actual feed
                rate used during the pass or the scan is distorted along
                travel. Written to sensor flash, so it is only pushed when
                it differs from what the sensor already holds.
            frame_rate_hz: Profile trigger rate. Omit (or null) to leave
                whatever frame-rate mode/rate the sensor already has
                (its factory default is max-frame-rate mode, but a prior
                configure() call that set an explicit rate disables that
                mode in sensor flash — omitting frame_rate_hz afterward
                does NOT re-enable it). To explicitly request the sensor's
                current maximum regardless of prior state, use
                frame_rate_max instead.
            frame_rate_max: If true, explicitly enable max-frame-rate mode
                and use whatever rate the sensor reports after flushing —
                mutually exclusive with frame_rate_hz. The achievable max
                is dynamic (depends on FOV/exposure/uniform spacing, and
                even on whether max-frame-rate mode itself is enabled — see
                docs/subsystems/scanner.md), so this reads it back live
                rather than assuming a fixed number.
            exposure_us: Optional laser exposure in microseconds.
            fixed_length_mm: Surface length along travel, mm. Should cover
                the intended scan distance.
            uniform_spacing: X-axis resampling, which also selects the
                message type: True gives a resampled UNIFORM_SURFACE
                heightmap (Z only, X implied by index), False gives a
                SURFACE_POINT_CLOUD with an explicit (x, y, z) per point at
                native non-uniform X spacing. Leaving it unset keeps
                whatever the sensor is configured with. See configure().
            data_capacity_bytes: Optional GoSystem receive-buffer size.
            sdk_lib_dir: Directory holding libGoSdk.so/libkApi.so. Falls back
                to $LAGUNA_GOSDK_LIB_DIR and conventional locations.
            output_dir: Where save_scan() writes by default.
    """

    subsystem_name = "gocator"

    def __init__(self, config: Dict[str, Any]):
        self._ip = config.get("ip", "192.168.1.10")
        self._travel_speed_mm_s = config.get("travel_speed_mm_s")
        self._frame_rate_hz = config.get("frame_rate_hz")
        self._frame_rate_max = bool(config.get("frame_rate_max", False))
        self._exposure_us = config.get("exposure_us")
        self._fixed_length_mm = config.get("fixed_length_mm")
        self._uniform_spacing = config.get("uniform_spacing")
        self._active_area = config.get("active_area") or None
        # How the sensor sits on the gantry. Identity unless configured, so
        # existing setups are unaffected — see laguna.scanner.mounting.
        self._mounting = SensorMounting.from_config(config.get("mounting"))
        self._subsampling = config.get("subsampling") or None
        self._spacing_interval = config.get("spacing_interval") or None
        self._filters = config.get("filters") or None
        self._data_capacity_bytes = config.get("data_capacity_bytes")
        self._sdk_lib_dir = config.get("sdk_lib_dir")
        self._output_dir = Path(config.get("output_dir", "./data/scans"))

        self._lib: Optional[GoSdkLib] = None
        self._api = None      # kAssembly handle from GoSdk_Construct
        self._system = None   # GoSystem handle
        self._sensor = None   # GoSensor handle
        self._is_connected = False
        self._is_running = False
        self._scan_count = 0
        self._last_scan_meta: Dict[str, Any] = {}

    # ------------------------------------------------------------------
    # Subsystem lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Load the SDK, find the sensor by IP, and open the control channel.

        Returns:
            True on success. False (with a logged warning) if the SDK
            libraries are missing or the sensor can't be reached — matching
            the other subsystems' non-raising connect() contract so
            ``lab.connect_all()`` can carry on.
        """
        if self._is_connected:
            return True
        try:
            self._lib = GoSdkLib(self._sdk_lib_dir)
            logger.info("Loaded GoSdk from %s", self._lib.lib_dir)

            api = _g.kObject()
            self._lib.call("GoSdk_Construct", byref(api))
            self._api = api

            system = _g.kObject()
            self._lib.call("GoSystem_Construct", byref(system), None)
            self._system = system

            address = self._lib.parse_ip(self._ip)
            sensor = _g.kObject()
            self._lib.call(
                "GoSystem_FindSensorByIpAddress",
                self._system,
                byref(address),
                byref(sensor),
            )
            self._sensor = sensor

            self._lib.call("GoSensor_Connect", self._sensor)

            if self._data_capacity_bytes:
                self._lib.call(
                    "GoSystem_SetDataCapacity",
                    self._system,
                    _g.kSize(int(self._data_capacity_bytes)),
                )

            self._is_connected = True
            logger.info("Connected to Gocator at %s", self._ip)
            return True
        except (FileNotFoundError, GoSdkError, OSError, AttributeError) as e:
            # AttributeError covers a symbol this SDK build doesn't export —
            # gosdk._declare() tolerates that at load time, so the failure
            # surfaces here at first call instead.
            logger.warning("Gocator connect failed (%s): %s", type(e).__name__, e)
            self._teardown()
            return False

    def disconnect(self) -> None:
        """Stop acquisition if running, then release SDK handles."""
        if self._is_running:
            try:
                self.stop()
            except GoSdkError as e:
                logger.warning("Error stopping Gocator during disconnect: %s", e)
        if self._lib and self._sensor is not None:
            try:
                self._lib.call("GoSensor_Disconnect", self._sensor)
            except GoSdkError as e:
                logger.warning("Error disconnecting Gocator sensor: %s", e)
        self._teardown()
        logger.info("Disconnected from Gocator")

    def _teardown(self) -> None:
        """Destroy SDK objects in reverse construction order, best-effort."""
        if self._lib:
            for handle_attr in ("_system", "_api"):
                handle = getattr(self, handle_attr)
                if handle is not None:
                    try:
                        self._lib.go.GoDestroy(handle)
                    except Exception as e:  # pragma: no cover - teardown path
                        logger.debug("GoDestroy(%s) failed: %s", handle_attr, e)
        self._sensor = None   # owned by the system, not separately destroyed
        self._system = None
        self._api = None
        self._lib = None
        self._is_connected = False
        self._is_running = False

    def get_status(self) -> Dict[str, Any]:
        """Return connection/config state. Reads live values when connected."""
        status: Dict[str, Any] = {
            "is_connected": self._is_connected,
            "is_running": self._is_running,
            "ip": self._ip,
            "scan_count": self._scan_count,
            "travel_speed_mm_s": self._travel_speed_mm_s,
            "frame_rate_hz": self._frame_rate_hz,
            "frame_rate_max": self._frame_rate_max,
            "fixed_length_mm": self._fixed_length_mm,
            "uniform_spacing": self._uniform_spacing,
            "last_scan": self._last_scan_meta or None,
        }
        if self._is_connected and self._lib:
            try:
                setup = self._lib.handle("GoSensor_Setup", self._sensor)
                transform = self._lib.handle("GoSensor_Transform", self._sensor)
                surface = self._lib.handle("GoSetup_SurfaceGeneration", setup)
                status.update(
                    {
                        "sensor_travel_speed_mm_s": float(
                            self._lib.go.GoTransform_Speed(transform)
                        ),
                        "sensor_frame_rate_hz": float(
                            self._lib.go.GoSetup_FrameRate(setup)
                        ),
                        "sensor_frame_rate_max_hz": float(
                            self._lib.go.GoSetup_FrameRateLimitMax(setup)
                        ),
                        "sensor_scan_mode": _name(
                            _SCAN_MODES, self._lib.go.GoSetup_ScanMode(setup)
                        ),
                        "sensor_trigger_source": _name(
                            _TRIGGER_SOURCES, self._lib.go.GoSetup_TriggerSource(setup)
                        ),
                        # The two settings that actually define the encoderless
                        # recipe — worth seeing at a glance, not just inferring.
                        "sensor_surface_generation": _name(
                            _GENERATION_TYPES,
                            self._lib.go.GoSurfaceGeneration_GenerationType(surface),
                        ),
                        "sensor_start_trigger": _name(
                            _START_TRIGGERS,
                            self._lib.go.GoSurfaceGenerationFixedLength_StartTrigger(
                                surface
                            ),
                        ),
                        "sensor_fixed_length_mm": float(
                            self._lib.go.GoSurfaceGenerationFixedLength_Length(surface)
                        ),
                        # The active area governs the frame-rate ceiling
                        # reported just above, so show them together.
                        "sensor_active_area_mm": {
                            f: round(
                                float(
                                    getattr(
                                        self._lib.go,
                                        f"GoSetup_ActiveArea{f.capitalize()}",
                                    )(setup, _g.k32s(_g.GO_ROLE_MAIN))
                                ),
                                3,
                            )
                            for f in self._ACTIVE_AREA_FIELDS
                        },
                    }
                )
            except GoSdkError as e:
                status["status_read_error"] = str(e)
        return status

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    #: Active-area fields, in the order the web UI presents them: an origin
    #: (x, y, z) and the extents from it (width, length, height), all mm.
    _ACTIVE_AREA_FIELDS = ("x", "y", "z", "width", "length", "height")

    def get_active_area(self) -> Dict[str, Any]:
        """Read the sensor's active area (region of interest), in mm.

        The active area is the volume the sensor actually acquires and
        searches for the laser line. **Shrinking it — above all in Z — is the
        most effective way to raise the frame-rate ceiling**, because the
        camera reads out fewer rows per profile. This is the same
        Scan > Sensor > Active Area panel the web UI exposes, and it's the
        first knob to reach for when `frame_rate_max` isn't fast enough (see
        docs/subsystems/scanner.md).

        Returns:
            Dict with, per field in ``_ACTIVE_AREA_FIELDS``, the current
            value plus the sensor's live ``*_limit_min``/``*_limit_max`` —
            those limits are what a set_active_area() call is validated
            against, and they're model- and configuration-dependent.

        Raises:
            RuntimeError: If not connected.
            GoSdkError: If any SDK call fails.
        """
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        role = _g.k32s(_g.GO_ROLE_MAIN)

        area: Dict[str, Any] = {}
        for field in self._ACTIVE_AREA_FIELDS:
            name = field.capitalize()
            area[f"{field}_mm"] = float(
                getattr(lib.go, f"GoSetup_ActiveArea{name}")(setup, role)
            )
            area[f"{field}_limit_min"] = float(
                getattr(lib.go, f"GoSetup_ActiveArea{name}LimitMin")(setup, role)
            )
            area[f"{field}_limit_max"] = float(
                getattr(lib.go, f"GoSetup_ActiveArea{name}LimitMax")(setup, role)
            )
        return area

    def set_active_area(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        width: Optional[float] = None,
        length: Optional[float] = None,
        height: Optional[float] = None,
        flush: bool = True,
    ) -> Dict[str, Any]:
        """Set the sensor's active area (region of interest), in mm.

        Restricting the active area is the main lever for scan speed on this
        sensor: fewer camera rows to read out per profile means a higher
        achievable frame rate, which in turn buys finer Y spacing at a given
        travel speed (``Y spacing = travel_speed / frame_rate``). Cutting Z
        (the range/standoff direction) helps most; narrowing width (X) helps
        too. The trade is exactly what you'd expect — anything outside the
        area simply isn't measured, so leave margin for the tallest feature
        and any gantry Z wander.

        Only the fields you pass are written; the rest are left alone. Each
        is validated against the sensor's own live limits first, since those
        are model- and configuration-dependent (and, like the frame-rate
        ceiling, can shift as other settings change).

        Args:
            x, y, z: Active-area origin, mm.
            width, length, height: Extents from that origin, mm — X, Y and Z
                respectively.
            flush: Push to the sensor with ``GoSensor_Flush``. Pass False to
                batch this with other changes and flush once yourself.

        Returns:
            The active area read back after applying (same shape as
            :meth:`get_active_area`).

        Raises:
            RuntimeError: If not connected.
            ValueError: If a value falls outside the sensor's live limits, or
                no field was given at all.
            GoSdkError: If any SDK call fails.
        """
        requested = {
            "x": x, "y": y, "z": z,
            "width": width, "length": length, "height": height,
        }
        given = {k: v for k, v in requested.items() if v is not None}
        if not given:
            raise ValueError(
                "set_active_area() needs at least one of "
                f"{', '.join(self._ACTIVE_AREA_FIELDS)}"
            )

        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        role = _g.k32s(_g.GO_ROLE_MAIN)

        # Validate everything before writing anything, so a bad value can't
        # leave the area half-applied — a partially-written ROI could silently
        # clip the scan rather than failing outright.
        for field, value in given.items():
            name = field.capitalize()
            lo = float(getattr(lib.go, f"GoSetup_ActiveArea{name}LimitMin")(setup, role))
            hi = float(getattr(lib.go, f"GoSetup_ActiveArea{name}LimitMax")(setup, role))
            if hi > lo and not (lo <= float(value) <= hi):
                raise ValueError(
                    f"active area {field}={value} mm is outside the sensor's "
                    f"supported range [{lo:.3f}, {hi:.3f}] mm. These limits are "
                    "model- and configuration-dependent, so read them live with "
                    "get_active_area() rather than assuming the datasheet's."
                )

        for field, value in given.items():
            lib.call(
                f"GoSetup_SetActiveArea{field.capitalize()}",
                setup,
                role,
                _g.k64f(float(value)),
            )

        if flush:
            lib.call("GoSensor_Flush", self._sensor)

        applied = self.get_active_area()
        logger.info(
            "Gocator active area set: %s -> %s",
            given,
            {f"{f}_mm": applied[f"{f}_mm"] for f in self._ACTIVE_AREA_FIELDS},
        )
        return applied

    # ------------------------------------------------------------------
    # Subsampling / spacing interval / filters
    # ------------------------------------------------------------------

    def _uniform_spacing_is_on(self) -> bool:
        """Live uniform-spacing state, straight off the sensor."""
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        return bool(lib.go.GoSetup_UniformSpacingEnabled(setup))

    def get_subsampling(self) -> Dict[str, Any]:
        """Read the X/Z resolution dividers and the options this sensor offers.

        Subsampling is the cheapest large frame-rate win available: measured
        on this 2690 on 2026-08-02, ``x=2`` and ``x=4`` scale the ceiling by
        exactly 2.000x and 4.000x, at every active-area height, **in both
        uniform-spacing and point-cloud modes**. Z subsampling had no
        measurable effect on rate at all (ratio 1.000 across the board) —
        it trades Z resolution for nothing here, so leave it alone unless
        you have a specific reason.

        Returns:
            Dict with ``x``/``z`` (current divider), ``x_options``/
            ``z_options`` (the divisors this sensor supports, e.g. [1, 2, 4]),
            and ``x_system_value``/``z_system_value`` (the sensor's own
            defaults).
        """
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        role = _g.k32s(_g.GO_ROLE_MAIN)

        out: Dict[str, Any] = {}
        for axis in ("x", "z"):
            a = axis.upper()
            count = int(getattr(lib.go, f"GoSetup_{a}SubsamplingOptionCount")(setup, role))
            out[axis] = int(getattr(lib.go, f"GoSetup_{a}Subsampling")(setup, role))
            out[f"{axis}_options"] = [
                int(
                    getattr(lib.go, f"GoSetup_{a}SubsamplingOptionAt")(
                        setup, role, _g.kSize(i)
                    )
                )
                for i in range(count)
            ]
            out[f"{axis}_system_value"] = int(
                getattr(lib.go, f"GoSetup_{a}SubsamplingSystemValue")(setup, role)
            )
        return out

    def set_subsampling(
        self,
        x: Optional[int] = None,
        z: Optional[int] = None,
        flush: bool = True,
    ) -> Dict[str, Any]:
        """Set the X/Z resolution dividers (1 = full, 2 = half, 4 = quarter).

        Works in both uniform-spacing and point-cloud modes. X subsampling
        multiplies the frame-rate ceiling by exactly the divider, at the
        cost of X resolution — on this unit native 0.124 mm becomes 0.248 mm
        at x=2, 0.496 mm at x=4.

        Args:
            x, z: Dividers. Validated against the sensor's own option list
                rather than an assumed {1, 2, 4}, since it is model- and
                mode-dependent.
            flush: Push to the sensor. False to batch with other changes.

        Raises:
            RuntimeError: If not connected.
            ValueError: If a divider isn't offered by this sensor, or
                neither was given.
        """
        if x is None and z is None:
            raise ValueError("set_subsampling() needs x= and/or z=")

        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        role = _g.k32s(_g.GO_ROLE_MAIN)
        available = self.get_subsampling()

        for axis, value in (("x", x), ("z", z)):
            if value is None:
                continue
            options = available[f"{axis}_options"]
            if options and int(value) not in options:
                raise ValueError(
                    f"{axis}_subsampling={value} is not offered by this sensor; "
                    f"supported dividers are {options}"
                )

        for axis, value in (("x", x), ("z", z)):
            if value is None:
                continue
            lib.call(
                f"GoSetup_Set{axis.upper()}Subsampling",
                setup,
                role,
                _g.k32u(int(value)),
            )

        if flush:
            lib.call("GoSensor_Flush", self._sensor)
        applied = self.get_subsampling()
        logger.info("Gocator subsampling set: x=%s z=%s", applied["x"], applied["z"])
        return applied

    def get_spacing_interval(self) -> Dict[str, Any]:
        """Read the X resampling bin size (uniform spacing's grid pitch)."""
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        role = _g.k32s(_g.GO_ROLE_MAIN)
        return {
            "type": _name(
                _SPACING_INTERVAL_TYPES, lib.go.GoSetup_SpacingIntervalType(setup, role)
            ),
            "value_mm": float(lib.go.GoSetup_SpacingInterval(setup, role)),
            "limit_min": float(lib.go.GoSetup_SpacingIntervalLimitMin(setup, role)),
            "limit_max": float(lib.go.GoSetup_SpacingIntervalLimitMax(setup, role)),
            "system_value_mm": float(
                lib.go.GoSetup_SpacingIntervalSystemValue(setup, role)
            ),
            "custom_value_in_use": bool(
                lib.go.GoSetup_SpacingIntervalUsed(setup, role)
            ),
            "available": self._uniform_spacing_is_on(),
        }

    def set_spacing_interval(
        self,
        type: Optional[str] = None,
        value_mm: Optional[float] = None,
        flush: bool = True,
    ) -> Dict[str, Any]:
        """Set the X resampling bin size — **uniform spacing only**.

        This is the pitch of the grid uniform spacing resamples onto, i.e.
        the web UI's Resolution / Balanced / Speed presets plus Custom. A
        coarser interval means fewer X samples per profile.

        Args:
            type: One of ``max_res``, ``balanced``, ``max_speed``,
                ``custom``. Passing ``value_mm`` without a type implies
                ``custom``.
            value_mm: Explicit interval, mm. Only meaningful with
                ``custom``; validated against the sensor's live limits.
            flush: Push to the sensor. False to batch with other changes.

        Raises:
            UniformSpacingRequiredError: If uniform spacing is off — the
                resampled grid this sizes doesn't exist in point-cloud mode.
            ValueError: On an unknown type, an out-of-range value, or no
                arguments.
        """
        if type is None and value_mm is None:
            raise ValueError("set_spacing_interval() needs type= and/or value_mm=")
        if not self._uniform_spacing_is_on():
            raise UniformSpacingRequiredError(
                "set_spacing_interval() requires uniform spacing, but the sensor "
                "is in point-cloud mode (uniform_spacing=False). The spacing "
                "interval is the pitch of the resampled X grid, which doesn't "
                "exist when resampling is off. Enable uniform spacing first, or "
                "drop this setting."
            )
        if type is None:
            type = "custom"
        if type not in _SPACING_INTERVAL_TYPE_VALUES:
            raise ValueError(
                f"spacing interval type={type!r} unknown; expected one of "
                f"{sorted(_SPACING_INTERVAL_TYPE_VALUES)}"
            )

        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        role = _g.k32s(_g.GO_ROLE_MAIN)

        if value_mm is not None:
            lo = float(lib.go.GoSetup_SpacingIntervalLimitMin(setup, role))
            hi = float(lib.go.GoSetup_SpacingIntervalLimitMax(setup, role))
            if hi > lo and not (lo <= float(value_mm) <= hi):
                raise ValueError(
                    f"spacing interval {value_mm} mm is outside the sensor's "
                    f"supported range [{lo:.4f}, {hi:.4f}] mm"
                )

        lib.call(
            "GoSetup_SetSpacingIntervalType",
            setup,
            role,
            _g.k32s(_SPACING_INTERVAL_TYPE_VALUES[type]),
        )
        if value_mm is not None:
            lib.call(
                "GoSetup_SetSpacingInterval", setup, role, _g.k64f(float(value_mm))
            )

        if flush:
            lib.call("GoSensor_Flush", self._sensor)
        applied = self.get_spacing_interval()
        logger.info("Gocator spacing interval set: %s", applied)
        return applied

    def get_filters(self) -> Dict[str, Dict[str, Any]]:
        """Read every post-processing filter's state, window, and limits.

        Returns:
            ``{filter_name: {available, enabled, window_mm, window_limit_min,
            window_limit_max}}`` for each of ``_FILTERS``. ``available``
            mirrors the sensor's own ``GoSetup_*Used`` — false in
            point-cloud mode, since these act on the resampled grid.
        """
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        out: Dict[str, Dict[str, Any]] = {}
        for name, infix in _FILTERS.items():
            out[name] = {
                "available": bool(getattr(lib.go, f"GoSetup_{infix}Used")(setup)),
                "enabled": bool(getattr(lib.go, f"GoSetup_{infix}Enabled")(setup)),
                "window_mm": float(getattr(lib.go, f"GoSetup_{infix}Window")(setup)),
                "window_limit_min": float(
                    getattr(lib.go, f"GoSetup_{infix}WindowLimitMin")(setup)
                ),
                "window_limit_max": float(
                    getattr(lib.go, f"GoSetup_{infix}WindowLimitMax")(setup)
                ),
            }
        return out

    def set_filters(self, flush: bool = True, **filters: Any) -> Dict[str, Dict[str, Any]]:
        """Enable/disable post-processing filters — **uniform spacing only**.

        Filter names are the keys of ``_FILTERS``: ``x_smoothing``,
        ``x_median``, ``x_decimation``, ``x_gap_filling``, and the ``y_``
        equivalents. Each value may be:

          - ``False`` — disable the filter.
          - ``True`` — enable it, keeping its current window.
          - a number — enable it and set its window, in mm.

        All of these run on the resampled X grid, so they need uniform
        spacing; the sensor reports them as unavailable otherwise (verified
        on hardware: every ``Used`` flag flips 0 -> 1 with uniform spacing).

        Args:
            flush: Push to the sensor. False to batch with other changes.
            **filters: As above.

        Raises:
            UniformSpacingRequiredError: If uniform spacing is off.
            ValueError: On an unknown filter name, an out-of-range window,
                or no filters given.
        """
        if not filters:
            raise ValueError(
                f"set_filters() needs at least one of: {', '.join(_FILTERS)}"
            )
        unknown = [k for k in filters if k not in _FILTERS]
        if unknown:
            raise ValueError(
                f"unknown filter(s) {unknown}; expected any of {sorted(_FILTERS)}"
            )
        if not self._uniform_spacing_is_on():
            raise UniformSpacingRequiredError(
                f"set_filters({', '.join(sorted(filters))}) requires uniform "
                "spacing, but the sensor is in point-cloud mode "
                "(uniform_spacing=False). These filters run on the resampled X "
                "grid, which doesn't exist when resampling is off — the sensor "
                "reports them unavailable and the writes would be silently "
                "ignored. Enable uniform spacing, or drop the filters."
            )

        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)

        # Validate every window before writing any, so a bad value can't
        # leave half the filters applied.
        for name, spec in filters.items():
            if isinstance(spec, bool) or spec is None:
                continue
            infix = _FILTERS[name]
            lo = float(getattr(lib.go, f"GoSetup_{infix}WindowLimitMin")(setup))
            hi = float(getattr(lib.go, f"GoSetup_{infix}WindowLimitMax")(setup))
            if hi > lo and not (lo <= float(spec) <= hi):
                raise ValueError(
                    f"{name} window {spec} mm is outside the sensor's supported "
                    f"range [{lo:.4f}, {hi:.4f}] mm"
                )

        for name, spec in filters.items():
            infix = _FILTERS[name]
            if spec is None:
                continue
            enable = bool(spec) if isinstance(spec, bool) else True
            lib.call(
                f"GoSetup_Enable{infix}",
                setup,
                _g.kBool(_g.kTRUE if enable else _g.kFALSE),
            )
            if not isinstance(spec, bool):
                lib.call(f"GoSetup_Set{infix}Window", setup, _g.k64f(float(spec)))

        if flush:
            lib.call("GoSensor_Flush", self._sensor)
        applied = self.get_filters()
        logger.info(
            "Gocator filters set: %s",
            {k: (applied[k]["enabled"], applied[k]["window_mm"]) for k in filters},
        )
        return applied

    def solve_scan_rates(
        self,
        feed_rate_mm_s: Optional[float] = None,
        frame_rate_hz: Optional[float] = None,
        y_spacing_mm: Optional[float] = None,
        x_resolution_mm: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Solve the feed-rate / frame-rate / Y-spacing triangle.

        The three are locked together by one relation::

            y_spacing_mm = feed_rate_mm_s / frame_rate_hz

        Give any two and this returns the third. Give one and the missing
        pair is filled from the sensor: the frame rate defaults to the live
        ceiling, and the Y spacing to the X resolution (isotropic sampling).
        Give none and you get the fastest feed rate that still samples at
        least as finely along travel as across it.

        "Along travel" is the gantry axis the sensor's Y maps to — see
        ``self._mounting`` and :mod:`laguna.scanner.mounting`. The feed rate
        returned is the speed to command on **that** gantry axis, not on
        gantry Y.

        Why isotropic-or-better is the sensible default: on this rig Y
        spacing already comes out far finer than X resolution in every
        configuration measured, so surplus frame rate is better spent on a
        higher feed rate (shorter scans) than on Y detail that X can't match.

        Args:
            feed_rate_mm_s: Travel speed, mm/s.
            frame_rate_hz: Profile rate, Hz. Defaults to the sensor's live
                ceiling; a value above it is rejected.
            y_spacing_mm: Spacing between profiles along travel, mm.
            x_resolution_mm: Across-laser sample pitch, mm. Defaults to the
                sensor's configured spacing interval.

        Returns:
            Dict with ``feed_rate_mm_s``, ``frame_rate_hz``, ``y_spacing_mm``,
            ``x_resolution_mm``, ``frame_rate_max_hz`` (the live ceiling),
            ``travel_axis`` (the gantry axis this feed rate applies to),
            ``isotropic`` (is Y spacing <= X resolution), and
            ``aspect_ratio`` (y_spacing / x_resolution).

        Raises:
            RuntimeError: If not connected.
            ValueError: If all three are given and inconsistent, if any is
                non-positive, or if the frame rate exceeds the live ceiling.
        """
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        ceiling = float(lib.go.GoSetup_FrameRateLimitMax(setup))

        for label, value in (
            ("feed_rate_mm_s", feed_rate_mm_s),
            ("frame_rate_hz", frame_rate_hz),
            ("y_spacing_mm", y_spacing_mm),
            ("x_resolution_mm", x_resolution_mm),
        ):
            if value is not None and float(value) <= 0:
                raise ValueError(f"{label} must be positive, got {value!r}")

        if x_resolution_mm is None:
            # In uniform-spacing mode the spacing interval IS the X pitch —
            # 0.124 mm configured read back as x_resolution_nm=124000 in the
            # 2026-08-02 scans. In point-cloud mode X is non-uniform, so this
            # is a representative figure rather than an exact one.
            x_resolution_mm = float(lib.go.GoSetup_SpacingInterval(setup, _g.k32s(_g.GO_ROLE_MAIN)))

        given = sum(v is not None for v in (feed_rate_mm_s, frame_rate_hz, y_spacing_mm))
        if given == 3:
            implied = float(feed_rate_mm_s) / float(frame_rate_hz)
            if abs(implied - float(y_spacing_mm)) > 1e-9 * max(1.0, implied):
                raise ValueError(
                    f"feed_rate_mm_s={feed_rate_mm_s}, frame_rate_hz={frame_rate_hz} "
                    f"and y_spacing_mm={y_spacing_mm} are inconsistent: the first "
                    f"two imply {implied:.6g} mm. Give any two and let this solve "
                    "the third."
                )
        else:
            # Fill in from the sensor, then solve for whatever is still missing.
            if frame_rate_hz is None and (feed_rate_mm_s is None or y_spacing_mm is None):
                frame_rate_hz = ceiling if ceiling > 0 else None
            if y_spacing_mm is None and feed_rate_mm_s is None:
                y_spacing_mm = x_resolution_mm      # isotropic target

            if feed_rate_mm_s is None:
                if frame_rate_hz is None or y_spacing_mm is None:
                    raise ValueError(
                        "not enough information: give at least one of "
                        "feed_rate_mm_s / frame_rate_hz / y_spacing_mm, or "
                        "ensure the sensor reports a frame-rate ceiling"
                    )
                feed_rate_mm_s = float(frame_rate_hz) * float(y_spacing_mm)
            elif frame_rate_hz is None:
                frame_rate_hz = float(feed_rate_mm_s) / float(y_spacing_mm)
            elif y_spacing_mm is None:
                y_spacing_mm = float(feed_rate_mm_s) / float(frame_rate_hz)

        if ceiling > 0 and float(frame_rate_hz) > ceiling + 1e-6:
            raise ValueError(
                f"frame_rate_hz={float(frame_rate_hz):.3f} exceeds the sensor's "
                f"live ceiling of {ceiling:.3f} Hz in its current configuration. "
                "Raise the ceiling first — shrink the active area, use "
                "x subsampling, or enable uniform spacing (each measured to "
                "help; see docs/subsystems/scanner.md) — or lower the feed rate."
            )

        travel_axis = self._mounting.grid_axes()[0]
        result = {
            "feed_rate_mm_s": float(feed_rate_mm_s),
            "frame_rate_hz": float(frame_rate_hz),
            "y_spacing_mm": float(y_spacing_mm),
            "x_resolution_mm": float(x_resolution_mm),
            "frame_rate_max_hz": ceiling,
            "travel_axis": travel_axis,
            "isotropic": float(y_spacing_mm) <= float(x_resolution_mm) + 1e-12,
            "aspect_ratio": float(y_spacing_mm) / float(x_resolution_mm),
        }
        logger.info(
            "Scan rates: feed %.3f mm/s on gantry %s at %.3f Hz -> Y spacing "
            "%.4f mm (X resolution %.4f mm, ceiling %.1f Hz)",
            result["feed_rate_mm_s"], travel_axis, result["frame_rate_hz"],
            result["y_spacing_mm"], result["x_resolution_mm"], ceiling,
        )
        return result

    def _require_connected(self) -> GoSdkLib:
        if not self._is_connected or self._lib is None:
            raise RuntimeError(
                "GocatorScanner is not connected — call connect() first"
            )
        return self._lib

    def configure(
        self,
        travel_speed_mm_s: Optional[float] = None,
        frame_rate_hz: Optional[float] = None,
        frame_rate_max: Optional[bool] = None,
        fixed_length_mm: Optional[float] = None,
        exposure_us: Optional[float] = None,
        uniform_spacing: Optional[bool] = None,
        active_area: Optional[Dict[str, float]] = None,
        subsampling: Optional[Dict[str, int]] = None,
        spacing_interval: Optional[Dict[str, Any]] = None,
        filters: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Apply the encoderless software-triggered surface-scan recipe.

        Any argument left as None falls back to the value from config; keys
        absent from both are left at whatever the sensor already holds.

        ``travel_speed_mm_s`` writes to sensor **flash**, so it is only
        pushed when it actually differs from the sensor's current value
        (guarding against needless flash wear across repeated scans).

        Args:
            travel_speed_mm_s: Assumed constant velocity along travel, mm/s.
            frame_rate_hz: Profile rate. Pass 0 or None with no configured
                value to leave whatever frame-rate mode/rate the sensor
                already has untouched — this does NOT re-enable max-frame-
                rate mode if a prior configure() call disabled it. Mutually
                exclusive with frame_rate_max.
            frame_rate_max: If true, explicitly (re-)enable max-frame-rate
                mode and use whatever rate the sensor reports after
                flushing, regardless of prior state. Mutually exclusive
                with frame_rate_hz.
            fixed_length_mm: Surface length along travel, mm.
            exposure_us: Laser exposure, microseconds.
            uniform_spacing: X-axis resampling. This chooses which message
                type the sensor emits, and so what kind of data comes back:

                  - True  -> ranges are resampled onto even X bins, and the
                    sensor sends UNIFORM_SURFACE: Z only, with X implied by
                    column index. A resampled *heightmap*.
                  - False -> no resampling; the sensor sends
                    SURFACE_POINT_CLOUD, an explicit (x, y, z) triple per
                    point at the sensor's native, non-uniform X spacing. A
                    true *point cloud*.

                Disabling it also raises the achievable frame-rate ceiling
                (LMI lists "uniform spacing disabled" in the 2600-series
                high-speed recipe), so it interacts with frame_rate_max.
                None leaves the sensor's current setting alone.
            active_area: Region-of-interest dict, any subset of
                ``{x, y, z, width, length, height}`` in mm — see
                :meth:`set_active_area`. Applied *before* the frame rate,
                since shrinking it is what raises the ceiling the rate is
                then validated against. None falls back to the config's
                ``active_area``, and to leaving the sensor's own if neither
                is set.

        Returns:
            Dict of the values actually applied (read back from the sensor).

        Raises:
            RuntimeError: If not connected.
            ValueError: If both frame_rate_hz and frame_rate_max are given.
            GoSdkError: If any SDK call fails.
        """
        if frame_rate_hz is not None and frame_rate_max:
            raise ValueError(
                "configure() got both frame_rate_hz and frame_rate_max=True — "
                "these are mutually exclusive. Pass an explicit rate, or "
                "frame_rate_max=True to use the sensor's current maximum, "
                "not both."
            )

        # Catch uniform-spacing-only settings against the value this call is
        # about to apply, before writing anything. Checking the live sensor
        # state instead would miss the common case: turning uniform spacing
        # off and passing filters in the same call.
        effective_uniform = (
            uniform_spacing if uniform_spacing is not None else self._uniform_spacing
        )
        if effective_uniform is False:
            conflicting = [
                label
                for label, value in (("filters", filters),
                                     ("spacing_interval", spacing_interval))
                if value
            ]
            if conflicting:
                raise UniformSpacingRequiredError(
                    f"configure() got {' and '.join(conflicting)} together with "
                    "uniform_spacing=False (point-cloud mode). These act on the "
                    "resampled X grid, which doesn't exist when resampling is "
                    "off — the sensor reports them unavailable and the writes "
                    "would be silently ignored. Either enable uniform spacing "
                    "or drop them. (Subsampling is unaffected and works in "
                    "both modes.)"
                )

        lib = self._require_connected()

        speed = travel_speed_mm_s if travel_speed_mm_s is not None else self._travel_speed_mm_s
        use_max = frame_rate_max if frame_rate_max is not None else self._frame_rate_max
        # Ignore any stale self._frame_rate_hz from an earlier explicit-rate
        # configure() call when max mode is what's actually being asked for
        # this time — otherwise frame_rate_max=True could still trip the
        # mutual-exclusivity intent via carried-over state.
        rate = None if use_max else (frame_rate_hz if frame_rate_hz is not None else self._frame_rate_hz)
        length = fixed_length_mm if fixed_length_mm is not None else self._fixed_length_mm
        exposure = exposure_us if exposure_us is not None else self._exposure_us
        if uniform_spacing is not None:
            self._uniform_spacing = bool(uniform_spacing)

        setup = lib.handle("GoSensor_Setup", self._sensor)

        # Active area first: it drives the frame-rate ceiling that the rate
        # logic below validates against, so applying it afterwards would
        # validate against a stale limit. Deferred flush — the one at the end
        # of this method covers it.
        if active_area is not None:
            self._active_area = dict(active_area)
        if self._active_area:
            self.set_active_area(flush=False, **self._active_area)

        # Surface mode — a 3D scan, not individual profiles.
        lib.call("GoSetup_SetScanMode", setup, _g.k32s(_g.GO_MODE_SURFACE))

        # Time trigger: the encoderless path. Y spacing comes from travel speed.
        lib.call("GoSetup_SetTriggerSource", setup, _g.k32s(_g.GO_TRIGGER_TIME))

        if use_max:
            # Explicitly requested, regardless of whether a previous
            # configure() call left max-frame-rate mode disabled in sensor
            # flash — the achieved rate is read back after flushing, below,
            # since GoSetup_FrameRateLimitMax is itself dynamic (depends on
            # FOV/exposure/uniform spacing, and on max-frame-rate mode being
            # enabled at all — see docs/subsystems/scanner.md).
            lib.call("GoSetup_EnableMaxFrameRate", setup, _g.kBool(_g.kTRUE))
            self._frame_rate_hz = None   # unknown until read back post-flush
            self._frame_rate_max = True
        elif rate:
            # The achievable max depends on FOV/exposure/spacing, so it's read
            # live rather than assumed from the datasheet — on this 2690 at
            # stock settings it's ~443 Hz, far below the datasheet's 10 kHz
            # headline figure (which needs reduced FOV + uniform spacing off).
            lo = float(lib.go.GoSetup_FrameRateLimitMin(setup))
            hi = float(lib.go.GoSetup_FrameRateLimitMax(setup))
            if hi > 0 and not (lo <= float(rate) <= hi):
                raise ValueError(
                    f"frame_rate_hz={rate} outside the sensor's current supported "
                    f"range [{lo:.3f}, {hi:.3f}] Hz. The upper limit depends on "
                    "field of view, exposure, and uniform spacing — lower the "
                    "rate, or relax those settings to raise the ceiling."
                )
            lib.call("GoSetup_EnableMaxFrameRate", setup, _g.kBool(_g.kFALSE))
            lib.call("GoSetup_SetFrameRate", setup, _g.k64f(float(rate)))
            self._frame_rate_hz = float(rate)
            self._frame_rate_max = False

        if exposure:
            lib.call(
                "GoSetup_SetExposure",
                setup,
                _g.k32s(_g.GO_ROLE_MAIN),
                _g.k64f(float(exposure)),
            )
            self._exposure_us = float(exposure)

        if self._uniform_spacing is not None:
            lib.call(
                "GoSetup_EnableUniformSpacing",
                setup,
                _g.kBool(_g.kTRUE if self._uniform_spacing else _g.kFALSE),
            )

        # Subsampling before the spacing interval and filters: it works in
        # both modes and shifts the X resolution those are then sized
        # against. Each defers its own flush to the single one below.
        if subsampling is not None:
            self._subsampling = dict(subsampling)
        if self._subsampling:
            self.set_subsampling(flush=False, **self._subsampling)

        # These two need uniform spacing, which was just applied above — so
        # their own live checks now see the intended state, not the previous
        # one. The guard at the top of this method has already rejected the
        # conflicting combination.
        if spacing_interval is not None:
            self._spacing_interval = dict(spacing_interval)
        if self._spacing_interval:
            self.set_spacing_interval(flush=False, **self._spacing_interval)

        if filters is not None:
            self._filters = dict(filters)
        if self._filters:
            self.set_filters(flush=False, **self._filters)

        # Fixed-length surface, started by our software trigger.
        surface = lib.handle("GoSetup_SurfaceGeneration", setup)
        lib.call(
            "GoSurfaceGeneration_SetGenerationType",
            surface,
            _g.k32s(_g.GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH),
        )
        lib.call(
            "GoSurfaceGenerationFixedLength_SetStartTrigger",
            surface,
            _g.k32s(_g.GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE),
        )
        if length:
            lo = float(lib.go.GoSurfaceGenerationFixedLength_LengthLimitMin(surface))
            hi = float(lib.go.GoSurfaceGenerationFixedLength_LengthLimitMax(surface))
            if hi > 0 and not (lo <= float(length) <= hi):
                raise ValueError(
                    f"fixed_length_mm={length} outside the sensor's supported "
                    f"range [{lo}, {hi}] mm"
                )
            lib.call(
                "GoSurfaceGenerationFixedLength_SetLength",
                surface,
                _g.k64f(float(length)),
            )
            self._fixed_length_mm = float(length)

        # Travel speed lives on GoTransform and writes to flash — only touch
        # it when it actually changes.
        if speed:
            transform = lib.handle("GoSensor_Transform", self._sensor)
            current = float(lib.go.GoTransform_Speed(transform))
            if abs(current - float(speed)) > 1e-6:
                logger.info(
                    "Updating Gocator travel speed %.4f -> %.4f mm/s (writes flash)",
                    current,
                    float(speed),
                )
                lib.call("GoTransform_SetSpeed", transform, _g.k64f(float(speed)))
            self._travel_speed_mm_s = float(speed)

        lib.call("GoSensor_Flush", self._sensor)

        if use_max:
            # There is no "requested" rate to validate against here — max
            # mode means "whatever the sensor can do," so just read it back
            # for Y-spacing bookkeeping (see the module docstring's
            # encoderless-scanning note: Y spacing = travel_speed /
            # frame_rate, so downstream code needs a real number here, not
            # None).
            self._frame_rate_hz = float(lib.go.GoSetup_FrameRate(setup))
            logger.info(
                "Max frame rate mode: sensor running at %.3f Hz",
                self._frame_rate_hz,
            )

        # Re-check the frame rate *after* flushing. The sensor's reported
        # ceiling is dynamic — observed on hardware 2026-07-30 dropping from
        # 443.127 Hz to 221.563 Hz once max-frame-rate mode was disabled — so
        # the pre-write check above can pass and still leave the sensor holding
        # an unachievable rate. That matters because Y spacing is
        # travel_speed / frame_rate: if the sensor silently runs slower than we
        # asked, the travel axis is scaled wrong and the scan is quietly
        # distorted rather than obviously broken. Only meaningful for the
        # explicit-rate path — use_max already read back the true rate above.
        if not use_max and self._frame_rate_hz:
            achieved = float(lib.go.GoSetup_FrameRate(setup))
            ceiling = float(lib.go.GoSetup_FrameRateLimitMax(setup))
            if ceiling > 0 and self._frame_rate_hz > ceiling + 1e-6:
                raise ValueError(
                    f"Sensor reports a maximum frame rate of {ceiling:.3f} Hz "
                    f"after applying this configuration, but frame_rate_hz is "
                    f"{self._frame_rate_hz:.3f} Hz. The sensor cannot deliver "
                    "that rate, so Y spacing (travel_speed / frame_rate) would "
                    f"be wrong. Set frame_rate_hz <= {ceiling:.3f}, or reduce "
                    "exposure / field of view / disable uniform spacing to "
                    "raise the ceiling. Note this ceiling is dynamic — it "
                    "depends on the rest of the configuration."
                )
            if abs(achieved - self._frame_rate_hz) > 1e-3:
                logger.warning(
                    "Sensor accepted frame rate %.3f Hz but reports %.3f Hz; "
                    "using the reported value for Y-spacing bookkeeping.",
                    self._frame_rate_hz,
                    achieved,
                )
                self._frame_rate_hz = achieved

        applied = {
            "travel_speed_mm_s": self._travel_speed_mm_s,
            "frame_rate_hz": self._frame_rate_hz,
            "frame_rate_max": use_max,
            "fixed_length_mm": self._fixed_length_mm,
            "uniform_spacing": self._uniform_spacing,
            "active_area": self._active_area,
            "subsampling": self._subsampling,
            "spacing_interval": self._spacing_interval,
            "filters": self._filters,
            "exposure_us": self._exposure_us,
            "trigger_source": "time",
            "surface_generation": "fixed_length",
            "start_trigger": "software",
        }
        logger.info("Gocator configured: %s", applied)
        return applied

    # ------------------------------------------------------------------
    # Scan lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Enable the data channel and start acquisition.

        The sensor now runs, but no surface is generated until :meth:`trigger`
        fires the software start trigger.
        """
        lib = self._require_connected()
        lib.call("GoSystem_EnableData", self._system, _g.kBool(_g.kTRUE))
        lib.call("GoSystem_Start", self._system)
        self._is_running = True
        logger.info("Gocator acquisition started (awaiting software trigger)")

    def trigger(self) -> None:
        """Fire the software start trigger, beginning one fixed-length surface.

        Call this once the gantry has reached constant velocity — the sensor
        starts accumulating profiles immediately.
        """
        lib = self._require_connected()
        if not self._is_running:
            raise RuntimeError("call start() before trigger()")
        lib.call("GoSensor_Trigger", self._sensor)
        logger.info("Gocator software trigger fired")

    def stop(self) -> None:
        """Stop acquisition."""
        lib = self._require_connected()
        lib.call("GoSystem_Stop", self._system)
        self._is_running = False
        logger.info("Gocator acquisition stopped")

    def receive_surface(
        self,
        timeout_s: float = DEFAULT_RECEIVE_TIMEOUT_S,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> SurfaceScan:
        """Poll the data channel until a surface message arrives.

        Drains stamp/intensity/profile messages, returning as soon as a
        ``UNIFORM_SURFACE`` or ``SURFACE_POINT_CLOUD`` message is found.

        Args:
            timeout_s: Total wall-clock budget for receiving a surface.
            metadata: Extra context merged into the result's metadata.

        Returns:
            The converted :class:`SurfaceScan`.

        Raises:
            RuntimeError: If not started.
            TimeoutError: If no surface message arrives within `timeout_s`.
            GoSdkError: On a non-timeout SDK failure.
        """
        lib = self._require_connected()
        if not self._is_running:
            raise RuntimeError("call start() before receive_surface()")

        deadline = time.monotonic() + timeout_s
        stamps: Dict[str, Any] = {}

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"No Gocator surface message within {timeout_s:.1f} s. "
                    "Check that the software trigger fired, that "
                    "fixed_length_mm is reachable at the configured travel "
                    "speed, and that the gantry actually moved."
                )

            dataset = _g.kObject()
            try:
                lib.call(
                    "GoSystem_ReceiveData",
                    self._system,
                    byref(dataset),
                    _g.k64u(int(remaining * 1_000_000)),  # SDK wants µs
                )
            except GoSdkTimeout:
                continue

            try:
                scan = self._extract_surface(dataset, stamps, metadata)
            finally:
                lib.go.GoDestroy(dataset)

            if scan is not None:
                self._scan_count += 1
                self._last_scan_meta = dict(scan.metadata)
                return scan

    def _extract_surface(
        self,
        dataset,
        stamps: Dict[str, Any],
        metadata: Optional[Dict[str, Any]],
    ) -> Optional[SurfaceScan]:
        """Scan one GoDataSet for a surface message, harvesting stamps too.

        Returns None when the dataset holds no surface (stamps/intensity
        only), signalling the caller to keep polling.
        """
        lib = self._lib
        assert lib is not None
        go = lib.go

        count = int(go.GoDataSet_Count(dataset))
        for i in range(count):
            msg = go.GoDataSet_At(dataset, _g.kSize(i))
            if not msg:
                continue
            msg = ctypes.c_void_p(msg)
            msg_type = int(go.GoDataMsg_Type(msg))

            if msg_type == _g.GO_DATA_MESSAGE_TYPE_STAMP:
                stamps.update(self._read_first_stamp(msg))
                continue

            if msg_type in (
                _g.GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE,
                _g.GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD,
            ):
                meta: Dict[str, Any] = {
                    "ip": self._ip,
                    "travel_speed_mm_s": self._travel_speed_mm_s,
                    "frame_rate_hz": self._frame_rate_hz,
                    "fixed_length_mm": self._fixed_length_mm,
                    "exposure_us": self._exposure_us,
                    "received_wall_time": time.time(),
                }
                meta.update(stamps)
                if metadata:
                    meta.update(metadata)

                if msg_type == _g.GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
                    return uniform_surface_to_scan(lib, msg, meta, self._mounting)
                return surface_point_cloud_to_scan(lib, msg, meta, self._mounting)

        return None

    def _read_first_stamp(self, msg) -> Dict[str, Any]:
        """Pull frame/time info from the first stamp in a stamp message."""
        lib = self._lib
        assert lib is not None
        try:
            if int(lib.go.GoStampMsg_Count(msg)) == 0:
                return {}
            stamp_ptr = lib.go.GoStampMsg_At(msg, _g.kSize(0))
            if not stamp_ptr:
                return {}
            stamp = stamp_ptr[0]
            return {
                "frame_index": int(stamp.frameIndex),
                # Header: true µs = timestamp / 1.024
                "timestamp_us": int(stamp.timestamp) / 1.024,
                "ptp_time_us": int(stamp.ptpTime),
            }
        except Exception as e:  # pragma: no cover - stamp is nice-to-have
            logger.debug("Could not read stamp: %s", e)
            return {}

    # ------------------------------------------------------------------
    # High-level scan
    # ------------------------------------------------------------------

    def scan(
        self,
        timeout_s: Optional[float] = None,
        metadata: Optional[Dict[str, Any]] = None,
        configure: bool = True,
    ) -> SurfaceScan:
        """Run one standalone surface scan: configure, start, trigger, receive.

        This does **not** move the gantry — motion must already be underway
        (or the target moving) when this is called, since the software
        trigger fires immediately. Use :meth:`scan_with_gantry` to have the
        move coordinated for you.

        Args:
            timeout_s: Receive budget. Defaults to the time the configured
                fixed length takes at the configured travel speed, plus 50%
                headroom and a 5 s floor.
            metadata: Extra context merged into the result metadata.
            configure: Apply :meth:`configure` first. Pass False if you've
                already configured and want to avoid re-touching settings.

        Returns:
            The captured :class:`SurfaceScan`.
        """
        if configure:
            self.configure()

        if timeout_s is None:
            timeout_s = self._default_timeout_s()

        started_here = not self._is_running
        if started_here:
            self.start()
        try:
            self.trigger()
            return self.receive_surface(timeout_s=timeout_s, metadata=metadata)
        finally:
            if started_here and self._is_running:
                try:
                    self.stop()
                except GoSdkError as e:
                    logger.warning("Error stopping after scan: %s", e)

    def _default_timeout_s(self) -> float:
        """Time for the configured length at the configured speed, +50%."""
        if self._fixed_length_mm and self._travel_speed_mm_s:
            travel_s = float(self._fixed_length_mm) / float(self._travel_speed_mm_s)
            return max(5.0, travel_s * 1.5)
        return DEFAULT_RECEIVE_TIMEOUT_S

    def scan_with_gantry(
        self,
        gantry,
        axis: str,
        end_mm: float,
        feed_rate_mm_s: float,
        settle_s: float = 0.5,
        fixed_length_mm: Optional[float] = None,
        metadata: Optional[Dict[str, Any]] = None,
        timeout_s: Optional[float] = None,
    ) -> SurfaceScan:
        """Coordinate a gantry pass and a triggered surface capture.

        Configures the sensor's travel speed to match `feed_rate_mm_s`
        (that match is what makes the Y axis correct), starts a non-blocking
        single-axis move, waits `settle_s` for the axis to come off its
        acceleration ramp, fires the software trigger, and receives the
        surface.

        Drives the axis through its ``AxisHandle`` (``gantry.axis(name)``)
        rather than ``gantry.move_to()``, because the trigger has to fire
        *while* the axis is mid-move and ``move_to()`` runs the coordinated
        gcode path, blocking until the move finishes. AxisHandle's
        ``begin_move_to`` is non-blocking and — importantly — still enforces
        the gantry's ``safe_mode`` gate, which the raw ``gantry.cmd`` path
        does not on the ethernet/rs232 transports.

        Note this path does **not** fence-check the target the way
        ``move_to()`` does, so the caller is responsible for the destination
        being inside the work envelope.

        Args:
            gantry: A connected GantryController.
            axis: Configured axis *name*, e.g. "X" or "Y".
            end_mm: Absolute target position, real mm.
            feed_rate_mm_s: Constant slew speed, mm/s — also written to the
                sensor as travel speed.
            settle_s: Delay between commanding motion and triggering, to skip
                the acceleration ramp. Increase if the leading edge of scans
                looks compressed along Y.
            fixed_length_mm: Surface length to configure on the sensor, mm.
                Defaults to None, meaning: derive it from the axis's actual
                position right now and `end_mm` (``abs(end_mm - current)``),
                so the sensor's capture window matches this call's commanded
                move by construction — it does *not* fall back to a
                previously-configured ``gocator.fixed_length_mm`` that could
                silently be scanning a different distance than `end_mm`
                implies. Pass an explicit value to deliberately scan only
                part of a longer traverse; a mismatch against the derived
                distance is logged as a warning, not rejected, since that
                may be exactly what's wanted.

        Returns:
            The captured :class:`SurfaceScan`, with gantry context in
            ``metadata``.

        Raises:
            RuntimeError: If not connected.
            KeyError: If `axis` isn't a configured axis on this gantry.
            SnapMotionError: If the gantry's safe_mode blocks the move.
        """
        handle = gantry.axis(axis)

        start_mm = None
        try:
            start_mm = handle.get_position()
        except Exception as e:
            logger.debug("Could not read gantry start position: %s", e)

        distance_mm = abs(end_mm - start_mm) if start_mm is not None else None

        if fixed_length_mm is None:
            if distance_mm is not None:
                fixed_length_mm = distance_mm
            else:
                fixed_length_mm = self._fixed_length_mm
                logger.warning(
                    "Could not read %s's current position, so fixed_length_mm "
                    "can't be derived from end_mm=%.3f — falling back to the "
                    "already-configured value (%s mm). If that doesn't match "
                    "the actual travel distance, the sensor will stop "
                    "generating the surface before (or long after) the move "
                    "finishes.",
                    axis, end_mm, self._fixed_length_mm,
                )
        elif distance_mm is not None and abs(fixed_length_mm - distance_mm) > 1e-6:
            logger.warning(
                "fixed_length_mm=%.3f mm was given explicitly, but %s "
                "%.3f -> %.3f mm implies %.3f mm of travel. The sensor stops "
                "generating the surface after fixed_length_mm regardless of "
                "how far the gantry actually goes — make sure that's what "
                "you want.",
                fixed_length_mm, axis, start_mm, end_mm, distance_mm,
            )

        self.configure(travel_speed_mm_s=feed_rate_mm_s, fixed_length_mm=fixed_length_mm)

        if timeout_s is None:
            timeout_s = self._default_timeout_s()
            if distance_mm and feed_rate_mm_s > 0:
                timeout_s = max(timeout_s, distance_mm / feed_rate_mm_s * 1.5 + 5.0)

        meta: Dict[str, Any] = {
            "gantry_axis": axis,
            "gantry_start_mm": start_mm,
            "gantry_end_mm": end_mm,
            "gantry_feed_rate_mm_s": feed_rate_mm_s,
            "trigger_settle_s": settle_s,
        }
        if metadata:
            meta.update(metadata)

        self.start()
        try:
            logger.info(
                "Starting gantry pass %s -> %.3f mm at %.3f mm/s",
                axis,
                end_mm,
                feed_rate_mm_s,
            )
            handle.set_speed(feed_rate_mm_s)
            handle.begin_move_to(end_mm)   # safe_mode-gated, non-blocking
            time.sleep(settle_s)
            self.trigger()
            return self.receive_surface(timeout_s=timeout_s, metadata=meta)
        finally:
            if self._is_running:
                try:
                    self.stop()
                except GoSdkError as e:
                    logger.warning("Error stopping after gantry scan: %s", e)

    # ------------------------------------------------------------------
    # Saving
    # ------------------------------------------------------------------

    #: Formats whose writers consume a flattened (N, 3) point array, so
    #: save_scan() can build it once and share it across all of them.
    _POINT_FORMATS = ("laz", "las", "ply", "csv")

    def save_scan(
        self,
        scan: SurfaceScan,
        name: Optional[str] = None,
        formats: tuple = ("npz", "laz"),
    ) -> Dict[str, Path]:
        """Save a scan under ``output_dir`` in the requested formats.

        Any point-cloud formats requested share a single ``to_points()``
        call rather than each rebuilding the flattened array — that array
        is hundreds of MB at full resolution, so recomputing it per format
        was a significant part of the old save cost.

        Args:
            scan: The scan to write.
            name: Base filename without extension. Defaults to a UTC
                timestamp, ``scan_YYYYmmdd_HHMMSS``.
            formats: Any of "npz" (full grid incl. NaNs, best for
                reprocessing), "laz"/"las" (ASPRS point cloud — LAZ is the
                fastest and smallest of these by a wide margin), "ply"
                (CloudCompare/MeshLab), "csv" (x,y,z text; slow and huge —
                see :meth:`SurfaceScan.save_csv`).

        Returns:
            Dict mapping format name to the written path.

        Raises:
            ValueError: On an unknown format name.
        """
        import datetime as _dt

        if name is None:
            name = "scan_" + _dt.datetime.now(_dt.timezone.utc).strftime(
                "%Y%m%d_%H%M%S"
            )
        self._output_dir.mkdir(parents=True, exist_ok=True)
        base = self._output_dir / name

        unknown = [f for f in formats if f not in self._POINT_FORMATS + ("npz",)]
        if unknown:
            raise ValueError(f"Unknown scan format: {unknown[0]!r}")

        # float32 is ~6e-5 mm at 1000 mm — two orders finer than the
        # sensor's 12 µm Z repeatability, and halves the array.
        points = (
            scan.to_points(drop_invalid=True, dtype=np.float32)
            if any(f in self._POINT_FORMATS for f in formats)
            else None
        )

        written: Dict[str, Path] = {}
        for fmt in formats:
            if fmt == "npz":
                written["npz"] = scan.save_npz(base.with_suffix(".npz"))
            elif fmt in ("laz", "las"):
                written[fmt] = scan.save_las(
                    base.with_suffix("." + fmt), points=points
                )
            elif fmt == "ply":
                written["ply"] = scan.save_ply(base.with_suffix(".ply"), points=points)
            elif fmt == "csv":
                written["csv"] = scan.save_csv(base.with_suffix(".csv"), points=points)
        logger.info("Saved scan: %s", {k: str(v) for k, v in written.items()})
        return written

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "GocatorScanner":
        """Build from a config dict — mirrors GantryController.from_config()."""
        return cls(config)
