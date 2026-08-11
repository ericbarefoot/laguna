"""Gocator configuration surface — everything that *sets up* a scan.

Split out from :mod:`laguna.scanner.gocator`, which keeps the acquisition
lifecycle (connect / start / trigger / receive / save). The two concerns had
grown tangled in one 1600-line module: this half decides *how the sensor is
configured*, the other half *runs a scan and gets the data back*.

Mixed into :class:`~laguna.scanner.gocator.GocatorScanner`, so the public API
is unchanged — ``scanner.set_active_area(...)`` still works exactly as before.
The mixin relies on the host class providing ``_lib``/``_sensor``,
``_require_connected()``, and the ``_travel_speed_mm_s``-style config
attributes set in its ``__init__``.

What lives here, and what each knob is worth (all measured on hardware
2026-08-02 — see docs/subsystems/scanner.md for the full sweep):

  - **active area** — the region of interest. The main lever for scan speed:
    fewer camera rows per profile, higher frame-rate ceiling. Sublinear, and
    it saturates above ~800 mm of height.
  - **subsampling** — x2/x4 multiply the ceiling *exactly*, in both uniform
    and point-cloud modes. Z subsampling does nothing for rate.
  - **spacing interval** and **filters** — act on the resampled X grid, so
    they require uniform spacing and raise
    :class:`UniformSpacingRequiredError` otherwise.
  - **frame rate**, **travel speed**, **fixed length**, **exposure**, and the
    encoderless recipe itself, applied by :meth:`configure`.
  - :meth:`solve_scan_rates` — ties feed rate, frame rate and Y spacing
    together.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from . import gosdk as _g

logger = logging.getLogger(__name__)


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

class GocatorSettingsMixin:
    """The Gocator's configuration surface.

    Mixed into :class:`~laguna.scanner.gocator.GocatorScanner`; see this
    module's docstring for the split and for what the host class must supply.
    """

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
            x: Active-area origin X, mm.
            y: Active-area origin Y, mm.
            z: Active-area origin Z, mm.
            width: Active-area extent in X (across laser), mm.
            length: Active-area extent in Y (along travel), mm.
            height: Active-area extent in Z (range/standoff), mm.
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

        # configure() re-applies self._active_area on every call (any field
        # it's already tracking, not just ones this particular configure()
        # call touches) — without updating it here too, a direct
        # set_active_area() call would be silently reverted by the next
        # configure() call for an unrelated setting (e.g. just frame_rate),
        # back to whatever configure() last knew about. Merge rather than
        # replace, matching "only the fields you pass are written" above.
        self._active_area = {**(self._active_area or {}), **given}

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
            x: X-axis resolution divider. Validated against the sensor's own
                option list rather than an assumed {1, 2, 4}, since it is
                model- and mode-dependent.
            z: Z-axis resolution divider (for standoff resolution).
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

        # See set_active_area()'s matching comment: configure() re-applies
        # self._subsampling on every call, so a direct set_subsampling()
        # call has to update it too or the next unrelated configure() call
        # would silently revert it.
        given = {k: v for k, v in (("x", x), ("z", z)) if v is not None}
        self._subsampling = {**(self._subsampling or {}), **given}

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

        # See set_active_area()'s matching comment. Uses the resolved
        # `type` (defaulted to "custom" above when omitted), not the raw
        # parameter, so the cache reflects what was actually applied.
        given = {k: v for k, v in (("type", type), ("value_mm", value_mm)) if v is not None}
        self._spacing_interval = {**(self._spacing_interval or {}), **given}

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

        # See set_active_area()'s matching comment.
        self._filters = {**(self._filters or {}), **filters}

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

    # ------------------------------------------------------------------
    # Frame rate
    # ------------------------------------------------------------------

    def get_frame_rate(self) -> Dict[str, Any]:
        """Read the sensor's current profile frame rate and live ceiling.

        The ceiling is dynamic — it depends on field of view (active
        area), exposure, and uniform spacing, and on whether max-frame-
        rate mode is enabled at all (see docs/subsystems/scanner.md).

        Returns:
            Dict with ``hz`` (current rate), ``limit_min``/``limit_max``
            (the sensor's live range), and ``max_mode`` (whether max-
            frame-rate mode is currently enabled).
        """
        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)
        return {
            "hz": float(lib.go.GoSetup_FrameRate(setup)),
            "limit_min": float(lib.go.GoSetup_FrameRateLimitMin(setup)),
            "limit_max": float(lib.go.GoSetup_FrameRateLimitMax(setup)),
            "max_mode": bool(lib.go.GoSetup_MaxFrameRateEnabled(setup)),
        }

    def set_frame_rate(
        self,
        hz: Optional[float] = None,
        max_mode: Optional[bool] = None,
        flush: bool = True,
    ) -> Dict[str, Any]:
        """Set the profile frame rate — an explicit Hz, or max-frame-rate mode.

        Mirrors configure()'s frame_rate_hz/frame_rate_max handling, split
        out as a standalone setter for changing just the rate without
        re-running the rest of the encoderless recipe (scan mode, trigger
        source, surface generation, ...) that configure() also touches.

        Args:
            hz: Explicit frame rate, Hz. Validated against the sensor's
                current live ceiling (see get_frame_rate). Mutually
                exclusive with max_mode.
            max_mode: If true, (re-)enable max-frame-rate mode and use
                whatever rate the sensor reports after flushing —
                regardless of prior state, same as configure()'s
                frame_rate_max. Mutually exclusive with hz.
            flush: Push to the sensor with GoSensor_Flush. Pass False to
                batch this with other changes and flush once yourself —
                the achieved-rate readback below is still accurate either
                way, since GoSetup's getters reflect staged (not-yet-
                flushed) values immediately.

        Returns:
            Dict with ``frame_rate_hz`` and ``frame_rate_max`` — the
            values actually applied (read back from the sensor).

        Raises:
            RuntimeError: If not connected.
            ValueError: If both hz and max_mode are given, neither is
                given, or hz is outside the sensor's current range.
            GoSdkError: If any SDK call fails.
        """
        if hz is not None and max_mode:
            raise ValueError(
                "set_frame_rate() got both hz and max_mode=True — these are "
                "mutually exclusive. Pass an explicit rate, or max_mode=True "
                "to use the sensor's current maximum, not both."
            )
        if hz is None and not max_mode:
            raise ValueError("set_frame_rate() needs hz= and/or max_mode=True")

        lib = self._require_connected()
        setup = lib.handle("GoSensor_Setup", self._sensor)

        if max_mode:
            lib.call("GoSetup_EnableMaxFrameRate", setup, _g.kBool(_g.kTRUE))
        else:
            lo = float(lib.go.GoSetup_FrameRateLimitMin(setup))
            hi = float(lib.go.GoSetup_FrameRateLimitMax(setup))
            if hi > 0 and not (lo <= float(hz) <= hi):
                raise ValueError(
                    f"frame_rate hz={hz} is outside the sensor's current supported "
                    f"range [{lo:.3f}, {hi:.3f}] Hz. The upper limit depends on "
                    "field of view, exposure, and uniform spacing — lower the "
                    "rate, or relax those settings to raise the ceiling."
                )
            lib.call("GoSetup_EnableMaxFrameRate", setup, _g.kBool(_g.kFALSE))
            lib.call("GoSetup_SetFrameRate", setup, _g.k64f(float(hz)))

        if flush:
            lib.call("GoSensor_Flush", self._sensor)

        achieved = float(lib.go.GoSetup_FrameRate(setup))
        if not max_mode and abs(achieved - float(hz)) > 1e-3:
            logger.warning(
                "Sensor accepted frame rate %.3f Hz but reports %.3f Hz; "
                "using the reported value for Y-spacing bookkeeping.",
                float(hz), achieved,
            )

        # See set_active_area()'s matching comment: configure() re-applies
        # self._frame_rate_hz/self._frame_rate_max whenever a caller
        # doesn't explicitly override them, so this direct call has to
        # update both or the next unrelated configure() call would
        # silently revert the rate.
        self._frame_rate_hz = achieved
        self._frame_rate_max = bool(max_mode)

        logger.info(
            "Gocator frame rate set: hz=%s max_mode=%s -> %.3f Hz",
            hz, max_mode, achieved,
        )
        return {"frame_rate_hz": self._frame_rate_hz, "frame_rate_max": self._frame_rate_max}

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
            subsampling: X/Z resolution divider dict, e.g. ``{x: 2, z: 1}``.
                Works in both uniform-spacing and point-cloud modes.
            spacing_interval: X resampling bin size config dict
                (``type`` and/or ``value_mm``). Requires uniform spacing.
            filters: Post-processing filter dict, with filter names as keys
                and bool or window size (mm) as values. Requires uniform
                spacing.

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
