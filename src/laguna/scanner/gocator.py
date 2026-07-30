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

from . import gosdk as _g
from .gosdk import GoSdkError, GoSdkLib, GoSdkTimeout
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
            frame_rate_hz: Profile trigger rate. Omit (or null) to let the
                sensor run at its maximum rate.
            exposure_us: Optional laser exposure in microseconds.
            fixed_length_mm: Surface length along travel, mm. Should cover
                the intended scan distance.
            uniform_spacing: If set, enable/disable X resampling. Leaving it
                unset keeps whatever the sensor is configured with.
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
        self._exposure_us = config.get("exposure_us")
        self._fixed_length_mm = config.get("fixed_length_mm")
        self._uniform_spacing = config.get("uniform_spacing")
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
            "fixed_length_mm": self._fixed_length_mm,
            "last_scan": self._last_scan_meta or None,
        }
        if self._is_connected and self._lib:
            try:
                setup = self._lib.handle("GoSensor_Setup", self._sensor)
                transform = self._lib.handle("GoSensor_Transform", self._sensor)
                status.update(
                    {
                        "sensor_travel_speed_mm_s": float(
                            self._lib.go.GoTransform_Speed(transform)
                        ),
                        "sensor_frame_rate_hz": float(
                            self._lib.go.GoSetup_FrameRate(setup)
                        ),
                        "sensor_trigger_source": int(
                            self._lib.go.GoSetup_TriggerSource(setup)
                        ),
                        "sensor_scan_mode": int(self._lib.go.GoSetup_ScanMode(setup)),
                    }
                )
            except GoSdkError as e:
                status["status_read_error"] = str(e)
        return status

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

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
        fixed_length_mm: Optional[float] = None,
        exposure_us: Optional[float] = None,
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
                value to leave max-frame-rate mode alone.
            fixed_length_mm: Surface length along travel, mm.
            exposure_us: Laser exposure, microseconds.

        Returns:
            Dict of the values actually applied (read back from the sensor).

        Raises:
            RuntimeError: If not connected.
            GoSdkError: If any SDK call fails.
        """
        lib = self._require_connected()

        speed = travel_speed_mm_s if travel_speed_mm_s is not None else self._travel_speed_mm_s
        rate = frame_rate_hz if frame_rate_hz is not None else self._frame_rate_hz
        length = fixed_length_mm if fixed_length_mm is not None else self._fixed_length_mm
        exposure = exposure_us if exposure_us is not None else self._exposure_us

        setup = lib.handle("GoSensor_Setup", self._sensor)

        # Surface mode — a 3D scan, not individual profiles.
        lib.call("GoSetup_SetScanMode", setup, _g.k32s(_g.GO_MODE_SURFACE))

        # Time trigger: the encoderless path. Y spacing comes from travel speed.
        lib.call("GoSetup_SetTriggerSource", setup, _g.k32s(_g.GO_TRIGGER_TIME))

        if rate:
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

        applied = {
            "travel_speed_mm_s": self._travel_speed_mm_s,
            "frame_rate_hz": self._frame_rate_hz,
            "fixed_length_mm": self._fixed_length_mm,
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
                    return uniform_surface_to_scan(lib, msg, meta)
                return surface_point_cloud_to_scan(lib, msg, meta)

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
        does not on the socket_bridge/ethernet/rs232 transports.

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
            metadata: Extra context merged into the result metadata.
            timeout_s: Receive budget. Defaults to the time the move itself
                should take (distance / feed rate) plus 50% and 5 s.

        Returns:
            The captured :class:`SurfaceScan`, with gantry context in
            ``metadata``.

        Raises:
            RuntimeError: If not connected.
            KeyError: If `axis` isn't a configured axis on this gantry.
            SnapMotionError: If the gantry's safe_mode blocks the move.
        """
        handle = gantry.axis(axis)

        self.configure(travel_speed_mm_s=feed_rate_mm_s)

        start_mm = None
        try:
            start_mm = handle.get_position()
        except Exception as e:
            logger.debug("Could not read gantry start position: %s", e)

        distance_mm = abs(end_mm - start_mm) if start_mm is not None else None
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

    def save_scan(
        self,
        scan: SurfaceScan,
        name: Optional[str] = None,
        formats: tuple = ("npz", "ply"),
    ) -> Dict[str, Path]:
        """Save a scan under ``output_dir`` in the requested formats.

        Args:
            scan: The scan to write.
            name: Base filename without extension. Defaults to a UTC
                timestamp, ``scan_YYYYmmdd_HHMMSS``.
            formats: Any of "npz" (full grid, best for reprocessing), "ply"
                (point cloud for CloudCompare/MeshLab), "csv" (x,y,z text).

        Returns:
            Dict mapping format name to the written path.
        """
        import datetime as _dt

        if name is None:
            name = "scan_" + _dt.datetime.now(_dt.timezone.utc).strftime(
                "%Y%m%d_%H%M%S"
            )
        self._output_dir.mkdir(parents=True, exist_ok=True)
        base = self._output_dir / name

        written: Dict[str, Path] = {}
        for fmt in formats:
            if fmt == "npz":
                written["npz"] = scan.save_npz(base.with_suffix(".npz"))
            elif fmt == "ply":
                written["ply"] = scan.save_ply(base.with_suffix(".ply"))
            elif fmt == "csv":
                written["csv"] = scan.save_csv(base.with_suffix(".csv"))
            else:
                raise ValueError(f"Unknown scan format: {fmt!r}")
        logger.info("Saved scan: %s", {k: str(v) for k, v in written.items()})
        return written

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "GocatorScanner":
        """Build from a config dict — mirrors GantryController.from_config()."""
        return cls(config)
