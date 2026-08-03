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
from ..robot.motion_arbiter import DEFAULT_ARBITER
from .mounting import SensorMounting
from .pointcloud import (
    SurfaceScan,
    surface_point_cloud_to_scan,
    uniform_surface_to_scan,
)
# The configuration half of this subsystem lives in settings.py; it is mixed
# in below so the public API is unchanged. FILTER_NAMES and
# UniformSpacingRequiredError are re-exported here because callers and tests
# have always imported them from this module.
from .settings import (  # noqa: F401  (re-exported for backwards compatibility)
    FILTER_NAMES,
    GocatorSettingsMixin,
    UniformSpacingRequiredError,
    _name,
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


class ScanNotPossibleError(RuntimeError):
    """A scheduled scan could not run at all.

    Raised rather than skipped. An experiment that quietly stops collecting
    topography produces an incomplete record of conditions nobody can
    reconstruct afterwards — missing data is as bad as a stationary gantry.
    ``experiment.runner`` escalates this to a lab-wide pause, so the cause
    can be fixed and the run resumed rather than continuing with a hole in
    the data.
    """


class GocatorScanner(GocatorSettingsMixin):
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
        #: Scan spec for the zero-arg acquire() entry point — see that method.
        self._scan_spec = config.get("scan") or None
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
        self._last_saved_path: Optional[str] = None

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


    def _require_connected(self) -> GoSdkLib:
        if not self._is_connected or self._lib is None:
            raise RuntimeError(
                "GocatorScanner is not connected — call connect() first"
            )
        return self._lib


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

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def _abort_acquisition(self) -> Optional[str]:
        """Stop acquiring and discard anything part-captured.

        A surface captured across a decelerating pass is quietly wrong rather
        than obviously broken: Y spacing is travel_speed / frame_rate, which
        assumes constant velocity, so the travel axis comes out distorted.

        Returns a note naming the discard when one happened, because the
        caller writes it to the experiment event log. Silently missing scan
        data can invalidate a whole experiment — an analyst has to be able to
        see that a scan was attempted and thrown away, not just find a gap.

        Never raises, and deliberately does NOT go through
        _require_connected(): a safety verb that needs the hardware to be
        reachable is no use in the situation it exists for.
        """
        if not self._is_running:
            return None
        note = (
            "DISCARDED a part-captured surface — a pass interrupted mid-travel "
            "is distorted along Y (Y spacing assumes constant velocity), so it "
            "is not usable data. No scan file was written for this attempt."
        )
        logger.warning("Gocator: %s", note)
        try:
            if self._lib is not None and self._system is not None:
                self._lib.call("GoSystem_Stop", self._system)
        except Exception as exc:
            logger.error("Could not stop Gocator acquisition: %s", exc)
            note += f" (acquisition may still be running: {exc})"
        finally:
            self._is_running = False
        return note

    def pause(self) -> Optional[str]:
        """Abort any in-flight scan and discard the partial surface."""
        return self._abort_acquisition()

    def resume(self) -> Optional[str]:
        """Nothing to restore — the discarded scan is not resumable.

        Acquisition restarts on the next scan(), which re-triggers from a
        known start rather than splicing onto an aborted pass.
        """
        return None

    def stop(self) -> Optional[str]:
        """End cleanly: stop acquiring, discarding anything part-captured."""
        return self._abort_acquisition()

    def estop(self) -> Optional[str]:
        """Same as stop: there is no harder halt available to a passive sensor."""
        return self._abort_acquisition()

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
        axis: Optional[str] = None,
        end_mm: Optional[float] = None,
        feed_rate_mm_s: Optional[float] = None,
        settle_s: Optional[float] = None,
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
        # Fall back to the configured scan spec for anything not given, so
        # a rig configured once in file can scan without repeating itself —
        # and so acquire() is a thin wrapper rather than a second code path.
        spec = self._scan_spec or {}
        axis = axis if axis is not None else spec.get("axis")
        end_mm = end_mm if end_mm is not None else spec.get("end_mm")
        feed_rate_mm_s = (
            feed_rate_mm_s if feed_rate_mm_s is not None else spec.get("feed_rate_mm_s")
        )
        settle_s = settle_s if settle_s is not None else float(spec.get("settle_s", 0.5))
        missing = [
            n for n, v in (("axis", axis), ("end_mm", end_mm),
                           ("feed_rate_mm_s", feed_rate_mm_s)) if v is None
        ]
        if missing:
            raise ScanNotPossibleError(
                f"scan_with_gantry() is missing {missing} and the gocator.scan: "
                "config block does not supply it"
            )
        end_mm = float(end_mm)
        feed_rate_mm_s = float(feed_rate_mm_s)

        # Hold the gantry for the whole pass. Without this, a second
        # scheduled action could move an axis mid-traverse and the surface
        # would be silently wrong — Y spacing assumes constant velocity.
        # Re-entrant, so handle.begin_move_to() re-acquiring is fine.
        arbiter = getattr(gantry, "arbiter", DEFAULT_ARBITER)
        with arbiter.hold(f"gocator scan {axis} -> {end_mm:.1f}mm"):
            return self._scan_with_gantry(
                gantry, axis, end_mm, feed_rate_mm_s, settle_s,
                fixed_length_mm, metadata, timeout_s,
            )

    def _scan_with_gantry(
        self,
        gantry,
        axis: str,
        end_mm: float,
        feed_rate_mm_s: float,
        settle_s: float,
        fixed_length_mm: Optional[float],
        metadata: Optional[Dict[str, Any]],
        timeout_s: Optional[float],
    ) -> SurfaceScan:
        """Body of scan_with_gantry(), with the gantry already held."""
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

    def acquire(self, gantry=None, **overrides: Any) -> Optional[SurfaceScan]:
        """Run one configured scan — a zero-argument entry point for schedulers.

        ``scan_with_gantry()`` needs four arguments including a gantry handle,
        so it cannot be handed to ``Scheduler.repeat(action=...)`` or to
        ``experiment.runner._register_action``. This closes over a scan spec
        from config instead, so a Gocator pass can be scheduled exactly like a
        camera capture.

        The spec comes from the ``gocator.scan:`` config block — ``axis``,
        ``end_mm``, ``feed_rate_mm_s``, and optionally ``settle_s``,
        ``return_to_start`` and ``formats`` — with any of them overridable
        per call.

        Args:
            gantry: A connected GantryController. Required for a coordinated
                pass; without one this raises rather than silently triggering
                on a stationary gantry, which would produce a surface with no
                travel at all.
            **overrides: Per-call overrides of the configured scan spec.

        Returns:
            The captured SurfaceScan.

        Raises:
            ScanNotPossibleError: If the scan cannot run at all — no spec, an
                incomplete spec, no gantry, or a disconnected scanner. The
                runner escalates this to a lab-wide pause rather than
                skipping the scan, because an experiment that quietly stops
                collecting topography leaves a hole nobody can reconstruct.
        """
        spec = dict(self._scan_spec or {})
        spec.update(overrides)

        # Every one of these is a reason to bring the run down rather than
        # skip a scan. A scheduled experiment that quietly stops collecting
        # topography is producing an incomplete record of conditions nobody
        # will be able to reconstruct — missing data is as bad as a
        # stationary gantry, and both are worse than a paused run.
        if not spec:
            raise ScanNotPossibleError(
                "no 'scan:' config block and no overrides — the scanner cannot "
                "run a scheduled pass. Add gocator.scan.{axis, end_mm, "
                "feed_rate_mm_s}."
            )
        missing = [k for k in ("axis", "end_mm", "feed_rate_mm_s") if spec.get(k) is None]
        if missing:
            raise ScanNotPossibleError(
                f"scan spec is missing {missing}; needs axis, end_mm and "
                "feed_rate_mm_s to run a coordinated pass"
            )
        if gantry is None:
            raise ScanNotPossibleError(
                "no connected gantry — a scan with no motion produces a "
                "surface with no travel. Register a 'gantry:' section so the "
                "runner can supply one."
            )
        if not self._is_connected:
            raise ScanNotPossibleError(
                f"scanner at {self._ip} is not connected, so this pass would "
                "collect nothing"
            )

        return_to_start = spec.pop("return_to_start", False)
        formats = spec.pop("formats", None)
        start = gantry.axis(spec["axis"]).get_position() if return_to_start else None

        scan = self.scan_with_gantry(
            gantry,
            axis=spec["axis"],
            end_mm=float(spec["end_mm"]),
            feed_rate_mm_s=float(spec["feed_rate_mm_s"]),
            settle_s=float(spec.get("settle_s", 0.5)),
        )
        if formats:
            self.save_scan(scan, formats=tuple(formats))
        if return_to_start and start is not None:
            # Repeat scans of the same transect need the axis back where it
            # began, or each pass starts further along than the last.
            gantry.move_to(**{spec["axis"]: start})
        return scan

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
        # Remembered so a scheduled run can put the output path in the event
        # log — otherwise the only cross-subsystem index has no idea a scan
        # happened. See experiment.runner and COSCRIPTING_ROADMAP workstream 2.
        self._last_saved_path = str(next(iter(written.values()))) if written else None
        logger.info("Saved scan: %s", {k: str(v) for k, v in written.items()})
        return written

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "GocatorScanner":
        """Build from a config dict — mirrors GantryController.from_config()."""
        return cls(config)
