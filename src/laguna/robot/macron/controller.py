"""FlumeLab-facing facade for the macron gantry driver.

Wires together a transport (RS232Connection/EthernetConnection/
PiGantryConnection), MMCCommands, HomingProcedure, FenceRegistry +
TrajectoryChecker, and GCodeExecutor into the single object FlumeLab.add()
expects: something with a subsystem_name attribute and connect()/
disconnect()/get_status()/stop() methods (see laguna.core.FlumeLab.add and
laguna.weir.SaflWeirController for the established pattern this mirrors).
"""

from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    from ...config import Config

from .commands import (
    Axis,
    AxisHandle,
    IOMap,
    MMCCommands,
    THETA_AXIS,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    poll_until_move_finished,
    predicted_move_s,
    resolve_timeout_s,
)
from .connection import EthernetConnection, RS232Connection, SnapConnection, SnapMotionError
from .fences import BoxFence, CylinderFence, Fence, FenceRegistry, TrajectoryChecker
from .gcode import GCodeExecutor
from .halt import HaltLatch, HaltLevel, MotionGuard, MotionHalted
from .homing import AxisHomingConfig, HomingConfig, HomingProcedure
from ..motion_arbiter import DEFAULT_ARBITER
from .move_handle import MoveHandle
from .pi_bridge import PiGantryConnection
from .position_store import GantryPositionStore

logger = logging.getLogger(__name__)

# Tolerance for deciding whether an axis actually needs to move — used only
# by the debug-patch Theta skip in move_to() (see its comment there). Needed
# because mm/degree values round-trip through raw controller units and back.
_POSITION_EPSILON_MM = 1e-3

_NAMED_AXES = {
    "X": X_AXIS,
    "Y": Y_AXIS,
    "Z": Z_AXIS,
    "Theta": THETA_AXIS,
}


def _resolve_axis(name: str, index: int) -> Axis:
    """Return the known named Axis singleton if name/index match one, else a fresh Axis.

    Axis is a frozen dataclass with structural equality, so a freshly built
    instance still compares equal to the module-level singletons (commands.py's
    brake helpers rely on this — they compare by == , not identity).
    """
    known = _NAMED_AXES.get(name)
    if known is not None and known.index == index:
        return known
    return Axis(name, index)


def _axis_index_by_name(axes_cfg: List[Dict[str, Any]], name: str) -> Optional[int]:
    for entry in axes_cfg:
        if entry.get("name") == name:
            return entry["index"]
    return None


def _lookup_axis(axes_cfg: List[Dict[str, Any]], name: str) -> Optional[Axis]:
    """Resolve an axis by name.

    Prefer an explicit axes_cfg entry, falling back to the known
    named-axis singletons (X/Y/Z/Theta) so that homing.order / gcode axis
    selection still works even when the config omits the axes: list entirely
    (using the all-default axis set).
    """
    index = _axis_index_by_name(axes_cfg, name)
    if index is not None:
        return _resolve_axis(name, index)
    return _NAMED_AXES.get(name)


class GantryController:
    """FlumeLab subsystem facade for the macron gantry.

    Build via GantryController.from_config(config) (config is the lab's
    whole Config, not just its 'gantry:' section — see
    config/example_config.yaml, or lab.add("gantry") which does this for
    you) rather than constructing directly, unless custom fences/axes are
    needed programmatically.
    """

    subsystem_name = "gantry"

    x: AxisHandle
    y: AxisHandle
    z: AxisHandle
    theta: AxisHandle

    def __init__(
        self,
        connection: SnapConnection,
        axes: Tuple[Axis, ...] = (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS),
        group_index: int = 1,
        io_map: Optional[IOMap] = None,
        homing_config: Optional[HomingConfig] = None,
        fences: Optional[List[Fence]] = None,
        gcode_axes: Tuple[Axis, Axis] = (X_AXIS, Y_AXIS),
        gcode_z_axis: Axis = Z_AXIS,
        gcode_theta_axis: Axis = THETA_AXIS,
        theta_group_index: int = 2,
        safe_mode: bool = True,
        mm_per_unit: float = 1.0,
        axis_mm_per_unit: Optional[Dict[str, float]] = None,
        coordinate_offset_mm: Optional[Dict[str, float]] = None,
        position_checkpoint_file: Optional[str] = None,
        arbiter: Optional[Any] = None,
        soft_limits: Optional[Dict[str, Tuple[Optional[float], Optional[float]]]] = None,
    ):
        """Initialize the gantry controller.

        Args:
            connection: SnapConnection transport to the controller.
            axes: Tuple of Axis objects to configure (default: X/Y/Z/Theta).
            group_index: Coordinated group index for XY motion.
            io_map: IOMap for brake/switch pin configuration.
            homing_config: HomingConfig for homing parameters.
            fences: List of exclusion zone fences.
            gcode_axes: XY axes for coordinated group (default X/Y).
            gcode_z_axis: Z axis for gcode (default Z_AXIS).
            gcode_theta_axis: Theta (rotary) axis (default THETA_AXIS).
            theta_group_index: Coordinated group index for Z/Theta.
            safe_mode: Whether no-motion restrictions are active.
            mm_per_unit: Default real mm per raw controller unit, for any
                linear axis not overridden in axis_mm_per_unit.
            axis_mm_per_unit: Per-axis mm_per_unit overrides — see
                MMCCommands.__init__.
            coordinate_offset_mm: Per-axis position offsets in mm.
            position_checkpoint_file: Path to position persistence file.
            arbiter: Motion arbiter for serializing operations.
            soft_limits: Per-axis (negative_limit_mm, positive_limit_mm)
                overrides, either value optional. Written to the
                controller's NLT/PLT registers by connect() — see
                _apply_soft_limits().
        """
        self._connection = connection
        #: Shared gantry lock — see laguna.robot.motion_arbiter.
        self.arbiter = arbiter or DEFAULT_ARBITER
        self._axes = axes
        self._group_index = group_index
        self._io_map = io_map or IOMap()
        self._safe_mode = safe_mode
        self._is_connected = False
        self._soft_limits = soft_limits or {}
        # Latched by pause()/stop()/estop(); refuses every motion path and
        # cancels in-flight moves until resume()/rearm(). See halt.py.
        self._halt = HaltLatch()
        # See position_store.py / restore_last_position() — off (None) unless
        # a path is configured. Useful any time a power cycle wipes the
        # PLC's ACP registers and a fresh home() isn't wanted right away.
        self._position_store = (
            GantryPositionStore(position_checkpoint_file)
            if position_checkpoint_file
            else None
        )

        self.cmd = MMCCommands(
            connection,
            group_index=group_index,
            mm_per_unit=mm_per_unit,
            axis_mm_per_unit=axis_mm_per_unit,
            coordinate_offset_mm=coordinate_offset_mm,
            group_axes=gcode_axes,
        )
        # A second coordinated group, entirely on the responder node
        # (Z, Theta) — used only when a G-code move changes both together
        # (see GCodeExecutor._execute_zt_leg/_execute_concurrent_pair).
        # Sharing `connection` is safe: the transport already serialises
        # individual request/response pairs, and MotionArbiter.hold()
        # (held by GantryController.move_to()) serialises whole operations
        # across both this and self.cmd.
        self.theta_cmd = MMCCommands(
            connection,
            group_index=theta_group_index,
            mm_per_unit=mm_per_unit,
            axis_mm_per_unit=axis_mm_per_unit,
            coordinate_offset_mm=coordinate_offset_mm,
            group_axes=(gcode_z_axis, gcode_theta_axis),
        )

        # Per-axis convenience handles — lab.gantry.axis("Y") always works;
        # lab.gantry.y (etc.) is set dynamically below for whatever axes are
        # actually configured. See AxisHandle in commands.py — reads,
        # settings, brakes and stops only; per-axis motion is
        # move_to_unfenced()/jog_unfenced() below.
        self._axis_handles: Dict[str, AxisHandle] = {}
        for axis in self._axes:
            handle = AxisHandle(self.cmd, axis, io_map=self._io_map)
            self._axis_handles[axis.name] = handle
            attr_name = axis.name.lower()
            if hasattr(self, attr_name):
                raise ValueError(
                    f"Axis name {axis.name!r} collides with an existing "
                    f"GantryController attribute ({attr_name!r}) — rename the "
                    f"axis in config to expose it as lab.gantry.{attr_name}"
                )
            setattr(self, attr_name, handle)

        self.fence_registry = FenceRegistry()
        for fence in fences or []:
            self.fence_registry.add(fence)
        self.checker = TrajectoryChecker(self.fence_registry)

        self.homing = HomingProcedure(self.cmd, homing_config or HomingConfig(), io_map=self._io_map)

        self.gcode = GCodeExecutor(
            self.cmd,
            self.checker,
            homing=self.homing,
            axes=gcode_axes,
            z_axis=gcode_z_axis,
            theta_axis=gcode_theta_axis,
            group_index=group_index,
            theta_cmd=self.theta_cmd,
            theta_group_index=theta_group_index,
        )

    @property
    def connection(self) -> SnapConnection:
        """The underlying transport (PiGantryConnection, RS232Connection, etc.).

        Exposed publicly so callers needing transport-specific capabilities
        not part of the generic SnapConnection interface (e.g.
        PiGantryConnection.start_scan/stop_scan/wait_for_scan_result, used
        by TopographicProfiler) can reach them directly.
        """
        return self._connection

    def __getattr__(self, name: str) -> Any:
        """Return a dynamically exposed axis handle for configured axes."""
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self._axis_handles[name]
        except KeyError as exc:
            raise AttributeError(f"{type(self).__name__!r} object has no attribute {name!r}") from exc

    def axis(self, name: str) -> AxisHandle:
        """Return the AxisHandle for a configured axis by name.

        Case-sensitive, matches the 'name' field in config's gantry.axes
        list — e.g. "X", "Y", "Z", "Theta". Equivalent to the dynamic
        lab.gantry.<name.lower()> attribute, but useful when the axis name
        is only known at runtime.
        """
        try:
            return self._axis_handles[name]
        except KeyError:
            raise ValueError(
                f"No axis named {name!r} configured on this gantry "
                f"(configured: {list(self._axis_handles)})"
            ) from None

    def _resolve_axis_handle(self, axis: "Axis | AxisHandle | str") -> AxisHandle:
        if isinstance(axis, AxisHandle):
            return axis
        if isinstance(axis, Axis):
            return self.axis(axis.name)
        if isinstance(axis, str):
            return self.axis(axis)
        raise TypeError(
            f"axis must be an AxisHandle, Axis, or axis name string, got {type(axis).__name__}"
        )

    def engage_brake(self, axis: "Axis | AxisHandle | str") -> None:
        """Engage the electromagnetic brake on the given axis.

        Y or Z only — raises ValueError for axes without a brake. Accepts
        an axis name ("Y"), an Axis object, or an AxisHandle (e.g.
        lab.gantry.y) — same effect as lab.gantry.y.engage_brake(), just
        callable with the axis as an argument instead. See AxisHandle.engage_brake
        in commands.py.
        """
        self._resolve_axis_handle(axis).engage_brake()

    def disengage_brake(self, axis: "Axis | AxisHandle | str") -> None:
        """Disengage the electromagnetic brake on the given axis.

        Y or Z only — raises ValueError for axes without a brake. See
        engage_brake() above for accepted `axis` forms.
        """
        self._resolve_axis_handle(axis).disengage_brake()

    def brake_is_disengaged(self, axis: "Axis | AxisHandle | str") -> bool:
        """True if the given axis's brake is currently disengaged (released).

        See engage_brake() above for accepted `axis` forms.
        """
        return self._resolve_axis_handle(axis).brake_is_disengaged()

    def read_home_switch(self, axis: "Axis | AxisHandle | str") -> bool:
        """Read the given axis's home switch state.

        X/Y/Z only — raises ValueError for Theta (no home switch) or if
        the underlying IOMap channel hasn't been configured yet. See
        engage_brake() above for accepted `axis` forms.
        """
        return self._resolve_axis_handle(axis).read_home_switch()

    def read_limit_switch(self, axis: "Axis | AxisHandle | str") -> bool:
        """Read the given axis's limit switch state.

        Raises ValueError if the underlying IOMap channel hasn't been
        configured yet, or NotImplementedError for Theta — its limit
        switch is architecturally unreachable via ASCII on this hardware
        (see IOMap in commands.py). See engage_brake() above for accepted
        `axis` forms.
        """
        return self._resolve_axis_handle(axis).read_limit_switch()

    @classmethod
    def from_config(cls, config: "Config") -> "GantryController":
        """Build a GantryController from the lab's Config (its 'gantry:' section)."""
        config = config.get("gantry")
        connection = _build_transport(config)

        axes_cfg: List[Dict[str, Any]] = config.get("axes") or []
        axes = (
            tuple(_resolve_axis(a["name"], a["index"]) for a in axes_cfg)
            if axes_cfg
            else (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)
        )

        io_map = _build_io_map(axes_cfg)
        homing_config = _build_homing_config(config.get("homing") or {}, axes_cfg, io_map)
        fences = _build_fences(config.get("fences") or [])
        gcode_axes = _resolve_gcode_axes(axes_cfg)
        gcode_z_axis = _lookup_axis(axes_cfg, "Z") or Z_AXIS
        gcode_theta_axis = _lookup_axis(axes_cfg, "Theta") or THETA_AXIS
        axis_mm_per_unit = {
            a["name"]: a["mm_per_unit"] for a in axes_cfg if "mm_per_unit" in a
        }
        soft_limits = _build_soft_limits(axes_cfg)

        return cls(
            connection=connection,
            axes=axes,
            group_index=config.get("group_index", 1),
            io_map=io_map,
            homing_config=homing_config,
            fences=fences,
            gcode_axes=gcode_axes,
            gcode_z_axis=gcode_z_axis,
            gcode_theta_axis=gcode_theta_axis,
            theta_group_index=config.get("theta_group_index", 2),
            safe_mode=config.get("safe_mode", True),
            # Temporary DSM-project workaround — see docs/archive/GANTRY_UNIT_CALIBRATION.md.
            # Flip gantry.mm_per_acp_unit to 1.0 in config once fixed at the source;
            # nothing else needs to change.
            mm_per_unit=config.get("mm_per_acp_unit", 1.0),
            axis_mm_per_unit=axis_mm_per_unit,
            coordinate_offset_mm=config.get("coordinate_offset"),
            position_checkpoint_file=config.get("position_checkpoint_file"),
            soft_limits=soft_limits,
        )

    # ------------------------------------------------------------------
    # FlumeLab subsystem interface (see FlumeLab.add/connect_all/
    # disconnect_all/get_system_status/emergency_stop in laguna.core)
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the connection to the Snap2Motion controller.

        A no-op if already connected. Calling ``self._connection.connect()``
        a second time (e.g. PiGantryConnection) overwrites its live
        SSH channel/agent-process handles with a fresh session *before*
        tearing down the old ones — leaking the old SSH session, its reader
        thread, and the remote gantry_agent.py process, which holds the
        serial port exclusively, so the new agent can't get ready either.
        Both ends up unrecoverable without killing the whole process. See
        PiGantryConnection.connect()'s docstring for the leak mechanics.

        Returns:
            True if successfully connected (or already was).
        """
        if self._is_connected and self._connection.is_connected:
            logger.info("connect() called while already connected — no-op")
            return True
        # A fresh connection is a new run: a clean stop() from the last one
        # no longer applies. An estop does — that still needs rearm().
        self._halt.clear(up_to=HaltLevel.STOP)
        try:
            self._connection.connect()
            self._is_connected = self._connection.is_connected
            # The controller may have been power-cycled/reflashed while we
            # were away, clearing its coordinated-group state — make the
            # next move re-send INI rather than assume it survived. See
            # GCodeExecutor._init_group.
            self.gcode.reset_group_init()
            if self._is_connected:
                # gcode's _current_pos/_current_theta default to (0,0,0)/0.0
                # at construction and are never otherwise touched until a
                # move updates them — reconnecting while the gantry is
                # parked elsewhere would leave that cache believing it's at
                # a phantom origin, silently no-oping the next move instead
                # of actually moving it. See sync_position_from_hardware's
                # docstring.
                self.gcode.sync_position_from_hardware()
        except Exception as exc:
            logger.error("Failed to connect gantry: %s", exc)
            self._is_connected = False
            return False

        if self._is_connected and not self._safe_mode:
            self._enable_and_release_brakes()
            self._apply_soft_limits()
        return self._is_connected

    def _apply_soft_limits(self) -> None:
        """Write configured software travel limits (NLT/PLT) to the controller.

        NLT/PLT writes are on the safe_mode allowlist as bare reads only
        (see SAFE_COMMANDS in pi_bridge.py) — actually setting a limit is
        blocked exactly like any other motion-adjacent write while
        safe_mode is True. Called by connect() and set_safe_mode(False),
        only once that check has passed — so limits are in place before
        any motion is possible, whichever way motion got enabled.

        After writing, reads them back via validate_soft_limits() to
        confirm the values actually took, catching a wiring/unit mistake
        in config before anything can move. Never raises: like
        _enable_and_release_brakes(), a failure here is logged, not
        propagated — connect() itself must still succeed.
        """
        if not self._soft_limits:
            return
        applied: List[Axis] = []
        for axis in self._axes:
            bounds = self._soft_limits.get(axis.name)
            if bounds is None:
                continue
            neg, pos = bounds
            try:
                if neg is not None:
                    self.cmd.set_negative_limit(axis, neg)
                if pos is not None:
                    self.cmd.set_positive_limit(axis, pos)
            except Exception as exc:
                logger.warning("Could not set %s's soft limits: %s", axis.name, exc)
                continue
            applied.append(axis)
            logger.info("%s: soft limits set to (%s, %s) mm", axis.name, neg, pos)
        if not applied:
            return
        try:
            self.cmd.validate_soft_limits(axes=tuple(applied))
        except Exception as exc:
            logger.warning("Soft limits did not validate after being set: %s", exc)

    def _enable_and_release_brakes(self) -> None:
        """Turn each brake-equipped axis's motor on, then release its brake.

        Once the motor is on, its own torque holds the axis, so a
        still-engaged brake serves no purpose — and worse, it is a hazard:
        the next motion command would stall directly against it, which is
        exactly what corrupted Y's encoder feedback on 2026-07-30 and
        needed a power-cycle to clear.

        Motor-on then brake-release, strictly in that order and never the
        reverse: Z's brake is a fail-safe, spring-engaged design (SOB ON =
        released), so releasing it before the motor is holding torque would
        let a loaded Z drop under gravity (see IOMap's docstring in
        commands.py).

        Sends MTR and SOB only. The source commit for this feature also
        sent ENA here, which would crash the responder node every time a
        connection was made with motion enabled — see _ENA_BANNED in
        commands.py.

        Called by connect() (only when safe_mode is already False) and by
        set_safe_mode(False). Callers own the safe_mode check: this method
        always sends output-setting commands, which safe_mode's "no motion,
        no output-setting commands" guarantee has to cover.
        """
        for axis in self._axes:
            if axis not in (Y_AXIS, Z_AXIS):
                continue
            handle = self._axis_handles[axis.name]
            try:
                handle.enable()
            except SnapMotionError as exc:
                logger.warning("Could not turn on %s's motor: %s", axis.name, exc)
                continue
            try:
                handle.disengage_brake()
            except ValueError as exc:
                logger.info("Not releasing %s's brake: %s", axis.name, exc)
            except SnapMotionError as exc:
                logger.warning("Could not disengage %s's brake: %s", axis.name, exc)
            else:
                logger.info("%s: motor on, brake disengaged", axis.name)

    def _engage_brakes(self) -> None:
        """Re-engage each brake-equipped axis's brake.

        Mirrors _enable_and_release_brakes() for the transition back to
        safe_mode=True — see set_safe_mode(). Only engages the brake;
        deliberately leaves the motor on, since safe_mode's own gate (both
        client-side and, on the pi_agent transport, agent-side) is what
        actually blocks further motion from here.
        """
        for axis in self._axes:
            if axis not in (Y_AXIS, Z_AXIS):
                continue
            handle = self._axis_handles[axis.name]
            try:
                handle.engage_brake()
            except ValueError as exc:
                logger.info("Not engaging %s's brake: %s", axis.name, exc)
            except SnapMotionError as exc:
                logger.warning("Could not engage %s's brake: %s", axis.name, exc)
            else:
                logger.info("%s: brake engaged", axis.name)

    def disconnect(self) -> None:
        """Close the connection to the Snap2Motion controller."""
        self._connection.disconnect()
        self._is_connected = False

    def get_status(self) -> Dict[str, Any]:
        """Get the current status of the gantry controller.

        Returns:
            Dictionary with connection status and axis information.
        """
        status: Dict[str, Any] = {
            "subsystem": self.subsystem_name,
            "is_connected": self._is_connected,
            "safe_mode": self._safe_mode,
            "halted": self._halt.level.name.lower() if self._halt.level else None,
        }
        if not self._is_connected:
            return status
        positions: Dict[str, Any] = {}
        for axis in self._axes:
            try:
                positions[axis.name] = self.cmd.get_actual_position(axis)
            except SnapMotionError as exc:
                positions[axis.name] = f"error: {exc}"
        status["positions"] = positions
        return status

    def get_position(self) -> List[float]:
        """Get the current position vector.

        Returns:
            One value per configured axis, in ``self._axes`` order — the
            same order move_to()'s and set_position()'s vector form use.
        """
        return [self.cmd.get_actual_position(axis) for axis in self._axes]

    def stop(self) -> Optional[str]:
        """End cleanly: decelerate on each axis's ramp, then park the brakes.

        **This changed meaning.** It used to be the zero-decel abort with
        motors disabled; that is now estop(), where the unified vocabulary
        says it belongs (see laguna.safety). stop() is the tier below —
        controlled deceleration into a state safe to disconnect from, with
        no stall against a brake and no encoder disturbance.

        Latches the halt (see halt.py): further motion is refused until
        rearm(), or a fresh connect() for a new run.
        """
        self._halt.trip(HaltLevel.STOP, "stop()")
        note = self._stop_agent_scan("stop()")
        self.soft_stop()
        for axis in self._axes:
            if axis not in (Y_AXIS, Z_AXIS):
                continue
            try:
                self._axis_handles[axis.name].engage_brake()
            except Exception as exc:
                # Broad on purpose: a safety verb must never propagate. A
                # brake that would not park is worth logging, not worth
                # aborting the rest of the shutdown for.
                logger.warning("Could not park %s's brake: %s", axis.name, exc)
        self._persist_position()
        return note

    def _stop_agent_scan(self, verb: str) -> Optional[str]:
        """Cancel a topographic scan running on the Pi agent, if there is one.

        The agent runs the scan's BMT itself, so the controller's own stop
        commands alone don't tell it to stop reading samples. Returns a note
        for the event log: the profile acquired so far is kept, but it is
        partial.
        """
        is_running = getattr(self._connection, "is_scan_running", False)
        if not is_running:
            return None
        try:
            self._connection.stop_scan()
        except Exception as exc:
            logger.error("%s: could not cancel the running topographic scan: %s", verb, exc)
            return f"topographic scan may still be running — cancel failed: {exc}"
        return "topographic scan cut short — the partial profile is kept, but the pass must be re-run"

    def _persist_position(self) -> None:
        """Snapshot every axis's live position to the position checkpoint file.

        Best-effort snapshot to the position checkpoint file, if one is
        configured (position_checkpoint_file — see position_store.py). Never
        raises: a persistence failure must not break the caller's actual
        operation.

        Called at the end of move_to(), set_position(), stop(), and
        soft_stop() — deliberately including the two stop paths, not just
        successful moves, so a move cancelled or aborted mid-flight persists
        wherever the gantry actually ended up, not the destination it was
        headed for (or nothing at all).

        Note for stop()/soft_stop(): ABT/BST are not blocking — soft_stop()
        in particular decelerates over its own accel/decel ramp rather than
        halting instantly, so a read taken immediately after issuing it
        reflects position at the moment the stop was commanded, not the
        final rest position a moment later. Deliberately not waited out
        here: stop() is the emergency path and must return fast, and
        soft_stop() is documented as non-blocking everywhere else already.
        Good enough for this store's purpose — see position_store.py.
        """
        if self._position_store is None or not self._is_connected:
            return
        positions: Dict[str, float] = {}
        for axis in self._axes:
            try:
                positions[axis.name] = self.cmd.get_actual_position(axis)
            except SnapMotionError as exc:
                logger.debug("Not persisting %s's position: %s", axis.name, exc)
        if not positions:
            return
        try:
            self._position_store.save(positions)
        except OSError as exc:
            logger.warning("Could not write gantry position checkpoint: %s", exc)

    def restore_last_position(self) -> bool:
        """Re-reference every axis from the last position checkpoint.

        Applies whatever _persist_position() (called at the end of
        move_to(), set_position(), stop(), and soft_stop()) most recently
        wrote, via set_position() — the same non-motion register
        recalibration described in its docstring. Useful after a power
        cycle, which wipes the PLC's ACP registers entirely: this restores
        the last known position instantly without running home() again.

        Deliberately NOT called automatically by connect() — unlike a fresh
        physical home, a checkpoint file only proves "this was the position
        the last time this process wrote it," not "this is where the axis
        is now." If anything moved an axis by hand while the power was off,
        or the file is simply stale, applying it silently would be actively
        wrong — worse than leaving position unreferenced and visibly so.
        Call this explicitly, only once you've confirmed nothing moved, and
        check the logged checkpoint age first.

        Returns:
            True if a checkpoint was found and applied, False if no
            position_checkpoint_file is configured, none exists yet, or it
            has no axes in common with this gantry's configured axes.
        """
        if self._position_store is None:
            logger.warning(
                "restore_last_position() called but no position_checkpoint_file "
                "is configured — nothing to restore."
            )
            return False
        data = self._position_store.load()
        if data is None:
            logger.warning("No gantry position checkpoint found to restore.")
            return False
        axes_by_name = {axis.name: axis for axis in self._axes}
        restorable = {
            name: value for name, value in data["positions"].items() if name in axes_by_name
        }
        if not restorable:
            logger.warning(
                "Position checkpoint has no axes matching this gantry's "
                "configured axes (%s) — nothing restored.",
                [a.name for a in self._axes],
            )
            return False
        age_s = time.time() - data.get("wall_time", time.time())
        logger.info(
            "Restoring gantry position from a %.0fs-old checkpoint: %s",
            age_s, restorable,
        )
        return self.set_position(**restorable)

    # ------------------------------------------------------------------
    # Simple verbs (mirrors laguna.weir.SaflWeirController's shape) —
    # the richer per-axis API (self.cmd, self.gcode, self.homing) stays
    # directly reachable for anything these don't cover.
    # ------------------------------------------------------------------

    def _require_motion_allowed(self, description: str) -> None:
        """Refuse motion unless connected, out of safe_mode, and not halted.

        Checked client-side for every motion path, whatever the transport —
        RS232Connection/EthernetConnection have no gate of their own, so
        without this a fenced move_to() reached the wire under safe_mode.

        Raises:
            SnapMotionError: If not connected, or safe_mode is on.
            MotionHalted: If pause()/stop()/estop() has latched the halt.
        """
        if not self._is_connected:
            raise SnapMotionError(
                0, f"{description} refused: not connected — call connect() first"
            )
        if self._safe_mode:
            raise SnapMotionError(
                0, f"{description} blocked by safe_mode — no-motion restriction active"
            )
        self._halt.require_clear(description)

    def _run_motion(
        self, description: str, prepare: Callable[[MotionGuard], Callable[[], Any]]
    ) -> MoveHandle:
        """Run one motion operation: checks and planning now, the traverse in the background.

        `prepare` runs with the gantry held and must do everything that can
        refuse the move (validation, live position read, fence check),
        returning the callable that actually moves. Anything it raises comes
        straight back out of the calling method, with nothing sent.

        If the calling thread already holds the arbiter (library code
        sequencing several moves inside one operation, e.g. acquire_scan()),
        the whole thing runs inline instead — a background thread would wait
        forever on the hold its own caller is sitting in.
        """
        self._require_motion_allowed(description)

        def _prepare() -> Callable[[], Any]:
            self._require_motion_allowed(description)
            guard = self._halt.guard(description)
            execute = prepare(guard)

            def _execute() -> Any:
                logger.info("%s — started", description)
                try:
                    result = execute()
                except MotionHalted as exc:
                    logger.warning("%s — %s", description, exc)
                    raise
                except Exception:
                    logger.exception("%s — failed", description)
                    raise
                finally:
                    self._persist_position()
                logger.info("%s — completed", description)
                return result

            return _execute

        if self.arbiter.held_by_current_thread():
            return MoveHandle.run_inline(description, lambda: _prepare()())
        # timeout_s=0: a second motion call while one is running is refused
        # immediately ("motion in progress"), not queued behind it.
        return MoveHandle.run_in_background(
            description, _prepare, lambda: self.arbiter.hold(description, timeout_s=0)
        )

    def _resolve_targets(
        self,
        verb: str,
        vector: Optional[List[float]],
        X: Optional[float],
        Y: Optional[float],
        Z: Optional[float],
        Theta: Optional[float],
    ) -> Dict[str, float]:
        """Turn move_to()/set_position()'s vector-or-keywords forms into {axis name: value}."""
        axes_by_name = {axis.name: axis for axis in self._axes}
        if vector is not None:
            if len(vector) != len(self._axes):
                raise ValueError(
                    f"{verb}(vector=...) expects {len(self._axes)} values "
                    f"(one per configured axis: {[a.name for a in self._axes]}), "
                    f"got {len(vector)}"
                )
            target_by_name: Dict[str, float] = {
                axis.name: value for axis, value in zip(self._axes, vector)
            }
        else:
            target_by_name = {}
            for name, value in (("X", X), ("Y", Y), ("Z", Z), ("Theta", Theta)):
                if value is None:
                    continue
                if name not in axes_by_name:
                    raise ValueError(f"Axis {name!r} is not configured on this gantry")
                target_by_name[name] = value
            if not target_by_name:
                raise ValueError(
                    f"{verb}() requires either vector=... or at least one of X=/Y=/Z=/Theta="
                )
        for name, value in target_by_name.items():
            # A NaN formats as "nan", which the G-code word regex doesn't
            # match — the axis would silently be backfilled with its current
            # position and the move would quietly not happen.
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{verb}(): {name} target must be a finite number, got {value!r}")
        return {name: float(value) for name, value in target_by_name.items()}

    @staticmethod
    def _require_valid_speed(verb: str, speed: Optional[float]) -> None:
        if speed is not None and not (isinstance(speed, (int, float)) and math.isfinite(speed) and speed > 0):
            raise ValueError(f"{verb}(): speed must be a positive finite number, got {speed!r}")

    def move_to(
        self,
        vector: Optional[List[float]] = None,
        *,
        X: Optional[float] = None,
        Y: Optional[float] = None,
        Z: Optional[float] = None,
        Theta: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> MoveHandle:
        """Move to an absolute position, in real mm (and degrees/units for Theta). Non-blocking.

        Checks and fence-checks on the calling thread — so a
        FenceViolation, a safe_mode refusal, or "motion in progress" raises
        right here with nothing sent — then traverses on a background thread
        and returns a MoveHandle at once. The REPL stays free, so pause()
        can be called immediately. Call ``.wait()`` on the handle when the
        next step depends on the move having finished.

          - ``move_to([x, y, z, theta])`` — one value per configured axis,
            in the same order as ``self._axes`` (X, Y, Z, Theta by
            default).
          - ``move_to(X=100)`` / ``move_to(X=100, Z=5)`` — move only the
            given axes. Any configured Cartesian axis *not* given is
            backfilled with its real current position (a live
            get_actual_position() read) before the fence check runs, so
            the checked path reflects where the gantry actually is, not
            an assumed one.

        X/Y/Z run through the coordinated gcode path
        (GCodeExecutor.plan/execute). Theta is outside the Cartesian
        gcode/fence model (fences.py only checks X/Y/Z — see
        ericbarefoot/laguna#62) and is moved as a separate single-axis
        command. A pure Theta-only call skips the gcode path entirely.

        To move one axis without the fence check — deliberately, e.g. to
        find where a fence should go — use move_to_unfenced().

        Args:
            vector: Full-length position vector, or None to use keywords.
            X: Absolute X target in real mm.
            Y: Absolute Y target in real mm.
            Z: Absolute Z target in real mm.
            Theta: Absolute Theta target in whatever unit that axis's
                raw-to-real conversion yields.
            speed: Optional feed rate (mm/s) applied to the move(s).

        Returns:
            A MoveHandle for the move in progress.

        Raises:
            ValueError: If vector's length doesn't match the configured
                axes, an axis keyword names an axis not configured on this
                gantry, neither vector nor any keyword was given, or a
                target/speed isn't a finite number (speed must be > 0).
            FenceViolation: If the X/Y/Z path would enter an exclusion zone.
            SnapMotionError: If not connected, or safe_mode is on.
            MotionHalted: If the gantry is paused/stopped/estopped.
            MotionBusyError: If another motion operation is in progress.
        """
        target_by_name = self._resolve_targets("move_to", vector, X, Y, Z, Theta)
        self._require_valid_speed("move_to", speed)
        return self._run_motion(
            f"move_to({target_by_name})",
            lambda guard: self._plan_move_to(target_by_name, speed, guard),
        )

    def _plan_move_to(
        self, target_by_name: Dict[str, float], speed: Optional[float], guard: MotionGuard
    ) -> Callable[[], None]:
        """Fence-check move_to()'s path from the live position; return the traverse."""
        target_by_name = dict(target_by_name)
        theta_value = target_by_name.pop("Theta", None)

        trajectory = None
        cartesian_axes = [axis for axis in self._axes if axis.name != "Theta"]
        if any(axis.name in target_by_name for axis in cartesian_axes):
            # Plan from where the gantry actually is, not from what gcode last
            # believed. Anything that moved an axis without going through
            # gcode (a scan pass, move_to_unfenced(), homing) leaves that
            # cache stale, and a stale start position is used for three
            # things at once: deciding which legs run (a move to "where the
            # cache thinks we are" silently no-ops), the fence check's path,
            # and the leg distances.
            self.gcode.sync_position_from_hardware()
            live_by_name = dict(zip(("X", "Y", "Z"), self.gcode.current_position))
            gcode_words: List[str] = []
            for axis in cartesian_axes:
                value = target_by_name.get(axis.name)
                if value is None:
                    # Backfill from the *same* reading the cache was just synced
                    # to, never a second read: two reads of a settling axis can
                    # differ by more than _POSITION_EPSILON_MM, and any such
                    # difference on an axis nobody asked to move is a phantom
                    # near-zero leg (see GCodeExecutor._sync_position_from_hardware
                    # and _apply_scaled_ramp for what those do to ACL/DCL).
                    value = live_by_name[axis.name]
                gcode_words.append(f"{axis.name}{value:.6f}")
            if speed is not None:
                gcode_words.append(f"F{speed * 60:.6f}")  # gcode feed rate is mm/min
            trajectory = self.gcode.plan("G90\nG1 " + " ".join(gcode_words))

        def _execute() -> None:
            if trajectory is not None:
                self.gcode.execute(trajectory, guard=guard)
            if theta_value is not None:
                self._move_single_axis(self._axis_by_name("Theta"), theta_value, speed, guard)

        return _execute

    def _axis_by_name(self, name: str) -> Axis:
        for axis in self._axes:
            if axis.name == name:
                return axis
        raise ValueError(
            f"No axis named {name!r} configured on this gantry "
            f"(configured: {[a.name for a in self._axes]})"
        )

    def _move_single_axis(
        self, axis: Axis, position: float, speed: Optional[float], guard: MotionGuard
    ) -> None:
        """One axis to `position`, under `guard`, waiting for it to finish. No fence check."""
        current = self.cmd.get_actual_position(axis)
        # Skip a move to where the axis already is (within rounding of the
        # raw-unit round trip), same as GCodeExecutor._execute_linear does.
        if abs(position - current) <= _POSITION_EPSILON_MM:
            return
        if speed is not None:
            self.cmd.set_speed(axis, speed)
        with guard.issuing():
            self.cmd._begin_move_to(axis, position)
        self._wait_for_axis_move_finished(
            axis,
            predicted_s=predicted_move_s(position - current, speed),
            check_halt=guard.check,
        )

    def move_to_unfenced(
        self, axis: "Axis | AxisHandle | str", position: float, speed: Optional[float] = None
    ) -> MoveHandle:
        """Move ONE axis to an absolute position WITHOUT checking fences. Non-blocking.

        For locating where fences belong, or recovering an axis that a
        fence (or a stale one) won't let move_to() touch. Everything else
        still applies: safe_mode, the halt latch, the motion arbiter, and the
        controller's own soft limits (NLT/PLT). Logged at WARNING so the
        record shows motion ran unfenced.

        Args:
            axis: Axis name ("X"), Axis, or AxisHandle.
            position: Absolute target, real mm (Theta: its own unit).
            speed: Optional speed for this axis, mm/s.

        Returns:
            A MoveHandle; ``.wait()`` to block until it finishes.

        Raises:
            ValueError: Unknown axis, or a non-finite position/speed.
            SnapMotionError: If not connected, or safe_mode is on.
            MotionHalted: If the gantry is halted.
            MotionBusyError: If another motion operation is in progress.
        """
        target = self._resolve_axis_handle(axis)._axis
        if not math.isfinite(position):
            raise ValueError(f"move_to_unfenced(): position must be finite, got {position!r}")
        self._require_valid_speed("move_to_unfenced", speed)
        description = f"move_to_unfenced({target.name}={position})"
        logger.warning("%s — NO FENCE CHECK", description)

        def _prepare(guard: MotionGuard) -> Callable[[], None]:
            def _execute() -> None:
                try:
                    self._move_single_axis(target, position, speed, guard)
                finally:
                    self._sync_position_after_direct_motion(description)
            return _execute

        return self._run_motion(description, _prepare)

    def jog_unfenced(self, axis: "Axis | AxisHandle | str", speed: float) -> None:
        """Start ONE axis jogging at `speed` mm/s WITHOUT any fence check; 0 stops it.

        Open-ended motion: it runs until jog_unfenced(axis, 0), pause(), a
        limit, or the controller's soft limits stop it, so nothing can
        fence-check it in advance. safe_mode and the halt latch still apply
        to starting a jog (never to stopping one), and it refuses while
        another motion operation holds the gantry. Logged at WARNING.

        Raises:
            ValueError: Unknown axis, or a non-finite speed.
            SnapMotionError: If starting a jog while not connected or in safe_mode.
            MotionHalted: If starting a jog while the gantry is halted.
            MotionBusyError: If another motion operation is in progress.
        """
        target = self._resolve_axis_handle(axis)._axis
        if not math.isfinite(speed):
            raise ValueError(f"jog_unfenced(): speed must be finite, got {speed!r}")
        if speed == 0:
            self.cmd._jog(target, 0)
            self._sync_position_after_direct_motion("jog_unfenced(0)")
            return
        description = f"jog_unfenced({target.name}, {speed} mm/s)"
        self._require_motion_allowed(description)
        logger.warning("%s — NO FENCE CHECK; stop with jog_unfenced(%r, 0) or pause()",
                       description, target.name)
        with self.arbiter.hold(description, timeout_s=0):
            guard = self._halt.guard(description)
            with guard.issuing():
                self.cmd._jog(target, speed)

    def plan_scan_move(self, axis: "Axis | AxisHandle | str", end_mm: float) -> MotionGuard:
        """Fence-check a single-axis scan pass from the live position, without moving.

        For scan paths that start the traverse themselves (the Pi agent's
        scan_start) — the caller issues the start under the returned guard's
        ``issuing()`` and checks it while waiting. The caller must already
        hold the motion arbiter for the whole pass.

        Args:
            axis: Axis name ("X"), its BLC token ("A1"), Axis, or AxisHandle.
            end_mm: Absolute end position of the pass, real mm.

        Returns:
            The MotionGuard for this pass.

        Raises:
            RuntimeError: If the calling thread doesn't hold the arbiter.
            FenceViolation: If the straight pass would enter an exclusion zone.
            SnapMotionError: If not connected, or safe_mode is on.
            MotionHalted: If the gantry is halted.
        """
        target = self._resolve_scan_axis(axis)
        if not math.isfinite(end_mm):
            raise ValueError(f"scan end_mm must be finite, got {end_mm!r}")
        description = f"scan pass {target.name} -> {end_mm:.1f}mm"
        if not self.arbiter.held_by_current_thread():
            raise RuntimeError(f"{description}: hold gantry.arbiter for the whole pass first")
        self._require_motion_allowed(description)
        guard = self._halt.guard(description)
        if target.name in ("X", "Y", "Z"):
            self.gcode.sync_position_from_hardware()
            start = self.gcode.current_position
            end = list(start)
            end[("X", "Y", "Z").index(target.name)] = end_mm
            violations = self.checker.check_segment(start, tuple(end))
            if violations:
                raise violations[0]
        return guard

    def begin_scan_move(
        self, axis: "Axis | AxisHandle | str", end_mm: float, feed_rate_mm_s: float
    ) -> MotionGuard:
        """Fence-check, then start (non-blocking) a constant-speed single-axis scan pass.

        For scan paths that must trigger acquisition while the axis is
        mid-move (the Gocator). Same contract as plan_scan_move(): hold the
        arbiter for the whole pass, and poll with the returned guard's
        ``check()`` so a pause/stop/estop ends the wait.

        Raises:
            ValueError: If feed_rate_mm_s isn't a positive finite number.
            Everything plan_scan_move() raises.
        """
        self._require_valid_speed("begin_scan_move", feed_rate_mm_s)
        guard = self.plan_scan_move(axis, end_mm)
        target = self._resolve_scan_axis(axis)
        self.cmd.set_speed(target, feed_rate_mm_s)
        with guard.issuing():
            self.cmd._begin_move_to(target, end_mm)
        return guard

    def _resolve_scan_axis(self, axis: "Axis | AxisHandle | str") -> Axis:
        if isinstance(axis, str) and axis[:1] == "A" and axis[1:].isdigit():
            index = int(axis[1:])
            for configured in self._axes:
                if configured.index == index:
                    return configured
            raise ValueError(f"No configured axis has BLC token {axis!r}")
        return self._resolve_axis_handle(axis)._axis

    def set_position(
        self,
        vector: Optional[List[float]] = None,
        *,
        X: Optional[float] = None,
        Y: Optional[float] = None,
        Z: Optional[float] = None,
        Theta: Optional[float] = None,
    ) -> bool:
        """Redefine the controller's notion of current position for the given axes.

        Mirrors laguna.weir.SaflWeirController.set_elevation() — this
        recalibrates each given axis's position register (ACP) to the given
        real-mm value without commanding any motion. Use it to re-reference
        the gantry after it has been repositioned by other means (e.g.
        manually), as an alternative to running home() again. To actually
        move, use move_to().

        Both forms mirror move_to()'s shape — a full vector (one value per
        configured axis, in self._axes order) or per-axis keywords — except
        that unlike move_to(), any axis *not* given is left completely
        untouched: there is no backfill, since there is no fence-checked
        path to compute one for.

        Args:
            vector: Full-length position vector, or None to use keywords.
            X: New X position in real mm.
            Y: New Y position in real mm.
            Z: New Z position in real mm.
            Theta: New Theta position in whatever unit that axis's
                raw-to-real conversion yields.

        Returns:
            True if every given axis's position register was set.

        Raises:
            ValueError: If vector's length doesn't match the configured
                axes, an axis keyword names an axis not configured on this
                gantry, or neither vector nor any keyword was given.
        """
        axes_by_name = {axis.name: axis for axis in self._axes}
        target_by_name = self._resolve_targets("set_position", vector, X, Y, Z, Theta)
        for name, value in target_by_name.items():
            self.cmd.set_actual_position(axes_by_name[name], value)
        self._sync_position_after_direct_motion("set_position()")
        return True

    def soft_stop(self) -> None:
        """Decelerate every configured axis to a stop.

        Leaves brakes and motors alone. The gentle counterpart to stop():
        each axis decelerates using its own accel/decel ramp (BST) rather
        than a zero-decel abort, and nothing is disabled — so the gantry is
        immediately ready for another move_to() with no re-enable cycle. Use
        this to cancel an ordinary move; use stop() for an emergency.

        Never raises: one axis failing to stop is logged and does not
        prevent the rest from getting their stop command.
        """
        for axis in self._axes:
            try:
                self.cmd.begin_stop(axis)
            except Exception as exc:
                logger.error("Error soft-stopping %s: %s", axis.name, exc)
        # Best-effort: BST isn't blocking, so this reads position while
        # still decelerating, not at final rest (same caveat noted in
        # _persist_position()'s docstring), but that's still far closer to
        # reality than the stale pre-move state.
        self._sync_position_after_direct_motion("soft_stop()")

    def resync_position(self, source: str) -> None:
        """Re-read every axis into the planning cache and the position checkpoint.

        Call after moving an axis directly — through an AxisHandle, self.cmd, or
        anything else that bypasses move_to()'s gcode path — once that motion
        has finished. move_to() also resyncs on its own before planning, so
        this mainly keeps the on-disk checkpoint honest. Never raises.

        Args:
            source: Caller name, for the log message if the resync fails.
        """
        self._sync_position_after_direct_motion(source)

    def _sync_position_after_direct_motion(self, source: str) -> None:
        """Resync gcode's cached position and the on-disk checkpoint after direct motion.

        home()/home_axis()/locate_limit_switch()/soft_stop()/estop()/
        set_position() all move hardware (or redefine position registers)
        directly through HomingProcedure/MMCCommands rather than through
        GCodeExecutor.plan()/execute(), which normally keeps two things in
        sync on its own: gcode's in-memory `_current_pos`/`_current_theta`
        cache, and (via _persist_position(), also called here) the
        on-disk position checkpoint file used by restore_last_position()
        after a power cycle. Skipping either leaves it believing wherever
        it was before this call:
          - A stale gcode cache makes the next move_to() plan a leg
            against the wrong starting position. A wrong-enough leg
            distance from that mismatch can scale ACL/DCL down to 0 raw
            units (ASCII escape 16/17, "0 Or Negative") — see
            GCodeExecutor._sync_position_from_hardware's docstring for
            the full mechanism.
          - A stale checkpoint file means restore_last_position() would
            recalibrate to the wrong place after a power cycle.
        Never raises: called from a `finally`/cleanup path, so a resync
        failure is logged, not propagated. _persist_position() itself
        never raises either — see its own docstring.

        Args:
            source: Caller name, for the log message if the resync fails.
        """
        try:
            self.gcode.sync_position_from_hardware()
        except Exception as exc:
            logger.error("Error resyncing gcode position after %s: %s", source, exc)
        self._persist_position()

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def pause(self) -> Optional[str]:
        """Decelerate on each axis's own ramp, leaving brakes and motors alone.

        Latches a pause (see halt.py): the move in flight is cancelled — its
        next leg is never issued — and new motion is refused until resume().
        Brakes and motors are untouched, so resuming needs no re-enable
        cycle, which is what makes a pause cheap enough to use liberally.
        """
        self._halt.trip(HaltLevel.PAUSE, "pause()")
        note = self._stop_agent_scan("pause()")
        self.soft_stop()
        return note

    def resume(self) -> Optional[str]:
        """Undo pause(): allow motion again. Never undoes a stop() or estop().

        Motion is re-commanded by the caller, not resumed implicitly: the
        gantry has no notion of an interrupted move to pick back up.
        """
        if not self._halt.clear(up_to=HaltLevel.PAUSE):
            level = self._halt.level.name.lower()
            logger.warning("Gantry not resumed: it is %s — call rearm() instead", level)
            return f"gantry still {level}ped — not resumed; rearm() is required"
        return None

    def estop(self) -> Optional[str]:
        """Zero-decel abort, brakes engaged, motors disabled. Never raises.

        This is what stop() used to do. Latches an estop: all motion is
        refused until rearm(), which leaves safe_mode on — re-enabling
        motion is a separate, explicit set_safe_mode(False).

        Deliberately does not re-read positions afterwards: estop must
        finish fast so FlumeLab can move on to the pump and valves.
        FlumeLab.estop() resyncs (resync_position()) once everything is safe.
        """
        self._halt.trip(HaltLevel.ESTOP, "estop()")
        note = self._stop_agent_scan("estop()")
        try:
            self.cmd.shutdown(axes=self._axes, io_map=self._io_map)
        except Exception as exc:
            logger.error("Error during gantry emergency stop: %s", exc)
        return note

    def rearm(self) -> bool:
        """Clear a stop() or estop() latch, leaving the gantry in safe_mode.

        Re-arming never re-enables motion by itself: whoever re-arms has
        checked the cause is cleared, which is not the same as checking
        it's safe to move. Motion needs a separate set_safe_mode(False),
        which turns the motors back on and releases the brakes.

        Returns:
            True if the latch is clear and safe_mode is on.
        """
        self._halt.clear(up_to=HaltLevel.ESTOP)
        if self._safe_mode:
            return True
        return bool(self.set_safe_mode(True))

    @property
    def halted(self) -> Optional[str]:
        """The latched halt tier ("pause", "stop", "estop"), or None if motion is allowed."""
        return self._halt.level.name.lower() if self._halt.level else None

    def set_safe_mode(self, enabled: bool) -> bool:
        """Enable or disable safe_mode.

        Reconnects the transport if needed so the change actually takes
        effect, and syncs Y/Z's brakes to match.

        Setting ``self.connection.safe_mode`` directly is not enough for the
        pi_agent transport: gantry_agent.py enforces its own independent
        safe-mode gate (deliberate defense-in-depth — see pi_bridge.py's
        module docstring), fixed at process launch via the --allow-motion
        flag baked into the SSH command in PiGantryConnection.connect(). An
        already-running agent keeps enforcing whatever it was launched with,
        no matter what the client-side attribute says. This method updates
        the flag and, if a PiGantryConnection is currently connected,
        disconnects and reconnects so the agent relaunches to match.

        Brakes follow the same transition, for the same reason as
        connect()'s automatic release (see _enable_and_release_brakes()):
        turning safe_mode off means motion is now possible, so Y/Z release
        (motor on first, brake released second — never leave an axis with
        neither holding it); turning it back on re-engages them, since
        safe_mode's own gate is about to stop motor torque being
        re-commanded.

        Turning safe_mode *off* requires an explicit connect() first and
        raises otherwise, changing nothing. Without a live connection there is
        no agent to relaunch and no brakes to release, so "off" would only
        flip flags while whatever agent might exist keeps its old gate — the
        flag would say motion is allowed when it isn't (or, worse, lie about
        which state the hardware is in). Turning safe_mode *on* while
        disconnected is always allowed: it can only make things safer, and
        connect() applies the release side itself using whatever safe_mode is
        set to by then.

        Returns:
            True if the change took effect (including a successful
            reconnect, if one was needed); False if a required reconnect
            failed — check logs and call connect() again once resolved.

        Raises:
            SnapMotionError: If ``enabled`` is False and connect() has not
                been called (or the gantry was disconnected).
        """
        if not enabled and not self._is_connected:
            raise SnapMotionError(
                0,
                "Cannot disable safe_mode while disconnected — call connect() first "
                "(lab.connect_all() or gantry.connect()). Nothing was changed.",
            )
        if enabled and self._is_connected and not self._safe_mode:
            # Stop and brake *before* the gate closes. Once safe_mode is on,
            # the transport's allowlist (and the relaunched Pi agent's) would
            # refuse anything but reads and stop-class commands — this used
            # to flip the flag first, so the brake commands that followed
            # were refused and Y/Z were left released with motors on.
            try:
                self.soft_stop()
                self._engage_brakes()
            except Exception as exc:
                # Closing the gate matters more than a clean brake park —
                # never leave motion enabled because a brake write failed.
                logger.error("set_safe_mode(True): could not stop/brake before gating: %s", exc)
        self._safe_mode = enabled
        if hasattr(self._connection, "safe_mode"):
            self._connection.safe_mode = enabled
        if isinstance(self._connection, PiGantryConnection) and self._is_connected:
            self.disconnect()
            # connect() releases the brakes and applies soft limits itself
            # when enabled=False (it checks self._safe_mode).
            return bool(self.connect())
        if self._is_connected and not enabled:
            self._enable_and_release_brakes()
            self._apply_soft_limits()
        return True

    def _run_homing(self, description: str, body: Callable[[], Any]) -> MoveHandle:
        """Run a HomingProcedure operation as a non-blocking, halt-aware motion.

        Homing is never fence-checked — until it finishes there is no
        reference frame for fences to mean anything in. safe_mode, the halt
        latch and the arbiter still apply, and the planning position and
        checkpoint are resynced afterwards however it ends.
        """
        def _prepare(guard: MotionGuard) -> Callable[[], Any]:
            def _execute() -> Any:
                self.homing._guard = guard
                try:
                    return body()
                finally:
                    self.homing._guard = None
                    self._sync_position_after_direct_motion(description)
            return _execute

        return self._run_motion(description, _prepare)

    def home(self) -> MoveHandle:
        """Run the homing routine on all configured axes. Non-blocking.

        Returns:
            A MoveHandle; its ``result`` is True once every axis homed,
            False if an axis failed (see the log for which and why).
        """
        def _home_all() -> bool:
            result = self.homing.home_all()
            if not result.success:
                logger.error("home() — did not find home: %s", result.error)
            return result.success

        return self._run_homing("home()", _home_all)

    def home_axis(self, axis: "Axis | AxisHandle | str") -> MoveHandle:
        """Home a single axis. Non-blocking.

        Prefer this over calling self.homing.home_axis() directly — that
        bypasses safe_mode, the halt latch, the arbiter, and the position
        resync afterwards. See engage_brake() above for accepted `axis` forms.

        Returns:
            A MoveHandle; its ``result`` is the standoff position after homing.
        """
        target = self._resolve_axis_handle(axis)._axis
        return self._run_homing(
            f"home_axis({target.name})", lambda: self.homing.home_axis(target)
        )

    def locate_limit_switch(self, axis: "Axis | AxisHandle | str") -> MoveHandle:
        """Jog toward and record the given axis's limit switch position. Non-blocking.

        Unlike home(), this does not redefine the origin — it reports the
        limit switch's position in the current (already-homed) coordinate
        frame, then backs off to standoff_distance so the axis isn't left
        resting against the hard stop. See
        HomingProcedure.locate_limit_switch for the direction/polarity
        assumptions this reuses from the axis's homing config. See
        engage_brake() above for accepted `axis` forms.

        Returns:
            A MoveHandle; its ``result`` is the limit switch's position.
        """
        target = self._resolve_axis_handle(axis)._axis
        return self._run_homing(
            f"locate_limit_switch({target.name})",
            lambda: self.homing.locate_limit_switch(target),
        )

    def enable(self) -> None:
        """Turn motor drive on for all configured axes (MTR only).

        Does NOT send ENA. Addressing ENA on a responder-node axis (Z,
        Theta) crashes this controller — see MMCCommands._ENA_BANNED. The
        controller's own DSM program enables the axes at power-up, so the
        drive-enable half of this was never load-bearing here.
        """
        for axis in self._axes:
            self.cmd.set_motor(axis, True)

    def disable(self) -> None:
        """Turn motor drive off for all configured axes.

        Allows manual repositioning (MTR only). Does NOT send ENA — see
        enable().
        """
        for axis in self._axes:
            self.cmd.set_motor(axis, False)

    def wait_for_move(self, timeout: Optional[float] = None, predicted_s: float = 0.0) -> None:
        """Block until the coordinated group's current move completes, or timeout elapses.

        Only tracks the coordinated (X/Y) group — a Theta-only move issued
        via move_to(Theta=...) isn't covered by this; poll
        self.cmd.move_is_finished(THETA_AXIS) directly for that.

        polls
        sparsely via commands.poll_until_move_finished. This is the public
        "wait for the move I just started" entry point, so it is exactly
        the loop most likely to be querying C<n> MIF while a group move is
        interpolating — the pattern confirmed on hardware to make this
        controller stop answering the wire; see commands.py's module note.
        Pass `predicted_s` (distance / speed) when known so most of the
        wait costs no wire traffic at all. `timeout` defaults to
        poll_until_move_finished's own predicted-duration-scaled timeout
        when omitted — see that function's docstring.

        Raises:
            TimeoutError: If the move hasn't finished within `timeout`.
        """
        if not poll_until_move_finished(
            self.cmd.group_move_is_finished, predicted_s=predicted_s, timeout_s=timeout
        ):
            effective_timeout = resolve_timeout_s(predicted_s, timeout)
            raise TimeoutError(f"Gantry move did not finish within {effective_timeout:.0f}s")

    def _wait_for_axis_move_finished(
        self,
        axis: Axis,
        timeout: Optional[float] = None,
        predicted_s: float = 0.0,
        check_halt: Optional[Callable[[], None]] = None,
    ) -> None:
        """Block until a single axis's move-finished flag is set, aborting on timeout.

        used by the
        Theta branch of move_to() now that it issues a non-blocking
        begin_move_to instead of a blocking move_to() (banned — see
        commands.py). Polls sparsely via commands.poll_until_move_finished
        — see that function's module note. `timeout` default matches
        wait_for_move's.
        """
        if not poll_until_move_finished(
            lambda: self.cmd.move_is_finished(axis), predicted_s=predicted_s, timeout_s=timeout,
            check_halt=check_halt,
        ):
            self.cmd.abort(axis)
            effective_timeout = resolve_timeout_s(predicted_s, timeout)
            raise TimeoutError(f"{axis.name} move did not finish within {effective_timeout:.0f}s — aborted")


def _build_transport(config: Dict[str, Any]) -> SnapConnection:
    transport = config.get("transport", "pi_agent")

    if transport == "simulated":
        # Offline rehearsal — no serial port, no Pi, no PLC. See
        # laguna.simulation for what this does and does not prove.
        from ...simulation import SimulatedSnapConnection

        return SimulatedSnapConnection()

    if transport == "socket_bridge":
        # Retired path (2026-08-02): a raw TCP<->serial passthrough
        # (serial_bridge.py) hand-started on the Pi. Still buildable if
        # configured explicitly, but no longer the default and nothing
        # should start that bridge again — it exposed an unauthenticated
        # port straight to the controller's ASCII interpreter, and could
        # hold the serial port alongside gantry_agent.py without either
        # noticing. See docs/MACRON_GANTRY.md, "Retired: serial_bridge.py".
        # pyserial's serial_for_url() understands socket:// URLs.
        host = config["host"]
        port = config.get("bridge_port", 9700)
        return RS232Connection(
            port=f"socket://{host}:{port}", baudrate=config.get("remote_baud", 9600)
        )

    if transport == "pi_agent":
        return PiGantryConnection(
            host=config["host"],
            ssh_user=config.get("ssh_user", "oak"),
            ssh_key=config.get("ssh_key"),
            remote_serial_device=config["remote_serial_device"],
            remote_baud=config.get("remote_baud", 9600),
            safe_mode=config.get("safe_mode", True),
        )

    if transport == "ethernet":
        eth = config.get("ethernet", {})
        return EthernetConnection(host=eth["host"], port=eth.get("port", 23))

    if transport == "rs232":
        rs = config.get("rs232", {})
        return RS232Connection(port=rs["port"], baudrate=rs.get("baud", 9600))

    raise ValueError(f"Unknown gantry transport: {transport!r}")


def _build_io_map(axes_cfg: List[Dict[str, Any]]) -> IOMap:
    kwargs: Dict[str, Any] = {}
    for entry in axes_cfg:
        name = entry.get("name", "").lower()
        if name == "y":
            if entry.get("brake_output") is not None:
                kwargs["y_brake_output"] = entry["brake_output"]
            if entry.get("brake_status_input") is not None:
                kwargs["y_brake_status_input"] = entry["brake_status_input"]
        elif name == "z":
            if entry.get("brake_output") is not None:
                kwargs["z_brake_output"] = entry["brake_output"]
            if entry.get("brake_status_input") is not None:
                kwargs["z_brake_status_input"] = entry["brake_status_input"]
        elif name == "theta":
            if entry.get("limit_input") is not None:
                kwargs["theta_limit_input"] = entry["limit_input"]
    return IOMap(**kwargs)


def _build_soft_limits(
    axes_cfg: List[Dict[str, Any]],
) -> Dict[str, Tuple[Optional[float], Optional[float]]]:
    """Resolve per-axis (negative_limit_mm, positive_limit_mm) from config.

    Only axes with at least one bound set in config appear in the result
    — see GantryController._apply_soft_limits(), the only consumer.
    """
    limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
    for entry in axes_cfg:
        neg = entry.get("soft_negative_limit_mm")
        pos = entry.get("soft_positive_limit_mm")
        if neg is not None or pos is not None:
            limits[entry["name"]] = (neg, pos)
    return limits


def _build_homing_config(
    homing_cfg: Dict[str, Any], axes_cfg: List[Dict[str, Any]], io_map: IOMap
) -> HomingConfig:
    config = HomingConfig(
        homing_speed=homing_cfg.get("speed_mm_s", 10.0),
        standoff_distance=homing_cfg.get("standoff_mm", 5.0),
    )
    order_names = homing_cfg.get("order")
    if order_names:
        resolved = []
        for name in order_names:
            axis = _lookup_axis(axes_cfg, name)
            if axis is None:
                raise KeyError(f"homing.order references unknown axis {name!r}")
            resolved.append(axis)
        config.home_order = tuple(resolved)

    axis_configs: Dict[Axis, AxisHomingConfig] = {}
    home_channels = {X_AXIS: io_map.x_home_input, Y_AXIS: io_map.y_home_input, Z_AXIS: io_map.z_home_input}
    limit_channels = {X_AXIS: io_map.x_limit_input, Y_AXIS: io_map.y_limit_input, Z_AXIS: io_map.z_limit_input}
    for entry in axes_cfg:
        axis = _lookup_axis(axes_cfg, entry.get("name", ""))
        if axis is None or axis not in config.home_order or axis not in home_channels:
            continue  # Theta has no home/limit switch homing support on this hardware

        switch = entry.get("home_switch", "home")
        if switch == "home":
            index = home_channels[axis]
        elif switch == "limit":
            index = limit_channels[axis]
        else:
            raise ValueError(
                f"axes[name={entry.get('name')!r}].home_switch must be 'home' or 'limit', got {switch!r}"
            )
        if index is None:
            raise ValueError(
                f"axes[name={entry.get('name')!r}].home_switch={switch!r} but IOMap has no "
                f"{switch}_input channel configured for this axis — set it in the axes config "
                f"(brake/limit fields) or pass a fully-populated io_map"
            )

        axis_configs[axis] = AxisHomingConfig(
            input_index=index,
            # Confirmed on hardware 2026-08-25: home switches read LOW when
            # triggered (normally-closed wiring) — default matches that,
            # override per axis with "home_trip_on_high" if a given switch
            # differs.
            trip_on_high=entry.get("home_trip_on_high", False),
            homing_direction=entry.get("home_direction", -1.0),
            max_travel_mm=entry.get("max_travel_mm"),
        )
    config.axis_configs = axis_configs
    return config


def _resolve_gcode_axes(axes_cfg: List[Dict[str, Any]]) -> Tuple[Axis, Axis]:
    """Pick the X/Y axes for the GCodeExecutor's commander-node group.

    By name for the coordinated group.

    Z/Theta are deliberately not included — they cannot join this group on
    this hardware (different PLC node) and are passed separately as the
    executor's z_axis/theta_axis/theta_cmd. See gcode.py's module
    docstring.
    """
    resolved = [axis for axis in (_lookup_axis(axes_cfg, name) for name in ("X", "Y")) if axis]
    if len(resolved) == 2:
        return tuple(resolved)
    return (X_AXIS, Y_AXIS)


def _build_fences(fence_configs: List[Dict[str, Any]]) -> List[Fence]:
    fences: List[Fence] = []
    for f in fence_configs:
        kind = f.get("type")
        if kind == "box":
            x_min, x_max = f["x"]
            y_min, y_max = f["y"]
            z_min, z_max = f["z"]
            fences.append(BoxFence(f["name"], x_min, x_max, y_min, y_max, z_min, z_max))
        elif kind == "cylinder":
            z_min, z_max = f["z"]
            fences.append(
                CylinderFence(f["name"], f["center_x"], f["center_y"], f["radius"], z_min, z_max)
            )
        else:
            raise ValueError(f"Unknown fence type: {kind!r}")
    return fences
