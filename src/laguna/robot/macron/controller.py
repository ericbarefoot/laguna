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
import time
from typing import Any, Dict, List, Optional, Tuple

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
)
from .connection import EthernetConnection, RS232Connection, SnapConnection, SnapMotionError
from .fences import BoxFence, CylinderFence, Fence, FenceRegistry, TrajectoryChecker
from .gcode import GCodeExecutor
from .homing import HomingConfig, HomingProcedure
from ..motion_arbiter import DEFAULT_ARBITER
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
    """Resolve an axis by name: prefer an explicit axes_cfg entry, falling
    back to the known named-axis singletons (X/Y/Z/Theta) so that
    homing.order / gcode axis selection still works even when the config
    omits the axes: list entirely (using the all-default axis set)."""
    index = _axis_index_by_name(axes_cfg, name)
    if index is not None:
        return _resolve_axis(name, index)
    return _NAMED_AXES.get(name)


class GantryController:
    """FlumeLab subsystem facade for the macron gantry.

    Build via GantryController.from_config(cfg) (cfg is the 'gantry:'
    section of a laguna config — see config/example_config.yaml) rather
    than constructing directly, unless custom fences/axes are needed
    programmatically.
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
        safe_mode: bool = True,
        mm_per_unit: float = 1.0,
        coordinate_offset_mm: Optional[Dict[str, float]] = None,
        position_checkpoint_file: Optional[str] = None,
        arbiter: Optional[Any] = None,
    ):
        self._connection = connection
        #: Shared gantry lock — see laguna.robot.motion_arbiter.
        self.arbiter = arbiter or DEFAULT_ARBITER
        self._axes = axes
        self._group_index = group_index
        self._io_map = io_map or IOMap()
        self._safe_mode = safe_mode
        self._is_connected = False
        # See position_store.py / restore_last_position() — off (None) unless
        # a path is configured, since it's a stopgap for the obstructed-
        # limit-switch homing situation, not something every gantry needs.
        self._position_store = (
            GantryPositionStore(position_checkpoint_file)
            if position_checkpoint_file
            else None
        )

        self.cmd = MMCCommands(
            connection,
            group_index=group_index,
            mm_per_unit=mm_per_unit,
            coordinate_offset_mm=coordinate_offset_mm,
            group_axes=gcode_axes,
        )

        # Per-axis convenience handles — lab.gantry.axis("Y") always works;
        # lab.gantry.y (etc.) is set dynamically below for whatever axes are
        # actually configured. See AxisHandle in commands.py for what these
        # wrap and how the safe_mode gate applies to their motion methods.
        self._axis_handles: Dict[str, AxisHandle] = {}
        for axis in self._axes:
            handle = AxisHandle(
                self.cmd, axis, is_safe_mode=lambda: self._safe_mode, io_map=self._io_map
            )
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
            group_index=group_index,
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
        """Return the AxisHandle for a configured axis by name (case-sensitive,
        matches the 'name' field in config's gantry.axes list — e.g. "X",
        "Y", "Z", "Theta"). Equivalent to the dynamic lab.gantry.<name.lower()>
        attribute, but useful when the axis name is only known at runtime.
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
        """Engage the electromagnetic brake on the given axis (Y or Z only —
        raises ValueError for axes without a brake). Accepts an axis name
        ("Y"), an Axis object, or an AxisHandle (e.g. lab.gantry.y) — same
        effect as lab.gantry.y.engage_brake(), just callable with the axis
        as an argument instead. See AxisHandle.engage_brake in commands.py.
        """
        self._resolve_axis_handle(axis).engage_brake()

    def disengage_brake(self, axis: "Axis | AxisHandle | str") -> None:
        """Disengage the electromagnetic brake on the given axis (Y or Z
        only — raises ValueError for axes without a brake). See
        engage_brake() above for accepted `axis` forms."""
        self._resolve_axis_handle(axis).disengage_brake()

    def brake_is_disengaged(self, axis: "Axis | AxisHandle | str") -> bool:
        """True if the given axis's brake is currently disengaged (released).
        See engage_brake() above for accepted `axis` forms."""
        return self._resolve_axis_handle(axis).brake_is_disengaged()

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "GantryController":
        """Build a GantryController from a laguna 'gantry:' config section."""
        connection = _build_transport(config)

        axes_cfg: List[Dict[str, Any]] = config.get("axes") or []
        axes = (
            tuple(_resolve_axis(a["name"], a["index"]) for a in axes_cfg)
            if axes_cfg
            else (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)
        )

        io_map = _build_io_map(axes_cfg)
        homing_config = _build_homing_config(config.get("homing") or {}, axes_cfg)
        fences = _build_fences(config.get("fences") or [])
        gcode_axes = _resolve_gcode_axes(axes_cfg)
        gcode_z_axis = _lookup_axis(axes_cfg, "Z") or Z_AXIS

        return cls(
            connection=connection,
            axes=axes,
            group_index=config.get("group_index", 1),
            io_map=io_map,
            homing_config=homing_config,
            fences=fences,
            gcode_axes=gcode_axes,
            gcode_z_axis=gcode_z_axis,
            safe_mode=config.get("safe_mode", True),
            # Temporary DSM-project workaround — see docs/GANTRY_UNIT_CALIBRATION.md.
            # Flip gantry.mm_per_acp_unit to 1.0 in config once fixed at the source;
            # nothing else needs to change.
            mm_per_unit=config.get("mm_per_acp_unit", 1.0),
            coordinate_offset_mm=config.get("coordinate_offset"),
            position_checkpoint_file=config.get("position_checkpoint_file"),
        )

    # ------------------------------------------------------------------
    # FlumeLab subsystem interface (see FlumeLab.add/connect_all/
    # disconnect_all/get_system_status/emergency_stop in laguna.core)
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._connection.connect()
            self._is_connected = self._connection.is_connected
            # The controller may have been power-cycled/reflashed while we
            # were away, clearing its coordinated-group state — make the
            # next move re-send INI rather than assume it survived. See
            # GCodeExecutor._init_group.
            self.gcode.reset_group_init()
        except Exception as exc:
            logger.error("Failed to connect gantry: %s", exc)
            self._is_connected = False
            return False

        if self._is_connected and not self._safe_mode:
            self._enable_and_release_brakes()
        return self._is_connected

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
        self._connection.disconnect()
        self._is_connected = False

    def get_status(self) -> Dict[str, Any]:
        status: Dict[str, Any] = {
            "subsystem": self.subsystem_name,
            "is_connected": self._is_connected,
            "safe_mode": self._safe_mode,
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

    def stop(self) -> Optional[str]:
        """End cleanly: decelerate on each axis's ramp, then park the brakes.

        **This changed meaning.** It used to be the zero-decel abort with
        motors disabled; that is now estop(), where the unified vocabulary
        says it belongs (see laguna.safety). stop() is the tier below —
        controlled deceleration into a state safe to disconnect from, with
        no stall against a brake and no encoder disturbance.
        """
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
        return None

    def _persist_position(self) -> None:
        """Best-effort snapshot of every axis's live position to the position
        checkpoint file, if one is configured (position_checkpoint_file — see
        position_store.py). Never raises: a persistence failure must not
        break the caller's actual operation.

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
        recalibration described in its docstring. This is the recovery path
        for issue #23: a power cycle wipes the PLC's ACP registers entirely,
        and physical homing is currently disabled (obstructed limit
        switches — see HomingProcedure.home_all()), so without this there is
        no way to re-reference position at all short of measuring by hand.

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

    def move_to(
        self,
        vector: Optional[List[float]] = None,
        *,
        X: Optional[float] = None,
        Y: Optional[float] = None,
        Z: Optional[float] = None,
        Theta: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """Move to an absolute position, in real mm (and degrees/units for Theta).

        Both forms are fence-checked wherever they touch X/Y/Z, routed
        through the coordinated gcode path (GCodeExecutor.plan/execute) —
        the same path a hand-written G-code program would use, so a
        FenceViolation is raised before anything moves.

          - ``move_to([x, y, z, theta])`` — one value per configured axis,
            in the same order as ``self._axes`` (X, Y, Z, Theta by
            default).
          - ``move_to(X=100)`` / ``move_to(X=100, Z=5)`` — move only the
            given axes. Any configured Cartesian axis *not* given is
            backfilled with its real current position (a live
            get_actual_position() read) before the fence check runs, so
            the checked path reflects where the gantry actually is, not
            an assumed one.

        Theta is outside the Cartesian gcode/fence model (fences.py only
        checks X/Y/Z) and is always moved as a separate, non-fence-checked
        single-axis command. A pure Theta-only call (no X/Y/Z given, in
        either form) skips the gcode path entirely — it doesn't query,
        move, or otherwise touch X/Y/Z motors or brakes at all.

        For anything this doesn't cover — bypassing the fence check
        deliberately, non-coordinated per-axis motion — use self.cmd
        directly.

        Args:
            vector: Full-length position vector, or None to use keywords.
            X, Y, Z, Theta: Per-axis absolute targets (real mm; Theta in
                whatever unit that axis's raw-to-real conversion yields).
            speed: Optional feed rate (mm/s) applied to the move(s).

        Returns:
            True if a move was issued.

        Raises:
            ValueError: If vector's length doesn't match the configured
                axes, an axis keyword names an axis not configured on this
                gantry, or neither vector nor any keyword was given.
            FenceViolation: If the X/Y/Z path would enter an exclusion zone.
        """
        # Serialise whole operations, not just individual commands. The
        # transport already locks per request/response pair, but a move is
        # many commands with a physical traverse in between — nothing else
        # stops a scheduled scan landing in the middle of one. Re-entrant,
        # so nesting inside scan_with_gantry() is fine. See motion_arbiter.
        with self.arbiter.hold(f"{type(self).__name__}.move_to"):
            return self._move_to(vector, X=X, Y=Y, Z=Z, Theta=Theta, speed=speed)

    def _move_to(
        self,
        vector: Optional[List[float]] = None,
        *,
        X: Optional[float] = None,
        Y: Optional[float] = None,
        Z: Optional[float] = None,
        Theta: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        """Body of move_to(), with the arbiter already held."""
        axes_by_name = {axis.name: axis for axis in self._axes}

        if vector is not None:
            if len(vector) != len(self._axes):
                raise ValueError(
                    f"move_to(vector=...) expects {len(self._axes)} values "
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
                raise ValueError("move_to() requires either vector=... or at least one of X=/Y=/Z=/Theta=")

        theta_value = target_by_name.pop("Theta", None)

        cartesian_axes = [axis for axis in self._axes if axis.name != "Theta"]
        if any(axis.name in target_by_name for axis in cartesian_axes):
            gcode_words: List[str] = []
            for axis in cartesian_axes:
                value = target_by_name.get(axis.name)
                if value is None:
                    value = self.cmd.get_actual_position(axis)  # backfill: real current position
                gcode_words.append(f"{axis.name}{value:.6f}")
            if speed is not None:
                gcode_words.append(f"F{speed * 60:.6f}")  # gcode feed rate is mm/min
            text = "G90\nG1 " + " ".join(gcode_words)
            trajectory = self.gcode.plan(text)
            self.gcode.execute(trajectory)

        if theta_value is not None:
            theta_axis = axes_by_name["Theta"]
            # the
            # vector move_to() form always passes theta_value (0.0 if the
            # caller didn't care about Theta at all), so this used to fire
            # a blocking MVT unconditionally — including when Theta was
            # already at the target, e.g. every scan-setup move in
            # example_07. move_to() is now banned outright (see
            # commands.py); this does a live read first and skips the move
            # entirely if Theta hasn't actually changed, same as the
            # X/Y/Z legs in GCodeExecutor._execute_linear.
            current_theta = self.cmd.get_actual_position(theta_axis)
            if abs(theta_value - current_theta) > _POSITION_EPSILON_MM:
                if speed is not None:
                    self.cmd.set_speed(theta_axis, speed)
                self.cmd.begin_move_to(theta_axis, theta_value)
                self._wait_for_axis_move_finished(
                    theta_axis,
                    predicted_s=predicted_move_s(theta_value - current_theta, speed),
                )

        self._persist_position()
        return True

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
        manually). This is currently the only way to (re-)establish a
        position reference, since home() is temporarily disabled — see
        HomingProcedure.home_all(). To actually move, use move_to().

        Both forms mirror move_to()'s shape — a full vector (one value per
        configured axis, in self._axes order) or per-axis keywords — except
        that unlike move_to(), any axis *not* given is left completely
        untouched: there is no backfill, since there is no fence-checked
        path to compute one for.

        Args:
            vector: Full-length position vector, or None to use keywords.
            X, Y, Z, Theta: Per-axis new position values (real mm; Theta in
                whatever unit that axis's raw-to-real conversion yields).

        Returns:
            True if every given axis's position register was set.

        Raises:
            ValueError: If vector's length doesn't match the configured
                axes, an axis keyword names an axis not configured on this
                gantry, or neither vector nor any keyword was given.
        """
        axes_by_name = {axis.name: axis for axis in self._axes}

        if vector is not None:
            if len(vector) != len(self._axes):
                raise ValueError(
                    f"set_position(vector=...) expects {len(self._axes)} values "
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
                    "set_position() requires either vector=... or at least one of X=/Y=/Z=/Theta="
                )

        for name, value in target_by_name.items():
            self.cmd.set_actual_position(axes_by_name[name], value)
        self._persist_position()
        return True

    def soft_stop(self) -> None:
        """Decelerate every configured axis to a stop, leaving brakes and
        motors alone.

        The gentle counterpart to stop(): each axis decelerates using its
        own accel/decel ramp (BST) rather than a zero-decel abort, and
        nothing is disabled — so the gantry is immediately ready for
        another move_to() with no re-enable cycle. Use this to cancel an
        ordinary move; use stop() for an emergency.

        Never raises: one axis failing to stop is logged and does not
        prevent the rest from getting their stop command.
        """
        for axis in self._axes:
            try:
                self.cmd.begin_stop(axis)
            except Exception as exc:
                logger.error("Error soft-stopping %s: %s", axis.name, exc)
        self._persist_position()

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def pause(self) -> Optional[str]:
        """Decelerate on each axis's own ramp, leaving brakes and motors alone.

        The gantry is immediately ready to move again with no re-enable
        cycle, which is what makes a pause cheap enough to use liberally.
        """
        self.soft_stop()
        self._persist_position()
        return None

    def resume(self) -> Optional[str]:
        """Nothing to undo — pause() left brakes and motors untouched.

        Motion is re-commanded by the caller, not resumed implicitly: the
        gantry has no notion of an interrupted move to pick back up.
        """
        return None

    def estop(self) -> Optional[str]:
        """Zero-decel abort, brakes engaged, motors disabled. Never raises.

        This is what stop() used to do. Recovery needs an explicit re-arm
        (enable() + disengage_brake(), or connect()/set_safe_mode(False),
        which do both) — see FlumeLab.rearm().
        """
        try:
            self.cmd.shutdown(axes=self._axes, io_map=self._io_map)
        except Exception as exc:
            logger.error("Error during gantry emergency stop: %s", exc)
        self._persist_position()
        return None

    def set_safe_mode(self, enabled: bool) -> bool:
        """Enable or disable safe_mode, reconnecting the transport if needed
        so the change actually takes effect, and syncing Y/Z's brakes to match.

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
        re-commanded. No-op if not currently connected — connect() applies
        the release side itself, using whatever safe_mode is set to by then.

        Returns:
            True if the change took effect (including a successful
            reconnect, if one was needed); False if a required reconnect
            failed — check logs and call connect() again once resolved.
        """
        self._safe_mode = enabled
        if hasattr(self._connection, "safe_mode"):
            self._connection.safe_mode = enabled
        if isinstance(self._connection, PiGantryConnection) and self._is_connected:
            self.disconnect()
            if not self.connect():
                return False
            # connect() already released the brakes if enabled=False (it
            # checks self._safe_mode itself); it never engages, so that
            # direction still needs an explicit call here.
            if enabled:
                self._engage_brakes()
            return True
        if self._is_connected:
            if enabled:
                self._engage_brakes()
            else:
                self._enable_and_release_brakes()
        return True

    def home(self) -> bool:
        """Run the homing routine on all configured axes.

        Returns:
            True if homing completed successfully on every axis.
        """
        result = self.homing.home_all()
        return result.success

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
        """Turn motor drive off for all configured axes (MTR only), allowing
        manual repositioning.

        Does NOT send ENA — see enable().
        """
        for axis in self._axes:
            self.cmd.set_motor(axis, False)

    def wait_for_move(self, timeout: float = 30.0, predicted_s: float = 0.0) -> None:
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
        wait costs no wire traffic at all.

        Raises:
            TimeoutError: If the move hasn't finished within `timeout`.
        """
        if not poll_until_move_finished(
            self.cmd.group_move_is_finished, predicted_s=predicted_s, timeout_s=timeout
        ):
            raise TimeoutError(f"Gantry move did not finish within {timeout:.0f}s")

    def _wait_for_axis_move_finished(
        self, axis: Axis, timeout: float = 30.0, predicted_s: float = 0.0
    ) -> None:
        """Block until a single axis's move-finished flag is set, aborting on timeout.

        used by the
        Theta branch of move_to() now that it issues a non-blocking
        begin_move_to instead of a blocking move_to() (banned — see
        commands.py). Polls sparsely via commands.poll_until_move_finished
        — see that function's module note.
        """
        if not poll_until_move_finished(
            lambda: self.cmd.move_is_finished(axis), predicted_s=predicted_s, timeout_s=timeout
        ):
            self.cmd.abort(axis)
            raise TimeoutError(f"{axis.name} move did not finish within {timeout:.0f}s — aborted")


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


def _build_homing_config(homing_cfg: Dict[str, Any], axes_cfg: List[Dict[str, Any]]) -> HomingConfig:
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
    return config


def _resolve_gcode_axes(axes_cfg: List[Dict[str, Any]]) -> Tuple[Axis, Axis]:
    """Pick the X/Y axes by name for the GCodeExecutor's coordinated group.

    Z is deliberately not included — it cannot join the group on this
    hardware and is passed separately as the executor's z_axis. See
    gcode.py's "Z/XY node split" note.
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
