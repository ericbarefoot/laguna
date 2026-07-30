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
    IOMap,
    MMCCommands,
    THETA_AXIS,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
)
from .connection import EthernetConnection, RS232Connection, SnapConnection, SnapMotionError
from .fences import BoxFence, CylinderFence, Fence, FenceRegistry, TrajectoryChecker
from .gcode import GCodeExecutor
from .homing import HomingConfig, HomingProcedure
from .pi_bridge import PiGantryConnection

logger = logging.getLogger(__name__)

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

    def __init__(
        self,
        connection: SnapConnection,
        axes: Tuple[Axis, ...] = (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS),
        group_index: int = 1,
        io_map: Optional[IOMap] = None,
        homing_config: Optional[HomingConfig] = None,
        fences: Optional[List[Fence]] = None,
        gcode_axes: Tuple[Axis, Axis, Axis] = (X_AXIS, Y_AXIS, Z_AXIS),
        safe_mode: bool = True,
        mm_per_unit: float = 1.0,
        coordinate_offset_mm: Optional[Dict[str, float]] = None,
    ):
        self._connection = connection
        self._axes = axes
        self._group_index = group_index
        self._io_map = io_map or IOMap()
        self._safe_mode = safe_mode
        self._is_connected = False

        self.cmd = MMCCommands(
            connection,
            group_index=group_index,
            mm_per_unit=mm_per_unit,
            coordinate_offset_mm=coordinate_offset_mm,
            group_axes=gcode_axes,
        )

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

        return cls(
            connection=connection,
            axes=axes,
            group_index=config.get("group_index", 1),
            io_map=io_map,
            homing_config=homing_config,
            fences=fences,
            gcode_axes=gcode_axes,
            safe_mode=config.get("safe_mode", True),
            # Temporary DSM-project workaround — see docs/GANTRY_UNIT_CALIBRATION.md.
            # Flip gantry.mm_per_acp_unit to 1.0 in config once fixed at the source;
            # nothing else needs to change.
            mm_per_unit=config.get("mm_per_acp_unit", 1.0),
            coordinate_offset_mm=config.get("coordinate_offset"),
        )

    # ------------------------------------------------------------------
    # FlumeLab subsystem interface (see FlumeLab.add/connect_all/
    # disconnect_all/get_system_status/emergency_stop in laguna.core)
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._connection.connect()
            self._is_connected = self._connection.is_connected
            return self._is_connected
        except Exception as exc:
            logger.error("Failed to connect gantry: %s", exc)
            self._is_connected = False
            return False

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

    def stop(self) -> None:
        """Abort all axes and engage Y/Z brakes (if their channels are configured).

        Called by FlumeLab.emergency_stop() for every registered subsystem
        that has a stop() method — this is the gantry's emergency-stop path.
        """
        try:
            self.cmd.shutdown(axes=self._axes, io_map=self._io_map)
        except Exception as exc:
            logger.error("Error during gantry shutdown: %s", exc)

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
            if speed is not None:
                self.cmd.set_speed(theta_axis, speed)
            self.cmd.move_to(theta_axis, theta_value)

        return True

    def home(self) -> bool:
        """Run the homing routine on all configured axes.

        Returns:
            True if homing completed successfully on every axis.
        """
        result = self.homing.home_all()
        return result.success

    def enable(self) -> None:
        """Enable motor drive on all configured axes."""
        for axis in self._axes:
            self.cmd.set_motor(axis, True)
            self.cmd.set_enable(axis, True)

    def disable(self) -> None:
        """Disable motor drive on all configured axes (allows manual repositioning)."""
        for axis in self._axes:
            self.cmd.set_enable(axis, False)
            self.cmd.set_motor(axis, False)

    def wait_for_move(self, timeout: float = 30.0) -> None:
        """Block until the coordinated group's current move completes, or timeout elapses.

        Only tracks the coordinated (X/Y/Z) group — a Theta-only move
        issued via move_to(Theta=...) isn't covered by this; poll
        self.cmd.move_is_finished(THETA_AXIS) directly for that.

        Raises:
            TimeoutError: If the move hasn't finished within `timeout`.
        """
        deadline = time.monotonic() + timeout
        while not self.cmd.group_move_is_finished():
            if time.monotonic() > deadline:
                raise TimeoutError(f"Gantry move did not finish within {timeout:.0f}s")
            time.sleep(0.05)


def _build_transport(config: Dict[str, Any]) -> SnapConnection:
    transport = config.get("transport", "socket_bridge")

    if transport == "socket_bridge":
        # Default path: the existing raw TCP<->serial passthrough already
        # running on the Pi (serial_bridge.py) — no Pi-side laguna code
        # needed. pyserial's serial_for_url() understands socket:// URLs.
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


def _resolve_gcode_axes(axes_cfg: List[Dict[str, Any]]) -> Tuple[Axis, Axis, Axis]:
    """Pick the X/Y/Z axes by name for the GCodeExecutor (Cartesian-only)."""
    resolved = [axis for axis in (_lookup_axis(axes_cfg, name) for name in ("X", "Y", "Z")) if axis]
    if len(resolved) == 3:
        return tuple(resolved)
    return (X_AXIS, Y_AXIS, Z_AXIS)


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
