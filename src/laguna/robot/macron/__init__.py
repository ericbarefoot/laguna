"""Macron Dynamics gantry robot driver for Modusystems OEM-2T/MMC-3T controllers."""

from .connection import (
    EthernetConnection,
    RS232Connection,
    SnapConnection,
    SnapMotionError,
    probe_connection,
    find_rs232_port,
    assert_controller_present,
)
from .commands import (
    MMCCommands,
    Axis,
    AxisState,
    GroupState,
    IOMap,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    THETA_AXIS,
    ALL_AXES,
)
from .homing import HomingConfig, HomingResult, HomingProcedure, AxisHomingConfig
from .fences import (
    BoxFence,
    CylinderFence,
    Fence,
    FenceRegistry,
    FenceViolation,
    CheckedTrajectory,
    TrajectoryChecker,
)
from .gcode import (
    GCodeError,
    GCodeExecutionAborted,
    GCodeExecutor,
    GCodeMove,
    GCodeParser,
    GCodeProgram,
)
from .halt import HaltLevel, MotionHalted
from .move_handle import MoveHandle
from .pi_bridge import (
    PiGantryConnection,
    SAFE_COMMANDS,
    SafeModeConnection,
    check_safe_mode,
    is_stop_command,
)
from .controller import GantryController

__all__ = [
    # connection
    "EthernetConnection",
    "RS232Connection",
    "SnapConnection",
    "SnapMotionError",
    "probe_connection",
    "find_rs232_port",
    "assert_controller_present",
    # commands
    "MMCCommands",
    "Axis",
    "AxisState",
    "GroupState",
    "IOMap",
    "X_AXIS",
    "Y_AXIS",
    "Z_AXIS",
    "THETA_AXIS",
    "ALL_AXES",
    # homing
    "AxisHomingConfig",
    "HomingConfig",
    "HomingResult",
    "HomingProcedure",
    # fences
    "BoxFence",
    "CylinderFence",
    "Fence",
    "FenceRegistry",
    "FenceViolation",
    "CheckedTrajectory",
    "TrajectoryChecker",
    # gcode
    "GCodeError",
    "GCodeExecutionAborted",
    "GCodeExecutor",
    "GCodeMove",
    "GCodeParser",
    "GCodeProgram",
    # halt / non-blocking moves
    "HaltLevel",
    "MotionHalted",
    "MoveHandle",
    # pi_bridge
    "PiGantryConnection",
    "SAFE_COMMANDS",
    "SafeModeConnection",
    "check_safe_mode",
    "is_stop_command",
    # controller
    "GantryController",
]
