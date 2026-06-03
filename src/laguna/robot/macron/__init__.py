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
from .commands import MMCCommands, Axis, AxisState, GroupState, IOMap, X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS, ALL_AXES
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
]
