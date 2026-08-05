"""3D surface scanner subsystem — LMI Gocator 2690 line-profile sensor.

Distinct from ``laguna.rangefinder`` (single-point OD2000/WTT12L distance
sensors reached through a Pi-side IO-Link master): the Gocator is a line-laser
3D scanner on the laguna PC's own Ethernet network, driven through LMI's GoSdk
C library via ctypes.

Usage sketch — see ``docs/subsystems/scanner.md`` for the full guide::

    from laguna import FlumeLab

    lab = FlumeLab("config/example_config.yaml")
    lab.add("gocator").add("gantry")
    lab.connect_all()

    scan = lab.gocator.scan_with_gantry(
        lab.gantry, axis="X", end_mm=400.0, feed_rate_mm_s=20.0
    )
    lab.gocator.save_scan(scan)
"""

from .gocator import (
    FILTER_NAMES,
    GocatorScanner,
    ScanNotPossibleError,
    UniformSpacingRequiredError,
)
from .gosdk import GoSdkError, GoSdkLib, GoSdkTimeout
from .pointcloud import (
    SurfaceScan,
    surface_point_cloud_to_scan,
    uniform_surface_to_scan,
)

__all__ = [
    "GocatorScanner",
    "UniformSpacingRequiredError",
    "ScanNotPossibleError",
    "FILTER_NAMES",
    "GoSdkLib",
    "GoSdkError",
    "GoSdkTimeout",
    "SurfaceScan",
    "uniform_surface_to_scan",
    "surface_point_cloud_to_scan",
]
