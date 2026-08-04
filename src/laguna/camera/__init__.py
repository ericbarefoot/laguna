"""Camera subsystem — manages local and networked Pi cameras uniformly.

Public API:
    CameraManager     — façade over one or more cameras defined in config
    CameraArray       — networked Pi camera array (also usable standalone)
    CaptureResult     — result dataclass from a network capture
    LocalCamera       — single locally-attached camera via OpenCV
    CameraAcquisition — backwards-compatibility alias for LocalCamera
    DslrCameraSubsystem — DSLR camera wrapper for dualcam-timelapse
"""

from .dslr import DslrCameraSubsystem
from .local import CameraAcquisition, LocalCamera
from .manager import CameraManager
from .network import DEFAULT_LEAD_TIME, CameraArray, CaptureResult, _resolve_passphrase

__all__ = [
    "CameraManager",
    "CameraArray",
    "CaptureResult",
    "LocalCamera",
    "CameraAcquisition",
    "DslrCameraSubsystem",
]
