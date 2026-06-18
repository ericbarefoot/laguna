"""DSLR camera subsystem wrapper for dualcam-timelapse package.

This module wraps Minsik's dualcam-timelapse (https://github.com/yukms/dualcam-timelapse)
as a proper laguna subsystem, allowing it to be registered with lab.add() and coordinated
with other subsystems.

The dualcam-timelapse package is not pip-installable. It must be cloned separately
and passed to DslrCameraSubsystem at initialization.
"""

import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class DslrCameraSubsystem:
    """DSLR camera subsystem using dualcam-timelapse.

    Wraps dualcam.CameraManager (Canon DSLR capture via gphoto2) to integrate with
    the laguna FlumeLab orchestrator.

    Note: gphoto2 and libgphoto2 are Linux-specific. This subsystem will not work
    on macOS or Windows.

    Example::

        dslr = DslrCameraSubsystem(
            config_path="/path/to/dualcam-timelapse/config/cameras.yaml",
            dualcam_path="/path/to/dualcam-timelapse"
        )
        lab.add(dslr)
        lab.connect_all()
        dslr.capture_all()
    """

    subsystem_name = "dslr_cameras"

    def __init__(
        self,
        config_path: str,
        dualcam_path: Optional[str] = None,
    ) -> None:
        """Initialize DSLR subsystem.

        Args:
            config_path: Path to dualcam YAML config (cameras.yaml).
            dualcam_path: Optional path to dualcam-timelapse repo root.
                         If provided, added to sys.path before import.
        """
        self.config_path = Path(config_path)
        self.dualcam_path = Path(dualcam_path) if dualcam_path else None
        self._camera_manager = None
        self._is_connected = False

    def connect(self) -> bool:
        """Load YAML config and connect to both cameras.

        Lazy-imports dualcam on first connect so import errors are clear.

        Returns:
            True if both cameras connected successfully, False otherwise.
        """
        try:
            from .gvfs import release_gphoto_usb, reset_usb_cameras

            # Level 1: kill gvfsd-gphoto2 so it can't re-claim cameras
            released = release_gphoto_usb()
            if released:
                logger.info("Released %d gvfsd-gphoto2 process(es)", released)

            # Lazy import — avoids hard dependency on gphoto2/libgphoto2 at startup
            if self.dualcam_path:
                dualcam_path_str = str(self.dualcam_path)
                if dualcam_path_str not in sys.path:
                    sys.path.insert(0, dualcam_path_str)
                    logger.info("Added dualcam-timelapse to sys.path: %s", dualcam_path_str)

            from dualcam import CameraManager

            logger.info("Loading DSLR config from %s", self.config_path)
            self._camera_manager = CameraManager.from_yaml(str(self.config_path))
            connect_results = self._camera_manager.connect_all()
            logger.info("Camera connection results (attempt 1): %s", connect_results)

            if not any(connect_results.values()):
                logger.warning(
                    "No cameras connected after gvfs release. "
                    "If cameras have a stale PTP session, call dslr.usb_reset() then reconnect."
                )

            if not any(connect_results.values()):
                logger.error("No cameras connected after USB reset")
                return False

            self._camera_manager.apply_settings_all()
            self._is_connected = True
            logger.info("DSLR cameras connected successfully")
            return True

        except ImportError as e:
            logger.error(
                "Failed to import dualcam — ensure gphoto2 and PyYAML are installed:\n"
                "  Linux: sudo apt install libgphoto2-dev gphoto2\n"
                "  Then: pip install gphoto2 PyYAML\n"
                "Error: %s",
                e,
            )
            return False
        except Exception as e:
            logger.error("Failed to connect DSLR cameras: %s", e)
            return False

    def usb_reset(self) -> list:
        """Level-2 recovery: USB bus-reset all Canon cameras and re-detect ports.

        Use this when connect() fails with a stale PTP session error (-1) that
        survives a gvfs release. After calling this, call connect() again —
        cameras will have new USB device numbers.

        Returns the list of device paths that were reset.
        """
        from .gvfs import reset_usb_cameras, release_gphoto_usb
        reset_devs = reset_usb_cameras()
        release_gphoto_usb()
        return reset_devs

    def disconnect(self) -> None:
        """Disconnect all cameras."""
        if self._camera_manager:
            try:
                self._camera_manager.disconnect_all()
                logger.info("DSLR cameras disconnected")
            except Exception as e:
                logger.warning("Error disconnecting DSLR cameras: %s", e)
        self._is_connected = False

    def stop(self) -> None:
        """Alias for disconnect() — called by lab.disconnect_all()."""
        self.disconnect()

    def capture_all(self) -> Dict[str, Optional[Path]]:
        """Trigger both cameras to capture simultaneously.

        Wraps dualcam.CameraManager.capture_all_parallel(), which uses ThreadPoolExecutor
        to fire cameras concurrently. This method is thread-safe and suitable for
        scheduling from the experiment scheduler.

        Returns:
            Dict mapping camera name to output Path, or None if capture failed.
            Example: {"Hangang": Path(...), "Nakdong": Path(...)}
        """
        if not self._is_connected or not self._camera_manager:
            logger.warning("DSLR cameras not connected — skipping capture")
            return {}

        try:
            results = self._camera_manager.capture_all_parallel()
            logger.info("DSLR capture complete: %s", list(results.keys()))
            return results
        except Exception as e:
            logger.error("DSLR capture failed: %s", e)
            return {}

    def get_status(self) -> Dict[str, Any]:
        """Return status snapshot of DSLR subsystem.

        Returns:
            Dict with subsystem name, connection state, and per-camera info.
        """
        return {
            "subsystem": self.subsystem_name,
            "is_connected": self._is_connected,
            "config_path": str(self.config_path),
            "num_cameras": len(self._camera_manager.cameras) if self._camera_manager else 0,
        }
