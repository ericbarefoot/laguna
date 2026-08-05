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
from typing import TYPE_CHECKING, Any, Dict, Optional

from ..subsystem_logging import SubsystemLogging

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)


class DslrCameraSubsystem(SubsystemLogging):
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
        config_path: Optional[str] = None,
        dualcam_path: Optional[str] = None,
        log_level: str = "INFO",
        event_log_verbosity: str = "INFO",
        simulated: bool = False,
    ) -> None:
        """Initialize DSLR subsystem.

        Args:
            config_path: Path to dualcam YAML config (cameras.yaml).
                         Not required when constructed via from_config().
            dualcam_path: Optional path to dualcam-timelapse repo root.
                         If provided, added to sys.path before import.
            log_level / event_log_verbosity: see laguna.subsystem_logging
                (both default 'INFO').
            simulated: Skip gphoto2/USB entirely — connect()/capture_all()
                succeed without touching real cameras, returning placeholder
                filenames instead of real images (default False; see
                laguna.simulation's module docstring).
        """
        self.config_path = Path(config_path) if config_path else None
        self.dualcam_path = Path(dualcam_path) if dualcam_path else None
        self.log_level = log_level
        self.event_log_verbosity = event_log_verbosity
        self._simulated = simulated
        self._camera_manager = None
        self._is_connected = False
        self._main_yaml_path: Optional[str] = None
        self._config: Optional[dict] = None

    @classmethod
    def from_config(cls, config: "Config") -> "DslrCameraSubsystem":
        """Build from the lab's Config: its 'dslr_cameras:' section, plus
        config.config_file to resolve output_dir paths and persist detected
        port assignments back to the same experiment YAML they came from.

        Raises:
            ValueError: If config.config_file is None (a Config built from
                defaults/a dict only, not FlumeLab(config_file)) — there is
                no YAML location to resolve relative paths against.
        """
        dslr_cfg = config.get("dslr_cameras")
        if config.config_file is None:
            raise ValueError(
                "DslrCameraSubsystem.from_config() needs config.config_file "
                "to resolve output_dir paths and persist port assignments — "
                "build the Config from a YAML file (FlumeLab(config_file=...) "
                "or Config(config_file=...)), not from defaults/a dict alone."
            )
        main_path = Path(config.config_file).resolve()
        main_dir = main_path.parent

        # Resolve relative output_dir values relative to the main YAML location
        cameras_raw = dslr_cfg.get("cameras", {})
        cameras_resolved: Dict[str, Any] = {}
        for cam_name, cam_cfg in cameras_raw.items():
            resolved = dict(cam_cfg)
            if "output_dir" in resolved:
                out = Path(resolved["output_dir"])
                if not out.is_absolute():
                    out = (main_dir / out).resolve()
                resolved["output_dir"] = str(out)
            cameras_resolved[cam_name] = resolved

        instance = cls(
            dualcam_path=dslr_cfg.get("dualcam_path"),
            log_level=dslr_cfg.get("log_level", "INFO"),
            event_log_verbosity=dslr_cfg.get("event_log_verbosity", "INFO"),
            simulated=dslr_cfg.get("simulated", False),
        )
        instance._config = {"cameras": cameras_resolved}
        instance._main_yaml_path = str(main_path)
        return instance

    def connect(self) -> bool:
        """Load YAML config and connect to cameras.

        Two-level recovery:
          Level 1 — kill gvfsd-gphoto2, connect with YAML ports.
          Level 2 — re-detect current ports (they change on every power cycle),
                    persist them to YAML, reload a fresh CameraManager, retry.

        Canon DSLRs maintain PTP session state through USB unplug/replug — only
        a full power cycle resets it. USB reset (USBDEVFS_RESET ioctl) is NOT
        used here because these cameras interpret it as a disconnect and drop off
        the USB bus entirely.

        Keep the connection alive for the duration of the experiment — capture_all()
        does not disconnect between captures, so connect() only needs to be called
        once per session. Power-cycle the cameras before each new session.

        Returns:
            True if at least one camera connected, False otherwise.
        """
        if self._simulated:
            # No gphoto2, no USB — self._camera_manager stays None;
            # capture_all() checks self._simulated before it would ever
            # need one.
            self._is_connected = True
            return True
        try:
            from .gvfs import release_gphoto_usb, detect_camera_ports

            if self.dualcam_path:
                dualcam_path_str = str(self.dualcam_path)
                if dualcam_path_str not in sys.path:
                    sys.path.insert(0, dualcam_path_str)

            from dualcam import CameraManager

            # --- Level 1: kill gvfsd-gphoto2, connect with YAML ports -------
            released = release_gphoto_usb()
            if released:
                logger.info("Released %d gvfsd-gphoto2 process(es)", released)

            if self._config is not None:
                self._camera_manager = CameraManager.from_config(self._config)
            else:
                logger.info("Loading DSLR config from %s", self.config_path)
                self._camera_manager = CameraManager.from_yaml(str(self.config_path))
            connect_results = self._camera_manager.connect_all()
            logger.info("DSLR connection (level 1): %s", connect_results)

            if not any(connect_results.values()):
                # --- Level 2: re-detect ports, fresh manager, retry ----------
                # Ports change on every power cycle. A failed init() also leaves
                # a poisoned gp.Context on the shared CameraManager. Canon cameras
                # also need a settling period after a PTP session ends before they
                # accept a new OpenSession. Fix: wait, re-detect ports, persist to
                # YAML, reload a fresh manager (new gp.Context) and retry.
                import time as _time
                logger.info(
                    "Level 1 failed — waiting 15 s for PTP session to settle, "
                    "then re-detecting ports"
                )
                _time.sleep(15)
                release_gphoto_usb()

                new_ports = detect_camera_ports()
                camera_names = list(self._camera_manager._cameras.keys())

                if not new_ports:
                    logger.error(
                        "No Canon cameras visible on USB. "
                        "Power-cycle the cameras (physical power off/on) and call connect() again. "
                        "USB unplug/replug alone does not reset Canon PTP state."
                    )
                    return False

                for i, name in enumerate(camera_names):
                    if i < len(new_ports):
                        logger.info("  %s -> %s", name, new_ports[i])
                    else:
                        logger.warning("  %s -> no port found", name)

                self._persist_ports(camera_names, new_ports)
                if self._config is not None:
                    self._camera_manager = CameraManager.from_config(self._config)
                else:
                    self._camera_manager = CameraManager.from_yaml(str(self.config_path))

                connect_results = self._camera_manager.connect_all()
                logger.info("DSLR connection (level 2): %s", connect_results)

            if not any(connect_results.values()):
                logger.error(
                    "No cameras connected after port re-detection. "
                    "Power-cycle the cameras (physical power off/on) and call connect() again."
                )
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

    def _persist_ports(self, camera_names: list, new_ports: list) -> None:
        """Persist updated USB port assignments to in-memory config and main YAML."""
        import yaml

        def _update_cameras_cfg(cameras_cfg: dict) -> None:
            for i, cam_name in enumerate(camera_names):
                if i < len(new_ports) and cam_name in cameras_cfg:
                    cameras_cfg[cam_name]["port"] = new_ports[i]

        if self._config is not None:
            _update_cameras_cfg(self._config.get("cameras", {}))
        elif self.config_path is not None:
            try:
                with open(self.config_path) as f:
                    config = yaml.safe_load(f)
                _update_cameras_cfg(config.get("cameras", {}))
                with open(self.config_path, "w") as f:
                    yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
                logger.info("Updated port assignments saved to %s", self.config_path)
            except Exception as e:
                logger.warning("Could not persist new ports to %s: %s", self.config_path, e)

        if self._main_yaml_path:
            try:
                with open(self._main_yaml_path) as f:
                    main_config = yaml.safe_load(f)
                _update_cameras_cfg(
                    main_config.get("dslr_cameras", {}).get("cameras", {})
                )
                with open(self._main_yaml_path, "w") as f:
                    yaml.dump(main_config, f, default_flow_style=False, allow_unicode=True)
                logger.info("Updated port assignments saved to %s", self._main_yaml_path)
            except Exception as e:
                logger.warning("Could not persist new ports to %s: %s", self._main_yaml_path, e)

    def usb_reset(self) -> list:
        """Explicit USB bus-reset for Canon cameras (use with caution).

        Canon cameras often drop off USB entirely when reset via USBDEVFS_RESET
        ioctl and require a full power cycle to recover. This method is provided
        for diagnostic use only — do not call it from automated code.

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
        to fire cameras concurrently. Cameras remain connected after capture.

        Returns:
            Dict mapping camera name to output Path, or None if capture failed.
            Example: {"Hangang": Path(...), "Nakdong": Path(...)}
        """
        if self._simulated:
            # No real capture — a placeholder path per configured camera,
            # clearly not a real filename, so log_event("capture", ...)
            # still fires and still proves the trigger reached every
            # camera, without pretending there's an actual image.
            if not self._is_connected:
                logger.warning("DSLR cameras not connected — skipping capture")
                return {}
            camera_names = list((self._config or {}).get("cameras", {}))
            results: Dict[str, Optional[Path]] = {}
            for cam_name in camera_names:
                path = Path(f"<simulated>/{cam_name}.jpg")
                results[cam_name] = path
                self.log_event("capture", camera=cam_name, file=str(path))
            logger.info("DSLR capture complete (simulated): %s", camera_names)
            return results

        if not self._is_connected or not self._camera_manager:
            logger.warning("DSLR cameras not connected — skipping capture")
            return {}

        try:
            results = self._camera_manager.capture_all_parallel()
            logger.info("DSLR capture complete: %s", list(results.keys()))
            for cam_name, path in results.items():
                if path:
                    self.log_event("capture", camera=cam_name, file=str(path))
                else:
                    self.log_event("capture_failed", level="WARNING", camera=cam_name)
            return results
        except Exception as e:
            logger.error("DSLR capture failed: %s", e)
            return {}

    def get_status(self) -> Dict[str, Any]:
        """Return status snapshot of DSLR subsystem."""
        return {
            "subsystem": self.subsystem_name,
            "is_connected": self._is_connected,
            "config_path": str(self.config_path) if self.config_path else None,
            "num_cameras": len(self._camera_manager._cameras) if self._camera_manager else 0,
        }
