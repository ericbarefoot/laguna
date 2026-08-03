"""Camera subsystem — manages local and networked Pi cameras uniformly.

Public API:
    CameraManager     — façade over one or more cameras defined in config
    CameraArray       — networked Pi camera array (also usable standalone)
    CaptureResult     — result dataclass from a network capture
    LocalCamera       — single locally-attached camera via OpenCV
    CameraAcquisition — backwards-compatibility alias for LocalCamera
    DslrCameraSubsystem — DSLR camera wrapper for dualcam-timelapse
"""

import logging
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from .local import LocalCamera, CameraAcquisition
from .network import CameraArray, CaptureResult, _resolve_passphrase, DEFAULT_LEAD_TIME
from .dslr import DslrCameraSubsystem

logger = logging.getLogger(__name__)

__all__ = [
    "CameraManager",
    "CameraArray",
    "CaptureResult",
    "LocalCamera",
    "CameraAcquisition",
    "DslrCameraSubsystem",
]


class CameraManager:
    """Façade that manages one or more cameras defined in a config list.

    Config format (list of camera dicts):

        cameras:
          - name: pi_array
            type: network
            hosts: [antares.laguna, sirius.laguna]
            ssh_user: pi
            ssh_key: ~/.ssh/id_rsa
            lead_time: 5.0
            output_dir: ./captures
          - name: side_view
            type: local
            device_id: 0
            fps: 30

    trigger_capture() fires all network cameras concurrently and collects
    local frames, returning a flat list of CaptureResult objects.
    """

    subsystem_name = "cameras"

    def __init__(self, configs: List[Dict[str, Any]]) -> None:
        self._network: List[CameraArray] = []
        self._local: List[LocalCamera] = []
        self._lead_times: Dict[int, float] = {}
        self._output_dirs: Dict[int, Path] = {}

        for cfg in configs:
            cam_type = cfg.get("type", "local")
            if cam_type == "network":
                passphrase = _resolve_passphrase(cfg.get("ssh_passphrase"))
                array = CameraArray(
                    hosts=cfg.get("hosts", []),
                    ssh_user=cfg.get("ssh_user", "pi"),
                    ssh_key=cfg.get("ssh_key"),
                    ssh_passphrase=passphrase,
                )
                idx = len(self._network)
                self._lead_times[idx] = cfg.get("lead_time", DEFAULT_LEAD_TIME)
                self._output_dirs[idx] = Path(cfg.get("output_dir", "./captures"))
                self._network.append(array)
                logger.info(
                    "CameraManager: registered network array '%s' (%d hosts)",
                    cfg.get("name", f"network_{idx}"),
                    len(array.hosts),
                )
            elif cam_type == "local":
                self._local.append(LocalCamera(cfg))
            else:
                logger.warning("CameraManager: unknown camera type '%s', skipping", cam_type)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Start all local cameras. Network cameras need no persistent connection."""
        ok = True
        for cam in self._local:
            if not cam.start():
                ok = False
        return ok

    def stop(self) -> None:
        """Stop all local cameras."""
        for cam in self._local:
            cam.stop()

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # Cameras have nothing hazardous to quiesce — no motion, no hydraulics —
    # so every tier reduces to the same action: stop each local camera's
    # capture loop. Scheduled captures already stop when FlumeLab halts the
    # scheduler ahead of these calls; this only covers the persistent local
    # capture thread, which keeps running independently of the scheduler.

    def pause(self) -> Optional[str]:
        """Stop local capture loops; resume() restarts them."""
        return self._halt_local()

    def resume(self) -> Optional[str]:
        """Restart local capture loops stopped by pause()."""
        try:
            if not self.start():
                return "one or more local cameras failed to restart"
        except Exception as exc:
            logger.error("Could not restart local cameras: %s", exc)
            return f"local cameras may not have restarted: {exc}"
        return None

    def estop(self) -> Optional[str]:
        """Same as stop: cameras have no harder halt available."""
        return self._halt_local()

    def _halt_local(self) -> Optional[str]:
        """Stop every local camera, each attempted independently. Never raises."""
        problems = []
        for cam in self._local:
            try:
                cam.stop()
            except Exception as exc:
                logger.error("Could not stop local camera: %s", exc)
                problems.append(str(exc))
        return f"could not stop: {'; '.join(problems)}" if problems else None

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def trigger_capture(
        self, lead_time: Optional[float] = None
    ) -> List[CaptureResult]:
        """Trigger all network cameras concurrently, then capture local frames.

        Args:
            lead_time: Override the per-array lead_time from config.

        Returns:
            Flat list of CaptureResult — one entry per Pi for network arrays,
            one stub entry per local camera.
        """
        all_results: List[Optional[List[CaptureResult]]] = [None] * len(self._network)

        def _network_worker(idx: int, array: CameraArray) -> None:
            lt = lead_time if lead_time is not None else self._lead_times.get(idx, DEFAULT_LEAD_TIME)
            all_results[idx] = array.trigger_capture(lead_time=lt)

        threads = [
            threading.Thread(target=_network_worker, args=(i, arr), daemon=True)
            for i, arr in enumerate(self._network)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        flat: List[CaptureResult] = []
        for result_list in all_results:
            if result_list:
                flat.extend(result_list)

        for cam in self._local:
            frame = cam.get_frame()
            flat.append(
                CaptureResult(
                    hostname=cam.name,
                    success=frame is not None or cam.is_recording,
                    filename=None,
                )
            )

        return flat

    def fetch_images(
        self,
        results: List[CaptureResult],
        output_dir: Optional[Union[str, Path]] = None,
    ) -> Dict[str, Path]:
        """SFTP-fetch images from all network arrays."""
        combined: Dict[str, Path] = {}
        for idx, array in enumerate(self._network):
            dest = Path(output_dir) if output_dir else self._output_dirs.get(idx, Path("./captures"))
            combined.update(array.fetch_images(results, dest))
        return combined

    def report_simultaneity(self, results: List[CaptureResult]) -> List[dict]:
        """Report timing spread for each network array separately."""
        reports = []
        for array in self._network:
            relevant = [r for r in results if r.hostname in array.hosts]
            if relevant:
                reports.append(array.report_simultaneity(relevant))
        return reports

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    @property
    def is_recording(self) -> bool:
        return any(c.is_recording for c in self._local)

    def get_frame_count(self) -> int:
        return sum(c.get_frame_count() for c in self._local)

    def get_status(self) -> dict:
        """Return a status snapshot for this subsystem.

        Returns:
            dict with at minimum 'subsystem' and 'num_cameras', plus
            counts of network vs local cameras, recording state, and
            total frames captured so far.
        """
        return {
            "subsystem": self.subsystem_name,
            "num_cameras": len(self._network) + len(self._local),
            "num_network_arrays": len(self._network),
            "num_local_cameras": len(self._local),
            "is_recording": self.is_recording,
            "frames_captured": self.get_frame_count(),
        }
