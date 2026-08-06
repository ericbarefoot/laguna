"""Local camera acquisition via OpenCV (single device on this machine)."""

from typing import Any, Dict, Optional
import logging

logger = logging.getLogger(__name__)


class LocalCamera:
    """Interface for a locally-attached camera (USB, CSI, etc.) via OpenCV.

    Mirrors the interface of NetworkCameraArray so CameraManager can treat
    both types uniformly.
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        """Initialize local camera from configuration dict.

        Args:
            config: Dict with optional keys: name, device_id, fps, resolution,
                capture_format.
        """
        self.name = config.get("name", "local")
        self.device_id = config.get("device_id", 0)
        self.fps = config.get("fps", 30)
        self.resolution = tuple(config.get("resolution", (1920, 1080)))
        self.capture_format = config.get("capture_format", "BGR")

        self.camera = None
        self.is_recording = False
        self.frame_count = 0

        logger.info("LocalCamera '%s' initialised (device %s)", self.name, self.device_id)

    def start(self) -> bool:
        """Initialise and start the camera."""
        try:
            # TODO: implement with cv2.VideoCapture
            # import cv2
            # self.camera = cv2.VideoCapture(self.device_id)
            # self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            self.is_recording = True
            self.frame_count = 0
            logger.info("LocalCamera '%s' started", self.name)
            return True
        except Exception as exc:
            logger.error("Failed to start LocalCamera '%s': %s", self.name, exc)
            return False

    def stop(self) -> None:
        """Stop the camera and release resources."""
        if self.is_recording:
            # TODO: self.camera.release()
            self.is_recording = False
            logger.info("LocalCamera '%s' stopped (frames: %d)", self.name, self.frame_count)

    def get_frame(self) -> Optional[Any]:
        """Capture and return a single frame (numpy array)."""
        if not self.is_recording:
            logger.warning("LocalCamera '%s': not recording", self.name)
            return None
        try:
            # TODO: ret, frame = self.camera.read(); return frame if ret else None
            self.frame_count += 1
            return None
        except Exception as exc:
            logger.error("LocalCamera '%s' frame error: %s", self.name, exc)
            return None

    def start_recording(self, output_file: str) -> bool:
        """Start recording frames to a video file."""
        try:
            # TODO: cv2.VideoWriter setup
            logger.info("LocalCamera '%s': recording to %s", self.name, output_file)
            return True
        except Exception as exc:
            logger.error("LocalCamera '%s': failed to start recording: %s", self.name, exc)
            return False

    def stop_recording(self) -> None:
        """Stop recording."""
        logger.info("LocalCamera '%s': recording stopped", self.name)

    def get_frame_count(self) -> int:
        """Return the total number of frames captured so far."""
        return self.frame_count


# Backwards-compatibility alias — existing code that imports CameraAcquisition still works.
CameraAcquisition = LocalCamera
