"""Core orchestrator module that combines all subsystems."""

from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Iterator, Optional
import logging

from .config import Config
from .robot import RobotController
from .camera import CameraAcquisition, CameraManager
from .hydraulics import HydraulicsSystem
from .data import DataProcessor
from .storage import RemoteStorage
from .timing import CheckpointStore, EventLog, ExperimentClock, Scheduler

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class FlumeLab:
    """Main orchestrator for the flume lab robotic system.

    Coordinates all subsystems (robot, cameras, hydraulics, data processing,
    storage, timing) and provides a unified interface for running experiments.

    Attributes:
        robot:          Robot control subsystem
        cameras:        Camera manager (local + networked Pi arrays)
        camera:         Backwards-compatibility alias for cameras
        hydraulics:     Hydraulics control subsystem
        data_processor: Data processing subsystem
        storage:        Remote storage subsystem
        clock:          Experiment clock (wall time + runtime)
        scheduler:      Action scheduler tied to the experiment clock
        event_log:      Append-only CSV event log
    """

    def __init__(self, config_file: Optional[str] = None) -> None:
        logger.info("Initializing FlumeLab system...")

        self.config = Config(config_file=config_file)

        # Core hardware subsystems
        self.robot = RobotController(self.config.get("robot"))
        self.hydraulics = HydraulicsSystem(self.config.get("hydraulics"))
        self.data_processor = DataProcessor(self.config.get("data"))
        self.storage = RemoteStorage(self.config.get("storage"))

        # Camera subsystem — prefer the 'cameras' list; fall back to legacy 'camera' dict.
        camera_configs = self.config.get("cameras") or []
        if isinstance(camera_configs, dict):
            camera_configs = [camera_configs]
        if not camera_configs:
            legacy = dict(self.config.get("camera"))
            legacy.setdefault("type", "local")
            legacy.setdefault("name", "default")
            camera_configs = [legacy]
        self.cameras = CameraManager(camera_configs)
        self.camera = self.cameras  # backwards-compat alias

        # Timing subsystem
        self.clock = ExperimentClock()
        self.event_log = EventLog(
            self.config.get_value("timing.event_log", "./experiment_events.csv")
        )
        self.scheduler = Scheduler(clock=self.clock, event_log=self.event_log)

        self.is_running = False
        logger.info("FlumeLab system initialized successfully")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect_all(self) -> bool:
        """Establish connections to all subsystems.

        Returns:
            True if all connections successful, False if any failed.
        """
        logger.info("Connecting to all subsystems...")
        ok = True

        if not self.robot.connect():
            logger.warning("Failed to connect robot")
            ok = False

        if not self.cameras.start():
            logger.warning("Failed to start cameras")
            ok = False

        if not self.hydraulics.connect():
            logger.warning("Failed to connect hydraulics")
            ok = False

        if not self.storage.connect():
            logger.warning("Failed to connect remote storage")
            ok = False

        if ok:
            logger.info("All subsystems connected successfully")
        return ok

    def disconnect_all(self) -> None:
        """Disconnect all subsystems."""
        logger.info("Disconnecting all subsystems...")
        self.robot.disconnect()
        self.cameras.stop()
        self.hydraulics.disconnect()
        self.storage.disconnect()
        self.is_running = False
        logger.info("All subsystems disconnected")

    # ------------------------------------------------------------------
    # Experiment context manager
    # ------------------------------------------------------------------

    @contextmanager
    def experiment(
        self,
        resume: bool = False,
        checkpoint_file: Optional[str] = None,
    ) -> Iterator[ExperimentClock]:
        """Context manager that starts/stops the clock and logs experiment boundaries.

        Yields the ExperimentClock so the caller can call clock.wait_until(),
        clock.elapsed(), etc. directly inside the with-block.

        A CheckpointStore is created automatically (use resume=True on restart).

        Example::

            lab = FlumeLab("config.yaml")
            lab.connect_all()
            with lab.experiment(resume=False) as clock:
                lab.scheduler.repeat(every=5, action=lab.cameras.trigger_capture)
                lab.scheduler.run(duration=300)

        Args:
            resume:          If True, reload a previous checkpoint file rather
                             than starting fresh.
            checkpoint_file: Override the path from config
                             (timing.checkpoint_file).
        """
        cp_path = checkpoint_file or self.config.get_value(
            "timing.checkpoint_file", "./experiment_checkpoint.json"
        )
        store = CheckpointStore(cp_path, resume=resume)

        self.clock.start()
        self.event_log.log(0.0, "flume_lab", "experiment_start")
        try:
            yield self.clock
        finally:
            self.clock.stop()
            self.event_log.log(self.clock.elapsed(), "flume_lab", "experiment_stop")
            self.event_log.close()

    # ------------------------------------------------------------------
    # High-level experiment helpers
    # ------------------------------------------------------------------

    def initialize_experiment(
        self, experiment_config: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Perform startup procedures (home robot, set pressure, etc.).

        Args:
            experiment_config: Optional experiment-specific overrides.

        Returns:
            True if initialization succeeded.
        """
        logger.info("Initializing experiment...")
        try:
            if not self.robot.home():
                logger.error("Failed to home robot")
                return False

            if experiment_config and "hydraulics" in experiment_config:
                pressure = experiment_config["hydraulics"].get("pressure_target")
                if pressure:
                    self.hydraulics.set_pressure(pressure)

            if not self.hydraulics.start():
                logger.error("Failed to start hydraulics")
                return False

            self.data_processor.clear_buffer()
            self.is_running = True
            logger.info("Experiment initialized successfully")
            return True
        except Exception as exc:
            logger.error("Experiment initialization failed: %s", exc)
            return False

    def run_experiment(
        self, experiment_config: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Run a full experiment: connect, initialize, execute, clean up.

        Args:
            experiment_config: Dictionary with experiment parameters.

        Returns:
            True if experiment completed successfully.
        """
        logger.info("Starting experiment...")
        try:
            if not self.connect_all():
                logger.error("Failed to connect to all systems")
                return False

            if not self.initialize_experiment(experiment_config):
                logger.error("Failed to initialize experiment")
                self.disconnect_all()
                return False

            # TODO: Implement main experiment loop using scheduler + clock.
            logger.info("Experiment completed successfully")

            self.data_processor.save_data("experiment_data.csv")

            if self.storage.enabled:
                output_path = self.data_processor.output_directory / "experiment_data.csv"
                self.storage.upload_file(str(output_path), "experiments/experiment_data.csv")

            return True
        except Exception as exc:
            logger.error("Experiment failed: %s", exc)
            return False
        finally:
            self.disconnect_all()

    # ------------------------------------------------------------------
    # Status / safety
    # ------------------------------------------------------------------

    def get_system_status(self) -> Dict[str, Any]:
        """Return a snapshot of all subsystem states."""
        wall, runtime = self.clock.now()
        return {
            "robot": {
                "connected": self.robot.is_connected,
                "position": self.robot.get_position(),
            },
            "cameras": {
                "recording": self.cameras.is_recording,
                "frames_captured": self.cameras.get_frame_count(),
            },
            "hydraulics": {
                "active": self.hydraulics.is_active,
                "status": self.hydraulics.get_status(),
            },
            "data": {
                "buffer_size": self.data_processor.get_buffer_size(),
            },
            "storage": {
                "enabled": self.storage.enabled,
                "connected": self.storage.is_connected,
            },
            "timing": {
                "clock_running": self.clock.is_running,
                "clock_paused": self.clock.is_paused,
                "runtime_s": runtime,
                "wall_time": wall,
            },
        }

    def emergency_stop(self) -> None:
        """Emergency stop — immediately shut down all systems."""
        logger.warning("EMERGENCY STOP activated!")
        self.robot.stop()
        self.hydraulics.stop()
        self.cameras.stop()
        if self.clock.is_running and not self.clock.is_paused:
            self.clock.pause()
        self.disconnect_all()
