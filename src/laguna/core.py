"""Core orchestrator module for FlumeLab.

FlumeLab uses an opt-in model: instantiate subsystems separately and attach them
with lab.add(subsystem). This avoids hardcoding hardware assumptions in the core.
"""
from contextlib import contextmanager
from typing import Any, Dict, Iterator, Optional
import logging

from .config import Config
from .timing import CheckpointStore, EventLog, ExperimentClock, Scheduler

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class FlumeLab:
    """Main orchestrator for the flume lab robotic system.

    Creates a timing backbone immediately; hardware subsystems are registered
    explicitly via ``lab.add(subsystem)``.

    Example::

        lab = FlumeLab("config.yaml")
        lab.add(CameraManager(configs)).add(RobotController(cfg))
        lab.connect_all()
        with lab.experiment() as clock:
            lab.scheduler.run(duration=300)

    Attributes:
        clock:      Experiment clock (wall time + runtime)
        scheduler:  Action scheduler tied to the experiment clock
        event_log:  Append-only CSV event log
    """

    def __init__(self, config_file: Optional[str] = None) -> None:
        logger.info("Initializing FlumeLab system...")

        self.config = Config(config_file=config_file)

        # Timing subsystem — always present
        self.clock = ExperimentClock()
        self.event_log = EventLog(
            self.config.get_value("timing.event_log", "./experiment_events.csv")
        )
        self.scheduler = Scheduler(clock=self.clock, event_log=self.event_log)

        # Registry for opt-in hardware subsystems
        self._subsystems: Dict[str, Any] = {}

        self.is_running = False
        logger.info("FlumeLab timing backbone ready — add subsystems via lab.add()")

    # ------------------------------------------------------------------
    # Opt-in subsystem registration
    # ------------------------------------------------------------------

    def add(self, subsystem: Any) -> "FlumeLab":
        """Register a hardware subsystem by its ``subsystem_name`` attribute.

        The subsystem is stored both in ``self._subsystems`` (keyed by name)
        and as a direct attribute (``self.<subsystem_name>``), making
        ``lab.cameras``, ``lab.robot``, etc. work naturally.

        Args:
            subsystem: Any object with a ``subsystem_name`` class or instance
                       attribute (e.g. CameraManager, RobotController).

        Returns:
            self — so calls can be chained: ``lab.add(cam).add(robot)``
        """
        name = getattr(subsystem, "subsystem_name", None)
        if not name:
            raise ValueError(
                f"{type(subsystem).__name__} has no 'subsystem_name' attribute; "
                "cannot register as a FlumeLab subsystem."
            )
        self._subsystems[name] = subsystem
        setattr(self, name, subsystem)
        logger.info("Registered subsystem '%s' (%s)", name, type(subsystem).__name__)
        return self

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect_all(self) -> bool:
        """Establish connections to all registered subsystems.

        Calls ``connect()`` or ``start()`` on each subsystem if the method
        exists, and collects failures.

        Returns:
            True if all connections successful, False if any failed.
        """
        logger.info("Connecting to all subsystems...")
        ok = True

        for name, subsystem in self._subsystems.items():
            # Prefer connect(); fall back to start() for camera-style APIs.
            connector = getattr(subsystem, "connect", None) or getattr(subsystem, "start", None)
            if connector is None:
                continue
            if not connector():
                logger.warning("Failed to connect subsystem '%s'", name)
                ok = False

        if ok:
            logger.info("All subsystems connected successfully")
        return ok

    def disconnect_all(self) -> None:
        """Disconnect all registered subsystems."""
        logger.info("Disconnecting all subsystems...")
        for name, subsystem in self._subsystems.items():
            disconnector = getattr(subsystem, "disconnect", None) or getattr(subsystem, "stop", None)
            if disconnector:
                disconnector()
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
    # Status / safety
    # ------------------------------------------------------------------

    def get_system_status(self) -> Dict[str, Any]:
        """Return a snapshot of all subsystem states.

        Each registered subsystem is queried via ``get_status()`` if available;
        otherwise a minimal ``{'registered': True}`` placeholder is used.
        The timing backbone is always included.
        """
        wall, runtime = self.clock.now()
        status: Dict[str, Any] = {
            "timing": {
                "clock_running": self.clock.is_running,
                "clock_paused": self.clock.is_paused,
                "runtime_s": runtime,
                "wall_time": wall,
            },
        }
        for name, subsystem in self._subsystems.items():
            getter = getattr(subsystem, "get_status", None)
            status[name] = getter() if getter else {"registered": True}
        return status

    def emergency_stop(self) -> None:
        """Emergency stop — immediately shut down all registered systems."""
        logger.warning("EMERGENCY STOP activated!")
        for subsystem in self._subsystems.values():
            stopper = getattr(subsystem, "stop", None) or getattr(subsystem, "disconnect", None)
            if stopper:
                stopper()
        if self.clock.is_running and not self.clock.is_paused:
            self.clock.pause()
        self.disconnect_all()

    def open_ocean_control_gui(self, gui_script_path: str = None) -> None:
        """Launch the OceanControl GUI as a subprocess."""
        import subprocess
        import sys

        path = gui_script_path or self.config.get("ocean_control", {}).get(
            "gui_path",
            "/home/eric/Desktop/safl-ocean-control/OceanControl/Python Controls/SAFL_OceanControl.py",
        )
        path = Path(path)
        if not path.exists():
            logger.warning(f"OceanControl GUI not found at {path} — skipping launch")
            return
        logger.info(f"Launching OceanControl GUI: {path}")
        subprocess.Popen([sys.executable, str(path)], cwd=str(path.parent))
