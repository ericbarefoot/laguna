"""Core orchestrator module for FlumeLab.

FlumeLab uses an opt-in model: instantiate subsystems separately and attach them
with lab.add(subsystem). This avoids hardcoding hardware assumptions in the core.
"""

from typing import Optional, Dict, Any
import logging
from pathlib import Path

from .config import Config

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class FlumeLab:
    """Main orchestrator for the flume lab system.

    Subsystems are attached at runtime via add(), which sets a named attribute
    and registers the subsystem for connect_all / disconnect_all / emergency_stop.
    """

    def __init__(self, config_file: Optional[str] = None):
        self.config = Config(config_file=config_file)
        self._subsystems: Dict[str, Any] = {}
        self.is_running = False
        logger.info("FlumeLab initialized (no subsystems attached)")

    def add(self, subsystem) -> "FlumeLab":
        name = subsystem.subsystem_name
        self._subsystems[name] = subsystem
        setattr(self, name, subsystem)
        logger.info(f"Subsystem '{name}' added: {type(subsystem).__name__}")
        return self

    def connect_all(self) -> bool:
        all_ok = True
        for name, sub in self._subsystems.items():
            if not sub.connect():
                logger.warning(f"Failed to connect subsystem '{name}'")
                all_ok = False
        return all_ok

    def disconnect_all(self) -> None:
        for sub in self._subsystems.values():
            sub.disconnect()
        self.is_running = False

    def get_system_status(self) -> Dict[str, Any]:
        return {name: sub.get_status() for name, sub in self._subsystems.items()}

    def emergency_stop(self) -> None:
        logger.warning("EMERGENCY STOP activated!")
        for sub in self._subsystems.values():
            if hasattr(sub, "stop"):
                sub.stop()
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
