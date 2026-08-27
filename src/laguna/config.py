"""Configuration management for the Laguna system.

This module handles loading and managing configuration for all subsystems.
Configuration can be loaded from YAML files or passed as dictionaries.
"""

import yaml
from pathlib import Path
from typing import Any, Dict, Optional, cast


class Config:
    """Central configuration manager for all subsystems.
    
    Attributes:
        config_dict: Dictionary containing all configuration parameters
    """
    
    def __init__(self, config_file: Optional[str] = None, defaults: Optional[Dict[str, Any]] = None):
        """Initialize configuration.

        Args:
            config_file: Path to YAML configuration file.
            defaults: Default configuration dictionary.
        """
        self.config_dict = defaults or self._get_defaults()

        #: Resolved path of the loaded config file, or None if this Config
        #: was built from defaults/a dict only. Subsystems that need to
        #: locate files relative to the experiment config (e.g.
        #: DslrCameraSubsystem) read this via from_config(config).
        self.config_file: Optional[str] = None

        #: Top-level section names that were literally present in the
        #: loaded YAML — as opposed to _get_defaults()'s unconditional
        #: defaults, which populate every section whether or not the user
        #: asked for it. FlumeLab.add_all() opt-in keys off this set, not
        #: config_dict, since config_dict can never distinguish "explicitly
        #: configured" from "just the default."
        self.explicit_sections: set = set()

        if config_file:
            self.load_from_file(config_file)
    
    @staticmethod
    def _get_defaults() -> Dict[str, Any]:
        """Get default configuration values.

        Returns:
            Dictionary with default subsystem configuration.
        """
        return {
            "robot": {
                "port": "/dev/ttyUSB0",
                "baudrate": 9600,
                "protocol": "modbus",
                "timeout": 5.0,
            },
            # Legacy single-camera config — still supported for backwards compatibility.
            # Prefer the 'cameras' list for new experiments.
            "camera": {
                "device_id": 0,
                "fps": 30,
                "resolution": (1920, 1080),
                "capture_format": "BGR",
            },
            # weir/gauge/flow are now MQTT clients of the confluence node on
            # red.lab rather than direct USB serial. Their topics are
            # derived from mqtt.node_name below (`{node_name}/{interface}`,
            # matching confluence's own convention — see
            # confluence/confluence_config.json's "Node Name" on red.lab)
            # in each subsystem's from_config(), not hardcoded per-section —
            # change the node name in one place, not three. A subsystem
            # section can still set its own topic_* key explicitly to
            # override the derived default (e.g. for a second node).
            "weir": {
                "command_timeout_s": 5.0,
                "home_offset_mm": 0.0,
            },
            "gauge": {
                # Index into the Massa interface's per-device arrays
                # (device names/dist_mm/signal_strength/...) — confluence
                # polls all configured Massa IDs in one message, so this
                # picks out which array element is this gauge's sensor.
                "sensor_index": 0,
                "offset_mm": 0.0,
            },
            "flow": {
                "command_timeout_s": 5.0,
                "C0": 4.902,
                "C1": 58.49,
                "C2": 0.08956,
            },
            "data": {
                "output_directory": "./data/",
                "compression": "gzip",
            },
            "storage": {
                "enabled": False,
                "type": "local",  # local, s3, sftp
            },
            "timing": {
                "checkpoint_file": "./experiment_checkpoint.json",
                "event_log": "./experiment_events.csv",
            },
            "gantry": {
                # pi_agent is the default: it launches gantry_agent.py over SSH
                # itself, is the sole owner of the serial port, and is the only
                # transport that supports topographic scanning. socket_bridge
                # was the previous default; it depended on serial_bridge.py, a
                # hand-started Pi-side script that has been retired — see
                # docs/MACRON_GANTRY.md.
                "transport": "pi_agent",  # pi_agent | ethernet | rs232 | socket_bridge (retired)
                "host": "red.dyn.ucr.edu",
                "bridge_port": 9700,  # only used by the retired socket_bridge transport
                "ssh_user": "oak",  # for the pi_agent transport
                "ssh_key": "~/.ssh/id_ed25519",
                "remote_serial_device": (
                    "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"
                ),
                "remote_baud": 9600,
                "group_index": 1,
                "safe_mode": True,
                "axes": [
                    # home_switch: "home" (default, INB1/3/5) or "limit"
                    # (INB2/4/6) — which switch HomingProcedure jogs toward
                    # and latches on. home_trip_on_high: whether that switch
                    # reads HIGH when triggered; confirmed LOW-on-trigger
                    # (normally-closed wiring) on this hardware 2026-08-25,
                    # so the default is False. See docs/MACRON_GANTRY.md.
                    #
                    # soft_negative_limit_mm / soft_positive_limit_mm:
                    # optional NLT/PLT overrides, written to the controller
                    # by connect() once safe_mode=False — see
                    # GantryController._apply_soft_limits(). Either bound
                    # can be omitted; an axis with neither is left at
                    # whatever the controller's own program has. No
                    # defaults are guessed here — set both once the real
                    # travel envelope is measured.
                    {"name": "X", "index": 1, "home_switch": "home", "home_trip_on_high": False},
                    {
                        "name": "Y", "index": 2, "brake_output": 4, "brake_status_input": 8,
                        "home_switch": "home", "home_trip_on_high": False,
                    },
                    # mm_per_unit override: confirmed 13.5 mm/unit on hardware
                    # 2026-08-10 (measured 90mm actual travel for a
                    # 100mm-commanded move), not the shared 15.0 default —
                    # see docs/archive/GANTRY_UNIT_CALIBRATION.md and
                    # MMCCommands.__init__'s axis_mm_per_unit docstring. X/Y
                    # were not re-measured this session and still use the
                    # shared default.
                    {
                        "name": "Z", "index": 5, "brake_output": 5, "mm_per_unit": 13.5,
                        "home_switch": "home", "home_trip_on_high": False,
                    },
                    {"name": "Theta", "index": 6},
                ],
                "homing": {
                    "speed_mm_s": 10.0,
                    "standoff_mm": 5.0,
                    "order": ["Z", "X", "Y"],
                },
                "fences": [],
                # TEMPORARY: 1 raw controller (ACP) unit measures as this many real
                # mm — default/fallback for any axis without its own override
                # in "axes" above (see Z's "mm_per_unit" key: confirmed off
                # from this shared value on 2026-08-10). Originally measured
                # 15.0 for X/Y/Z uniformly on 2026-07-28, see
                # docs/archive/GANTRY_UNIT_CALIBRATION.md — that "uniform
                # across axes" finding didn't hold up on Z under a second
                # measurement. Once the Snap2Motion/DSM project's axis scale
                # is fixed at the source, change this to 1.0, remove the
                # per-axis overrides, and update gantry_agent.py's matching
                # MM_PER_ACP_UNIT constant.
                "mm_per_acp_unit": 15.0,
                # Real-mm translation from the gantry's raw zero to a real-world
                # origin, applied on top of mm_per_acp_unit. Not a rotation/affine
                # matrix — nothing in this codebase needs axis rotation today.
                "coordinate_offset": {"X": 0.0, "Y": 0.0, "Z": 0.0, "Theta": 0.0},
            },
            "mqtt": {
                "broker_host": "red.lab",
                "broker_port": 1883,
                "client_id": "laguna",
                "keepalive": 60,
                "topics": [],
                "qos": 0,
                # Must match confluence_config.json's "Node Name" on red.lab —
                # weir/gauge/flow derive their MQTT topics from this in their
                # own from_config() (see the comment above the "weir" section).
                "node_name": "UCRS Confluence Node 1",
            },
            "rangefinder": {
                "topic": "laguna/od2000",
                "pdin_port": 1,       # IO-Link port the OD2000 is connected to (1-8)
                "offset_mm": 0.0,     # physical mounting offset if needed
                "al1342_host": None,  # AL1342 raw IP — required for activate()/read_mm()
                "calibration_file": None,  # path to a LinearCalibration CSV, see
                                            # scripts/calibrate_rangefinder.py
            },
        }
    
    def load_from_file(self, config_file: str) -> None:
        """Load configuration from a YAML file.

        Args:
            config_file: Path to the YAML configuration file.

        Raises:
            FileNotFoundError: If configuration file does not exist.
        """
        path = Path(config_file)
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_file}")

        with open(path, "r") as f:
            file_config = yaml.safe_load(f) or {}

        self.config_file = str(path.resolve())
        self.explicit_sections = set(file_config.keys())

        # Recursively merge loaded config with defaults
        self._merge_config(self.config_dict, file_config)
    
    @staticmethod
    def _merge_config(base: Dict[str, Any], override: Dict[str, Any]) -> None:
        """Merge override configuration into base configuration.

        Args:
            base: Base configuration dictionary (modified in place).
            override: Override configuration dictionary.
        """
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                Config._merge_config(base[key], value)
            else:
                base[key] = value
    
    def get(self, subsystem: str) -> Dict[str, Any]:
        """Get configuration for a specific subsystem.

        Args:
            subsystem: Name of the subsystem (e.g., 'robot', 'camera').

        Returns:
            Configuration dictionary for the subsystem.

        Raises:
            KeyError: If subsystem configuration not found.
        """
        if subsystem not in self.config_dict:
            raise KeyError(f"Configuration for subsystem '{subsystem}' not found")
        return cast(Dict[str, Any], self.config_dict[subsystem])
    
    def get_value(self, path: str, default: Any = None) -> Any:
        """Get a specific configuration value using dot notation.

        Args:
            path: Dot-separated path (e.g., 'robot.port').
            default: Default value if path not found.

        Returns:
            Configuration value or default if not found.
        """
        keys = path.split(".")
        value = self.config_dict
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
