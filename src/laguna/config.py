"""Configuration management for the Laguna system.

This module handles loading and managing configuration for all subsystems.
Configuration can be loaded from YAML files or passed as dictionaries.
"""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional


class Config:
    """Central configuration manager for all subsystems.
    
    Attributes:
        config_dict: Dictionary containing all configuration parameters
    """
    
    def __init__(self, config_file: Optional[str] = None, defaults: Optional[Dict[str, Any]] = None):
        """Initialize configuration.
        
        Args:
            config_file: Path to YAML configuration file
            defaults: Default configuration dictionary
        """
        self.config_dict = defaults or self._get_defaults()
        
        if config_file:
            self.load_from_file(config_file)
    
    @staticmethod
    def _get_defaults() -> Dict[str, Any]:
        """Get default configuration values.
        
        Returns:
            Dictionary with default configuration
        """
        return {
            "robot": {
                "port": "/dev/ttyUSB0",
                "baudrate": 9600,
                "protocol": "modbus",
                "timeout": 5.0,
            },
            "camera": {
                "device_id": 0,
                "fps": 30,
                "resolution": (1920, 1080),
                "capture_format": "BGR",
            },
            "weir": {
                "port": "/dev/ttyUSB0",
                "baudrate": 9600,
                "steps_per_mm": 1000.0,
                "home_offset_mm": 0.0,
            },
            "gauge": {
                "port": "/dev/ttyUSB2",
                "sensor_ids": [0],
                "offset_mm": 0.0,
            },
            "flow": {
                "vfd_port": "/dev/ttyUSB1",
                "vfd_slave_id": 1,
                "motor_port": "/dev/ttyUSB0",
                "motor_baudrate": 9600,
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
        }
    
    def load_from_file(self, config_file: str) -> None:
        """Load configuration from a YAML file.
        
        Args:
            config_file: Path to the YAML configuration file
            
        Raises:
            FileNotFoundError: If configuration file does not exist
        """
        path = Path(config_file)
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_file}")
        
        with open(path, "r") as f:
            file_config = yaml.safe_load(f) or {}
        
        # Recursively merge loaded config with defaults
        self._merge_config(self.config_dict, file_config)
    
    @staticmethod
    def _merge_config(base: Dict[str, Any], override: Dict[str, Any]) -> None:
        """Merge override configuration into base configuration.
        
        Args:
            base: Base configuration dictionary (modified in place)
            override: Override configuration dictionary
        """
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                Config._merge_config(base[key], value)
            else:
                base[key] = value
    
    def get(self, subsystem: str) -> Dict[str, Any]:
        """Get configuration for a specific subsystem.
        
        Args:
            subsystem: Name of the subsystem (e.g., 'robot', 'camera')
            
        Returns:
            Configuration dictionary for the subsystem
            
        Raises:
            KeyError: If subsystem configuration not found
        """
        if subsystem not in self.config_dict:
            raise KeyError(f"Configuration for subsystem '{subsystem}' not found")
        return self.config_dict[subsystem]
    
    def get_value(self, path: str, default: Any = None) -> Any:
        """Get a specific configuration value using dot notation.
        
        Args:
            path: Dot-separated path (e.g., 'robot.port')
            default: Default value if path not found
            
        Returns:
            Configuration value or default if not found
        """
        keys = path.split(".")
        value = self.config_dict
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
