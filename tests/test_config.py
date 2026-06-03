"""Test suite for configuration management."""

import pytest
import tempfile
from pathlib import Path
from laguna.config import Config


class TestConfig:
    """Test cases for Config class."""
    
    def test_default_config(self):
        """Test loading default configuration."""
        config = Config()
        
        assert "robot" in config.config_dict
        assert "camera" in config.config_dict
        assert "weir" in config.config_dict
        assert "gauge" in config.config_dict
        assert "flow" in config.config_dict
        assert "data" in config.config_dict
        assert "storage" in config.config_dict
    
    def test_get_subsystem(self):
        """Test retrieving subsystem configuration."""
        config = Config()
        robot_config = config.get("robot")
        
        assert "port" in robot_config
        assert "protocol" in robot_config
    
    def test_get_value(self):
        """Test getting specific configuration value."""
        config = Config()
        
        port = config.get_value("robot.port")
        assert port == "/dev/ttyUSB0"
    
    def test_get_value_default(self):
        """Test default value for missing config."""
        config = Config()
        
        value = config.get_value("nonexistent.value", default="not_found")
        assert value == "not_found"
