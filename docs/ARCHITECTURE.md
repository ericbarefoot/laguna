# Laguna Architecture Guide

This document provides an overview of the Laguna software architecture and how to extend it.

## System Architecture

Laguna uses a **modular subsystem architecture** where each major hardware/functional component has its own module. A central orchestrator (`FlumeLab`) coordinates all subsystems.

```
┌─────────────────────────────────────────┐
│          FlumeLab (core.py)             │
│     Main experiment orchestrator        │
└────────────┬────────────────────────────┘
             │
    ┌────────┼────────┬────────────┬──────────────┐
    │        │        │            │              │
    ▼        ▼        ▼            ▼              ▼
┌────────┐ ┌──────────┐ ┌────────────┐ ┌─────────┐ ┌────────────┐
│ Robot  │ │ Camera   │ │ Hydraulics │ │  Data   │ │  Storage   │
│Control │ │Acquisition│ │  System    │ │Processor│ │ Interface  │
└────────┘ └──────────┘ └────────────┘ └─────────┘ └────────────┘
     │          │             │            │             │
     └──────────┴─────────────┴────────────┴─────────────┘
                  Configuration (config.py)
```

## Module Breakdown

### `src/laguna/`

**`__init__.py`**
- Main package entry point
- Exports `FlumeLab` for public API

**`core.py`**
- `FlumeLab` class: Main orchestrator that coordinates all subsystems
- Entry point for all experiments
- Manages system lifecycle (connect, initialize, run, disconnect)
- Example usage:
  ```python
  lab = FlumeLab()
  lab.run_experiment(config)
  ```

**`config.py`**
- `Config` class: Manages system-wide configuration
- Loads YAML configuration files
- Merges with defaults
- Provides clean API for accessing settings
- Supports dot notation for nested values

### `src/laguna/robot/`
- `RobotController`: Main interface for robot positioning
- Supports multiple protocols via abstract `ProtocolHandler`:
  - `ModbusProtocol`: Modbus RTU/TCP communication
  - `AsciiProtocol`: ASCII serial commands
- Methods: `connect()`, `disconnect()`, `move_to()`, `home()`, `get_position()`, `stop()`

### `src/laguna/camera/`
- `CameraAcquisition`: Real-time video capture interface
- Methods: `start()`, `stop()`, `get_frame()`, `start_recording()`, `stop_recording()`
- Handles frame format conversion and optional compression

### `src/laguna/weir/`
- `WeirController` (ABC) / `SaflWeirController`: Tailgate elevation via Teknic ClearCore stepper motor
- Methods: `connect()`, `disconnect()`, `set_elevation(mm)`, `get_elevation()`, `set_velocity(mm_per_sec)`, `enable()`, `disable()`, `wait_for_move(timeout)`, `home()`, `stop()`, `clear_faults()`, `get_status()`

### `src/laguna/flow/`
- `FlowController` (ABC) / `SaflFlowController`: Pump flow via Fuji VFD, solenoid valves via motor IO pins
- Methods: `connect()`, `disconnect()`, `set_flowrate(lpm)`, `get_flowrate()`, `start()`, `stop()`, `clear_faults()`, `get_status()`
- Properties: `qin`, `qaux` (solenoid open/close)

### `src/laguna/gauge/`
- `WaterLevelSensor` (ABC) / `SaflWaterLevelSensor`: Water surface elevation via Massa ultrasonic sensor
- Methods: `connect()`, `disconnect()`, `read_mm()`, `read_mm_smoothed()`, `get_status()`

### `src/laguna/data/`
- `DataProcessor`: Data aggregation, processing, and packaging
- Methods: `add_data_point()`, `process_data()`, `save_data()`, `export_data()`
- Buffer management with size tracking
- Export formats: CSV, HDF5, JSON (extensible)

### `src/laguna/storage/`
- `RemoteStorage`: Abstract interface for cloud/remote storage
- Supports multiple backends:
  - `S3Backend`: AWS S3 cloud storage
  - `SFTPBackend`: Remote SSH/SFTP storage
  - `LocalBackend`: Local filesystem
- Methods: `connect()`, `disconnect()`, `upload_file()`, `download_file()`, `list_files()`

## Design Patterns

### 1. **Factory Pattern**
Subsystem modules use factory methods to select implementations:
```python
protocol = RobotController._get_protocol(config)  # Returns ModbusProtocol or AsciiProtocol
backend = RemoteStorage._get_backend()            # Returns S3Backend, SFTPBackend, etc.
```

### 2. **Abstract Base Classes**
Protocol handlers and storage backends use ABC for extensibility:
```python
class ProtocolHandler(ABC):
    @abstractmethod
    def connect(self) -> bool: pass
    
class StorageBackend(ABC):
    @abstractmethod
    def upload_file(self, local: str, remote: str) -> None: pass
```

### 3. **Configuration Over Code**
All subsystems configured via YAML, not hardcoded:
```yaml
robot:
  protocol: "modbus"
  port: "/dev/ttyUSB0"
```

### 4. **Uniform Interface**
All subsystems follow similar patterns:
- Constructor takes `config` dict
- `connect()` / `disconnect()` or `start()` / `stop()`
- Status methods (e.g., `get_status()`)
- Error logging throughout

## Adding New Subsystems

To add a new subsystem (e.g., environmental sensors):

### 1. Create Module Structure
```
src/laguna/sensors/
├── __init__.py
└── sensor_class.py  # Optional if all code fits in __init__.py
```

### 2. Implement Main Class
```python
# src/laguna/sensors/__init__.py
class EnvironmentalSensor:
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.is_connected = False
    
    def connect(self) -> bool:
        # Implement connection logic
        pass
    
    def disconnect(self) -> None:
        # Implement disconnection
        pass
    
    def get_temperature(self) -> float:
        # Get sensor data
        pass
```

### 3. Add to Config Defaults
```python
# In config.py _get_defaults()
"sensors": {
    "port": "/dev/ttyUSB2",
    "baudrate": 9600,
}
```

### 4. Integrate into FlumeLab
```python
# In core.py
class FlumeLab:
    def __init__(self, config_file: Optional[str] = None):
        # ... existing code ...
        self.sensors = EnvironmentalSensor(self.config.get("sensors"))
    
    def connect_all(self) -> bool:
        # ... existing connections ...
        if not self.sensors.connect():
            return False
        # ... rest of method ...
```

### 5. Add Tests
```python
# tests/test_sensors.py
class TestEnvironmentalSensor:
    def test_initialization(self):
        sensor = EnvironmentalSensor(config)
        assert not sensor.is_connected
```

## Communication Protocols

### Adding New Robot Protocol

1. Subclass `ProtocolHandler` in `robot/__init__.py`:
```python
class CANBusProtocol(ProtocolHandler):
    def connect(self) -> bool: ...
    def send_command(self, command: str, params: Dict) -> None: ...
    def read_position(self) -> Tuple[float, float, float]: ...
```

2. Register in `RobotController._get_protocol()`:
```python
elif protocol_type == "canbus":
    return CANBusProtocol(config)
```

3. Update config example:
```yaml
robot:
  protocol: "canbus"
  can_interface: "can0"
```

## Configuration System

### Hierarchy
1. Defaults (hardcoded in `_get_defaults()`)
2. YAML file (if provided)
3. Runtime overrides

### Accessing Configuration
```python
# Get entire subsystem config
robot_config = config.get("robot")

# Get specific value with dot notation
port = config.get_value("robot.port")

# Get with default
value = config.get_value("robot.timeout", default=5.0)
```

## Testing Strategy

### Unit Tests
- Test each subsystem in isolation (mocked connections)
- Located in `tests/test_*.py`
- Run with: `pytest`

### Integration Tests
- Test FlumeLab orchestration
- Test configuration loading
- Test error handling

### Test Coverage
- Aim for >80% coverage
- Check with: `pytest --cov=src/laguna`

## Logging

All modules use Python's standard `logging` module:
```python
import logging
logger = logging.getLogger(__name__)

logger.info("Robot connected")
logger.warning("Pressure exceeds threshold")
logger.error("Failed to read sensor data")
```

Main logging configured in `core.py` at module load.

## Performance Considerations

### Threading
- Current implementation is synchronous
- For real-time requirements, consider:
  - Separate threads for I/O operations
  - Queue-based communication between subsystems
  - Use `threading` or `asyncio`

### Data Buffering
- `DataProcessor` buffers data in memory
- For long experiments, consider:
  - Streaming to disk
  - Periodic flush to storage
  - Memory-mapped files for large datasets

## Error Handling

### Design Principles
- All network operations wrapped in try/except
- Errors logged with context
- Methods return bool for success/failure
- Critical errors don't crash entire system
- Emergency stop available (`FlumeLab.emergency_stop()`)

### Recovery
- Reconnect on communication failure
- Graceful degradation (continue with connected subsystems)
- User notification via logging

## Next Steps

1. **Implement Protocol Handlers**: Fill in TODO sections for actual hardware communication
2. **Add Hardware Drivers**: Implement real serial/Modbus communication
3. **Create Calibration Module**: Store/apply sensor calibration factors
4. **Expand Data Processing**: Add filtering, interpolation, statistical analysis
5. **Add GUI**: Simple interface for experiment control
6. **Cloud Integration**: Full S3/cloud storage implementation
