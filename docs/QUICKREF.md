# Quick Reference Guide

## Installation

```bash
cd laguna
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
```

## Basic Usage

### Simplest Example
```python
from laguna import FlumeLab

lab = FlumeLab()
lab.run_experiment()
```

### With Configuration
```python
from laguna import FlumeLab
from laguna.weir import SaflWeirController
from laguna.flow import SaflFlowController

lab = FlumeLab(config_file="config/my_experiment.yaml")
lab.add(SaflWeirController(lab.config.get("weir")))
lab.add(SaflFlowController(lab.config.get("flow")))
lab.connect_all()
```

### Manual Control
```python
from laguna import FlumeLab

lab = FlumeLab()
lab.robot.connect()
lab.robot.move_to((100, 100, 50))
pos = lab.robot.get_position()
lab.robot.disconnect()
```

## Configuration Basics

### Default Config Locations
- Hardcoded defaults: `src/laguna/config.py` → `Config._get_defaults()`
- Custom config: Any YAML file passed to `FlumeLab(config_file="path")`

### Common Settings
```yaml
robot:
  port: "/dev/ttyUSB0"
  protocol: "modbus"

camera:
  fps: 30
  resolution: [1920, 1080]

weir:
  port: "/dev/ttyUSB0"
  steps_per_mm: 1000.0

flow:
  vfd_port: "/dev/ttyUSB1"
  motor_port: "/dev/ttyUSB0"

gauge:
  port: "/dev/ttyUSB2"
  offset_mm: 0.0

data:
  output_directory: "./data/"
```

## Common Tasks

### Check System Status
```python
status = lab.get_system_status()
print(status)
```

### Emergency Stop
```python
lab.emergency_stop()  # Immediately stop all systems
```

### Access Individual Subsystem
```python
# Robot
lab.robot.move_to((x, y, z))
lab.robot.get_position()

# Camera  
lab.camera.start()
frame = lab.camera.get_frame()

# Weir
lab.weir.set_elevation(200.0)       # mm
lab.weir.set_velocity(5.0)          # mm/s
lab.weir.wait_for_move()

# Flow
lab.flow.start()
lab.flow.set_flowrate(20.0)         # L/min
lab.flow.qin = True                 # open solenoid

# Gauge
elevation = lab.gauge.read_mm()
elevation_smooth = lab.gauge.read_mm_smoothed()

# Data
lab.data_processor.add_data_point({"temp": 25.5})
lab.data_processor.save_data("output.csv")

# Storage
lab.storage.upload_file("local.csv", "remote.csv")
```

## Running Tests
```bash
pytest                              # All tests
pytest tests/test_robot.py         # Specific test
pytest --cov                       # With coverage
pytest -v                          # Verbose
```

## Development Commands
```bash
black src/ tests/                  # Format code
isort src/ tests/                  # Sort imports
mypy src/laguna                    # Type checking
pytest --cov                       # Test coverage
```

## File Structure Quick Map
```
laguna/
├── pyproject.toml          ← Package config & dependencies
├── README.md               ← Project overview
├── src/laguna/
│   ├── __init__.py         ← Exports FlumeLab
│   ├── core.py             ← Main FlumeLab class
│   ├── config.py           ← Configuration management
│   ├── robot/              ← Robot control subsystem
│   ├── camera/             ← Camera acquisition subsystem
│   ├── weir/               ← Weir elevation control
│   ├── flow/               ← Pump flow and solenoid control
│   ├── gauge/              ← Water level sensing
│   ├── data/               ← Data processing subsystem
│   └── storage/            ← Remote storage interface
├── tests/                  ← Unit tests
├── examples/               ← Example scripts
├── config/                 ← Configuration templates
└── docs/                   ← Documentation
    └── ARCHITECTURE.md     ← Detailed architecture guide
```

## Common Errors

### "Module not found"
```bash
# Make sure you installed in development mode
pip install -e "."
```

### "Failed to connect to robot"
- Check serial port: `ls /dev/tty*`
- Verify settings in config match your hardware
- Try example: `python examples/example_03_subsystem_control.py`

### Port Permission Denied (Linux/Mac)
```bash
# Grant permissions to serial port
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
```

## Next: What to Implement

For each subsystem, the TODO sections show what needs implementation:

1. **Robot control** (`src/laguna/robot/__init__.py`):
   - Implement `ModbusProtocol.send_command()` with pymodbus
   - Implement `AsciiProtocol` with pyserial

2. **Camera** (`src/laguna/camera/__init__.py`):
   - Implement actual OpenCV camera initialization
   - Implement `get_frame()` with format conversion

3. **Weir / Flow / Gauge** — wired to `safl_ocean_hardware`; see `src/laguna/weir/`, `flow/`, `gauge/`

4. **Data processing** (`src/laguna/data/__init__.py`):
   - Implement actual HDF5/CSV export
   - Add data filtering and calibration

5. **Storage** (`src/laguna/storage/__init__.py`):
   - Implement boto3 for S3 backend
   - Implement paramiko for SFTP backend

## Documentation

- **Architecture**: `docs/ARCHITECTURE.md` - Detailed system design
- **Contributing**: `CONTRIBUTING.md` - How to add features
- **README**: `README.md` - Project overview
- **Examples**: `examples/` - Working code samples

## Getting Help

1. Check the examples in `examples/`
2. Read subsystem docstrings: `help(lab.robot)`
3. Review test files for usage patterns
4. See architecture guide: `docs/ARCHITECTURE.md`
