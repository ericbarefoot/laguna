# Laguna - Robotic Flume Control System

Software suite for controlling robotic systems, acquiring camera data, managing water flow hardware, and processing experimental data from hydraulic flume experiments.

## Overview

Laguna provides a modular Python framework for coordinating multiple subsystems:

- **Robot Control**: ASCII serial and Modbus protocol support for robotic positioning
- **Camera Acquisition**: Real-time video capture and frame processing
- **Hydraulics Management**: Control and monitoring of hydraulic systems
- **Data Processing**: Acquisition, processing, and packaging of experimental data
- **Remote Storage**: Integration with cloud and remote storage solutions

## Quick Start

### Installation

```bash
# Clone the repository
git clone <repo-url>
cd laguna

# Create a virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in development mode
pip install -e ".[dev]"
```

### Basic Usage

```python
from laguna import FlumeLab

# Initialize the system with configuration
lab = FlumeLab(config_file="config/experiment.yaml")

from laguna.weir import SaflWeirController
from laguna.flow import SaflFlowController

# Attach hardware subsystems and run
lab.add(SaflWeirController(lab.config.get("weir")))
lab.add(SaflFlowController(lab.config.get("flow")))
lab.connect_all()
```

## Project Structure

```
laguna/
├── src/laguna/           # Main package
│   ├── core.py          # Main FlumeLab orchestrator
│   ├── config.py        # Configuration management
│   ├── robot/           # Robot control subsystem
│   ├── camera/          # Camera acquisition subsystem
│   ├── weir/            # Weir (tailgate) elevation control
│   ├── flow/            # Pump flow and solenoid control
│   ├── gauge/           # Water level sensing
│   ├── data/            # Data processing subsystem
│   └── storage/         # Remote storage interface
├── tests/               # Unit and integration tests
├── examples/            # Example scripts and workflows
├── docs/                # Documentation
└── pyproject.toml       # Project configuration
```

## Subsystems

### Robot Controller
Provides high-level interface for robotic positioning through multiple protocols:
- ASCII serial communication
- Modbus RTU/TCP
- Other industrial protocols

### Camera Acquisition
Real-time video capture and frame processing with optional compression and format conversion.

### Hydraulics System
Monitoring and control of hydraulic pressure, flow rates, and actuator positions.

### Data Processing
Aggregation, filtering, and packaging of data from multiple sensors and systems.

### Remote Storage
Interface for cloud storage (AWS S3, Google Cloud) and remote storage solutions (SSH/SFTP).

## Development

### Running Tests

```bash
pytest                          # Run all tests
pytest --cov                   # Run with coverage report
pytest tests/robot/            # Run specific test module
```

### Code Style

This project uses Black for formatting and isort for import sorting:

```bash
black src/ tests/
isort src/ tests/
```

## Configuration

Configuration is managed through YAML files. Example files are provided in `config/examples/`.

## Documentation

For detailed documentation, see the [docs](docs/) directory.

## License

MIT License - See LICENSE file for details
