# Getting Started with Laguna

Welcome to the Laguna project! This guide will help you get started with the robotic flume control system.

## What You Have

A complete, professional Python package structure with:

✅ **Modular Architecture**
- Separate subsystems for robot, camera, hydraulics, data, and storage
- Central orchestrator (`FlumeLab`) that coordinates everything
- Easy to extend with new subsystems

✅ **Modern Python Packaging**
- `pyproject.toml` for project configuration
- `src/` layout (industry standard)
- Ready for pip installation

✅ **Configuration Management**
- YAML-based configuration system
- Hierarchical defaults + overrides
- Easy to switch between experiments

✅ **Extensible Protocol Support**
- Multiple robot communication protocols (Modbus, ASCII serial, extensible)
- Multiple storage backends (local, S3, SFTP, extensible)
- Abstract base classes for easy plugin architecture

✅ **Professional Development Setup**
- Comprehensive docstrings (Google style)
- Unit test framework (pytest)
- Code style tools (Black, isort, mypy)
- Git repository initialized

✅ **Documentation**
- Architecture guide explaining design patterns
- Quick reference for common tasks
- Contributing guidelines
- Three example scripts

## Next Steps

### 1. Install and Verify
```bash
cd /Users/eric/Library/CloudStorage/Dropbox/software/laguna

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install in development mode
pip install -e ".[dev]"

# Verify installation
python -c "from laguna import FlumeLab; print('✓ Installation successful')"
```

### 2. Run Examples
```bash
# These work immediately without hardware
python examples/example_01_basic_init.py
python examples/example_02_experiment.py
```

### 3. Implement Hardware Communication

This is where your domain expertise comes in. The structure is ready; you need to fill in the hardware communication:

**Start with the robot controller** (`src/laguna/robot/__init__.py`):
- Line ~160: Implement `ModbusProtocol.connect()` and `send_command()`
- Line ~180: Implement `AsciiProtocol` for your specific robot
- Use `pymodbus` or `pyserial` libraries (already in dependencies)

Then similar work for hydraulics, camera, etc.

### 4. Create Your Experiments

Once hardware is connected, create experiment scripts:

```python
# my_experiment.py
from laguna import FlumeLab

lab = FlumeLab(config_file="config/flume_setup.yaml")

# Define your experiment procedure
experiment_config = {
    "robot": {"start_position": (0, 0, 0)},
    "hydraulics": {"pressure_target": 1000},
    "camera": {"fps": 30}
}

# Run it
lab.run_experiment(experiment_config)
```

### 5. Extend the System

Need more subsystems? Follow the pattern:

1. Create `src/laguna/new_subsystem/__init__.py`
2. Implement your class following existing patterns
3. Add config defaults to `config.py`
4. Integrate into `FlumeLab` class in `core.py`
5. Add tests in `tests/test_new_subsystem.py`

See `docs/ARCHITECTURE.md` for detailed instructions.

## File Organization Reference

### For Hardware Implementation
- **Robot protocols**: `src/laguna/robot/__init__.py` (lines 160-200)
- **Camera driver**: `src/laguna/camera/__init__.py` (line ~55)
- **Hydraulics interface**: `src/laguna/hydraulics/__init__.py` (line ~50)
- **Serial communication**: `src/laguna/data/__init__.py` for data logging

### For Configuration
- **Default settings**: `src/laguna/config.py` lines ~40-60
- **Example config**: `config/example_config.yaml`
- **Loading mechanism**: `src/laguna/config.py` class `Config`

### For Testing
- **Test templates**: `tests/test_*.py`
- **Run tests**: `pytest` or `pytest --cov`

### For Documentation
- **Architecture details**: `docs/ARCHITECTURE.md`
- **Quick reference**: `docs/QUICKREF.md`
- **Contributing guide**: `CONTRIBUTING.md`

## Key Design Principles (For Reference)

1. **One subsystem = one module**
   - Clear separation of concerns
   - Easy to test independently
   - Can work on different subsystems in parallel

2. **Configuration over code**
   - Change behavior with YAML, not code edits
   - Repeatability for different experimental setups

3. **Factory pattern for extensibility**
   - Add new protocols/backends without modifying existing code
   - Example: Add new robot protocol → implement `ProtocolHandler` subclass → register in factory

4. **Uniform interfaces**
   - All subsystems follow similar patterns
   - Reduces cognitive load when switching between them

5. **Logging everywhere**
   - Helps debug issues in lab environment
   - Understanding what the system is doing

## Common Commands You'll Use

```bash
# Development
pip install -e ".[dev]"                    # Install with dev tools
black src/ tests/ examples/                # Format code
pytest                                     # Run tests
pytest --cov                              # Test coverage

# Configuration
# Edit: config/your_experiment.yaml
# Use: FlumeLab(config_file="config/your_experiment.yaml")

# Git
git status                                 # Check changes
git add .                                  # Stage changes
git commit -m "Description"                # Commit
git log                                    # View history
```

## Troubleshooting

**"No module named laguna"**
- Did you run `pip install -e "."` in the environment?
- Is the venv activated? (`which python` should show venv path)

**Tests fail to import**
- Run from project root: `cd laguna`
- Make sure in venv: `which pytest` should show venv path

**Can't connect to hardware**
- This is normal - protocols are stubs for now
- Follow the TODO comments to implement real hardware communication
- Examples work without hardware to verify setup

**Need to understand a module**
- Read docstrings: `python -c "import laguna.robot; help(laguna.robot.RobotController)"`
- Check examples in `examples/`
- Review architecture guide: `docs/ARCHITECTURE.md`

## Support Resources

Within the project:
- **How the system works**: `docs/ARCHITECTURE.md`
- **How to use it**: `docs/QUICKREF.md` and examples
- **How to extend it**: `CONTRIBUTING.md` and architecture guide
- **Code reference**: Docstrings in each module

Your expertise:
- You know your hardware and experimental procedures
- Fill in the hardware-specific parts (protocols, drivers)
- Create experiment scripts that use this framework

## You're Ready!

The foundation is solid and ready for you to build on. The hard part (package structure, configuration, testing framework) is done. Now it's about:

1. Implementing your specific hardware communication
2. Creating your experiment procedures
3. Tuning and extending as needed

Start with implementing one protocol (e.g., Modbus), verify it with a simple test, then move to the next component.

Good luck! 🚀
