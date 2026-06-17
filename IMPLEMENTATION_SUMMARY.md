# Implementation Summary: Weir-Gauge-Camera Experiment with Non-Blocking Scheduler

## Problem Statement

You wanted to assemble an experiment using three subsystems (Weir, Gauge, DSLR Cameras) with the ability to:
1. Log water level periodically
2. Move the weir smoothly following a time-elevation schedule
3. Capture timelapse images from two Canon DSLRs every minute
4. **Run non-blocking** so a REPL can issue `lab.stop()` at any time to pause everything

The key challenge was that the existing `scheduler.run()` method blocks the foreground thread, freezing a REPL until the experiment ends.

## Solution Overview

### 1. Non-Blocking Scheduler (`scheduler.run_async()`)

**File:** `src/laguna/timing/scheduler.py`

Added a new method that wraps `run()` in a daemon thread:

```python
def run_async(self, duration: float) -> threading.Thread:
    """Run the scheduler in a background daemon thread."""
    t = threading.Thread(target=self.run, args=(duration,), daemon=True, name="scheduler-main")
    t.start()
    return t
```

The existing `stop()` method already works cross-thread (sets `_stop_event`), so no internal changes were needed.

**Usage:**
```python
with lab.experiment() as clock:
    scheduler_thread = lab.scheduler.run_async(duration=3600)
    # Foreground is now free — REPL stays responsive
```

### 2. Pause / Resume Control (`lab.stop()` and `lab.resume()`)

**File:** `src/laguna/core.py`

Added two new FlumeLab methods:

```python
def stop(self) -> None:
    """Pause experiment: stop scheduler + pause clock + stop weir."""
    self.scheduler.stop()  # Sets stop event, pauses clock internally
    weir = self._subsystems.get("weir")
    if weir and hasattr(weir, "stop"):
        weir.stop()  # Abort any in-progress motor move

def resume(self, remaining_s: float) -> threading.Thread:
    """Resume after stop(): restart scheduler loop in background."""
    return self.scheduler.run_async(remaining_s)
```

Unlike `emergency_stop()`, these methods:
- Do NOT disconnect hardware
- Do NOT close the event log
- Preserve clock state for resumption

**Usage:**
```python
lab.stop()           # Pause everything
lab.resume(60)       # Resume for 60 more experiment-seconds
```

Also fixed a pre-existing bug: added missing `from pathlib import Path` import (was causing `NameError` in `open_ocean_control_gui`).

### 3. DSLR Camera Integration (`DslrCameraSubsystem`)

**File:** `src/laguna/camera/dslr.py` (new)

Created a laguna subsystem wrapper around Minsik's `dualcam-timelapse` package:

```python
class DslrCameraSubsystem:
    subsystem_name = "dslr_cameras"
    
    def __init__(self, config_path: str, dualcam_path: Optional[str] = None):
        # Lazy imports dualcam to avoid hard dependency on gphoto2
    
    def connect(self) -> bool:
        # Load YAML config, connect to both cameras
    
    def capture_all(self) -> Dict[str, Optional[Path]]:
        # Trigger parallel capture via dualcam.CameraManager.capture_all_parallel()
    
    def get_status(self) -> Dict[str, Any]:
        # Return connection state and camera info
```

**Key design decisions:**

1. **Lazy import** — `dualcam` is only imported on `connect()`, so you get a clear error message on unsupported platforms (macOS, Windows) rather than a silent failure at startup.

2. **No fork of dualcam** — Zero modifications to Minsik's code. We just wrap his `CameraManager` and add `subsystem_name` + lifecycle methods.

3. **Path injection** — The `dualcam_path` argument allows the code to find dualcam even if it's not in `sys.path` by default.

4. **Thread-safe** — `capture_all()` wraps `capture_all_parallel()`, which uses `ThreadPoolExecutor` internally. Suitable for scheduling from the daemon thread.

### 4. Experiment Orchestration Script

**File:** `experiments/weir_gauge_camera_experiment.py` (new)

The main script demonstrating all three subsystems working together:

```python
def main(
    schedule_file: str,
    dslr_config: str,
    dualcam_path: Optional[str] = None,
    lab_config: Optional[str] = None,
    duration: float = 3600.0,
    interactive: bool = False,
) -> None:
```

**Three scheduled actions:**

1. **Gauge logging (every 5s):**
   ```python
   elev_mm = gauge.read_mm_smoothed()  # Fetch water level
   lab.event_log.log(...)  # Log to CSV
   ```

2. **Weir control (every 10s):**
   ```python
   target_mm = schedule.weir_elevation(lab.clock.elapsed())  # Spline interpolation
   weir.set_elevation(target_mm)  # Non-blocking command
   ```

3. **Camera timelapse (every 60s):**
   ```python
   dslr.capture_all()  # Parallel dual-camera trigger
   ```

**Two operating modes:**

- **Foreground mode** (default): `scheduler.run()` blocks until duration expires
- **Interactive mode** (`--interactive`): `scheduler.run_async()` leaves REPL live; you can `lab.stop()` / `lab.resume()` interactively

**Optional:** `main_interactive()` function for Jupyter notebook use:
```python
lab, thread = main_interactive(...)
# In notebook: lab.stop(), lab.resume(60), etc.
```

### 5. Supporting Files

**File:** `examples/example_schedule.csv`

A sample schedule CSV with keyframes for weir elevation:
```csv
time_s,weir_elevation_mm,pump_flow_lpm,qin_open,qaux_open
0,100,0.0,1,0
60,90,0.0,1,0
...
300,50,0.0,1,0
```

The `ExperimentSchedule` class (existing in laguna) uses cubic spline interpolation on the `weir_elevation_mm` column for smooth motion.

**File:** `WEIR_GAUGE_CAMERA_SETUP.md`

Comprehensive setup and usage guide covering:
- Prerequisites and installation
- Configuration for weir, gauge, DSLR
- Running foreground vs. interactive mode
- Troubleshooting common issues
- Advanced scheduling examples

## Integration with Existing Code

### No Breaking Changes

All changes are **backwards compatible**:

- `scheduler.run()` continues to work as before (foreground blocking)
- `lab.emergency_stop()` is unchanged
- New methods (`run_async()`, `stop()`, `resume()`) are pure additions
- `DslrCameraSubsystem` is opt-in via `lab.add()`

### Reuses Existing Patterns

- Schedule interpolation: leverages `ExperimentSchedule.from_csv()` (already in laguna)
- Clock/timing: uses existing `ExperimentClock`, `Scheduler`, `EventLog`
- Subsystem registration: follows opt-in model via `lab.add(subsystem_name)`
- Hardware drivers: relies on existing `SaflWeirController`, `SaflWaterLevelSensor`

## How It Enables Your Workflow

### On a Linux Lab Machine

```bash
# Discover camera USB ports
python dualcam/discover.py

# Create cameras.yaml with those ports

# Run the experiment (interactive mode)
python experiments/weir_gauge_camera_experiment.py \
    --schedule my_schedule.csv \
    --dslr-config cameras.yaml \
    --dualcam-path /path/to/dualcam-timelapse \
    --interactive
```

While running, in another terminal:

```python
>>> from laguna import FlumeLab
>>> lab = ...  # (somehow access the running instance)
>>> lab.stop()       # Pause clock + weir + cameras
>>> lab.resume(60)   # Resume for 60 more seconds
```

Or from Jupyter:

```python
lab, thread = main_interactive(...)
# Cells in notebook can now call lab.stop(), lab.get_system_status(), etc.
# while the experiment runs in the background
```

### On a macOS Dev Machine

The DSLR subsystem will fail to import (gphoto2 is Linux-only), but that's expected and handled gracefully:

```
WARNING: DSLR config not found at ... — skipping DSLR setup
```

You can still test the weir + gauge logic with a mock schedule.

## Testing

### Unit Test Ideas

1. **Scheduler async:** Verify `run_async()` returns immediately and scheduler runs in background
2. **Stop/resume:** Check that `stop()` pauses clock and `resume()` resumes at the right time
3. **DSLR import guards:** Ensure `DslrCameraSubsystem` raises clear errors on import failure
4. **Schedule interpolation:** Verify weir elevation follows the spline correctly

### Integration Test Ideas

1. Run the script for 60s with mocked hardware, check `experiment_events.csv` for expected log entries
2. Pause at 30s with `lab.stop()`, resume with `lab.resume(30)`, verify total runtime = 60s
3. Load a small schedule, confirm weir setpoints track the spline

### Manual Testing (Lab Machine)

```bash
# Small test: 60 seconds, one capture
python experiments/weir_gauge_camera_experiment.py \
    --schedule examples/example_schedule.csv \
    --dslr-config cameras.yaml \
    --dualcam-path /path/to/dualcam-timelapse \
    --duration 60
```

Check `experiment_events.csv` for:
- Gauge reads at t=5, 10, 15, ...
- Weir updates at t=10, 20, 30, ...
- Camera captures at t=60

## Files Changed

| File | Action | Purpose |
|---|---|---|
| `src/laguna/timing/scheduler.py` | Edit | Add `run_async()` method |
| `src/laguna/core.py` | Edit | Add `stop()`, `resume()`, fix `Path` import |
| `src/laguna/camera/__init__.py` | Edit | Export `DslrCameraSubsystem` |
| `src/laguna/camera/dslr.py` | Create | DSLR subsystem wrapper |
| `experiments/weir_gauge_camera_experiment.py` | Create | Main experiment script |
| `examples/example_schedule.csv` | Create | Sample schedule |
| `WEIR_GAUGE_CAMERA_SETUP.md` | Create | Setup & usage guide |
| `IMPLEMENTATION_SUMMARY.md` | Create | This file |

## Next Steps

1. **Install dualcam-timelapse** on the lab machine (sibling directory)
2. **Configure cameras.yaml** with your USB port discoveries
3. **Test the weir + gauge** with a short 60s run
4. **Run the full experiment** with your real schedule

Minsik's code requires zero changes — we just import and wrap it. The non-blocking scheduler gives you full REPL control while the experiment runs.
