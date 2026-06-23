# Weir-Gauge-Camera Experiment Setup

This guide walks through setting up and running the coordinated weir-gauge-camera experiment with non-blocking scheduler operation.

## Architecture Overview

The experiment orchestrates three subsystems running on a coordinated clock:

1. **Gauge (Water Level Sensor)** — Logs water surface elevation via Massa ultrasonic sensor
2. **Weir (Elevation Control)** — Stepper motor controlling flume weir position, following a time-elevation schedule
3. **DSLR Cameras (Canon)** — Two USB-attached Canon Rebel T7 cameras capturing timelapse images via `gphoto2`

All three are coordinated by a **non-blocking scheduler** that runs in a background daemon thread, leaving the foreground free for interactive REPL control.

## Prerequisites

### Laguna

Install the laguna package from the repo root:

```bash
pip install -e .
```

This installs `laguna`, `laguna.schedule`, `laguna.timing`, `laguna.camera`, `laguna.weir`, `laguna.gauge`, and core modules.

### Required Dependencies

```bash
pip install numpy scipy pandas pyyaml
```

### DSLR Camera Support (Linux Lab Machine Only)

The DSLR integration requires Minsik's `dualcam-timelapse` package:

1. **Clone the repo as a sibling to laguna:**

   ```bash
   cd /path/to/repos
   git clone https://github.com/yukms/dualcam-timelapse.git
   ```

2. **Install system dependencies (Linux):**

   ```bash
   sudo apt install libgphoto2-dev gphoto2
   ```

3. **Install Python dependencies:**

   ```bash
   cd dualcam-timelapse
   pip install -r requirements.txt  # gphoto2, PyYAML
   ```

4. **Discover camera USB ports (required for config):**

   ```bash
   python dualcam/discover.py
   ```

   This outputs something like:

   ```
   Camera: Canon Rebel T7 - (Usb 001,005)
   usb:001,005
   ```

### Hardware Subsystem Drivers

For the weir and gauge to work, you also need:

```bash
pip install safl-ocean-hardware
```

This includes drivers for the Teknic ClearCore stepper motor and Massa ultrasonic sensor.

## Configuration

### 1. Create `config/example_config.yaml`

```yaml
weir:
  port: /dev/ttyACM0
  home_offset_mm: 0.0

gauge:
  port: /dev/ttyUSB2
  sensor_ids: [0]
  offset_mm: 1000.0

timing:
  event_log: ./experiment_events.csv
  checkpoint_file: ./experiment_checkpoint.json
```

### 2. Create DSLR Config

Copy `dualcam-timelapse/config/cameras.yaml` and customize with your USB ports:

```yaml
cameras:
  Hangang:
    iso: 1600
    aperture: 5.6
    shutter: "1/125"
    output_dir: ./captures/Hangang
    port: usb:001,005

  Nakdong:
    iso: 1600
    aperture: 5.6
    shutter: "1/125"
    output_dir: ./captures/Nakdong
    port: usb:001,006

timelapse:
  interval_seconds: 10
  total_photos: 100
```

### 3. Create Experiment Schedule

A CSV file with `time_s, weir_elevation_mm, pump_flow_lpm, qin_open, qaux_open` columns:

```csv
time_s,weir_elevation_mm,pump_flow_lpm,qin_open,qaux_open
0,100,0.0,1,0
60,90,0.0,1,0
120,80,0.0,1,0
180,70,0.0,1,0
240,60,0.0,1,0
300,50,0.0,1,0
```

The weir elevation is interpolated via cubic spline, so you get smooth motion between keyframes.

## Running the Experiment

### Basic Usage (Foreground)

```bash
python experiments/weir_gauge_camera_experiment.py \
    --schedule examples/example_schedule.csv \
    --dslr-config ./cameras.yaml \
    --dualcam-path /path/to/dualcam-timelapse \
    --duration 300
```

### Interactive Mode (REPL-Accessible)

```bash
python experiments/weir_gauge_camera_experiment.py \
    --schedule examples/example_schedule.csv \
    --dslr-config ./cameras.yaml \
    --dualcam-path /path/to/dualcam-timelapse \
    --duration 3600 \
    --interactive
```

Then in the terminal, while the experiment runs, you have full REPL access:

```python
# From another terminal or in a REPL session:
lab.stop()                   # Pause clock + weir + camera captures
lab.resume(60)               # Resume for 60 more experiment-seconds
lab.get_system_status()      # Query all subsystem states
lab.emergency_stop()         # Full emergency shutdown
```

### From Python / Jupyter Notebook

```python
from experiments.weir_gauge_camera_experiment import main_interactive

lab, scheduler_thread = main_interactive(
    schedule_file="examples/example_schedule.csv",
    dslr_config="./cameras.yaml",
    dualcam_path="/path/to/dualcam-timelapse",
)

# Experiment is running in background; REPL/notebook is live
lab.get_system_status()
lab.stop()           # Pause
lab.resume(120)      # Resume for 120 more seconds
```

## How It Works

### Scheduling and Non-Blocking Behavior

The scheduler normally **blocks** the calling thread until the experiment finishes. To make it non-blocking, we use `scheduler.run_async()`:

```python
with lab.experiment() as clock:
    # Run scheduler in background daemon thread
    scheduler_thread = lab.scheduler.run_async(duration=3600)
    
    # Foreground is now free for REPL/notebook interaction
    # Call lab.stop() from another terminal to pause
```

Internally:
- The scheduler loop runs in a daemon thread at 50ms polling intervals
- All scheduled actions fire in their own daemon threads (non-blocking)
- Calling `lab.stop()` sets the stop event and pauses the clock
- The clock is *paused*, not stopped — calling `lab.resume()` restarts the loop

### Gauge Logging

Every 5 seconds (experiment time):

```python
gauge.read_mm_smoothed()  # Fetch water level
# Result logged to experiment_events.csv
```

### Weir Control

Every 10 seconds:

```python
target_mm = schedule.weir_elevation(clock.elapsed())  # Get target from spline
weir.set_elevation(target_mm)  # Issue command (non-blocking)
```

The motor handles motion profiling; the stepper accelerates smoothly to reach the target.

### DSLR Timelapse

Every 60 seconds:

```python
dslr.capture_all()  # Trigger both cameras concurrently (in a daemon thread)
```

This wraps `dualcam.CameraManager.capture_all_parallel()`, which uses `ThreadPoolExecutor` to fire both cameras at the same instant and collect images asynchronously.

## Troubleshooting

### "Failed to import dualcam — ensure gphoto2 and PyYAML are installed"

This error means either:
- `gphoto2` Python bindings are not installed: `pip install gphoto2`
- `libgphoto2` system library is not installed: `sudo apt install libgphoto2-dev gphoto2`
- You're not on Linux (gphoto2 is Linux-only)

### "DSLR config not found at..."

The DSLR config path doesn't exist. Check the path and ensure it points to a valid YAML file.

### Weir doesn't move

- Check hardware connection: is the ClearCore connected to `/dev/ttyACM0`?
- Verify `config/example_config.yaml` has the correct port
- Call `lab.weir.get_status()` to check motor state

### Gauge reads NaN or garbage values

- Check the Massa sensor connection: is it on `/dev/ttyUSB2`?
- Verify `offset_mm` in config matches your flume setup
- Ensure sensor power is on

### Cameras don't trigger

- Did you discover the USB ports with `dualcam/discover.py`?
- Verify `cameras.yaml` has the correct `port` for each camera
- On a dev machine (macOS), the DSLR subsystem won't work — this is expected (gphoto2 is Linux-only)

## Advanced: Custom Scheduling

You can add more scheduled actions by extending the experiment script:

```python
def _custom_action():
    """Your custom logic here."""
    print(f"Elapsed: {lab.clock.elapsed()}s")
    lab.event_log.log(lab.clock.elapsed(), "custom", "action_name", "ok")

lab.scheduler.repeat(
    every=15,  # Every 15 experiment-seconds
    action=_custom_action,
    subsystem="custom",
    name="my_action",
)
```

Or register one-shot actions at specific times:

```python
lab.scheduler.at(
    runtime_s=120,  # Fire once at 120 experiment-seconds
    action=some_function,
    subsystem="flume",
    name="halfway_event",
)
```

## Event Log Format

All actions are logged to `experiment_events.csv`:

```csv
runtime_s,subsystem,action_name,result
0.0,flume_lab,experiment_start,ok
5.0,gauge,log_level,elevation_mm=87.50
10.0,weir,update_elevation,target_mm=99.50
15.0,gauge,log_level,elevation_mm=87.52
...
300.0,flume_lab,experiment_stop,ok
```

This lets you post-process and visualize sensor data, events, and commands in synchrony.

## Files Modified / Created

### New Files
- `experiments/weir_gauge_camera_experiment.py` — Main experiment orchestration
- `src/laguna/camera/dslr.py` — DSLR subsystem wrapper
- `examples/example_schedule.csv` — Example schedule CSV

### Modified Files
- `src/laguna/timing/scheduler.py` — Added `run_async()` method
- `src/laguna/core.py` — Added `stop()`, `resume()`, fixed missing `Path` import
- `src/laguna/camera/__init__.py` — Export `DslrCameraSubsystem`
