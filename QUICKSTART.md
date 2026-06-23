# Weir-Gauge-Camera Experiment — Quick Start

## Before You Run

### Step 1: Clone dualcam-timelapse (One-Time Setup)

```bash
cd /path/to/repos  # sibling to laguna
git clone https://github.com/yukms/dualcam-timelapse.git
cd dualcam-timelapse
pip install -r requirements.txt
```

### Step 2: Discover Camera USB Ports

```bash
cd dualcam-timelapse
python dualcam/discover.py
```

Output will look like:
```
Camera: Canon Rebel T7 - (Usb 001,005)
usb:001,005
```

Note both port values.

### Step 3: Create cameras.yaml

In dualcam-timelapse root, copy and edit `config/cameras.yaml`:

```yaml
cameras:
  Hangang:
    iso: 1600
    aperture: 5.6
    shutter: "1/125"
    output_dir: /path/to/captures/Hangang
    port: usb:001,005        # <- From step 2

  Nakdong:
    iso: 1600
    aperture: 5.6
    shutter: "1/125"
    output_dir: /path/to/captures/Nakdong
    port: usb:001,006        # <- From step 2

timelapse:
  interval_seconds: 60
  total_photos: 100
```

### Step 4: Create laguna config

Create `config/example_config.yaml` in laguna repo:

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

(Adjust port values for your hardware.)

### Step 5: Create Experiment Schedule

Create a CSV like `my_schedule.csv`:

```csv
time_s,weir_elevation_mm,pump_flow_lpm,qin_open,qaux_open
0,100,0.0,1,0
30,90,0.0,1,0
60,80,0.0,1,0
90,70,0.0,1,0
120,60,0.0,1,0
```

The weir will smoothly lower following the spline interpolation.

## Running the Experiment

### Quick Test (30 seconds, no cameras)

```bash
cd laguna  # repo root
python experiments/weir_gauge_camera_experiment.py \
    --schedule my_schedule.csv \
    --dslr-config ../dualcam-timelapse/config/cameras.yaml \
    --dualcam-path ../dualcam-timelapse \
    --duration 30
```

Check `experiment_events.csv` for gauge + weir log entries. No camera captures will occur (you can add them to the schedule later).

### Full Run (Interactive Mode)

```bash
python experiments/weir_gauge_camera_experiment.py \
    --schedule my_schedule.csv \
    --dslr-config ../dualcam-timelapse/config/cameras.yaml \
    --dualcam-path ../dualcam-timelapse \
    --duration 3600 \
    --interactive
```

While this runs, open another terminal:

```python
python3
>>> from laguna import FlumeLab
>>> # (Access the lab object from the running experiment)
>>> lab.stop()        # Pause everything at any time
>>> lab.resume(60)    # Resume for 60 more seconds
>>> lab.get_system_status()
```

Or in Jupyter:

```python
from experiments.weir_gauge_camera_experiment import main_interactive

lab, thread = main_interactive(
    schedule_file="my_schedule.csv",
    dslr_config="../dualcam-timelapse/config/cameras.yaml",
    dualcam_path="../dualcam-timelapse",
)
# Now in cells:
lab.stop()
lab.resume(120)
lab.get_system_status()
```

## What Happens

Every experiment run logs to `experiment_events.csv`:

```csv
runtime_s,subsystem,action_name,result
0.0,flume_lab,experiment_start,ok
5.0,gauge,log_level,elevation_mm=987.50
10.0,weir,update_elevation,target_mm=99.50
15.0,gauge,log_level,elevation_mm=987.55
60.0,dslr_cameras,capture_all,files=['Hangang', 'Nakdong']
...
```

Camera images are downloaded to the `output_dir` specified in `cameras.yaml`.

## Troubleshooting

| Problem | Solution |
|---|---|
| `ModuleNotFoundError: No module named 'gphoto2'` | Linux only. On macOS, DSLR will skip. On Linux: `pip install gphoto2` + `sudo apt install libgphoto2-dev gphoto2` |
| `DSLR config not found at ...` | Check path to `cameras.yaml`. Full path recommended. |
| `Weir doesn't move` | Check `/dev/ttyACM0` exists. Try `ls -la /dev/ttyACM0`. Verify `config/example_config.yaml` port. |
| `Gauge reads NaN` | Check `/dev/ttyUSB2` exists. Verify `offset_mm` in config. |
| `No camera images` | Verify USB cameras are connected. Run `dualcam/discover.py` again. Check `cameras.yaml` port values. |

## For More Details

See `WEIR_GAUGE_CAMERA_SETUP.md` for comprehensive setup and troubleshooting.

See `IMPLEMENTATION_SUMMARY.md` for technical architecture.
