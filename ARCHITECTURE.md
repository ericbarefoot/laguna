# Weir-Gauge-Camera Experiment Architecture

## System Diagram

```
┌────────────────────────────────────────────────────────────────────────────┐
│                          EXPERIMENT ORCHESTRATOR                           │
│                          (FlumeLab Lab Instance)                           │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐ │
│  │               TIMING BACKBONE (Always Present)                       │ │
│  ├──────────────────────────────────────────────────────────────────────┤ │
│  │  ExperimentClock      Scheduler        EventLog                     │ │
│  │  ├─ elapsed()         ├─ repeat()      └─ log(runtime, subsys,     │ │
│  │  ├─ start()           ├─ at()            action, result)            │ │
│  │  ├─ stop()            ├─ run() [BLOCKING] → experiment_events.csv   │ │
│  │  ├─ pause()           ├─ run_async() ←── [NEW: DAEMON THREAD]      │ │
│  │  └─ resume()          ├─ stop()                                     │ │
│  │                       └─ fire() [daemon threads per action]         │ │
│  │                                                                      │ │
│  │  CONTROL METHODS (NEW):                                            │ │
│  │  ├─ lab.stop()    → pauses clock + stops scheduler + stops weir   │ │
│  │  └─ lab.resume(s) → restarts scheduler for s more seconds         │ │
│  └──────────────────────────────────────────────────────────────────────┘ │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐ │
│  │            OPT-IN SUBSYSTEMS (Registered via lab.add())             │ │
│  ├──────────────────────────────────────────────────────────────────────┤ │
│  │                                                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────┐  │ │
│  │  │ WEIR: SaflWeirController (Hardware: Teknic ClearCore)        │  │ │
│  │  ├──────────────────────────────────────────────────────────────┤  │ │
│  │  │ • set_elevation(mm) — issue command (non-blocking)          │  │ │
│  │  │ • wait_for_move(timeout) — block until done                 │  │ │
│  │  │ • stop() — abort in-progress move                           │  │ │
│  │  │ • get_elevation() — query current position                  │  │ │
│  │  └──────────────────────────────────────────────────────────────┘  │ │
│  │                                                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────┐  │ │
│  │  │ GAUGE: SaflWaterLevelSensor (Hardware: Massa Ultrasonic)    │  │ │
│  │  ├──────────────────────────────────────────────────────────────┤  │ │
│  │  │ • read_mm() — instantaneous elevation reading               │  │ │
│  │  │ • read_mm_smoothed() — FIFO moving average                  │  │ │
│  │  │ • get_status() — return last_read + metadata                │  │ │
│  │  └──────────────────────────────────────────────────────────────┘  │ │
│  │                                                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────┐  │ │
│  │  │ DSLR CAMERAS: DslrCameraSubsystem (Hardware: USB Canon)  [NEW] │ │
│  │  ├──────────────────────────────────────────────────────────────┤  │ │
│  │  │ • Wraps: dualcam.CameraManager (Minsik's code, unmodified)  │  │ │
│  │  │ • connect() — lazy import gphoto2, load cameras.yaml        │  │ │
│  │  │ • capture_all() — parallel trigger via ThreadPoolExecutor   │  │ │
│  │  │ • disconnect() — close USB connections                      │  │ │
│  │  │ • get_status() — return connection state + camera count     │  │ │
│  │  └──────────────────────────────────────────────────────────────┘  │ │
│  │                                                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────┐  │ │
│  │  │ SCHEDULE: ExperimentSchedule (Existing, not a subsystem)     │  │ │
│  │  ├──────────────────────────────────────────────────────────────┤  │ │
│  │  │ • from_csv(path) — load time_s, weir_elevation_mm, etc.    │  │ │
│  │  │ • weir_elevation(t) → float [spline interpolation]          │  │ │
│  │  │ • pump_flow(t), qin_open(t), qaux_open(t) [other channels] │  │ │
│  │  └──────────────────────────────────────────────────────────────┘  │ │
│  │                                                                      │ │
│  └──────────────────────────────────────────────────────────────────────┘ │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

## Execution Flow — Non-Blocking Mode

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ Main Script (weir_gauge_camera_experiment.py)                               │
└──────────────────────────────────────────────────────────────────────────────┘
         │
         ├─ lab.add(weir).add(gauge).add(dslr)
         ├─ lab.connect_all()  [hardware ready]
         ├─ schedule = ExperimentSchedule.from_csv("schedule.csv")
         │
         ├─ lab.scheduler.repeat(every=5, action=_log_gauge, ...)
         ├─ lab.scheduler.repeat(every=10, action=_update_weir, ...)
         ├─ lab.scheduler.repeat(every=60, action=_capture_dslr, ...)
         │
         ├─ with lab.experiment() as clock:
         │       │
         │       ├─ scheduler_thread = lab.scheduler.run_async(duration=3600)
         │       │     │
         │       │     └─► [DAEMON THREAD SPAWNED]
         │       │          ├─ while not stopped:
         │       │          │   ├─ [Poll every 50ms]
         │       │          │   ├─ [Fire _log_gauge at t=5, 10, 15, ...]
         │       │          │   │   └─ spawn daemon thread for gauge read
         │       │          │   ├─ [Fire _update_weir at t=10, 20, 30, ...]
         │       │          │   │   └─ spawn daemon thread for weir move
         │       │          │   ├─ [Fire _capture_dslr at t=60, 120, ...]
         │       │          │   │   └─ spawn daemon thread for camera capture
         │       │          │   └─ sleep(0.05)
         │       │          └─ return
         │       │
         │       └─ [FOREGROUND STAYS LIVE → REPL ACCESSIBLE]
         │
         └─ lab.disconnect_all()  [cleanup]


         ┌───────────────────────────────────────────────────────────────┐
         │ FROM ANOTHER TERMINAL / REPL:                                │
         ├───────────────────────────────────────────────────────────────┤
         │                                                               │
         │ >>> lab.stop()          # At any time!                       │
         │     ├─ scheduler._stop_event.set()  [loop exits next iter]  │
         │     ├─ clock.pause()    [preserves elapsed time]            │
         │     └─ weir.stop()      [abort motor move]                  │
         │                                                               │
         │ >>> lab.resume(60)      # Continue for 60 more seconds      │
         │     └─ scheduler.run_async(60)  [restart loop in new thread]│
         │                                                               │
         │ >>> lab.get_system_status()  # Query all subsystems         │
         │     └─ {"timing": {...}, "weir": {...}, "gauge": {...}}   │
         │                                                               │
         └───────────────────────────────────────────────────────────────┘
```

## Scheduled Action Execution (Daemon Threads)

```
Main Scheduler Loop Thread          Gauge Thread            Weir Thread
(run_async)                        (every 5s)              (every 10s)
────────────────────────────────────────────────────────────────────────────

t=0
  clock.start()
  |
  |
  +─ repeat: _log_gauge (every 5)
  +─ repeat: _update_weir (every 10)
  +─ repeat: _capture_dslr (every 60)
  |
t=5 [elapsed]
  |
  +─ Fire _log_gauge  ─────────────→ ┌──────────────────┐
  |                                  │ gauge.read_mm()  │
  |                                  └──────────────────┘
  |                                  ⬇
  |                                  event_log.log(...)
  |
  sleep(50ms)
  |
t=10 [elapsed]
  |
  +─ Fire _update_weir ────────────→ ┌──────────────────────────┐
  |                                  │ target = sched.weir(...) │
  |                                  │ weir.set_elevation(...)  │
  |                                  └──────────────────────────┘
  |
  +─ Fire _log_gauge  ─────────────→ [gauge read again]
  |
  sleep(50ms)
  |
t=15
  |
  +─ Fire _log_gauge  ─────────────→ [gauge read]
  |
  ... (repeat pattern)
  |
t=60
  |
  +─ Fire _capture_dslr ──────────→ ┌─────────────────────────┐
  |                                 │ dslr.capture_all()      │
  |                                 │ [ThreadPoolExecutor]    │
  |                                 │  ├─ Camera 1 trigger    │
  |                                 │  └─ Camera 2 trigger    │
  |                                 └─────────────────────────┘
  |
  +─ Fire _log_gauge  ─────────────→ [gauge read]
  |
  +─ Fire _update_weir ────────────→ [weir move]
  |
  ... (continue until duration expires or stop() called)
  |
  |
t=3600 [duration reached]
  |
  └─ return from run()

All daemon threads:
  ├─ Catch exceptions → log
  ├─ Log result to event_log → experiment_events.csv
  └─ Return immediately (don't block scheduler loop)
```

## State Transitions

```
                ┌──────────────┐
                │   Created    │
                │  FlumeLab()  │
                └──────┬───────┘
                       │
                       ├─ add(weir, gauge, dslr)
                       ├─ connect_all()
                       │
                       ▼
                ┌──────────────────┐
                │   Idle/Ready     │
                │ Clock not started │
                └──────┬───────────┘
                       │
                       ├─ with lab.experiment():
                       │   ├─ clock.start()
                       │
                       ▼
        ┌──────────────────────────────────┐
        │   Running (Foreground Blocked)   │
        │   lab.scheduler.run(duration)    │
        └──────────────────────────────────┘
                       │
        (OR) RECOMMENDED PATTERN:
                       │
                       ├─ scheduler_thread = lab.scheduler.run_async()
                       │
                       ▼
        ┌──────────────────────────────────┐
        │   Running (Foreground Live)      │
        │  Thread: run_async() in background │
        ├──────────────────────────────────┤
        │ REPL / Notebook can:             │
        │  • lab.stop()    → Paused state  │
        │  • lab.resume(s) → Running again │
        │  • lab.get_system_status()       │
        └──────────────────────────────────┘
                       │
                       ├─ lab.stop()
                       │
                       ▼
        ┌──────────────────────────────────┐
        │   Paused                         │
        │   Clock paused (elapsed preserved) │
        ├──────────────────────────────────┤
        │  Can:                            │
        │  • lab.resume(s) → Running       │
        │  • lab.emergency_stop() → Stop   │
        └──────────────────────────────────┘
                       │
                       ├─ [duration expires] OR [lab.stop()]
                       │
                       ▼
                ┌──────────────────┐
                │   Stopped        │
                │ clock.stop()     │
                │ event_log.close()│
                └──────┬───────────┘
                       │
                       ├─ lab.disconnect_all()
                       │
                       ▼
                ┌──────────────────┐
                │   Disconnected   │
                │  Hardware released│
                └──────────────────┘
```

## Event Log Output

```csv
runtime_s,subsystem,action_name,result
0.0,flume_lab,experiment_start,ok
5.0,gauge,log_level,elevation_mm=987.50
10.0,weir,update_elevation,target_mm=99.50
10.0,gauge,log_level,elevation_mm=987.52
15.0,gauge,log_level,elevation_mm=987.54
20.0,weir,update_elevation,target_mm=98.75
20.0,gauge,log_level,elevation_mm=987.55
25.0,gauge,log_level,elevation_mm=987.57
30.0,weir,update_elevation,target_mm=98.00
30.0,gauge,log_level,elevation_mm=987.58
...
60.0,dslr_cameras,capture_all,files=['Hangang', 'Nakdong']
60.0,gauge,log_level,elevation_mm=987.60
...
3600.0,flume_lab,experiment_stop,ok
```

## Thread Safety

- **Scheduler loop** runs in daemon thread; checked for `_stop_event.is_set()` every 50ms
- **Scheduled actions** spawn their own daemon threads (never block scheduler)
- **Clock pause/resume** is thread-safe (uses threading.Event internally)
- **Event logging** is thread-safe (uses file append semantics)
- **Cross-thread calls:** `lab.stop()` from REPL sets the stop event; scheduler loop detects it next polling cycle

## Files Involved

```
src/laguna/
├── core.py                      [EDIT: add stop(), resume()]
├── timing/
│   └── scheduler.py             [EDIT: add run_async()]
└── camera/
    ├── dslr.py                  [NEW: DslrCameraSubsystem]
    └── __init__.py              [EDIT: export DslrCameraSubsystem]

experiments/
└── weir_gauge_camera_experiment.py   [NEW: main orchestration script]

examples/
└── example_schedule.csv         [NEW: sample schedule]
```
