#!/usr/bin/env python3
"""Weir-Gauge-Camera experiment with non-blocking scheduler.

Orchestrates a coordinated flume experiment:
  1. Logs water level (gauge) every 5 seconds
  2. Smoothly lowers the weir following a time-elevation schedule file
  3. Captures timelapse images from two Canon DSLRs every minute

The scheduler runs in a background daemon thread, so you can:
  - Pause and resume from a REPL: lab.stop(), then lab.resume(60)
  - Query system status: lab.get_system_status()
  - Emergency stop: lab.emergency_stop()

Usage:

    # Fresh experiment run
    python experiments/weir_gauge_camera_experiment.py \
        --schedule path/to/schedule.csv \
        --dslr-config path/to/cameras.yaml \
        --dualcam-path /path/to/dualcam-timelapse

    # Interactive REPL use
    from experiments.weir_gauge_camera_experiment import main_interactive
    lab, scheduler_thread = main_interactive(...)
    # ... query, pause, resume, etc.
"""

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional, Tuple

from laguna import FlumeLab
from laguna.schedule import ExperimentSchedule
from laguna.camera import DslrCameraSubsystem
from laguna.weir import SaflWeirController
from laguna.gauge import SaflWaterLevelSensor

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
)
logger = logging.getLogger(__name__)


def create_weir_from_config(config_path: str = "config/example_config.yaml") -> SaflWeirController:
    """Create and return a weir controller from config."""
    cfg = {}
    try:
        import yaml
        with open(config_path) as f:
            full_cfg = yaml.safe_load(f)
            cfg = full_cfg.get("weir", {})
    except FileNotFoundError:
        logger.warning("Config file %s not found; using defaults", config_path)
    except Exception as e:
        logger.warning("Could not load config: %s; using defaults", e)

    return SaflWeirController(cfg)


def create_gauge_from_config(config_path: str = "config/example_config.yaml") -> SaflWaterLevelSensor:
    """Create and return a water level sensor from config."""
    cfg = {}
    try:
        import yaml
        with open(config_path) as f:
            full_cfg = yaml.safe_load(f)
            cfg = full_cfg.get("gauge", {})
    except FileNotFoundError:
        logger.warning("Config file %s not found; using defaults", config_path)
    except Exception as e:
        logger.warning("Could not load config: %s; using defaults", e)

    return SaflWaterLevelSensor(cfg)


def main(
    schedule_file: str,
    dslr_config: str,
    dualcam_path: Optional[str] = None,
    lab_config: Optional[str] = None,
    duration: float = 3600.0,
    interactive: bool = False,
) -> None:
    """Run the weir-gauge-camera experiment.

    Args:
        schedule_file: Path to schedule CSV (time_s, weir_elevation_mm columns)
        dslr_config: Path to dualcam cameras.yaml config
        dualcam_path: Path to dualcam-timelapse repo root
        lab_config: Path to laguna config YAML
        duration: Experiment runtime in seconds
        interactive: If True, start a REPL-friendly version
    """
    logger.info("="*70)
    logger.info("Weir-Gauge-Camera Experiment")
    logger.info("="*70)

    # Initialize lab
    lab = FlumeLab(lab_config)

    # Add weir subsystem
    weir = create_weir_from_config(lab_config or "config/example_config.yaml")
    lab.add(weir)

    # Add gauge subsystem
    gauge = create_gauge_from_config(lab_config or "config/example_config.yaml")
    lab.add(gauge)

    # Add DSLR cameras
    if Path(dslr_config).exists():
        dslr = DslrCameraSubsystem(config_path=dslr_config, dualcam_path=dualcam_path)
        lab.add(dslr)
        has_dslr = True
    else:
        logger.warning("DSLR config not found at %s — skipping DSLR setup", dslr_config)
        has_dslr = False

    # Connect all subsystems
    logger.info("Connecting subsystems...")
    if not lab.connect_all():
        logger.error("Failed to connect all subsystems")
        return

    # Load schedule
    logger.info("Loading experiment schedule from %s", schedule_file)
    schedule = ExperimentSchedule.from_csv(schedule_file)

    # Track state for stopping camera captures
    should_capture = True

    # Define scheduled actions
    def _log_gauge():
        """Log water level reading."""
        try:
            elev_mm = gauge.read_mm_smoothed()
            logger.info("Water level: %.2f mm", elev_mm)
            lab.event_log.log(lab.clock.elapsed(), "gauge", "read_mm_smoothed", f"elevation_mm={elev_mm:.2f}")
        except Exception as e:
            logger.error("Gauge read failed: %s", e)

    def _update_weir():
        """Update weir elevation following the schedule."""
        try:
            target_mm = schedule.weir_elevation(lab.clock.elapsed())
            weir.set_elevation(target_mm)
            logger.info("Weir target: %.2f mm", target_mm)
            lab.event_log.log(lab.clock.elapsed(), "weir", "set_elevation", f"target_mm={target_mm:.2f}")
        except Exception as e:
            logger.error("Weir update failed: %s", e)

    def _capture_dslr():
        """Capture images from both DSLR cameras."""
        nonlocal should_capture
        if not should_capture:
            logger.info("Camera capture suppressed (experiment paused)")
            return

        if not has_dslr:
            return

        try:
            results = dslr.capture_all()
            logger.info("DSLR timelapse capture: %s", {k: v.name if v else None for k, v in results.items()})
            lab.event_log.log(lab.clock.elapsed(), "dslr_cameras", "capture_all", f"files={list(results.keys())}")
        except Exception as e:
            logger.error("DSLR capture failed: %s", e)

    # Register scheduled actions
    lab.scheduler.repeat(every=5, action=_log_gauge, subsystem="gauge", name="log_level")
    lab.scheduler.repeat(every=10, action=_update_weir, subsystem="weir", name="update_elevation")

    if has_dslr:
        lab.scheduler.repeat(every=60, action=_capture_dslr, subsystem="dslr_cameras", name="timelapse")

    # Run experiment
    try:
        with lab.experiment(resume=False) as clock:
            logger.info("Experiment started. Duration: %.1f seconds", duration)

            if interactive:
                logger.info("Starting in INTERACTIVE mode — REPL is live")
                logger.info("Available commands in REPL:")
                logger.info("  lab.stop()              — pause clock and scheduler")
                logger.info("  lab.resume(60)          — resume for 60 more seconds")
                logger.info("  lab.get_system_status() — query all subsystem states")
                logger.info("  lab.emergency_stop()    — emergency shutdown")
                logger.info("  should_capture = False  — suppress camera captures")
                # Start scheduler in background so REPL is accessible
                scheduler_thread = lab.scheduler.run_async(duration)
                logger.info("Waiting for scheduler thread to complete...")
                scheduler_thread.join()
            else:
                # Blocking mode: foreground loop until duration expires
                lab.scheduler.run(duration)

            elapsed = clock.elapsed()
            logger.info("Scheduler completed. Elapsed: %.1f seconds", elapsed)

    except KeyboardInterrupt:
        logger.info("Interrupted by user — stopping scheduler")
        lab.stop()
    except Exception as e:
        logger.error("Experiment error: %s", e)
        lab.emergency_stop()
    finally:
        logger.info("Disconnecting subsystems...")
        lab.disconnect_all()
        logger.info("Experiment finished")


def main_interactive(
    schedule_file: str,
    dslr_config: str,
    dualcam_path: Optional[str] = None,
    lab_config: Optional[str] = None,
) -> Tuple:
    """Initialize and return lab + scheduler thread for interactive REPL use.

    Example::

        lab, thread = main_interactive(
            schedule_file="schedule.csv",
            dslr_config="cameras.yaml",
            dualcam_path="/path/to/dualcam"
        )
        # ... in REPL:
        # lab.stop()
        # lab.resume(60)
        # lab.get_system_status()
    """
    lab = FlumeLab(lab_config)
    weir = create_weir_from_config(lab_config or "config/example_config.yaml")
    gauge = create_gauge_from_config(lab_config or "config/example_config.yaml")
    lab.add(weir).add(gauge)

    if Path(dslr_config).exists():
        dslr = DslrCameraSubsystem(config_path=dslr_config, dualcam_path=dualcam_path)
        lab.add(dslr)

    if not lab.connect_all():
        raise RuntimeError("Failed to connect subsystems")

    schedule = ExperimentSchedule.from_csv(schedule_file)

    def _log_gauge():
        try:
            elev_mm = gauge.read_mm_smoothed()
            lab.event_log.log(lab.clock.elapsed(), "gauge", "read_mm_smoothed", f"elevation_mm={elev_mm:.2f}")
        except Exception as e:
            logger.error("Gauge read failed: %s", e)

    def _update_weir():
        try:
            target_mm = schedule.weir_elevation(lab.clock.elapsed())
            weir.set_elevation(target_mm)
            lab.event_log.log(lab.clock.elapsed(), "weir", "set_elevation", f"target_mm={target_mm:.2f}")
        except Exception as e:
            logger.error("Weir update failed: %s", e)

    lab.scheduler.repeat(every=5, action=_log_gauge, subsystem="gauge", name="log_level")
    lab.scheduler.repeat(every=10, action=_update_weir, subsystem="weir", name="update_elevation")

    with lab.experiment(resume=False) as clock:
        logger.info("Interactive experiment started")
        thread = lab.scheduler.run_async(duration=3600)

    return lab, thread


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Weir-Gauge-Camera experiment with non-blocking scheduler"
    )
    parser.add_argument(
        "--schedule",
        required=True,
        help="Path to schedule CSV (time_s, weir_elevation_mm)",
    )
    parser.add_argument(
        "--dslr-config",
        required=True,
        help="Path to dualcam cameras.yaml config",
    )
    parser.add_argument(
        "--dualcam-path",
        default=None,
        help="Path to dualcam-timelapse repo root (optional, if not in sys.path)",
    )
    parser.add_argument(
        "--lab-config",
        default="config/example_config.yaml",
        help="Path to laguna config YAML",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3600.0,
        help="Experiment duration in seconds (default: 3600)",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Start in interactive mode (REPL-accessible)",
    )

    args = parser.parse_args()

    main(
        schedule_file=args.schedule,
        dslr_config=args.dslr_config,
        dualcam_path=args.dualcam_path,
        lab_config=args.lab_config,
        duration=args.duration,
        interactive=args.interactive,
    )
