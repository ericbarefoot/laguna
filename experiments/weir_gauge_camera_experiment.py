#!/usr/bin/env python3
"""Weir-Gauge-Camera experiment.

All subsystem config (serial ports, hosts, capture intervals) lives in a single
YAML config file. Pass --schedule to enable actuator movement.

Interactive use:
    from experiments.weir_gauge_camera_experiment import main_interactive
    lab = main_interactive()                           # monitoring only
    lab = main_interactive(schedule="schedule.csv")   # weir/flow follow CSV
    lab.start(3600)
    lab.get_system_status()
    lab.stop() / lab.resume() / lab.disconnect_all()
"""

import argparse
import logging

from laguna.experiment import setup_run, run_blocking

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
)


def main_interactive(
    lab_config: str = "config/example_config.yaml",
    schedule: str = None,
    verbose_cameras: bool = False,
):
    """Set up experiment and return lab — nothing runs until lab.start()."""
    lab = setup_run(lab_config=lab_config, schedule=schedule, verbose_cameras=verbose_cameras)
    print("lab.start(N)            — run for N seconds")
    print("lab.get_system_status() — check subsystem states")
    print("lab.stop()              — pause")
    print("lab.resume()            — resume for remaining time")
    print("lab.disconnect_all()    — clean shutdown")
    return lab


def main(
    lab_config: str = "config/example_config.yaml",
    duration: float = 3600.0,
    schedule: str = None,
):
    """Run experiment in blocking mode with signal-based pause/resume."""
    lab = setup_run(lab_config=lab_config, schedule=schedule)
    run_blocking(lab, duration)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Weir-Gauge-Camera experiment")
    p.add_argument("--lab-config", default="config/example_config.yaml")
    p.add_argument("--duration", type=float, default=3600.0)
    p.add_argument(
        "--schedule",
        default=None,
        help="Schedule CSV. If set, actuators with use_schedule=true will move.",
    )
    args = p.parse_args()
    main(args.lab_config, args.duration, args.schedule)
