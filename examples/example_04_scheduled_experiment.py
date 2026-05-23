#!/usr/bin/env python3
"""Example 04 — Scheduled experiment with camera array and crash recovery.

Demonstrates:
  - FlumeLab with a networked Pi camera array defined in config
  - Scheduler driving periodic hydraulics polls and one-shot camera triggers
  - CheckpointStore so captures that already succeeded are skipped on restart
  - lab.experiment() context manager handling clock start/stop and event logging

Run once for a fresh experiment:
    python3 example_04_scheduled_experiment.py

Resume after a crash (skips already-completed captures):
    python3 example_04_scheduled_experiment.py --resume
"""

import argparse
import logging
from pathlib import Path

from laguna import FlumeLab, CheckpointStore
from laguna.camera import CameraManager

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

CAPTURE_TIMES = [10, 20, 30, 40, 50, 60]   # runtime seconds
EXPERIMENT_DURATION = 75                         # seconds
CHECKPOINT_FILE = "./experiment_checkpoint.json"

# Camera configs: a networked Pi array.  Edit hosts/paths for your setup.
CAMERA_CONFIGS = [
    {
        "name": "pi_array",
        "type": "network",
        "hosts": ["antares.laguna", "sirius.laguna"],
        "ssh_user": "pi",
        "ssh_key": "~/.ssh/id_rsa",
        "lead_time": 5.0,
        "output_dir": "./captures",
    }
]


def main(resume: bool) -> None:
    lab = FlumeLab("config/example_config.yaml")
    lab.add(CameraManager(CAMERA_CONFIGS))

    if not lab.connect_all():
        logging.error("Could not connect all subsystems — aborting.")
        return

    store = CheckpointStore(CHECKPOINT_FILE, resume=resume)

    # Register recurring hydraulics poll every 5 s.
    lab.scheduler.repeat(
        every=5,
        action=lab.hydraulics.get_status,
        subsystem="hydraulics",
        name="status_poll",
    )

    # Register one-shot camera triggers at each capture time.
    for i, t in enumerate(CAPTURE_TIMES):
        if store.is_complete(i):
            logging.info("Capture %d (t=%ds) already complete — skipping.", i, t)
            continue

        def _capture(capture_idx=i, capture_t=t):
            results = lab.cameras.trigger_capture()
            lab.cameras.report_simultaneity(results)
            lab.cameras.fetch_images(results, output_dir=Path("./captures"))
            store.mark_complete(
                capture_idx,
                runtime_s=lab.clock.elapsed(),
                wall_time=lab.clock.wall_time(),
                name=f"capture_{capture_idx}",
            )

        lab.scheduler.at(runtime_s=t, action=_capture, subsystem="camera", name=f"capture_{i}")

    with lab.experiment(resume=resume, checkpoint_file=CHECKPOINT_FILE) as clock:
        logging.info("Experiment started. Duration: %ds", EXPERIMENT_DURATION)
        lab.scheduler.run(duration=EXPERIMENT_DURATION)
        logging.info("Scheduler finished. Runtime: %.1fs", clock.elapsed())

    lab.disconnect_all()
    logging.info("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", action="store_true", help="Resume from checkpoint")
    args = parser.parse_args()
    main(resume=args.resume)
