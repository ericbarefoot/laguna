"""
Example 14: A user-defined scheduled action — a repeated tiled survey

setup_run() only wires up the built-in per-subsystem scheduled actions
(gauge polling, weir/flow setpoints, camera captures, one fixed Gocator
transect via gocator.scan:). Anything more involved — like repeating a
whole multi-pass Tile survey on an interval — isn't something you add to
the codebase; you register it yourself using schedule_action(), the same
building block every built-in action goes through internally (see
src/laguna/experiment/runner.py and
docs/subsystems/experiment.md#user-defined-scheduled-actions).

Note there's no manual "am I still running?" guard in tiled_scan() below
— schedule_action() already refuses to let one action overlap itself
(escalates to a lab-wide pause instead), so a schedule dense enough to
fire this again before a previous tile finishes surfaces loudly as a
pause, not a silent skip or a race. See
docs/subsystems/experiment.md#user-defined-scheduled-actions.

This runs in simulate=True by default — SAFE, no real hardware commanded
regardless of config. Read through it, then point CONFIG_FILE at a real
rig's config and flip SIMULATE = False yourself when ready.
"""

from laguna.experiment import run_blocking, schedule_action, setup_run
from laguna.survey import SurveyRunner, Tile
from laguna.timing.checkpoint import CheckpointStore

CONFIG_FILE = "config/example_config.yaml"
DURATION_S = 120.0  # short, for a quick rehearsal — raise for a real run

SIMULATE = True


def make_tiled_scan_hook(lab):
    """Build the zero-arg closure schedule_action() will fire.

    Wrapping construction in a factory (rather than a bare closure) keeps
    the CheckpointStore/escalate() plumbing in one place, next to the
    Tile spec it belongs to, instead of scattered across the call site.
    """
    checkpoint = CheckpointStore("tmp/tiled_scan_checkpoint.json", resume=True)

    def tiled_scan():
        active_area = lab.gocator.get_active_area()
        tile = Tile(
            origin=[0, 0, 0],
            length_mm=1000.0,
            width_mm=600.0,
            swath_mm=active_area["width_mm"],
            instrument="gocator",
            speed=20.0,
        )
        try:
            SurveyRunner(lab, tile, checkpoint=checkpoint).run()
        except Exception as exc:
            # A partially-completed tile is recoverable (the checkpoint
            # remembers which passes finished); a survey that silently
            # stops collecting topography is not — escalate rather than
            # letting the scheduler swallow the exception.
            lab.escalate(f"tiled_scan failed: {exc}")

    return tiled_scan


def main():
    lab = setup_run(CONFIG_FILE, simulate=SIMULATE)

    # config/example_config.yaml has no tiled_scan: section of its own —
    # a real experiment would add one (e.g. `tiled_scan: {interval_s: 900}`)
    # and use lab.config.get("tiled_scan") here instead.
    schedule_action(
        lab, {"interval_s": 60}, subsystem="gocator", name="tiled_scan",
        action=make_tiled_scan_hook(lab),
    )

    print(f"Running for {DURATION_S:.0f}s (simulate={SIMULATE})...")
    run_blocking(lab, duration=DURATION_S)
    print("Done — see data/experiment_events.csv for the 'gocator'/'tiled_scan' rows.")


if __name__ == "__main__":
    main()
