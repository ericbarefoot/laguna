"""
Example 11: Live-hardware check for the wait_for_move() stale-status race

Confluence only republishes weir status on its own schedule (0.5s interval
by default), not in response to a command. go_to_elevation() guards
against wait_for_move() trusting a pre-move status message by draining the
status topic's backlog right after the move is accepted — see
src/laguna/weir/controller.py's go_to_elevation()/wait_for_move()
docstrings and tests/test_weir.py's
test_wait_for_move_does_not_trust_pre_move_status /
test_wait_for_move_ignores_pre_move_backlog_but_honors_fresh_status for
the mocked coverage of this fix.

This script is the real-hardware half of that test plan: command a small
move sized to take clearly longer than one publish interval, call
wait_for_move() immediately (worst case for the race — no manual delay
inserted), and check that it actually blocked for close to the real move
duration rather than returning near-instantly.

NO MOTION IS EXECUTED BY DEFAULT. ALLOW_MOTION below is False — with it
False, this script only connects, prints status, and disconnects. Read
through the whole thing, confirm MOVE_DISTANCE_MM/MOVE_VELOCITY_MM_S are
safe for your setup, and set ALLOW_MOTION = True yourself once ready to
actually move hardware.

--- Background you need before setting ALLOW_MOTION = True ---

1. confluence.service must already be running on red.lab and connected to
   the Teknic ClearCore, with the weir gate physically clear to move
   MOVE_DISTANCE_MM in either direction from its current position.
2. mqtt.node_name in the config file must match confluence_config.json's
   "Node Name" on red.lab exactly.
3. This moves real hardware. Nothing in laguna auto-triggers this script
   or this move — you are running it and setting ALLOW_MOTION yourself.
4. Repeats REPEAT_COUNT times back-to-back with no inter-move delay, since
   that's the specific failure mode the race describes (status not yet
   republished since the *previous* move by the time the next one starts).
"""

import time

from laguna import FlumeLab

CONFIG_FILE = "sandbox/confluence_testing/config.yaml"

# --- Move settings — confirm these are safe for your setup before setting
# ALLOW_MOTION = True. Slow and small is the point: it needs to take
# clearly longer than confluence's 0.5s status publish interval so a
# near-instant wait_for_move() return is unambiguous evidence of the race. ---
MOVE_DISTANCE_MM = 20.0
MOVE_VELOCITY_MM_S = 3.0  # ~6-7s per move at this distance/velocity
REPEAT_COUNT = 4
WAIT_TIMEOUT_S = 30.0

ALLOW_MOTION = True


def wait_for_valid_elevation(lab, timeout=5.0, poll_interval=0.1):
    """Poll get_elevation() until it returns a real (non-NaN) reading.

    get_elevation()/get_status() both drain the whole status queue and keep
    only the newest message (see MqttSubscriber.get_latest()) — calling
    either one empties the queue, so a call made right after another can
    find nothing buffered yet and return NaN. Never treat a single
    get_elevation() call as reliable; always retry through this instead of
    passing its result straight into a move computation.

    Raises:
        RuntimeError: If no valid reading arrives within `timeout`.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        elevation = lab.weir.get_elevation()
        if elevation == elevation:  # NaN != NaN
            return elevation
        time.sleep(poll_interval)
    raise RuntimeError(
        f"no valid elevation reading arrived within {timeout:.1f}s — "
        "refusing to compute a move target from NaN"
    )


def run_move_and_check(lab, target_mm, expected_min_s):
    """Command one move, time wait_for_move(), and report pass/fail."""
    if target_mm != target_mm:  # NaN check — never send an invalid target
        raise RuntimeError("refusing to send a NaN elevation target to real hardware")
    before = wait_for_valid_elevation(lab)
    t0 = time.monotonic()
    accepted = lab.weir.go_to_elevation(target_mm)
    if not accepted:
        print(f"  go_to_elevation({target_mm:.1f}) was NOT accepted — skipping wait")
        return
    lab.weir.wait_for_move(timeout=WAIT_TIMEOUT_S)
    elapsed = time.monotonic() - t0
    after = wait_for_valid_elevation(lab)

    verdict = "OK" if elapsed >= expected_min_s else "SUSPECT — returned early"
    print(
        f"  target={target_mm:.1f}mm before={before:.1f}mm after={after:.1f}mm "
        f"elapsed={elapsed:.2f}s (expected >= {expected_min_s:.2f}s) [{verdict}]"
    )


def main():
    lab = FlumeLab(config_file=CONFIG_FILE)
    lab.add("weir")

    broker_host = lab.config.get("mqtt")["broker_host"]
    print(f"Connecting to {broker_host}...")
    if not lab.weir.connect():
        raise RuntimeError(f"could not reach the MQTT broker at {broker_host}")

    print("Waiting for a status reading...")
    wait_for_valid_elevation(lab)
    print(f"Full status: {lab.weir.get_status()}")

    if not ALLOW_MOTION:
        print()
        print("ALLOW_MOTION is False — no moves were run.")
        print("Set ALLOW_MOTION = True at the top of this file to run the")
        print("real-hardware race check.")
        lab.weir.disconnect()
        return

    lab.weir.enable()
    lab.weir.set_velocity(MOVE_VELOCITY_MM_S)
    expected_min_s = (MOVE_DISTANCE_MM / MOVE_VELOCITY_MM_S) * 0.5

    start_mm = wait_for_valid_elevation(lab)
    print(f"\nStarting elevation: {start_mm:.1f}mm")
    print(f"Running {REPEAT_COUNT} back-to-back moves of {MOVE_DISTANCE_MM:.1f}mm...\n")

    target = start_mm
    for i in range(REPEAT_COUNT):
        target = target + MOVE_DISTANCE_MM if i % 2 == 0 else target - MOVE_DISTANCE_MM
        print(f"Move {i + 1}/{REPEAT_COUNT}:")
        run_move_and_check(lab, target, expected_min_s)

    # lab.weir.disable()
    lab.weir.disconnect()


if __name__ == "__main__":
    main()
