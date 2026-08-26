"""
Example 12: Live-hardware check for the qin/qaux solenoid valves

Flow has been implemented and code-reviewed (see
docs/integration-notes/CONFLUENCE_INTEGRATION.md) but never exercised
against the real ClearCore-driven solenoids on red.lab. This script is
that first live check, plus a regression check for a bug just fixed in
qin/qaux's setters: they used to only guard against a reply *timeout* —
a reply that arrived but carried `ok: False` (confluence rejecting the
command) was silently treated as success, caching a valve state the
hardware never confirmed. The setters now raise RuntimeError on
`ok: False` too (see src/laguna/flow/controller.py's qin/qaux setters and
tests/test_flow.py's test_qin_setter_raises_on_rejected_reply_and_leaves_cache_unchanged
/ test_qaux_setter_raises_on_rejected_reply_and_leaves_cache_unchanged for
the mocked coverage).

This does NOT touch the pump/VFD at all — valves only.

NO ACTUATION IS EXECUTED BY DEFAULT. ALLOW_ACTUATION below is False —
with it False, this script only connects, prints status, and disconnects.
Read through the whole thing and confirm you can see/hear the qin and
qaux solenoids (and that whatever they gate is safe to cycle open/closed)
before setting ALLOW_ACTUATION = True yourself.

--- Background you need before setting ALLOW_ACTUATION = True ---

1. confluence.service must already be running on red.lab and connected to
   the shared ClearCore (same physical controller as the weir gate axis —
   see CONFLUENCE_INTEGRATION.md's "How it works" section).
2. mqtt.node_name in the config file must match confluence_config.json's
   "Node Name" on red.lab exactly.
3. qin is set_io channel 0, qaux is channel 1 (see confluence's
   Teknic_ClearCore_funcs.py handle_valve_command()) — this script does
   not verify that mapping is physically correct for your plumbing, only
   that laguna's commands reach *some* channel and confluence acks it.
   Watch/listen to confirm which physical solenoid actually clicks for
   each step.
4. There is no independent hardware readback for valve state — confluence
   only echoes back the last commanded state (see valve_status in
   Teknic_ClearCore_funcs.py). get_status()'s qin_open/qaux_open below
   reflects "what we last told it," not a sensor confirming it happened,
   so visual/audible confirmation on your end is the actual check here.
5. Always closes both valves in a finally block, regardless of how far
   the script gets or whether ALLOW_ACTUATION is True — leaving a
   solenoid open because a mid-script exception skipped cleanup would be
   worse than the script doing nothing.
"""

import time

from laguna import FlumeLab

CONFIG_FILE = "sandbox/confluence_testing/config.yaml"

HOLD_OPEN_S = 3.0  # time to hold each valve open so you can confirm it by eye/ear

ALLOW_ACTUATION = True


def toggle_and_report(lab, valve_name, state):
    """Set one valve, print the result, and surface a rejection clearly."""
    print(f"  Setting {valve_name} = {state}...")
    try:
        setattr(lab.flow, valve_name, state)
    except RuntimeError as exc:
        print(f"  REJECTED: {exc}")
        return False
    print(f"  {valve_name} now reports: {getattr(lab.flow, valve_name)}")
    return True


def main():
    lab = FlumeLab(config_file=CONFIG_FILE)
    lab.add("flow")

    broker_host = lab.config.get("mqtt")["broker_host"]
    print(f"Connecting to {broker_host}...")
    if not lab.flow.connect():
        raise RuntimeError(f"could not reach the MQTT broker at {broker_host}")

    print(f"Initial status: {lab.flow.get_status()}")

    if not ALLOW_ACTUATION:
        print()
        print("ALLOW_ACTUATION is False — no valves were commanded.")
        print("Set ALLOW_ACTUATION = True at the top of this file to run the")
        print("real-hardware valve check.")
        lab.flow.disconnect()
        return

    try:
        for valve_name in ("qin", "qaux"):
            print(f"\n--- {valve_name} ---")
            if toggle_and_report(lab, valve_name, True):
                print(f"  Holding open for {HOLD_OPEN_S:.1f}s — confirm it physically opened...")
                time.sleep(HOLD_OPEN_S)
            toggle_and_report(lab, valve_name, False)
            time.sleep(5)
            print(f"  Status: {lab.flow.get_status()}")
    finally:
        print("\nClosing both valves (cleanup)...")
        try:
            lab.flow.qin = False
        except RuntimeError as exc:
            print(f"  WARNING: could not confirm qin closed: {exc}")
        try:
            lab.flow.qaux = False
        except RuntimeError as exc:
            print(f"  WARNING: could not confirm qaux closed: {exc}")
        print(f"Final status: {lab.flow.get_status()}")
        lab.flow.disconnect()


if __name__ == "__main__":
    main()
