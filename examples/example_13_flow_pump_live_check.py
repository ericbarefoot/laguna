"""
Example 13: Live-hardware check for the Fuji VFD pump

Flow's pump path has been code-reviewed but never exercised against the
real Fuji Frenic VFD on red.lab. This script is that first live check,
plus a regression check for two bugs just fixed:

Commands frequency directly via set_frequency_hz() rather than
set_flowrate() — the L/min-to-Hz calibration curve (C0/C1/C2) was found
to be wrong on first live test (1 L/min computed to 63.48 Hz, clamped to
the VFD's 60 Hz max — essentially full speed for what was meant to be a
deliberately low test rate). Use set_flowrate() again once that curve is
re-derived from known-good (L/min, Hz) pairs on this pump.

- laguna side: _vfd_stop() used to swallow a timeout or an `ok: False`
  reply into a plain `False` return that pause()/stop()/estop() never
  checked (they only catch exceptions) — those safety verbs could report
  success while the pump kept running. resume() had the matching gap:
  it ignored set_flowrate()/start()'s bool returns. Both now raise/check
  properly (see src/laguna/flow/controller.py and the
  test_*_reports_pump_rejection / test_resume_reports_start_rejection
  tests in tests/test_flow.py).
- confluence side: start_motor()/stop_motor()/clear_faults()/
  send_setpoint() used to catch their own Modbus exceptions and never
  re-raise, so a real communication failure was always reported back to
  laguna as `ok: True` — a false acknowledgment. Now re-raises so a
  genuine comm fault produces `ok: False` (commit 7a6d788 in confluence).

This does NOT touch qin/qaux — pump/VFD only (see example_12 for valves).

NO ACTUATION IS EXECUTED BY DEFAULT. ALLOW_ACTUATION below is False —
with it False, this script only connects, prints status, and disconnects.
Read through the whole thing, confirm the pump is primed/safe to run
briefly at a low flow rate (whatever it's plumbed to — don't run this
dry if the pump can't tolerate that), before setting
ALLOW_ACTUATION = True yourself.

--- Background you need before setting ALLOW_ACTUATION = True ---

1. confluence.service must be running on red.lab and actually able to
   reach the Fuji VFD over Modbus/RS-485 — confirm with
   `journalctl -u confluence -n 50 | grep -i error` first; a comm fault
   will now surface as this script's commands failing loudly (raising)
   instead of silently no-opping, which is the point, but it means a
   real wiring/power issue on the VFD will block this whole script.
2. FREQUENCY_HZ below is a raw VFD drive frequency, not a calibrated flow
   rate — pick a rational, known-safe value for this pump/plumbing
   yourself. This is a functional check (does start/stop/setpoint
   actually reach the drive), not a real flow test.
3. Exercises pause()/resume() and estop() at the end as their own checks,
   since those are the safety verbs this session's confluence fix was
   specifically about making trustworthy.
4. Always stops the pump in a finally block regardless of how far the
   script gets.
"""

import time

from laguna import FlumeLab

CONFIG_FILE = "sandbox/confluence_testing/config.yaml"

FREQUENCY_HZ = 10  # set this — a raw VFD Hz value, see background note 2 above
RUN_TIME_S = 5.0     # how long to hold the pump running before stopping

# Confluence's Fuji_Frenic_VFD status publishes on a cron schedule (every
# 5s in confluence_config.json), not on connect — so a get_status() call
# right after connect() can legitimately see vfd_state: None if no message
# has arrived yet, depending on where in that cycle you connected. Give it
# a full cycle plus margin before treating a still-None status as real.
VFD_STATUS_TIMEOUT_S = 7.0

ALLOW_ACTUATION = True


def wait_for_vfd_status(lab, timeout=VFD_STATUS_TIMEOUT_S, poll_interval=0.2):
    """Poll get_status() until vfd_state is populated, or timeout."""
    deadline = time.monotonic() + timeout
    status = lab.flow.get_status()
    while status.get("vfd_state") is None and time.monotonic() < deadline:
        time.sleep(poll_interval)
        status = lab.flow.get_status()
    return status


def main():
    lab = FlumeLab(config_file=CONFIG_FILE)
    lab.add("flow")

    broker_host = lab.config.get("mqtt")["broker_host"]
    print(f"Connecting to {broker_host}...")
    if not lab.flow.connect():
        raise RuntimeError(f"could not reach the MQTT broker at {broker_host}")

    print(f"Waiting up to {VFD_STATUS_TIMEOUT_S:.0f}s for a VFD status message...")
    status = wait_for_vfd_status(lab)
    print(f"Initial status: {status}")
    if status["vfd_state"] is None:
        print(f"  WARNING: no VFD status arrived within {VFD_STATUS_TIMEOUT_S:.0f}s — "
              "check confluence.service / the Modbus link before proceeding.")

    if not ALLOW_ACTUATION:
        print()
        print("ALLOW_ACTUATION is False — the pump was not commanded.")
        print("Set ALLOW_ACTUATION = True at the top of this file to run the")
        print("real-hardware pump check.")
        lab.flow.disconnect()
        return

    if FREQUENCY_HZ is None:
        lab.flow.disconnect()
        raise ValueError("Set FREQUENCY_HZ at the top of this file to a rational value first.")

    try:
        print(f"\nClearing faults...")
        cleared = lab.flow.clear_faults()
        print(f"  clear_faults() -> {cleared}")

        print(f"\nSetting frequency to {FREQUENCY_HZ} Hz (raw, bypassing L/min calibration)...")
        acked = lab.flow.set_frequency_hz(FREQUENCY_HZ)
        print(f"  set_frequency_hz() -> {acked}")
        if not acked:
            raise RuntimeError("set_frequency_hz() was not acknowledged — aborting before start()")

        print("\nStarting the pump...")
        started = lab.flow.start()
        print(f"  start() -> {started}")
        if not started:
            raise RuntimeError("start() was not acknowledged — aborting")

        print(f"  Running for {RUN_TIME_S:.1f}s — confirm the pump is actually running...")
        time.sleep(RUN_TIME_S)
        print(f"  Status while running: {lab.flow.get_status()}")

        print("\nTesting pause()/resume()...")
        pause_note = lab.flow.pause()
        print(f"  pause() -> {pause_note!r}")
        time.sleep(1.0)
        print(f"  Status while paused: {lab.flow.get_status()}")
        resume_note = lab.flow.resume()
        print(f"  resume() -> {resume_note!r}")
        time.sleep(RUN_TIME_S)
        print(f"  Status after resume: {lab.flow.get_status()}")

        print("\nTesting estop() (stops pump, closes both valves)...")
        estop_note = lab.flow.estop()
        print(f"  estop() -> {estop_note!r}")
        print(f"  Status after estop: {lab.flow.get_status()}")

    finally:
        print("\nStopping the pump (cleanup)...")
        stop_note = lab.flow.stop()
        print(f"  stop() -> {stop_note!r}")
        print(f"Final status: {lab.flow.get_status()}")
        lab.flow.disconnect()


if __name__ == "__main__":
    main()
