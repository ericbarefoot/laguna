"""
Example 7: FlumeLab with gantry + OD2000 + WTT12L PowerProx together

Demonstrates the "simple verb" API added on top of FlumeLab: one lab
instance owns the gantry and both rangefinders, then does a few
move_to()s and acquire_scan()s through it — the same shapes described in
the API-consistency refactor (see docs/GANTRY_UNIT_CALIBRATION.md and
docs/subsystems/rangefinder.md), rather than reaching into
GantryController/TopographicProfiler/MMCCommands directly (still
available via lab.gantry.cmd / lab.gantry.gcode for anything this
convenience layer doesn't cover).

NO MOTION IS EXECUTED BY DEFAULT. ALLOW_MOTION below is False — with it
False, this script only connects, prints status, and disconnects; the
move_to()/acquire_scan() calls further down are never reached. Read
through the whole thing and set ALLOW_MOTION = True yourself once you're
ready to actually move hardware (also requires safe_mode=False on the
gantry connection, handled automatically below from the same flag).

--- Background you need before setting ALLOW_MOTION = True ---

1. Requires the pi_agent transport (gantry_agent.py running on the Pi,
   reached via PiGantryConnection). That is the default in
   config/example_config.yaml; the retired socket_bridge transport could
   not scan at all. This script sets transport: pi_agent explicitly below
   so it stays runnable standalone against an older local config file; a
   real experiment script would just rely on the config default. See
   docs/MACRON_GANTRY.md and
   examples/example_05_gantry_single_axis.py.
2. Requires both rangefinders wired through the AL1342 IO-Link master —
   see docs/MQTT_AL1342_SETUP.md (OD2000) and
   docs/WTT12L_POWERPROX_SETUP.md (WTT12L, via a DP4200 analog bridge —
   no programmatic laser control on that path, hence no activate() call
   for it below). pdin_port defaults here (2 for OD2000, 7 for WTT12L)
   are what was confirmed on the 2026-07-28 session's hardware —
   re-confirm on yours.
3. mm_per_acp_unit (config/example_config.yaml's gantry section, default
   15.0) is a stopgap for this specific machine's uncorrected axis scale,
   not a universal constant — see docs/GANTRY_UNIT_CALIBRATION.md. It's
   what makes the move_to()/acquire_scan() calls below speak real mm.
4. move_to()'s vector form is [X, Y, Z, Theta], matching the axis order
   in config/example_config.yaml's gantry.axes list — reorder the values
   below if your own config's axes list is ordered differently.
5. acquire_scan() infers which axis to scan from the single component
   that differs between `start` and `end` — both are full [X, Y, Z, Theta]
   vectors, not just the scanned axis's value.
6. A fence (keepout zone) is defined below and demonstrated rejecting an
   out-of-bounds move, in the "Fence check" section of main() — that part
   runs BEFORE connect_all() and needs no hardware at all, since fence
   checking is a pure in-memory computation (see
   laguna.robot.macron.fences.TrajectoryChecker) done before any command
   reaches the wire.
"""

import time

from laguna import FlumeLab
from laguna.robot.macron.fences import FenceViolation

# --- Connection settings — adjust for your setup ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "~/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"
AL1342_HOST = "192.168.1.251"
OD2000_PDIN_PORT = 2
WTT12L_PDIN_PORT = 7

# --- Motion AND scanning are off by default. Flip this only when you've
#     decided to actually move something. ---
ALLOW_MOTION = False


def build_lab() -> FlumeLab:
    """Construct a FlumeLab with gantry, od2000, and wtt12l registered
    (not yet connected).

    od2000/wtt12l aren't in the default config/example_config.yaml (they're
    commented-out example blocks there) — added inline here so this script
    runs standalone. In a real experiment, uncomment and adjust those
    blocks in your own config file instead of patching config_dict by hand
    like this.
    """
    lab = FlumeLab("config/example_config.yaml")

    gantry_config = lab.config.config_dict["gantry"]
    gantry_config["transport"] = "pi_agent"
    gantry_config["host"] = PI_HOST
    gantry_config["ssh_user"] = PI_USER
    gantry_config["ssh_key"] = PI_KEY
    gantry_config["remote_serial_device"] = REMOTE_SERIAL_DEVICE

    # Example keepout zone — e.g. a fixed obstacle (equipment stand, camera
    # tripod leg) sitting in the gantry's travel envelope. This one is
    # placed well away from the moves/scans later in this script so it
    # never blocks them; see the "Fence check" section in main() for a
    # deliberately-rejected move that does hit it. Swap for a
    # laguna.robot.macron.fences.CylinderFence if your obstacle has a
    # circular footprint (a post) instead of a rectangular one.
    gantry_config["fences"] = [
        {"type": "box", "name": "equipment_post", "x": [400, 420], "y": [400, 420], "z": [0, 50]},
    ]

    lab.config.config_dict["od2000"] = {
        "topic": "laguna/od2000",
        "pdin_port": OD2000_PDIN_PORT,
        "offset_mm": 0.0,
        "al1342_host": AL1342_HOST,
    }
    lab.config.config_dict["wtt12l"] = {
        "topic": "laguna/wtt12l",
        "pdin_port": WTT12L_PDIN_PORT,
        "offset_mm": 0.0,
        "al1342_host": AL1342_HOST,
    }

    # lab.add("name") looks up the registry (laguna.registry), pulls that
    # section's config, and calls from_config() for you — each rangefinder's
    # from_config() builds its own MqttSubscriber from the shared 'mqtt:'
    # section automatically (see RangefinderSubsystem.from_config()), which
    # is genuinely useful on its own, not just plumbing get_distance_mm()/
    # get_latest_sample() need: get_status() (see the status printout below)
    # surfaces the live topic, sample count, and achieved rate — a quick,
    # human-readable way to confirm the AL1342 is actually publishing and
    # which port/sensor a given reading came from.
    lab.add("gantry").add("od2000").add("wtt12l")

    # This script owns the motion decision, not the config file — set_safe_mode()
    # is the real API for it (rather than mutating gantry_config["safe_mode"]
    # before construction): it's an explicit, auditable verb call, and it
    # correctly no-ops until connect() if called before connecting (see its
    # docstring), same as here.
    lab.gantry.set_safe_mode(not ALLOW_MOTION)

    return lab


def main():
    lab = build_lab()

    # ------------------------------------------------------------------
    # Fence check — no hardware needed for this part at all. The fence
    # check (TrajectoryChecker.check_and_wrap) runs entirely in Python
    # before anything is sent over the wire, so this works even before
    # connect_all(). Deliberately targets a point inside the
    # "equipment_post" keepout zone defined in build_lab().
    # ------------------------------------------------------------------

    print("Fence check demo (runs before connecting to anything):")
    try:
        lab.move_to([410.0, 410.0, 10.0, 0.0])
    except FenceViolation as exc:
        print(f"  Rejected, as expected: {exc}")
    else:
        print("  ERROR: expected a FenceViolation but the move was accepted!")
    print()

    print("Connecting all subsystems (gantry, od2000, wtt12l)...")
    if not lab.connect_all():
        print("Not all subsystems connected — see warnings above.")

    # Give the AL1342's MQTT stream a moment to deliver a message, then
    # show a live reading from each sensor — read-only, no motion.
    print()
    print("Waiting 1s for a live MQTT reading from each sensor...")
    time.sleep(1.0)
    print(f"  od2000 latest (MQTT): {lab.od2000.get_distance_mm()} mm")
    print(f"  wtt12l latest (MQTT): {lab.wtt12l.get_distance_mm()} mm")

    print()
    print("System status:")
    for name, status in lab.get_system_status().items():
        print(f"  {name}: {status}")

    if not ALLOW_MOTION:
        print()
        print("ALLOW_MOTION is False — no moves or scans were run.")
        print("Set ALLOW_MOTION = True at the top of this file to run the")
        print("moves/scans below once you're ready to actually move hardware.")
        lab.disconnect_all()
        return

    # ------------------------------------------------------------------
    # A few moves
    # ------------------------------------------------------------------

    print()
    # Homing is temporarily disabled: physical obstructions currently block
    # several of the limit switches the routine depends on, so
    # lab.gantry.home() raises NotImplementedError rather than jogging into
    # them (see HomingProcedure.home_all). Until that is cleared, declare
    # the reference frame instead — set_position() tells the controller
    # where the gantry already is, and commands no motion:
    #
    #     lab.gantry.set_position([0.0, 0.0, 0.0, 0.0])
    #
    # This example assumes the gantry is already referenced and just moves
    # from wherever it is.
    print("Skipping homing (temporarily disabled — see comment above).")

    print("Moving to (100, 50, 10, 0) mm...")
    lab.move_to([100.0, 50.0, 10.0, 0.0], speed=10.0)

    print("Moving X to 150mm only (Y/Z held at their current position, fence-checked)...")
    lab.move_to(X=150.0, speed=10.0)

    print("Moving Z to 20mm only...")
    lab.move_to(Z=20.0, speed=5.0)

    # ------------------------------------------------------------------
    # A few scans — one per sensor, along X
    # ------------------------------------------------------------------

    print()
    print("Activating OD2000 laser...")
    lab.od2000.activate()

    print("Scanning X 0 -> 100mm with the OD2000...")
    od2000_result = lab.acquire_scan(
        "od2000",
        start=[0.0, 50.0, 20.0, 0.0],
        end=[100.0, 50.0, 20.0, 0.0],
        feed_rate_mm_s=10.0,
        output="experiments/scan_output/example_07_od2000.csv",
    )
    print(f"  -> {od2000_result.path} ({od2000_result.metadata.get('samples')} samples)")

    lab.od2000.deactivate()

    print("Scanning X 100 -> 0mm with the WTT12L PowerProx...")
    wtt12l_result = lab.acquire_scan(
        "wtt12l",
        start=[100.0, 50.0, 20.0, 0.0],
        end=[0.0, 50.0, 20.0, 0.0],
        feed_rate_mm_s=10.0,
        output="experiments/scan_output/example_07_wtt12l.csv",
    )
    print(f"  -> {wtt12l_result.path} ({wtt12l_result.metadata.get('samples')} samples)")

    # Park at the origin so repeated runs start from a known place. Note
    # this is the controller's zero, not wherever this run happened to
    # start.
    print("Returning to the origin...")
    lab.gantry.move_to([0.0, 0.0, 0.0, 0.0], speed=10.0)

    lab.disconnect_all()
    print("Done!")


if __name__ == "__main__":
    # main()
    pass
