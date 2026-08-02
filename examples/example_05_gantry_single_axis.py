"""
Example 5: Single-axis gantry moves via GantryController

A minimal tutorial for commanding the Macron gantry from Python using
GantryController's simple verbs (move_to(), home(), etc.) instead of raw
ASCII. Useful for quick manual testing, calibration, and one-off moves.

--- Background you need before running this ---

1. Transport: this uses PiGantryConnection ("pi_agent" transport), which
   is also the default. PiGantryConnection SFTPs and launches
   gantry_agent.py on the Pi over SSH; that process owns the BLC serial
   port for the whole session (interactive commands AND full topographic
   scans both go through it — see docs/MACRON_GANTRY.md). Make sure
   nothing else already holds the serial port first — a stale agent from
   an interrupted session, or a tio terminal. Two writers interleave
   bytes mid-command and leave the controller parsing garbage. Check with:
       ssh oak@red.lab 'fuser /dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0'

2. safe_mode: PiGantryConnection defaults to safe_mode=True, which allows
   only read-only queries (ACP, INB, MIF, etc — see SAFE_COMMANDS in
   pi_bridge.py). Any motion or output-setting command (SPD with a value,
   BMT, BST, SOB) is rejected both client-side and agent-side (defense in
   depth) unless safe_mode=False. Treat safe_mode=False as a deliberate,
   one-off decision for a specific connection — not something to leave on.

3. Axis indices (from config/example_config.yaml, confirmed on hardware):
       X = A1
       Y = A2  (has a brake — see below)
       Z = A5  (has a brake — see below)
       Theta = A6  (rotary, no soft limits — PLT/NLT time out on it)

   Theta's ACP units are NOT degrees, and are NOT affected by
   MM_PER_ACP_UNIT below (that conversion only applies to X/Y/Z — see
   MMCCommands._is_linear). On the 2026-07-28 session's specific hardware,
   rotating between two limit-switch trigger points (observed physically,
   since the switch's status isn't ASCII-readable from here — see IOMap in
   commands.py) measured ~10.365 ACP units per revolution. That number is
   specific to that one machine's gearing/encoder config — re-measure it
   yourself the same way before trusting it on any other setup.

4. Brakes (Y and Z only): before moving Y or Z, its brake must be
   disengaged or the motor will stall against it. Z's brake is a fail-safe,
   spring-engaged design (SOB ON = power applied = brake released) — if Z
   carries any load or isn't otherwise supported, disengaging it can let
   the axis drop under gravity. Don't disengage Z's brake without first
   confirming what will happen physically. Y's brake status readback (INB
   8) was found unreliable on 2026-07-28 hardware (write succeeded and was
   physically confirmed by the brake's audible click, but the readback
   didn't change to match) — confirm brake state by ear/eye, not by
   trusting the status read. Z's brake status is not ASCII-readable at all
   (lives on the responder PLC node).

5. THE ACP UNIT IS NOT A MILLIMETER. Confirmed on hardware 2026-07-28 by
   ruler-measured manual moves: 10 ACP units on Z = 150mm actual travel, 20
   ACP units on X = 300mm, 20 ACP units on Y = 300mm — all three give the
   identical ratio, 15 mm per ACP unit. This is now handled by
   GantryController/MMCCommands themselves (via the mm_per_unit param
   below), not by this example — see docs/GANTRY_UNIT_CALIBRATION.md for
   the finding writeup. If the Snap2Motion/DSM project's axis scale is
   fixed at the source, change MM_PER_ACP_UNIT below to 1.0 (and
   gantry.mm_per_acp_unit in config/example_config.yaml) — nothing else
   needs to change.

--- Usage ---

Reading positions is always safe to run (default safe_mode=True). Running
an actual move requires setting ALLOW_MOTION = True below AND passing
safe_mode=False when connecting — two separate, deliberate steps.
"""

from laguna.robot.macron.commands import THETA_AXIS, X_AXIS, Y_AXIS, Z_AXIS
from laguna.robot.macron.controller import GantryController
from laguna.robot.macron.pi_bridge import PiGantryConnection

# --- Connection settings — adjust for your setup ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "/home/eric/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"
REMOTE_BAUD = 9600

# --- Unit conversion — confirmed on hardware 2026-07-28, see point 5 above
# and docs/GANTRY_UNIT_CALIBRATION.md. The single toggle: flip to 1.0 once
# the DSM project's axis scale is fixed at the source. ---
MM_PER_ACP_UNIT = 15.0

# --- Motion is off by default. Flip this to True only when you've decided
#     to actually move something. ---
ALLOW_MOTION = False


def connect(safe_mode: bool = True) -> GantryController:
    """Open a connection to the gantry agent and wrap it in a GantryController.

    safe_mode=True (default): only read-only queries are allowed.
    safe_mode=False: motion/output commands are allowed — pass this
    deliberately, never as a default.
    """
    conn = PiGantryConnection(
        host=PI_HOST,
        ssh_user=PI_USER,
        ssh_key=PI_KEY,
        remote_serial_device=REMOTE_SERIAL_DEVICE,
        remote_baud=REMOTE_BAUD,
        safe_mode=safe_mode,
    )
    controller = GantryController(connection=conn, mm_per_unit=MM_PER_ACP_UNIT)
    controller.connect()
    return controller


def get_all_positions(gantry: GantryController) -> dict:
    """Read every axis: X/Y/Z in real mm, Theta in its own raw rotary units
    (see docstring point 3 — MM_PER_ACP_UNIT does not apply to Theta)."""
    return {
        axis.name: gantry.cmd.get_actual_position(axis)
        for axis in (X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS)
    }


def zero_axis(gantry: GantryController, axis) -> None:
    """Zero the current position of an axis in the controller.

    This is a relative zeroing — it does not move the axis, it just sets
    the current ACP value to zero. Safe under safe_mode.
    """
    gantry.cmd.set_actual_position(axis, 0.0)


def set_brake(gantry: GantryController, axis, disengaged: bool) -> None:
    """Engage/disengage the Y or Z brake. Requires safe_mode=False.

    See point 4 in the module docstring before disengaging Z's brake.
    """
    if disengaged:
        gantry.cmd.disengage_brake(axis, gantry._io_map)
    else:
        gantry.cmd.engage_brake(axis, gantry._io_map)


def stop_axis(gantry: GantryController, axis) -> None:
    """Send an immediate stop (BST) to one axis. Requires safe_mode=False.

    Useful to call from a second connection while a move issued via
    gantry.move_to() is still blocking on another one — BST takes effect
    immediately, independent of any particular connection.
    """
    gantry.cmd.begin_stop(axis)


def main():
    print("=== Read-only position check (safe_mode=True) ===")
    print(f"(X/Y/Z in real mm, {MM_PER_ACP_UNIT} mm/unit conversion applied; Theta in raw units)")
    gantry = connect(safe_mode=True)
    try:
        positions = get_all_positions(gantry)
        for name, pos in positions.items():
            unit = "mm" if name != "Theta" else "units"
            print(f"{name:6s}: {pos:.3f} {unit}")
    finally:
        gantry.disconnect()

    if not ALLOW_MOTION:
        print()
        print("ALLOW_MOTION is False — skipping the move example.")
        print("Set ALLOW_MOTION = True at the top of this file to run it.")
        return

    print()
    print("=== Single-axis move example (safe_mode=False) ===")
    gantry = connect(safe_mode=False)
    try:
        current = gantry.cmd.get_actual_position(X_AXIS)  # real mm
        target = current - 2.0  # small 2mm move — adjust as needed
        print(f"Moving X from {current:.3f} mm to {target:.3f} mm at 1 mm/s...")
        # move_to() is fence-checked (routed through the coordinated gcode
        # path — it reads Y/Z's real current position to do so) and blocks
        # until the move completes.
        gantry.move_to(X=target, speed=1.0)
        final = gantry.cmd.get_actual_position(X_AXIS)
        print(f"Done. Final X position: {final:.3f} mm")
    finally:
        gantry.disconnect()


if __name__ == "__main__":
    main()
