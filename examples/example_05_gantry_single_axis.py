"""
Example 5: Single-axis gantry moves via PiGantryConnection

A minimal tutorial + toolkit for commanding one axis of the Macron gantry
directly from Python, without going through FlumeLab/GantryController.
Useful for quick manual testing, calibration, and one-off moves — this is
exactly the pattern used interactively during the 2026-07-28 hardware
session that first exercised the rewritten gantry_agent.py.

--- Background you need before running this ---

1. Transport: this uses PiGantryConnection ("pi_agent" transport), NOT the
   default "socket_bridge". PiGantryConnection SFTPs and launches
   gantry_agent.py on the Pi over SSH; that process owns the BLC serial
   port for the whole session (interactive commands AND full topographic
   scans both go through it — see docs/MACRON_GANTRY.md). Make sure
   serial_bridge.py is NOT running on the Pi first — the two cannot
   coexist, since serial_bridge.py opens the serial device once at
   startup and never releases it. Check with:
       ssh oak@red.lab 'pgrep -af serial_bridge.py'

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

   Theta's ACP units are NOT degrees. On the 2026-07-28 session's specific
   hardware, rotating between two limit-switch trigger points (observed
   physically, since the switch's status isn't ASCII-readable from here —
   see IOMap in commands.py) measured ~10.365 ACP units per revolution.
   That number is specific to that one machine's gearing/encoder config —
   re-measure it yourself the same way before trusting it on any other
   setup: rotate slowly, note the ACP value where a fixed reference point
   repeats, and take the difference between two consecutive triggers.

4. Brakes (Y and Z only): before moving Y or Z, its brake must be
   disengaged or the motor will stall against it.
       Y disengage: SOB 4 1     Y engage: SOB 4 0
       Z disengage: SOB 5 1     Z engage: SOB 5 0
   Y's brake status is readable back via INB 8 — though this was found to
   be unreliable/possibly mis-wired on 2026-07-28 hardware (write
   succeeded and was physically confirmed by the brake's audible click,
   but the INB 8 readback didn't change to match). Z's brake status is not
   ASCII-readable at all (lives on the responder PLC node). Confirm brake
   state by ear/eye, not by trusting the status read.

   Z's brake is a fail-safe, spring-engaged design (SOB ON = power applied
   = brake released). If Z carries any load or isn't otherwise supported,
   disengaging it can let the axis drop under gravity. Don't disengage
   Z's brake without first confirming what will happen physically.

5. The MIF gotcha: MIF (move-in-progress flag) returns floats like
   "1.000"/"0.000", not bare "1"/"0". Compare with float(mif) == 1.0, not
   mif == "1" — a bare string comparison silently never matches, so your
   poll loop spins until it times out even though the move already
   finished. This bit the 2026-07-28 session once; move_axis() below
   already does it correctly.

6. THE ACP UNIT IS NOT A MILLIMETER. Confirmed on hardware 2026-07-28 by
   ruler-measured manual moves: 10 ACP units on Z = 150mm actual travel,
   20 ACP units on X = 300mm, 20 ACP units on Y = 300mm — all three give
   the identical ratio, 15 mm per ACP unit. Every "mm" position/speed
   used earlier in this session (before this was caught) was actually 15x
   larger in real distance than reported. get_position()/move_axis() below
   now convert automatically for X/Y/Z (NOT Theta — that's rotary, a
   different conversion entirely, see point 3). This is a stopgap patched
   into this example script only — it is NOT yet applied anywhere else in
   the codebase (gantry_agent.py's scan math, TopographicProfiler, the
   fence/homing config, etc. all still assume raw ACP units). See
   docs/GANTRY_UNIT_CALIBRATION.md for the finding writeup and the plan
   for a real fix — likely reconfiguring the axis scale factor inside the
   Snap2Motion/DSM project itself (so 1 ACP unit = 1 mm at the source)
   rather than compensating in software everywhere downstream.

--- Usage ---

Reading positions is always safe to run (default safe_mode=True). Running
an actual move requires setting ALLOW_MOTION = True below AND passing
safe_mode=False when connecting — two separate, deliberate steps.
"""

import time

from laguna.robot.macron.pi_bridge import PiGantryConnection

# --- Connection settings — adjust for your setup ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "/home/eric/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"
REMOTE_BAUD = 9600

# --- Axis map — confirmed on hardware 2026-07-28 ---
AXES = {"X": 1, "Y": 2, "Z": 5, "Theta": 6}
LINEAR_AXES = ("X", "Y", "Z")  # Theta is rotary — no mm conversion applies to it
BRAKE_OUTPUTS = {"Y": 4, "Z": 5}   # SOB index
BRAKE_STATUS_INPUTS = {"Y": 8}    # INB index — Z has no ASCII-readable status; see docstring

# --- Unit conversion — confirmed on hardware 2026-07-28, see point 6 above ---
# and docs/GANTRY_UNIT_CALIBRATION.md. Ruler-measured: 10 units on Z = 150mm,
# 20 units on X = 300mm, 20 units on Y = 300mm. Same ratio on all three.
MM_PER_ACP_UNIT = 15.0


def units_to_mm(units: float) -> float:
    """Convert raw ACP units to real-world mm. X/Y/Z only — not Theta."""
    return units * MM_PER_ACP_UNIT


def mm_to_units(mm: float) -> float:
    """Convert real-world mm to raw ACP units. X/Y/Z only — not Theta."""
    return mm / MM_PER_ACP_UNIT

# --- Motion is off by default. Flip this to True only when you've decided
#     to actually move something. ---
ALLOW_MOTION = False


def connect(safe_mode: bool = True) -> PiGantryConnection:
    """Open a connection to the gantry agent.

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
    conn.connect()
    return conn


def get_position_raw_units(conn: PiGantryConnection, axis: str) -> float:
    """Read the current actual position (ACP) in raw controller units,
    with no mm conversion applied. Use this for Theta; use get_position()
    for X/Y/Z. Safe under safe_mode.
    """
    idx = AXES[axis]
    return float(conn.send(f"A{idx} ACP"))


def get_position(conn: PiGantryConnection, axis: str) -> float:
    """Read the current position of an X/Y/Z axis, in real mm.

    Applies the MM_PER_ACP_UNIT conversion (point 6 in the module
    docstring) — do not call this for Theta, which is rotary and has no
    mm meaning; use get_position_raw_units() for Theta instead.
    Safe under safe_mode.
    """
    if axis not in LINEAR_AXES:
        raise ValueError(f"{axis!r} is not a linear axis — use get_position_raw_units() for Theta")
    return units_to_mm(get_position_raw_units(conn, axis))


def get_all_positions(conn: PiGantryConnection) -> dict:
    """Read every axis: X/Y/Z in real mm, Theta in raw units (see docstring point 3)."""
    positions = {name: get_position(conn, name) for name in LINEAR_AXES}
    positions["Theta"] = get_position_raw_units(conn, "Theta")
    return positions


def set_brake(conn: PiGantryConnection, axis: str, disengaged: bool) -> None:
    """Engage/disengage the Y or Z brake. Requires safe_mode=False.

    See point 4 in the module docstring before disengaging Z's brake.
    """
    if axis not in BRAKE_OUTPUTS:
        raise ValueError(f"No brake configured for axis {axis!r} (only Y, Z have brakes)")
    idx = BRAKE_OUTPUTS[axis]
    conn.send(f"SOB {idx} {1 if disengaged else 0}")


def move_axis(
    conn: PiGantryConnection,
    axis: str,
    target: float,
    speed: float,
    poll_interval: float = 0.2,
    timeout: float = 30.0,
) -> float:
    """Move an X/Y/Z axis to an absolute target position (mm) at the given
    speed (mm/s). NOT for Theta — its ACP units aren't mm at all (rotary,
    ~10.365 units/revolution as measured, see point 3); command it in raw
    units directly via conn.send() if needed.

    Requires safe_mode=False. Blocks until the move completes (or raises
    TimeoutError) by polling MIF. Returns the final position in mm.

    target/speed here are real mm / mm-per-second — converted to raw ACP
    units internally (point 6 in the module docstring) before being sent
    as SPD/BMT.
    """
    if axis not in LINEAR_AXES:
        raise ValueError(f"{axis!r} is not a linear axis — move it via raw conn.send() calls instead")

    idx = AXES[axis]
    raw_target = mm_to_units(target)
    raw_speed = mm_to_units(speed)  # mm/s -> units/s, same ratio applies to speed
    conn.send(f"A{idx} SPD {raw_speed}")
    conn.send(f"A{idx} BMT {raw_target}")

    deadline = time.monotonic() + timeout
    while True:
        mif = float(conn.send(f"A{idx} MIF"))  # compare as float — see point 5 above
        if mif == 1.0:
            break
        if time.monotonic() > deadline:
            raise TimeoutError(f"Move on axis {axis!r} did not complete within {timeout}s")
        time.sleep(poll_interval)

    return get_position(conn, axis)


def stop_axis(conn: PiGantryConnection, axis: str) -> None:
    """Send an immediate stop (BST) to one axis. Requires safe_mode=False.

    Useful to call from a second connection while a move from move_axis()
    is still blocking on another one — BST takes effect immediately,
    independent of any particular connection.
    """
    idx = AXES[axis]
    conn.send(f"A{idx} BST")


def main():
    print("=== Read-only position check (safe_mode=True) ===")
    print(f"(X/Y/Z in real mm, {MM_PER_ACP_UNIT} mm/unit conversion applied; Theta in raw units)")
    conn = connect(safe_mode=True)
    try:
        positions = get_all_positions(conn)
        for name, pos in positions.items():
            unit = "mm" if name in LINEAR_AXES else "units"
            print(f"{name:6s}: {pos:.3f} {unit}")
    finally:
        conn.disconnect()

    if not ALLOW_MOTION:
        print()
        print("ALLOW_MOTION is False — skipping the move example.")
        print("Set ALLOW_MOTION = True at the top of this file to run it.")
        return

    print()
    print("=== Single-axis move example (safe_mode=False) ===")
    conn = connect(safe_mode=False)
    try:
        axis = "X"
        current = get_position(conn, axis)  # real mm
        target = current - 2.0  # small 2mm move — adjust as needed
        print(f"Moving {axis} from {current:.3f} mm to {target:.3f} mm at 1 mm/s...")
        final = move_axis(conn, axis, target=target, speed=1.0)
        print(f"Done. Final {axis} position: {final:.3f} mm")
    finally:
        conn.disconnect()


if __name__ == "__main__":
    main()
