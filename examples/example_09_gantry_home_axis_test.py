"""
Example 9: Per-axis homing test (REAL MOTION)

Homes one axis at a time, prompting for confirmation before each one, so a
human at the gantry can watch every pass and stop between axes. Homing
itself jogs an axis toward its home switch — see HomingProcedure's module
docstring (src/laguna/robot/macron/homing.py) for the full sequence.

--- Stopping ---

Ctrl-C at any point — during the confirmation prompt or mid-jog — is caught
and immediately calls lab.gantry.stop(): every configured axis decelerates
on its own ramp (BST) and Y/Z brakes park. That is the "stop" tier of the
safety vocabulary (controlled, not an instant abort) — see
src/laguna/safety.py's module docstring. If a harder halt is ever needed,
that's lab.gantry.estop() from a REPL, not something this script reaches
for on its own.

--- Before running this for real ---

NO MOTION IS EXECUTED BY DEFAULT. ALLOW_MOTION below is False — with it
False, this script only connects and exits; homing is never reached. Flip
it to True only once you've confirmed (via
examples/example_08_gantry_io_verify.py) that the home switches actually
toggle the INB channels IOMap expects, and you're physically present at
the gantry able to hit Ctrl-C.

1. Requires the pi_agent transport, same as example_07/08.
2. Homing direction, which switch to poll (home vs. limit), and trip
   polarity all come from config/example_config.yaml's gantry.axes
   entries (home_switch / home_trip_on_high) — see
   GantryController._build_homing_config. Confirm those match what
   example_08 showed on your hardware before trusting this script.
   Homing is software-polled (jog + poll INB), not hardware capture-latch
   — see homing.py's module docstring for why.
3. AXES_TO_HOME defaults to Z, X, Y (Z first) to avoid the instrument
   crashing into the bed during a lateral search — see homing.py's module
   docstring for why. Don't reorder this without a reason.
"""

from laguna import FlumeLab

# --- Connection settings — adjust for your setup ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "~/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"

# --- Motion is off by default. Flip this only when you've decided to
#     actually move something, and read the docstring above first. ---
ALLOW_MOTION = False

AXES_TO_HOME = ["Z", "X", "Y"]  # Z first — see docstring above


def build_lab() -> FlumeLab:
    """Construct a FlumeLab with only the gantry registered."""
    lab = FlumeLab("config/example_config.yaml")

    gantry_config = lab.config.config_dict["gantry"]
    gantry_config["transport"] = "pi_agent"
    gantry_config["host"] = PI_HOST
    gantry_config["ssh_user"] = PI_USER
    gantry_config["ssh_key"] = PI_KEY
    gantry_config["remote_serial_device"] = REMOTE_SERIAL_DEVICE

    lab.add("gantry")
    lab.gantry.set_safe_mode(not ALLOW_MOTION)

    return lab


def main():
    lab = build_lab()

    print("Connecting to gantry...")
    if not lab.connect_all():
        print("Gantry did not connect — see warnings above.")
        return

    if not ALLOW_MOTION:
        print()
        print("ALLOW_MOTION is False — nothing will move. Set it True at the")
        print("top of this file once you're ready, and re-read the docstring.")
        lab.disconnect_all()
        return

    axes_by_name = {axis.name: axis for axis in lab.gantry._axes}

    try:
        for name in AXES_TO_HOME:
            axis = axes_by_name.get(name)
            if axis is None:
                print(f"Skipping {name} — not in this gantry's configured axes.")
                continue

            answer = input(f"Home {name} now? [y/N] ").strip().lower()
            if answer != "y":
                print(f"  Skipped {name}.")
                continue

            print(f"Homing {name}... (Ctrl-C stops motion immediately)")
            # lab.gantry.home_axis(), not lab.gantry.homing.home_axis() —
            # the wrapper also resyncs GCodeExecutor's cached position
            # afterward, which move_to() needs to plan correctly next.
            final_pos = lab.gantry.home_axis(axis)
            print(f"  {name} homed. Standoff position: {final_pos:.3f} mm")
    except KeyboardInterrupt:
        print()
        print("Ctrl-C — stopping all motion now.")
        lab.gantry.stop()
    finally:
        lab.disconnect_all()
        print("Disconnected.")


if __name__ == "__main__":
    main()
