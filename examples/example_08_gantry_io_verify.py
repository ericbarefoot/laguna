"""
Example 8: Live INB digital-input verification (read-only, no motion)

Watches all 8 native digital inputs (INB 1-8) and prints only the bits that
change, with a timestamp, so a human standing at the gantry can toggle each
home/limit switch by hand and confirm it lands on the expected INB index.

This is READ-ONLY. `read_input_bit`/INB is a bare-read command, allowed
under safe_mode=True (see pi_bridge.SAFE_COMMANDS) — safe_mode stays True
throughout this script, no ALLOW_MOTION flag needed.

Run it, then trigger each switch listed below one at a time and confirm the
printed index matches. The expected mapping is IOMap's default (see
docs/MACRON_GANTRY.md's Digital IO section for how it was derived) — not
yet physically toggle-tested switch-by-switch, which is exactly what this
script is for.

    INB 1  X home switch
    INB 2  X limit switch
    INB 3  Y home switch
    INB 4  Y limit switch
    INB 5  Z home switch
    INB 6  Z limit switch
    INB 7  spare / unused
    INB 8  Y brake status

Ctrl-C to stop.
"""

import time

from laguna import FlumeLab

# --- Connection settings — adjust for your setup ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "~/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"

EXPECTED_SIGNAL = {
    1: "X home",
    2: "X limit",
    3: "Y home",
    4: "Y limit",
    5: "Z home",
    6: "Z limit",
    7: "spare",
    8: "Y brake status",
}

POLL_INTERVAL_S = 0.1


def build_lab() -> FlumeLab:
    """Construct a FlumeLab with only the gantry registered, safe_mode on."""
    lab = FlumeLab("config/example_config.yaml")

    gantry_config = lab.config.config_dict["gantry"]
    gantry_config["transport"] = "pi_agent"
    gantry_config["host"] = PI_HOST
    gantry_config["ssh_user"] = PI_USER
    gantry_config["ssh_key"] = PI_KEY
    gantry_config["remote_serial_device"] = REMOTE_SERIAL_DEVICE

    lab.add("gantry")
    lab.gantry.set_safe_mode(True)  # read-only verification — never disabled here

    return lab


def main():
    lab = build_lab()

    print("Connecting to gantry...")
    if not lab.connect_all():
        print("Gantry did not connect — see warnings above.")
        return

    print()
    print("Watching INB 1-8. Trigger each switch by hand and confirm the")
    print("printed index matches the expected signal below:")
    for index, label in EXPECTED_SIGNAL.items():
        print(f"  INB {index}  {label}")
    print()
    print("Ctrl-C to stop.")
    print()

    last = {index: None for index in range(1, 9)}
    try:
        while True:
            for index in range(1, 9):
                value = lab.gantry.cmd.read_input_bit(index)
                if value != last[index]:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] INB {index} ({EXPECTED_SIGNAL[index]}) -> {int(value)}")
                    last[index] = value
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print()
        print("Stopped.")
    finally:
        lab.disconnect_all()


if __name__ == "__main__":
    main()
