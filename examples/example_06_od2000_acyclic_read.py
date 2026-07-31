"""
Example 6: Acyclic (on-demand) distance reads from the OD2000 via the AL1342

"Acyclic" here means a single request/response read of the current
distance, as opposed to the continuous polling loop TopographicProfiler
uses during a scan (gantry_agent.py's _poll_pdin_loop, ~380 Hz over a
persistent connection). For a one-off read like this, plain urllib is
fine — the tight-loop-choking issue documented in
docs/MQTT_AL1342_SETUP.md only shows up under rapid repeated polling with
a fresh TCP connection per request, not a single occasional read.

--- Background you need before running this ---

1. The AL1342 has no MQTT push mechanism worth using for this (only a
   500ms/2Hz-floor timer subscribe — see docs/MQTT_AL1342_SETUP.md). All
   reads here go over plain HTTP POST to the device directly.

2. The AL1342 has no DNS of its own — always address it by raw IP.

3. Addressing: /iolinkmaster/port[N]/iolinkdevice/pdin/getdata, where N is
   the IO-Link port the OD2000 is physically connected to. On the
   2026-07-28 session's hardware this was port 2 — confirm on your own
   setup (see docs/MQTT_AL1342_SETUP.md Step 6 for how the port was found:
   walk ports 1-8, read vendorid/productname on each).

4. The response value is a hex string; decode_od2000_pdin() (this repo's
   confirmed decoder — big-endian int32 nanometers, validated on hardware
   against a physical 808.4mm +/- 0.1mm reference) turns it into
   distance_mm plus the q1/q2 switching-output bits.

5. The OD2000's laser ("Sender" in the IODD) can be switched on/off via
   IO-Link acyclic write — IODD parameter index 97 (0x61), subindex 0.
   Note the inverted convention: value "00" = laser ON (Sender active),
   "01" = laser OFF (Sender not active) — confirmed on hardware
   2026-07-28. If you read 2000.0 mm (or another suspiciously round
   max-range value) with q1=q2=False, that's usually the sensor's
   no-valid-return fallback — check the laser is on first.
"""

from laguna.rangefinder import decode_od2000_pdin
from laguna.rangefinder.al1342 import read_pdin_hex, write_acyclic

# --- Connection settings — adjust for your setup ---
AL1342_IP = "192.168.1.251"
PDIN_PORT = 2


def read_distance(al1342_ip: str = AL1342_IP, pdin_port: int = PDIN_PORT, timeout: float = 5.0) -> dict:
    """Read the OD2000's current distance once, on demand.

    Returns the decoded dict from decode_od2000_pdin(): distance_nm,
    distance_mm, scale, q1, q2. Raises on HTTP/network failure or if the
    AL1342 returns a non-200 code (e.g. 503 if the OD2000 is disconnected
    or the port number is wrong).
    """
    hex_str = read_pdin_hex(al1342_ip, pdin_port, timeout=timeout)
    return decode_od2000_pdin(hex_str)


def set_laser(on: bool, al1342_ip: str = AL1342_IP, pdin_port: int = PDIN_PORT, timeout: float = 5.0) -> None:
    """Turn the OD2000 laser on or off.

    See point 5 in the module docstring — value "00" means ON, "01" means
    OFF, which is backwards from what you'd guess. Raises RuntimeError if
    the AL1342 doesn't accept the write (e.g. wrong pdin_port).
    """
    write_acyclic(al1342_ip, pdin_port, index=97, subindex=0, value="00" if on else "01", timeout=timeout)


def main():
    print("Turning laser ON...")
    set_laser(on=True)

    print(f"Reading OD2000 distance from AL1342 {AL1342_IP}, port {PDIN_PORT}...")
    sample = read_distance()
    print(f"Distance: {sample['distance_mm']:.3f} mm")
    print(f"Q1 (switching output 1): {sample['q1']}")
    print(f"Q2 (switching output 2): {sample['q2']}")
    print(f"Full decoded sample: {sample}")

    print("Turning laser OFF...")
    set_laser(on=False)  # comment this out if you want to leave the laser on for further testing


if __name__ == "__main__":
    main()
