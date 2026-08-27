"""
Example 10: Reading water level from the confluence node over MQTT

The gauge subsystem is an MQTT client of the confluence node on red.lab,
not a direct serial connection — see src/laguna/gauge/sensor.py and
src/laguna/mqtt/subscriber.py. confluence owns the Massa ultrasonic
sensor's USB serial port and publishes readings on
"{node_name}/Massa_Ultrasonic"; this just subscribes and decodes.

--- Background you need before running this ---

1. confluence_daemon.py must already be running on red.lab (as the
   confluence.service systemd unit — see confluence/confluence.service)
   and actually connected to the Massa sensor. If it isn't, connect()
   below will still succeed (that's just the MQTT transport), but
   read_mm() will raise RuntimeError forever since no message ever
   arrives.

2. mqtt.node_name must match confluence_config.json's "Node Name" on
   red.lab exactly — gauge.topic is derived from it
   (f"{node_name}/Massa_Ultrasonic"). See config.py's comment above its
   "weir" default section for why this is one shared value instead of
   being duplicated per subsystem.

3. Readings arrive asynchronously at whatever interval confluence's
   Massa_Ultrasonic "publish" schedule uses (0.1s in the current
   confluence_config.json) — read_mm() does not trigger a fresh hardware
   read like the old serial version did, it just returns the most
   recently delivered MQTT message. Give it a moment after connect()
   before reading.
"""

import time

from laguna import FlumeLab

# See config/example_gauge_mqtt.yaml — mqtt.broker_host/node_name and
# gauge.offset_mm live there; adjust it for your setup rather than this file.
CONFIG_FILE = "config/example_gauge_mqtt.yaml"


def main():
    lab = FlumeLab(config_file=CONFIG_FILE)
    lab.add("gauge")

    broker_host = lab.config.get("mqtt")["broker_host"]
    print(f"Connecting to {broker_host}...")
    if not lab.gauge.connect():
        raise RuntimeError(f"could not reach the MQTT broker at {broker_host}")

    print("Waiting for a reading...")
    time.sleep(1.0)

    elevation_mm = lab.gauge.read_mm()
    print(f"Water elevation: {elevation_mm:.2f} mm")
    print(f"Full status: {lab.gauge.get_status()}")

    lab.gauge.disconnect()


if __name__ == "__main__":
    main()
