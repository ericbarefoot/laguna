"""Standalone gauge publisher script for deployment to Pi via SFTP.

This file is SFTP'd to /tmp/laguna_gauge_publisher.py and run as a standalone
process on the Pi. It reads the Massa water-level gauge over serial and
publishes to MQTT. Requires only pyserial and paho-mqtt on the Pi.

Usage:
    python3 laguna_gauge_publisher.py --port /dev/ttyUSB2 --baud 9600 \
        --broker localhost --rate 1.0 [--offset-mm 0.0]
"""

import argparse
import datetime
import json
import logging
import signal
import sys
import time

logging.basicConfig(
    stream=sys.stderr,
    level=logging.INFO,
    format="%(asctime)s [gauge_publisher] %(levelname)s: %(message)s",
)
logger = logging.getLogger(__name__)

TOPIC_DATA = "laguna/gauge/water_level_mm"
TOPIC_STATUS = "laguna/gauge/status"

_running = True


def _sigterm(signum, frame):
    global _running
    _running = False


def _read_massa(ser) -> dict:
    """Send a poll to the Massa sensor and parse the response.

    The Massa M300 series uses a simple ASCII request/response protocol:
    send '!000R\\r\\n' (for sensor ID 0) and get back a CSV line with
    distance_cm and optional temperature.
    """
    ser.reset_input_buffer()
    ser.write(b"!000R\r\n")
    line = ser.readline().decode("ascii", errors="replace").strip()
    # Expected: "000 R DDDD.D  TT.T  SS" (distance_cm, temp_c, signal)
    parts = line.split()
    if len(parts) < 3:
        raise ValueError(f"unexpected Massa response: {line!r}")
    distance_cm = float(parts[2])
    temperature_c = float(parts[3]) if len(parts) > 3 else None
    signal_strength = float(parts[4]) if len(parts) > 4 else None
    return {
        "distance_cm": distance_cm,
        "temperature_c": temperature_c,
        "signal_strength": signal_strength,
    }


def main():
    parser = argparse.ArgumentParser(description="Massa gauge → MQTT publisher")
    parser.add_argument("--port", required=True, help="Serial device for Massa sensor")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz")
    parser.add_argument("--offset-mm", type=float, default=0.0,
                        help="Elevation (mm) at zero distance; elevation = offset - distance_cm*10")
    parser.add_argument("--client-id", default="laguna_gauge_publisher")
    args = parser.parse_args()

    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, _sigterm)

    try:
        import serial  # type: ignore[import]
    except ImportError:
        logger.error("pyserial is not installed")
        sys.exit(1)

    try:
        import paho.mqtt.client as mqtt  # type: ignore[import]
    except ImportError:
        logger.error("paho-mqtt is not installed")
        sys.exit(1)

    client = mqtt.Client(client_id=args.client_id)
    client.on_connect = lambda c, u, f, rc: logger.info("MQTT connected (rc=%d)", rc)
    client.on_disconnect = lambda c, u, rc: logger.warning("MQTT disconnected (rc=%d)", rc)

    try:
        client.connect(args.broker, args.broker_port, keepalive=60)
    except Exception as e:
        logger.error("Failed to connect to MQTT broker: %s", e)
        sys.exit(1)

    client.loop_start()
    client.publish(TOPIC_STATUS, json.dumps({"status": "online", "port": args.port}))

    try:
        ser = serial.Serial(args.port, baudrate=args.baud, timeout=2.0)
        logger.info("Opened serial port %s at %d baud", args.port, args.baud)
    except Exception as e:
        logger.error("Failed to open serial port %s: %s", args.port, e)
        client.publish(TOPIC_STATUS, json.dumps({"status": "error", "reason": str(e)}))
        client.loop_stop()
        client.disconnect()
        sys.exit(1)

    interval = 1.0 / max(args.rate, 0.01)
    logger.info("Publishing at %.2f Hz (interval %.3f s)", args.rate, interval)

    try:
        while _running:
            t_start = time.time()
            wall_time = time.time()
            wall_iso = datetime.datetime.utcfromtimestamp(wall_time).isoformat() + "Z"
            try:
                reading = _read_massa(ser)
                elevation_mm = args.offset_mm - reading["distance_cm"] * 10.0
                payload = {
                    "wall_time": wall_time,
                    "wall_time_iso": wall_iso,
                    "distance_cm": reading["distance_cm"],
                    "elevation_mm": elevation_mm,
                    "temperature_c": reading["temperature_c"],
                    "signal_strength": reading["signal_strength"],
                }
                client.publish(TOPIC_DATA, json.dumps(payload))
            except Exception as e:
                logger.warning("Read error: %s", e)

            elapsed = time.time() - t_start
            sleep_s = max(0.0, interval - elapsed)
            time.sleep(sleep_s)
    finally:
        client.publish(TOPIC_STATUS, json.dumps({"status": "offline", "port": args.port}))
        try:
            ser.close()
        except Exception:
            pass
        client.loop_stop()
        client.disconnect()
        logger.info("Gauge publisher exiting")


if __name__ == "__main__":
    main()
