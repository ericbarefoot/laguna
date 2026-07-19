"""Standalone topographic scan script for deployment to Pi via SFTP.

This file is SFTP'd to /tmp/laguna_scan_runner.py and run as a standalone
process on the Pi. It handles all BLC serial communication AND subscribes to
the OD2000 MQTT topic — everything runs in one Pi-local clock domain.
Requires only pyserial and paho-mqtt on the Pi.

Wire protocol (newline-delimited JSON):
  stdout → laguna PC:
    {"ready": true, "start_pos_mm": ..., "accel_mm_s2": ..., "decel_mm_s2": ...}
    {"done": true, "csv_path": "...", "actual_start_mm": ...,
     "actual_end_mm": ..., "samples": N}
    {"error": "..."}
  stdin → scan_runner (optional):
    {"op": "cancel"}   # triggers BST then clean exit

Usage:
    python3 laguna_scan_runner.py \
        --serial-port /dev/ttyUSB0 --baud 9600 \
        --axis A1 \
        --end-mm 500.0 --feed-rate-mm-s 5.0 \
        --od2000-topic laguna/od2000 --pdin-port 1 \
        --output /tmp/profile_20260718_143200.csv
"""

import argparse
import csv
import datetime
import json
import logging
import queue
import select
import signal
import sys
import time
import threading

logging.basicConfig(
    stream=sys.stderr,
    level=logging.INFO,
    format="%(asctime)s [scan_runner] %(levelname)s: %(message)s",
)
logger = logging.getLogger(__name__)

_cancel_requested = False
_mqtt_samples: "queue.Queue[dict]" = queue.Queue()


def _sigterm(signum, frame):
    global _cancel_requested
    _cancel_requested = True


def _emit(obj: dict) -> None:
    print(json.dumps(obj), flush=True)


def _serial_send(ser, command: str, timeout: float = 2.0) -> str:
    """Send one ASCII command to the BLC and return the stripped response line."""
    ser.reset_input_buffer()
    ser.write((command + "\r\n").encode("ascii"))
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        buf += chunk
        if b">" in buf:
            # BLC response ends with ' >'
            break
    response = buf.decode("ascii", errors="replace").strip()
    # Strip the echoed command and prompt; response is the token before '>'
    # Typical: "A1 ACP\r\n 100.000 >"  → "100.000"
    parts = response.split()
    # Drop prefix tokens that match the command words
    cmd_tokens = command.split()
    while parts and parts[0].upper() in [t.upper() for t in cmd_tokens]:
        parts.pop(0)
    # The numeric value is the first remaining token; discard trailing '>'
    value = parts[0].rstrip(">").strip() if parts else ""
    return value


def _decode_pdin(hex_str: str, pdin_port: int) -> dict:
    """Decode OD2000 7002T15 6-byte PDIN hex string."""
    raw = bytes.fromhex(hex_str)
    distance_nm = int.from_bytes(raw[0:4], "big", signed=True)
    return {
        "distance_nm": distance_nm,
        "distance_mm": distance_nm / 1_000_000,
        "scale": raw[4],
        "q1": bool(raw[5] & 0x01),
        "q2": bool(raw[5] & 0x02),
    }


def _extract_pdin_hex(payload: dict, pdin_port: int) -> str:
    """Extract pdin hex string from AL1342 MQTT event payload."""
    key = f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/pdin"
    return payload["data"]["payload"][key]["data"]


def _on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        wall_time = time.time()
        _mqtt_samples.put({"wall_time": wall_time, "payload": payload})
    except Exception as e:
        logger.debug("Failed to parse MQTT message: %s", e)


def _check_stdin_cancel() -> bool:
    """Non-blocking check of stdin for a cancel message."""
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if ready:
        try:
            line = sys.stdin.readline().strip()
            if line:
                msg = json.loads(line)
                if msg.get("op") == "cancel":
                    return True
        except Exception:
            pass
    return False


def main():
    parser = argparse.ArgumentParser(description="Topographic scan runner (Pi-local)")
    parser.add_argument("--serial-port", required=True)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--axis", required=True, help="Axis prefix, e.g. A1")
    parser.add_argument("--end-mm", type=float, required=True)
    parser.add_argument("--feed-rate-mm-s", type=float, required=True)
    parser.add_argument("--od2000-topic", default="laguna/od2000")
    parser.add_argument("--pdin-port", type=int, default=1, help="IO-Link port OD2000 is on")
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--broker-port", type=int, default=1883)
    args = parser.parse_args()

    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, _sigterm)

    try:
        import serial  # type: ignore[import]
    except ImportError:
        _emit({"error": "pyserial is not installed on Pi"})
        sys.exit(1)

    try:
        import paho.mqtt.client as mqtt  # type: ignore[import]
    except ImportError:
        _emit({"error": "paho-mqtt is not installed on Pi"})
        sys.exit(1)

    # ------------------------------------------------------------------ serial
    try:
        ser = serial.Serial(args.serial_port, baudrate=args.baud, timeout=2.0)
        logger.info("Opened %s at %d baud", args.serial_port, args.baud)
    except Exception as e:
        _emit({"error": f"Failed to open serial port {args.serial_port}: {e}"})
        sys.exit(1)

    ax = args.axis  # e.g. "A1"

    try:
        start_raw = _serial_send(ser, f"{ax} ACP")
        start_pos_mm = float(start_raw)
        acl_raw = _serial_send(ser, f"{ax} ACL")
        accel_mm_s2 = float(acl_raw)
        dcl_raw = _serial_send(ser, f"{ax} DCL")
        decel_mm_s2 = float(dcl_raw)
    except Exception as e:
        ser.close()
        _emit({"error": f"Failed to query axis state: {e}"})
        sys.exit(1)

    # ------------------------------------------------------------------ MQTT
    mqtt_client = mqtt.Client(client_id="laguna_scan_runner")
    mqtt_client.on_message = _on_message
    mqtt_client.on_connect = lambda c, u, f, rc: (
        c.subscribe(args.od2000_topic) if rc == 0
        else logger.error("MQTT connect failed rc=%d", rc)
    )

    try:
        mqtt_client.connect(args.broker, args.broker_port, keepalive=60)
    except Exception as e:
        ser.close()
        _emit({"error": f"Failed to connect to MQTT broker: {e}"})
        sys.exit(1)

    mqtt_client.loop_start()
    # Short settle so subscription is registered before we start the move
    time.sleep(0.2)

    _emit({
        "ready": True,
        "start_pos_mm": start_pos_mm,
        "accel_mm_s2": accel_mm_s2,
        "decel_mm_s2": decel_mm_s2,
    })

    # ------------------------------------------------------------------ move
    records = []  # {"wall_time": float, "distance_nm": int, ...}

    try:
        _serial_send(ser, f"{ax} SPD {args.feed_rate_mm_s}")
        _serial_send(ser, f"{ax} BMT {args.end_mm}")
        t_move_start = time.time()
        logger.info("Move started: %s → %.3f mm at %.3f mm/s", ax, args.end_mm, args.feed_rate_mm_s)

        # Drain any MQTT samples that arrived before the move (clear queue)
        while not _mqtt_samples.empty():
            try:
                _mqtt_samples.get_nowait()
            except queue.Empty:
                break

        # Poll MIF until done; drain MQTT samples; watch for cancel
        while True:
            if _cancel_requested or _check_stdin_cancel():
                logger.info("Cancel requested — sending BST")
                _serial_send(ser, f"{ax} BST")
                break

            # Drain any queued MQTT samples
            while True:
                try:
                    item = _mqtt_samples.get_nowait()
                    wall_time = item["wall_time"]
                    try:
                        hex_str = _extract_pdin_hex(item["payload"], args.pdin_port)
                        decoded = _decode_pdin(hex_str, args.pdin_port)
                        decoded["wall_time"] = wall_time
                        records.append(decoded)
                    except Exception as e:
                        logger.debug("PDIN decode error: %s", e)
                except queue.Empty:
                    break

            mif_raw = _serial_send(ser, f"{ax} MIF")
            if mif_raw.strip() == "1":
                break
            time.sleep(0.1)

        t_move_done = time.time()

        # Final drain after move complete
        while True:
            try:
                item = _mqtt_samples.get_nowait()
                wall_time = item["wall_time"]
                if wall_time <= t_move_done:
                    try:
                        hex_str = _extract_pdin_hex(item["payload"], args.pdin_port)
                        decoded = _decode_pdin(hex_str, args.pdin_port)
                        decoded["wall_time"] = wall_time
                        records.append(decoded)
                    except Exception as e:
                        logger.debug("PDIN decode error: %s", e)
            except queue.Empty:
                break

        actual_end_raw = _serial_send(ser, f"{ax} ACP")
        actual_end_mm = float(actual_end_raw)

    except Exception as e:
        ser.close()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        _emit({"error": f"Error during scan: {e}"})
        sys.exit(1)
    finally:
        ser.close()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

    # ------------------------------------------------------------------ fuse
    ramp_t_accel = args.feed_rate_mm_s / accel_mm_s2 if accel_mm_s2 > 0 else 0.0
    ramp_t_decel = args.feed_rate_mm_s / decel_mm_s2 if decel_mm_s2 > 0 else 0.0
    t_slew_start = t_move_start + ramp_t_accel
    t_slew_end = t_move_done - ramp_t_decel

    csv_rows = []
    for rec in records:
        t = rec["wall_time"]
        in_ramp = not (t_slew_start <= t <= t_slew_end)
        pos_mm = start_pos_mm + args.feed_rate_mm_s * (t - t_slew_start)
        wall_iso = datetime.datetime.utcfromtimestamp(t).isoformat() + "Z"
        csv_rows.append({
            "wall_time_unix": t,
            "wall_time_iso": wall_iso,
            "pos_mm": pos_mm,
            "distance_nm": rec["distance_nm"],
            "distance_mm": rec["distance_mm"],
            "q1": int(rec["q1"]),
            "q2": int(rec["q2"]),
            "in_ramp": int(in_ramp),
        })

    # ------------------------------------------------------------------ write
    fieldnames = ["wall_time_unix", "wall_time_iso", "pos_mm",
                  "distance_nm", "distance_mm", "q1", "q2", "in_ramp"]
    try:
        with open(args.output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(csv_rows)
    except Exception as e:
        _emit({"error": f"Failed to write CSV: {e}"})
        sys.exit(1)

    sidecar_path = args.output.replace(".csv", "_meta.json")
    duration_s = t_move_done - t_move_start
    achieved_rate = len(records) / duration_s if duration_s > 0 else 0.0
    metadata = {
        "axis": ax,
        "end_mm": args.end_mm,
        "feed_rate_mm_s": args.feed_rate_mm_s,
        "actual_start_mm": start_pos_mm,
        "actual_end_mm": actual_end_mm,
        "accel_mm_s2": accel_mm_s2,
        "decel_mm_s2": decel_mm_s2,
        "ramp_t_accel_s": ramp_t_accel,
        "ramp_t_decel_s": ramp_t_decel,
        "t_move_start": t_move_start,
        "t_move_done": t_move_done,
        "duration_s": duration_s,
        "samples": len(records),
        "achieved_rate_hz": achieved_rate,
        "od2000_topic": args.od2000_topic,
        "pdin_port": args.pdin_port,
    }
    try:
        with open(sidecar_path, "w") as f:
            json.dump(metadata, f, indent=2)
    except Exception as e:
        logger.warning("Failed to write metadata sidecar: %s", e)

    _emit({
        "done": True,
        "csv_path": args.output,
        "meta_path": sidecar_path,
        "actual_start_mm": start_pos_mm,
        "actual_end_mm": actual_end_mm,
        "samples": len(records),
        "achieved_rate_hz": achieved_rate,
    })


if __name__ == "__main__":
    main()
