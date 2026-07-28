# Rangefinder

OD2000 laser displacement sensor data via an ifm AL1342 IO-Link master and MQTT.
Lives in `src/laguna/rangefinder/` (subsystem class) and `src/laguna/mqtt/`
(transport). Pi-side scripts in `src/laguna/pi/` are deployed on-demand via SFTP.

---

## Hardware chain

```
SICK OD2000 → IO-Link COM3 → ifm AL1342 → Ethernet → Pi Mosquitto :1883
                                                              ↓
                                               MqttSubscriber (paho-mqtt)
                                                              ↓
                                               RangefinderSubsystem (laguna)
```

The AL1342 acts as an MQTT **client**: it connects to the Pi's Mosquitto broker
and publishes OD2000 process data cyclically on a timer. Laguna subscribes and
decodes the raw IO-Link PDIN hex payload.

Before using this subsystem for the first time, complete the one-time hardware
bring-up in `docs/MQTT_AL1342_SETUP.md`.

---

## Wiring

| OD2000 pin | Color | Signal | Connection |
|------------|-------|--------|------------|
| 1 | Brown | V+ (10–30 VDC) | AL1342 port V+ |
| 3 | Blue  | GND | AL1342 port GND |
| 4 | Black | Q1/C (IO-Link) | AL1342 port C/Q |
| 2 | White | Q2/Qa (independent) | spare — configurable as digital/analog |

Pin 4 carries IO-Link communication while in COM mode. Pin 2 (`Q2/Qa`) is
electrically independent and can run as a switching output simultaneously,
making it a candidate for hardware-triggered capture-latch (Approach B in
`docs/RANGEFINDER_PROFILING.md`).

---

## Sensor configuration

The OD2000 has two useful operating modes:

| Mode | Cycle time | Averaging | Best for |
|------|-----------|-----------|---------|
| Speed | 133 µs | off | Profiling scans (maximize rate) |
| Precision | 25.6 ms | avg=512, median=31 | Static water-level gauge use |

Set mode via the OD2000's IO-Link parameter interface (LR DEVICE or AL1342
web UI parameter write) before a scan session. The laguna code makes no
assumptions about the sensor's internal mode — the delivered MQTT rate is
what matters.

---

## Quick start (monitoring)

```python
from laguna.config import Config
from laguna.mqtt import MqttSubscriber
from laguna.rangefinder import RangefinderSubsystem

config = Config("config/example_config.yaml")

mqtt_sub = MqttSubscriber(config.get("mqtt"))
rangefinder = RangefinderSubsystem(config.get("rangefinder"), mqtt_sub)

rangefinder.connect()        # connects to MQTT broker, subscribes to od2000 topic

# Instantaneous read (drains buffer, returns latest)
dist_mm = rangefinder.get_distance_mm()
print(f"{dist_mm:.3f} mm")

# Burst collection
import time
time.sleep(1.0)
sample = rangefinder.get_latest_sample()   # (wall_time, distance_mm)
print(rangefinder.get_status())

rangefinder.disconnect()
```

---

## Topographic profiling

A `TopographicProfiler` scan runs entirely inside `gantry_agent.py` on the
Pi — the same persistent, SSH-connected process that already handles
interactive axis commands (`transport: pi_agent`, `PiGantryConnection`).
There is no separate deployed script: `gantry_agent.py` is the sole owner
of the BLC serial port for its whole session, and starting a scan
(`start_scan()`) just launches a background thread inside it that runs the
move, polls the OD2000 over HTTP, and writes the CSV — reusing the same
locked serial connection as everything else. Call `profiler.stop()` at any
time to cancel a scan in progress (sends `BST` on the scanning axis; the
scan still finishes normally through the same completion path, just with
fewer samples).

**Requires the `pi_agent` transport** (`transport: pi_agent` in the
`gantry:` config section) — the default `socket_bridge` transport talks to
`serial_bridge.py`, a separate process that holds the serial port
permanently and cannot participate in scanning. See `docs/MACRON_GANTRY.md`.

```python
from laguna import FlumeLab
from laguna.robot.macron import GantryController
from laguna.robot.macron.profiler import TopographicProfiler

lab = FlumeLab("config/example_config.yaml")
gantry = GantryController.from_config(lab.config.get("gantry"))  # transport: pi_agent
lab.add(gantry)
lab.connect_all()

profiler = TopographicProfiler(
    gantry=gantry,
    pi_host="red.lab",
    pi_user="oak",
    pi_key="~/.ssh/id_ed25519",
    al1342_host="192.168.1.251",  # raw IP — the AL1342 has no DNS of its own
    pdin_port=2,                   # IO-Link port OD2000 is on
    output_dir="./data/profiles",
)

result = profiler.scan(axis="A1", end_mm=500.0, feed_rate_mm_s=5.0)
# profiler.stop() from another thread cancels a scan in progress

print(result.metadata)
print(result.df.head())

# Open-loop validation: commanded distance vs. actual
expected_mm = abs(500.0 - result.metadata["actual_start_mm"])
actual_mm = result.metadata["actual_distance_mm"]
print(f"Expected: {expected_mm:.1f} mm, Actual: {actual_mm:.1f} mm, "
      f"Error: {abs(actual_mm - expected_mm):.2f} mm")
```

The result DataFrame has columns:
`wall_time_unix, wall_time_iso, pos_mm, distance_nm, distance_mm, q1, q2, in_ramp`

Rows with `in_ramp=1` were recorded during the acceleration or deceleration
phase at either end of the move. Filter them for the constant-velocity portion:

```python
slew_df = result.df[result.df["in_ramp"] == 0]
```

---

## Gauge MQTT publishing

`gauge_publisher.py` reads the Massa water-level gauge over serial and
publishes it to `laguna/gauge/water_level_mm`:

```python
import paramiko, json

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect("red.lab", username="oak", key_filename="~/.ssh/id_ed25519")

sftp = client.open_sftp()
sftp.put("src/laguna/pi/gauge_publisher.py", "/tmp/laguna_gauge_publisher.py")
sftp.close()

client.exec_command(
    "python3 /tmp/laguna_gauge_publisher.py"
    " --port /dev/ttyUSB2 --rate 1.0 --offset-mm 1500.0"
)
```

Or subscribe with `MqttSubscriber` to `laguna/gauge/water_level_mm` from the
laguna PC to receive live gauge readings alongside OD2000 data.

---

## PDIN payload decoding

The AL1342 publishes OD2000 process data as nested JSON. The PDIN hex string
lives at:

```python
msg["data"]["payload"]["/iolinkmaster/port[1]/iolinkdevice/pdin"]["data"]
```

`RangefinderSubsystem._decode()` calls `decode_od2000_pdin()` directly:

```python
from laguna.rangefinder import decode_od2000_pdin
result = decode_od2000_pdin("0BEE0D000000")
# {"distance_nm": 199987456, "distance_mm": 199.987, "scale": 0, "q1": False, "q2": False}
```

The decode logic is confirmed against the OD2000 7002T15 IODD. If the actual
payload layout differs from hardware testing, override `_decode()` in a
subclass without changing the public API.

---

## Open items

- Measure achieved AL1342 MQTT publish rate on real hardware (start at 10 Hz,
  lower the timer interval until the rate stabilizes)
- Confirm PDIN byte layout against actual output (decode and compare with LR
  DEVICE readout)
- Verify whether hostname `red.lab` resolves in AL1342 MQTT callback URLs
  (substitute Pi IP if not)
- Verify `serial_bridge.py` port-holding behavior — see `docs/MQTT_AL1342_SETUP.md`
- Approach B upgrade path: wire OD2000 Q2/Qa → INB 7 for hardware-triggered
  capture-latch position recording (see `docs/RANGEFINDER_PROFILING.md`)
