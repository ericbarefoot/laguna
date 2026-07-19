# MQTT + ifm AL1342 Bring-Up Guide

This is the one-time hardware setup guide for getting the SICK OD2000 laser
rangefinder data flowing from the AL1342 IO-Link master into laguna's MQTT
broker on the Pi. Run these steps once on the real hardware before using
`RangefinderSubsystem` or `TopographicProfiler`.

---

## Prerequisites

- The Pi (`red.lab`) is on the lab LAN and reachable via SSH
- The AL1342 is on the lab LAN with a DHCP reservation in dnsmasq → accessible
  at `al1342.lab`
- The OD2000 is physically connected to one IO-Link port on the AL1342
  (note the port number — you will need it throughout)
- Mosquitto is installed and running on the Pi (see Step 1)

---

## Step 1: Install Mosquitto on the Pi

```bash
ssh oak@red.lab
sudo apt update && sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

Create `/etc/mosquitto/conf.d/laguna.conf`:

```
listener 1883
allow_anonymous true
log_dest file /var/log/mosquitto/mosquitto.log
```

Then reload:

```bash
sudo systemctl restart mosquitto
```

> **Security note:** This is intentionally unauthenticated for the isolated lab
> LAN. If the network is ever reachable beyond the lab, add
> `password_file /etc/mosquitto/passwd` and create credentials with
> `mosquitto_passwd`.

Verify the broker is up:

```bash
mosquitto_pub -h localhost -t test -m hello
mosquitto_sub -h localhost -t test -C 1
```

---

## Step 2: Confirm OD2000 on AL1342

Open the **IoT-Core Visualizer** web UI in a browser:

```
http://al1342.lab/web/subscribe
```

Navigate to **Parameter → Iolinkmaster** and find the port the OD2000 is
connected to. Confirm it shows:

- `vendorid` = 85 (SICK's ifm vendor code)
- `productname` = something like `OD2000-xxxxxT15`

Note the port number — it appears in every MQTT topic path and every config
value as `pdin_port`.

---

## Step 3: Configure the AL1342 MQTT Command Channel

The AL1342 must be told about the MQTT broker before it can accept MQTT
commands (chicken-and-egg: MQTT config is delivered over MQTT, but MQTT isn't
set up yet). Use the IoT-Core Visualizer or direct HTTP POST for bootstrap.

### Via IoT-Core Visualizer (recommended for first-time setup)

In the Visualizer's **Notification** tab, use the wizard to:

1. Set the broker address to `red.lab` (or the Pi's IP if hostname resolution
   fails — see note below)
2. Set the port to `1883`
3. Set the command topic to `laguna/al1342/cmd`
4. Set the reply topic to `laguna/al1342/reply`
5. Click **Start** / **Apply**

### Via HTTP POST (scriptable alternative)

Send these IoT-Core JSON requests as HTTP POST bodies to
`http://al1342.lab/iolinkmaster`:

```bash
# Start the MQTT command channel
curl -X POST http://al1342.lab/iolinkmaster \
  -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":1,"adr":"/connections/mqttConnection/MQTTSetup/mqttCmdChannel/status/start"}'

# Set broker hostname
curl -X POST http://al1342.lab/iolinkmaster \
  -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":2,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/brokerIP/setdata","data":{"red.lab"}}'

# Set broker port
curl -X POST http://al1342.lab/iolinkmaster \
  -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":3,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/brokerPort/setdata","data":{"1883"}}'

# Set command topic
curl -X POST http://al1342.lab/iolinkmaster \
  -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":4,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/cmdTopic/setdata","data":{"laguna/al1342/cmd"}}'

# Set reply topic
curl -X POST http://al1342.lab/iolinkmaster \
  -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":5,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/defaultReplyTopic/setdata","data":{"laguna/al1342/reply"}}'
```

> **Hostname vs. IP in callback URLs:** The AL1342 manual examples only show
> IP addresses. If `red.lab` is rejected in the `brokerIP` field, substitute
> the Pi's actual IP address — nothing else needs to change.

Verify the command channel is live: subscribe to the reply topic on the Pi,
then publish a test request to the command topic:

```bash
# Terminal 1
mosquitto_sub -h localhost -t 'laguna/al1342/reply' -v

# Terminal 2
mosquitto_pub -h localhost -t 'laguna/al1342/cmd' \
  -m '{"code":"request","cid":99,"adr":"/processdatamaster/temperature/getdata"}'
```

You should see a JSON response in Terminal 1. If not, check that Mosquitto is
running and the AL1342 command channel was started successfully.

---

## Step 4: Subscribe AL1342 to OD2000 Cyclic Data

Replace `N` with the actual IO-Link port number the OD2000 is on:

```bash
# Subscribe to cyclic OD2000 pdin data via timer[1]
mosquitto_pub -h localhost -t 'laguna/al1342/cmd' -m '{
  "code": "request",
  "cid": 10,
  "adr": "/timer[1]/counter/datachanged/subscribe",
  "data": {
    "callback": "mqtt://red.lab:1883/laguna/od2000",
    "datatosend": ["/iolinkmaster/port[N]/iolinkdevice/pdin"]
  }
}'

# Set publish interval to 100 ms (10 Hz)
mosquitto_pub -h localhost -t 'laguna/al1342/cmd' \
  -m '{"code":"request","cid":11,"adr":"/timer[1]/interval/setdata","data":{"newvalue":100}}'
```

---

## Step 5: Verify the OD2000 Data Stream

```bash
mosquitto_sub -h localhost -t 'laguna/od2000' -v
```

Expected payload (one message per 100 ms):

```json
{
  "code": "event",
  "cid": 10,
  "adr": "",
  "data": {
    "eventno": "6317",
    "srcurl": "/timer[1]/counter/datachanged",
    "payload": {
      "/iolinkmaster/port[1]/iolinkdevice/pdin": {
        "code": 200,
        "data": "AABBCCDD0000"
      }
    }
  }
}
```

The `data` field is a hex string. For the OD2000 7002T15 (6 bytes = 12 hex chars):

| Bytes | Type | Meaning |
|-------|------|---------|
| 0–3   | big-endian int32 | Distance in **nm** |
| 4     | uint8 | Scale (normally 0) |
| 5     | uint8 | bit 0 = Q1, bit 1 = Q2 switching outputs |

Quick manual decode:

```python
hex_str = "0BEE0D000000"   # example
raw = bytes.fromhex(hex_str)
distance_nm = int.from_bytes(raw[0:4], "big", signed=True)
distance_mm = distance_nm / 1_000_000
print(f"{distance_mm:.3f} mm")
```

Plausible range for a lab flume setup: 200–1200 mm (200,000,000–1,200,000,000 nm).

---

## Step 6: Measure Achieved Publish Rate

The 100 ms interval is a starting point. Measure the actual delivered rate:

```bash
# Count messages over 10 seconds
mosquitto_sub -h localhost -t 'laguna/od2000' -C 100 | wc -l &
sleep 10; wait
```

Divide 100 by 10 = target rate in Hz. Lower the timer interval if needed:

```bash
# e.g., set to 50 ms (20 Hz)
mosquitto_pub -h localhost -t 'laguna/al1342/cmd' \
  -m '{"code":"request","cid":12,"adr":"/timer[1]/interval/setdata","data":{"newvalue":50}}'
```

The IO-Link COM3 cycle time for the OD2000 is ~0.7 ms, but the MQTT delivery
chain has overhead. Measure until the rate is stable before trusting it for
spatial resolution calculations.

---

## Step 7: serial_bridge.py Port Conflict

`serial_bridge.py` (port 9700 on the Pi) provides a raw TCP↔RS232 passthrough
to the BLC motion controller. `scan_runner.py` needs the same serial device
directly via pyserial — they cannot coexist.

Check whether `serial_bridge.py` holds the port permanently or opens it lazily:

```bash
ssh oak@red.lab
lsof /dev/serial/by-id/usb-FTDI_...      # check if serial_bridge.py has an fd open
```

**If it lazy-opens (only holds the port while a TCP client is connected):**
No extra steps needed — laguna disconnects `gantry_agent.py` before deploying
`scan_runner.py`, which releases the TCP side, and `serial_bridge.py` will
then release the serial fd.

**If it holds the port permanently:**
`scan_runner.py` will fail to open the serial device. Options:
- SIGSTOP `serial_bridge.py` during the scan and SIGCONT after:
  ```bash
  kill -STOP $(pgrep -f serial_bridge.py)
  # ... run scan ...
  kill -CONT $(pgrep -f serial_bridge.py)
  ```
- Or add a `--release-serial` mode to `serial_bridge.py` that closes the port
  on receiving a SIGUSR1.

Document which behavior is confirmed here once verified on real hardware.

---

## Topic Reference

| Topic | Direction | Publisher | Purpose |
|-------|-----------|-----------|---------|
| `laguna/al1342/cmd` | → AL1342 | laguna / mosquitto_pub | IoT-Core JSON command requests |
| `laguna/al1342/reply` | AL1342 → | AL1342 | IoT-Core JSON responses |
| `laguna/od2000` | AL1342 → | AL1342 | Cyclic OD2000 PDIN data |
| `laguna/gauge/water_level_mm` | Pi → | gauge_publisher.py | Massa gauge readings |
| `laguna/gauge/status` | Pi → | gauge_publisher.py | Online/offline status |
