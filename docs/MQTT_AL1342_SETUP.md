# MQTT + ifm AL1342 Bring-Up Guide

This is the one-time hardware setup guide for getting the SICK OD2000 laser
rangefinder data flowing from the AL1342 IO-Link master to laguna. All steps
below have been verified on real hardware. The OD2000's scan data path uses
**plain HTTP polling, not MQTT** — see "OD2000 Data Collection Strategy" below.
MQTT remains in use for the gauge publisher and optional live monitoring.

---

## Prerequisites

- The Pi (`red.lab`, `192.168.1.58`) is on the lab LAN and reachable via SSH
- The AL1342 has a **static IP** (`192.168.1.251` in this lab) — it is not
  currently issued by dnsmasq DHCP. Add it to `/etc/hosts` on the laguna PC
  for hostname convenience:
  ```
  192.168.1.251    al1342.lab al1342
  ```
  This only resolves the name *on the laguna PC*. The AL1342 itself has no
  DNS of its own — every address we give *it* (broker IP, callback URLs)
  must be a raw IP, not a hostname.
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

**Note:** the default `/etc/mosquitto/mosquitto.conf` on the Pi already sets
`log_dest file /var/log/mosquitto/mosquitto.log` at line 11. Adding the same
`log_dest` line again in a `conf.d/` override file causes Mosquitto to refuse
to start (`Duplicate "log_dest" value`, exit status 3). Check the main config
first:

```bash
grep -n "log_dest" /etc/mosquitto/mosquitto.conf
```

If it's already set, **omit** `log_dest` from the override file. Create
`/etc/mosquitto/conf.d/laguna.conf`:

```
listener 1883
allow_anonymous true
```

Then reload:

```bash
sudo systemctl restart mosquitto
systemctl is-active mosquitto   # should print "active"
```

> **Security note:** This is intentionally unauthenticated for the isolated
> lab LAN. If the network is ever reachable beyond the lab, add
> `password_file /etc/mosquitto/passwd` and create credentials with
> `mosquitto_passwd`.

Verify the broker is up. **Order matters** — MQTT does not buffer messages,
so a publish before any subscriber is connected is simply lost. Start the
subscriber first:

```bash
# Terminal 1 — start first, blocks waiting for one message
mosquitto_sub -h localhost -t test -C 1

# Terminal 2 — run after Terminal 1 is waiting
mosquitto_pub -h localhost -t test -m hello
```

Or, single-terminal, use a retained message (broker holds it for late
subscribers):

```bash
mosquitto_pub -h localhost -t test -m hello -r
mosquitto_sub -h localhost -t test -C 1
```

---

## Step 2: Network addressing

The AL1342's IP is currently static (`192.168.1.251`), assigned directly on
the device rather than via dnsmasq DHCP. This is fine — no DHCP reservation
work is needed right now. Two things to keep straight:

1. **laguna PC → AL1342**: add the `/etc/hosts` entry above so `al1342.lab`
   resolves locally. Verify: `http://al1342.lab/` should return HTTP 200.
2. **AL1342 → Pi (broker)**: the AL1342 has no DNS resolution of its own.
   Every broker address we configure on the AL1342 (see Step 3) must be the
   Pi's raw IP, `192.168.1.58` — **not** `red.lab`. Use the raw IP for all
   addresses passed to the device.

---

## Step 3: Confirm OD2000 on AL1342

Locate the OD2000 on the AL1342. The device may have a web UI accessible at:

```
http://al1342.lab/web/subscribe
```

If the web UI is reachable, navigate to **Parameter → Iolinkmaster** to find
the port. Otherwise, or to verify programmatically, query via HTTP using the
`gettree`/`querytree` services (see Step 4). Either way, confirm the OD2000's
port shows:

- `vendorid` = 85 (SICK's ifm vendor code)
- `productname` = something like `OD2000-xxxxxT15`

Note the port number — it appears in every MQTT topic path and every config
value as `pdin_port`.

---

## Step 4: The AL1342 HTTP control interface

All configuration (`setdata`) and all subscription requests (`subscribe`) are
sent as plain HTTP POST requests to the device root:

```
POST http://192.168.1.251/
Content-Type: application/json
```

The AL1342 has an optional feature called `mqttCmdChannel` that enables it to
receive IoT-Core commands over MQTT (publish to a `cmdTopic`, AL1342 replies
on a `defaultReplyTopic`). **HTTP POST is the primary and recommended method**
for all operations. The optional MQTT command channel is configured below for
reference, but HTTP is more reliable and sufficient for all needs.

### Discover the device tree

Before guessing paths, always start here — the exact substructure names and
available services differ by firmware version:

```bash
curl -s -X POST http://192.168.1.251/ \
  -H "Content-Type: application/json" \
  -d '{"code":"request","cid":-1,"adr":"gettree"}' | python3 -m json.tool
```

This can be large (~77 KB on this unit). Save it to a file and grep/parse
rather than reading raw.

### Bootstrap the (optional) MQTT command channel

When configuring the MQTT command channel, note:

1. The path is `/connections/mqttConnection/mqttCmdChannel/...` (no
   `MQTTSetup` segment). Using `/connections/mqttConnection/MQTTSetup/mqttCmdChannel/...`
   will cause a 404.
2. `brokerPort` must be an integer, not a string. Sending
   `{"newvalue":"1883"}` (a string) returns code 400; use
   `{"newvalue":1883}` (an integer).

```bash
# Set broker IP (must be a raw IP, not a hostname)
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":2,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/brokerIP/setdata","data":{"newvalue":"192.168.1.58"}}'

# Set broker port — must be an integer
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":3,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/brokerPort/setdata","data":{"newvalue":1883}}'

# Set command topic
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":4,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/cmdTopic/setdata","data":{"newvalue":"laguna/al1342/cmd"}}'

# Set reply topic
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":5,"adr":"/connections/mqttConnection/mqttCmdChannel/mqttCmdChannelSetup/defaultReplyTopic/setdata","data":{"newvalue":"laguna/al1342/reply"}}'

# Start the command channel (corrected path, no data needed)
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":1,"adr":"/connections/mqttConnection/mqttCmdChannel/status/start","data":{}}'

# Confirm it's running
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":30,"adr":"/connections/mqttConnection/mqttCmdChannel/status/getdata"}'
# Expect: {"cid":30,"data":{"value":"running"},"code":200}
```

When running, the AL1342 (client ID `00-02-01-AD-F2-AD`, its own MAC) connects
to the Pi's Mosquitto broker and appears in the log:

```
New connection from 192.168.1.251:49153 on port 1883.
New client connected from 192.168.1.251:49153 as 00-02-01-AD-F2-AD (p2, c1, k10).
```

---

## Step 5: Which data points can actually be pushed via MQTT

**Not every data point supports `datachanged` subscription.** Walking the
full `gettree` output and searching for any node whose `subs` include an entry
named `datachanged` reveals exactly these categories:

| Path pattern | Type | Fires when |
|---|---|---|
| `/timer[1]/counter`, `/timer[2]/counter` | periodic | Every `interval` ms (interval has a **documented and measured 500 ms floor** — see below) |
| `/iolinkmaster/port[n]/portevent` | discrete event | IO-Link device connected/disconnected, or port operating mode changed |
| `/iolinkmaster/port[n]/iolinkdevice/iolinkevent` | discrete event | IO-Link diagnostic/fault events from the connected device |
| various `mqttConnection`/`mqttCmdChannel` config values | discrete event | The config value itself is edited via `setdata` |

**`processdatamaster/temperature` and `iolinkmaster/port[n]/iolinkdevice/pdin`
do NOT have their own `datachanged` subelement.** There is no way to get a push
notification "whenever this continuous value changes" for process data. The only
way to get continuous values pushed is to attach them to a `timer[n]`'s periodic
tick via `datatosend`.

Note: `portevent` and `iolinkevent` are true event-driven subscriptions with
**no interval floor** — they fire the instant the underlying condition occurs,
not on a timer. However, they only fire on discrete state transitions
(connect/disconnect/mode-change/fault), not on every value update. They cannot
substitute for a continuous data stream.

**Timer-based push has a 500 ms floor** — empirically confirmed: subscribing
`timer[1]` to push `/processdatamaster/temperature` at `interval=500` produces
approximately 2.00 Hz (roughly 24 messages in 12.0 s), matching the floor
almost exactly.

```bash
# Subscribe timer[1] to push temperature every tick (works for ANY value —
# used here as a wiring test since it doesn't require the OD2000)
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":40,"adr":"/timer[1]/counter/datachanged/subscribe","data":{"callback":"mqtt://192.168.1.58:1883/laguna/al1342/temp","datatosend":["/processdatamaster/temperature"]}}'

curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":41,"adr":"/timer[1]/interval/setdata","data":{"newvalue":500}}'

mosquitto_sub -h 192.168.1.58 -t 'laguna/al1342/temp' -v
```

The actual payload shape has the structure (note `srcurl` reflects the
*triggering* event, and `payload` is a dict keyed by each requested data
point path):

```json
{
  "code": "event",
  "cid": 40,
  "adr": "/laguna/al1342/temp",
  "data": {
    "eventno": "22",
    "srcurl": "00-02-01-AD-F2-AD/timer[1]/counter/datachanged",
    "payload": {
      "/timer[1]/counter": {"code": 200, "data": 22},
      "/processdatamaster/temperature": {"code": 200, "data": 36}
    }
  }
}
```

This matches what `RangefinderSubsystem._extract_pdin_hex()` and
`scan_runner.py`'s `_extract_pdin_hex()` already expect — the payload dict
is keyed by the literal data-point path string, with `{"code":200,"data":
...}` as the value. Good sign; no decoder rewrite needed for the envelope,
only (possibly) for the value inside.

---

## Step 6: OD2000-specific verification

**Port and identity.** The OD2000 is located on **port 2** (`productname=OD2000-7002T15`).
Note: `vendorid=26`, not 85 as documented in some manuals — do not rely on
vendorid alone to identify the sensor; match on `productname` instead.

**MQTT push limitations.** `pdin` has no direct `datachanged` subelement — same
pattern as `processdatamaster/temperature`. Only `port[2]/portevent` and
`port[2]/iolinkdevice/iolinkevent` support `datachanged` on this port, neither
of which carries continuous distance data (see Step 5). MQTT push for `pdin`
is therefore capped at the `timer[1]` 500 ms / 2 Hz floor by design.

**PDIN decoder verification.** The big-endian signed int32 nanometer decoding
is correct: With a physical reference of 808.4 mm ± 0.1 mm, a `pdin/getdata`
read of hex `302D56F7F700` decodes to **808.2778 mm**, matching within
measurement noise. Little-endian decoding produces a negative, physically
impossible value, confirming that bytes 0–3 as big-endian is correct. The
implementation in `decode_od2000_pdin()` and `scan_runner._decode_pdin()` is
hardware-verified.

Note: byte 4 ("scale") reads as `247`, not `0` as some manual excerpts suggest.
This is unexplained and currently unused in the decode (no scale multiplication
applied), so it does not affect correctness. Investigate further if distance
readings ever appear systematically offset by a fixed factor.

**HTTP polling solution.** Given the 2 Hz MQTT ceiling, the data collection
strategy shifted to HTTP polling — see "OD2000 Data Collection Strategy" below.
This is now the primary mechanism for OD2000 scan data; MQTT subscribe is
retained only for the gauge publisher's 1 Hz use case and general non-scanning
monitoring via `RangefinderSubsystem` if needed.

---

## OD2000 Data Collection Strategy: HTTP Polling, not MQTT

Given the 2 Hz MQTT ceiling, three alternatives were evaluated: direct HTTP
polling of `pdin/getdata`, Modbus TCP (this AL1342 model has a fieldbus
interface — see §9.2.5, p.45, ports X21/X22), and hardware capture-latch
(OD2000 Q2/Qa → gantry INB 7). **HTTP polling proved sufficient; Modbus TCP
and the hardware latch were not needed.**

**Initial naive approach: connection exhaustion.** Opening a fresh TCP
connection on every poll overwhelms the AL1342's embedded HTTP server:

```python
import urllib.request
# Opening a fresh TCP connection per poll exhausts resources
req = urllib.request.Request(AL1342_URL, data=..., headers=...)
urllib.request.urlopen(req, timeout=5)   # Overwhelms device in ~2 seconds
```

The AL1342's embedded HTTP server cannot sustain tight-loop polling with new
TCP connections. A single request still works fine — the device isn't down,
it simply cannot handle that connection churn rate.

**Solution: persistent HTTP connection.**

```python
import http.client, json, time

conn = http.client.HTTPConnection('192.168.1.251', 80, timeout=5)
payload = json.dumps({'code':'request','cid':-1,
                       'adr':'/iolinkmaster/port[2]/iolinkdevice/pdin/getdata'})
headers = {'Content-Type': 'application/json'}

samples = []
t_start = time.time()
while time.time() - t_start < 5.0:
    conn.request('POST', '/', body=payload, headers=headers)
    resp = conn.getresponse()
    data = json.loads(resp.read())
    hex_str = data['data']['value']
    samples.append((time.time(), hex_str))
conn.close()
```

This approach sustains **380.7 Hz (1904 samples in 5.00 s), zero errors.**
At 5 mm/s scan speed, that provides ~0.013 mm/sample spatial resolution —
roughly 190x finer than the 2 Hz MQTT ceiling, and far beyond what is needed.

`scan_runner.py` now runs this polling loop on a background thread
(`_poll_pdin_loop`) for the duration of each scan, with reconnect-on-error
logic (close and reopen the connection on any exception, rather than
crashing). `TopographicProfiler` takes an `al1342_host` constructor arg
(a raw IP — the AL1342 has no DNS of its own) instead of the old
`od2000_topic` MQTT parameter.

**Alternatives not pursued (380 Hz is sufficient):**
- **Modbus TCP** — this AL1342 model supports it (see manual §9.2.5). It
  would likely reach 50–200+ Hz and use a protocol designed for polling, but
  requires `pymodbus` dependency and PDIN register-map addresses not yet
  located in the manual. Revisit only if 380 Hz proves insufficient.
- **Hardware capture-latch** — OD2000 Q2/Qa → gantry INB 7. This would
  provide sub-network-latency fidelity, but added wiring and control logic
  complexity cannot be justified while polling already exceeds requirements.

---

## Topic Reference

| Topic | Direction | Publisher | Purpose |
|-------|-----------|-----------|---------|
| `laguna/al1342/cmd` | → AL1342 | laguna / mosquitto_pub | Optional MQTT-based command mirror of the HTTP interface (enabled, not exercised) |
| `laguna/al1342/reply` | AL1342 → | AL1342 | Replies to the above, if used |
| `laguna/al1342/temp` | AL1342 → | AL1342 | Wiring-test topic used to confirm the pipeline with `processdatamaster/temperature` (2 Hz, confirmed) |
| `laguna/od2000` | AL1342 → | AL1342 | **Not used by `scan_runner.py`** — scans poll `pdin/getdata` directly over HTTP instead (380 Hz vs. 2 Hz). This topic remains available for non-scanning live monitoring via `RangefinderSubsystem`/`MqttSubscriber` if ever needed, capped at 2 Hz. |
| `laguna/gauge/water_level_mm` | Pi → | gauge_publisher.py | Massa gauge readings |
| `laguna/gauge/status` | Pi → | gauge_publisher.py | Online/offline status |

---

## Confirmed hardware facts

- Pi (`red.lab`): `192.168.1.58`
- AL1342: static IP `192.168.1.251`, MQTT client ID = its own MAC
  `00-02-01-AD-F2-AD`
- AL1342 control is 100% HTTP POST to `http://192.168.1.251/`; the
  `gettree`/`querytree` services are the authoritative source of truth for
  available paths — don't trust path names from the printed manual without
  checking the tree first, since this firmware's tree differs from documented
  examples (`MQTTSetup` segment does not exist)
- `/etc/hosts` entry on laguna PC: `192.168.1.251 al1342.lab al1342`
  (dnsmasq DHCP reservation not needed while the AL1342 keeps a static IP)
- Timer-based push (`timer[1]`, `timer[2]`) has a 500 ms / 2 Hz floor
- Event-driven push (`portevent`, `iolinkevent`) has no rate floor but only
  fires on discrete state transitions, not continuous value changes
- OD2000 is on IO-Link **port 2** (`productname=OD2000-7002T15`; `vendorid=26`,
  not 85 as originally guessed)
- `pdin` has no direct `datachanged` subelement — same pattern as
  `processdatamaster/temperature`; only `portevent`/`iolinkevent` support it,
  and neither carries continuous distance data
- PDIN decoder (big-endian nanometers) is correct: 808.4 mm ± 0.1 mm physical
  reference decodes from hex `302D56F7F700` to 808.2778 mm. Byte 4 ("scale")
  reads as 247, not 0 as some documentation suggests — unexplained and unused
  in the current decode, does not affect correctness
- `pdin/getdata` HTTP polling with persistent connection sustains **380.7 Hz,
  zero errors** (1904 samples in 5.0 s); a fresh TCP connection per request
  exhausts the AL1342's embedded HTTP server within ~2 s
- `scan_runner.py` and `TopographicProfiler` use HTTP polling (not MQTT
  subscribe) for all OD2000 scan data; `TopographicProfiler`'s constructor
  takes `al1342_host` (a raw IP) instead of the old `od2000_topic` parameter
- OD2000 laser on/off is controlled via IODD parameter "Sender configuration",
  IO-Link index 97 (0x61), subindex 0: value `"00"` = laser ON (Sender active),
  `"01"` = laser OFF (Sender not active). **Note: these values are inverted from
  typical "0/1" convention.** Set via `iolwriteacyclic`:
  ```json
  {"code":"request","cid":-1,
   "adr":"/iolinkmaster/port[2]/iolinkdevice/iolwriteacyclic",
   "data":{"index":97,"subindex":0,"value":"01"}}
  ```
  `scan_runner.py`'s `_set_laser()` wraps this — called with `on=True` before
  each scan begins, and `on=False` in the `finally` block to turn the laser
  off even if the scan errors out.
