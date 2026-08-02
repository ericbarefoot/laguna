# MQTT + ifm AL1342 Bring-Up Guide

This is the one-time hardware setup guide for getting the SICK OD2000 laser
rangefinder data flowing from the AL1342 IO-Link master to laguna. All steps
below are confirmed working on real hardware (2026-07-27/28). Despite the
title, the OD2000's actual scan data path ended up being **plain HTTP
polling, not MQTT** — see "OD2000 Data Collection Strategy" below. MQTT
remains in use for the gauge publisher and optional live monitoring.

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

## Step 1: Install Mosquitto on the Pi — confirmed working

```bash
ssh oak@red.lab
sudo apt update && sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

**Pitfall hit on hardware:** the default `/etc/mosquitto/mosquitto.conf` on
this Pi already sets `log_dest file /var/log/mosquitto/mosquitto.log` at
line 11. Adding the same `log_dest` line again in a `conf.d/` override file
causes Mosquitto to refuse to start (`Duplicate "log_dest" value`, exit
status 3). Check the main config first:

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

## Step 2: Network addressing — confirmed working

The AL1342's IP is currently static (`192.168.1.251`), assigned directly on
the device rather than via dnsmasq DHCP. This is fine — no DHCP reservation
work is needed right now. Two things to keep straight:

1. **laguna PC → AL1342**: add the `/etc/hosts` entry above so `al1342.lab`
   resolves locally. Confirmed working: `http://al1342.lab/` returns HTTP 200.
2. **AL1342 → Pi (broker)**: the AL1342 has no DNS resolution of its own.
   Every broker address we configure on the AL1342 (see Step 3) must be the
   Pi's raw IP, `192.168.1.58` — **not** `red.lab`. This was tested and the
   hostname form was never attempted against the device; treat the IP as the
   required form until proven otherwise.

---

## Step 3: Confirm OD2000 on AL1342

Open the **IoT-Core Visualizer** web UI in a browser:

```
http://al1342.lab/web/subscribe
```

**Correction:** on this firmware/hardware, no web UI was reachable at
`/web/subscribe` — control is done entirely via raw HTTP POST to the device
root (see below). If a web UI does exist on your unit, the
**Parameter → Iolinkmaster** tab is where you'd find the OD2000's port;
otherwise use the `gettree`/`querytree` services below.

Navigate to **Parameter → Iolinkmaster** (or query via HTTP, see Step 4) and
find the port the OD2000 is connected to. Confirm it shows:

- `vendorid` = 85 (SICK's ifm vendor code)
- `productname` = something like `OD2000-xxxxxT15`

Note the port number — it appears in every MQTT topic path and every config
value as `pdin_port`.

---

## Step 4: The AL1342 HTTP control interface — confirmed working

**Key correction from earlier drafts of this doc:** there is no
MQTT-based "command channel" required to control the AL1342. All
configuration (`setdata`) and all subscription requests (`subscribe`) are
sent as plain HTTP POST requests to the device root:

```
POST http://192.168.1.251/
Content-Type: application/json
```

The AL1342 has an *additional*, optional feature called `mqttCmdChannel`
that lets it also *receive* the same IoT-Core commands over MQTT (publish to
a `cmdTopic`, AL1342 replies on a `defaultReplyTopic`). We enabled it below
because it seemed required at first, but **HTTP POST worked reliably for
every operation we needed and is the recommended default.** The MQTT command
channel is optional and its command/reply behavior over MQTT was not
actually exercised — treat it as unverified.

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

### Bootstrap the (optional) MQTT command channel — confirmed working

**Two corrections vs. the original plan:**

1. There is **no `MQTTSetup` path segment.** The real path is
   `/connections/mqttConnection/mqttCmdChannel/...`, not
   `/connections/mqttConnection/MQTTSetup/mqttCmdChannel/...`. The extra
   segment caused a 404.
2. `brokerPort`'s data type is `number`/`integer`, not `string`. Sending
   `{"newvalue":"1883"}` (a string) returns code 400; send
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

Confirmed on hardware: after this, the AL1342 (client ID
`00-02-01-AD-F2-AD`, its own MAC) shows up in the Pi's Mosquitto log:

```
New connection from 192.168.1.251:49153 on port 1883.
New client connected from 192.168.1.251:49153 as 00-02-01-AD-F2-AD (p2, c1, k10).
```

---

## Step 5: Which data points can actually be pushed via MQTT — confirmed working (with temperature)

This is the most important correction from the original plan.

**Not every data point supports `datachanged` subscription.** Walking the
full `gettree` output on this unit and searching for any node whose `subs`
include an entry named `datachanged` found exactly these categories:

| Path pattern | Type | Fires when |
|---|---|---|
| `/timer[1]/counter`, `/timer[2]/counter` | periodic | Every `interval` ms (interval has a **documented and measured 500 ms floor** — see below) |
| `/iolinkmaster/port[n]/portevent` | discrete event | IO-Link device connected/disconnected, or port operating mode changed |
| `/iolinkmaster/port[n]/iolinkdevice/iolinkevent` | discrete event | IO-Link diagnostic/fault events from the connected device |
| various `mqttConnection`/`mqttCmdChannel` config values | discrete event | The config value itself is edited via `setdata` |

**`processdatamaster/temperature` and (by the same pattern, almost
certainly) `iolinkmaster/port[n]/iolinkdevice/pdin` do NOT have their own
`datachanged` subelement.** There is no way to get a push notification
"whenever this continuous value changes" for process data. The only way to
get continuous values pushed is to attach them to a `timer[n]`'s periodic
tick via `datatosend`.

**Correction to an earlier draft of this doc:** `portevent` and
`iolinkevent` are true event-driven subscriptions with **no interval
floor** — they fire the instant the underlying condition occurs, since
they're not polled on a timer at all. But they don't help for streaming a
continuously changing quantity like distance: they only fire on discrete
state transitions (connect/disconnect/mode-change/fault), not on every
value update. Do not confuse "no rate floor" with "can substitute for a
continuous data stream" — it can't, for these path types.

**The 500 ms timer floor is real, not just documentation** — confirmed by
measurement: subscribing `timer[1]` to push `/processdatamaster/temperature`
at `interval=500` produced 24 messages in 12.0 s ≈ 2.00 Hz, matching the
floor almost exactly.

```bash
# Subscribe timer[1] to push temperature every tick (works for ANY value —
# used here as a wiring test since it doesn't require the OD2000)
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":40,"adr":"/timer[1]/counter/datachanged/subscribe","data":{"callback":"mqtt://192.168.1.58:1883/laguna/al1342/temp","datatosend":["/processdatamaster/temperature"]}}'

curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":41,"adr":"/timer[1]/interval/setdata","data":{"newvalue":500}}'

mosquitto_sub -h 192.168.1.58 -t 'laguna/al1342/temp' -v
```

Confirmed real payload shape (differs slightly from the original draft's
assumption — note `srcurl` reflects the *triggering* event, and `payload`
is a dict keyed by each requested data point path):

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

## Step 6: OD2000-specific verification — confirmed working ✓ (2026-07-28)

The OD2000 is installed and reading. Findings from live hardware testing:

**Port and identity confirmed.** Walking `iolinkmaster/port[n]/iolinkdevice`
for `n` in 1–8 and reading `vendorid`/`productname` found the OD2000 on
**port 2** (`productname=OD2000-7002T15`). Note: `vendorid=26`, not 85 as
originally guessed from the manual — don't rely on the vendorid value alone
to identify the sensor, match on `productname` instead.

**`pdin` has no direct `datachanged`, as predicted.** Searching the tree for
paths ending in `/pdin` with a `datachanged` subelement found none — same
pattern as `processdatamaster/temperature`. Only `port[2]/portevent` and
`port[2]/iolinkdevice/iolinkevent` support `datachanged` on this port, and
neither carries continuous distance data (see Step 5). So MQTT push for
`pdin` is capped at the `timer[1]` 500 ms / 2 Hz floor, confirmed by design,
not just by the temperature test.

**Decoder confirmed correct against a physical reference.** With the OD2000
reading a physically measured 808.4 mm ± 0.1 mm, a single `pdin/getdata`
read returned hex `302D56F7F700`. Decoding bytes 0–3 as big-endian signed
int32 nanometers gives **808.2778 mm** — matches within measurement noise.
Little-endian gives a negative, physically impossible value, ruling that out
unambiguously. The `nm`, big-endian assumption in `decode_od2000_pdin()` and
`scan_runner._decode_pdin()` is now hardware-confirmed, not just documented.

One anomaly: byte 4 ("scale") read as `247`, not `0` as the manual excerpt
suggested was "normal". Unexplained, and currently unused in the decode (no
scale multiplication is applied), so it doesn't affect correctness — but
worth further investigation if distance readings ever look systematically
off by a fixed factor.

**The 2 Hz MQTT ceiling was solved by switching to HTTP polling — see
"OD2000 Data Collection Strategy" below.** This is now the primary and only
mechanism `scan_runner.py` uses for OD2000 data; MQTT subscribe was
abandoned for this purpose (kept only for the gauge publisher's low-rate
1 Hz use case, and general non-scanning monitoring via `RangefinderSubsystem`
if ever needed).

---

## OD2000 Data Collection Strategy: HTTP Polling, not MQTT — confirmed working ✓ (2026-07-28)

Given the 2 Hz MQTT ceiling, three alternatives were considered (discussed
2026-07-27): direct HTTP polling of `pdin/getdata`, Modbus TCP (this AL1342
model has a fieldbus interface — see §9.2.5, p.45, ports X21/X22), and a
hardware capture-latch (OD2000 Q2/Qa → gantry INB 7). **HTTP polling was
tried first as the cheapest option and turned out to be more than
sufficient — Modbus TCP and the hardware latch were not needed.**

**First attempt failed with a connection timeout after a couple seconds:**

```python
import urllib.request
# a fresh request() call per poll — opens a new TCP connection every time
req = urllib.request.Request(AL1342_URL, data=..., headers=...)
urllib.request.urlopen(req, timeout=5)   # loop this tightly → times out after ~2s
```

Root cause: opening a new TCP connection (handshake + AL1342-side socket
setup) on every single poll overwhelmed the AL1342's embedded HTTP server
within a couple of seconds of tight-loop polling. A single request afterward
still worked fine — the device wasn't down, it just couldn't keep up with
that specific pattern.

**Fix: reuse one persistent connection.**

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

**Result: 1904 samples in 5.00 s = 380.7 Hz, zero errors.** At 5 mm/s scan
speed that's ~0.013 mm/sample spatial resolution — roughly 190x finer than
the 2 Hz MQTT ceiling would allow, and far beyond what's actually needed.

`scan_runner.py` now runs this polling loop on a background thread
(`_poll_pdin_loop`) for the duration of each scan, with reconnect-on-error
logic (close and reopen the connection on any exception, rather than
crashing). `TopographicProfiler` takes an `al1342_host` constructor arg
(a raw IP — the AL1342 has no DNS of its own) instead of the old
`od2000_topic` MQTT parameter.

**Not investigated further, since HTTP polling was sufficient:**
- **Modbus TCP** — this AL1342 model does support it (see manual §9.2.5).
  Would likely reach even higher rates (50–200+ Hz range is typical for
  Modbus TCP on a local LAN) and offload the polling pattern to a protocol
  built for it, but requires a new dependency (`pymodbus`) and locating the
  PDIN register-map addresses, which weren't in the sections of the manual
  reviewed so far. Revisit only if 380 Hz ever proves insufficient.
- **Hardware capture-latch (Approach B)** — OD2000 Q2/Qa → gantry INB 7.
  Still the right answer if sub-network-latency positional fidelity is ever
  needed, but the added wiring/BLC-side complexity isn't justified now that
  polling comfortably exceeds requirements.

---

## Step 7: serial_bridge.py Port Conflict

> **Obsolete as of 2026-08-02 — kept as a record of the investigation.**
> `serial_bridge.py` is retired (see `docs/MACRON_GANTRY.md`, "Retired:
> `serial_bridge.py`") and `scan_runner.py` no longer exists — scans run
> inside `gantry_agent.py`, which owns the serial port for its whole
> session. None of the SIGSTOP/SIGCONT workarounds below are needed or
> should be used. Note also that the premise "they cannot coexist" was
> **wrong**: nothing prevented both from holding the port at once, which
> is precisely why the bridge was retired rather than merely scheduled
> around.

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
| `laguna/al1342/cmd` | → AL1342 | laguna / mosquitto_pub | Optional MQTT-based command mirror of the HTTP interface (enabled, not exercised) |
| `laguna/al1342/reply` | AL1342 → | AL1342 | Replies to the above, if used |
| `laguna/al1342/temp` | AL1342 → | AL1342 | Wiring-test topic used to confirm the pipeline with `processdatamaster/temperature` (2 Hz, confirmed) |
| `laguna/od2000` | AL1342 → | AL1342 | **Not used by `scan_runner.py`** — scans poll `pdin/getdata` directly over HTTP instead (380 Hz vs. 2 Hz). This topic remains available for non-scanning live monitoring via `RangefinderSubsystem`/`MqttSubscriber` if ever needed, capped at 2 Hz. |
| `laguna/gauge/water_level_mm` | Pi → | gauge_publisher.py | Massa gauge readings |
| `laguna/gauge/status` | Pi → | gauge_publisher.py | Online/offline status |

---

## Confirmed hardware facts

*(2026-07-27 unless noted)*

- Pi (`red.lab`): `192.168.1.58`
- AL1342: static IP `192.168.1.251`, MQTT client ID = its own MAC
  `00-02-01-AD-F2-AD`
- AL1342 control is 100% HTTP POST to `http://192.168.1.251/`; the
  `gettree`/`querytree` services are the authoritative source of truth for
  available paths — don't trust path names from the printed manual without
  checking the tree first, since this firmware's tree differs from at least
  one documented example (`MQTTSetup` segment does not exist)
- `/etc/hosts` entry added on laguna PC: `192.168.1.251 al1342.lab al1342`
  (dnsmasq DHCP reservation not needed while the AL1342 keeps a static IP)
- Timer-based push (`timer[1]`, `timer[2]`) has a confirmed 500 ms / 2 Hz
  floor
- Event-driven push (`portevent`, `iolinkevent`) has no rate floor but only
  fires on discrete state transitions, not continuous value changes
- **(2026-07-28)** OD2000 confirmed on IO-Link **port 2**
  (`productname=OD2000-7002T15`; `vendorid=26`, not 85 as originally guessed)
- **(2026-07-28)** `pdin` confirmed to have no direct `datachanged` — same
  pattern as `processdatamaster/temperature`; only `portevent`/`iolinkevent`
  support it on that port, and neither carries continuous distance data
- **(2026-07-28)** PDIN decoder (big-endian nm) confirmed against a physical
  reference: 808.4 mm ± 0.1 mm measured, 808.2778 mm decoded from
  hex `302D56F7F700`. Byte 4 ("scale") read as 247, not 0 as assumed —
  unexplained, unused in the decode, doesn't affect correctness
- **(2026-07-28)** `pdin/getdata` HTTP polling with a persistent connection
  sustained **380.7 Hz, zero errors** over a 5 s test — a fresh TCP
  connection per request (e.g. `urllib`) instead chokes the AL1342's
  embedded HTTP server within ~2 s of tight-loop polling
- **(2026-07-28)** `scan_runner.py` and `TopographicProfiler` were updated
  to use HTTP polling (not MQTT subscribe) for all OD2000 scan data;
  `TopographicProfiler`'s `od2000_topic` constructor arg was replaced with
  `al1342_host` (a raw IP)
- **(2026-07-28)** OD2000 laser on/off is controlled via IODD parameter
  "Sender configuration", IO-Link index 97 (0x61), subindex 0: value `"00"`
  = laser ON (Sender active), `"01"` = laser OFF (Sender not active) —
  **note the values are inverted from what "0/1" might suggest**, easy to
  get backwards. Written via `iolwriteacyclic`:
  ```json
  {"code":"request","cid":-1,
   "adr":"/iolinkmaster/port[2]/iolinkdevice/iolwriteacyclic",
   "data":{"index":97,"subindex":0,"value":"01"}}
  ```
  Confirmed working on hardware. `scan_runner.py`'s `_set_laser()` wraps
  this — called with `on=True` right before each scan's move begins, and
  `on=False` in the `finally` block so the laser turns off even if the scan
  errors out.
