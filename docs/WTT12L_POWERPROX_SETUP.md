# WTT12L-A2523 PowerProx Bring-Up Guide

This is the bring-up doc for a second SICK IO-Link distance sensor — the
WTT12L-A2523 "PowerProx" — connected through the same ifm AL1342 IO-Link
master already in use for the OD2000 (see
[MQTT_AL1342_SETUP.md](MQTT_AL1342_SETUP.md), which this doc assumes you've
read — it covers the AL1342's HTTP control interface and `gettree`
discovery in detail; this doc doesn't repeat that material).

**Status: working, but via the analog output, not native IO-Link.** The
WTT12L's own IO-Link process data never validated on this AL1342 — every
attempt returned "invalid process data." The sensor's analog output (Qa),
digitized through an ifm DP4200 IO-Link analog-input bridge, does work and
is confirmed against two physical reference distances. See "What actually
happened on hardware" below before reading the rest of this doc as
prescriptive — it's a record of the investigation, not a clean recipe.

---

## What this device is

- Part: **WTT12L-A2523** (SICK part no. 1082477), "WTT12 PowerProx" —
  a photoelectric time-of-flight proximity sensor, not to be confused with
  the OD2000 (that's a dedicated laser rangefinder; this is a compact
  background-suppression sensor that also outputs distance).
- Sensing range: 100 mm ... 1,400 mm (max 50 mm ... 1,400 mm), 1 mm
  resolution, typ. accuracy ±15-20 mm.
- Has three output paths: IO-Link process data (pin 4), a switched analog
  output 4-20 mA / 0-10 V (pin 2, "Qa"), and a discrete switching output Q1
  (also pin 4, in SIO mode). **Only the analog path has been made to work
  on this hardware so far** — see below.
- Connector: M12, 5-pin, connection diagram CD-375:

  | Pin | Wire | Function |
  |---|---|---|
  | 1 | BN | +(L+) |
  | 2 | WH | Qa (analog out) |
  | 3 | BU | -(M) |
  | 4 | BK | Q1 / IO-Link C/Q |
  | 5 | GY | Sender off |

---

## What actually happened on hardware (2026-07-28)

### Physical connection took two tries

First attempt (wired to physical port **X01**) showed the AL1342 port
correctly configured for IO-Link (`mode = 3`) but `status = 0` ("State not
connected") — no device seen at all. Root cause was wiring/power, not
software: after rechecking connections and confirming the sensor's laser
was actually lit, it turned out to be plugged into physical port **X07**,
not X01. **Don't assume the physical port label matches where you think
you plugged in — verify by scanning all 8 ports' `productname`, same as
the OD2000 doc recommends, rather than trusting the physical label alone.**

### Native IO-Link process data never validated

Once found on port 7, identification worked cleanly:

```bash
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":-1,"adr":"/iolinkmaster/port[7]/iolinkdevice/productname/getdata"}'
# {"cid":-1,"data":{"value":"WTT12L-A2523"},"code":200}
```

`vendorid` = 26 (same as the OD2000 — both SICK), `deviceid` = 8388951,
`serial` = 25170048, `status` = 2 ("State operate" — link layer up, cyclic
comms established).

But `pdin/getdata` consistently returned:

```json
{"cid":-1,"error":"00","code":530}
```

**Code 530 = "The requested data is invalid" ("invalid process data")** —
confirmed from ifm's own AL13xx diagnostic-codes table (a sibling master
model's manual; the AL1342 doesn't publish its own copy of this table
anywhere we found, but the IoT Core response-code scheme is shared across
the AL1xxx family). This is distinct from `503` ("Service Unavailable" —
no device / wrong port mode, which is what showed up on the empty ports)
— the link was genuinely up, but the master was refusing to hand back the
process data as valid.

Every dynamic ISDU read also failed, with a different, IO-Link-level error:

```bash
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":-1,"adr":"/iolinkmaster/port[7]/iolinkdevice/iolreadacyclic","data":{"index":97,"subindex":0}}'
# {"cid":-1,"error":"8011","code":531}
```

`8011 = IDX_NOTAVAIL` (index not available), for indices 36 (Device
Status), 37 (Detailed Device Status), 83 (Detection/Operation mode), 97
(Sender configuration — the exact index that works fine on the OD2000), 120
(Process data select), and 229 (Distance to object). Meanwhile the *static*
identification indices — 16 (Vendor Name), 18 (Product Name), 21 (Serial
Number) — read back correctly. That split (static ID readable, everything
live/dynamic refused) survived a full power-cycle of the sensor and moving
a target through the sensing range (300-1115 mm tried), so it wasn't a
"target out of range" issue as first suspected. **Root cause not
isolated** — plausible candidates, untested: the device may need an
explicit configuration/teach step before it exposes the full photoelectric
ISDU set (the generic "Technical Information" doc this repo's decode was
built from explicitly warns not every documented index is implemented on
every device), or there's a firmware/profile mismatch between this unit and
the IODD the AL1342 expects. `decode_wtt12l_pdin()` in
[`src/laguna/rangefinder/decoders.py`](../src/laguna/rangefinder/decoders.py)
is kept in the code as the documented byte layout, but is unvalidated and
currently unreachable — don't trust it against real hardware yet.

### Working path: analog output through a DP4200 bridge

Rather than keep chasing the native IO-Link fault, an ifm DP4200 IO-Link
analog-input module was wired in series with the WTT12L's Qa (analog, pin
2) output and plugged into AL1342 port 7 in place of the WTT12L's own
IO-Link connection. **The AL1342's IO-Link ports have no analog-input
capability of their own** — port `mode` is one of `Disabled` / `DI` / `DO`
/ `IO-Link` only, nothing analog — so a bridge device that itself speaks
IO-Link (like the DP4200) is required to get an analog signal into this
system at all.

Confirmed on port 7 after the swap:

```bash
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":-1,"adr":"/iolinkmaster/port[7]/iolinkdevice/productname/getdata"}'
# {"cid":-1,"data":{"value":"DP4200"},"code":200}
```

`vendorid` = 310, `deviceid` = 610 (both ifm, as expected — different from
SICK's 26). `pdin/getdata` returns valid data immediately, no 530 errors.

**Process data: 4 bytes (8 hex chars), big-endian, two 16-bit channel
fields.**

| Bytes | Field |
|---|---|
| 0-1 | Channel 1 raw reading |
| 2-3 | Channel 2 raw reading |

Two physical readings anchor the decode:

| Distance | pdin | Ch1 raw | Ch2 raw |
|---|---|---|---|
| 600 mm | `2890FD01` | `0x2890` = 10384 | `0xFD01` = 64769 |
| 1115 mm | `3F02FD01` (jittered 3F01-3F06 across repeats) | `0x3F02` ≈ 16130 | `0xFD01` = 64769 |

**Channel 2 is unused** — identical `0xFD01` at both distances, doesn't
track the target at all. Consistent with an unconnected/open second input
on the DP4200 (only channel 1 is wired to the WTT12L). Not decoded.

**Channel 1 = current in µA**, i.e. divide by 1000 for mA. Both readings
fall inside the WTT12L's 4-20 mA analog span (10.384 mA and 16.130 mA), and
back-solving each independently for the sensor's un-taught default full-scale
distance (datasheet: 4 mA = 100 mm, 20 mA = max range) gives 1354 mm and
1439 mm respectively — both close to the WTT12L-A2523's actual rated max
range of 1,400 mm. Two independent readings agreeing with the datasheet
spec is a reasonable confirmation, not just a coincidence.

```
current_mA = channel1_raw / 1000
distance_mm = 100 + (current_mA - 4) / 16 * 1300
```

Forward-checking this formula against the two calibration points gives
~619 mm (vs. actual 600 mm) and ~1086 mm (vs. actual 1115 mm) — 19 and
29 mm off respectively, within the sensor's own ±15-20 mm accuracy spec
plus tape-measure/target-placement slop. **Expect more error here than the
OD2000 or a hypothetically-working native WTT12L reading** — this is going
through an extra analog conversion stage the OD2000 doesn't have.

`decode_dp4200_wtt12l_analog_pdin()` in
[`src/laguna/rangefinder/decoders.py`](../src/laguna/rangefinder/decoders.py)
implements this, with `near_mm`/`far_mm` as overridable parameters in case
the sensor ever gets taught a different span than the un-taught default.

### Consequence: no programmatic laser control on this path

The OD2000's laser on/off control (`_set_laser()` in `gantry_agent.py`)
works by writing ISDU index 97 ("Sender configuration") directly to the
sensor over IO-Link acyclic service data. **That's not available here.**
The DP4200 is a distinct IO-Link device sitting between the AL1342 and the
WTT12L — an `iolwriteacyclic` call against port 7 now addresses the
DP4200's own parameter space, not the WTT12L's, since the WTT12L isn't
the thing actually on the IO-Link bus anymore. There is no pass-through:
the DP4200 only forwards the analog signal value, not IO-Link service
requests to whatever's feeding its analog input.

Two ways to regain laser control, neither exercised yet:
1. **Wire it externally instead**: pin 5 (GY, "Sender off") is a
   high-active hardware input on the WTT12L itself — driving it directly
   (e.g. from a spare digital output elsewhere in the system, or an
   AL1342 port configured as `DO`) would toggle the laser without going
   through IO-Link at all. Needs an extra wire from pin 5 to whatever
   drives it.
2. **Resolve the native IO-Link fault** — if the 530/8011 errors get
   root-caused and fixed, `_set_laser()`'s exact call shape should work
   unmodified against port 7 (same ISDU 97 convention SICK uses across
   this product line), and you'd get index 97 write access back along with
   valid `pdin`.

If the application doesn't need to turn the laser off programmatically,
this is moot — the WTT12L's laser runs continuously by default like most
SICK photoelectric proximity sensors, same as the OD2000 before
`_set_laser()` was added.

---

## Reference: what was tried and abandoned

The rest of this section is kept for anyone revisiting the native IO-Link
path later.

- **Sender configuration ISDU** (index 97, subindex 0): `"00"` = sender
  active, `"01"` = sender not active — same convention as the OD2000,
  confirmed to exist in SICK's documentation for this product line, but
  returned `IDX_NOTAVAIL` on this specific unit as described above.
- **Process data select** (index 120): controls whether process data
  outputs distance vs. switching signals on WTT variants that support
  multiple structures. Also `IDX_NOTAVAIL` here.
- **MQTT push vs. HTTP polling**: not evaluated for this device at all,
  since native `pdin` access never worked. If the DP4200 analog path is
  the long-term answer, the same tradeoff from the OD2000 doc applies —
  `timer[n]/counter/datachanged/subscribe` for a ~2 Hz live readout,
  persistent `http.client.HTTPConnection` polling (not `urllib`) if a
  faster/scan-feeding rate is ever needed.

## Open questions

- Why did the WTT12L's native IO-Link process data validate the link layer
  (`status = operate`) but refuse `pdin` and every dynamic ISDU? Not
  root-caused.
- Is the DP4200's channel-1-as-µA interpretation exactly right, or is there
  a small offset/scale error hiding inside the ~20-30 mm decode residual?
  A third calibration point at a very different distance (e.g. near the
  100 mm or 1400 mm ends of the range) would tighten this up.
- Does the DP4200 have its own ISDU-configurable input range/scaling that
  might explain the residual error, or is it running on its own default?
