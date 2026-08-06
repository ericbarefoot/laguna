# WTT12L-A2523 PowerProx Bring-Up Guide

This is the bring-up doc for a second SICK IO-Link distance sensor — the
WTT12L-A2523 "PowerProx" — connected through the same ifm AL1342 IO-Link
master already in use for the OD2000 (see
[MQTT_AL1342_SETUP.md](MQTT_AL1342_SETUP.md), which this doc assumes you've
read — it covers the AL1342's HTTP control interface and `gettree`
discovery in detail; this doc doesn't repeat that material).

The WTT12L is operational via its analog output, not native IO-Link. Native
IO-Link process data has never validated on this AL1342 — all attempts
returned "invalid process data" (error code 530). The sensor's analog output
(Qa), digitized through an ifm DP4200 IO-Link analog-input bridge, works and
has been confirmed against two physical reference distances. The sections
below describe both the working analog approach and the investigation into
why native IO-Link failed, in case the native path is revisited in the future.

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

## Hardware integration and validation

### Physical connection

Initial troubleshooting revealed the AL1342 port was correctly configured for
IO-Link (`mode = 3`) but showed `status = 0` ("State not connected") — no
device detected. The root cause was physical: the sensor was actually
connected to physical port **X07**, not X01, despite labeling suggesting
otherwise. **Verify the actual connection by scanning all 8 ports'
`productname` (using the same discovery approach as documented for the OD2000),
rather than trusting physical port labels alone.** The AL1342 HTTP interface
makes this verification straightforward and avoids miswired connections.

### Native IO-Link process data — not functional

Device identification works on port 7:

```bash
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":-1,"adr":"/iolinkmaster/port[7]/iolinkdevice/productname/getdata"}'
# {"cid":-1,"data":{"value":"WTT12L-A2523"},"code":200}
```

`vendorid` = 26 (same as the OD2000 — both SICK), `deviceid` = 8388951,
`serial` = 25170048, `status` = 2 ("State operate" — link layer up, cyclic
comms established).

However, `pdin/getdata` consistently returns:

```json
{"cid":-1,"error":"00","code":530}
```

**Code 530 = "The requested data is invalid" ("invalid process data").** This
is confirmed from ifm's AL13xx diagnostic-codes table (published in sibling
master models' manuals; the AL1342 itself doesn't publish its own, but the
IoT Core response-code scheme is shared across the AL1xxx family). This
differs from code 503 ("Service Unavailable"), which appears on empty ports
— the link is up, but the master refuses to return process data as valid.

All dynamic ISDU reads also fail with a different error:

```bash
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":-1,"adr":"/iolinkmaster/port[7]/iolinkdevice/iolreadacyclic","data":{"index":97,"subindex":0}}'
# {"cid":-1,"error":"8011","code":531}
```

Error 8011 = `IDX_NOTAVAIL` (index not available). This affects indices 36
(Device Status), 37 (Detailed Device Status), 83 (Detection/Operation mode),
97 (Sender configuration — the same index that works on the OD2000), 120
(Process data select), and 229 (Distance to object). By contrast, static
identification indices — 16 (Vendor Name), 18 (Product Name), 21 (Serial
Number) — read back correctly. This split (static ID readable, dynamic
refused) persists across full power-cycles and targets moved through the
sensing range (300–1115 mm), ruling out out-of-range conditions.

**Root cause not identified.** Plausible explanations, untested:
- The device may require an explicit configuration or teach step before
  exposing the full photoelectric ISDU set (the generic SICK "Technical
  Information" documentation explicitly warns that not every documented
  index is implemented on every device).
- There may be a firmware or profile mismatch between this unit and the IODD
  the AL1342 expects.

`decode_wtt12l_pdin()` in
[`src/laguna/rangefinder/decoders.py`](https://github.com/ericbarefoot/laguna/blob/develop/src/laguna/rangefinder/decoders.py)
preserves the documented byte layout but is unvalidated and currently
unreachable — it should not be trusted against real hardware until the
native IO-Link fault is resolved.

### Working path: analog output through a DP4200 bridge

An ifm DP4200 IO-Link analog-input module bridges the WTT12L's analog
output (Qa, pin 2) to the AL1342. **The AL1342's IO-Link ports lack analog
input capability** — their modes are `Disabled`, `DI`, `DO`, or `IO-Link`
only. A bridge device that speaks IO-Link (like the DP4200) is necessary
to digitize an analog signal into the system.

On port 7 after wiring the DP4200:

```bash
curl -X POST http://192.168.1.251/ -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":-1,"adr":"/iolinkmaster/port[7]/iolinkdevice/productname/getdata"}'
# {"cid":-1,"data":{"value":"DP4200"},"code":200}
```

`vendorid` = 310, `deviceid` = 610 (ifm, as expected). `pdin/getdata`
returns valid data with no 530 errors.

**Process data: 4 bytes (8 hex chars), big-endian, two 16-bit channel
fields.**

| Bytes | Field |
|---|---|
| 0-1 | Channel 1 raw reading |
| 2-3 | Channel 2 raw reading |

Two physical readings provide calibration data:

| Distance | pdin | Ch1 raw | Ch2 raw |
|---|---|---|---|
| 600 mm | `2890FD01` | `0x2890` = 10384 | `0xFD01` = 64769 |
| 1115 mm | `3F02FD01` (jittered 3F01-3F06 across repeats) | `0x3F02` ≈ 16130 | `0xFD01` = 64769 |

**Channel 2 is unused** — the value `0xFD01` is identical at both distances
and does not track target motion, indicating an unconnected second input on
the DP4200. Only channel 1 is wired to the WTT12L.

**Channel 1 = current in µA** (divide by 1000 for mA). Both readings are
within the WTT12L's 4–20 mA analog range (10.384 mA and 16.130 mA). Using
the un-taught default full-scale mapping (datasheet: 4 mA = 100 mm, 20 mA =
max range), these readings reverse-solve to 1354 mm and 1439 mm respectively
— both consistent with the WTT12L-A2523's rated max range of 1,400 mm.

```
current_mA = channel1_raw / 1000
distance_mm = 100 + (current_mA - 4) / 16 * 1300
```

This formula yields ~619 mm and ~1086 mm for the two calibration points
(vs. actual 600 mm and 1115 mm), errors of 19 and 29 mm respectively.
These fall within the sensor's stated ±15–20 mm accuracy plus measurement
noise (tape-measure precision, target placement). **Expect higher error
than the OD2000 or a working native WTT12L path** — this adds an extra
analog conversion stage the OD2000 does not have.

`decode_dp4200_wtt12l_analog_pdin()` in
[`src/laguna/rangefinder/decoders.py`](https://github.com/ericbarefoot/laguna/blob/develop/src/laguna/rangefinder/decoders.py)
implements this formula. The `near_mm` and `far_mm` parameters are
overridable if the sensor is later taught to a different span than the
default un-taught range.

### Laser control limitation on this path

The OD2000's laser on/off control (`_set_laser()` in `gantry_agent.py`)
works by writing ISDU index 97 ("Sender configuration") directly to the
sensor over IO-Link. **This is not available with the DP4200 bridge.** The
DP4200 is a separate IO-Link device between the AL1342 and WTT12L.
Acyclic write calls to port 7 now address the DP4200's parameter space,
not the WTT12L's, since the WTT12L is no longer directly on the IO-Link
bus. The DP4200 forwards only the analog signal value, not IO-Link service
requests upstream to the WTT12L's firmware.

To recover laser control, either:
1. **Wire the laser control externally**: pin 5 (GY, "Sender off") is a
   high-active hardware input on the WTT12L. Driving it directly from a
   spare digital output (elsewhere in the system or an AL1342 port in `DO`
   mode) toggles the laser without IO-Link. Requires one additional wire.
2. **Resolve the native IO-Link fault**: if the 530/8011 errors are
   root-caused, `_set_laser()` should work unmodified against port 7
   (SICK uses ISDU 97 consistently across this product line), restoring
   both laser control and valid `pdin`.

If the application does not require programmatic laser control, this
limitation is irrelevant — the WTT12L's laser operates continuously by
default, like most SICK photoelectric sensors.

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

## Known limitations and open issues

- **Native IO-Link fault unresolved**: The WTT12L's IO-Link link layer
  establishes (`status = operate`), but `pdin` and all dynamic ISDU reads
  return errors. Root cause not identified; see "Native IO-Link process
  data — not functional" above for details and plausible explanations.
- **Decode accuracy**: The DP4200 analog path introduces ~20–30 mm decode
  residuals beyond the sensor's ±15–20 mm spec. The channel-1-as-µA
  interpretation is calibrated to two points; a third measurement at a
  very different distance (e.g. near 100 mm or 1,400 mm) would clarify
  whether a small offset or scale error is present.
- **DP4200 internal scaling**: Unknown whether the DP4200 has
  ISDU-configurable input range/scaling parameters that might improve
  accuracy, or if it operates on its own default.
