# LMI Gocator 2690 / GoSDK Notes

> **Archived 2026-08-05.** Kept for historical context; no longer maintained.

Research notes from the vendor GO_SDK (`14400-6.5.2.5_SOFTWARE_GO_SDK`,
unzipped locally at `<local path, not checked into this repo>`,
**not** checked into this repo) for planning the Gocator 2690 laser
line-profile sensor integration (IP `192.168.1.10`, branch
`feat-gocator-integration`). No encoder — motion comes from an existing
gantry axis at constant velocity; we issue software triggers to start/stop
a scan and want a 3D point cloud/surface out the other end.

This is a **facts-gathering doc**, not a design doc — the integration
approach (ctypes wrapper vs. C helper subprocess vs. raw protocol) is not
yet decided. See "Open questions" at the end.

All function/type names below are cited with header paths relative to
`GO_SDK/Gocator/GoSdk/GoSdk/` unless noted otherwise.

---

## 1. Connection model

GoSDK talks to the sensor over plain TCP/IP sockets — there is no USB or
proprietary transport. Ports are enumerated in `GoSdkReservedPorts.h`:

| Port | Purpose |
|------|---------|
| 80 | Sensor HTTP server (web UI) |
| 3190 | **Sensor control channel** (`GO_SDK_RESERVED_PORT_SENSOR_CONTROL`) |
| 3192 | Sensor firmware upgrade |
| 3194 | **Sensor health channel** (`GO_SDK_RESERVED_PORT_SENSOR_HEALTH`) |
| 3195 | Sensor private data channel |
| 3196 | **Sensor public data channel** (`GO_SDK_RESERVED_PORT_SENSOR_PUBLIC_DATA`) — this is where GoSurfaceMsg/GoProfileMsg/etc. actually arrive |
| 3220 | Discovery protocol (UDP broadcast, used by `GoSystem_FindSensor*`) |
| 3400 | Remote procedure call |
| 502 | Modbus TCP server (optional, separate protocol/output path) |
| 8190 | Ethernet ASCII server (optional, separate protocol/output path) |
| 44818 | EtherNet/IP explicit server (optional) |

`GoSensor.h` exposes getters/setters for all of these
(`GoSensor_ControlPort`, `GoSensor_DataPort`, `GoSensor_HealthPort`,
`GoSensor_UpgradePort`, `GoSensor_SetDataPort`, etc.), confirming control
and data are two independent TCP connections, both opened by
`GoSensor_Connect()`.

**Lower-level protocol without linking the C library**: the control-channel
protocol (port 3190) and the data-channel wire format (port 3196) are
LMI's own binary framing — nothing in this SDK drop documents the byte
format directly (no protocol spec file was found; the `.c` files under
`Gocator/GoSdk/GoSdk/Messages/*.c`, e.g. `GoDataSet.c`, `GoDataTypes.c`,
implement `kSerializer`-based binary (de)serialization of the messages,
which would have to be reverse-engineered from those `.c` files to
reimplement in pure Python). By contrast, the **optional output protocols**
are standard/scriptable and don't need this SDK at all:
- **Modbus TCP** (port 502) — could be read with any Modbus client library
  (e.g. `pymodbus`) if the health/measurement values needed are exposed
  over Modbus registers. Unclear if full surface/point-cloud data is
  available this way (Modbus is register-based, better suited to scalar
  measurements than bulk profile data — needs verification against the
  Gocator user manual, which isn't in this SDK drop).
- **Ethernet ASCII** (port 8190) — a plaintext/scriptable protocol,
  again likely aimed at simple measurement values, not bulk surfaces.

Given the goal is bulk point-cloud/surface data at scan rate, the binary
data channel (via GoSDK, or a reverse-engineered client) is almost
certainly required — Modbus/ASCII are probably too low-bandwidth/scalar
for this. This needs to be confirmed against the Gocator 2690 user manual
(available from the sensor's own web UI under Manage → Support, per
`samples/README.md`; not present in the downloaded SDK package).

**Discovery**: `GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)`
(`GoSystem.h:530`) resolves a `GoSensor` handle from `192.168.1.10` without
needing broadcast discovery — this is the path used by every C sample.

---

## 2. Acquisition/trigger modes

### Profile/frame trigger source — `GoSetup.h`

`GoTrigger` (`GoSdkDef.h:297-303`):

| Value | Constant | Meaning |
|-------|----------|---------|
| 0 | `GO_TRIGGER_TIME` | Internal clock — sensor free-runs at a configured frame rate |
| 1 | `GO_TRIGGER_ENCODER` | Encoder-triggered (not applicable — no encoder here) |
| 2 | `GO_TRIGGER_INPUT` | Digital input triggered |
| 3 | `GO_TRIGGER_SOFTWARE` | Software-triggered — each frame is emitted only when the host calls `GoSensor_Trigger()` |

Set via `GoSetup_SetTriggerSource(setup, GoTrigger source)` /
read via `GoSetup_TriggerSource(setup)` (`GoSetup.h:665-692`).

`GoSensor_Trigger(GoSensor sensor)` (`GoSensor.h:477`) — doc comment:

> "This method is used in conjunction with sensors that are configured to
> accept software triggers. The sensor must be running (e.g. by calling
> GoSensor_Start) for triggers to be accepted. When the trigger mode is set
> to Software, this command will trigger individual frames in Profile or
> Surface mode. For G2 sensors with other trigger modes, this command can
> also be used to trigger Fixed Length surface generation when the Fixed
> Length Start Trigger option is set to 'Software'."

So `GoSensor_Trigger()` has **two distinct meanings** depending on config:
1. If `TriggerSource == SOFTWARE`: it fires one individual profile/surface
   frame per call (a per-sample trigger, not what we want for continuous
   constant-velocity scanning).
2. If surface generation is `FIXED_LENGTH` and its start-trigger is set to
   `SOFTWARE` (see below): it starts one fixed-length surface capture pass.

No C sample in this SDK actually calls `_Trigger()` (`grep` over
`samples/**/*.c` found zero hits) — the trigger start/stop pattern has to
be inferred from headers, not copied from an example.

### Surface generation type (encoderless vs. encoder) — `GoSurfaceGeneration.h`

`GoSurfaceGenerationType` (`GoSdkDef.h:2627-2633`):

| Value | Constant |
|---|---|
| 0 | `GO_SURFACE_GENERATION_TYPE_CONTINUOUS` |
| 1 | `GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH` |
| 2 | `GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH` |
| 3 | `GO_SURFACE_GENERATION_TYPE_ROTATIONAL` (encoder-based, turntable use case) |

Set via `GoSurfaceGeneration_SetGenerationType(surface, type)`.

`GoSurfaceGenerationStartTrigger` (`GoSdkDef.h:2644-2653`, used by
`FIXED_LENGTH` mode only):

| Value | Constant |
|---|---|
| 0 | `GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL` |
| 1 | `GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL` |
| 2 | `GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE` |

Fixed-length-specific setters: `GoSurfaceGenerationFixedLength_SetLength`,
`_SetStartTrigger`, `_SetTriggerExternalInputIndex` (`GoSurfaceGeneration.h`).

**RESOLVED (confirmed on real 2690 hardware 2026-07-30, and via
`GoTransform.h`)**: the "assumed velocity → Y spacing" accessor does exist —
it's just not on `GoSetup`/`GoSurfaceGeneration` where this doc originally
looked. It's `GoTransform_SetSpeed(transform, k64f value)` / `GoTransform_Speed(transform)`
(mm/sec) in `GoTransform.h`, retrieved via `GoSensor_Transform(sensor)`. This
is the SDK accessor behind the web UI's **Manage > Motion and Alignment >
Speed** field ("Travel Speed") — see `GOCATOR_CONCEPTS.md` §2c for the
corrected web-UI picture (it's a standalone Speed setting, not buried inside
the Bar/Disk Alignment panel, though a Moving-type alignment pass can also
populate it automatically). `GoTransform_SetEncoderResolution` sits right
next to it for the encoder case — same shape, two different knobs.

Confirmed-working recipe on hardware, driven from the sensor's own web UI
(not yet from the SDK, but the SDK calls are the obvious 1:1 mapping):
1. Set travel speed via Manage > Motion and Alignment > Speed
   (`GoTransform_SetSpeed`).
2. Trigger source = `GO_TRIGGER_TIME` (`GoSetup_SetTriggerSource`).
3. Surface Generation type = `GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH`
   (`GoSurfaceGeneration_SetGenerationType`), sized to the expected travel
   distance (`GoSurfaceGenerationFixedLength_SetLength`).
4. Surface Generation start trigger = `GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE`
   (`GoSurfaceGenerationFixedLength_SetStartTrigger`) — i.e. **not**
   `CONTINUOUS`, which this doc originally guessed. `CONTINUOUS` free-runs
   fixed-length surfaces back-to-back with no external event; `FIXED_LENGTH`
   + `SOFTWARE` start trigger is the one that waits for our
   `GoSensor_Trigger()`/software command to bracket each gantry pass.
5. Start gantry motion, then hit start-scan (software trigger) once the
   gantry is at constant velocity; the sensor emits one on-sensor-generated
   surface (`GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE` /
   `GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD`) per pass, already correctly
   scaled in Y using the configured travel speed.

This means **on-sensor surface generation is the way to go** — no need for
the PC-side profile-reconstruction fallback described earlier in this doc.
Still open: whether `GoSensor_Trigger()` is the right call to arm/fire the
`FIXED_LENGTH`+`SOFTWARE`-start surface from the SDK (vs. some other
API only discoverable by tracing what the web UI's "start scanning" button
actually sends), and the exact sequencing/timing tolerance between
"gantry has reached constant velocity" and "fire software trigger" for
clean start-of-scan data.

### Starting/stopping a scan under software control

- `GoSystem_Start(system)` / `GoSystem_Stop(system)` (`GoSystem.h:374,440`)
  — starts/stops the whole sensor acquisition stream. This is the natural
  "start scan" / "stop scan" pair for our use case (gantry-timed scan
  window): call `GoSystem_Start()` when the gantry begins its constant-
  velocity pass, stream/collect profile messages, call `GoSystem_Stop()`
  when the pass ends.
- `GoSystem_ScheduledStart(system, k64s value)` (`GoSystem.h:390`) exists
  for a delayed/scheduled start — not needed here (software-driven timing
  is under our control anyway).
- `GoSensor_Trigger(sensor)` is the finer-grained "trigger one frame" (or
  "start one fixed-length surface capture") call described above — only
  relevant if `TriggerSource == SOFTWARE` (per-frame softtrigger) or if
  using `FIXED_LENGTH` surface generation with a software start trigger.
  For a continuous constant-velocity gantry pass of unknown-in-advance
  duration, `GO_TRIGGER_TIME` + `GoSystem_Start()`/`GoSystem_Stop()` is
  simpler than juggling per-frame `GoSensor_Trigger()` calls.

---

## 3. Output data types

`GoDataMsgType` values (`GoSdkDef.h:1974-2024`, `GO_DATA_MESSAGE_TYPE_*`)
relevant to us:

| Value | Type | Shape |
|---|---|---|
| 0 | `GO_DATA_MESSAGE_TYPE_STAMP` (`GoStampMsg`) | per-frame metadata batch |
| 5 | `GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD` (`GoProfileMsg`) | raw, non-resampled profile: unordered/sparse X positions along the laser line |
| 7 | `GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE` (`GoResampledProfileMsg`) | resampled profile: dense 1-D array along X, indexed by column |
| 8 | `GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE` (`GoSurfaceMsg`) | **dense 2-D grid** (rows = Y/travel, cols = X/laser-line) of Z heights |
| 28 | `GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD` (`GoSurfacePointCloudMsg`) | dense 2-D grid of full `kPoint3d16s{x,y,z}` triples (not just Z) |
| 9 | `GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY` (`GoSurfaceIntensityMsg`) | dense 2-D grid, 8-bit grayscale, same width/height as height map |

Source: `samples/C/ReceiveSurface/src/ReceiveSurface.c` handles all of
these types explicitly in one big `switch(GoDataMsg_Type(dataObj))`.

**Units and scaling** (confirmed from `ReceiveSurface.c` and
`ReceiveProfile.c`):
- Resolutions (`GoSurfaceMsg_XResolution/YResolution/ZResolution`) are
  **nanometres** (`k32u`), converted to mm via `/1_000_000.0`.
- Offsets (`GoSurfaceMsg_XOffset/YOffset/ZOffset`) are **micrometres**
  (`k32s`), converted to mm via `/1_000.0`.
- Row/point data itself (`GoSurfaceMsg_RowAt` → `k16s*`, or
  `GoSurfacePointCloudMsg_RowAt` → `kPoint3d16s*`) is **raw 16-bit signed
  integer counts**; engineering value = `offset + resolution * raw_count`
  per axis. `0x8000` (`INVALID_RANGE_16BIT`) marks an invalid/missing
  point (occlusion, no return, etc.) — must be filtered before use.
- `GoProfileMsg_At(profileMsg, k)` for the *non*-resampled profile returns
  `kPoint16s* data`, i.e. `{x, y}` pairs per point (`data[i].x` = lateral
  position raw count, `data[i].y` = height raw count) — this is genuinely
  an unordered/sparse point set along the line, not a fixed-width array
  (`ReceiveProfile.c:179-212`).
- The *resampled* profile (`GoResampledProfileMsg_At`) returns a plain
  `short*` array of fixed width (`GoResampledProfileMsg_Width`), one Z
  value per evenly-spaced X column — this is the "uniform" 1-D analogue of
  `GoSurfaceMsg`'s 2-D grid.

**Is there a built-in point-cloud output mode, or do we reconstruct
ourselves?** Both exist on-sensor: `GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE`
(dense height-map grid, Z only + fixed X/Y resolution+offset) and
`GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD` (dense grid, full XYZ per
cell — still a **grid**, not an unordered cloud, despite the name — it is
literally called "Surface Point Cloud (Un-Resampled surface)" in the
enum comment (`GoSdkDef.h:1999`), meaning it's the *un-resampled* X/Y grid
with full per-cell XYZ, as opposed to `UNIFORM_SURFACE`'s implied-XY/
explicit-Z-only grid). Both are only meaningful if the sensor's own
surface generator produced them — which, per §2, is unclear for our
encoderless/time-triggered case. If on-sensor surface generation turns out
not to support useful encoderless output, we fall back to receiving
`GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD` (or `UNIFORM_PROFILE`) frames
and building the surface/point cloud in Python ourselves using assumed
velocity for Y, per §2.

`GoStamp` struct (`Messages/GoDataTypes.h:123-144`) fields available on
every frame regardless of mode: `frameIndex` (k64u, counts from 0),
`timestamp`, `encoder` (irrelevant, no encoder), `status` (bitmask incl.
digital input states/pulse counts), `id` (source device), `ptpTime`
(microseconds since PTP epoch) — `frameIndex`/`ptpTime` are the natural
time base for our own Y reconstruction.

---

## 4. Minimal control flow

From `samples/C/ReceiveSurface/src/ReceiveSurface.c` (surface mode) and
`samples/C/Configure/src/Configure.c` (parameter changes), the sequence is:

```c
GoSdk_Construct(&api);                                    // load kApi/GoSdk runtime
GoSystem_Construct(&system, kNULL);
kIpAddress_Parse(&ipAddress, "192.168.1.10");
GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor);
GoSensor_Connect(sensor);                                 // opens control channel

GoSetup setup = GoSensor_Setup(sensor);
GoSetup_SetExposure(setup, GO_ROLE_MAIN, exposure_us);    // Configure.c pattern
GoSetup_SetTriggerSource(setup, GO_TRIGGER_TIME);         // or GO_TRIGGER_SOFTWARE
GoSetup_SetFrameRate(setup, hz);                          // requires EnableMaxFrameRate(kFALSE) first
// surface-specific, if using on-sensor surface generation:
//   GoSurfaceGeneration surfGen = GoSetup_SurfaceGeneration(setup, ...);  // exact accessor not confirmed — see open items
//   GoSurfaceGeneration_SetGenerationType(surfGen, GO_SURFACE_GENERATION_TYPE_CONTINUOUS);
GoSensor_Flush(sensor);                                   // push config to sensor immediately

GoSystem_EnableData(system, kTRUE);                       // opens/enables data channel
GoSystem_Start(system);                                   // sensor starts acquiring (== "start scan")

// per gantry-scan-window loop:
GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT);  // blocking poll, or...
// ...GoSystem_SetDataHandler(system, onData, ctx) for an async callback model instead (see ReceiveAsync.c)
for (i = 0; i < GoDataSet_Count(dataset); ++i) {
    GoDataMsg msg = GoDataSet_At(dataset, i);
    switch (GoDataMsg_Type(msg)) { /* GO_DATA_MESSAGE_TYPE_* cases, see §3 */ }
}
GoDestroy(dataset);

GoSystem_Stop(system);                                    // "stop scan"
GoDestroy(system);
GoDestroy(api);
```

`GoSensor_Flush(sensor)` (used in `Configure.c`) is notable: per its doc
comment, setting functions do **not** automatically push to the sensor —
sync happens automatically on read calls and always before `Start()`, but
`Flush()` forces an immediate sync if the caller needs it sooner.

Two receive models exist side by side, both over the same data channel:
- **Polling**: `GoSystem_ReceiveData(system, &dataset, timeoutUsec)` —
  blocking call with an explicit timeout (used by `ReceiveSurface.c`,
  `ReceiveProfile.c`, all the simplest samples).
- **Callback/async**: `GoSystem_SetDataHandler(system, GoDataFx function,
  kPointer receiver)` (`GoSystem.h:242`) — `samples/C/ReceiveAsync/src/ReceiveAsync.c`
  demonstrates this; the callback runs on **"a separate thread spawned by
  the GoSDK library"** per that file's own comment, with an explicit
  warning to keep callback processing minimal. This matters for a ctypes
  wrapper: if wrapping the callback model, the Python callback would be
  invoked from a non-Python-owned thread and must acquire the GIL
  correctly (ctypes `CFUNCTYPE`/`WINFUNCTYPE` callback marshaling) —
  polling is much simpler to wrap correctly and is probably the better
  starting point for a first cut.

---

## 5. Gocator 2690 model specifics

**Nothing 2690-specific (measurement range, resolution, max scan rate)
was found anywhere in this SDK drop** — no header, sample, or doc/*.html
file mentions "2690" in a way that ties to sensor specs (the few
grep hits are unrelated substring matches in generated Doxygen
filenames/indices). The SDK is model-agnostic; all range/resolution/rate
limits are queried live from the connected sensor at runtime via
`GoSetup_FrameRateLimitMin/Max`, `GoSetup_SpacingIntervalLimitMin/Max`,
etc. (`GoSetup.h`) rather than being compile-time constants.

**Action needed**: pull the actual 2690 datasheet/specs (measurement
range, X resolution, Z resolution/repeatability, max profile/frame rate)
from LMI's public site or the sensor's own web UI (Manage → Support →
Development Kits → SDK, per `samples/README.md`) to size exposure time
and the maximum gantry velocity the chosen frame rate can support without
under-sampling the travel axis. This doc cannot supply those numbers.

**Update**: the companion doc `GOCATOR_CONCEPTS.md` (§6) found these via
the public 2600-series datasheet — scan rate 900–10000 Hz, 3700 points/
profile, X resolution 124–550 µm, FOV 385–2000 mm, Z range 1550 mm,
Z repeatability 12 µm. Use those as the starting point for exposure/
frame-rate/velocity sizing; still worth confirming against the specific
2690 unit's web UI once reachable.

---

## 6. GoAccelerator, licensing, health, and other notes

**GoAccelerator** (`GoAccelerator.h`, `samples/C/AcceleratorReceiveMeasurement/`):
**not** a hardware-free simulator/replay tool. Per the sample file's own
purpose comment: "Demonstrates the simple use of the Accelerator by
connecting to a sensor and receiving a measurement. This allows processing
to be performed on the PC rather than on the sensor." I.e. it's a PC-side
process that a live sensor's measurement/tool pipeline can be routed
through (`GoAccelerator_Attach(accelerator, sensor)`) so that CPU-heavy
tool computations run on a PC instead of the sensor's embedded CPU — it
still requires a live, reachable physical sensor. **Not useful for offline
development without hardware.**

**GoReplay / GoRecordingFilter** (`GoReplay.h`, `GoRecordingFilter.h`):
on-sensor recording/replay configuration — lets the sensor re-run
previously recorded raw frames through its measurement/tool pipeline. This
is also sensor-resident functionality (configures how the physical sensor
replays its own buffer), not a way to run a virtual/software-only Gocator
on a dev machine. No emulator or software-only sensor stand-in was found
anywhere in this SDK.

**Licensing**: every sample directory ships its own `license.txt`; per
`samples/C/ReceiveSurface/src/ReceiveSurface.c`'s header comment, samples
are "Licensed under The MIT License. Redistributions of files must retain
the above copyright notice." No top-level SDK license file was found
outside the samples — the core `GoSdk`/`kApi` library license was not
independently confirmed in this pass (check `GO_SDK_4`-referenced
documentation or LMI's site before redistributing/linking in a shipped
product).

**Health/status reporting**: separate channel and API from the data
channel — `GoSystem_SetHealthHandler`, `GoSystem_ReceiveHealth`,
`GoSystem_ClearHealth` (`GoSystem.h:337-363`), backed by `GoHealth.h`/`.c`
in `Messages/`, and `samples/C/ReceiveHealth/` demonstrates the pattern
(same poll-or-callback duality as the data channel, on port 3194).

**Threading/buffer gotchas found**:
- The async data-handler callback runs on an SDK-internal thread — see §4.
- `GoSystem_SetDataCapacity(system, kSize capacity)` (`GoSystem.h:257`)
  exists to size the internal receive buffer/queue — relevant if a scan
  produces a burst of frames faster than the Python side drains them
  (likely, if wrapping via ctypes with per-call marshaling overhead).
  Needs sizing once real frame rate / scan duration is known.
- `GoDestroy(dataset)` must be called on every `GoDataSet` received (both
  poll and callback paths) — this is a manual reference-counted/owned
  object; a ctypes wrapper must not leak these.
- `RECEIVE_TIMEOUT` in all polling samples is `20000000` (µs = 20 seconds)
  — generous, presumably to allow for slow/first-connection frames; a
  production wrapper would want a much shorter timeout tied to expected
  frame period.

**Python bindings**: no existing Python wrapper, `ctypes` reference, or
`.pyi`/binding hints of any kind were found anywhere in this SDK
(`samples/` only ships C, C#, and VB.NET). Any Python integration starts
from zero — either via `ctypes`/`cffi` against `GoSdk.so`/`kApi.so` (Linux
Arm64/X64/X86 makefiles exist under `Gocator/GoSdk/GoSdk-Linux_*.mk` and
`samples/C/*/*-Linux_*.mk`, confirming Linux shared-library builds are a
first-class target — not Windows-only), a small C helper subprocess
communicating over stdio/local-socket, or a from-scratch reimplementation
of the wire protocol (discouraged — see §1, undocumented binary framing).

---

## Open questions

1. ~~**Encoderless surface generation on-sensor**~~ — **RESOLVED**, see §2:
   confirmed working via `GoTransform_SetSpeed` + `GO_TRIGGER_TIME` +
   `FIXED_LENGTH`/`SOFTWARE`-start surface generation. Remaining sub-question:
   confirm `GoSensor_Trigger()` is the correct SDK call to fire the software
   start trigger (not yet tested from code, only from the web UI's
   start-scan button).
2. **`GoSetup_SurfaceGeneration()` accessor**: the sample control-flow in
   §4 assumes some `GoSetup_SurfaceGeneration(setup, ...)`-shaped call
   returns the `GoSurfaceGeneration` handle for use with
   `GoSurfaceGeneration_SetGenerationType()`, but the exact accessor name/
   signature was not directly confirmed by grep in the time available —
   verify against `GoSetup.h`'s full text or a surface-mode sample before
   writing code against it.
3. **Whether Modbus (port 502) or Ethernet ASCII (port 8190) output can
   carry full profile/surface data**, which would allow a scriptable
   integration without GoSDK/ctypes at all — plausible for scalar
   measurements, unconfirmed (likely no) for bulk point-cloud data. Check
   the user manual.
4. **2690 hardware specs** (range, resolution, max rate) — not in this SDK
   drop at all; pull from LMI datasheet or the sensor's own web UI before
   picking exposure/frame-rate/expected-velocity numbers.
5. **Core `GoSdk`/`kApi` library license** — only sample-code MIT licenses
   were confirmed; the core library's license terms for
   linking/redistribution weren't independently verified in this pass.
6. **Binary wire-protocol reverse-engineering feasibility** — if a
   non-C-linking approach is preferred, someone would need to read
   `Messages/GoDataSet.c` / `GoDataTypes.c`'s `kSerializer`-based
   read/write code to pin down the exact byte layout; not attempted here
   beyond confirming those files are where it lives.
