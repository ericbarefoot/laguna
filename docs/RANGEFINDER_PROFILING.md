# Topographic Profiling with the SICK OD2000 Rangefinder

Scanning is implemented and unified into the gantry agent. `gantry_agent.py`
(`src/laguna/robot/macron/gantry_agent.py`, the same persistent Pi-side
process that already handles interactive axis commands via
`PiGantryConnection`) is the sole owner of the BLC serial port and runs
scans on a background thread, with a live STOP path
(`TopographicProfiler.stop()` → agent-side `BST`). `TopographicProfiler`
(`src/laguna/robot/macron/profiler.py`) is a thin coordinator over
`gantry.connection.start_scan()`/`wait_for_scan_result()`. See
`docs/subsystems/rangefinder.md` for usage and `docs/MQTT_AL1342_SETUP.md`
for the one-time AL1342 hardware bring-up steps.

Serial safety is managed through a single `threading.Lock` inside `SerialBridge.send()`
that serializes interactive commands and the scan worker thread on one shared
connection — no disconnect/reconnect handoff between two processes is needed.
Clock correction is not required because BLC serial and OD2000 polling both run on
the Pi's single local clock.

The AL1342 MQTT push rate tops out at 2 Hz (`timer[n]` floor); scans instead poll
`pdin/getdata` directly over a persistent HTTP connection, achieving **380.7 Hz, zero errors**.
The PDIN byte layout is big-endian int32 nm, validated against a physical 808.4 mm ± 0.1 mm
reference (decoded 808.2778 mm). See `docs/MQTT_AL1342_SETUP.md` for details.

Note that `serial_bridge.py` holds the serial port permanently for its process lifetime — it and
`gantry_agent.py` must never run concurrently. `TopographicProfiler` no longer needs to work
around this constraint since it does not touch the serial port itself.

**Not yet implemented:** commanded-vs-actual validation on real hardware (comparing predicted end
position to ACP after a real move); STOP latency measurement (expected ~one MIF poll tick, ~0.1s).

---

## Design constraints and rationale

The controller is capped at approximately 3.48 Hz for safe position polling, and
pipelining reads causes a hang requiring physical power-cycle. This design avoids
depending on high-rate position feedback from the controller at all.

The scan operates on a single axis — **X or Y** (a separate tool handles 2D scans,
out of scope here). The profiling and fusion logic runs **on the Pi** (`red.dyn.ucr.edu`),
not centrally from the laguna PC — see "Infrastructure" below for why that matters.

## The sensor and its data path

A SICK OD2000 laser displacement sensor, IO-Link enabled. Per SICK's public
datasheet, the OD2000 measures internally at up to **7.5 kHz**.

**Connector pinout resolves the "switching output + IO-Link at once"
question.** The OD2000 uses a 5-pin M12 connector: pin 4 (`Q1/C`) carries
IO-Link communication (and can alternatively act as a plain switching
output when *not* in IO-Link COM mode — but not both at the same time, same
as any IO-Link device's C/Q line). Pin 2 (`Q2/Qa`) is a **separate,
independent output**, configurable as either analog or digital switching,
per SICK's operating instructions. Because it's a physically distinct pin
from the IO-Link line, **it can run simultaneously while pin 4 stays in
normal IO-Link communication with the master** — so a per-sample switching
pulse on `Q2/Qa` and the ordinary IO-Link/MQTT data path are not mutually
exclusive. This is what makes Approach B's wiring concrete: `Q2/Qa` → the
gantry controller's spare digital input, IO-Link left untouched.

## Infrastructure

- **IO-Link master:** an ifm **AL1342** — "IO-Link master with Modbus TCP
  interface," 8 IO-Link ports (class A, IO-Link rev. 1.1, COM1/2/3 up to
  230.4 kBaud), but per ifm's own product listing it also has a **separate
  IoT connection** with native MQTT-JSON support alongside its primary
  Modbus TCP interface — so the existing MQTT stream is very likely coming
  directly from this master's built-in IoT feature, not a hand-rolled
  Modbus→MQTT bridge script. **Could not confirm the exact MQTT publish
  interval/cycle-time from public docs** (the ifm operating-instructions
  PDFs weren't fetchable) — treat any assumed rate as unverified until
  measured directly against the live stream; don't design spatial
  resolution around a guessed number.
- **Both the MQTT broker and the BLC's `serial_bridge.py` will run on the
  same Pi** (`red.dyn.ucr.edu`). This is architecturally significant: if the
  profiling logic (subscribing to the rangefinder, issuing the move,
  correlating timestamps into positions) also runs **on that Pi**, both
  data streams share one local clock — no PC↔Pi clock-offset correction is
  needed at all, unlike the multi-camera code's `capture_time_mid` →
  `capture_time_mid_pc` correction (`src/laguna/camera/network.py`), which
  exists specifically to solve that cross-clock problem for a case where it
  couldn't be avoided. Here it can be: the laguna PC's role shrinks to
  "trigger a scan run over SSH, retrieve the finished profile file
  afterward" — the fusion itself never has to leave the Pi.
- The useful overall takeaway on rate: **the sensor's real bottleneck is
  almost certainly the IO-Link/MQTT delivery path, not the OEM-2T** — 7.5kHz
  internal sensor sampling is far beyond what any practical IO-Link/MQTT
  chain delivers, but that chain's actual rate should still be measured, not
  assumed, once it's observable live.

## What laguna already has that's relevant

- **Nothing SICK/IO-Link/MQTT-specific exists yet.** No client dependency
  (`pyproject.toml` currently has `pyserial`, `pymodbus`, `numpy`,
  `opencv-python`, `pyyaml`, `scipy`, `pandas`, `openpyxl`, `paramiko` — no
  MQTT library), no integration code, no docs. Any of the approaches below
  needs a new sensor subsystem written from scratch, following the
  documented extension point in `ARCHITECTURE.md` ("Adding New Subsystems"):
  a class with `connect()`/`disconnect()`/`get_status()`, registered via
  `lab.add(subsystem)`.

- **Time-based sync, not position-based sync.** Every existing subsystem
  (gauge, weir, cameras) is synchronized to the experiment's shared
  `ExperimentClock` (`src/laguna/timing/clock.py`) via a poll-based
  `Scheduler` (`src/laguna/timing/scheduler.py`) and correlated after the
  fact through the single append-only `EventLog`
  (`src/laguna/timing/event_log.py`, columns `wall_time_iso,
  wall_time_unix, runtime_s, subsystem, event_type, result, notes`). The
  multi-camera code (`src/laguna/camera/network.py`) even has a real
  clock-offset-correction mechanism between two independent clocks
  (`capture_time_mid` → `capture_time_mid_pc`). None of this ever
  correlates a reading to gantry *position* — only to time. A rangefinder
  integration would either reuse this time-only pattern (see Approach D
  below) or would need to add a genuinely new position-correlation
  mechanism (Approaches A-C).

- **Constant-velocity moves are already supported.** The G-code executor
  (`src/laguna/robot/macron/gcode.py`) parses the standard `F` feed-rate
  word and, for coordinated moves, calls `group_set_speed()` (`C<n> SPD`)
  before `group_begin_move_to()` (`C<n> BMT`) — this is the ordinary,
  already-used, safe way to use the `C<n>` group prefix **for motion**.
  This is worth being explicit about, since it's easy to conflate with the
  earlier finding that `C<n> ACP` (a group *read*) is unsafe/nonfunctional
  — that finding was specific to reads, not moves; coordinated group moves
  are normal, tested, everyday usage in this codebase. Nothing currently
  computes expected move duration from feed rate and distance, but both are
  known before a move starts, so nothing prevents adding that arithmetic.

- **No commanded-vs-actual calibration exists yet.** `ARCHITECTURE.md`
  lists a calibration module as an unimplemented "Next Steps" item. Any
  approach that trusts commanded motion (rather than continuously reading
  back actual position) needs its own validation that the assumption holds
  on this hardware — steppers can lose steps, belts can have backlash.

- **The hardware capture-latch mechanism is more general than its one
  current use.** `homing.py` uses `SCS`/`SCT`/`AIC`/`CAP`/`CAT` to latch an
  axis's position at the interrupt level the instant a configured digital
  input changes state (today, always a home/limit switch). The API itself
  (`set_capture_source(axis, source_index)`) takes an arbitrary input
  index — nothing in the firmware or driver restricts the source to a
  limit switch. But: there is exactly **one spare digital input** on the
  whole controller (`INB 7`, documented as unused in `MACRON_GANTRY.md`),
  and it's only reachable on the **commander node** — the responder node
  (which drives Z and Theta) has no ASCII-reachable input bus at all. So
  this mechanism, if repurposed for an external trigger, can only ever
  serve X or Y.

## Four candidate approaches

### A. Time-synchronized dead-reckoning (recommended starting point)

Command a constant-velocity move on the scan axis (X or Y) with a known
feed rate. Take one `ACP` read before the move (cheap, safe, well within the
proven ~3.48 Hz ceiling) to get the true starting position, and one after,
to sanity-check total distance traveled against what was commanded.
Meanwhile — running **on the Pi**, in the same process or a sibling one to
`serial_bridge.py` — subscribe to the SICK's MQTT stream and timestamp every
reading against one local clock. Reconstruct each reading's position as:

```
position(t) = start_position + feed_rate × (t_reading − t_move_start)
```

using only the constant-velocity middle portion of the move (discard or
separately model the accel/decel ramps at each end). Because both the move
timing reference and the MQTT timestamps are recorded on the same Pi, there
is no cross-machine clock correction to build at all — the laguna PC's role
is just to kick off the run (over SSH) and retrieve the finished profile
(timestamps + reconstructed positions + distances) afterward.

**Pros:** no new hardware or wiring at all; reuses laguna's existing G-code
feed-rate handling and the `ExperimentClock`/`EventLog`-style
timestamp-everything pattern (just single-clock instead of needing the
camera code's cross-clock correction); the gantry's contribution to
telemetry drops to one or two slow reads per pass, so the OEM-2T's polling
ceiling stops mattering; spatial resolution ends up governed by feed rate ×
the sensor's actual MQTT delivery interval (to be measured, see
"Infrastructure" above), not by anything the BLC does.

**Cons:** open-loop — this trusts that the axis actually moved at the
commanded constant velocity with no missed steps or backlash for the whole
pass. That assumption is untested on this hardware and needs its own
validation pass (e.g., comparing the predicted end position against the
actual `ACP` read, and possibly `ENP` — the encoder position, whose own
docstring says it's there "to detect lost steps" — before trusting this for
real survey data).

### B. Hardware-triggered capture-latch

Wire the OD2000's `Q2/Qa` output (pin 2 — independent of the IO-Link line,
see "The sensor and its data path" above) configured as a per-sample
switching pulse into the controller's one spare input (`INB 7`), and reuse
the capture-latch mechanism — currently only ever used for homing — to get
a true, interrupt-level "axis position at the exact instant this sample was
taken" reading, with no constant-velocity assumption needed. Since the
confirmed scan axis is X or Y (commander node), `INB 7` is reachable and
this approach stays on the table.

**Pros:** substantially higher positional fidelity than dead-reckoning,
since it doesn't assume anything about velocity constancy — it captures
where the axis actually was, precisely, at each sample event. Wiring is now
concrete (`Q2/Qa` → `INB 7`) rather than hypothetical.

**Cons:** requires new physical wiring into the controller; each capture
still costs a software re-arm-and-read round trip (roughly 70-80ms for a
single-axis read based on the earlier benchmark, so realistically something
like a 12-14 Hz ceiling — much better than 3.48 Hz, but not free, and
ultimately paced by however fast the sensor pulses and how fast the software
can re-arm); and — importantly — using this mechanism to respond to an
*external* device's trigger, rather than the axis's own limit switch, has
never been tried on this hardware. It would need careful, deliberate bench
validation, ideally once the controller's current state is confirmed fully
healthy again, not folded into a first attempt at anything else.

### C. Independent encoder tap

Physically tap the gantry's own encoder signal lines (`Encoder_1/2`,
`Remote_Encoder_1/2`, referenced in the axis declarations inside the
`.dsm` project files) with separate counting hardware — a microcontroller,
a small DAQ, or a USB quadrature-encoder reader — that shares a clock with
whatever timestamps the MQTT stream. This bypasses the OEM-2T's serial link
(and its now-demonstrated fragility under pipelined load) for position
sensing entirely.

**Pros:** highest achievable fidelity, continuous position at whatever rate
the tapping hardware supports (often kHz-class), zero dependency on the
BLC's serial link for position data, and zero risk of hanging the shared
production controller again.

**Cons:** the most hardware-development-heavy option here — new wiring, new
counting hardware to select/build, and the encoder-tap itself needs
verifying against the specific encoder's output stage before physically
wiring anything (parallel-tapping an output is usually safe, but "usually"
isn't good enough on shared lab equipment without checking first).

### D. Slow down and use what's already proven safe

Don't build anything new for the gantry side at all. Just constrain the
physical scan speed so that the existing, already-safe ~3.48 Hz sequential
`ACP` polling gives adequate spatial resolution for the profile, and
correlate the rangefinder stream to position purely by `EventLog`
timestamp — exactly the pattern the camera/gauge/weir subsystems already
use today.

**Pros:** uses 100% already-proven-safe primitives, literally zero new
mechanisms, no new risk of repeating the pipelining hang.

**Cons:** caps scan speed at whatever the target spatial resolution
requires (e.g., 1mm resolution implies ≤3.48mm/s scan speed) — this may or
may not be acceptable depending on the survey area and available time.

## Recommendation

Start with **Approach A**, implemented as a script running on the Pi
(`red.dyn.ucr.edu`) alongside `serial_bridge.py`, orchestrated remotely from
the laguna PC. It requires no new hardware, most directly answers "can we
avoid needing feedback from the BLC," and — since both the move-timing
reference and the MQTT subscription live in the same clock domain — is
simpler to get right than laguna's existing cross-clock camera-correction
pattern. Run **Approach D** alongside it as a cheap cross-check on the same
physical pass — since D needs nothing new either, and the two approaches
should agree if A's open-loop assumption is holding up.

**Approach B is a credible next upgrade, not just a hypothetical.** The
wiring question that originally made it uncertain is resolved (`Q2/Qa` →
`INB 7`, independent of the IO-Link line), and the confirmed X/Y scan axis
means it's actually reachable. It's reasonable to treat B as the
follow-up path once A's open-loop accuracy is measured and, if found
wanting, worth the wiring effort and the (separate, careful) bench
validation of using the capture-latch for an external trigger rather than a
limit switch. **C** remains the highest-fidelity, highest-effort fallback
if neither A nor B proves accurate enough.

## Outstanding technical work

- **Measure the actual MQTT publish rate empirically** — the AL1342's exact IoT/MQTT
  cycle-time spec is not confirmed from public documentation, and this rate
  (not the sensor's internal 7.5kHz, and not the OEM-2T) is the real ceiling on
  spatial resolution for Approach A.
- Confirm the AL1342's IoT/MQTT feature is enabled and configured before
  depending on it.
- Conduct a commanded-vs-actual validation pass (comparing predicted end
  position to `ACP`/`ENP` after a real move once Approach A is prototyped) —
  this has not been done for this hardware yet, per `ARCHITECTURE.md`'s
  own "Next Steps" list.
