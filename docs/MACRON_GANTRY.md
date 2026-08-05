# Macron Gantry (Snap2Motion / OEM-2T) Integration

Driver for the lab's 3-axis-plus-rotary (plus 4 more on a networked expansion
node — see "Axis model" below) robotic gantry, controlled through a
Modusystems OEM-2T rev D PLC over an ASCII text protocol. Lives at
`src/laguna/robot/macron/`.

**Status as of 2026-07-17: M0-M3 complete on `feat-macron-api` (rebased on
`develop`), all hardware-free work tested. M4 (read-only hardware
verification) is UNBLOCKED and passed a first read-only pass against real
hardware — see "Current status" below. Digital IO channel assignments were
substantially corrected on 2026-07-17 (see "Digital IO" below) — the axis
mapping (X=1/Y=2/Z=5/Theta=6) and the commander-native `INB`/`SOB` table
were both confirmed via a combination of vendor developer conversation, live
hardware queries, and decoding the `.dsm` project files' Named-IO
declarations.**

## Topology

```
laguna (this PC) --SSH--> red.dyn.ucr.edu (Pi, "oak" account)
                               |
                          gantry_agent.py (sole owner of the serial port)
                               |
                          RS232 --> OEM-2T rev D controller --> gantry motors
```

The controller is physically too far from this PC for direct serial, so a
Raspberry Pi sits next to it. The transports below all implement the same
`SnapConnection` interface, so the rest of the driver doesn't care which is
active:

1. **`PiGantryConnection`** (`pi_bridge.py`) — **the default.** Persistent
   SSH+JSON-line transport. Deploys and drives `gantry_agent.py` (standalone,
   pyserial-only, no `laguna` install needed on the Pi) as a long-running
   process over one SSH channel, for lower per-command latency than a fresh
   SSH connect+exec each time. **Required for `TopographicProfiler`**
   (`profiler.py`) — `gantry_agent.py` is the sole owner of the BLC serial
   port for its whole session and, since 2026-07-28, also runs full
   topographic scans on a background thread with a live STOP path
   (`start_scan()`/`stop_scan()`/`wait_for_scan_result()`), sharing the same
   serial connection as interactive commands via a lock rather than taking
   turns with a separate process. See `docs/subsystems/rangefinder.md`.
2. **`EthernetConnection` / `RS232Connection`** — direct transports for a
   controller reachable over the network or a local serial port.

All transports can be wrapped in **`SafeModeConnection`** to add the same
query-only allowlist gate `PiGantryConnection` has natively — see "Safety
model" below.

### Retired: `serial_bridge.py` (the `socket_bridge` transport)

`socket_bridge` — `RS232Connection(port="socket://red.dyn.ucr.edu:9700")`
pointed at `serial_bridge.py`, a raw protocol-unaware TCP↔serial passthrough
on the Pi — **was** the default. It is retired as of 2026-08-02. The
transport code still exists and works if you configure it explicitly, but
nothing should start `serial_bridge.py` again. Why:

- It bound `0.0.0.0:9700` with **no authentication and no protocol
  validation** — arbitrary bytes straight into the controller's ASCII
  interpreter. Its `bridge.log` records 92 connections from ~32 unique IPs,
  nearly all datacenter/scanner ranges rather than lab machines.
- It never lived in this repo — only on the Pi, unversioned, hand-started,
  and it did not survive a reboot. The default transport depended on a file
  tracked nowhere.
- It cannot do topographic scanning; only `pi_agent` can.
- `tio` on the Pi covers the direct-serial debugging use case, and covers it
  better — a human at a terminal is inherently a single writer.

**Correction to an earlier claim in this document:** it previously said
`gantry_agent.py` "would fail to open the port if `serial_bridge.py` is
already running." **That was wrong**, both before and after
`gantry_agent.py` gained `exclusive=True`. Linux does not lock tty devices
by default, and pyserial's `exclusive` flag uses `fcntl.flock(LOCK_EX |
LOCK_NB)`, which is *advisory* — it only conflicts with other flock holders.
`serial_bridge.py` took no lock, so the two could open the same port
simultaneously and interleave bytes mid-command, leaving the controller's
ASCII interpreter parsing spliced garbage. `exclusive=True` protects against
a second **agent**, not against `serial_bridge.py`. Retiring the bridge is
what actually closes that hole.

## Protocol

Verified two independent ways: (1) the vendor's shipped ASCII-interpreter
Pascal source, extracted from `modusystem/Snap2Motion/For_OEM-2T/TextHelp.chm`
→ `Resources/Methods Of Use/Ascii Commands/Ascii Command Interpreter OEM2T
RS232.DSM`; (2) hardware-tested code already on the Pi at
`~/modusystems_dev/oem2t.py` (written by the user directly against the real
controller). Both agree exactly.

- **Syntax**: `A<n><CMD> [params]` (single axis, n=1-16), `C<n><CMD>
  [params]` (coordinated group, n=1-10, max 6 axes/group, must `INI` first),
  or `<CMD> [params]` (global, no prefix).
- **Framing**: commands terminate on CR; responses terminate at a literal
  `>` prompt character, not CRLF.
- **Response envelope**: success = `"0 <value> >"`; error = `"<code> >"` (no
  leading `"0 "`). Parsed by checking the first token equals `"0"` — never by
  numeric magnitude (the previous branch's `>=600` heuristic was wrong: a
  legitimate position of `700.0` would have been misidentified as an error).
- **Get/set duality**: most 3-letter mnemonics read with no argument,
  write+echo with one.
- **No firmware homing** (`HMX` is a stub) — built in the driver via the
  hardware capture-latch mechanism (`SCS`/`SCT`/`AIC`/`CAB`/`CAP`/`CAT`).

Full command table, error codes, and grammar details are in the git history
of this integration (see the M1 commit message on `feat-macron-api`) and in
`src/laguna/robot/macron/commands.py`'s docstrings — not duplicated here to
avoid drift.

## Axis model

**8 axis slots are physically real**, not 4, split across two PLC nodes —
confirmed via direct conversation with the Snap2Motion vendor's developers
and via live hardware queries on 2026-07-17 (`A5`/`A6 ACP` both returned
real position values; `A1`/`A2`/`A5 PLT` all returned sane real soft-limit
numbers rather than the old ±8.2e8 "uninitialized" garbage seen previously).
`A6 PLT`/`A6 NLT` timing out (comm error 600) is expected — Theta is a
rotary axis with no position limits, not a bug.

| Axis | Index | Notes |
|---|---|---|
| X | 1 | commander (local controller) |
| Y | 2 | commander, electromagnetic brake |
| — (encoder) | 3 | commander, internal encoder slot — not exposed/commandable |
| — (encoder) | 4 | commander, internal encoder slot — not exposed/commandable |
| Z | 5 | responder (second networked PLC node), electromagnetic brake |
| Theta | 6 | responder, rotary |
| — (encoder) | 7 | responder, internal encoder slot — not exposed/commandable |
| — (encoder) | 8 | responder, internal encoder slot — not exposed/commandable |

The commander (local controller) drives slots 1-4; the responder (second
networked PLC node) drives slots 5-8, addressed transparently through the
same ASCII grammar. Slots 3/4/7/8 are internal encoder-only slots on their
respective nodes, not exposed/commandable motion axes, so the driver does
not model them as `Axis` objects or send `A3`/`A4`/`A7`/`A8` commands.

Naming: plain `X`/`Y`/`Z`/`Theta` (matches the user's own current project
files `eab-2026-07-16.dsm` / `600011-00-eab4.dsm`), not `XXPrime` (only in
the older vendor demo file and not-yet-updated `oem2t.py` on the Pi).

## Digital IO — read this before touching brakes or limit switches

**Correction (2026-07-17): these are NOT IsoIO channels.** Earlier notes
here called `XXHome`/`XXLim`/`YHome`/`YLim`/`Zhome`/`ZLim`/`Y_Brake`/
`Z_Brake`/`Y_Brake_Status` "IsoIO expansion-board channels declared stale in
the `.dsm` files" — that was a misreading. The IsoIO board genuinely isn't
installed (`ISI`/`ISO` commands still fail with error 263), but the `.dsm`
files' own `TNamedIO` records for these signals use `Type=1` (plain digital
input) with `ModuleNumber=16` ($10) — and the vendor's own runtime
(`standard.inc`) treats `ModuleNumber=16` as the **local/commander native**
input bus, not an IsoIO designation. So these were commander-native
`INB`/`SOB` channels all along; decoded directly from the `.dsm`'s Named-IO
block declarations (`eab-2026-07-16.dsm` and `eab-2026-07-17.dsm` agree
exactly) and cross-checked against a live `INB 1-8` read on 2026-07-17
(values `1,1,1,0,1,1,0,0` — consistent with the table below). Not yet
physically toggle-tested switch-by-switch.

**Commander native IO** (all `ModuleNumber=16` in the `.dsm`):

| Channel | Signal |
|---|---|
| `INB 1` | `XXHome` (X home switch) |
| `INB 2` | `XXLim` (X limit switch) |
| `INB 3` | `YHome` |
| `INB 4` | `YLim` |
| `INB 5` | `Zhome` |
| `INB 6` | `ZLim` |
| `INB 7` | *(unused/spare in the `.dsm`)* |
| `INB 8` | `Y_Brake_Status` |
| `SOB 4` | `Y_Brake` (output) |
| `SOB 5` | `Z_Brake` (output) |

**Two signals are the exception — and this matters.** `Z_Brake_Status` and
`TLim` (Theta's limit switch) both have `ModuleNumber=1` in the `.dsm`, not
`16` — per the vendor's own local/remote rule, they live on the
**responder's own input bank** (index 1 and 2 there), not the commander's.
The Z brake's *output* (`SOB 5`) is still on the commander — only the brake
*status input* and the Theta limit switch are responder-side.

Worse: **there is no ASCII text command that reaches the responder's own
inputs at all.** Traced directly in the interpreter's dispatch code: `INB`
is a flat, non-scoped call straight into the local `InputBit()` function,
with no axis/node prefix anywhere in the grammar. The only path to a remote
node's IO in this firmware family is the GUI-configured Named IO block
feature (which resolves `ModuleNumber` internally on the controller itself)
or the separate, vendor-encrypted Binary Commands node protocol used for
responder axis motion — neither is reachable from the ASCII RS232 interpreter
this driver talks to. `IOMap.z_brake_status_input` / `theta_limit_input`
therefore default to `None` and are treated as **architecturally
unimplemented, not just unprobed**: `brake_is_disengaged()` raises
`NotImplementedError` (not the usual `ValueError`) if asked to use them.
Getting a real reading on either would need a different mechanism — Named
IO config via the Snap2Motion IDE, or a from-scratch Binary Commands client
— not something to build without deciding it's worth the added complexity.

Everything else in the table above still needs physical toggle-testing
(diff `INB`/`SOB` snapshots — the pattern in the user's own
`~/modusystems_dev/status_snapshot.py` on the Pi is the validated tool for
this) to fully confirm, though the live `INB 1-8` read today is a strong
cross-check. Brake-control methods (`disengage_brake`/`engage_brake`) still
raise `ValueError` (not `NotImplementedError`) if a channel is explicitly
unset — that's the "not yet probed/configured" case, distinct from the
responder's structural unreachability.

**Homing is explicitly deprioritized** — the user does not plan to run it
soon ("we will not run homing anyway"). `HomingProcedure`'s architecture is
sound and fully unit-tested, just not a near-term verification priority.

**Soft limits, updated 2026-07-17**: earlier notes here said X/Y had
uninitialized software position limits (`PLT`/`NLT` ≈ ±822,536,056). A live
read today showed real, sane values instead (`A1 PLT=122`, `A2 PLT=80`,
`A5 PLT=24`) — so soft-limit protection does appear to be configured on
X/Y/Z now. These values are from an unhomed position, though, and haven't
been validated as correct for the actual travel envelope — treat them as
"present" not "verified correct." `MMCCommands.validate_soft_limits()` still
exists to catch the old garbage-value failure mode if it recurs.

## Safety model (defense in depth, multiple independent layers)

1. **`SAFE_COMMANDS` allowlist** (`pi_bridge.py`) — mnemonic → max arg count
   (0 = bare read only). Checked *before any byte reaches the wire*, on
   both the PC side (`PiGantryConnection.send()` / `SafeModeConnection`) and
   independently on the Pi side (`gantry_agent.py` keeps its own copy —
   intentionally duplicated, not imported, since the agent must run
   standalone without the `laguna` package). While `safe_mode=True` (the
   default everywhere), only read-only queries can reach the hardware,
   structurally — this cannot be bypassed by a bug in a higher layer.
2. **Fences** (`fences.py`) — `GCodeExecutor.execute()` only accepts a
   `CheckedTrajectory` produced by its own `plan()` call (checked by
   identity), so a fence check can never be skipped for G-code-driven
   motion.
3. **`dry_run`** on `GCodeExecutor` — logs intended ASCII commands without
   sending anything.
4. **`confirm_cb`** on `GCodeExecutor` — per-motion-segment human
   confirmation hook, for eventual Stage 3 (real motion) testing.

**No motion has ever been sent to the hardware.** Every verification so far
has been read-only queries or, where even those failed, connectivity checks.

## Files

| File | Purpose |
|---|---|
| `connection.py` | `SnapConnection` ABC, `EthernetConnection`, `RS232Connection`, envelope parsing, port discovery |
| `commands.py` | `MMCCommands` (typed ASCII command wrapper), `Axis`/`AxisState`/`IOMap`, `validate_soft_limits()` |
| `homing.py` | `HomingProcedure` — capture-latch homing (deprioritized for now, architecture complete) |
| `fences.py` | `BoxFence`/`CylinderFence`/`TrajectoryChecker`/`CheckedTrajectory` — exclusion-zone safety |
| `gcode.py` | `GCodeParser`/`GCodeExecutor` — G0/G1/G2/G3/G28/G90/G91/G21/G4/M0/M1/M114 |
| `pi_bridge.py` | `PiGantryConnection`, `SafeModeConnection`, `SAFE_COMMANDS`, `check_safe_mode` |
| `gantry_agent.py` | Standalone Pi-side agent (deployed via SFTP, not part of the `laguna` install) |
| `controller.py` | `GantryController` — `FlumeLab` subsystem facade, `from_config()` |

Tests: `tests/test_macron_{connection,commands,fences,homing,gcode,pi_bridge,controller}.py`
+ shared `tests/macron_fixtures.py` (`FakeSnapConnection` test double).

## Config

New `gantry:` section in `config/example_config.yaml` and
`Config._get_defaults()` — see either for the full schema. Build a live
controller with:

```python
from laguna.config import Config
from laguna.robot.macron import GantryController

config = Config(config_file="config/example_config.yaml")
gantry = GantryController.from_config(config)
lab.add(gantry)  # subsystem_name = "gantry" -> lab.gantry
# or, if lab already owns this Config: lab.add("gantry")
```

## Current status / resuming work

**M0-M3 done** (branch/protocol fixes, G-code, Pi bridge, config — all
hardware-free, all tested). See `feat-macron-api` branch commit history for
the milestone-by-milestone breakdown.

**M4 (read-only hardware verification) is UNBLOCKED as of 2026-07-17.** The
prior day's timeout (controller likely not running a program with the ASCII
interpreter active) resolved itself — a raw connectivity test (`1234\r` →
`0 1234.000 >`) succeeded, and the full read-only pass ran clean:
- `WHT` → `0.000`
- `INB 1-8` → real values (`1,1,1,0,1,1,0,0`, now decoded — see "Digital IO"
  above); `INB 9-16` → error 31 (`ParameterOutOfRangeEscapeCode`, confirmed
  in the vendor's `standard.inc`) — a real hardware limit (each board has
  only 8 native inputs), not a bug.
- `A1`/`A2`/`A5 ACP`/`PLT`/`NLT` all returned sane values; `A6` (Theta)
  `PLT`/`NLT` timed out, expected since it's a rotary axis with no position
  limits.

Not yet done: physically toggling switches/brakes to confirm the decoded
`INB`/`SOB` table above, and Stage 3 (actual motion) — both still gated on
explicit authorization in a future conversation.

### Resuming after a Pi reboot or session gap

Nothing needs starting by hand any more. `PiGantryConnection.connect()` SFTPs
`gantry_agent.py` to the Pi and launches it itself, so the default
`pi_agent` transport recovers from a reboot with no Pi-side step:

```python
from laguna.robot.macron import GantryController
gantry = GantryController.from_config(config)  # transport: pi_agent
gantry.connect()
gantry.connection.send("WHT")  # should return "0"
```

First, confirm nothing else already holds the serial port — a stale agent
orphaned by an interrupted session (Ctrl-C in a REPL, a dropped SSH
connection, an exception before `disconnect()`) will still be holding it,
and two writers on one controller interleave bytes mid-command:

```bash
ssh -i ~/.ssh/id_ed25519 oak@red.dyn.ucr.edu \
  'ps -eo pid,cmd | grep -E "[g]antry_agent|[s]erial_bridge"'
# and, definitively:
ssh -i ~/.ssh/id_ed25519 oak@red.dyn.ucr.edu \
  'fuser /dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0'
```

Both should be empty. If either shows a process, stop it before connecting.

For direct, protocol-free serial access (the job `serial_bridge.py` used to
do), use `tio` in a terminal on the Pi — one human, one writer:

```bash
ssh -i ~/.ssh/id_ed25519 oak@red.dyn.ucr.edu
tio -m ONLCRNL -b 9600 /dev/ttyUSB0 --local-echo
```

Do not run `tio` and a `gantry_agent.py` session at the same time, for the
same two-writer reason.

### Remaining open items (not blockers for M0-M3, tracked for later)

1. The commander native `INB`/`SOB` table (see "Digital IO" above) is
   decoded from the `.dsm` files' Named-IO declarations and cross-checked
   against one live `INB 1-8` read — still needs physical toggle-testing
   switch-by-switch to fully confirm.
2. `Z_Brake_Status`/`TLim` are confirmed to live on the responder's own
   input bank with no ASCII-reachable path from here (see "Digital IO"
   above) — resolving this for real would need either the Snap2Motion IDE's
   Named IO config or a from-scratch Binary Commands protocol client;
   not planned unless it's decided to be worth building.
3. `MTT` (motor type) observed as `16` on this hardware in addition to the
   documented `0`=stepper/`8`=servo — meaning unknown, treated as opaque.
4. Two files referenced in the user's own code comments
   (`python_package_plan.md`, `oem2t_protocol_reference.md`) were not found
   anywhere searched on the Pi — may exist elsewhere.
5. Minor polarity-flag disagreements between `eab-2026-07-16.dsm` and
   `600011-00-eab4.dsm` on a few home/limit inputs — the former (and its
   2026-07-17 successor, which agrees with it exactly) is treated as
   authoritative per the user, worth a physical sanity check during probing.
