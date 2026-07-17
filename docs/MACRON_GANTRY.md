# Macron Gantry (Snap2Motion / OEM-2T) Integration

Driver for the lab's 3-axis-plus-rotary (plus 4 more on a networked expansion
node — see "Axis model" below) robotic gantry, controlled through a
Modusystems OEM-2T rev D PLC over an ASCII text protocol. Lives at
`src/laguna/robot/macron/`.

**Status as of 2026-07-16: M0-M3 complete and merged onto `feat-macron-api`
(rebased on `develop`). All hardware-free work is done and tested (182/184
suite passing — 2 pre-existing failures in `test_core.py` unrelated to this
work). M4 (read-only hardware verification) is blocked on the controller not
currently running a program with the ASCII interpreter active — see "Current
status" below.**

## Topology

```
laguna (this PC) --SSH/socket--> red.dyn.ucr.edu (Pi, "oak" account)
                                      |
                                 serial_bridge.py (raw TCP<->serial passthrough, port 9700)
                                      |
                                 RS232 --> OEM-2T rev D controller --> gantry motors
```

The controller is physically too far from this PC for direct serial, so a
Raspberry Pi sits next to it. Two ways to reach it, both implementing the
same `SnapConnection` interface so the rest of the driver doesn't care which
is active:

1. **`RS232Connection(port="socket://red.dyn.ucr.edu:9700")`** — default.
   Talks to `serial_bridge.py`, a raw, protocol-unaware TCP↔serial
   passthrough already running on the Pi (started manually — see "Resuming
   after a Pi reboot" below). No Pi-side `laguna` code needed. Requires
   `serial.serial_for_url()`, not plain `serial.Serial()` — the plain form
   silently fails on `socket://` URLs.
2. **`PiGantryConnection`** (`pi_bridge.py`) — persistent SSH+JSON-line
   transport. Deploys and drives `gantry_agent.py` (standalone, pyserial-only,
   no `laguna` install needed on the Pi) as a long-running process over one
   SSH channel, for lower per-command latency than a fresh SSH connect+exec
   each time. Intended for eventual production use.

Both transports (and `RS232Connection`/`EthernetConnection` generally) can be
wrapped in **`SafeModeConnection`** to add the same query-only allowlist gate
`PiGantryConnection` has natively — see "Safety model" below.

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

**The `.dsm` project files are stale for IO wiring.** All of `XXHome`,
`XXLim`, `YHome`, `YLim`, `Zhome`, `ZLim`, `Y_Brake`, `Z_Brake` (outputs), and
`Y_Brake_Status` are declared in every `.dsm` file as **IsoIO** expansion-
board channels — but that board is **not physically installed** (`ISI`/`ISO`
commands fail with error 263). The user has since **physically rewired all
of these onto the controller's native `INB`/`SOB` bus** (also fixing a 24V
pull-up wiring issue from an earlier attempt), but the project files were
never updated to reflect the rewire.

**Only two native channels are currently confirmed** (from
`eab-2026-07-16.dsm`, treated as authoritative over `600011-00-eab4.dsm`
where they disagree on a few polarity flags):
- `Z_Brake_Status` = `INB 1`
- `TLim` = `INB 2`

Everything else needs physical probing (toggle each switch/output, diff
`INB`/`SOB` snapshots — the pattern in the user's own
`~/modusystems_dev/status_snapshot.py` on the Pi is the validated tool for
this) before `IOMap` can be filled in. `IOMap` in `commands.py` defaults
every unconfirmed field to `None`; brake-control methods
(`disengage_brake`/`engage_brake`/`brake_is_disengaged`) raise `ValueError`
if asked to use a channel that isn't set, rather than silently doing nothing
or guessing.

**Homing is explicitly deprioritized** — the user does not plan to run it
soon ("we will not run homing anyway"). `HomingProcedure`'s architecture is
sound and fully unit-tested, just not a near-term verification priority.

**Also real, not hypothetical**: `X` and `Y` currently have uninitialized
software position limits (`PLT`/`NLT` ≈ ±822,536,056) — soft-limit
protection is **not active** on those two axes right now.
`MMCCommands.validate_soft_limits()` detects and raises on this.

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
gantry = GantryController.from_config(config.get("gantry"))
lab.add(gantry)  # subsystem_name = "gantry" -> lab.gantry
```

## Current status / resuming work

**M0-M3 done** (branch/protocol fixes, G-code, Pi bridge, config — all
hardware-free, all tested). See `feat-macron-api` branch commit history for
the milestone-by-milestone breakdown (5 commits: rebase, M1, M2, M3, and a
`SafeModeConnection` safety fix made just before the first hardware
attempt).

**M4 (read-only hardware verification) is blocked**, not by our code: a
`WHT` query times out through three independent paths (our new bridge code,
a raw socket bypassing everything, and the user's own `oem2t.py` running
*locally on the Pi* talking directly to the serial port with zero laguna
code involved). The USB-serial adapter itself is healthy (`dmesg`/`lsusb`
clean, no disconnects). The user's diagnosis: the controller may not
currently be running a program with the `AsciiCommands`/`MonitorCommPort`
component active — this needs the Snap2Motion Windows IDE connected
directly to check/load/run the right program. This is not something
reachable from the Pi (`serial_bridge.py` only forwards raw bytes) or from
this codebase.

### Resuming after a Pi reboot or session gap

`serial_bridge.py` is **not** a systemd service — it was started manually
and will not survive a Pi reboot. To check/restart it:

```bash
ssh -i ~/.ssh/id_ed25519 oak@red.dyn.ucr.edu 'ps -ef | grep serial_bridge | grep -v grep'
# if nothing:
ssh -i ~/.ssh/id_ed25519 oak@red.dyn.ucr.edu \
  'cd ~/modusystems_dev && nohup python3 serial_bridge.py > bridge.log 2>&1 & disown'
```

Once the controller is confirmed running the right program, retry:

```python
from laguna.robot.macron import RS232Connection, SafeModeConnection
conn = SafeModeConnection(RS232Connection(port="socket://red.dyn.ucr.edu:9700"), safe_mode=True)
conn.connect()
conn.send("WHT")  # should return "0", not time out
```

### Remaining open items (not blockers for M0-M3, tracked for later)

1. Native `INB`/`SOB` channels for X/Y/Z home+limit switches and both brake
   outputs — unknown, need physical probing (toggle + diff snapshots).
2. `MTT` (motor type) observed as `16` on this hardware in addition to the
   documented `0`=stepper/`8`=servo — meaning unknown, treated as opaque.
3. Names/roles for Responder-node axes 5-8 — unknown.
4. Two files referenced in the user's own code comments
   (`python_package_plan.md`, `oem2t_protocol_reference.md`) were not found
   anywhere searched on the Pi — may exist elsewhere.
5. Minor polarity-flag disagreements between `eab-2026-07-16.dsm` and
   `600011-00-eab4.dsm` on a few home/limit inputs — the former is treated
   as authoritative per the user, worth a physical sanity check during
   probing.
