# Confluence MQTT Integration — Status

## What this is

`weir`, `flow`, and `gauge` used to talk to their hardware (a Teknic
ClearCore stepper for the weir gate + flow's solenoid valves, a Fuji VFD
pump drive, a Massa ultrasonic sensor) over direct USB serial from
whatever machine ran the laguna script. That hardware is now wired
instead to **red.lab**, a Raspberry Pi running
[confluence](https://github.com/ericbarefoot/confluence) — a small daemon
that owns the serial connections and exposes them over MQTT (mosquitto,
also running on red.lab). Laguna's three subsystems are now MQTT clients
of that broker rather than touching serial directly.

Branches: `feat-confluence-integration` (laguna), `feat-laguna-integration`
(confluence). Neither is merged yet.

## How it works

No real discovery — pure naming convention. Confluence publishes/listens
on topics `{node_name}/{interface}[/commands|/replies]`; laguna derives
the identical strings from one shared config value (`mqtt.node_name`,
currently `"UCRS Confluence Node 1"` to match red.lab's
`confluence_config.json`). Both sides independently build the same
string; mosquitto just relays whatever gets published on it. See
`config.py`'s comment above its `weir` default section, and each
subsystem's `from_config()`.

- **Gauge** is pure async streaming — confluence polls the Massa sensor on
  a schedule and publishes readings; laguna just subscribes and decodes.
- **Weir/flow** are motion/actuation, so they use a request/reply layer on
  top of plain pub/sub (`laguna/src/laguna/mqtt/request_reply.py`):
  laguna publishes a command with a `request_id`, and blocks until
  confluence replies echoing that ID (or times out). This keeps
  `go_to_elevation()` etc. meaning "hardware acknowledged," not just
  "message sent." Weir's gate axis and flow's qin/qaux valves both live on
  one physical ClearCore (`confluence/Interfaces/Teknic_ClearCore/`);
  flow's pump goes through `confluence/Interfaces/Fuji_Frenic_VFD/`
  (extended with the same request/reply handling, alongside its
  pre-existing legacy command path).

## Status by subsystem

- **Gauge — done, validated live.** Full round trip confirmed against
  real red.lab hardware (`examples/example_10_gauge_mqtt_read.py`). One
  real bug found and fixed in the process: confluence's `dist_mm` field is
  already millimeters, not centimeters — the old serial driver's `*10`
  scaling had been carried over by mistake.
- **Weir — implemented, partially validated live, one bug found and fixed
  tonight.** Real hardware testing reached "set velocity" before hitting
  `get_velocity()` returning NaN — root cause was a design bug, not a
  session/config issue: `get_velocity()` only read a client-side cache
  that resets on reconnect, instead of the live `velocity_setpoint` field
  confluence already publishes (from the ClearCore's real `VelSetPoint`
  register). Fixed to read live, matching `get_elevation()`'s pattern; 4
  new tests added. Not yet committed.
- **Flow — implemented, not yet live-tested.** One protocol mismatch
  found and fixed via code review (not live testing): `get_status()`
  assumed field names (`state_message`, `e_stop`, `setpoint`) that don't
  match confluence's actual published VFD status dict (`state`,
  `"freq setpoint"` as a `"42.00 Hz"` string, no `e_stop` published at
  all). Fixed to match reality.

## Known open issue

`SaflWeirController.wait_for_move()` polls the status topic, but
confluence only republishes weir status on its own schedule (0.5s
interval), not in response to a command — so a status message that's
stale from *before* a move was issued could make `wait_for_move()` return
instantly, as if the move already finished. Documented in the method's
docstring; not fixed, since fixing it blind (without real hardware timing
to verify against) risked guessing wrong. Needs a live-hardware check.

## Infrastructure on red.lab

- `confluence.service` (systemd unit) — runs the daemon detached from any
  SSH session, `Restart=on-failure`, ordered after mosquitto. Config
  changes require `sudo systemctl restart confluence` (no hot-reload —
  each interface module reads `confluence_config.json` once at import).
- `scripts/setup_udev_aliases.sh` (in laguna) — gives the shared 4-port
  FTDI hub stable `/dev/teknic_clearcore` / `/dev/massa_ultrasonic` /
  `/dev/fuji_vfd` aliases (plus `/dev/macron_gantry` for the separate,
  already-reserved gantry bridge cable), keyed on USB serial number +
  interface number so they survive reboots/re-enumeration.
- `confluence/scripts/probe_serial_ports.py` and `probe_clearcore.py` —
  one-off diagnostics used to map the hub's ports to physical devices;
  kept in the repo but not part of the running system.

## What's left

1. Live-hardware validation of weir motion (small test move, verify
   direction/magnitude, resolve the `wait_for_move()` question above) and
   flow (valve toggle, low-flow pump start/stop, combined `estop()`).
2. Commit laguna's currently-uncommitted weir/flow work once that
   validation passes.
3. Docs/example config polish once both subsystems are confirmed working
   (currently only gauge has a working example script).
4. Eventually merge both feature branches.
