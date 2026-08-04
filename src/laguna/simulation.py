"""Rehearse a whole experiment with no hardware attached.

A flume run is expensive to set up and impossible to repeat identically. A
schedule with a typo, a survey that overruns the gap between events, or a
scan spec missing a key are all mistakes you would rather find at a desk
than after the water is up — but until now the only way to exercise a script
end to end was to attach the rig.

The pieces already existed, just not anywhere a user could reach them: the
test suite has a scripted serial transport and a full fake GoSdk, and
``GCodeExecutor`` has had a ``dry_run`` flag that was never surfaced through
``GantryController``, config, or ``FlumeLab``. This module promotes them into
the package and wires them to one switch::

    lab = FlumeLab("config.yaml", simulate=True)
    lab.connect_all()          # succeeds; nothing is on the network
    lab.scheduler.run(300)     # the real scheduler, the real clock

What is real and what is not
----------------------------
**Real:** the scheduler, the clock, the event log, the run manifest, the
frame transforms, the survey planner, and the safety verbs. Timing is real
too — a 300-second run takes 300 seconds unless you speed the clock up.

**Simulated (gantry, gocator only):** the wire. Serial commands are answered
from a small model of the controller rather than a PLC, and the Gocator
returns synthetic surfaces of the right shape and dtype.

**Not present at all (weir, flow, gauge, pi_cameras, dslr_cameras,
od2000/wtt12l):** there is no simulated Modbus VFD, serial stepper, ultrasonic
sensor, or SSH camera agent yet. ``simulate_config()`` drops these sections
from the config rather than constructing the real hardware controllers for
them — a fail-closed guard against a "rehearsal" silently commanding real
hydraulics, not a simulation of what they would do. A schedule that depends
on one of these firing is not exercised by simulate=True today.

So this catches structural mistakes in the gantry/gocator half — a schedule
that never fires, a survey that overruns, a scan spec missing a key, a
subsystem that fails to quiesce — and cannot catch physical ones (a mounting
sign, a feed rate the gantry cannot actually hold, a target outside the work
envelope) or anything involving the dropped subsystems. It is a rehearsal of
part of the rig, not a simulator of all of it, and the distinction is worth
keeping in mind when a rehearsed script meets real hardware.
"""

from __future__ import annotations

import logging
import re
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class SimulatedSnapConnection:
    """Answers the OEM-2T's ASCII protocol from a small in-memory model.

    Not a scripted lookup table like the test fixture — a script has to
    anticipate every command, and a rehearsal's whole point is to run
    commands nobody anticipated. This tracks per-axis position and speed and
    answers queries consistently, so a move followed by a position read
    returns what the move asked for.

    Moves complete instantly. That keeps a rehearsal about *structure*
    rather than duration; a survey's real cost comes from
    ``Survey.duration_s()``, which is computed rather than waited out.
    """

    #: "A1 ACP", "C1 BMT 10 0", "SOB 4 1" ...
    _COMMAND = re.compile(r"^(?P<prefix>[AC]\d+|SOB|INB)\s*(?P<verb>[A-Z]{3})?\s*(?P<args>.*)$")

    def __init__(self) -> None:
        self._connected = False
        self.sent: list = []
        #: axis token -> {"acp": mm, "spd": mm/s, "mtr": bool}
        self.axes: Dict[str, Dict[str, Any]] = {}
        self.outputs: Dict[str, int] = {}
        #: Which axes a coordinated group command addresses, in order. The
        #: gcode path drives X/Y as group 1 — see GCodeExecutor's Z/XY split.
        self._group_members = ["A1", "A2"]

    # -- SnapConnection interface --------------------------------------

    def connect(self) -> None:
        self._connected = True
        logger.info("Simulated gantry connected (no hardware)")

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def send(self, command: str) -> str:
        self.sent.append(command)
        return self._answer(command.strip())

    # -- the model ------------------------------------------------------

    def _axis(self, token: str) -> Dict[str, Any]:
        return self.axes.setdefault(token, {"acp": 0.0, "spd": 10.0, "mtr": False})

    def _answer(self, command: str) -> str:
        match = self._COMMAND.match(command)
        if not match:
            return "0"
        prefix = match.group("prefix")
        verb = (match.group("verb") or "").upper()
        args = match.group("args").split()

        if prefix == "SOB":                      # set output bit (brakes)
            if len(args) >= 2:
                self.outputs[args[0]] = int(args[1])
            return "0"
        if prefix == "INB":                      # read input bit
            return "0"

        # A coordinated group command (C1) drives its member axes. Without
        # this a rehearsed move appeared to do nothing — reading A1 ACP after
        # "C1 BMT ..." returned 0 — which would make a survey look like it
        # never moved and hide exactly the mistakes rehearsal is for.
        if prefix.startswith("C") and verb in ("BMT", "MVT", "BMB", "MVB"):
            for i, value in enumerate(args):
                member = self._group_members[i] if i < len(self._group_members) else None
                if member is None:
                    continue
                target = self._axis(member)
                if verb in ("BMT", "MVT"):
                    target["acp"] = float(value)
                else:
                    target["acp"] += float(value)
            return "0"

        axis = self._axis(prefix)
        if verb == "ACP":
            if args:                             # write: redefine position
                axis["acp"] = float(args[0])
            return f"{axis['acp']:.4f}"
        if verb == "SPD":
            if args:
                axis["spd"] = float(args[0])
            return f"{axis['spd']:.4f}"
        if verb in ("BMT", "MVT"):               # absolute move
            if args:
                axis["acp"] = float(args[0])
            return "0"
        if verb in ("BMB", "MVB"):               # relative move
            if args:
                axis["acp"] += float(args[0])
            return "0"
        if verb == "MIF":                        # move finished?
            return "1"                           # instantly, by construction
        if verb == "MTR":
            if args:
                axis["mtr"] = bool(int(args[0]))
            return "1" if axis["mtr"] else "0"
        if verb in ("BST", "ABT", "STP"):
            return "0"
        if verb == "INI":
            return "0"
        # Unknown queries answer 0 rather than raising: a rehearsal should
        # not fall over on a command the model doesn't know about.
        return "0"


def simulated_gocator_lib() -> Any:
    """A fake GoSdk that returns synthetic surfaces.

    Imported lazily from the scanner's own simulation helper so the SDK
    bindings are not required to rehearse a run.
    """
    from .scanner.simulation import SimulatedGoSdkLib

    return SimulatedGoSdkLib()


#: Config sections with a real simulated transport (see SimulatedSnapConnection
#: and SimulatedGoSdkLib). Anything else has no simulated backend at all.
_SIMULATED_SECTIONS = ("gantry", "gocator")

#: Sections that would otherwise construct a real hardware controller with no
#: simulated backend to fall back to — a Modbus VFD, a serial stepper motor,
#: an ultrasonic sensor, SSH to a Pi. Dropped under simulate=True rather than
#: silently connecting to real hardware during what is supposed to be a
#: rehearsal — a fail-closed guard, not a simulation of what they would do.
_NO_SIMULATED_BACKEND = ("weir", "flow", "gauge", "pi_cameras", "dslr_cameras",
                          "od2000", "wtt12l", "wtt12l_powerprox")


def simulate_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Rewrite a config so every subsystem builds a simulated transport.

    Deliberately does **not** remove the gantry/gocator subsystems, which do
    have simulated backends: a rehearsal must exercise the same set the real
    run would, or it proves nothing about the schedule.

    Every other subsystem section (weir, flow, gauge, pi_cameras,
    dslr_cameras, the rangefinders) has **no simulated backend at all** —
    there is no simulated VFD, stepper, ultrasonic sensor, or SSH camera
    agent. Constructing the real controller classes for these under
    simulate=True would silently contact real hardware during what is
    supposed to be a hardware-free rehearsal, so they are dropped from the
    config entirely rather than exercised for real. This is a fail-closed
    guard, not equivalent coverage — see module docstring.
    """
    out = dict(config)
    if "gantry" in out:
        gantry = dict(out["gantry"])
        gantry["transport"] = "simulated"
        # Motion must be permitted, or every move is refused and the
        # rehearsal only proves the safety gate works.
        gantry["safe_mode"] = False
        out["gantry"] = gantry
    if "gocator" in out:
        gocator = dict(out["gocator"])
        gocator["simulated"] = True
        out["gocator"] = gocator

    dropped = [s for s in _NO_SIMULATED_BACKEND if s in out]
    for section in dropped:
        del out[section]
    if dropped:
        logger.warning(
            "SIMULATION MODE: dropped section(s) %s — no simulated backend "
            "exists for them yet, so they are excluded rather than "
            "connecting to real hardware. The rehearsal does not exercise "
            "their schedules.", dropped,
        )
    return out


__all__ = [
    "SimulatedSnapConnection",
    "simulated_gocator_lib",
    "simulate_config",
]
