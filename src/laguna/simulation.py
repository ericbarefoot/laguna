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
Fence checking is real too — a rehearsed move that would violate a fence
still raises, since that's a structural mistake in the script, not a
physical unknown.

**Simulated (gantry, gocator, weir, flow, gauge, pi_cameras,
dslr_cameras):** commands succeed and log exactly as they would against
real hardware — this is what actually exercises a schedule end to end,
proving the right commands fire in the right order. Readings come back as
NaN (or an equivalent "unknown" placeholder for non-numeric fields), never
a fabricated physically-plausible number — a rehearsal checks that the
script and plan are well-formed and execute as scheduled, not physical
feasibility. Physical-limit checking is fences.py's job (real, see above),
not this module's. Gantry moves complete instantly and camera captures
"return" placeholder filenames with no file behind them.

**Not present at all (od2000, wtt12l):** there is no simulated IO-Link/AL1342
model yet. ``simulate_config()`` drops these sections from the config
rather than constructing the real controllers for them — a fail-closed
guard against a "rehearsal" silently commanding real hardware, not a
simulation of what they would do. A schedule that depends on one of these
firing is not exercised by simulate=True today.

So this catches structural mistakes across almost the whole rig — a
schedule that never fires, a survey that overruns, a scan spec missing a
key, a subsystem that fails to quiesce, a camera trigger wired to the
wrong schedule column — and cannot catch physical ones (a mounting sign, a
feed rate the gantry cannot actually hold, a target outside the work
envelope, whether a valve would actually open) or anything involving the
still-dropped rangefinder subsystems. It is a rehearsal of the script and
plan, not a simulator of the physical apparatus, and the distinction is
worth keeping in mind when a rehearsed script meets real hardware.
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


class SimulatedTeknicMotor:
    """Stand-in for safl_ocean_hardware's TeknicMotor — shared by
    SaflWeirController (position) and SaflFlowController (digital IO).

    Every command succeeds instantly; every reading comes back NaN (or None
    for non-numeric status fields) rather than a fabricated position or
    fault state — see module docstring on why this deliberately does not
    track a fake position the way SimulatedSnapConnection tracks a real
    one. Commanding a move here proves the schedule sends the right
    command at the right time; it says nothing about where the axis
    "ended up," because there is no physical model to ask.
    """

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        pass

    def set_absolute_position(self, mm: float) -> None:
        pass

    def move_to_position(self, mm: float) -> bool:
        return True

    def get_position(self) -> float:
        return float("nan")

    def set_velocity(self, mm_per_sec: float) -> None:
        pass

    def poll_status(self) -> Dict[str, Any]:
        return {
            "position": float("nan"), "VelSetPoint": float("nan"),
            "Enabled": None, "MotorInFault": None, "StepsActive": None,
        }

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def wait_for_HLFB(self, timeout: float) -> None:
        pass

    def clear_faults(self) -> None:
        pass

    def find_home(self, home_position_mm: float = 0.0) -> bool:
        return True

    def stop(self) -> None:
        pass

    def set_io(self, channel: int, state: bool) -> None:
        pass


class SimulatedVFD:
    """Stand-in for safl_ocean_hardware's VFD (Fuji pump drive) — see
    SimulatedTeknicMotor's docstring for the same "commands succeed,
    readings are NaN" reasoning.
    """

    def __init__(self) -> None:
        self.setpoint = float("nan")

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        pass

    def set_freq_from_flowrate(self, lpm: float, c0: float, c1: float, c2: float) -> None:
        pass

    def start(self) -> bool:
        return True

    def stop(self) -> bool:
        return True

    def clear_faults(self) -> bool:
        return True

    def poll_state(self) -> Dict[str, Any]:
        return {"state_message": None, "e_stop": None}

    def poll_setpoint(self) -> None:
        pass


class SimulatedMassaSensor:
    """Stand-in for safl_ocean_hardware's MassaSensor (ultrasonic water
    level) — readings are NaN rather than a fabricated water level; see
    SimulatedTeknicMotor's docstring for the same reasoning. Deliberately
    has no dist_cm_array_moving_avg attribute — SaflWaterLevelSensor's
    read_mm_smoothed() already treats that as "no moving average yet" and
    returns NaN, so this needs no special-casing there either.
    """

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        pass

    def read(self) -> Dict[str, Any]:
        return {
            "distance_cm": float("nan"), "temperature": float("nan"),
            "signal_strength": float("nan"),
        }


def simulated_gocator_lib() -> Any:
    """A fake GoSdk that returns synthetic surfaces.

    Imported lazily from the scanner's own simulation helper so the SDK
    bindings are not required to rehearse a run.
    """
    from .scanner.simulation import SimulatedGoSdkLib

    return SimulatedGoSdkLib()


#: Config sections with a simulated backend (see SimulatedSnapConnection,
#: SimulatedGoSdkLib, SimulatedTeknicMotor/SimulatedVFD/SimulatedMassaSensor,
#: and the pi_cameras/dslr_cameras "simulated" flag each subsystem's own
#: connect()/action methods check directly). Anything else has no simulated
#: backend at all.
_SIMULATED_SECTIONS = ("gantry", "gocator", "weir", "flow", "gauge",
                        "pi_cameras", "dslr_cameras")

#: Sections that would otherwise construct a real hardware controller with no
#: simulated backend to fall back to — an IO-Link/AL1342 rangefinder. Dropped
#: under simulate=True rather than silently connecting to real hardware
#: during what is supposed to be a rehearsal — a fail-closed guard, not a
#: simulation of what they would do.
_NO_SIMULATED_BACKEND = ("od2000", "wtt12l", "wtt12l_powerprox")


def simulate_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Rewrite a config so every subsystem with a simulated backend uses it.

    Every _SIMULATED_SECTIONS entry keeps its section and gets a
    `"simulated": True` flag added (gantry/gocator use their own existing
    `transport`/`simulated` conventions instead) — a rehearsal must
    exercise the same set the real run would, or it proves nothing about
    the schedule. The rangefinder sections in _NO_SIMULATED_BACKEND have no
    simulated backend yet, so they're dropped entirely rather than
    constructing the real controller classes under simulate=True. This is
    a fail-closed guard, not equivalent coverage — see module docstring.
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
    for section in ("weir", "flow", "gauge", "pi_cameras", "dslr_cameras"):
        if section in out:
            sub = dict(out[section])
            sub["simulated"] = True
            out[section] = sub

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
    "SimulatedTeknicMotor",
    "SimulatedVFD",
    "SimulatedMassaSensor",
    "simulated_gocator_lib",
    "simulate_config",
]
