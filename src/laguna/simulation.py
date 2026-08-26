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

**Simulated — every subsystem in laguna.registry.SUBSYSTEM_REGISTRY**
(gantry, gocator, weir, flow, gauge, pi_cameras, dslr_cameras, od2000,
wtt12l): commands succeed and log exactly as they would against real
hardware — this is what actually exercises a schedule end to end, proving
the right commands fire in the right order. Readings come back as NaN (or
an equivalent "unknown" placeholder for non-numeric fields), never a
fabricated physically-plausible number — a rehearsal checks that the
script and plan are well-formed and execute as scheduled, not physical
feasibility. Physical-limit checking is fences.py's job (real, see above),
not this module's. Gantry moves complete instantly, a simulated Gocator
scan is a small fixed-size synthetic surface (~80,000 cells, not scaled to
whatever a real scan's config asks for — see
laguna.scanner.simulation.SIM_ROWS/SIM_COLS — so a rehearsal with scans on
a tight interval does not accumulate large files), and camera captures
"return" placeholder filenames with no file behind them.

``_NO_SIMULATED_BACKEND`` is empty today — every registered subsystem has
a simulated path. It's kept as the fail-closed guard (drop the section
rather than construct the real controller) for whatever gets added to the
registry next without one yet, not deleted outright.

So this catches structural mistakes across the whole opt-in rig — a
schedule that never fires, a survey that overruns, a scan spec missing a
key, a subsystem that fails to quiesce, a camera trigger wired to the
wrong schedule column — and cannot catch physical ones (a mounting sign, a
feed rate the gantry cannot actually hold, a target outside the work
envelope, whether a valve would actually open). It is a rehearsal of the
script and plan, not a simulator of the physical apparatus, and the
distinction is
worth keeping in mind when a rehearsed script meets real hardware.
"""

from __future__ import annotations

import logging
import re
from typing import Any, Dict, Optional, Tuple

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
        """Initialize the simulated Snap connection."""
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
        """Connect the simulated gantry."""
        self._connected = True
        logger.info("Simulated gantry connected (no hardware)")

    def disconnect(self) -> None:
        """Disconnect the simulated gantry."""
        self._connected = False

    @property
    def is_connected(self) -> bool:
        """Check if connected.

        Returns:
            Connection status.
        """
        return self._connected

    def send(self, command: str) -> str:
        """Send a command and get the response.

        Args:
            command: ASCII command string.

        Returns:
            Response string.
        """
        self.sent.append(command)
        return self._answer(command.strip())

    # -- the model ------------------------------------------------------

    def _axis(self, token: str) -> Dict[str, Any]:
        """Get or create an axis state dictionary."""
        return self.axes.setdefault(token, {"acp": 0.0, "spd": 10.0, "mtr": False})

    def _answer(self, command: str) -> str:
        """Process a command and return the response."""
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


#: Config sections with a simulated backend (see SimulatedSnapConnection,
#: SimulatedGoSdkLib, and the weir/flow/gauge/pi_cameras/dslr_cameras/
#: od2000/wtt12l "simulated" flag each subsystem's own connect()/action
#: methods check directly — weir/flow/gauge are MQTT clients now, so their
#: "simulated" path just skips MQTT entirely rather than swapping in a
#: fake serial driver). This is now every entry in
#: laguna.registry.SUBSYSTEM_REGISTRY — simulate=True rehearses the whole
#: opt-in rig, not a subset of it.
_SIMULATED_SECTIONS = ("gantry", "gocator", "weir", "flow", "gauge",
                        "pi_cameras", "dslr_cameras", "od2000", "wtt12l")

#: Sections that would otherwise construct a real hardware controller with
#: no simulated backend to fall back to. Empty today — kept as the
#: fail-closed guard for whatever gets added to the registry next without a
#: simulated path yet, rather than deleting the mechanism. Dropped under
#: simulate=True rather than silently connecting to real hardware during
#: what is supposed to be a rehearsal.
_NO_SIMULATED_BACKEND: Tuple[str, ...] = ()


def simulate_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Rewrite a config to use simulated backends for all subsystems.

    Every _SIMULATED_SECTIONS entry adds `"simulated": True` (gantry/gocator
    use their own transport/simulated conventions). A rehearsal must exercise
    the same set the real run would. Sections in _NO_SIMULATED_BACKEND are
    dropped entirely rather than connecting to real hardware.

    Args:
        config: Original config dictionary.

    Returns:
        Modified config with simulated backends enabled.
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
    for section in ("weir", "flow", "gauge", "pi_cameras", "dslr_cameras",
                     "od2000", "wtt12l"):
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
    "simulated_gocator_lib",
    "simulate_config",
]
