"""Pump flow control subsystem."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Optional
import logging

from ..mqtt import MqttSubscriber, RequestTimeout, request
from ..subsystem_logging import SubsystemLogging
from .calibration import PumpCalibration

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)


def _parse_hz(raw: Optional[str]) -> float:
    """Parse confluence's "42.00 Hz"-style display string into a bare float.

    Returns NaN if raw is None or not in the expected format, rather than
    raising — a malformed/missing status field shouldn't crash get_status().
    """
    if raw is None:
        return float("nan")
    try:
        return float(str(raw).split()[0])
    except (ValueError, IndexError):
        return float("nan")


class FlowController(ABC):
    """Abstract interface for pump flow rate and inlet/aux valve control.

    Concrete implementations drive whatever VFD/pump hardware and solenoid
    valves are physically deployed, exposing flow rate in L/min regardless
    of the underlying protocol.
    """

    subsystem_name = "flow"

    @abstractmethod
    def connect(self) -> bool:
        """Open the hardware connection(s).

        Returns:
            True if all required connections were established, False
            otherwise.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the hardware connection(s) and release any held resources."""
        ...

    @abstractmethod
    def set_flowrate(self, lpm: float) -> bool:
        """Set pump flow rate in L/min."""
        ...

    @abstractmethod
    def get_flowrate(self) -> float:
        """Return the most recently commanded flow rate in L/min.

        This is the setpoint last sent via set_flowrate(), not a live
        measurement from a flow sensor.
        """
        ...

    @abstractmethod
    def start(self) -> bool:
        """Start the pump at its current flow rate setpoint.

        Returns:
            True if the start command was sent successfully.
        """
        ...

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # There is deliberately no abstract `stop() -> bool` pump primitive here:
    # `stop()` belongs to the shared safety vocabulary, so how a backend
    # actually stops its drive is its own private business (SaflFlowController
    # uses `_vfd_stop()`). All four verbs must return a note about anything a
    # human needs to know, or None, and must never raise.

    @abstractmethod
    def pause(self) -> Optional[str]:
        """Quiesce the pump, remembering the setpoint so resume() can restore it."""
        ...

    @abstractmethod
    def resume(self) -> Optional[str]:
        """Restore the setpoint captured by pause() and restart the pump."""
        ...

    @abstractmethod
    def stop(self) -> Optional[str]:
        """End cleanly: stop the pump, leaving the valves as they are."""
        ...

    @abstractmethod
    def estop(self) -> Optional[str]:
        """Pump off and both valves closed, each step attempted independently."""
        ...

    @abstractmethod
    def clear_faults(self) -> bool:
        """Clear any latched VFD fault/alarm state.

        Returns:
            True if faults were cleared successfully.
        """
        ...

    @property
    @abstractmethod
    def qin(self) -> bool:
        """Whether the inlet solenoid valve is currently commanded open."""
        ...

    @qin.setter
    @abstractmethod
    def qin(self, state: bool) -> None: ...

    @property
    @abstractmethod
    def qaux(self) -> bool:
        """Whether the auxiliary solenoid valve is currently commanded open."""
        ...

    @qaux.setter
    @abstractmethod
    def qaux(self, state: bool) -> None: ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Return a snapshot of pump/VFD status and valve states.

        Returns:
            Dictionary of status fields. Contents are backend-specific, but
            always include at least `is_connected` and `flowrate_lpm`.
        """
        ...


class SaflFlowController(FlowController, SubsystemLogging):
    """Flow controller backed by a Fuji VFD pump and a shared ClearCore valve axis, via MQTT.

    Both the VFD and the ClearCore that drives the qin/qaux solenoid
    digital outputs are wired to the confluence node on red.lab, which
    exposes each over its own MQTT interface — this class no longer talks
    to serial itself. The valve axis is the same physical ClearCore
    controller the weir gate axis lives on (one controller, two axes), but
    published/commanded as its own confluence interface/topic set
    ("flow_valve"). A single MqttSubscriber is shared across both channels
    to avoid the duplicate-client-ID reconnect fight documented in
    laguna.mqtt.subscriber.MqttSubscriber.
    """

    def __init__(self, config: Dict[str, Any], mqtt_subscriber: MqttSubscriber):
        """Build the controller from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary (see
                Config._get_defaults()'s 'flow' section for the expected
                shape). Recognized keys:
                - vfd_topic_status / vfd_topic_commands / vfd_topic_replies:
                  MQTT topics for the confluence Fuji_Frenic_VFD interface.
                - valve_topic_status / valve_topic_commands /
                  valve_topic_replies: MQTT topics for the confluence
                  Teknic_ClearCore interface's flow-valve axis.
                - command_timeout_s: Seconds to wait for a command reply
                  before treating it as failed (default 5.0).
                - calibration_file: Path to a CSV written by
                  PumpCalibration.to_csv() (see laguna.flow.calibration and
                  docs/reference/pump_calibration.md) mapping VFD frequency
                  to measured discharge for this specific pump/plumbing
                  configuration. When set, set_flowrate() uses this fitted
                  curve instead of the C0/C1/C2 formula below. Preferred
                  over C0/C1/C2 for anything beyond a first rough pass —
                  see that doc for why the hardcoded default coefficients
                  were found to be wrong on first live hardware test.
                - C0, C1, C2: Coefficients of the legacy quadratic pump
                  calibration curve `freq_hz = C2*Q^2 + C1*Q + C0` (Q in
                  L/min), used by set_flowrate() only when calibration_file
                  isn't set. These are empirically fit per pump/plumbing
                  configuration — not physical constants — and this
                  inline-coefficient form can't represent a
                  non-monotonic/oddly-shaped curve the way a real
                  calibration fit can. The resulting frequency is clamped
                  to [0, 60] Hz here before sending either way.
                - log_level / event_log_verbosity: see laguna.subsystem_logging
                  (both default 'INFO').
                - simulated: skip MQTT entirely — every command "succeeds"
                  immediately (default False).
            mqtt_subscriber: MqttSubscriber shared across the VFD and valve
                channels.
        """
        self._vfd_status_topic = config.get(
            "vfd_topic_status", "SAFL Confluence Node 1/Fuji_Frenic_VFD"
        )
        self._vfd_commands_topic = config.get(
            "vfd_topic_commands", "SAFL Confluence Node 1/Fuji_Frenic_VFD/commands"
        )
        self._vfd_replies_topic = config.get(
            "vfd_topic_replies", "SAFL Confluence Node 1/Fuji_Frenic_VFD/replies"
        )
        self._valve_status_topic = config.get(
            "valve_topic_status", "SAFL Confluence Node 1/flow_valve"
        )
        self._valve_commands_topic = config.get(
            "valve_topic_commands", "SAFL Confluence Node 1/flow_valve/commands"
        )
        self._valve_replies_topic = config.get(
            "valve_topic_replies", "SAFL Confluence Node 1/flow_valve/replies"
        )
        self._command_timeout_s = config.get("command_timeout_s", 5.0)
        self.C0 = config.get("C0", 4.902)
        self.C1 = config.get("C1", 58.49)
        self.C2 = config.get("C2", 0.08956)
        calibration_file = config.get("calibration_file")
        self._calibration: Optional[PumpCalibration] = (
            PumpCalibration.from_csv(calibration_file) if calibration_file else None
        )
        self.log_level = config.get("log_level", "INFO")
        self.event_log_verbosity = config.get("event_log_verbosity", "INFO")
        self._simulated = config.get("simulated", False)

        self._mqtt = mqtt_subscriber
        self._is_connected = False
        self._current_flowrate = 0.0
        self._qin_state = False
        self._qaux_state = False

    @classmethod
    def from_config(cls, config: "Config") -> "SaflFlowController":
        """Build from the lab's Config (its 'flow:' section and shared 'mqtt:' section).

        Derives default topics from mqtt.node_name — the flow section
        itself only needs topic_* keys to override those defaults.
        """
        mqtt_config = config.get("mqtt")
        node_name = mqtt_config.get("node_name", "SAFL Confluence Node 1")
        section = dict(config.get("flow"))
        section.setdefault("vfd_topic_status", f"{node_name}/Fuji_Frenic_VFD")
        section.setdefault("vfd_topic_commands", f"{node_name}/Fuji_Frenic_VFD/commands")
        section.setdefault("vfd_topic_replies", f"{node_name}/Fuji_Frenic_VFD/replies")
        section.setdefault("valve_topic_status", f"{node_name}/flow_valve")
        section.setdefault("valve_topic_commands", f"{node_name}/flow_valve/commands")
        section.setdefault("valve_topic_replies", f"{node_name}/flow_valve/replies")
        mqtt_subscriber = MqttSubscriber(mqtt_config)
        return cls(section, mqtt_subscriber)

    def connect(self) -> bool:
        """Connect the underlying MQTT subscriber and subscribe to VFD + valve topics.

        Blocks (up to a few seconds) for the broker handshake to actually
        complete before returning — connect() alone only starts it
        asynchronously (see MqttSubscriber.wait_until_connected()). Without
        this, a command issued immediately after connect() (e.g. setting
        qin/qaux) can race the handshake and fail with "MqttSubscriber is
        not connected" even though this method already returned True.

        Returns:
            True if connected successfully (or if simulated); False if the
            broker handshake didn't complete in time.
        """
        if self._simulated:
            self._is_connected = True
            return True
        if not self._mqtt._is_connected:
            ok = self._mqtt.connect()
            if not ok or not self._mqtt.wait_until_connected():
                return False
        self._mqtt.subscribe(self._vfd_status_topic)
        self._mqtt.subscribe(self._vfd_replies_topic)
        self._mqtt.subscribe(self._valve_status_topic)
        self._mqtt.subscribe(self._valve_replies_topic)
        self._is_connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect the underlying MQTT subscriber. Safe to call when already disconnected."""
        if self._simulated:
            self._is_connected = False
            return
        self._mqtt.disconnect()
        self._is_connected = False

    def _require_connected(self) -> None:
        """Raise RuntimeError if connect() hasn't succeeded yet."""
        if not self._is_connected:
            raise RuntimeError(f"{self.__class__.__name__} is not connected")

    def _command_vfd(self, command: str, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Send a command to the confluence Fuji_Frenic_VFD interface and block for its reply."""
        if self._simulated:
            return {"ok": True}
        return request(
            self._mqtt,
            self._vfd_commands_topic,
            self._vfd_replies_topic,
            command,
            args,
            timeout=self._command_timeout_s,
        )

    def _command_valve(self, command: str, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Send a command to the confluence Teknic_ClearCore valve axis and block for its reply."""
        if self._simulated:
            return {"ok": True}
        return request(
            self._mqtt,
            self._valve_commands_topic,
            self._valve_replies_topic,
            command,
            args,
            timeout=self._command_timeout_s,
        )

    def set_flowrate(self, lpm: float) -> bool:
        """Set the pump's target flow rate.

        Converts `lpm` to a VFD drive frequency using this instance's
        PumpCalibration (loaded from `calibration_file`, see __init__) if
        one is configured, falling back to the legacy `C2*Q^2 + C1*Q + C0`
        formula otherwise. Clamps to the VFD's [0, 60] Hz range and sends
        it as the new setpoint. This does not itself start the pump — call
        start() to begin running at the new setpoint.

        Returns:
            True if the confluence node acknowledged the new setpoint;
            False on timeout (the cached setpoint is left unchanged).

        Raises:
            RuntimeError: If not connected.
            ValueError: If a PumpCalibration is configured and `lpm` isn't
                achievable within its calibrated frequency range (see
                PumpCalibration.hz_for_lpm()).
        """
        self._require_connected()
        if self._calibration is not None:
            freq_hz = self._calibration.hz_for_lpm(lpm)
        else:
            freq_hz = self.C2 * lpm**2 + self.C1 * lpm + self.C0
        freq_hz = max(0.0, min(60.0, freq_hz))
        try:
            reply = self._command_vfd("set_setpoint_hz", {"freq_hz": freq_hz})
        except RequestTimeout:
            logger.error("set_flowrate(%.2f): no reply from confluence node", lpm)
            return False
        ok = bool(reply.get("ok", True))
        if ok:
            self._current_flowrate = lpm
        self.log_event("set_flowrate", flowrate_lpm=f"{lpm:.2f}")
        return ok

    def set_frequency_hz(self, freq_hz: float) -> bool:
        """Set the VFD drive frequency directly, bypassing the L/min calibration curve.

        set_flowrate() converts L/min to Hz via the C0/C1/C2 quadratic
        curve, which is only as good as that calibration — useful for
        commanding a known frequency directly while that curve is being
        verified/re-derived on real hardware (e.g. example_13's live
        pump check), or for anything that doesn't want the L/min
        abstraction at all. Clamps to the VFD's [0, 60] Hz range, same as
        set_flowrate(). Does not itself start the pump — call start() to
        begin running at the new setpoint.

        Since this bypasses the L/min curve entirely, it invalidates
        get_flowrate()'s cached setpoint rather than leaving a stale
        number that no longer corresponds to what's actually commanded —
        get_flowrate() returns NaN after this until set_flowrate() is
        called again.

        Returns:
            True if the confluence node acknowledged the new setpoint;
            False on timeout (the cached L/min setpoint is left as NaN
            regardless).

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        freq_hz = max(0.0, min(60.0, freq_hz))
        try:
            reply = self._command_vfd("set_setpoint_hz", {"freq_hz": freq_hz})
        except RequestTimeout:
            logger.error("set_frequency_hz(%.2f): no reply from confluence node", freq_hz)
            self._current_flowrate = float("nan")
            return False
        ok = bool(reply.get("ok", True))
        self._current_flowrate = float("nan")
        self.log_event("set_frequency_hz", freq_hz=f"{freq_hz:.2f}")
        return ok

    def get_flowrate(self) -> float:
        """Return the most recently commanded flow rate in L/min.

        This is a locally cached setpoint, not a live sensor measurement —
        it is available even when disconnected, reflecting whatever was
        last successfully passed to set_flowrate().
        """
        return self._current_flowrate

    def start(self) -> bool:
        """Start the pump at its current VFD frequency setpoint.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            reply = self._command_vfd("start_motor")
        except RequestTimeout:
            logger.error("start(): no reply from confluence node")
            return False
        self.log_event("start")
        return bool(reply.get("ok", True))

    def _vfd_stop(self) -> None:
        """Stop the pump drive itself. See stop() for the safety verb.

        Raises rather than returning a bool that a caller could ignore —
        pause()/stop()/estop() all call this expecting a failure to stop
        the pump to surface as an exception they catch and turn into their
        required "note about anything a human needs to know" (see the
        module-level safety-verb contract above FlowController.pause());
        this used to swallow both a timeout and an `ok: False` rejection
        into a return value none of those three callers checked, so they
        could report success while the pump kept running.

        Raises:
            RuntimeError: If not connected, if the confluence node doesn't
                reply in time, or if it replies with `ok: False`.
        """
        self._require_connected()
        try:
            reply = self._command_vfd("stop_motor")
        except RequestTimeout as exc:
            raise RuntimeError("_vfd_stop(): no reply from confluence node") from exc
        if not reply.get("ok", True):
            raise RuntimeError(f"_vfd_stop(): confluence node rejected the command: {reply}")

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def pause(self) -> Optional[str]:
        """Stop the pump, remembering the setpoint so resume() can restore it.

        Returns:
            Note about any issues encountered, or None if successful.
        """
        self._paused_flowrate = self._current_flowrate
        try:
            self._vfd_stop()
        except Exception as exc:
            logger.error("Could not stop the pump for pause: %s", exc)
            return f"pump may still be running: {exc}"
        return f"pump stopped, setpoint {self._paused_flowrate} L/min held for resume"

    def resume(self) -> Optional[str]:
        """Restore the setpoint captured by pause() and restart the pump.

        set_flowrate()/start() return a bool rather than raising on
        failure, so their result is checked explicitly here — silently
        ignoring a False (as this used to) would report resume() as
        successful even when the confluence node never acknowledged the
        setpoint or the start command.

        If the pump was running via set_frequency_hz() rather than
        set_flowrate() when paused, there is no L/min setpoint to
        restore (get_flowrate() reads NaN in that case) — resume() just
        restarts at the VFD's already-set frequency rather than treating
        NaN as a real target (NaN is truthy in Python, so a naive `if
        target:` would call set_flowrate(nan) and send NaN to the drive).
        """
        target = getattr(self, "_paused_flowrate", None)
        try:
            if target and target == target and not self.set_flowrate(target):  # target==target: not NaN
                raise RuntimeError(f"set_flowrate({target}) was not acknowledged")
            if not self.start():
                raise RuntimeError("start() was not acknowledged")
        except Exception as exc:
            logger.error("Could not restart the pump on resume: %s", exc)
            return f"pump did not restart: {exc}"
        return None

    def stop(self) -> Optional[str]:
        """Stop the pump, leaving the valves at their current state.

        Returns:
            Note about any issues encountered, or None if successful.
        """
        try:
            self._vfd_stop()
            self.log_event("stop")
        except Exception as exc:
            logger.error("Could not stop the pump: %s", exc)
            return f"pump may still be running: {exc}"
        return None

    def estop(self) -> Optional[str]:
        """Pump off and both valves closed, each attempted independently.

        Every step is guarded separately: a failure closing qin must not
        prevent qaux from being closed, and neither must prevent the pump
        being stopped. Never raises.
        """
        problems = []
        for label, action in (
            ("stop the pump", lambda: self._vfd_stop()),
            ("close qin", lambda: setattr(self, "qin", False)),
            ("close qaux", lambda: setattr(self, "qaux", False)),
        ):
            try:
                action()
            except Exception as exc:
                logger.error("ESTOP: could not %s: %s", label, exc)
                problems.append(label)
        return f"could not: {', '.join(problems)}" if problems else None

    def clear_faults(self) -> bool:
        """Clear any latched VFD alarm state.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            reply = self._command_vfd("clear_faults")
        except RequestTimeout:
            logger.error("clear_faults(): no reply from confluence node")
            return False
        return bool(reply.get("ok", True))

    @property
    def qin(self) -> bool:
        """Whether the inlet solenoid valve is currently commanded open.

        This reflects the last value successfully written via the setter,
        not a live hardware readback.
        """
        return self._qin_state

    @qin.setter
    def qin(self, state: bool) -> None:
        """Open (True) or close (False) the inlet solenoid valve.

        Sends a set_io command to the shared ClearCore's valve axis
        (channel 0) and blocks for acknowledgement.

        Raises:
            RuntimeError: If not connected, if the confluence node doesn't
                acknowledge the command in time, or if it replies with
                `ok: False` (the cached state is only updated on a
                confirmed `ok`, so a rejected command can't leave qin
                looking open/closed when it wasn't).
        """
        self._require_connected()
        try:
            reply = self._command_valve("set_io", {"channel": 0, "state": state})
        except RequestTimeout as exc:
            raise RuntimeError(f"qin={state}: no reply from confluence node") from exc
        if not reply.get("ok", True):
            raise RuntimeError(f"qin={state}: confluence node rejected the command: {reply}")
        self._qin_state = state
        self.log_event("qin", state=state)

    @property
    def qaux(self) -> bool:
        """Whether the auxiliary solenoid valve is currently commanded open.

        This reflects the last value successfully written via the setter,
        not a live hardware readback.
        """
        return self._qaux_state

    @qaux.setter
    def qaux(self, state: bool) -> None:
        """Open (True) or close (False) the auxiliary solenoid valve.

        Sends a set_io command to the shared ClearCore's valve axis
        (channel 1) and blocks for acknowledgement.

        Raises:
            RuntimeError: If not connected, if the confluence node doesn't
                acknowledge the command in time, or if it replies with
                `ok: False` (the cached state is only updated on a
                confirmed `ok`, so a rejected command can't leave qaux
                looking open/closed when it wasn't).
        """
        self._require_connected()
        try:
            reply = self._command_valve("set_io", {"channel": 1, "state": state})
        except RequestTimeout as exc:
            raise RuntimeError(f"qaux={state}: no reply from confluence node") from exc
        if not reply.get("ok", True):
            raise RuntimeError(f"qaux={state}: confluence node rejected the command: {reply}")
        self._qaux_state = state
        self.log_event("qaux", state=state)

    def get_status(self) -> Dict[str, Any]:
        """Return connection state, flow setpoint, valve states, and raw VFD/valve status.

        Reads the latest cached messages on the VFD and valve status
        topics (non-blocking) rather than polling hardware synchronously.

        Returns:
            Dict with `is_connected`, `flowrate_lpm`, `qin_open`, and
            `qaux_open`; when connected, also includes `vfd_state` and
            `vfd_setpoint_hz`. `vfd_estop` is always None — see below.
        """
        status: Dict[str, Any] = {
            "is_connected": self._is_connected,
            "flowrate_lpm": self._current_flowrate,
            "qin_open": self._qin_state,
            "qaux_open": self._qaux_state,
        }
        if self._is_connected:
            if self._simulated:
                status["vfd_state"] = None
                status["vfd_estop"] = None
                status["vfd_setpoint_hz"] = float("nan")
            else:
                vfd_status = self._mqtt.get_latest(self._vfd_status_topic) or {}
                # Confluence's Fuji_VFD_funcs.py poll_status() publishes
                # "state" (not "state_message") and "freq setpoint" as a
                # display string like "42.00 Hz" (not a bare "setpoint"
                # float) — confirmed against the actual published dict, not
                # guessed. e_stop is computed internally on the driver
                # (self.e_stop) but never included in that published dict,
                # so there is currently no wire-level way to read it; always
                # None here rather than silently reporting a stale/wrong
                # value.
                status["vfd_state"] = vfd_status.get("state")
                status["vfd_estop"] = None
                status["vfd_setpoint_hz"] = _parse_hz(vfd_status.get("freq setpoint"))
        return status
