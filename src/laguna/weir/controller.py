"""Weir (tailgate) elevation control subsystem."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Optional
import logging

from ..mqtt import MqttSubscriber, RequestTimeout, request
from ..subsystem_logging import SubsystemLogging

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)


class WeirController(ABC):
    """Abstract interface for tailgate weir elevation control.

    Concrete implementations drive whatever motor/actuator raises or lowers
    the flume's tailgate weir, exposing elevation and velocity in physical
    units (mm, mm/s) regardless of the underlying hardware protocol.
    """

    subsystem_name = "weir"

    @abstractmethod
    def connect(self) -> bool:
        """Open the hardware connection.

        Returns:
            True if the connection was established, False otherwise.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close the hardware connection and release any held resources."""
        ...

    @abstractmethod
    def set_elevation(self, mm: float) -> None:
        """Redefine the controller's notion of the current elevation as `mm`.

        This recalibrates the position reference without commanding any
        motion — it does not move the weir. Use it to re-reference the
        controller after the weir has been repositioned by other means
        (e.g. manually, or as the final step of a homing routine). To
        actually move the weir, use go_to_elevation().
        """
        ...

    @abstractmethod
    def go_to_elevation(self, mm: float) -> bool:
        """Command the weir to move to an absolute target elevation in mm.

        Returns:
            True if the move command was accepted by the hardware.
        """
        ...

    @abstractmethod
    def get_elevation(self) -> float:
        """Return current weir elevation in mm."""
        ...

    @abstractmethod
    def set_velocity(self, mm_per_sec: float) -> None:
        """Set motor move speed in mm/s (applies to the next go_to_elevation call)."""
        ...

    @abstractmethod
    def get_velocity(self) -> float:
        """Return current velocity setpoint in mm/s."""
        ...

    @abstractmethod
    def enable(self) -> None:
        """Enable motor drive."""
        ...

    @abstractmethod
    def disable(self) -> None:
        """Disable motor drive (allows manual repositioning)."""
        ...

    @abstractmethod
    def wait_for_move(self, timeout: float = 30.0) -> None:
        """Block until the current move completes or timeout elapses."""
        ...

    @abstractmethod
    def clear_faults(self) -> bool:
        """Clear any latched motor fault state.

        Returns:
            True if faults were cleared successfully.
        """
        ...

    @abstractmethod
    def home(self) -> bool:
        """Run the homing routine to establish a reference position.

        Returns:
            True if homing completed successfully, False on timeout or fault.
        """
        ...

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # The weir holds its elevation mechanically, so every tier reduces to the
    # same action: halt any in-progress move. All four verbs must return a
    # note about anything a human needs to know, or None, and never raise.

    @abstractmethod
    def pause(self) -> Optional[str]:
        """Halt any in-progress elevation move."""
        ...

    @abstractmethod
    def resume(self) -> Optional[str]:
        """Resume motion (no-op: elevation is mechanically held).

        Returns:
            None.
        """
        ...

    @abstractmethod
    def stop(self) -> Optional[str]:
        """Halt the move. Same action as pause: the weir has one stop."""
        ...

    @abstractmethod
    def estop(self) -> Optional[str]:
        """Halt the move. Same action as pause: the weir has one stop."""
        ...

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Return a snapshot of controller/motor status.

        Returns:
            Dictionary of status fields. Contents are backend-specific, but
            always include at least `is_connected` and `elevation_mm`.
        """
        ...


class SaflWeirController(WeirController, SubsystemLogging):
    """Weir controller backed by a Teknic ClearCore stepper motor, via MQTT.

    The ClearCore is wired to the confluence node on red.lab, which owns
    the USB serial connection and exposes it over MQTT — this class no
    longer talks to serial itself. Status (elevation_mm, ...) streams in
    on a status topic; motion/config commands (go_to_elevation, home, ...)
    are sent as request/reply envelopes (see laguna.mqtt.request_reply) so
    a return value keeps meaning "hardware acknowledged," not just
    "message published." The ClearCore firmware handles all unit
    conversion internally (configured on its SD card), so positions and
    velocities are passed through in mm and mm/s respectively with no
    scaling applied here.
    """

    def __init__(self, config: Dict[str, Any], mqtt_subscriber: MqttSubscriber):
        """Build the controller from a config dict; does not open a connection.

        Args:
            config: Subsystem configuration dictionary (see
                Config._get_defaults()'s 'weir' section for the expected
                shape). Recognized keys:
                - topic_status: MQTT topic the confluence Teknic_ClearCore
                  interface publishes weir-gate status on.
                - topic_commands / topic_replies: Command/reply topics for
                  the weir-gate axis (see laguna.mqtt.request_reply).
                - command_timeout_s: Seconds to wait for a command reply
                  before treating it as failed (default 5.0).
                - home_offset_mm: Position value sent to the controller's
                  position register once home() finds the limit switch
                  (default 0.0).
                - log_level / event_log_verbosity: see laguna.subsystem_logging
                  (both default 'INFO').
                - simulated: skip MQTT entirely — every command "succeeds"
                  immediately and every reading is NaN (default False).
            mqtt_subscriber: MqttSubscriber for MQTT operations.
        """
        self._status_topic = config.get("topic_status", "SAFL Confluence Node 1/weir")
        self._commands_topic = config.get(
            "topic_commands", "SAFL Confluence Node 1/weir/commands"
        )
        self._replies_topic = config.get(
            "topic_replies", "SAFL Confluence Node 1/weir/replies"
        )
        self._command_timeout_s = config.get("command_timeout_s", 5.0)
        self._home_offset_mm = config.get("home_offset_mm", 0.0)
        self.log_level = config.get("log_level", "INFO")
        self.event_log_verbosity = config.get("event_log_verbosity", "INFO")
        self._simulated = config.get("simulated", False)
        self._mqtt = mqtt_subscriber
        self._is_connected = False
        self._velocity_mm_per_sec = float("nan")
        self._target_mm: Optional[float] = None
        self._move_start_mm: Optional[float] = None

    @classmethod
    def from_config(cls, config: "Config") -> "SaflWeirController":
        """Build from the lab's Config (its 'weir:' section and shared 'mqtt:' section).

        Derives default topics from mqtt.node_name — the weir section
        itself only needs topic_* keys to override those defaults.
        """
        mqtt_config = config.get("mqtt")
        node_name = mqtt_config.get("node_name", "SAFL Confluence Node 1")
        section = dict(config.get("weir"))
        section.setdefault("topic_status", f"{node_name}/weir")
        section.setdefault("topic_commands", f"{node_name}/weir/commands")
        section.setdefault("topic_replies", f"{node_name}/weir/replies")
        mqtt_subscriber = MqttSubscriber(mqtt_config)
        return cls(section, mqtt_subscriber)

    def connect(self) -> bool:
        """Connect the underlying MQTT subscriber and subscribe to weir topics.

        Blocks (up to a few seconds) for the broker handshake to actually
        complete before returning — connect() alone only starts it
        asynchronously (see MqttSubscriber.wait_until_connected()). Without
        this, a command issued immediately after connect() can race the
        handshake and fail with "MqttSubscriber is not connected" even
        though this method already returned True.

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
        self._mqtt.subscribe(self._status_topic)
        self._mqtt.subscribe(self._replies_topic)
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

    def _command(
        self, command: str, args: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """Send a command to the confluence Teknic_ClearCore interface and block for its reply.

        Raises:
            RequestTimeout: If no reply arrives within the timeout.
        """
        if self._simulated:
            return {"ok": True, "accepted": True}
        return request(
            self._mqtt,
            self._commands_topic,
            self._replies_topic,
            command,
            args,
            timeout=timeout if timeout is not None else self._command_timeout_s,
        )

    def set_elevation(self, mm: float) -> None:
        """Redefine the ClearCore's internal position register as `mm`.

        This only recalibrates what the firmware believes its current
        position is — it commands no motion. To actually move the weir,
        use go_to_elevation().

        Raises:
            RuntimeError: If not connected.
            RequestTimeout: If the confluence node doesn't reply in time.
        """
        self._require_connected()
        self._command("set_position", {"mm": mm})

    def go_to_elevation(self, mm: float) -> bool:
        """Command the weir to move to an absolute elevation of `mm`.

        Blocks (up to `command_timeout_s`) for the confluence node's
        acknowledgement that the move was accepted — it does not wait for
        the move to finish. Call wait_for_move() to block until arrival.

        Returns:
            True if the move command was accepted by the hardware; False
            if the confluence node didn't reply in time.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        start_mm = self.get_elevation()
        try:
            reply = self._command("move_absolute", {"mm": mm})
        except RequestTimeout:
            logger.error("go_to_elevation(%.2f): no reply from confluence node", mm)
            return False
        accepted = bool(reply.get("accepted", False))
        if accepted:
            self._target_mm = mm
            self._move_start_mm = start_mm
            if not self._simulated:
                # Discard any status message still buffered from before this
                # move was commanded — belt-and-suspenders against
                # wait_for_move() reading a position left over from an
                # unrelated older command.
                self._mqtt.drain(self._status_topic)
        self.log_event("go_to_elevation", target_mm=f"{mm:.2f}")
        return accepted

    def get_elevation(self) -> float:
        """Return the most recently published elevation in mm.

        Unlike the old serial version, this does not poll hardware — it
        reads the latest message on the status topic.

        Returns:
            NaN if no status message has arrived yet.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        if self._simulated:
            return float("nan")
        status = self._mqtt.get_latest(self._status_topic)
        if status is None:
            return float("nan")
        return float(status.get("elevation_mm", float("nan")))

    def set_velocity(self, mm_per_sec: float) -> None:
        """Set the move speed used by the *next* go_to_elevation() call.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            self._command("set_velocity", {"mm_per_sec": mm_per_sec})
        except RequestTimeout:
            logger.warning("set_velocity(%.2f): no reply from confluence node", mm_per_sec)
        self._velocity_mm_per_sec = mm_per_sec

    def get_velocity(self) -> float:
        """Return the most recently published velocity setpoint in mm/s.

        Reads the status topic's `velocity_setpoint` field (confluence's
        poll_weir_status() sources this from the ClearCore's real
        VelSetPoint register — see Teknic_ClearCore_funcs.py), same
        live-read pattern as get_elevation(). Falls back to the locally
        cached value set_velocity() last commanded if no status message
        has arrived yet (e.g. right after connect()).

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        if self._simulated:
            return float("nan")
        status = self._mqtt.get_latest(self._status_topic)
        if status is None or status.get("velocity_setpoint") is None:
            return self._velocity_mm_per_sec
        return float(status["velocity_setpoint"])

    def enable(self) -> None:
        """Enable the motor drive so it can accept move commands.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            self._command("enable")
        except RequestTimeout:
            logger.warning("enable(): no reply from confluence node")

    def disable(self) -> None:
        """Disable the motor drive, allowing the weir to be repositioned by hand.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            self._command("disable")
        except RequestTimeout:
            logger.warning("disable(): no reply from confluence node")

    #: How close elevation_mm must get to the commanded target for
    #: wait_for_move() to consider the move finished.
    POSITION_TOLERANCE_MM = 0.5

    #: Steady-state polling interval once past the predicted-finish sleep.
    #: Matches confluence's own status-publish interval (0.5s) — polling
    #: faster than the source republishes just re-reads the same message.
    #: Same value/rationale as SPARSE_POLL_INTERVAL_S in
    #: robot.macron.commands (see that module's "Move-completion polling"
    #: note): tight polling has been observed to destabilize motion
    #: controllers on this project's hardware, so sparse-by-default is the
    #: house pattern, not just an optimization here.
    SPARSE_POLL_INTERVAL_S = 0.5

    #: Fraction of the move's predicted duration to sleep through before
    #: the first status check. Below 1.0 so a slightly-optimistic
    #: prediction (accel/decel isn't modeled) still lands the first poll
    #: before completion rather than long after it.
    PREDICTED_SLEEP_FRACTION = 0.85

    def wait_for_move(self, timeout: float = 30.0) -> None:
        """Block until elevation_mm reaches the last commanded target, or timeout elapses.

        Polls the status topic's `elevation_mm` field and compares it
        against the target passed to the most recent go_to_elevation()
        call, rather than trusting the status topic's `is_moving` flag.
        `is_moving` reflects the ClearCore's own belief about its state,
        which can lag the real motor by up to one status-publish interval
        in either direction — e.g. a fresh status sample taken in the brief
        window between "move accepted" and "ClearCore control loop actually
        starts stepping" can genuinely (not staleness — this is a fresh
        message) read `is_moving: False` before real motion has begun,
        which would make an is_moving-based wait return instantly. Position
        is ground truth: it can only read "arrived" once the gate actually
        has, and fails safe (times out rather than falsely succeeding) if
        the gate stalls short of the target.

        Sleeps through PREDICTED_SLEEP_FRACTION of the move's predicted
        duration (distance / the velocity set_velocity() last commanded)
        before the first status check, then polls every
        SPARSE_POLL_INTERVAL_S — same sparse-by-default shape as the
        gantry's poll_until_move_finished(), so this doesn't hammer the
        MQTT status topic for the whole move when the finish time is
        largely predictable.

        Args:
            timeout: Maximum seconds to wait before giving up (default 30).

        Raises:
            RuntimeError: If not connected, or if go_to_elevation() has
                never been called (there is no target to wait for).
        """
        self._require_connected()
        if self._simulated:
            return
        if self._target_mm is None:
            raise RuntimeError("wait_for_move() called before any go_to_elevation()")

        predicted_s = 0.0
        start_mm = self._move_start_mm
        velocity = self._velocity_mm_per_sec
        if start_mm == start_mm and velocity == velocity and velocity > 0:  # neither is NaN
            predicted_s = abs(self._target_mm - start_mm) / velocity

        deadline = time.monotonic() + timeout
        initial_sleep = max(0.0, min(predicted_s * self.PREDICTED_SLEEP_FRACTION, timeout))
        if initial_sleep > 0:
            time.sleep(initial_sleep)

        while time.monotonic() < deadline:
            status = self._mqtt.get_latest(self._status_topic)
            if status is not None:
                elevation = status.get("elevation_mm")
                if elevation is not None and abs(elevation - self._target_mm) <= self.POSITION_TOLERANCE_MM:
                    return
            time.sleep(self.SPARSE_POLL_INTERVAL_S)
        logger.warning("wait_for_move: timed out after %.1fs", timeout)

    def clear_faults(self) -> bool:
        """Send a clear-faults command to the confluence node.

        Returns:
            True if the confluence node acknowledged the command.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            reply = self._command("clear_faults")
        except RequestTimeout:
            logger.error("clear_faults(): no reply from confluence node")
            return False
        return bool(reply.get("ok", True))

    def home(self) -> bool:
        """Command the confluence node to run the homing routine.

        Uses a longer timeout than other commands since homing physically
        jogs to a limit switch before replying.

        Returns:
            True if homing completed successfully, False on timeout or fault.

        Raises:
            RuntimeError: If not connected.
        """
        self._require_connected()
        try:
            reply = self._command(
                "home", {"home_offset_mm": self._home_offset_mm},
                timeout=max(30.0, self._command_timeout_s),
            )
        except RequestTimeout:
            logger.error("home(): no reply from confluence node")
            return False
        return bool(reply.get("ok", False))

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------

    def _halt(self) -> Optional[str]:
        """Halt any in-progress move.

        Returns:
            Note about any issues encountered, or None if successful.
        """
        try:
            self._require_connected()
            self._command("stop")
        except Exception as exc:
            logger.error("Could not halt the weir: %s", exc)
            return f"weir may still be moving: {exc}"
        return None

    def pause(self) -> Optional[str]:
        """Halt any in-progress elevation move."""
        return self._halt()

    def resume(self) -> Optional[str]:
        """Resume motion (no-op: elevation is mechanically held).

        Returns:
            None.
        """
        return None

    def stop(self) -> Optional[str]:
        """Halt the move. Same action as pause: the weir has one stop."""
        return self._halt()

    def estop(self) -> Optional[str]:
        """Halt the move. Same action as pause: the weir has one stop."""
        return self._halt()

    def get_status(self) -> Dict[str, Any]:
        """Return connection state, current elevation, and raw status.

        Returns:
            Dict with `is_connected` and `elevation_mm` (None when not
            connected); when connected, also includes the full raw status
            dict from the latest status-topic message under the `motor` key.
        """
        if not self._is_connected:
            return {"is_connected": False, "elevation_mm": None}
        if self._simulated:
            return {"is_connected": True, "elevation_mm": float("nan"), "motor": {}}
        status = self._mqtt.get_latest(self._status_topic) or {}
        return {
            "is_connected": self._is_connected,
            "elevation_mm": status.get("elevation_mm"),
            "motor": status,
        }
