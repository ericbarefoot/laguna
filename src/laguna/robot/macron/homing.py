"""Homing procedure for the Macron gantry, by software-polling the home/limit switch.

Originally designed around the Snap2Motion hardware capture-latch
(SCS/SCT/AIC/CAT/CAP) for sub-millisecond, interrupt-timestamped zeroing.
Abandoned 2026-08-25: the vendor's own ASCII reference (SCS's parameter
table) shows the capture source can only be a front-encoder channel (A/B/
Index) or an "Option N Index" pulse from an expansion card — never an
arbitrary native INB digital input, which is how every home/limit switch
on this machine is wired (see IOMap in commands.py). Confirmed on
hardware: axis A2 (Y) rejects SCS with every parameter value, including
ones the vendor's own table lists as valid, with an escape code (14) that
isn't in the vendor's documented escape table at all — almost certainly
custom to this machine's compiled DSM program, not something we can
reason about from outside it. Pending word from the vendor. Until then,
homing here just jogs and polls read_home_switch()/read_limit_switch() in
software — the zeroing position is only as accurate as poll_interval_s
allows, not hardware-latched.

Sequence per axis:
  1. Disengage brake (Y and Z only)
  2. If the switch is already tripped, jog away first
  3. Begin slow jog toward the switch
  4. Poll the switch (INB) until it trips
  5. Controlled stop (BST), wait for move finished
  6. Zero relative to the position read at the moment of detection
  7. Move to standoff distance
  8. Leave brakes disengaged (caller decides when to re-engage)

Z homes first — it moves up before XY search, preventing the instrument
from crashing into the bed during lateral homing moves.
"""

from __future__ import annotations

import time
import logging
from dataclasses import dataclass, field
from typing import Optional

from .commands import (
    MMCCommands, Axis, IOMap, X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS,
    poll_until_move_finished, predicted_move_s,
)
from .connection import SnapMotionError

logger = logging.getLogger(__name__)


@dataclass
class AxisHomingConfig:
    """Per-axis homing parameters."""
    input_index: int                    # INB index of this axis's home/limit switch
    trip_on_high: bool = True           # True = switch reads HIGH when triggered; False = LOW
    homing_direction: float = -1.0      # -1 = jog toward negative limit (usual); +1 if inverted


@dataclass
class HomingConfig:
    """Full homing configuration for all axes."""
    homing_speed: float = 10.0          # mm/s — slow enough to stop cleanly
    standoff_distance: float = 5.0      # mm to back off after zeroing
    poll_interval_s: float = 0.05       # 50 ms between switch polls
    timeout_s: float = 60.0             # per-axis timeout
    backoff_timeout_s: float = 10.0     # timeout when backing away from a pre-tripped switch
    home_order: tuple[Axis, ...] = field(
        default_factory=lambda: (Z_AXIS, X_AXIS, Y_AXIS)
    )
    # Per-axis switch config. Required for every axis in home_order —
    # home_axis() raises if an axis has none, since there is no other way
    # to know which INB bit to poll. Keys are Axis objects; values are
    # AxisHomingConfig.
    axis_configs: dict[Axis, AxisHomingConfig] = field(default_factory=dict)

    def axis_config(self, axis: Axis) -> Optional[AxisHomingConfig]:
        """Get the homing config for a specific axis.

        Args:
            axis: The axis to look up.

        Returns:
            The AxisHomingConfig for this axis, or None if not configured.
        """
        return self.axis_configs.get(axis)


@dataclass
class HomingResult:
    """Result of a homing sequence execution."""

    success: bool
    axis_results: dict[str, float]  # axis name → final standoff position
    error: Optional[str] = None


class HomingProcedure:
    """Executes the homing sequence against the controller.

    Args:
        cmd:      Active MMCCommands instance (connection must already be open).
        config:   HomingConfig with per-axis switches and motion parameters.
        io_map:   IOMap for brake control. Defaults to standard pin mapping.
    """

    def __init__(
        self,
        cmd: MMCCommands,
        config: HomingConfig,
        io_map: Optional[IOMap] = None,
    ):
        """Initialize the homing procedure.

        Args:
            cmd: Active MMCCommands instance (connection must already be open).
            config: HomingConfig with per-axis switches and motion parameters.
            io_map: IOMap for brake control. Defaults to standard pin mapping.
        """
        self._cmd = cmd
        self._config = config
        self._io_map = io_map or IOMap()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def home_axis(self, axis: Axis) -> float:
        """Home a single axis. Returns the standoff position after homing.

        Raises:
            ValueError: If no AxisHomingConfig is set for this axis — there
                is no other way to know which INB bit to poll.
            SnapMotionError: On timeout or connection failure.
        """
        cfg = self._config
        ax_cfg = cfg.axis_config(axis)
        if ax_cfg is None:
            raise ValueError(
                f"No AxisHomingConfig for axis {axis.name} — set input_index "
                "(and trip_on_high) in HomingConfig.axis_configs before homing this axis."
            )

        logger.info("Homing axis %s", axis.name)

        is_tripped = lambda: self._switch_is_tripped(axis, ax_cfg)

        self._disengage_brake_if_needed(axis)

        self._backoff_if_already_tripped(axis, is_tripped, ax_cfg.homing_direction)

        self._cmd.jog(axis, ax_cfg.homing_direction * cfg.homing_speed)

        trip_pos = self._wait_for_switch_trip(axis, is_tripped)

        # Controlled stop — gives a clean decel instead of a hard cut
        self._cmd.begin_stop(axis)
        self._wait_for_move_finished(axis, cfg.backoff_timeout_s)

        # Zero relative to trip_pos (the position read the instant polling
        # detected the switch), not the current (post-decel) position. A
        # controlled stop travels some distance past the trip point before
        # actually stopping, so setting ACP to a flat 0.0 here would zero
        # at wherever we happen to have stopped — not at the trip point.
        # Instead, compute how far we've travelled past the trip point
        # (current - trip) and zero relative to that, so the trip point
        # itself lands exactly on 0 in the new frame. Note trip_pos is only
        # as accurate as poll_interval_s + one round trip, not a hardware
        # timestamp — see this module's docstring.
        current_pos = self._cmd.get_actual_position(axis)
        self._cmd.set_actual_position(axis, current_pos - trip_pos)
        logger.info(
            "Axis %s: switch tripped at %.4f (software-polled), zeroed relative to trip point",
            axis.name, trip_pos,
        )

        # Move to standoff. Hardware-driven change, 2026-07/08:
        # non-blocking begin_move_to + poll, like the backoff move above —
        # move_to() (blocking MVT) is banned; see commands.py. Distance is
        # measured from the just-rezeroed frame, in which the trip point is
        # 0 and we're sitting `current_pos - trip_pos` past it.
        self._cmd.begin_move_to(axis, cfg.standoff_distance)
        self._wait_for_move_finished(
            axis, cfg.backoff_timeout_s,
            predicted_s=predicted_move_s(
                abs(cfg.standoff_distance - (current_pos - trip_pos)), cfg.homing_speed
            ),
        )
        final_pos = self._cmd.get_actual_position(axis)
        logger.info("Axis %s homed. Standoff position: %.4f", axis.name, final_pos)
        return final_pos

    def locate_limit_switch(self, axis: Axis) -> float:
        """Jog toward and record this axis's limit switch position.

        Unlike home_axis(), this does not re-zero the axis — it reports
        the limit switch's position in the current (already-homed)
        coordinate frame, then backs off by standoff_distance so the axis
        isn't left resting against the hard stop. The returned value is
        the trip position itself, not the post-backoff resting position.

        Reuses this axis's AxisHomingConfig: jogs in the opposite
        direction from homing_direction (the limit switch is assumed to
        sit at the far travel extreme from the home switch) and assumes
        the same trip polarity (trip_on_high) as the home switch — both
        unconfirmed assumptions until verified on hardware.

        Raises:
            ValueError: If no AxisHomingConfig is set for this axis.
            SnapMotionError: On timeout or connection failure.
        """
        cfg = self._config
        ax_cfg = cfg.axis_config(axis)
        if ax_cfg is None:
            raise ValueError(
                f"No AxisHomingConfig for axis {axis.name} — set input_index "
                "(and trip_on_high) in HomingConfig.axis_configs before locating "
                "this axis's limit switch."
            )

        logger.info("Locating limit switch on axis %s", axis.name)

        direction = -ax_cfg.homing_direction
        is_tripped = lambda: self._limit_switch_is_tripped(axis, ax_cfg.trip_on_high)

        self._disengage_brake_if_needed(axis)

        self._backoff_if_already_tripped(axis, is_tripped, direction)

        self._cmd.jog(axis, direction * cfg.homing_speed)

        trip_pos = self._wait_for_switch_trip(axis, is_tripped)

        # Controlled stop — gives a clean decel instead of a hard cut
        self._cmd.begin_stop(axis)
        self._wait_for_move_finished(axis, cfg.backoff_timeout_s)
        logger.info("Axis %s: limit switch located at %.4f", axis.name, trip_pos)

        # Back off standoff_distance the way we came, so the axis isn't
        # left resting against the hard stop. Relative move, unlike
        # home_axis()'s absolute one — there is no rezeroed frame here to
        # measure an absolute target from.
        self._cmd.begin_move_by(axis, -direction * cfg.standoff_distance)
        self._wait_for_move_finished(
            axis, cfg.backoff_timeout_s,
            predicted_s=predicted_move_s(cfg.standoff_distance, cfg.homing_speed),
        )
        return trip_pos

    def home_all(self) -> HomingResult:
        """Home all axes in the configured order (default: Z, X, Y).

        Stops and returns a failure result on the first axis that fails rather
        than leaving the gantry in a partially homed state.
        """
        results: dict[str, float] = {}
        for axis in self._config.home_order:
            try:
                pos = self.home_axis(axis)
                results[axis.name] = pos
            except Exception as exc:
                logger.error("Homing failed on axis %s: %s", axis.name, exc)
                return HomingResult(success=False, axis_results=results, error=str(exc))
        return HomingResult(success=True, axis_results=results)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _disengage_brake_if_needed(self, axis: Axis) -> None:
        """Disengage brake for Y/Z axes before homing motion.

        Waits up to 0.5s for brake feedback to confirm disengagement, where
        that feedback is reachable at all. Z's brake status input lives on
        the responder node's own input bank and cannot be read via ASCII
        (see IOMap in commands.py) — for axes in that situation, this
        trusts the just-issued SOB command instead of trying to read it
        back, the same way engage_brake()/disengage_brake() are trusted
        everywhere else they're called without a confirm step. Logs a
        warning (not the same as raising) if a *readable* status input
        does not confirm within 0.5s, but continues anyway.

        Args:
            axis: Target axis (no-op for axes without brakes).
        """
        if axis not in (Y_AXIS, Z_AXIS):
            return
        self._cmd.disengage_brake(axis, self._io_map)
        # Wait up to 0.5 s for brake feedback to confirm release
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            try:
                if self._cmd.brake_is_disengaged(axis, self._io_map):
                    return
            except NotImplementedError:
                logger.info(
                    "Axis %s: brake status input is not reachable via ASCII — "
                    "trusting the SOB command just issued instead of a live readback",
                    axis.name,
                )
                return
            time.sleep(0.05)
        logger.warning(
            "Axis %s: brake release feedback did not confirm within 0.5 s — continuing anyway",
            axis.name,
        )

    def _switch_is_tripped(self, axis: Axis, ax_cfg: AxisHomingConfig) -> bool:
        """Read this axis's configured (home or limit) switch and check its trip polarity."""
        state = self._cmd.read_input_bit(ax_cfg.input_index)
        return state if ax_cfg.trip_on_high else not state

    def _limit_switch_is_tripped(self, axis: Axis, trip_on_high: bool) -> bool:
        """Read this axis's limit switch (IOMap, not AxisHomingConfig) and check trip polarity."""
        state = self._cmd.read_limit_switch(axis, self._io_map)
        return state if trip_on_high else not state

    def _backoff_if_already_tripped(self, axis: Axis, is_tripped, direction: float) -> None:
        """If the switch is already tripped, jog away before starting the search.

        Backs off 2× standoff distance, opposite the direction about to be
        searched, to ensure the switch is fully cleared.

        Args:
            axis: Target axis.
            is_tripped: Callable returning whether the target switch is
                currently tripped.
            direction: The direction the caller is about to jog in
                (search direction) — backoff moves the opposite way.
        """
        if not is_tripped():
            return

        backoff = -direction * abs(self._config.standoff_distance) * 2
        logger.info(
            "Axis %s: switch already tripped at start, backing off %.1f mm",
            axis.name, backoff,
        )
        self._cmd.begin_move_by(axis, backoff)
        self._wait_for_move_finished(
            axis, self._config.backoff_timeout_s,
            predicted_s=predicted_move_s(backoff, self._config.homing_speed),
        )

    def _wait_for_switch_trip(self, axis: Axis, is_tripped) -> float:
        """Poll `is_tripped` until it trips. Returns the axis position at detection.

        Aborts axis motion if timeout elapses before the switch trips. The
        returned position is only as accurate as poll_interval_s plus one
        wire round trip — see this module's docstring for why there's no
        hardware timestamp here.

        Args:
            axis: Target axis.
            is_tripped: Callable returning whether the target switch is
                currently tripped.

        Raises:
            SnapMotionError: If the switch does not trip within the
                configured timeout.
        """
        cfg = self._config
        deadline = time.monotonic() + cfg.timeout_s
        while not is_tripped():
            if time.monotonic() > deadline:
                self._cmd.abort(axis)
                raise SnapMotionError(
                    0,
                    f"Homing timeout on axis {axis.name} after {cfg.timeout_s:.0f} s "
                    f"— switch not reached. Check wiring and input_index.",
                )
            time.sleep(cfg.poll_interval_s)
        return self._cmd.get_actual_position(axis)

    def _wait_for_move_finished(self, axis: Axis, timeout_s: float, predicted_s: float = 0.0) -> None:
        """Wait for a single-axis move to finish, warning (not raising) on timeout.

        polls
        sparsely via commands.poll_until_move_finished rather than at
        config.poll_interval_s — see that function's module note. Callers
        that know the move's distance/speed pass `predicted_s` so most of
        the wait costs no wire traffic at all; the homing-jog decel wait
        (after BST) leaves it at 0.0 since decel time isn't known here.
        """
        if not poll_until_move_finished(
            lambda: self._cmd.move_is_finished(axis), predicted_s=predicted_s, timeout_s=timeout_s
        ):
            self._cmd.abort(axis)
            logger.warning(
                "Axis %s: move-finished timeout after %.1f s, aborted",
                axis.name, timeout_s,
            )
