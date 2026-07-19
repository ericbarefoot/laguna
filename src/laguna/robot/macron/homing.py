"""Homing procedure for the Macron gantry using hardware capture latching.

Uses the Snap2Motion capture mechanism (AIC/CAT/CAP) rather than polling INB,
so the zeroing position is timestamped at interrupt level — not at poll interval.
This gives sub-millisecond positional accuracy at the limit switch event.

Sequence per axis:
  1. Disengage brake (Y and Z only)
  2. Configure capture source and polarity
  3. If limit switch is already tripped, jog away first
  4. Arm capture (AIC), begin slow jog toward negative limit
  5. Poll CAT until capture trips
  6. Controlled stop (BST), wait for move finished
  7. Zero at CAP (exact hardware-latched position), not at current position
  8. Move to standoff distance
  9. Leave brakes disengaged (caller decides when to re-engage)

Z homes first — it moves up before XY search, preventing the instrument
from crashing into the bed during lateral homing moves.
"""

from __future__ import annotations

import time
import logging
from dataclasses import dataclass, field
from typing import Optional

from .commands import MMCCommands, Axis, IOMap, X_AXIS, Y_AXIS, Z_AXIS, THETA_AXIS
from .connection import SnapMotionError

logger = logging.getLogger(__name__)


@dataclass
class AxisHomingConfig:
    """Per-axis homing parameters."""
    capture_source_index: int           # INB index wired to this axis's limit switch
    capture_trip_on_high: bool = True   # True = switch closes to +V; False = normally closed
    homing_direction: float = -1.0      # -1 = jog toward negative limit (usual); +1 if inverted


@dataclass
class HomingConfig:
    """Full homing configuration for all axes."""
    homing_speed: float = 10.0          # mm/s — slow enough to stop cleanly
    standoff_distance: float = 5.0      # mm to back off after zeroing
    poll_interval_s: float = 0.05       # 50 ms between CAT polls
    timeout_s: float = 60.0             # per-axis timeout
    backoff_timeout_s: float = 10.0     # timeout when backing away from a pre-tripped switch
    home_order: tuple[Axis, ...] = field(
        default_factory=lambda: (Z_AXIS, X_AXIS, Y_AXIS)
    )
    # Capture source config per axis. Must be set before homing.
    # Keys are Axis objects; values are AxisHomingConfig.
    axis_configs: dict[Axis, AxisHomingConfig] = field(default_factory=dict)

    def axis_config(self, axis: Axis) -> Optional[AxisHomingConfig]:
        return self.axis_configs.get(axis)


@dataclass
class HomingResult:
    success: bool
    axis_results: dict[str, float]  # axis name → final standoff position
    error: Optional[str] = None


class HomingProcedure:
    """Executes the homing sequence against the controller.

    Args:
        cmd:      Active MMCCommands instance (connection must already be open).
        config:   HomingConfig with capture sources and motion parameters.
        io_map:   IOMap for brake control. Defaults to standard pin mapping.
    """

    def __init__(
        self,
        cmd: MMCCommands,
        config: HomingConfig,
        io_map: Optional[IOMap] = None,
    ):
        self._cmd = cmd
        self._config = config
        self._io_map = io_map or IOMap()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def home_axis(self, axis: Axis) -> float:
        """Home a single axis. Returns the standoff position after homing.

        Raises SnapMotionError on timeout or connection failure.
        """
        cfg = self._config
        ax_cfg = cfg.axis_config(axis)

        logger.info("Homing axis %s", axis.name)

        self._disengage_brake_if_needed(axis)

        if ax_cfg is not None:
            self._cmd.set_capture_source(axis, ax_cfg.capture_source_index)
            self._cmd.set_capture_trip(axis, ax_cfg.capture_trip_on_high)

        self._backoff_if_already_tripped(axis)

        self._cmd.arm_capture(axis)
        direction = ax_cfg.homing_direction if ax_cfg is not None else -1.0
        self._cmd.jog(axis, direction * cfg.homing_speed)

        self._wait_for_capture(axis)

        # Controlled stop — gives a clean decel instead of a hard cut
        self._cmd.begin_stop(axis)
        self._wait_for_move_finished(axis, cfg.backoff_timeout_s)

        # Zero at the hardware-latched position (CAP), not at the current
        # (post-decel) position. A controlled stop travels some distance
        # past the trip point before actually stopping, so setting ACP to a
        # flat 0.0 here would zero at wherever we happen to have stopped —
        # not at the trip point. Instead, compute how far we've travelled
        # past the trip point (current - trip) and zero relative to that,
        # so the trip point itself lands exactly on 0 in the new frame.
        trip_pos = self._cmd.get_capture_position(axis)
        current_pos = self._cmd.get_actual_position(axis)
        self._cmd.set_actual_position(axis, current_pos - trip_pos)
        logger.info(
            "Axis %s: limit switch tripped at %.4f (hardware latch), zeroed relative to trip point",
            axis.name, trip_pos,
        )

        # Move to standoff
        self._cmd.move_to(axis, cfg.standoff_distance)
        final_pos = self._cmd.get_actual_position(axis)
        logger.info("Axis %s homed. Standoff position: %.4f", axis.name, final_pos)
        return final_pos

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
        if axis not in (Y_AXIS, Z_AXIS):
            return
        self._cmd.disengage_brake(axis, self._io_map)
        # Wait up to 0.5 s for brake feedback to confirm release
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if self._cmd.brake_is_disengaged(axis, self._io_map):
                return
            time.sleep(0.05)
        logger.warning(
            "Axis %s: brake release feedback did not confirm within 0.5 s — continuing anyway",
            axis.name,
        )

    def _backoff_if_already_tripped(self, axis: Axis) -> None:
        """If the limit switch is already active, jog away before starting homing."""
        self._cmd.arm_capture(axis)
        if not self._cmd.get_capture_bit(axis):
            return

        logger.info(
            "Axis %s: limit switch already tripped at start, backing off %.1f mm",
            axis.name, self._config.standoff_distance * 2,
        )
        # Back off 2× standoff to ensure we clear the switch
        backoff = abs(self._config.standoff_distance) * 2
        self._cmd.begin_move_by(axis, backoff)
        self._wait_for_move_finished(axis, self._config.backoff_timeout_s)

    def _wait_for_capture(self, axis: Axis) -> None:
        cfg = self._config
        deadline = time.monotonic() + cfg.timeout_s
        while not self._cmd.capture_has_tripped(axis):
            if time.monotonic() > deadline:
                self._cmd.abort(axis)
                raise SnapMotionError(
                    0,
                    f"Homing timeout on axis {axis.name} after {cfg.timeout_s:.0f} s "
                    f"— limit switch not reached. Check wiring and capture source index.",
                )
            time.sleep(cfg.poll_interval_s)

    def _wait_for_move_finished(self, axis: Axis, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while not self._cmd.move_is_finished(axis):
            if time.monotonic() > deadline:
                self._cmd.abort(axis)
                logger.warning(
                    "Axis %s: move-finished timeout after %.1f s, aborted",
                    axis.name, timeout_s,
                )
                return
            time.sleep(self._config.poll_interval_s)
