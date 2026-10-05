"""Tests for the software-polled homing procedure.

All scripted against FakeSnapConnection — no hardware.
"""

import time

import pytest

from laguna.robot.macron.commands import IOMap, MMCCommands, X_AXIS, Y_AXIS, Z_AXIS
from laguna.robot.macron.connection import SnapMotionError
from laguna.robot.macron.homing import AxisHomingConfig, HomingConfig, HomingProcedure
from tests.macron_fixtures import FakeSnapConnection


def _trip_after(n, before="0", after="1"):
    """Callable response: returns `before` for the first n calls, then `after`."""
    state = {"calls": 0}

    def _resp(cmd):
        state["calls"] += 1
        return after if state["calls"] > n else before

    return _resp


class TestHomeAxisHappyPath:
    def _make_procedure(self, responses, **config_overrides):
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0,
            standoff_distance=5.0,
            poll_interval_s=0.001,
            default_timeout_s=1.0,
            backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},
            **config_overrides,
        )
        return HomingProcedure(cmd, config), conn

    def test_homes_x_axis_and_zeros_relative_to_trip_point(self):
        # current(-25.1) - trip(-24.5) == -0.6 (formatted to 6 sig figs by MMCCommands)
        zero_cmd = f"A1 ACP {(-25.1) - (-24.5):.6g}"
        responses = {
            "INB 1": _trip_after(2),  # not tripped at start, trips on the 3rd poll
            "A1 JOG -10": "-10",
            "A1 BST": "0",
            # Same MIF response is polled twice: once for the post-BST
            # controlled-stop wait, once for the standoff move below (both
            # non-blocking BMT/MIF now — move_to() is banned, see
            # commands.py). By the second poll the counter is already past
            # threshold, so it reports finished on the first call.
            "A1 MIF": _trip_after(1),
            # First ACP-read call: at the moment INB 1 tripped (-24.500).
            # Second: after BST decel (-25.100). Third: final standoff
            # readout (5.000).
            "A1 ACP": _make_sequence(["-24.500", "-25.100", "5.000"]),
            zero_cmd: str((-25.1) - (-24.5)),
            "A1 BMT 5": "0",
        }
        proc, conn = self._make_procedure(responses)
        final_pos = proc.home_axis(X_AXIS)
        assert final_pos == 5.0
        assert zero_cmd in conn.sent  # zeroed relative to trip point, not a flat 0

    def test_backs_off_when_already_tripped_at_start(self):
        responses = {
            "INB 1": "1",  # already tripped
            "A1 MIF": _trip_after(0),  # move-is-finished immediately true after backoff
            "A1 BMB 10": "0",  # backoff = 2 * standoff_distance(5) = 10
            "A1 JOG -10": "-10",
            "A1 BST": "0",
            "A1 ACP": "0",
            "A1 ACP 0": "0",
            "A1 BMT 5": "0",
        }
        proc, conn = self._make_procedure(responses)
        proc.home_axis(X_AXIS)
        assert "A1 BMB 10" in conn.sent
        # backoff must happen before the jog
        assert conn.sent.index("A1 BMB 10") < conn.sent.index("A1 JOG -10")

    def test_no_axis_config_raises(self):
        # There is no fallback: without input_index there is no way to know
        # which INB bit to poll for the trip.
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        config = HomingConfig(homing_speed=10.0, standoff_distance=5.0)
        proc = HomingProcedure(cmd, config)
        with pytest.raises(ValueError, match="No AxisHomingConfig"):
            proc.home_axis(X_AXIS)
        assert conn.sent == []  # never reached the wire

    def test_trip_on_high_false_treats_low_as_tripped(self):
        # Normally-closed wiring: the switch reads LOW when triggered.
        responses = {
            "INB 1": _trip_after(1, before="1", after="0"),  # HIGH (untripped) then LOW (tripped)
            "A1 JOG -10": "-10",
            "A1 BST": "0",
            "A1 MIF": _trip_after(0),
            "A1 ACP": "0",
            "A1 ACP 0": "0",
            "A1 BMT 5": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=1.0, backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1, trip_on_high=False)},
        )
        proc = HomingProcedure(cmd, config)
        proc.home_axis(X_AXIS)
        assert conn.sent.count("INB 1") == 2  # untripped read, then the tripped read


def _make_sequence(values):
    """Callable response: returns successive values from `values`, repeating the last."""
    state = {"calls": 0}

    def _resp(cmd):
        idx = min(state["calls"], len(values) - 1)
        state["calls"] += 1
        return values[idx]

    return _resp


class TestHomeAxisTimeout:
    def test_switch_never_trips_aborts_and_raises(self):
        responses = {
            "INB 1": "0",  # never trips
            "A1 JOG -10": "-10",
            "A1 ABT": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=0.02, backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},
        )
        proc = HomingProcedure(cmd, config)
        with pytest.raises(SnapMotionError):
            proc.home_axis(X_AXIS)
        assert "A1 ABT" in conn.sent


class TestHomeAxisBrakeHandling:
    def test_disengages_brake_for_y_axis(self):
        responses = {
            "SOB 4 1": "0",  # disengage
            # 1st call is the not-already-tripped backoff check; trips on
            # the 2nd (the first poll inside the jog-and-wait loop).
            "INB 3": _trip_after(1),
            "A2 JOG -10": "-10",
            "A2 BST": "0",
            "A2 MIF": _trip_after(0),
            "A2 ACP": "0",
            "A2 ACP 0": "0",
            "A2 BMT 5": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        io_map = IOMap(y_brake_output=4)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=1.0, backoff_timeout_s=1.0,
            axis_configs={Y_AXIS: AxisHomingConfig(input_index=3)},
        )
        proc = HomingProcedure(cmd, config, io_map=io_map)
        proc.home_axis(Y_AXIS)
        assert "SOB 4 1" in conn.sent
        # Command-only: INB 8 proved unreliable on hardware and is never read.
        assert "INB 8" not in conn.sent
        assert conn.sent.index("SOB 4 1") < conn.sent.index("A2 JOG -10")

    def test_raises_if_brake_channel_not_configured(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0,
            axis_configs={Y_AXIS: AxisHomingConfig(input_index=3)},
        )
        io_map = IOMap(y_brake_output=None)  # explicitly unset
        proc = HomingProcedure(cmd, config, io_map=io_map)
        with pytest.raises(ValueError):
            proc.home_axis(Y_AXIS)

    def test_z_brake_release_trusts_the_commanded_output_and_proceeds(self):
        # Brakes are command-only — no status input is read for any axis.
        responses = {
            "SOB 5 1": "0",  # disengage — never followed by an INB read for status
            "INB 5": _trip_after(1),  # 1st call = backoff check, 2nd = tripped
            "A5 JOG -10": "-10", "A5 BST": "0", "A5 MIF": _trip_after(0),
            "A5 ACP": "0", "A5 ACP 0": "0", "A5 BMT 5": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=1.0, backoff_timeout_s=1.0,
            axis_configs={Z_AXIS: AxisHomingConfig(input_index=5)},
        )
        proc = HomingProcedure(cmd, config)  # default IOMap
        final_pos = proc.home_axis(Z_AXIS)
        assert final_pos == 0.0
        assert "SOB 5 1" in conn.sent
        assert "INB 8" not in conn.sent  # Y's status channel, never queried for Z


class TestLocateLimitSwitch:
    def test_locates_without_rezeroing_and_backs_off(self):
        # homing_direction defaults to -1.0 (X's config below), so
        # locate_limit_switch should jog +1 (opposite) and, on trip, back
        # off by -direction*standoff = -5 (relative move).
        responses = {
            "INB 2": _trip_after(1),  # X's limit input; 1st call is the backoff check
            "A1 JOG 10": "10",
            "A1 BST": "0",
            "A1 MIF": _trip_after(0),
            "A1 ACP": _make_sequence(["42.0", "45.0"]),  # trip position, then post-backoff readout
            "A1 BMB -5": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=1.0, backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},  # home switch, unused here
        )
        proc = HomingProcedure(cmd, config)
        trip_pos = proc.locate_limit_switch(X_AXIS)
        assert trip_pos == 42.0  # the found location, not the post-backoff position
        assert "A1 BMB -5" in conn.sent  # backed off, away from the switch
        assert not any(c.startswith("A1 ACP ") for c in conn.sent)  # never re-zeroed

    def test_no_axis_config_raises(self):
        conn = FakeSnapConnection({})
        cmd = MMCCommands(conn)
        config = HomingConfig(homing_speed=10.0, standoff_distance=5.0)
        proc = HomingProcedure(cmd, config)
        with pytest.raises(ValueError, match="No AxisHomingConfig"):
            proc.locate_limit_switch(X_AXIS)
        assert conn.sent == []


class TestHomeAll:
    def test_homes_all_axes_in_configured_order(self):
        responses = {
            # X — 1st INB call is the backoff check, 2nd is the tripped poll
            "INB 1": _trip_after(1),
            "A1 JOG -10": "-10", "A1 BST": "0",
            "A1 MIF": _trip_after(0), "A1 ACP": "0",
            "A1 ACP 0": "0", "A1 BMT 5": "0",
            # Y (brake release + home)
            "SOB 4 1": "0", "INB 8": "1",
            "INB 3": _trip_after(1),
            "A2 JOG -10": "-10", "A2 BST": "0",
            "A2 MIF": _trip_after(0), "A2 ACP": "0",
            "A2 ACP 0": "0", "A2 BMT 5": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        io_map = IOMap(y_brake_output=4)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=1.0, backoff_timeout_s=1.0,
            home_order=(X_AXIS, Y_AXIS),
            axis_configs={
                X_AXIS: AxisHomingConfig(input_index=1),
                Y_AXIS: AxisHomingConfig(input_index=3),
            },
        )
        proc = HomingProcedure(cmd, config, io_map=io_map)
        result = proc.home_all()
        assert result.success is True
        assert result.axis_results == {"X": 0.0, "Y": 0.0}
        # order matters: X homes before Y, as configured
        assert conn.sent.index("A1 JOG -10") < conn.sent.index("A2 JOG -10")

    def test_stops_and_reports_failure_on_first_axis_that_fails(self):
        responses = {
            "INB 1": "0",  # never trips -> timeout
            "A1 JOG -10": "-10", "A1 ABT": "0",
        }
        conn = FakeSnapConnection(responses)
        cmd = MMCCommands(conn)
        config = HomingConfig(
            homing_speed=10.0, standoff_distance=5.0, poll_interval_s=0.001,
            default_timeout_s=0.02, backoff_timeout_s=1.0,
            home_order=(X_AXIS, Y_AXIS),
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},
        )
        proc = HomingProcedure(cmd, config)
        result = proc.home_all()
        assert result.success is False
        assert "X" not in result.axis_results
        assert "Y" not in result.axis_results  # never reached — X failed first
        assert not any(c.startswith("A2") for c in conn.sent)


class TestSearchTimeoutFromTravel:
    """A flat 60 s timed out long before X's home switch: ~8000 mm at 10 mm/s is ~800 s."""

    def test_timeout_scales_with_the_axis_throw(self):
        config = HomingConfig(homing_speed=10.0)
        x = AxisHomingConfig(input_index=1, max_travel_mm=8000.0)
        z = AxisHomingConfig(input_index=5, max_travel_mm=400.0)
        assert config.search_timeout_s(x) == pytest.approx(1.5 * 8000 / 10 + 30)
        assert config.search_timeout_s(z) == pytest.approx(1.5 * 400 / 10 + 30)

    def test_an_axis_without_a_throw_uses_the_default(self):
        config = HomingConfig(default_timeout_s=60.0)
        assert config.search_timeout_s(AxisHomingConfig(input_index=1)) == 60.0

    def test_config_defaults_give_every_linear_axis_a_throw(self):
        from laguna.config import Config
        from laguna.robot.macron.controller import GantryController

        gantry = GantryController.from_config(Config())
        throws = {
            axis.name: cfg.max_travel_mm for axis, cfg in gantry.homing._config.axis_configs.items()
        }
        assert throws == {"X": 8000.0, "Y": 1200.0, "Z": 400.0}


class TestHomingIsHaltable:
    def _procedure(self, responses):
        conn = FakeSnapConnection(responses)
        config = HomingConfig(
            poll_interval_s=0.001, default_timeout_s=5.0, backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},
        )
        return HomingProcedure(MMCCommands(conn), config), conn

    def test_a_halt_during_the_switch_search_raises_motion_halted(self):
        from laguna.robot.macron.halt import HaltLatch, HaltLevel, MotionHalted

        latch = HaltLatch()
        polls = {"n": 0}

        def switch(_cmd):
            polls["n"] += 1
            if polls["n"] == 3:
                latch.trip(HaltLevel.PAUSE, "pause()")
            return "0"  # never trips (trip_on_high default) — would run to timeout

        proc, conn = self._procedure({"INB 1": switch, "A1 JOG -10": "-10"})
        proc._guard = latch.guard("home_axis(X)")
        start = time.monotonic()
        with pytest.raises(MotionHalted):
            proc.home_axis(X_AXIS)
        assert time.monotonic() - start < 2.0

    def test_home_all_does_not_swallow_a_halt_as_an_axis_failure(self):
        from laguna.robot.macron.halt import HaltLatch, HaltLevel, MotionHalted

        latch = HaltLatch()
        proc, conn = self._procedure({"INB 1": "0", "A1 JOG -10": "-10"})
        proc._config.home_order = (X_AXIS,)
        proc._guard = latch.guard("home()")
        latch.trip(HaltLevel.ESTOP, "estop()")
        with pytest.raises(MotionHalted):
            proc.home_all()
        assert "A1 JOG -10" not in conn.sent


class TestAFailureMidSearchStopsTheAxis:
    """Hardware regression 2026-10-05: an INB read timed out mid-search, the
    error escaped, and the jog kept running until a person paused it."""

    def _procedure(self, inb):
        conn = FakeSnapConnection({"INB 1": inb, "A1 JOG -10": "-10", "A1 ABT": "0"})
        config = HomingConfig(
            poll_interval_s=0.001, default_timeout_s=5.0, backoff_timeout_s=1.0,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},
        )
        return HomingProcedure(MMCCommands(conn), config), conn

    def test_a_switch_read_timeout_aborts_the_jog(self):
        reads = {"n": 0}

        def inb(_cmd):
            reads["n"] += 1
            if reads["n"] >= 3:
                raise SnapMotionError(600, "Timed out waiting for agent response to 'INB 1'")
            return "0"

        proc, conn = self._procedure(inb)
        with pytest.raises(SnapMotionError, match="Timed out"):
            proc.home_axis(X_AXIS)
        assert conn.sent.index("A1 JOG -10") < conn.sent.index("A1 ABT")

    def test_home_all_still_reports_the_failure_after_stopping(self):
        def inb(_cmd):
            raise SnapMotionError(600, "timeout")

        proc, conn = self._procedure(inb)
        proc._config.home_order = (X_AXIS,)
        result = proc.home_all()
        assert result.success is False
        # The first INB read (already-tripped check) fails before any jog, so
        # an abort is still sent — harmless, and the axis is certainly stopped.
        assert "A1 ABT" in conn.sent

    def test_the_abort_is_retried_when_the_link_is_flaky(self):
        attempts = {"n": 0}

        def abt(_cmd):
            attempts["n"] += 1
            if attempts["n"] < 3:
                raise SnapMotionError(600, "timeout")
            return "0"

        reads = {"n": 0}

        def inb(_cmd):
            reads["n"] += 1
            if reads["n"] >= 2:
                raise SnapMotionError(600, "timeout")
            return "0"

        proc, conn = self._procedure(inb)
        conn.responses["A1 ABT"] = abt
        with pytest.raises(SnapMotionError):
            proc.home_axis(X_AXIS)
        assert attempts["n"] == 3


class TestSwitchReadsAreShortTimeout:
    def test_switch_reads_during_a_search_use_the_short_timeout(self):
        """A stalled read at the 5 s default let the jog run 5 s unwatched
        (hardware, 2026-10-05: ~50 mm past Y's switch)."""
        conn = FakeSnapConnection({
            "INB 1": _trip_after(2), "A1 JOG -10": "-10", "A1 BST": "0",
            "A1 MIF": _trip_after(0), "A1 ACP": "0", "A1 ACP 0": "0", "A1 BMT 5": "0",
        })
        config = HomingConfig(
            poll_interval_s=0.001, default_timeout_s=1.0, backoff_timeout_s=1.0,
            switch_read_timeout_s=0.3,
            axis_configs={X_AXIS: AxisHomingConfig(input_index=1)},
        )
        HomingProcedure(MMCCommands(conn), config).home_axis(X_AXIS)
        inb_timeouts = [t for c, t in zip(conn.sent, conn.timeouts) if c == "INB 1"]
        assert inb_timeouts and all(t == 0.3 for t in inb_timeouts)
