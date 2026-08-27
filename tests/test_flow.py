"""Tests for SaflFlowController — construct, disconnected safety verbs, and
the MQTT request/reply command path across its two channels (Fuji VFD +
shared ClearCore valve axis).

See FakeMqttSubscriber in mqtt_fixtures.py for the double used here in
place of a real broker.
"""

import math

import pytest

from laguna.config import Config
from laguna.flow import SaflFlowController
from laguna.flow.calibration import PumpCalibration, PumpCalibrationPoint

from mqtt_fixtures import FakeMqttSubscriber


def _make_flow(**config):
    mqtt = FakeMqttSubscriber()
    controller = SaflFlowController(
        {
            "vfd_topic_status": "node/vfd",
            "vfd_topic_commands": "node/vfd/commands",
            "vfd_topic_replies": "node/vfd/replies",
            "valve_topic_status": "node/valve",
            "valve_topic_commands": "node/valve/commands",
            "valve_topic_replies": "node/valve/replies",
            **config,
        },
        mqtt,
    )
    return controller, mqtt


def _accept_all_vfd_replies(mqtt, **fields):
    mqtt.auto_reply_on(
        "node/vfd/commands",
        "node/vfd/replies",
        lambda payload: {"request_id": payload["request_id"], **fields},
    )


def _accept_all_valve_replies(mqtt, **fields):
    mqtt.auto_reply_on(
        "node/valve/commands",
        "node/valve/replies",
        lambda payload: {"request_id": payload["request_id"], **fields},
    )


class TestSaflFlowControllerDisconnected:
    def test_construct(self):
        controller, _ = _make_flow()
        assert controller.subsystem_name == "flow"

    def test_pause_never_raises(self):
        controller, _ = _make_flow()
        note = controller.pause()
        assert note is not None

    def test_resume_never_raises(self):
        controller, _ = _make_flow()
        note = controller.resume()
        assert note is not None

    def test_stop_never_raises(self):
        controller, _ = _make_flow()
        note = controller.stop()
        assert note is not None

    def test_estop_never_raises(self):
        controller, _ = _make_flow()
        note = controller.estop()
        assert note is not None

    def test_disconnect_is_safe_when_never_connected(self):
        controller, _ = _make_flow()
        controller.disconnect()  # must not raise

    def test_get_status_reports_disconnected(self):
        controller, _ = _make_flow()
        status = controller.get_status()
        assert status["is_connected"] is False


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 7.0


class TestSaflFlowControllerConnected:
    def test_connect_subscribes_to_all_four_topics(self):
        controller, mqtt = _make_flow()
        assert controller.connect() is True
        for topic in ("node/vfd", "node/vfd/replies", "node/valve", "node/valve/replies"):
            assert topic in mqtt._topics

    def test_connect_fails_if_broker_handshake_never_completes(self):
        """Regression: connect() used to return True as soon as the async
        MQTT handshake was *started*, not once it actually completed —
        a command issued right after connect() could then race the
        handshake and fail with "MqttSubscriber is not connected" even
        though connect() had already reported success."""
        controller, mqtt = _make_flow()
        mqtt.wait_until_connected = lambda timeout=5.0, poll_interval=0.05: False
        assert controller.connect() is False

    def test_set_flowrate_acked_updates_cached_setpoint_and_logs(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.set_flowrate(12.5) is True
        assert controller.get_flowrate() == 12.5
        assert event_log.rows == [(7.0, "flow", "set_flowrate", "ok", "flowrate_lpm=12.50")]

    def test_set_flowrate_not_acked_leaves_cached_setpoint_unchanged(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=False)

        assert controller.set_flowrate(12.5) is False
        assert controller.get_flowrate() == 0.0

    def test_set_flowrate_times_out_without_a_reply(self):
        controller, mqtt = _make_flow(command_timeout_s=0.05)
        controller.connect()
        assert controller.set_flowrate(12.5) is False

    def test_set_flowrate_uses_calibration_file_when_configured(self, tmp_path):
        """When calibration_file is set, set_flowrate() must send the
        frequency PumpCalibration.hz_for_lpm() computes, not the legacy
        C0/C1/C2 formula — this is the whole point of the calibration
        file: it overrides the (possibly wrong) hardcoded coefficients."""
        pts = [
            PumpCalibrationPoint(freq_hz=hz, discharge_lpm=lpm)
            for hz, lpm in [(10, 1.0), (20, 2.2), (30, 3.1), (40, 4.3), (50, 5.2)]
        ]
        cal = PumpCalibration.fit("test pump", pts)
        cal_path = tmp_path / "pump_cal.csv"
        cal.to_csv(cal_path)

        controller, mqtt = _make_flow(calibration_file=str(cal_path))
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)

        assert controller.set_flowrate(1.0) is True
        expected_hz = cal.hz_for_lpm(1.0)
        commands = [
            payload["args"]
            for topic, payload in mqtt.published
            if topic == "node/vfd/commands" and payload["command"] == "set_setpoint_hz"
        ]
        assert abs(commands[-1]["freq_hz"] - expected_hz) < 1e-6
        assert expected_hz < 20.0  # sanity: nowhere near the C0/C1/C2 bug's 63 Hz result

    def test_set_flowrate_without_calibration_file_uses_legacy_coefficients(self):
        """No calibration_file configured — falls back to the C0/C1/C2
        formula, unchanged from before calibration support was added."""
        controller, mqtt = _make_flow(C0=4.902, C1=58.49, C2=0.08956)
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)

        controller.set_flowrate(1.0)
        commands = [
            payload["args"]
            for topic, payload in mqtt.published
            if topic == "node/vfd/commands" and payload["command"] == "set_setpoint_hz"
        ]
        assert abs(commands[-1]["freq_hz"] - 60.0) < 1e-6  # clamped, matches the known bug

    def test_set_frequency_hz_acked_sends_raw_hz_and_invalidates_lpm_cache(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)

        assert controller.set_frequency_hz(30.0) is True
        assert math.isnan(controller.get_flowrate())
        commands = [
            (topic, payload)
            for topic, payload in mqtt.published
            if topic == "node/vfd/commands"
        ]
        assert commands[-1][1]["command"] == "set_setpoint_hz"
        assert commands[-1][1]["args"]["freq_hz"] == 30.0

    def test_set_frequency_hz_clamps_to_vfd_range(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)

        controller.set_frequency_hz(999.0)
        commands = [
            payload for topic, payload in mqtt.published if topic == "node/vfd/commands"
        ]
        assert commands[-1]["args"]["freq_hz"] == 60.0

    def test_set_frequency_hz_invalidates_lpm_cache_even_when_rejected(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        controller.set_flowrate(12.5)
        assert controller.get_flowrate() == 12.5

        _accept_all_vfd_replies(mqtt, ok=False)
        assert controller.set_frequency_hz(30.0) is False
        assert math.isnan(controller.get_flowrate())

    def test_start_and_stop_publish_to_the_vfd_channel(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        assert controller.start() is True
        assert controller.stop() is None
        commands = [payload["command"] for topic, payload in mqtt.published if topic == "node/vfd/commands"]
        assert commands == ["start_motor", "stop_motor"]

    def test_qin_and_qaux_publish_to_the_valve_channel_and_log(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_valve_replies(mqtt, ok=True)
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        controller.qin = True
        controller.qaux = False

        assert controller.qin is True
        assert controller.qaux is False
        assert event_log.rows == [
            (7.0, "flow", "qin", "ok", "state=True"),
            (7.0, "flow", "qaux", "ok", "state=False"),
        ]
        commands = [payload["command"] for topic, payload in mqtt.published if topic == "node/valve/commands"]
        assert commands == ["set_io", "set_io"]

    def test_qin_setter_raises_without_a_reply(self):
        """A valve command that never gets acked must not be silently assumed
        to have happened — it raises rather than caching a state the
        hardware never confirmed."""
        controller, mqtt = _make_flow(command_timeout_s=0.05)
        controller.connect()
        try:
            controller.qin = True
            assert False, "expected RuntimeError"
        except RuntimeError:
            pass
        assert controller.qin is False

    def test_qin_setter_raises_on_rejected_reply_and_leaves_cache_unchanged(self):
        """Regression: the setter used to only catch RequestTimeout, so a
        reply that arrived but carried ok:False (e.g. confluence rejecting
        the command) was silently treated as success — the cached state
        flipped and the caller had no way to know the valve didn't
        actually move."""
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_valve_replies(mqtt, ok=False)
        with pytest.raises(RuntimeError):
            controller.qin = True
        assert controller.qin is False

    def test_qaux_setter_raises_on_rejected_reply_and_leaves_cache_unchanged(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_valve_replies(mqtt, ok=False)
        with pytest.raises(RuntimeError):
            controller.qaux = True
        assert controller.qaux is False

    def test_estop_closes_both_valves_and_stops_pump_independently(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        _accept_all_valve_replies(mqtt, ok=True)
        assert controller.estop() is None
        assert controller.qin is False
        assert controller.qaux is False

    def test_estop_reports_valve_rejection_but_still_stops_pump(self):
        """estop() must not silently believe a rejected valve command
        succeeded — the qin/qaux setters' ok:False check needs to surface
        through estop()'s per-step exception handling, not just when the
        setter is called directly."""
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        _accept_all_valve_replies(mqtt, ok=False)
        note = controller.estop()
        assert note is not None
        assert "close qin" in note
        assert "close qaux" in note
        assert "stop the pump" not in note

    def test_estop_reports_pump_rejection(self):
        """Regression: _vfd_stop() used to swallow a rejected stop_motor
        reply into a `False` return that estop() never checked (it only
        catches exceptions) — the pump could keep running while estop()
        reported no problems at all."""
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=False)
        _accept_all_valve_replies(mqtt, ok=True)
        note = controller.estop()
        assert note is not None
        assert "stop the pump" in note

    def test_stop_reports_pump_rejection(self):
        """Same regression as estop(): stop() ignored _vfd_stop()'s return
        value and only caught exceptions, so a rejected stop_motor reply
        was invisible to it."""
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=False)
        note = controller.stop()
        assert note is not None

    def test_pause_reports_pump_rejection(self):
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=False)
        note = controller.pause()
        assert note is not None

    def test_resume_reports_start_rejection(self):
        """Regression: resume() called start() and ignored its bool return
        (start() returns False rather than raising), so a rejected
        start_motor command was reported as a successful resume()."""
        controller, mqtt = _make_flow()
        controller.connect()
        controller._paused_flowrate = 10.0
        _accept_all_vfd_replies(mqtt, ok=False)
        note = controller.resume()
        assert note is not None

    def test_resume_after_pause_with_nan_setpoint_does_not_send_nan_to_hardware(self):
        """Regression: pause() captures get_flowrate(), which is NaN after
        set_frequency_hz() (bypasses the L/min cache) rather than
        set_flowrate(). NaN is truthy in Python, so resume()'s old `if
        target:` guard would call set_flowrate(nan) — sending a NaN
        frequency command to the real VFD. resume() must restart at
        whatever frequency is already set instead."""
        controller, mqtt = _make_flow()
        controller.connect()
        _accept_all_vfd_replies(mqtt, ok=True)
        controller.set_frequency_hz(30.0)
        assert math.isnan(controller.get_flowrate())

        controller.pause()
        assert controller.resume() is None

        set_setpoint_calls = [
            payload["args"]
            for topic, payload in mqtt.published
            if topic == "node/vfd/commands" and payload["command"] == "set_setpoint_hz"
        ]
        assert set_setpoint_calls == [{"freq_hz": 30.0}]  # only the original call, no NaN resend

    def test_get_status_reads_latest_vfd_status_message(self):
        """Field names/format here match confluence's actual published dict
        (Fuji_VFD_funcs.py poll_status()) — "state" not "state_message",
        "freq setpoint" as a "X.XX Hz" display string not a bare "setpoint"
        float, and no e_stop key at all (see get_status()'s docstring)."""
        controller, mqtt = _make_flow()
        controller.connect()
        mqtt.push("node/vfd", {"state": "motor On - Forward", "freq setpoint": "42.00 Hz"})
        status = controller.get_status()
        assert status["vfd_state"] == "motor On - Forward"
        assert status["vfd_estop"] is None
        assert status["vfd_setpoint_hz"] == 42.0

    def test_get_status_handles_missing_or_malformed_vfd_fields(self):
        controller, mqtt = _make_flow()
        controller.connect()
        mqtt.push("node/vfd", {})
        status = controller.get_status()
        assert status["vfd_state"] is None
        assert math.isnan(status["vfd_setpoint_hz"])


class TestSaflFlowControllerSimulated:
    """simulated: True skips MQTT entirely — commands succeed (and log
    normally), readings come back NaN/None rather than fabricated data."""

    def test_connect_succeeds_without_a_broker(self):
        controller, mqtt = _make_flow(simulated=True)
        assert controller.connect() is True
        assert controller._is_connected is True
        assert mqtt.connected_called == 0

    def test_set_flowrate_start_stop_all_succeed_and_log(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        event_log = _FakeEventLog()
        controller.attach_event_log(event_log, _FakeClock())

        assert controller.set_flowrate(15.0) is True
        assert controller.start() is True
        assert controller.stop() is None

        actions = [row[2] for row in event_log.rows]
        assert actions == ["set_flowrate", "start", "stop"]

    def test_get_flowrate_echoes_the_commanded_setpoint_not_nan(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        controller.set_flowrate(22.5)
        assert controller.get_flowrate() == 22.5

    def test_get_status_vfd_fields_are_nan_or_none(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        status = controller.get_status()
        assert status["is_connected"] is True
        assert status["vfd_state"] is None
        assert status["vfd_estop"] is None
        assert math.isnan(status["vfd_setpoint_hz"])

    def test_qin_qaux_succeed_and_log(self):
        controller, _ = _make_flow(simulated=True)
        controller.connect()
        controller.qin = True
        assert controller.qin is True


class TestSaflFlowControllerFromConfig:
    """from_config() derives topics from mqtt.node_name — see
    config.py's comment above the "weir" section."""

    def test_topics_derived_from_node_name(self):
        config = Config(defaults={"flow": {}, "mqtt": {"node_name": "UCRS Confluence Node 1"}})
        controller = SaflFlowController.from_config(config)
        assert controller._vfd_status_topic == "UCRS Confluence Node 1/Fuji_Frenic_VFD"
        assert controller._vfd_commands_topic == "UCRS Confluence Node 1/Fuji_Frenic_VFD/commands"
        assert controller._valve_status_topic == "UCRS Confluence Node 1/flow_valve"
        assert controller._valve_commands_topic == "UCRS Confluence Node 1/flow_valve/commands"
