"""Tests for SaflWaterLevelSensor — construct, disconnected behavior, MQTT
message decoding, and read_mm()'s operational-log-only logging (a reading
is data, not an archival "step taken" — see laguna.subsystem_logging's
module docstring).

The sensor is now an MQTT client of the confluence node on red.lab rather
than a direct serial driver — see FakeMqttSubscriber in mqtt_fixtures.py
for the double used here in place of a real broker.
"""

import logging
import math

from laguna.config import Config
from laguna.gauge import SaflWaterLevelSensor

from mqtt_fixtures import FakeMqttSubscriber


def _massa_message(dist_mm=50.0, temperature=20.0, signal_strength="100%", index=0, count=1):
    """Build a confluence Massa_Ultrasonic payload (see
    confluence/Interfaces/Massa_Ultrasonic/Massa_funcs.py's `status` dict)."""
    dist_array = [float("nan")] * count
    temp_array = [float("nan")] * count
    strength_array = ["0%"] * count
    dist_array[index] = dist_mm
    temp_array[index] = temperature
    strength_array[index] = signal_strength
    return {
        "timestamp": "2026-08-25T00:00:00.000",
        "device names": ["gauge"] * count,
        "massa IDs": list(range(count)),
        "errors": ["OK"] * count,
        "dist_mm": dist_array,
        "signal_strength": strength_array,
        "target acquired": ["Yes"] * count,
        "massa temperature": temp_array,
        "offsets": [0] * count,
        "water_depth": [0.0] * count,
    }


def _make_gauge(**config):
    mqtt = FakeMqttSubscriber()
    sensor = SaflWaterLevelSensor({"topic": "node/Massa_Ultrasonic", **config}, mqtt)
    return sensor, mqtt


class TestSaflWaterLevelSensorDisconnected:
    def test_construct(self):
        sensor, _ = _make_gauge()
        assert sensor.subsystem_name == "gauge"

    def test_disconnect_is_safe_when_never_connected(self):
        sensor, _ = _make_gauge()
        sensor.disconnect()  # must not raise

    def test_get_status_reports_disconnected_with_no_reading_yet(self):
        sensor, _ = _make_gauge()
        status = sensor.get_status()
        assert status["is_connected"] is False
        assert status["elevation_mm"] is None

    def test_read_mm_raises_when_never_connected(self):
        sensor, _ = _make_gauge()
        try:
            sensor.read_mm()
            assert False, "expected RuntimeError"
        except RuntimeError:
            pass


class TestSaflWaterLevelSensorConnected:
    def test_connect_delegates_to_mqtt_and_subscribes(self):
        sensor, mqtt = _make_gauge()
        assert sensor.connect() is True
        assert mqtt.connected_called == 1
        assert "node/Massa_Ultrasonic" in mqtt._topics

    def test_connect_fails_if_broker_handshake_never_completes(self):
        """Regression: connect() used to return True as soon as the async
        MQTT handshake was *started*, not once it actually completed."""
        sensor, mqtt = _make_gauge()
        mqtt.wait_until_connected = lambda timeout=5.0, poll_interval=0.05: False
        assert sensor.connect() is False

    def test_connect_does_not_double_connect(self):
        sensor, mqtt = _make_gauge()
        mqtt._is_connected = True
        sensor.connect()
        assert mqtt.connected_called == 0

    def test_disconnect_delegates(self):
        sensor, mqtt = _make_gauge()
        sensor.connect()
        sensor.disconnect()
        assert mqtt.disconnect_called == 1
        assert sensor._is_connected is False

    def test_read_mm_before_any_message_raises(self):
        sensor, _ = _make_gauge()
        sensor.connect()
        try:
            sensor.read_mm()
            assert False, "expected RuntimeError"
        except RuntimeError:
            pass

    def test_read_mm_from_mqtt_message(self):
        sensor, mqtt = _make_gauge(offset_mm=100.0)
        sensor.connect()
        mqtt.push("node/Massa_Ultrasonic", _massa_message(dist_mm=5.0))
        # dist_mm is already millimeters (Massa_funcs.py computes it as
        # dist * 25.4, inches -> mm) — confirmed against a live red.lab
        # reading, no *10 scaling despite the old serial driver's
        # distance_cm convention.
        elevation_mm = sensor.read_mm()
        assert elevation_mm == 100.0 - 5.0

    def test_sensor_index_selects_correct_array_element(self):
        sensor, mqtt = _make_gauge(sensor_index=1, offset_mm=0.0)
        sensor.connect()
        mqtt.push(
            "node/Massa_Ultrasonic",
            _massa_message(dist_mm=7.0, index=1, count=2),
        )
        elevation_mm = sensor.read_mm()
        assert elevation_mm == -7.0

    def test_get_status_reflects_latest_message(self):
        sensor, mqtt = _make_gauge(offset_mm=0.0)
        sensor.connect()
        mqtt.push(
            "node/Massa_Ultrasonic",
            _massa_message(dist_mm=3.0, temperature=22.5, signal_strength="75%"),
        )
        status = sensor.get_status()
        assert status["is_connected"] is True
        assert status["elevation_mm"] == -3.0
        assert status["temperature_c"] == 22.5
        assert status["signal_strength"] == "75%"

    def test_malformed_message_is_dropped_not_raised(self):
        sensor, mqtt = _make_gauge()
        sensor.connect()
        mqtt.push("node/Massa_Ultrasonic", {"unexpected": "shape"})
        try:
            sensor.read_mm()
            assert False, "expected RuntimeError (no valid reading cached)"
        except RuntimeError:
            pass

    def test_read_mm_logs_to_the_operational_log(self, caplog):
        sensor, mqtt = _make_gauge(offset_mm=100.0)
        sensor.connect()
        mqtt.push("node/Massa_Ultrasonic", _massa_message(dist_mm=5.0))
        with caplog.at_level(logging.INFO, logger="laguna.gauge.sensor"):
            sensor.read_mm()
        assert "elevation_mm=95.00" in caplog.text

    def test_read_mm_never_writes_to_the_archival_event_log(self):
        """A reading measures the experiment's state without changing it —
        it's data, not a milestone. Stays out of the archival CSV even when
        attached; a caller that wants it archived uses lab.log_note() or
        runner.py's log_as_event opt-in instead."""
        sensor, mqtt = _make_gauge()
        sensor.connect()
        mqtt.push("node/Massa_Ultrasonic", _massa_message())

        class _FakeEventLog:
            def __init__(self):
                self.rows = []

            def log(self, runtime_s, subsystem, action, result="ok", notes=""):
                self.rows.append((runtime_s, subsystem, action, result, notes))

        class _FakeClock:
            def elapsed(self):
                return 1.5

        event_log = _FakeEventLog()
        sensor.attach_event_log(event_log, _FakeClock())
        sensor.read_mm()
        assert event_log.rows == []


class TestSaflWaterLevelSensorSimulated:
    """simulated: True skips MQTT entirely — connect() succeeds, but every
    reading is NaN rather than a fabricated water level."""

    def test_connect_succeeds_without_a_broker(self):
        sensor, mqtt = _make_gauge(simulated=True)
        assert sensor.connect() is True
        assert sensor._is_connected is True
        assert mqtt.connected_called == 0

    def test_read_mm_is_nan(self):
        sensor, _ = _make_gauge(simulated=True)
        sensor.connect()
        assert math.isnan(sensor.read_mm())

    def test_read_mm_smoothed_is_nan(self):
        sensor, _ = _make_gauge(simulated=True)
        sensor.connect()
        assert math.isnan(sensor.read_mm_smoothed())

    def test_get_status_elevation_is_nan_after_connect(self):
        sensor, _ = _make_gauge(simulated=True)
        sensor.connect()
        status = sensor.get_status()
        assert status["is_connected"] is True
        assert math.isnan(status["elevation_mm"])


class TestSaflWaterLevelSensorFromConfig:
    """from_config() derives the topic from mqtt.node_name — changing the
    node name should be a one-line edit, not a hunt through three
    subsystem sections (see config.py's comment above the "weir" section)."""

    def test_topic_derived_from_node_name(self):
        config = Config(defaults={"gauge": {}, "mqtt": {"node_name": "UCRS Confluence Node 1"}})
        sensor = SaflWaterLevelSensor.from_config(config)
        assert sensor._topic == "UCRS Confluence Node 1/Massa_Ultrasonic"

    def test_explicit_topic_overrides_derived_default(self):
        config = Config(
            defaults={
                "gauge": {"topic": "custom/topic"},
                "mqtt": {"node_name": "UCRS Confluence Node 1"},
            }
        )
        sensor = SaflWaterLevelSensor.from_config(config)
        assert sensor._topic == "custom/topic"
