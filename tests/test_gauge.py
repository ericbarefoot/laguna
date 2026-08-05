"""Smoke tests for SaflWaterLevelSensor — construct, disconnected behavior,
and read_mm()'s operational-log-only logging (a reading is data, not an
archival "step taken" — see laguna.subsystem_logging's module docstring).

safl_ocean_hardware is not installed in CI/dev, so connect() always fails
here; that is exactly the "disconnected" state get_status()/disconnect()
must survive without raising.
"""

import logging

from laguna.gauge import SaflWaterLevelSensor


class TestSaflWaterLevelSensorDisconnected:
    def test_construct(self):
        sensor = SaflWaterLevelSensor({})
        assert sensor.subsystem_name == "gauge"

    def test_connect_fails_without_hardware_driver(self):
        sensor = SaflWaterLevelSensor({})
        assert sensor.connect() is False

    def test_disconnect_is_safe_when_never_connected(self):
        sensor = SaflWaterLevelSensor({})
        sensor.disconnect()  # must not raise

    def test_get_status_reports_disconnected_with_no_reading_yet(self):
        sensor = SaflWaterLevelSensor({})
        status = sensor.get_status()
        assert status["is_connected"] is False
        assert status["elevation_mm"] is None


class _FakeEventLog:
    def __init__(self):
        self.rows = []

    def log(self, runtime_s, subsystem, action, result="ok", notes=""):
        self.rows.append((runtime_s, subsystem, action, result, notes))


class _FakeClock:
    def elapsed(self):
        return 1.5


class _FakeMassaSensor:
    def read(self):
        return {"distance_cm": 5.0, "temperature": 20.0, "signal_strength": 90}


class TestSaflWaterLevelSensorLogging:
    def _connected_sensor(self, offset_mm=100.0):
        sensor = SaflWaterLevelSensor({"offset_mm": offset_mm})
        sensor._sensor = _FakeMassaSensor()
        return sensor

    def test_read_mm_logs_to_the_operational_log(self, caplog):
        sensor = self._connected_sensor()
        with caplog.at_level(logging.INFO, logger="laguna.gauge.sensor"):
            elevation_mm = sensor.read_mm()

        assert elevation_mm == 100.0 - 5.0 * 10.0
        assert "elevation_mm=50.00" in caplog.text

    def test_read_mm_never_writes_to_the_archival_event_log(self):
        """A reading measures the experiment's state without changing it —
        it's data, not a milestone. Stays out of the archival CSV even when
        attached; a caller that wants it archived uses lab.log_note() or
        runner.py's log_as_event opt-in instead."""
        sensor = self._connected_sensor()
        event_log = _FakeEventLog()
        sensor.attach_event_log(event_log, _FakeClock())

        sensor.read_mm()

        assert event_log.rows == []

    def test_read_mm_does_not_raise_before_attach(self):
        sensor = self._connected_sensor()
        sensor.read_mm()  # must not raise even though never attached


class TestSaflWaterLevelSensorSimulated:
    """simulated: True builds a SimulatedMassaSensor instead of the real
    driver — connect() succeeds, but every reading is NaN rather than a
    fabricated water level. See laguna.simulation."""

    def test_connect_succeeds_without_safl_ocean_hardware(self):
        sensor = SaflWaterLevelSensor({"simulated": True})
        assert sensor.connect() is True
        assert sensor._is_connected is True

    def test_read_mm_is_nan(self):
        import math

        sensor = SaflWaterLevelSensor({"simulated": True})
        sensor.connect()
        assert math.isnan(sensor.read_mm())

    def test_read_mm_smoothed_is_nan(self):
        import math

        sensor = SaflWaterLevelSensor({"simulated": True})
        sensor.connect()
        assert math.isnan(sensor.read_mm_smoothed())

    def test_get_status_elevation_is_nan_after_a_read(self):
        import math

        sensor = SaflWaterLevelSensor({"simulated": True})
        sensor.connect()
        sensor.read_mm()
        status = sensor.get_status()
        assert status["is_connected"] is True
        assert math.isnan(status["elevation_mm"])
