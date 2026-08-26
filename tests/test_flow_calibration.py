"""Tests for laguna.flow.calibration — pure math + CSV I/O, no hardware
needed."""

import math

import pytest

from laguna.flow.calibration import PumpCalibration, PumpCalibrationPoint


def _points(pairs):
    """Helper: [(freq_hz, discharge_lpm), ...] -> [PumpCalibrationPoint, ...]"""
    return [PumpCalibrationPoint(freq_hz=hz, discharge_lpm=lpm) for hz, lpm in pairs]


class TestFit:
    def test_perfect_quadratic_three_points(self):
        """lpm = 0.01*hz^2 + 0.5*hz, exact — three points should recover it."""

        def lpm_of(hz):
            return 0.01 * hz**2 + 0.5 * hz

        pts = _points([(hz, lpm_of(hz)) for hz in [10, 30, 50]])
        cal = PumpCalibration.fit("test pump", pts)
        assert abs(cal.coeffs[0] - 0.01) < 1e-9
        assert abs(cal.coeffs[1] - 0.5) < 1e-9
        assert abs(cal.coeffs[2] - 0.0) < 1e-9
        assert abs(cal.r_squared - 1.0) < 1e-9

    def test_noisy_points_r_squared_below_one(self):
        pts = _points([(10, 5.1), (20, 10.3), (30, 14.8), (40, 20.5), (50, 24.9)])
        cal = PumpCalibration.fit("test pump", pts)
        assert cal.r_squared < 1.0
        assert cal.r_squared > 0.9

    def test_raises_with_fewer_than_three_points(self):
        with pytest.raises(ValueError):
            PumpCalibration.fit("test pump", _points([(10, 5.0), (20, 10.0)]))

    def test_raises_with_zero_points(self):
        with pytest.raises(ValueError):
            PumpCalibration.fit("test pump", [])

    def test_created_at_is_set(self):
        cal = PumpCalibration.fit("test pump", _points([(10, 5), (30, 15), (50, 25)]))
        assert cal.created_at

    def test_device_stored_verbatim(self):
        cal = PumpCalibration.fit("main inlet pump", _points([(10, 5), (30, 15), (50, 25)]))
        assert cal.device == "main inlet pump"

    def test_hz_max_defaults_and_is_stored(self):
        cal = PumpCalibration.fit("test pump", _points([(10, 5), (30, 15), (50, 25)]))
        assert cal.hz_max == 60.0
        cal2 = PumpCalibration.fit("test pump", _points([(10, 5), (30, 15), (50, 25)]), hz_max=50.0)
        assert cal2.hz_max == 50.0


class TestLpmForHz:
    def test_matches_fit_points(self):
        pts = _points([(10, 5.0), (30, 15.0), (50, 25.0)])  # exactly linear: lpm = 0.5*hz
        cal = PumpCalibration.fit("test pump", pts)
        assert abs(cal.lpm_for_hz(10) - 5.0) < 1e-6
        assert abs(cal.lpm_for_hz(30) - 15.0) < 1e-6

    def test_manual_construction(self):
        cal = PumpCalibration(device="test pump", coeffs=[0.01, 0.5, 1.0])
        assert abs(cal.lpm_for_hz(0) - 1.0) < 1e-9
        assert abs(cal.lpm_for_hz(10) - (0.01 * 100 + 5.0 + 1.0)) < 1e-9


class TestHzForLpm:
    def test_inverts_linear_relationship(self):
        pts = _points([(10, 5.0), (30, 15.0), (50, 25.0)])  # lpm = 0.5*hz
        cal = PumpCalibration.fit("test pump", pts)
        assert abs(cal.hz_for_lpm(10.0) - 20.0) < 1e-6

    def test_roundtrip_with_lpm_for_hz(self):
        pts = _points([(10, 5.1), (20, 10.3), (30, 14.8), (40, 20.5), (50, 24.9)])
        cal = PumpCalibration.fit("test pump", pts)
        for hz in [15.0, 25.0, 35.0]:
            lpm = cal.lpm_for_hz(hz)
            recovered_hz = cal.hz_for_lpm(lpm)
            assert abs(recovered_hz - hz) < 1e-3

    def test_raises_when_target_unreachable(self):
        """A target discharge far outside the calibrated/physical range
        (e.g. requesting more flow than the pump can produce even at
        hz_max) has no valid solution — must raise, not silently return a
        frequency outside [0, hz_max] or a nonsense root."""
        pts = _points([(10, 5.0), (30, 15.0), (50, 25.0)])  # lpm = 0.5*hz, hz_max=60 -> max 30 lpm
        cal = PumpCalibration.fit("test pump", pts)
        with pytest.raises(ValueError):
            cal.hz_for_lpm(1000.0)

    def test_this_session_bad_calibration_example(self):
        """Regression context: the old hardcoded C0/C1/C2 curve computed
        1 L/min to 63.48 Hz (clamped to the VFD's 60 Hz max) — essentially
        full speed for what was meant to be a low test rate. A real
        calibration fit for the same nominal range should not reproduce
        that: a low discharge target should map to a low frequency."""
        pts = _points([(10, 1.0), (20, 2.2), (30, 3.1), (40, 4.3), (50, 5.2)])
        cal = PumpCalibration.fit("test pump", pts)
        hz = cal.hz_for_lpm(1.0)
        assert hz < 20.0  # nowhere near saturating a 60 Hz drive for 1 L/min


class TestResiduals:
    def test_residuals_zero_for_exact_fit(self):
        pts = _points([(10, 5.0), (30, 15.0), (50, 25.0)])
        cal = PumpCalibration.fit("test pump", pts)
        for r in cal.residuals_lpm():
            assert abs(r) < 1e-6

    def test_residuals_length_matches_points(self):
        pts = _points([(10, 5.1), (20, 10.3), (30, 14.8), (40, 20.5), (50, 24.9)])
        cal = PumpCalibration.fit("test pump", pts)
        assert len(cal.residuals_lpm()) == len(pts)


class TestCsvRoundtrip:
    def test_roundtrip_preserves_fit_params(self, tmp_path):
        pts = _points([(10, 5.1), (20, 10.3), (30, 14.8), (40, 20.5), (50, 24.9)])
        cal = PumpCalibration.fit("test pump", pts)
        path = tmp_path / "pump_cal.csv"
        cal.to_csv(path)
        loaded = PumpCalibration.from_csv(path)

        assert loaded.device == cal.device
        for a, b in zip(loaded.coeffs, cal.coeffs):
            assert abs(a - b) < 1e-9
        assert loaded.hz_max == cal.hz_max
        assert abs(loaded.r_squared - cal.r_squared) < 1e-9
        assert loaded.created_at == cal.created_at

    def test_roundtrip_preserves_points(self, tmp_path):
        pts = _points([(10, 5.1), (20, 10.3), (30, 14.8)])
        cal = PumpCalibration.fit("test pump", pts)
        path = tmp_path / "pump_cal.csv"
        cal.to_csv(path)
        loaded = PumpCalibration.from_csv(path)

        assert len(loaded.points) == len(pts)
        for orig, rt in zip(cal.points, loaded.points):
            assert abs(orig.freq_hz - rt.freq_hz) < 1e-9
            assert abs(orig.discharge_lpm - rt.discharge_lpm) < 1e-9

    def test_roundtrip_hz_for_lpm_matches(self, tmp_path):
        """The whole point: a loaded calibration should convert flow rates
        identically to the original."""
        pts = _points([(10, 5.1), (20, 10.3), (30, 14.8), (40, 20.5), (50, 24.9)])
        cal = PumpCalibration.fit("test pump", pts)
        path = tmp_path / "pump_cal.csv"
        cal.to_csv(path)
        loaded = PumpCalibration.from_csv(path)

        for lpm in [6.0, 12.0, 18.0]:
            assert abs(loaded.hz_for_lpm(lpm) - cal.hz_for_lpm(lpm)) < 1e-6

    def test_from_csv_rejects_non_calibration_file(self, tmp_path):
        path = tmp_path / "not_a_calibration.csv"
        path.write_text("foo,bar\n1,2\n")
        with pytest.raises(ValueError):
            PumpCalibration.from_csv(path)

    def test_to_csv_accepts_str_path(self, tmp_path):
        pts = _points([(10, 5.0), (30, 15.0), (50, 25.0)])
        cal = PumpCalibration.fit("test pump", pts)
        path_str = str(tmp_path / "pump_cal.csv")
        cal.to_csv(path_str)  # should not raise
        loaded = PumpCalibration.from_csv(path_str)
        assert abs(loaded.hz_for_lpm(10.0) - cal.hz_for_lpm(10.0)) < 1e-6

    def test_negative_and_float_values_survive_roundtrip(self, tmp_path):
        pts = _points([(10.5, 5.123456), (25.75, 12.987654), (40.1, 19.333333)])
        cal = PumpCalibration.fit("test pump", pts)
        path = tmp_path / "pump_cal.csv"
        cal.to_csv(path)
        loaded = PumpCalibration.from_csv(path)
        for a, b in zip(loaded.coeffs, cal.coeffs):
            assert math.isclose(a, b, rel_tol=1e-9)
