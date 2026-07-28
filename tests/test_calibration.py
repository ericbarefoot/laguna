"""Tests for laguna.rangefinder.calibration — pure math + CSV I/O, no
hardware needed."""

import math

import pytest

from laguna.rangefinder.calibration import CalibrationPoint, LinearCalibration


def _points(pairs):
    """Helper: [(known_height_mm, raw_value), ...] -> [CalibrationPoint, ...]"""
    return [CalibrationPoint(known_height_mm=h, raw_value=r) for h, r in pairs]


class TestFit:
    def test_perfect_line_two_points(self):
        """height = 2*raw + 10, exact — two points should recover it exactly."""
        pts = _points([(10, 0), (30, 10)])
        cal = LinearCalibration.fit("od2000", pts)
        assert abs(cal.slope - 2.0) < 1e-9
        assert abs(cal.intercept - 10.0) < 1e-9
        assert abs(cal.r_squared - 1.0) < 1e-9

    def test_perfect_line_many_points(self):
        """height = -1*raw + 500 (inverted, as distance-down sensors are),
        exact across several points."""
        pts = _points([(500 - r, r) for r in [0, 100, 200, 300, 400]])
        cal = LinearCalibration.fit("wtt12l_powerprox", pts)
        assert abs(cal.slope - (-1.0)) < 1e-9
        assert abs(cal.intercept - 500.0) < 1e-9
        assert abs(cal.r_squared - 1.0) < 1e-9

    def test_noisy_points_r_squared_below_one(self):
        """Points that don't fall exactly on a line should fit with
        r_squared < 1, not raise or silently report a perfect fit."""
        pts = _points([(100, 0), (200, 10), (295, 20), (410, 30)])  # slight noise
        cal = LinearCalibration.fit("od2000", pts)
        assert cal.r_squared < 1.0
        assert cal.r_squared > 0.9  # still a good fit, just not exact

    def test_raises_with_fewer_than_two_points(self):
        with pytest.raises(ValueError):
            LinearCalibration.fit("od2000", _points([(100, 0)]))

    def test_raises_with_zero_points(self):
        with pytest.raises(ValueError):
            LinearCalibration.fit("od2000", [])

    def test_created_at_is_set(self):
        cal = LinearCalibration.fit("od2000", _points([(10, 0), (30, 10)]))
        assert cal.created_at  # non-empty

    def test_device_stored_verbatim(self):
        cal = LinearCalibration.fit("wtt12l_powerprox", _points([(10, 0), (30, 10)]))
        assert cal.device == "wtt12l_powerprox"


class TestApply:
    def test_apply_matches_fit_points(self):
        pts = _points([(10, 0), (30, 10)])
        cal = LinearCalibration.fit("od2000", pts)
        assert abs(cal.apply(0) - 10.0) < 1e-9
        assert abs(cal.apply(10) - 30.0) < 1e-9

    def test_apply_extrapolates_linearly(self):
        pts = _points([(10, 0), (30, 10)])
        cal = LinearCalibration.fit("od2000", pts)
        assert abs(cal.apply(20) - 50.0) < 1e-9

    def test_apply_manual_construction(self):
        """apply() should work on a hand-built calibration too, not just
        one produced by fit()."""
        cal = LinearCalibration(device="od2000", slope=1.05, intercept=-3.2, r_squared=1.0)
        assert abs(cal.apply(100) - (1.05 * 100 - 3.2)) < 1e-9


class TestResiduals:
    def test_residuals_zero_for_exact_fit(self):
        pts = _points([(10, 0), (30, 10), (50, 20)])
        cal = LinearCalibration.fit("od2000", pts)
        for r in cal.residuals_mm():
            assert abs(r) < 1e-9

    def test_residuals_length_matches_points(self):
        pts = _points([(100, 0), (200, 10), (295, 20), (410, 30)])
        cal = LinearCalibration.fit("od2000", pts)
        assert len(cal.residuals_mm()) == len(pts)


class TestCsvRoundtrip:
    def test_roundtrip_preserves_fit_params(self, tmp_path):
        pts = _points([(100, 0), (200, 10), (295, 20), (410, 30)])
        cal = LinearCalibration.fit("od2000", pts)
        path = tmp_path / "cal.csv"
        cal.to_csv(path)
        loaded = LinearCalibration.from_csv(path)

        assert loaded.device == cal.device
        assert abs(loaded.slope - cal.slope) < 1e-9
        assert abs(loaded.intercept - cal.intercept) < 1e-9
        assert abs(loaded.r_squared - cal.r_squared) < 1e-9
        assert loaded.created_at == cal.created_at

    def test_roundtrip_preserves_points(self, tmp_path):
        pts = _points([(100, 0), (200, 10), (295, 20)])
        cal = LinearCalibration.fit("wtt12l_powerprox", pts)
        path = tmp_path / "cal.csv"
        cal.to_csv(path)
        loaded = LinearCalibration.from_csv(path)

        assert len(loaded.points) == len(pts)
        for orig, rt in zip(cal.points, loaded.points):
            assert abs(orig.known_height_mm - rt.known_height_mm) < 1e-9
            assert abs(orig.raw_value - rt.raw_value) < 1e-9

    def test_roundtrip_apply_matches(self, tmp_path):
        """The whole point: a loaded calibration should transform readings
        identically to the original."""
        pts = _points([(100, 0), (200, 10), (295, 20), (410, 30)])
        cal = LinearCalibration.fit("od2000", pts)
        path = tmp_path / "cal.csv"
        cal.to_csv(path)
        loaded = LinearCalibration.from_csv(path)

        for raw in [0, 5, 15, 25, 35, -10, 100]:
            assert abs(loaded.apply(raw) - cal.apply(raw)) < 1e-9

    def test_from_csv_rejects_non_calibration_file(self, tmp_path):
        path = tmp_path / "not_a_calibration.csv"
        path.write_text("foo,bar\n1,2\n")
        with pytest.raises(ValueError):
            LinearCalibration.from_csv(path)

    def test_to_csv_accepts_str_path(self, tmp_path):
        pts = _points([(10, 0), (30, 10)])
        cal = LinearCalibration.fit("od2000", pts)
        path_str = str(tmp_path / "cal.csv")
        cal.to_csv(path_str)  # should not raise
        loaded = LinearCalibration.from_csv(path_str)
        assert abs(loaded.slope - cal.slope) < 1e-9

    def test_negative_and_float_values_survive_roundtrip(self, tmp_path):
        """repr() round-tripping should preserve full float precision,
        including negative raw values (e.g. current_ma readings near a
        sensor's low end could plausibly read slightly negative noise)."""
        pts = _points([(-5.5, -0.001), (123.456789, 17.333333)])
        cal = LinearCalibration.fit("wtt12l_powerprox", pts)
        path = tmp_path / "cal.csv"
        cal.to_csv(path)
        loaded = LinearCalibration.from_csv(path)
        assert math.isclose(loaded.slope, cal.slope, rel_tol=1e-12)
        assert math.isclose(loaded.intercept, cal.intercept, rel_tol=1e-12)
