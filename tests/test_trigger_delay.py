"""Calibrating the trigger delay of a gantry-coordinated scan."""

import numpy as np
import pytest

from laguna.scanner.trigger_delay import block_travel_offset, fit_trigger_delay, summarize_trigger_delay


def scene(centre_travel):
    x, y = np.meshgrid(np.arange(-150.0, 150.0, 3.0), np.arange(400.0, 900.0, 3.0))
    x, y, z = x.ravel(), y.ravel(), np.zeros(x.size)
    pts = np.c_[x, y, z]
    on = (np.abs(pts[:, 1] - centre_travel) < 98.4) & (np.abs(pts[:, 0]) < 49)
    pts[on, 2] += 45
    return pts


class TestFit:
    def test_one_speed_gives_the_delay_directly(self):
        r = fit_trigger_delay([100.0], [-13.3])
        assert r["delay_s"] == pytest.approx(0.0665)
        assert r["constant_mm"] == 0.0

    def test_several_speeds_recover_a_pure_time_delay(self):
        v = np.array([50.0, 100.0, 150.0])
        r = fit_trigger_delay(v, -2 * v * 0.066)
        assert r["delay_s"] == pytest.approx(0.066) and r["constant_mm"] == pytest.approx(0.0, abs=1e-9)
        assert r["rms_residual_mm"] == pytest.approx(0.0, abs=1e-9)

    def test_a_constant_offset_in_mm_is_separated_from_the_delay(self):
        v = np.array([50.0, 100.0, 200.0])
        r = fit_trigger_delay(v, -2 * v * 0.05 + 4.0)
        assert r["delay_s"] == pytest.approx(0.05) and r["constant_mm"] == pytest.approx(4.0)

    def test_a_late_trigger_gives_a_negative_delay(self):
        assert fit_trigger_delay([100.0], [+10.0])["delay_s"] == pytest.approx(-0.05)

    def test_noisy_measurements_still_land_near_the_truth(self):
        rng = np.random.default_rng(3)
        v = np.repeat([50.0, 100.0, 150.0], 4)
        r = fit_trigger_delay(v, -2 * v * 0.066 + rng.normal(0, 0.5, v.size))
        assert r["delay_s"] == pytest.approx(0.066, abs=0.006)

    @pytest.mark.parametrize("v, d", [([], []), ([1.0, 2.0], [1.0]), ([0.0], [1.0]), ([-5.0], [1.0])])
    def test_bad_input_is_rejected(self, v, d):
        with pytest.raises(ValueError):
            fit_trigger_delay(v, d)


class TestBlockTravelOffset:
    @pytest.mark.parametrize("travel_axis", [0, 1])
    def test_reverse_minus_forward_along_the_travel_axis(self, travel_axis):
        fwd, rev = scene(650.0), scene(650.0 - 13.0)
        if travel_axis == 0:                       # travel along X instead: swap the columns
            for p in (fwd, rev):
                p[:, [0, 1]] = p[:, [1, 0]]
        d = block_travel_offset(fwd, rev, travel_axis, expected_size_mm=(196.85, 98.4))
        assert d == pytest.approx(-13.0, abs=3.1)      # cell-size resolution of the finder

    def test_none_when_a_pass_has_no_block(self):
        flat = scene(650.0)
        flat[:, 2] = 0.0
        assert block_travel_offset(scene(650.0), flat, 1, expected_size_mm=(196.85, 98.4)) is None


class TestSummarize:
    def test_mean_and_scatter_of_repeats_at_one_speed(self):
        offsets = [-12.0, -13.0, -14.0, -13.0, -13.0]                  # at 100 mm/s
        r = summarize_trigger_delay(100.0, offsets)
        assert r["delay_s"] == pytest.approx(0.065) and r["n"] == 5
        assert r["std_s"] == pytest.approx(np.std([0.06, 0.065, 0.07, 0.065, 0.065], ddof=1))
        assert r["sem_s"] == pytest.approx(r["std_s"] / np.sqrt(5))
        assert r["ci95_s"] > r["sem_s"]                                  # Student's t, small n
        assert r["seam_sigma_mm"] == pytest.approx(2 * 100.0 * r["std_s"])
        assert r["mean_offset_mm"] == pytest.approx(-13.0)

    def test_applying_the_mean_delay_leaves_a_seam_of_that_sigma(self):
        offsets = np.array([-11.0, -14.0, -12.5, -15.0, -13.0])
        r = summarize_trigger_delay(100.0, offsets)
        residual = offsets + 2 * 100.0 * r["delay_s"]
        assert np.std(residual, ddof=1) == pytest.approx(r["seam_sigma_mm"])

    def test_a_single_measurement_has_no_scatter(self):
        r = summarize_trigger_delay(100.0, [-13.0])
        assert r["delay_s"] == pytest.approx(0.065) and np.isnan(r["std_s"]) and np.isnan(r["ci95_s"])

    def test_missing_blocks_are_ignored(self):
        assert summarize_trigger_delay(100.0, [-13.0, np.nan, -13.0])["n"] == 2

    @pytest.mark.parametrize("v, d", [(0.0, [1.0]), (-1.0, [1.0]), (100.0, []), (100.0, [np.nan])])
    def test_bad_input_is_rejected(self, v, d):
        with pytest.raises(ValueError):
            summarize_trigger_delay(v, d)
