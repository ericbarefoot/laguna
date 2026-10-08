"""Finding the block on an uneven bed."""

import numpy as np
import pytest

from laguna.scanner.block_finder import find_block


def scene(bed_fn, block_xy=(300.0, 100.0), size=(196.85, 98.4), height=45.0, seed=0, strays=0):
    rng = np.random.default_rng(seed)
    x, y = np.meshgrid(np.arange(-300.0, 800.0, 3.0), np.arange(-250.0, 450.0, 3.0))
    x, y = x.ravel(), y.ravel()
    z = bed_fn(x, y) + rng.normal(0, 0.3, x.size)
    on = (np.abs(x - block_xy[0]) < size[0] / 2) & (np.abs(y - block_xy[1]) < size[1] / 2)
    z[on] += height
    if strays:
        idx = rng.choice(x.size, strays, replace=False)
        z[idx] += 60.0                         # scattered high returns all over the scan
    return np.c_[x, y, z]


FLAT = lambda x, y: np.full_like(x, 100.0)
TWO_PLATEAUS = lambda x, y: np.where(x > 450, 125.0, 102.0)     # a ledge 23 mm above the rest
TILTED = lambda x, y: 100.0 + 0.03 * x - 0.02 * y


class TestFindBlock:
    @pytest.mark.parametrize("bed", [FLAT, TWO_PLATEAUS, TILTED], ids=["flat", "two plateaus", "tilted"])
    def test_finds_the_block_on_any_of_these_beds(self, bed):
        b = find_block(scene(bed), expected_size_mm=(196.85, 98.4))
        np.testing.assert_allclose(b["center"], [300.0, 100.0], atol=4.0)
        np.testing.assert_allclose(b["size"], [196.85, 98.4], atol=8.0)
        assert b["top_z"] - b["bed_z"] == pytest.approx(45.0, abs=3.0)

    def test_a_ledge_higher_than_the_threshold_is_not_the_block(self):
        """The failure this module exists for: with the bed taken as a low global
        percentile, a ledge 23 mm up reads as 'block' across a third of the scan."""
        pts = scene(TWO_PLATEAUS, block_xy=(100.0, 100.0))
        b = find_block(pts, expected_size_mm=(196.85, 98.4), min_height_mm=20.0)
        assert b["size"][0] < 260 and b["size"][1] < 140

    def test_scattered_high_returns_do_not_inflate_the_footprint(self):
        b = find_block(scene(FLAT, strays=400), expected_size_mm=(196.85, 98.4))
        np.testing.assert_allclose(b["size"], [196.85, 98.4], atol=8.0)

    def test_orientation_of_the_expected_size_does_not_matter(self):
        b = find_block(scene(FLAT, size=(98.4, 196.85)), expected_size_mm=(196.85, 98.4))
        np.testing.assert_allclose(b["size"], [98.4, 196.85], atol=8.0)

    def test_the_blob_that_matches_the_expected_footprint_wins_over_a_bigger_one(self):
        pts = scene(FLAT)
        wall = (np.abs(pts[:, 0] - 650) < 40) & (np.abs(pts[:, 1] - 100) < 200)      # 80 x 400 mm raised strip
        pts[wall, 2] += 50
        b = find_block(pts, expected_size_mm=(196.85, 98.4))
        np.testing.assert_allclose(b["center"], [300.0, 100.0], atol=4.0)

    def test_without_an_expected_size_the_largest_blob_wins(self):
        pts = scene(FLAT)
        wall = (np.abs(pts[:, 0] - 650) < 40) & (np.abs(pts[:, 1] - 100) < 200)
        pts[wall, 2] += 50
        assert find_block(pts)["center"][0] == pytest.approx(650.0, abs=4.0)

    def test_no_block_returns_none(self):
        assert find_block(scene(FLAT, height=0.0), expected_size_mm=(196.85, 98.4)) is None

    def test_a_raised_feature_far_off_the_expected_size_is_rejected(self):
        pts = scene(FLAT, size=(20.0, 20.0))
        assert find_block(pts, expected_size_mm=(196.85, 98.4)) is None

    def test_a_roi_restricts_the_search_but_not_the_bed_estimate(self):
        pts = scene(TWO_PLATEAUS, block_xy=(300.0, 100.0))
        second = (np.abs(pts[:, 0] - 700) < 98) & (np.abs(pts[:, 1] - 100) < 49)
        pts[second, 2] += 45                                             # a decoy block, also a perfect match
        roi = (np.abs(pts[:, 0] - 300) < 150) & (np.abs(pts[:, 1] - 100) < 100)
        b = find_block(pts, expected_size_mm=(196.85, 98.4), roi_mask=roi)
        np.testing.assert_allclose(b["center"], [300.0, 100.0], atol=4.0)

    def test_nan_and_tiny_inputs_are_handled(self):
        pts = scene(FLAT)
        pts[::7, 2] = np.nan
        assert find_block(pts, expected_size_mm=(196.85, 98.4)) is not None
        assert find_block(pts[:50]) is None
        assert find_block(np.zeros((500, 2))) is None
