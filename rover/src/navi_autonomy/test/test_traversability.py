"""The traversability maths on synthetic grids, holes included - spec section 9
rung 1. Pure numpy, no ROS.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_traversability.py -q'
"""
import math

import numpy as np
import pytest

from navi_autonomy.traversability import (
    ROUGHNESS_REF_M, SLOPE_LETHAL_DEG, SLOPE_LETHAL_RAD, STEP_LETHAL_M, derive,
    roughness_layer, slope_layer, step_layer, valid_layer)


def pit(depth=0.2, size=6, extent=24):
    """Flat ground at z = 0 with a `size` x `size` pit `depth` deep in the
    middle. The ZED sees into it, so its floor is measured, not NaN - which is
    the realistic case for a 0.2 m pit at a few metres."""
    grid = np.zeros((extent, extent), dtype=np.float32)
    lo = (extent - size) // 2
    grid[lo:lo + size, lo:lo + size] = -depth
    return grid, lo, size


def plane(degrees, extent=20, resolution=0.05):
    xs = np.arange(extent, dtype=np.float32) * resolution
    return np.tile((xs * math.tan(math.radians(degrees))).astype(np.float32), (extent, 1))


def test_the_thresholds_are_the_spec_numbers():
    assert STEP_LETHAL_M == 0.14
    assert SLOPE_LETHAL_DEG == 25.0
    assert SLOPE_LETHAL_RAD == pytest.approx(math.radians(25.0))


# -- the acceptance criterion of this sub-project ------------------------

def test_a_pit_makes_lethal_step_on_the_flat_ground_around_its_rim():
    """The point of SP7. The obstacle voxels are positive-only, so a hole is
    invisible to them; a max *absolute* neighbour difference makes the flat
    cells beside a 0.2 m pit lethal, because the ground beside them drops
    away. A positive-only kernel scores those same cells exactly zero."""
    grid, lo, size = pit()
    step = step_layer(grid)

    # A flat cell diagonally off the pit's corner, and one along its edge.
    assert step[lo - 1, lo - 1] == pytest.approx(0.2)
    assert step[lo - 1, lo + 2] == pytest.approx(0.2)
    assert step[lo - 1, lo - 1] > STEP_LETHAL_M
    # A positive-only kernel would see nothing there:
    rise = np.zeros_like(grid)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted = np.roll(np.roll(grid, dy, axis=0), dx, axis=1)
            rise = np.maximum(rise, shifted - grid)
    assert rise[lo - 1, lo - 1] == pytest.approx(0.0)


def test_the_pit_floor_is_lethal_at_its_edge_and_free_in_its_middle():
    grid, lo, size = pit(size=6)
    step = step_layer(grid)
    assert step[lo, lo] == pytest.approx(0.2)                 # floor, against the wall
    assert step[lo + 2, lo + 2] == pytest.approx(0.0)         # floor, interior
    assert step[2, 2] == pytest.approx(0.0)                   # far flat ground


def test_a_positive_step_is_lethal_too():
    """A rock, for symmetry with the pit: the ground *around* it and its own
    edge are lethal, the flat top of it is not."""
    grid = np.zeros((20, 20), dtype=np.float32)
    grid[9:12, 9:12] = 0.3
    step = step_layer(grid)
    assert step[8, 8] == pytest.approx(0.3)      # ground beside the rock
    assert step[9, 9] == pytest.approx(0.3)      # the rock's own edge
    assert step[10, 10] == pytest.approx(0.0)    # the middle of its flat top
    assert step[8, 8] > STEP_LETHAL_M


def test_a_step_just_under_the_threshold_is_not_lethal():
    grid = np.zeros((20, 20), dtype=np.float32)
    grid[10:, :] = -0.13
    step = step_layer(grid)
    assert step[9, 5] == pytest.approx(0.13)
    assert step[9, 5] < STEP_LETHAL_M


def test_step_ignores_unseen_neighbours_and_is_nan_where_unseen():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    step = step_layer(grid)
    assert not np.isfinite(step[5, 5])
    assert step[4, 5] == pytest.approx(0.0)      # the NaN neighbour is ignored


def test_step_does_not_wrap_around_the_grid_edge():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[0, :] = 1.0
    step = step_layer(grid)
    assert step[9, 5] == pytest.approx(0.0)      # row 9 must not see row 0


# -- slope ---------------------------------------------------------------

def test_slope_of_a_plane_is_its_inclination_everywhere_including_the_edge():
    grid = plane(20.0)
    slope = np.degrees(slope_layer(grid))
    assert slope[10, 10] == pytest.approx(20.0, abs=1e-3)
    assert slope[10, 0] == pytest.approx(20.0, abs=1e-3)     # one-sided difference
    assert slope[10, 19] == pytest.approx(20.0, abs=1e-3)


def test_slope_of_flat_ground_is_zero():
    assert np.degrees(slope_layer(np.zeros((10, 10), dtype=np.float32)))[5, 5] \
        == pytest.approx(0.0)


def test_a_30_degree_plane_is_over_the_slope_threshold_and_25_is_not_under_it():
    assert slope_layer(plane(30.0))[10, 10] > SLOPE_LETHAL_RAD
    assert slope_layer(plane(20.0))[10, 10] < SLOPE_LETHAL_RAD


# -- roughness -----------------------------------------------------------

def test_roughness_of_any_plane_is_zero_so_it_is_not_a_second_slope():
    for degrees in (0.0, 10.0, 20.0, 35.0):
        assert roughness_layer(plane(degrees))[10, 10] == pytest.approx(0.0, abs=1e-6)


def test_roughness_measures_a_bump():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = 0.04
    assert roughness_layer(grid)[5, 5] == pytest.approx(0.04)
    assert ROUGHNESS_REF_M == 0.05


# -- valid ---------------------------------------------------------------

def test_valid_needs_the_cell_and_its_four_axial_neighbours():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    valid = valid_layer(grid)
    assert valid[5, 5] == 0.0
    assert valid[4, 5] == 0.0        # a NaN neighbour
    assert valid[3, 3] == 1.0
    assert valid[0, 3] == 0.0        # the grid edge has no south neighbour


def test_derive_returns_the_four_layers_at_the_input_shape():
    grid, _, _ = pit()
    layers = derive(grid)
    assert set(layers) == {'slope', 'step', 'roughness', 'valid'}
    for name, array in layers.items():
        assert array.shape == grid.shape, name
        assert array.dtype == np.float32, name
