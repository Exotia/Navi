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
    LETHAL, MAX_SCALED_COST, ROUGHNESS_REF_M, SLOPE_LETHAL_DEG, SLOPE_LETHAL_RAD,
    STEP_LETHAL_M, UNKNOWN, clear_startup_patch, costmap_seed, derive, roughness_layer,
    seed_from_elevation, slope_layer, stamp_wheel_trail, step_layer, valid_layer)


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


# -- the cost curve --------------------------------------------------------

def test_the_seed_values_are_the_occupancy_grid_conventions():
    assert (LETHAL, UNKNOWN, MAX_SCALED_COST) == (100, -1, 99)


def test_a_pit_rim_is_lethal_in_the_seed():
    """End of the chain the whole sub-project exists for."""
    grid, lo, size = pit()
    layers, cost = seed_from_elevation(grid)
    assert cost.dtype == np.int8
    assert cost[lo - 1, lo - 1] == LETHAL
    assert cost[lo - 1, lo + 2] == LETHAL
    assert cost[lo, lo] == LETHAL                 # the floor against the wall
    assert cost[2, 2] == 0                        # far flat ground is free
    assert (cost == LETHAL).sum() == 48            # the 6x6 pit's two rings of rim


def test_never_seen_ground_is_unknown_not_free():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    _, cost = seed_from_elevation(grid)
    assert cost[5, 5] == UNKNOWN
    assert cost[4, 5] == UNKNOWN                  # incomplete neighbourhood
    assert cost[0, 3] == UNKNOWN                  # the grid edge
    assert cost[3, 3] == 0


def test_a_lethal_step_beats_an_incomplete_neighbourhood():
    """A cell can be short of support and still have a measured, lethal drop
    beside it. Lethal wins: that is the safe direction, and it is exactly the
    case at the frontier of a hole."""
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = -0.3
    grid[5, 7] = np.nan            # takes valid away from (5, 6) and (5, 7)
    _, cost = seed_from_elevation(grid)
    assert valid_layer(grid)[5, 6] == 0.0
    assert cost[5, 6] == LETHAL


def test_slope_scales_below_the_threshold_and_is_lethal_above_it():
    _, cost_20 = seed_from_elevation(plane(20.0))
    _, cost_30 = seed_from_elevation(plane(30.0))
    assert cost_20[10, 10] == 79           # round(99 * 20/25)
    assert cost_30[10, 10] == LETHAL


def test_the_scaled_band_takes_the_worst_indicator_not_their_average():
    zeros = np.zeros((4, 4), dtype=np.float32)
    ones = np.ones((4, 4), dtype=np.float32)
    cost = costmap_seed(slope=zeros, step=np.full((4, 4), 0.07, dtype=np.float32),
                        roughness=zeros, valid=ones)
    assert cost[1, 1] == 50                # round(99 * 0.5), not a third of it


def test_flat_seen_ground_costs_nothing():
    cost = costmap_seed(slope=np.zeros((4, 4), dtype=np.float32),
                        step=np.zeros((4, 4), dtype=np.float32),
                        roughness=np.zeros((4, 4), dtype=np.float32),
                        valid=np.ones((4, 4), dtype=np.float32))
    assert (cost == 0).all()


def test_the_seed_never_emits_a_value_outside_the_occupancy_grid_range():
    rng = np.random.default_rng(7)
    grid = rng.normal(0.0, 0.3, (60, 60)).astype(np.float32)
    grid[rng.random((60, 60)) < 0.2] = np.nan
    _, cost = seed_from_elevation(grid)
    assert cost.min() >= -1 and cost.max() <= 100
    assert set(np.unique(cost)) <= set(range(-1, 101))


# -- "the wheels have been here" (one startup patch) -----------------------

def test_the_startup_patch_clears_a_disc_of_the_right_radius_leaving_the_rest_unknown():
    cost = np.full((21, 21), UNKNOWN, dtype=np.int8)
    clear_startup_patch(cost, (10, 10), radius_cells=3)
    # the centre and its axial neighbours at the radius are cleared
    assert cost[10, 10] == 0
    assert cost[7, 10] == 0
    assert cost[10, 13] == 0
    # a corner outside the Euclidean disc, at (3, 3) offset, is not
    assert cost[7, 7] == UNKNOWN
    # far away, still unknown
    assert cost[0, 0] == UNKNOWN
    # exactly the disc's cell count for radius 3 (a diamond-ish 29-cell disc)
    expected = int(((np.mgrid[-3:4, -3:4][0] ** 2 + np.mgrid[-3:4, -3:4][1] ** 2) <= 9).sum())
    assert (cost == 0).sum() == expected


def test_a_measured_lethal_cell_inside_the_disc_beats_the_patch():
    cost = np.full((11, 11), UNKNOWN, dtype=np.int8)
    cost[5, 6] = LETHAL           # a camera-seen obstacle right beside the centre
    clear_startup_patch(cost, (5, 5), radius_cells=3)
    assert cost[5, 6] == LETHAL   # untouched
    assert cost[5, 4] == 0        # unknown neighbour still cleared


def test_a_measured_free_or_scaled_cell_inside_the_disc_is_unchanged():
    cost = np.full((11, 11), UNKNOWN, dtype=np.int8)
    cost[5, 6] = 0                # already measured free
    cost[5, 4] = 42               # a measured, scaled cost
    clear_startup_patch(cost, (5, 5), radius_cells=3)
    assert cost[5, 6] == 0
    assert cost[5, 4] == 42


def test_no_centre_clears_nothing():
    cost = np.full((5, 5), UNKNOWN, dtype=np.int8)
    result = clear_startup_patch(cost, None, radius_cells=3)
    assert (result == UNKNOWN).all()


def test_a_centre_off_the_grid_clears_nothing_in_bounds():
    cost = np.full((5, 5), UNKNOWN, dtype=np.int8)
    clear_startup_patch(cost, (100, 100), radius_cells=2)
    assert (cost == UNKNOWN).all()


# -- the wheel trail --------------------------------------------------------

def test_the_wheel_trail_overrides_even_a_measured_lethal():
    # The night finding this exists for: z-drift phantoms painted the
    # rover's own driven path lethal and stranded it. Wheels outrank the
    # camera - unknown, scaled cost and LETHAL alike go free on the trail.
    cost = np.full((20, 20), -1, dtype=np.int8)
    cost[10, 10] = 100
    cost[10, 12] = 99
    stamp_wheel_trail(cost, [(10, 10), (10, 12)], radius_cells=1)
    assert cost[10, 10] == 0
    assert cost[10, 12] == 0
    assert cost[9, 10] == 0 and cost[11, 12] == 0     # the disc, not a point
    assert cost[10, 15] == -1                          # off-trail untouched


def test_the_trail_disc_is_a_disc_and_stays_in_bounds():
    cost = np.full((6, 6), 100, dtype=np.int8)
    stamp_wheel_trail(cost, [(0, 0)], radius_cells=2)   # corner: clips safely
    assert cost[0, 0] == 0 and cost[2, 0] == 0
    assert cost[2, 2] == 100        # corner of the square, outside the disc
    assert cost[5, 5] == 100


def test_a_zero_radius_disables_the_trail():
    cost = np.full((4, 4), 100, dtype=np.int8)
    stamp_wheel_trail(cost, [(2, 2)], radius_cells=0)
    assert (cost == 100).all()


# -- floating cells ---------------------------------------------------------

def test_a_floating_blob_with_no_floor_connection_is_dropped_as_unseen():
    from navi_autonomy.traversability import mask_floating_cells
    elevation = np.zeros((15, 15), dtype=np.float32)
    elevation[7, 7] = 0.9          # a blob in mid-air over flat ground
    elevation[7, 8] = 0.8
    masked = mask_floating_cells(elevation, gap_m=0.35)
    assert np.isnan(masked[7, 7]) and np.isnan(masked[7, 8])
    assert masked[7, 6] == 0.0     # the floor around it is untouched
    # And through the seed: the blob becomes unknown, not a lethal wall.
    _, cost = seed_from_elevation(elevation, floating_gap_m=0.35)
    assert cost[7, 7] == UNKNOWN
    assert not (cost[6:9, 5:10] == LETHAL).any()


def test_a_real_wall_face_connects_to_the_floor_and_survives():
    from navi_autonomy.traversability import mask_floating_cells
    elevation = np.zeros((15, 15), dtype=np.float32)
    elevation[:, 8:] = 0.9         # a wall: every high cell neighbours high cells
    masked = mask_floating_cells(elevation, gap_m=0.35)
    # The wall's interior and its edge keep their measured height - their
    # neighbour median is high too, so nothing "floats".
    assert masked[7, 9] == np.float32(0.9)
    assert masked[7, 10] == np.float32(0.9)
    _, cost = seed_from_elevation(elevation, floating_gap_m=0.35)
    assert (cost[7, 7:10] == LETHAL).any()   # the step onto the wall stays lethal


def test_a_zero_gap_disables_the_floating_filter():
    from navi_autonomy.traversability import mask_floating_cells
    elevation = np.zeros((9, 9), dtype=np.float32)
    elevation[4, 4] = 2.0
    masked = mask_floating_cells(elevation, gap_m=0.0)
    assert masked[4, 4] == np.float32(2.0)
