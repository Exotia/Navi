"""The grid arithmetic, with no ROS and no camera anywhere near it.

Run on this laptop with:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization python3 -m pytest \
    rover/src/navi_localization/test/test_elevation_grid.py -q'
and on the Orin as part of ./deploy_rover.sh --test.
"""

import numpy as np
import pytest

from navi_localization.elevation_grid import (
    MAX_CELLS, RESOLUTION, ElevationGrid, finite_points)


def test_a_single_point_makes_a_single_cell_holding_its_height():
    grid = ElevationGrid()
    grid.update([(0.05, 0.05, 1.25)])

    snapshot = grid.snapshot()
    assert snapshot.elevation.shape == (1, 1)
    assert snapshot.elevation[0, 0] == pytest.approx(1.25)
    # The cell spans [0.0, 0.1) in both axes, so its centre - and the
    # one-cell grid's centre - is at (0.05, 0.05).
    assert snapshot.center_x == pytest.approx(0.05)
    assert snapshot.center_y == pytest.approx(0.05)
    assert snapshot.resolution == pytest.approx(RESOLUTION)


def test_a_cell_holds_the_mean_of_the_points_in_it():
    grid = ElevationGrid()
    grid.update([(0.01, 0.01, 1.0), (0.09, 0.09, 2.0), (0.05, 0.05, 3.0)])

    assert grid.snapshot().elevation[0, 0] == pytest.approx(2.0)


def test_the_grid_grows_to_cover_new_points_and_leaves_the_gap_empty():
    grid = ElevationGrid()
    grid.update([(0.05, 0.05, 1.0), (0.25, 0.05, 3.0)])

    elevation = grid.snapshot().elevation
    assert elevation.shape == (1, 3)
    assert elevation[0, 0] == pytest.approx(1.0)
    assert np.isnan(elevation[0, 1])          # nothing was seen in this cell
    assert elevation[0, 2] == pytest.approx(3.0)


def test_rows_run_along_y_and_columns_along_x():
    grid = ElevationGrid()
    grid.update([(0.05, 0.05, 1.0), (0.05, 0.25, 2.0)])

    elevation = grid.snapshot().elevation
    assert elevation.shape == (3, 1)
    assert elevation[0, 0] == pytest.approx(1.0)   # row 0 is the smallest y
    assert elevation[2, 0] == pytest.approx(2.0)


def test_the_wrappers_padding_zeros_and_nans_are_thrown_away():
    # Both are real: the fused cloud is published with is_dense=false, and
    # the chunks the SDK did not update arrive as exact zeros.
    kept = finite_points([(0.0, 0.0, 0.0), (np.nan, 1.0, 1.0),
                          (1.0, np.inf, 1.0), (0.05, 0.05, 1.0)])

    assert kept.shape == (1, 3)
    assert kept[0, 2] == pytest.approx(1.0)


def test_a_later_cloud_replaces_the_cells_it_covers_and_keeps_the_rest():
    grid = ElevationGrid()
    grid.update([(0.05, 0.05, 1.0), (0.25, 0.05, 5.0)])
    grid.update([(0.05, 0.05, 4.0)])

    elevation = grid.snapshot().elevation
    # Replaced, not averaged with the old value: the fused cloud is
    # cumulative, so accumulating across messages counts points twice.
    assert elevation[0, 0] == pytest.approx(4.0)
    assert elevation[0, 2] == pytest.approx(5.0)


def test_the_grid_never_grows_past_sixty_metres():
    grid = ElevationGrid()
    grid.update([(0.05, 0.05, 1.0)])
    grid.update([(100.0, 0.05, 1.0)])

    snapshot = grid.snapshot()
    assert snapshot.elevation.shape[1] <= MAX_CELLS
    assert grid.points_outside_cap == 1


def test_an_unchanged_grid_compares_equal_so_it_is_not_republished():
    grid = ElevationGrid()
    grid.update([(0.05, 0.05, 1.0), (0.25, 0.05, 2.0)])
    first = grid.snapshot()
    grid.update([(0.05, 0.05, 1.0)])
    second = grid.snapshot()

    assert second.equals(first)          # NaN in the middle cell included

    grid.update([(0.05, 0.05, 9.0)])
    assert not grid.snapshot().equals(first)


def test_an_empty_grid_has_no_snapshot():
    assert ElevationGrid().snapshot() is None
