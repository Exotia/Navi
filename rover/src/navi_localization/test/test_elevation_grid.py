"""The grid arithmetic, with no ROS and no camera anywhere near it.

Run on this laptop with:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization python3 -m pytest \
    rover/src/navi_localization/test/test_elevation_grid.py -q'
and on the Orin as part of ./deploy_rover.sh --test.
"""

import dataclasses
import time

import numpy as np
import pytest

from navi_localization.elevation_grid import (
    MAX_CELLS, RESOLUTION, ElevationGrid, finite_points)


def test_a_single_point_makes_a_single_cell_holding_its_height():
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.25)])

    snapshot = grid.snapshot()
    assert snapshot.elevation.shape == (1, 1)
    assert snapshot.elevation[0, 0] == pytest.approx(1.25)
    # A single-point cell reports that point at both height and top.
    assert snapshot.top[0, 0] == pytest.approx(1.25)
    # The cell spans [0.0, 0.05) in both axes, so its centre - and the
    # one-cell grid's centre - is at (0.025, 0.025).
    assert snapshot.center_x == pytest.approx(0.025)
    assert snapshot.center_y == pytest.approx(0.025)
    assert snapshot.resolution == pytest.approx(RESOLUTION)


def test_a_cell_holds_the_20th_percentile_height_and_the_max_as_top():
    # A wall, a person or a flying pixel must not pull the drawn height up
    # towards it: four points on the ground and one spike at z=1.0 should
    # leave the height at the ground and put the spike only in `top`.
    grid = ElevationGrid()
    grid.update([(0.01, 0.01, 0.0), (0.02, 0.02, 0.0), (0.03, 0.03, 0.0),
                (0.04, 0.04, 0.0), (0.015, 0.015, 1.0)])

    snapshot = grid.snapshot()
    assert snapshot.elevation[0, 0] == pytest.approx(0.0)
    assert snapshot.top[0, 0] == pytest.approx(1.0)


def test_the_grid_grows_to_cover_new_points_and_leaves_the_gap_empty():
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 3.0)])

    elevation = grid.snapshot().elevation
    assert elevation.shape == (1, 3)
    assert elevation[0, 0] == pytest.approx(1.0)
    assert np.isnan(elevation[0, 1])          # nothing was seen in this cell
    assert elevation[0, 2] == pytest.approx(3.0)


def test_rows_run_along_y_and_columns_along_x():
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0), (0.02, 0.12, 2.0)])

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
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 5.0)])
    grid.update([(0.02, 0.02, 4.0)])

    elevation = grid.snapshot().elevation
    # Replaced, not averaged with the old value: the fused cloud is
    # cumulative, so accumulating across messages counts points twice.
    assert elevation[0, 0] == pytest.approx(4.0)
    assert elevation[0, 2] == pytest.approx(5.0)


def test_a_later_cloud_replaces_the_cells_percentile_not_just_a_single_point():
    grid = ElevationGrid()
    # First update: cell (0, 0) sees a spread of points; 20th percentile of
    # five ascending values [0, 1, 2, 3, 4] lands on index floor(0.2*4)=0.
    grid.update([(0.02, 0.02, 0.0), (0.02, 0.02, 1.0), (0.02, 0.02, 2.0),
                (0.02, 0.02, 3.0), (0.02, 0.02, 4.0)])
    first = grid.snapshot()
    assert first.elevation[0, 0] == pytest.approx(0.0)
    assert first.top[0, 0] == pytest.approx(4.0)

    # A later, unrelated cloud for the same cell must compute its own
    # percentile from only this update's points, not blend with the old one.
    grid.update([(0.02, 0.02, 10.0), (0.02, 0.02, 11.0)])
    second = grid.snapshot()
    assert second.elevation[0, 0] == pytest.approx(10.0)   # floor(0.2*1)=0 -> the lower of the two
    assert second.top[0, 0] == pytest.approx(11.0)


def test_growth_clips_to_the_cap_instead_of_freezing_short_of_it():
    # Regression: the growth branch used to abort - and leave the window
    # frozen at its old size forever - the instant a single batch's
    # required extent overshot max_cells by even one cell, even though
    # there was still room to grow up to the cap. It must clip to exactly
    # max_cells instead, keeping the cells already mapped.
    grid = ElevationGrid(max_cells=5)
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 2.0)])   # cols 0, 2 -> 3 cols
    assert grid.snapshot().elevation.shape == (1, 3)

    # This point's column index is 5, so the union with the existing
    # window (cols 0-2) needs 6 columns - one past the cap of 5.
    grid.update([(0.27, 0.02, 3.0)])

    snapshot = grid.snapshot()
    elevation = snapshot.elevation
    assert elevation.shape == (1, 5)              # clipped to the cap, not frozen at 3
    assert elevation[0, 0] == pytest.approx(1.0)   # existing cells kept
    assert elevation[0, 2] == pytest.approx(2.0)
    assert np.isnan(elevation[0, 1])
    # Column 5 still doesn't fit in a 5-wide window anchored at column 0
    # (columns 0-4), so this point is genuinely past the 60-cell budget.
    assert grid.points_outside_cap == 1

    # A later point at column 4 now falls inside the grown-to-cap window,
    # proving the window is not frozen: it must be binned, not counted
    # outside.
    grid.update([(0.22, 0.02, 4.0)])
    elevation = grid.snapshot().elevation
    assert elevation.shape == (1, 5)
    assert elevation[0, 4] == pytest.approx(4.0)
    assert grid.points_outside_cap == 1            # unchanged: this one fit


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


def test_the_resolution_is_five_centimetres_and_the_cap_sixty_metres():
    from navi_localization.elevation_grid import RESOLUTION, MAX_CELLS, MAX_EXTENT_M
    assert RESOLUTION == 0.05
    assert MAX_EXTENT_M == 60.0
    assert MAX_CELLS == 1200


def test_state_round_trips_through_replace():
    from navi_localization.elevation_grid import ElevationGrid
    grid = ElevationGrid()
    grid.update([[0.12, 0.31, 1.0], [0.12, 0.31, 3.0], [-0.4, 0.9, 2.0]])
    state = grid.state()

    other = ElevationGrid()
    other.replace(state)

    assert other.snapshot().equals(grid.snapshot())
    assert state.elevation.dtype == np.float32
    assert state.top.dtype == np.float32
    assert state.count.dtype == np.int32
    # The two-point cell's height is the 20th percentile of [1.0, 3.0]
    # (floor(0.2*1)=0 -> the lower one), its top the max, and both round
    # trip through replace() unchanged.
    two_point_cell = (state.count == 2)
    assert int(two_point_cell.sum()) == 1
    assert state.elevation[two_point_cell].item() == pytest.approx(1.0)
    assert state.top[two_point_cell].item() == pytest.approx(3.0)
    assert other.snapshot().top[two_point_cell].item() == pytest.approx(3.0)
    # Replacing reproduces the grid's exact internal state, not just its
    # visible height: the same later update(), on the original and on the
    # replayed copy, must land on the same value. (update() replaces a
    # touched cell's percentile rather than blending with the old one - see
    # test_a_later_cloud_replaces_the_cells_percentile_not_just_a_single_point -
    # so this cell's two points become this new single point, 5.0, on both
    # grids alike.)
    grid.update([[0.12, 0.31, 5.0]])
    other.update([[0.12, 0.31, 5.0]])
    assert other.snapshot().equals(grid.snapshot())
    assert other.snapshot().elevation[state.count.argmax() // state.count.shape[1],
                                      state.count.argmax() % state.count.shape[1]] \
        == pytest.approx(5.0)
    assert other.snapshot().top[two_point_cell].item() == pytest.approx(5.0)


def test_state_of_an_empty_grid_is_none_and_clear_empties_a_populated_grid():
    from navi_localization.elevation_grid import ElevationGrid
    grid = ElevationGrid()
    assert grid.state() is None
    grid.update([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0]])
    grid.clear()
    assert grid.snapshot() is None
    assert grid.points_outside_cap == 0


def test_the_snapshot_says_where_its_origin_is_on_the_lattice():
    from navi_localization.elevation_grid import ElevationGrid
    grid = ElevationGrid()
    grid.update([[0.52, -0.31, 1.0]])           # cell ix=10, iy=-7 at 0.05 m
    snapshot = grid.snapshot()
    assert (snapshot.origin_ix, snapshot.origin_iy) == (10, -7)


def test_replace_refuses_a_state_built_at_a_different_resolution():
    from navi_localization.elevation_grid import ElevationGrid, GridState
    grid = ElevationGrid()
    state = GridState(elevation=np.array([[1.0]], dtype=np.float32),
                       top=np.array([[1.0]], dtype=np.float32),
                       count=np.array([[1]], dtype=np.int32),
                       origin_ix=0, origin_iy=0, resolution=0.10)

    with pytest.raises(ValueError):
        grid.replace(state)


def test_a_200k_point_cloud_bins_in_under_a_hundred_milliseconds():
    # The spec's own budget: the percentile binning is vectorised (lexsort
    # + np.unique) precisely so a full-size fused cloud never costs a
    # Python loop over cells. A generous 400 ms ceiling absorbs laptop/CI
    # jitter around the true, much smaller, number - see the report for
    # the measured figure.
    rng = np.random.default_rng(0)
    n = 200_000
    xy = rng.uniform(-15.0, 15.0, size=(n, 2))
    z = rng.uniform(0.0, 2.0, size=n)
    points = np.column_stack([xy, z])

    grid = ElevationGrid()
    started = time.perf_counter()
    grid.update(points)
    elapsed = time.perf_counter() - started

    assert grid.snapshot() is not None
    assert elapsed < 0.4, f"binning 200k points took {elapsed * 1000:.1f} ms"


def test_height_at_reads_seen_gap_outside_and_negative_indices():
    grid = ElevationGrid()
    # Window spans ix -2..0 (iy=0 throughout): -2 seen, -1 a gap cell no
    # point ever touched, 0 seen - mirrors
    # test_the_grid_grows_to_cover_new_points_and_leaves_the_gap_empty, but
    # anchored so the window includes negative indices too.
    grid.update([(-0.09, 0.02, 1.0), (0.02, 0.02, 3.0)])
    snapshot = grid.snapshot()
    assert snapshot.elevation.shape[1] == 3            # -2, -1 (gap), 0

    origin_ix = snapshot.origin_ix
    left_ix, gap_ix, right_ix = origin_ix, origin_ix + 1, origin_ix + 2
    outside_ix = origin_ix + 100                        # nowhere near the window

    result = grid.height_at(
        np.array([left_ix, gap_ix, right_ix, outside_ix]), np.array([0, 0, 0, 0]))

    assert result[0] == pytest.approx(1.0)              # seen, negative index
    assert np.isnan(result[1])                          # inside the window, never seen
    assert result[2] == pytest.approx(3.0)              # seen, positive index
    assert np.isnan(result[3])                          # outside the window entirely


def test_height_at_before_any_point_is_all_nan():
    grid = ElevationGrid()
    result = grid.height_at(np.array([0, -5, 12]), np.array([0, 3, -7]))
    assert np.isnan(result).all()


def test_already_filtered_points_bin_the_same_as_the_default_path():
    # The obstacle voxeliser needs the exact same filtered array
    # elevation_mapper._on_cloud hands to both update() calls - this pins
    # already_filtered=True to produce the identical grid as filtering
    # inside update() itself, for the same raw cloud.
    points = [(0.02, 0.02, 1.0), (0.12, 0.02, 3.0),
              (np.nan, 1.0, 1.0), (0.0, 0.0, 0.0)]      # a NaN and the padding zero

    default_grid = ElevationGrid()
    default_grid.update(points)

    pre_filtered_grid = ElevationGrid()
    pre_filtered_grid.update(finite_points(points), already_filtered=True)

    assert pre_filtered_grid.snapshot().equals(default_grid.snapshot())


def test_replace_refuses_a_state_bigger_than_the_cap():
    from navi_localization.elevation_grid import ElevationGrid, GridState, MAX_CELLS, RESOLUTION
    grid = ElevationGrid()
    oversized_rows = MAX_CELLS + 1
    state = GridState(elevation=np.full((oversized_rows, 1), np.nan, dtype=np.float32),
                       top=np.full((oversized_rows, 1), np.nan, dtype=np.float32),
                       count=np.zeros((oversized_rows, 1), dtype=np.int32),
                       origin_ix=0, origin_iy=0, resolution=RESOLUTION)

    with pytest.raises(ValueError):
        grid.replace(state)


def test_fusing_a_cloud_stamps_exactly_the_cells_it_wrote_and_only_those():
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 2.0)], observed_at=100.0)

    stamp = grid.snapshot().stamp
    assert stamp[0, 0] == pytest.approx(100.0)
    assert stamp[0, 2] == pytest.approx(100.0)
    assert np.isnan(stamp[0, 1])                # the gap cell nothing ever touched

    # A later cloud that only rewrites one of those two cells leaves the
    # other cell's stamp exactly as it was.
    grid.update([(0.02, 0.02, 5.0)], observed_at=200.0)
    stamp = grid.snapshot().stamp
    assert stamp[0, 0] == pytest.approx(200.0)
    assert stamp[0, 2] == pytest.approx(100.0)


def test_update_without_an_observed_at_leaves_stamps_untouched():
    # A grid has no clock of its own - update() must not invent a time
    # when the caller does not supply one, rather than silently stamping
    # cells with something meaningless.
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0)])
    assert np.isnan(grid.snapshot().stamp[0, 0])


def test_equals_ignores_the_observation_stamp():
    # Mirrors the "top is not published" exclusion just below: a cell
    # re-fused to the *same* height still gets a fresh stamp, so if stamp
    # gated equality every single cloud would count as a change and the
    # "unchanged map is not republished" check this stands in for would
    # never fire again.
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0)], observed_at=1.0)
    first = grid.snapshot()
    grid.update([(0.02, 0.02, 1.0)], observed_at=999.0)
    second = grid.snapshot()
    assert second.equals(first)


def test_growth_keeps_stamps_attached_to_the_cells_they_describe():
    # Same shape as test_growth_clips_to_the_cap_instead_of_freezing_short_of_it:
    # the second update forces _fit_window's grow-and-copy branch, and the
    # stamp array has to move with height/top/count through it rather than
    # being left behind or scrambled.
    grid = ElevationGrid(max_cells=5)
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 2.0)], observed_at=10.0)   # cols 0, 2 -> 3 cols
    grid.update([(0.22, 0.02, 4.0)], observed_at=20.0)                      # col 4 -> grows to 5

    stamp = grid.snapshot().stamp
    assert stamp.shape == (1, 5)
    assert stamp[0, 0] == pytest.approx(10.0)    # carried across the grow, untouched
    assert stamp[0, 2] == pytest.approx(10.0)
    assert stamp[0, 4] == pytest.approx(20.0)    # freshly written by the second update
    assert np.isnan(stamp[0, 1])
    assert np.isnan(stamp[0, 3])                 # a gap the grow introduced, still never seen


def test_state_round_trips_the_observation_stamp():
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 2.0)], observed_at=42.0)
    state = grid.state()

    assert state.stamp.dtype == np.float64       # never float32 - see the module docstring
    assert state.stamp[0, 0] == pytest.approx(42.0)
    assert np.isnan(state.stamp[0, 1])

    other = ElevationGrid()
    other.replace(state)
    stamp = other.snapshot().stamp
    assert stamp[0, 0] == pytest.approx(42.0)
    assert stamp[0, 2] == pytest.approx(42.0)
    assert np.isnan(stamp[0, 1])


def test_replacing_a_state_with_no_stamp_field_marks_every_seen_cell_observed_now():
    # The stand-in for "loading an old .npz saved before stamps existed":
    # map_store.py builds a stamp-less GridState today for every load, so
    # this is the fallback path that actually runs in production right now.
    grid = ElevationGrid()
    grid.update([(0.02, 0.02, 1.0), (0.12, 0.02, 2.0)])
    old_style_state = dataclasses.replace(grid.state(), stamp=None)

    other = ElevationGrid()
    other.replace(old_style_state, now=999.0)

    stamp = other.snapshot().stamp
    assert stamp[0, 0] == pytest.approx(999.0)
    assert stamp[0, 2] == pytest.approx(999.0)
    assert np.isnan(stamp[0, 1])                 # never observed, still never observed


def test_replace_without_a_stamp_or_now_refuses_rather_than_guessing():
    grid = ElevationGrid()
    grid.update([(0.0, 0.0, 1.0)])
    state = dataclasses.replace(grid.state(), stamp=None)

    other = ElevationGrid()
    with pytest.raises(ValueError):
        other.replace(state)


def test_update_remembers_the_tiles_it_touched_including_the_halo():
    from navi_localization.elevation_grid import ElevationGrid
    grid = ElevationGrid()
    # Cell (52, 3) is inside tile (1, 0); cell (50, 3) is tile (1, 0)'s
    # first column, which tile (0, 0) publishes as its halo sample too.
    grid.update(np.array([[52 * 0.05 + 0.01, 3 * 0.05 + 0.01, 0.0],
                          [50 * 0.05 + 0.01, 3 * 0.05 + 0.01, 0.0]]))
    assert grid.take_touched_tiles() == {(1, 0), (0, 0)}
    assert grid.take_touched_tiles() == set()          # reset by the take
    grid.update(np.array([[-0.01, -0.01, 0.0]]))       # cell (-1, -1): tile (-1, -1)
    assert grid.take_touched_tiles() == {(-1, -1)}
