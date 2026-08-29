"""Tiles, halo, dirty tracking and scheduling - pure numpy, no ROS.
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest \
    rover/src/navi_localization/test/test_tiles.py -q'
"""
import numpy as np
import pytest

from navi_localization.elevation_grid import GridSnapshot
from navi_localization.tiles import (
    DIRTY_THRESHOLD_M, MAX_DIRTY_PER_TICK, TILE_CELLS, TILE_SAMPLES, TileScheduler,
    tile_center, tile_index_of, tiles_of_snapshot)


def snapshot(elevation, origin_ix=0, origin_iy=0, resolution=0.05):
    elevation = np.asarray(elevation, dtype=np.float32)
    rows, cols = elevation.shape
    return GridSnapshot(elevation=elevation,
                        center_x=(origin_ix + cols / 2.0) * resolution,
                        center_y=(origin_iy + rows / 2.0) * resolution,
                        resolution=resolution, origin_ix=origin_ix, origin_iy=origin_iy)


def test_tile_centre_and_index_are_inverses():
    for ix, iy in [(0, 0), (3, -2), (-7, 11)]:
        assert tile_index_of(*tile_center(ix, iy)) == (ix, iy)
    assert tile_center(0, 0) == pytest.approx((1.275, 1.275))


def test_a_grid_inside_one_tile_gives_that_tile_with_nan_halo():
    grid = np.full((10, 10), 2.0)                  # cells ix 5..14, iy 5..14
    tiles = tiles_of_snapshot(snapshot(grid, origin_ix=5, origin_iy=5))

    assert list(tiles) == [(0, 0)]
    tile = tiles[(0, 0)]
    assert tile.shape == (TILE_SAMPLES, TILE_SAMPLES)
    assert tile[5:15, 5:15] == pytest.approx(2.0)
    assert np.isnan(tile[0, 0]) and np.isnan(tile[50, 50])


def test_the_halo_is_the_neighbours_first_row_and_column():
    # Two tiles side by side in x: cells 0..99 in x, one row in y.
    grid = np.arange(100, dtype=np.float32).reshape(1, 100)
    tiles = tiles_of_snapshot(snapshot(grid))

    assert set(tiles) == {(0, 0), (1, 0)}
    left = tiles[(0, 0)]
    assert left[0, 49] == 49.0
    assert left[0, 50] == 50.0            # halo column = tile (1, 0)'s first column
    right = tiles[(1, 0)]
    assert right[0, 0] == 50.0
    assert np.isnan(right[0, 50])          # nothing beyond the grid


def test_a_tile_that_is_only_touched_by_a_halo_is_not_a_tile():
    # Cells 0..49 fill tile 0 exactly; tile 1 would only ever get halo data
    # from tile 0's side, which is not its own ground.
    grid = np.zeros((1, 50), dtype=np.float32)
    assert list(tiles_of_snapshot(snapshot(grid))) == [(0, 0)]


def test_negative_indices_partition_the_same_way():
    grid = np.zeros((1, 3), dtype=np.float32)      # cells ix -2, -1, 0
    tiles = tiles_of_snapshot(snapshot(grid, origin_ix=-2))
    assert set(tiles) == {(-1, 0), (0, 0)}
    assert tiles[(-1, 0)][0, 48] == 0.0 and tiles[(-1, 0)][0, 49] == 0.0
    assert tiles[(0, 0)][0, 0] == 0.0


def flat(value):
    return {(0, 0): np.full((TILE_SAMPLES, TILE_SAMPLES), value, dtype=np.float32)}


def test_the_first_offer_is_due_at_once_and_stays_quiet_once_published():
    scheduler = TileScheduler()
    scheduler.offer(flat(1.0), now=0.0)
    due = scheduler.due(now=0.0)
    assert [key for key, _ in due] == [(0, 0)]
    scheduler.published((0, 0), due[0][1], now=0.0)
    scheduler.offer(flat(1.0), now=0.5)
    # Unchanged and published half a second ago: nothing, not even keepalive.
    assert scheduler.due(now=0.5) == []


def test_a_change_below_one_centimetre_is_not_dirty_but_above_is():
    scheduler = TileScheduler()
    scheduler.offer(flat(1.0), now=0.0)
    scheduler.published((0, 0), flat(1.0)[(0, 0)], now=0.0)
    scheduler.offer(flat(1.0 + DIRTY_THRESHOLD_M / 2), now=5.0)
    # Keepalive round-robin will still name it; dirty is what we test here.
    assert not scheduler.is_dirty((0, 0))
    scheduler.offer(flat(1.0 + 2 * DIRTY_THRESHOLD_M), now=5.0)
    assert scheduler.is_dirty((0, 0))


def test_a_newly_seen_cell_makes_the_tile_dirty():
    scheduler = TileScheduler()
    tile = flat(np.nan)[(0, 0)]
    tile[10, 10] = 1.0
    scheduler.offer({(0, 0): tile}, now=0.0)
    scheduler.published((0, 0), tile, now=0.0)
    changed = tile.copy()
    changed[11, 11] = 1.0
    scheduler.offer({(0, 0): changed}, now=3.0)
    assert scheduler.is_dirty((0, 0))


def test_a_dirty_tile_waits_a_second_between_publications():
    scheduler = TileScheduler()
    scheduler.offer(flat(1.0), now=0.0)
    scheduler.published((0, 0), flat(1.0)[(0, 0)], now=0.0)
    scheduler.offer(flat(2.0), now=0.4)
    assert scheduler.due(now=0.4) == []
    assert [k for k, _ in scheduler.due(now=1.0)] == [(0, 0)]


def test_at_most_eight_dirty_tiles_per_tick_oldest_first():
    scheduler = TileScheduler()
    tiles = {(i, 0): np.full((TILE_SAMPLES, TILE_SAMPLES), float(i), dtype=np.float32)
             for i in range(12)}
    scheduler.offer(tiles, now=0.0)
    first = [k for k, _ in scheduler.due(now=0.0)]
    assert len(first) == MAX_DIRTY_PER_TICK
    for key, tile in scheduler.due(now=0.0):
        scheduler.published(key, tile, now=0.0)
    rest = [k for k, _ in scheduler.due(now=1.0)]
    assert set(first) | set(rest) == set(tiles) and not set(first) & set(rest)


def test_one_clean_tile_per_tick_goes_out_as_keepalive_round_robin():
    scheduler = TileScheduler()
    tiles = {(i, 0): np.full((TILE_SAMPLES, TILE_SAMPLES), 1.0, dtype=np.float32)
             for i in range(3)}
    scheduler.offer(tiles, now=0.0)
    for key, tile in scheduler.due(now=0.0):
        scheduler.published(key, tile, now=0.0)
    seen = []
    for tick in range(1, 7):
        scheduler.offer(tiles, now=float(tick) + 1.0)
        due = scheduler.due(now=float(tick) + 1.0)
        assert len(due) == 1
        seen.append(due[0][0])
        scheduler.published(due[0][0], due[0][1], now=float(tick) + 1.0)
    assert seen == [(0, 0), (1, 0), (2, 0), (0, 0), (1, 0), (2, 0)]


def test_mark_all_dirty_republishes_everything_and_forget_all_names_the_dead():
    scheduler = TileScheduler()
    scheduler.offer(flat(1.0), now=0.0)
    scheduler.published((0, 0), flat(1.0)[(0, 0)], now=0.0)
    scheduler.mark_all_dirty()
    assert scheduler.is_dirty((0, 0))
    assert scheduler.forget_all() == [(0, 0)]
    assert scheduler.due(now=10.0) == []
