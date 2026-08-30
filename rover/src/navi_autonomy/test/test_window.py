"""The rolling window's geometry - pure numpy, no ROS.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_window.py -q'
"""
import numpy as np
import pytest

from navi_autonomy.window import (
    RECENTRE_MARGIN_CELLS, WINDOW_CELLS, RollingWindow, cell_index_of)


def tile(value=1.0):
    """A 51 x 51 tile whose own 50 x 50 is `value` and whose halo is NaN."""
    out = np.full((51, 51), np.nan, dtype=np.float32)
    out[:50, :50] = value
    return out


def test_cell_index_floors_towards_minus_infinity():
    assert cell_index_of(0.0) == 0
    assert cell_index_of(0.049) == 0
    assert cell_index_of(0.05) == 1
    assert cell_index_of(-0.01) == -1
    assert cell_index_of(-0.05) == -1


def test_a_fresh_window_is_all_unseen_and_centred_on_the_origin():
    w = RollingWindow()
    assert w.elevation.shape == (WINDOW_CELLS, WINDOW_CELLS)
    assert not np.isfinite(w.elevation).any()
    assert w.origin_ix == -WINDOW_CELLS // 2
    assert w.origin_iy == -WINDOW_CELLS // 2
    assert w.center == pytest.approx((0.0, 0.0))


def test_a_tile_lands_on_the_cells_it_owns():
    w = RollingWindow(cells=200)          # origin at lattice cell -100
    w.paste_tile(0, 0, tile(2.0))         # tile (0, 0) owns cells [0, 50)
    assert w.elevation[100:150, 100:150] == pytest.approx(2.0)
    assert not np.isfinite(w.elevation[99, 100])
    assert not np.isfinite(w.elevation[150, 100])
    assert not np.isfinite(w.elevation[100, 150])


def test_a_negative_tile_index_lands_on_negative_cells():
    w = RollingWindow(cells=200)
    w.paste_tile(-1, -2, tile(3.0))       # cells [-50, 0) in x, [-100, -50) in y
    assert w.elevation[0:50, 50:100] == pytest.approx(3.0)


def test_the_halo_row_and_column_are_ignored():
    """The 51st row and column are copies of the +x / +y neighbours' first
    cells, and those neighbours publish their own tiles. Merging a halo here
    would let a stale tile overwrite a fresher neighbour whenever the two
    arrive in the wrong order."""
    w = RollingWindow(cells=200)
    halo = tile(4.0)
    halo[:, 50] = 9.0                     # the +x neighbour's first column
    halo[50, :] = 9.0                     # the +y neighbour's first row
    w.paste_tile(0, 0, halo)
    assert np.allclose(w.elevation[100:150, 100:150], 4.0)
    assert not np.isfinite(w.elevation[100, 150])        # the +x neighbour's cell
    assert not np.isfinite(w.elevation[150, 100])        # the +y neighbour's cell
    w.paste_tile(1, 0, tile(5.0))                        # the neighbour's own message
    assert w.elevation[100, 150] == pytest.approx(5.0)
    w.paste_tile(0, 0, halo)                             # a stale (0, 0) again
    assert w.elevation[100, 150] == pytest.approx(5.0)   # and it stays the neighbour's


def test_an_all_nan_tile_blanks_the_tile_it_names():
    w = RollingWindow(cells=200)
    w.paste_tile(0, 0, tile(2.0))
    w.paste_tile(0, 0, np.full((51, 51), np.nan, dtype=np.float32))
    assert not np.isfinite(w.elevation[100:150, 100:150]).any()


def test_a_tile_outside_the_window_is_dropped_without_raising():
    w = RollingWindow(cells=200)
    w.paste_tile(40, 40, tile(1.0))       # cells [2000, 2050): nowhere near
    assert not np.isfinite(w.elevation).any()


def test_a_tile_straddling_the_edge_is_clipped():
    w = RollingWindow(cells=200)          # cells [-100, 100)
    w.paste_tile(1, 0, tile(7.0))         # cells [50, 100) in x - the last 50
    assert w.elevation[100:150, 150:200] == pytest.approx(7.0)
    w.paste_tile(2, 0, tile(8.0))         # cells [100, 150): entirely outside
    assert np.nanmax(w.elevation) == pytest.approx(7.0)
    assert np.nanmin(w.elevation) == pytest.approx(7.0)


def test_a_wrong_shaped_tile_raises():
    w = RollingWindow(cells=200)
    with pytest.raises(ValueError):
        w.paste_tile(0, 0, np.zeros((50, 50), dtype=np.float32))


def test_the_window_does_not_move_until_the_rover_is_far_from_its_centre():
    w = RollingWindow()
    before = (w.origin_ix, w.origin_iy)
    assert w.recentre(7.9, -7.9) is False
    assert (w.origin_ix, w.origin_iy) == before


def test_the_window_recentres_on_the_rover_and_carries_its_cells_along():
    w = RollingWindow()                        # 960 cells, origin -480
    w.paste_tile(0, 0, tile(6.0))              # lattice cells [0, 50)
    moved = w.recentre(10.0, 0.0)              # 200 cells: past the 160-cell margin
    assert moved is True
    assert w.origin_ix == cell_index_of(10.0) - WINDOW_CELLS // 2 == -280
    assert w.origin_iy == -WINDOW_CELLS // 2   # y never moved
    # The same ground, at its new place in the window.
    row, column = 0 - w.origin_iy, 0 - w.origin_ix
    assert w.elevation[row:row + 50, column:column + 50] == pytest.approx(6.0)
    assert RECENTRE_MARGIN_CELLS == 160


def test_cells_that_leave_the_window_are_gone():
    w = RollingWindow(cells=200)
    w.paste_tile(0, 0, tile(6.0))
    w.recentre(60.0, 0.0)                      # 1200 cells away: nothing survives
    assert not np.isfinite(w.elevation).any()


def test_snapshot_is_a_copy():
    w = RollingWindow(cells=200)
    w.paste_tile(0, 0, tile(1.0))
    snap = w.snapshot()
    w.paste_tile(0, 0, tile(2.0))
    assert snap[100, 100] == pytest.approx(1.0)
