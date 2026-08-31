"""The fixture is generated, so its determinism is part of the contract."""

import hashlib

import numpy as np
import pytest

from navi_autonomy.traversability import LETHAL, UNKNOWN
from navi_autonomy.window import WINDOW_CELLS
from navi_localization.elevation_grid import RESOLUTION
from navi_nav2 import fixture


def test_the_window_is_sp7s_window():
    elevation = fixture.elevation()
    assert elevation.shape == (WINDOW_CELLS, WINDOW_CELLS)
    assert elevation.dtype == np.float32


def test_the_generator_is_deterministic_in_this_process():
    """Two calls, one terrain.  This half is a bit-exact pin and safe as
    one: it never crosses an architecture."""
    first = hashlib.sha256(fixture.elevation().tobytes()).hexdigest()
    second = hashlib.sha256(fixture.elevation().tobytes()).hexdigest()
    assert first == second


def test_it_is_the_same_fixture_on_every_machine():
    """The same terrain on the laptop, on the Orin and in six months -
    pinned by the structure of the int8 seed, which is what the planner
    actually consumes.

    NOT by a hash of the float32 elevation: numpy's sin/cos are
    SIMD-dispatched and are not bit-identical between amd64 (AVX2/AVX512)
    and arm64 (NEON/SVE), and a 1-ULP float64 difference survives the
    float32 cast whenever it straddles a rounding tie.  deploy_rover.sh
    --test runs this file on the Orin, so a float hash would fail there for
    a fixture that is, physically, the same terrain.
    """
    cost = fixture.seed()
    lethal = int((cost == LETHAL).sum())
    assert lethal == pytest.approx(fixture.LETHAL_CELLS,
                                   rel=fixture.LETHAL_CELLS_REL_TOLERANCE), \
        f"the fixture's lethal set moved: {lethal} cells"
    assert fixture.cell_of(*fixture.PIT_CENTRE) == fixture.PIT_CELL
    elevation = fixture.elevation()
    tolerance = fixture.ELEVATION_EXTREME_TOLERANCE_M
    assert float(elevation.min()) == pytest.approx(fixture.ELEVATION_MIN_M,
                                                   abs=tolerance)
    assert float(elevation.max()) == pytest.approx(fixture.ELEVATION_MAX_M,
                                                   abs=tolerance)


def test_the_pit_puts_lethal_cells_on_its_rim():
    """SP7's whole point: a hole is invisible to positive-only voxels, and
    this is the terrain that proves the seed carries it."""
    cost = fixture.seed()
    ix, iy = fixture.cell_of(*fixture.PIT_CENTRE)
    ring = cost[iy - 30:iy + 30, ix - 30:ix + 30]
    assert (ring == LETHAL).any(), "the pit rim must be lethal"


def test_the_pit_blocks_the_straight_line_from_start_to_goal():
    cost = fixture.seed()
    x0, y0 = fixture.START
    x1, y1 = fixture.GOAL
    blocked = False
    for t in np.linspace(0.0, 1.0, 400):
        ix, iy = fixture.cell_of(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        if cost[iy, ix] == LETHAL:
            blocked = True
    assert blocked, "a planner that ignored the seed must not be able to pass"


def test_the_start_and_the_goal_are_on_clear_ground():
    cost = fixture.seed()
    for x, y in (fixture.START, fixture.GOAL):
        assert fixture.clearance_cells(cost, x, y, radius_m=0.85) == 0, \
            "a start or goal inside the inflation makes the test meaningless"


def test_nothing_inside_the_window_is_unknown():
    """track_unknown_space is true on both costmaps, so an unknown cell in
    the middle of the fixture would make the problem unsolvable rather than
    hard, and the two failures look identical.

    The outermost ring is unknown by construction and stays that way:
    traversability._padded pads with NaN, so valid_layer is 0 there - the
    frontier of a mapped area really is unknown, and the fixture must not
    pretend otherwise.  Nothing plans within 20 m of the window edge."""
    interior = fixture.seed()[1:-1, 1:-1]
    assert (interior == UNKNOWN).sum() == 0


def test_the_occupancy_grid_says_where_it_starts():
    grid = fixture.occupancy_grid(stamp=None)
    assert grid.header.frame_id == 'map'
    assert grid.info.resolution == pytest.approx(RESOLUTION)
    assert grid.info.width == grid.info.height == WINDOW_CELLS
    # info.origin is the corner of cell (0, 0): a 48 m window centred on the
    # map origin starts at (-24, -24).
    assert grid.info.origin.position.x == pytest.approx(-24.0)
    assert grid.info.origin.position.y == pytest.approx(-24.0)


@pytest.mark.parametrize('x', [6.0, 9.0])
def test_the_north_corridor_is_wide_enough_for_the_rover(x):
    """A path has to exist, or a failing planner and an impossible problem
    look the same.  Checked where the corridor is narrowest: beside the pit
    (x = 6, squeezed between the rim and the northern boulder) and at the
    wall's north end (x = 9)."""
    cost = fixture.seed()
    clear = [y for y in np.arange(1.0, 9.0, 0.05)
             if fixture.clearance_cells(cost, x, float(y), radius_m=0.85) == 0]
    assert len(clear) * 0.05 >= 1.8, \
        f"corridor at x={x} is narrower than the rover plus margin"
