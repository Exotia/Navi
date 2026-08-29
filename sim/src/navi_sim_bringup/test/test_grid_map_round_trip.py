"""The one test that stops the rover's map and the laptop's terrain drifting apart.

elevation_mapper writes grid_map's index convention (index (0, 0) at the
largest x and largest y, rows in -x, columns in -y, column-major data) and
terrain_writer reads it back. Both are in this repository but in different
workspaces, so nothing else forces them to agree; this does.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PWD/rover/src/navi_localization \
    python3 -m pytest sim/src/navi_sim_bringup/test/test_grid_map_round_trip.py -q'
"""

import pathlib
import sys

# navi_localization is the rover's package and is not installed in the sim
# workspace; the round trip needs its grid, so it is loaded from the source
# tree (this file lives at sim/src/navi_sim_bringup/test/).
_ROVER_PKG = pathlib.Path(__file__).resolve().parents[4] / "rover" / "src" / "navi_localization"
if str(_ROVER_PKG) not in sys.path:
    sys.path.insert(0, str(_ROVER_PKG))

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from navi_localization.elevation_mapper import build_tile_message
from navi_sim_bringup.terrain_writer import elevation_from_message, tile_index_of


def test_what_the_rover_publishes_is_what_the_simulation_reads():
    tile = np.full((51, 51), np.nan, dtype=np.float32)
    tile[0, 0] = 1.0
    tile[0, 1] = 2.0
    tile[1, 0] = 4.0
    tile[50, 50] = 6.0
    tile[25, 25] = 3.0

    message = build_tile_message((1, -1), tile, 'map', Time())
    elevation, resolution, center_x, center_y = elevation_from_message(message)

    assert np.array_equal(elevation, tile, equal_nan=True)
    assert resolution == pytest.approx(0.05)
    assert tile_index_of(center_x, center_y) == (1, -1)
