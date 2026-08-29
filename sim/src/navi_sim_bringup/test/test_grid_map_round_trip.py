"""The one test that stops the rover's map and the laptop's terrain drifting apart.

elevation_mapper writes grid_map's index convention (index (0, 0) at the
largest x and largest y, rows in -x, columns in -y, column-major data) and
terrain_writer reads it back. Both are in this repository but in different
workspaces, so nothing else forces them to agree; this does.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PWD/rover/src/navi_localization \
    python3 -m pytest sim/src/navi_sim_bringup/test/test_grid_map_round_trip.py -q'
"""

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from navi_localization.elevation_grid import GridSnapshot
from navi_localization.elevation_mapper import build_grid_map_message
from navi_sim_bringup.terrain_writer import elevation_from_message


def test_what_the_rover_publishes_is_what_the_simulation_reads():
    elevation = np.array([[1.0, 2.0, 3.0],
                          [4.0, np.nan, 6.0]], dtype=np.float32)
    snapshot = GridSnapshot(elevation=elevation, center_x=3.25, center_y=-7.5,
                            resolution=0.10)

    message = build_grid_map_message(snapshot, 'map', Time())
    read_back, resolution, center_x, center_y = elevation_from_message(message)

    assert np.array_equal(read_back, elevation, equal_nan=True)
    assert resolution == pytest.approx(0.10)
    assert center_x == pytest.approx(3.25)
    assert center_y == pytest.approx(-7.5)
