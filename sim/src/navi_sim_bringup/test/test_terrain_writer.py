"""Reading the map message back, and the two rules on respawning.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup python3 -m pytest \
    sim/src/navi_sim_bringup/test/test_terrain_writer.py -q'
"""

import numpy as np
import pytest
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from navi_sim_bringup.terrain_writer import (
    LAYER, RespawnPolicy, elevation_from_message)


def message(grid_rows, grid_cols, values, resolution=0.10,
            center=(10.0, -5.0), layers=(LAYER,)):
    """A GridMap laid out the way elevation_mapper lays one out."""
    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=grid_cols,
                            stride=grid_rows * grid_cols),
        MultiArrayDimension(label='row_index', size=grid_rows, stride=grid_rows),
    ]
    layer.data = list(values)
    out = GridMap()
    out.header.frame_id = 'map'
    out.info.resolution = resolution
    out.info.length_x = grid_rows * resolution
    out.info.length_y = grid_cols * resolution
    out.info.pose.position.x, out.info.pose.position.y = center
    out.layers = list(layers)
    out.basic_layers = list(layers)
    out.data = [layer]
    return out


def test_the_message_reads_back_into_the_grid_it_was_made_from():
    # grid_map matrix (3 rows along -x, 2 columns along -y), column-major:
    # [ (0,0) (1,0) (2,0) (0,1) (1,1) (2,1) ]
    grid_map = message(3, 2, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])

    elevation, resolution, center_x, center_y = elevation_from_message(grid_map)

    assert elevation.shape == (2, 3)          # (rows along y, columns along x)
    assert resolution == pytest.approx(0.10)
    assert center_x == pytest.approx(10.0)
    assert center_y == pytest.approx(-5.0)
    # grid_map (0, 0) is the largest x and largest y, which is the last
    # column of the last row here.
    assert elevation[1, 2] == pytest.approx(10.0)
    assert elevation[0, 0] == pytest.approx(60.0)


def test_empty_cells_survive_as_nan():
    grid_map = message(1, 1, [float('nan')])

    assert np.isnan(elevation_from_message(grid_map)[0][0, 0])


def test_a_message_without_an_elevation_layer_is_refused():
    grid_map = message(1, 1, [1.0], layers=('traversability',))

    with pytest.raises(ValueError):
        elevation_from_message(grid_map)


def test_a_circular_buffer_message_is_refused_rather_than_read_wrongly():
    grid_map = message(2, 2, [1.0, 2.0, 3.0, 4.0])
    grid_map.outer_start_index = 1

    with pytest.raises(ValueError):
        elevation_from_message(grid_map)


def test_a_map_that_did_not_change_never_asks_for_a_respawn():
    policy = RespawnPolicy(interval_seconds=5.0)
    assert policy.offer(b'first', now=0.0) is True
    policy.respawned(b'first', now=0.0)

    assert policy.offer(b'first', now=100.0) is False
    assert policy.due(now=100.0) is False


def test_the_first_terrain_appears_at_once():
    assert RespawnPolicy().offer(b'first', now=0.0) is True


def test_a_change_inside_five_seconds_waits_for_the_cap():
    policy = RespawnPolicy(interval_seconds=5.0)
    policy.offer(b'first', now=0.0)
    policy.respawned(b'first', now=0.0)

    assert policy.offer(b'second', now=2.0) is False
    assert policy.due(now=4.9) is False
    assert policy.due(now=5.0) is True


def test_changes_inside_the_window_collapse_into_one_respawn_of_the_newest():
    policy = RespawnPolicy(interval_seconds=5.0)
    policy.offer(b'first', now=0.0)
    policy.respawned(b'first', now=0.0)
    policy.offer(b'second', now=1.0)
    policy.offer(b'third', now=2.0)

    assert policy.due(now=5.0) is True
    assert policy.pending == b'third'


def test_a_map_that_arrives_while_a_respawn_is_in_flight_is_not_lost():
    policy = RespawnPolicy(interval_seconds=5.0)
    policy.offer(b'first', now=0.0)
    policy.offer(b'second', now=0.1)          # arrived before 'first' spawned
    policy.respawned(b'first', now=0.2)       # Gazebo confirmed the older one

    assert policy.pending == b'second'
    assert policy.due(now=5.2) is True
