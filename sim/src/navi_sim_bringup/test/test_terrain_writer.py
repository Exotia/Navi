"""Reading the tile message back, and the rate/ordering policy on respawning.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PYTHONPATH python3 -m pytest \
    sim/src/navi_sim_bringup/test/test_terrain_writer.py -q'
"""

import numpy as np
import pytest
import rclpy
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from navi_sim_bringup.terrain_writer import (
    LAYER, TileRespawnPolicy, elevation_from_message, model_name, tile_index_of)


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


def test_model_names_alternate_per_tile():
    assert model_name((3, -2), 0) == 'terrain_3_-2_a'
    assert model_name((3, -2), 1) == 'terrain_3_-2_b'
    assert model_name((3, -2), 2) == 'terrain_3_-2_a'


def test_tile_index_of_matches_the_rovers_convention():
    assert tile_index_of(1.275, 1.275) == (0, 0)
    assert tile_index_of(2.5 * 4 + 1.275, 2.5 * -3 + 1.275) == (4, -3)


def test_a_new_tile_is_due_at_once():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    assert policy.next_due(now=0.0) == [((0, 0), b'a')]


def test_an_unchanged_payload_is_not_due_again():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    policy.started((0, 0))
    policy.finished((0, 0), b'a', now=0.0, ok=True)
    policy.offer((0, 0), b'a', now=5.0)
    assert policy.next_due(now=5.0) == []


def test_a_tile_replaces_at_most_once_per_second_and_the_newest_wins():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    policy.started((0, 0))
    policy.finished((0, 0), b'a', now=0.0, ok=True)
    policy.offer((0, 0), b'b', now=0.3)
    policy.offer((0, 0), b'c', now=0.6)
    assert policy.next_due(now=0.9) == []
    assert policy.next_due(now=1.0) == [((0, 0), b'c')]


def test_at_most_four_spawns_in_flight():
    policy = TileRespawnPolicy()
    for i in range(6):
        policy.offer((i, 0), bytes([i]), now=0.0)
    due = policy.next_due(now=0.0)
    assert len(due) == 4
    for key, _ in due:
        policy.started(key)
    assert policy.next_due(now=0.0) == []
    policy.finished((0, 0), b'\x00', now=0.1, ok=True)
    assert len(policy.next_due(now=0.1)) == 1


def test_a_failed_spawn_keeps_the_tile_pending_for_a_retry():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    policy.started((0, 0))
    policy.finished((0, 0), b'a', now=0.0, ok=False)
    assert policy.next_due(now=1.0) == [((0, 0), b'a')]


from navi_sim_bringup.terrain_writer import TerrainWriter


class FakeFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._result

    def resolve(self, result):
        self._result = result
        for callback in self._callbacks:
            callback(self)


class FakeService:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = FakeFuture()
        self.calls.append((request, future))
        return future


class Response:
    def __init__(self, success=True, status_message=''):
        self.success = success
        self.status_message = status_message


@pytest.fixture
def writer(tmp_path):
    rclpy.init()
    node = TerrainWriter(model_dir=str(tmp_path))
    node._spawn = FakeService()
    node._delete = FakeService()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def tile_message(key, value=1.0):
    from navi_sim_bringup.terrain_writer import TILE_SAMPLES, tile_center
    cx, cy = tile_center(*key)
    values = [value] * (TILE_SAMPLES * TILE_SAMPLES)
    return message(TILE_SAMPLES, TILE_SAMPLES, values, resolution=0.05, center=(cx, cy))


def test_the_replacement_is_spawned_before_the_old_tile_is_deleted(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    request, future = writer._spawn.calls[0]
    assert request.name == 'terrain_0_0_a'
    future.resolve(Response())
    assert writer._delete.calls == []                    # nothing to delete yet

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=2.0)
    request, future = writer._spawn.calls[1]
    assert request.name == 'terrain_0_0_b'
    assert writer._delete.calls == []                    # old one still standing
    future.resolve(Response())
    assert [r.name for r, _ in writer._delete.calls] == ['terrain_0_0_a']


def test_a_failed_spawn_leaves_the_old_tile_and_deletes_nothing(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._spawn.calls[0][1].resolve(Response())
    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=2.0)
    writer._spawn.calls[1][1].resolve(Response(False, 'nope'))
    assert writer._delete.calls == []
    assert writer._current[(0, 0)] == 'terrain_0_0_a'


def test_an_all_nan_tile_deletes_the_model(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._spawn.calls[0][1].resolve(Response())
    writer._on_tile(tile_message((0, 0), float('nan')))
    writer._pump(now=2.0)
    assert [r.name for r, _ in writer._delete.calls] == ['terrain_0_0_a']
    assert len(writer._spawn.calls) == 1
