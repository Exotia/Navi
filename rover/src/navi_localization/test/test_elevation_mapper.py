"""The two conversions the map hangs on: a PointCloud2 in, tiles and
commands out.

Needs grid_map_msgs and sensor_msgs importable, so:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization python3 -m pytest \
    rover/src/navi_localization/test/test_elevation_mapper.py -q'
"""

import json

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from navi_localization.elevation_mapper import (
    MAP_COMMAND_TOPIC, MAP_STATUS_TOPIC, MAP_TILE_TOPIC, OBSTACLE_TILE_TOPIC, POSE_TOPIC,
    ElevationMapper, build_obstacle_message, build_tile_message, parse_obstacle_frame,
    points_from_cloud)
from navi_localization.tiles import TILE_SAMPLES, tile_center, tile_index_of
from navi_localization.voxels import VOXEL
from std_msgs.msg import String


def cloud(points, with_rgb=True):
    """A PointCloud2 shaped exactly like the ZED wrapper's fused cloud."""
    message = PointCloud2()
    message.header.frame_id = 'map'
    message.height = 1
    message.width = len(points)
    names = ['x', 'y', 'z'] + (['rgb'] if with_rgb else [])
    message.fields = [
        PointField(name=name, offset=4 * index,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(names)]
    message.is_bigendian = False
    message.point_step = 4 * len(names)
    message.row_step = message.point_step * message.width
    message.is_dense = False
    values = []
    for x, y, z in points:
        values.extend([x, y, z] + ([0.0] if with_rgb else []))
    message.data = np.asarray(values, dtype=np.float32).tobytes()
    return message


def test_the_xyz_columns_come_out_of_a_four_field_cloud():
    points = points_from_cloud(cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]))

    assert points.shape == (2, 3)
    assert points[1, 0] == pytest.approx(4.0)
    assert points[1, 2] == pytest.approx(6.0)


def test_a_cloud_without_the_colour_field_still_reads():
    points = points_from_cloud(cloud([(1.0, 2.0, 3.0)], with_rgb=False))

    assert points.shape == (1, 3)


def test_a_cloud_with_an_unexpected_layout_is_refused_rather_than_misread():
    message = cloud([(1.0, 2.0, 3.0)])
    message.fields[0].name = 'intensity'

    with pytest.raises(ValueError):
        points_from_cloud(message)


def test_a_big_endian_cloud_is_refused():
    message = cloud([(1.0, 2.0, 3.0)])
    message.is_bigendian = True

    with pytest.raises(ValueError):
        points_from_cloud(message)


def test_a_cloud_whose_xyz_fields_are_not_float32_is_refused():
    message = cloud([(1.0, 2.0, 3.0)])
    message.fields[0].datatype = PointField.FLOAT64

    with pytest.raises(ValueError):
        points_from_cloud(message)


def tile(value=1.0):
    return np.full((TILE_SAMPLES, TILE_SAMPLES), value, dtype=np.float32)


def test_the_tile_message_is_a_51_sample_grid_map_centred_on_the_tile():
    message = build_tile_message((2, -1), tile(), 'map', Time())
    assert message.layers == ['elevation'] and message.basic_layers == ['elevation']
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.length_x == pytest.approx(2.55)
    assert message.info.length_y == pytest.approx(2.55)
    cx, cy = tile_center(2, -1)
    assert message.info.pose.position.x == pytest.approx(cx)
    assert message.info.pose.position.y == pytest.approx(cy)
    assert tile_index_of(message.info.pose.position.x, message.info.pose.position.y) == (2, -1)
    assert len(message.data[0].data) == TILE_SAMPLES * TILE_SAMPLES
    assert message.outer_start_index == 0 and message.inner_start_index == 0


def test_the_tile_layout_is_grid_maps_column_major_with_index_zero_at_max_x_max_y():
    values = tile(np.nan)
    values[0, 0] = 1.0          # smallest y, smallest x
    values[50, 50] = 9.0        # largest y, largest x
    message = build_tile_message((0, 0), values, 'map', Time())
    data = message.data[0]
    assert data.layout.dim[0].label == 'column_index'
    assert data.data[0] == 9.0
    assert data.data[-1] == 1.0


def test_build_obstacle_message_carries_tile_identity_and_voxel_centres():
    voxels = np.array([[2, 2, 10], [2, 3, 10]], dtype=np.int32)
    message = build_obstacle_message((0, 1), voxels, Time())

    assert message.header.frame_id == 'map|0|1'
    assert parse_obstacle_frame(message.header.frame_id) == (0, 1)
    assert message.height == 1 and message.width == 2
    assert message.is_dense is True
    assert message.point_step == 12
    names = [field.name for field in message.fields]
    offsets = [field.offset for field in message.fields]
    assert names == ['x', 'y', 'z'] and offsets == [0, 4, 8]
    points = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(-1, 3)
    expected = (voxels.astype(np.float64) + 0.5) * VOXEL
    assert points == pytest.approx(expected, abs=1e-6)


def test_build_obstacle_message_with_no_voxels_is_an_empty_but_valid_message():
    message = build_obstacle_message((3, -2), np.zeros((0, 3), dtype=np.int32), Time())
    assert message.width == 0
    assert bytes(message.data) == b''
    assert parse_obstacle_frame(message.header.frame_id) == (3, -2)


def test_parse_obstacle_frame_refuses_a_plain_map_frame():
    with pytest.raises(ValueError):
        parse_obstacle_frame('map')


# Node-level tests: the node is exercised with messages fed straight into its
# callbacks and its publishers replaced with recorders, the same pattern
# test_localization_status.py uses. No spinning, no executor - the timer is
# ticked by calling _tick() directly.


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros, tmp_path):
    node = ElevationMapper(map_directory=str(tmp_path))
    node._tile_publisher = Recorder()
    node._obstacle_publisher = Recorder()
    node._status_publisher = Recorder()
    yield node
    node.destroy_node()


def points_at(x0, y0, n=60, z=1.0):
    return [[x0 + 0.05 * i, y0 + 0.05 * j, z] for i in range(n) for j in range(2)]


def wall_cloud(n=4):
    """A small ground patch plus a two-point wall voxel directly above its
    first cell (2, 2 in grid indices, tile (0, 0)) - enough ground for
    ground_height() to answer for that cell, and enough points in one voxel
    (MIN_POINTS_PER_VOXEL) for the wall to register as an obstacle."""
    ground = points_at(0.1, 0.1, n=n, z=0.0)
    wall = [[0.1, 0.1, 0.5], [0.1, 0.1, 0.5]]
    return cloud(ground + wall)


def test_a_cloud_then_a_tick_publishes_the_tiles_it_touched(node):
    node._on_cloud(cloud(points_at(0.1, 0.1)))     # x 0.1..3.05: tiles 0 and 1
    node._tick(now=0.0)
    keys = sorted(tile_index_of(m.info.pose.position.x, m.info.pose.position.y)
                  for m in node._tile_publisher.messages)
    assert keys == [(0, 0), (1, 0)]


def test_an_unchanged_map_sends_one_keepalive_tile_per_tick(node):
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=0.0)
    node._tile_publisher.messages.clear()
    node._tick(now=2.0)
    node._tick(now=3.0)
    assert len(node._tile_publisher.messages) == 2


def test_a_malformed_cloud_is_logged_and_does_not_publish(node):
    message = cloud([(1.0, 2.0, 3.0)])
    message.fields[0].name = 'intensity'

    node._on_cloud(message)
    node._tick(now=0.0)

    assert node._tile_publisher.messages == []


def test_a_save_command_writes_the_file_and_the_status_reports_it(node, tmp_path):
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._on_command(String(data='{"action":"save","name":"yard"}'))
    assert (tmp_path / 'yard.npz').exists()
    node._publish_status()
    status = json.loads(node._status_publisher.messages[-1].data)
    assert status['maps'] == ['yard']
    assert status['last_command'] == {**status['last_command'], 'action': 'save',
                                      'name': 'yard', 'ok': True, 'error': None}
    assert status['resolution'] == 0.05 and status['cells_seen'] > 0
    assert status['loaded'] is None


def test_a_bad_command_is_reported_not_raised(node):
    node._on_command(String(data='not json'))
    node._on_command(String(data='{"action":"save","name":"bad name"}'))
    node._on_command(String(data='{"action":"teleport"}'))
    node._publish_status()
    last = json.loads(node._status_publisher.messages[-1].data)['last_command']
    assert last['ok'] is False and 'teleport' in last['error']


def test_load_replaces_the_grid_and_republishes_every_tile(node):
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._on_command(String(data='{"action":"save","name":"yard"}'))
    node._on_cloud(cloud(points_at(10.0, 10.0)))
    node._tick(now=0.0)
    node._tile_publisher.messages.clear()

    node._on_command(String(data='{"action":"load","name":"yard"}'))
    node._tick(now=5.0)
    # The loaded map's own tiles are republished with finite data; the
    # stale (10, 10)-area tiles are also republished, but as all-NaN - see
    # test_load_sends_all_nan_for_tiles_the_loaded_map_no_longer_covers.
    keys = {tile_index_of(m.info.pose.position.x, m.info.pose.position.y)
            for m in node._tile_publisher.messages}
    assert {(0, 0), (1, 0)} <= keys
    node._publish_status()
    assert json.loads(node._status_publisher.messages[-1].data)['loaded'] == 'yard'


def test_load_sends_all_nan_for_tiles_the_loaded_map_no_longer_covers(node):
    node._on_cloud(cloud(points_at(0.1, 0.1)))     # tiles (0,0),(1,0)
    node._on_command(String(data='{"action":"save","name":"yard"}'))
    node._on_cloud(cloud(points_at(10.0, 10.0)))    # far away: different tiles
    node._tick(now=0.0)
    node._tile_publisher.messages.clear()

    node._on_command(String(data='{"action":"load","name":"yard"}'))
    node._tick(now=5.0)

    by_key = {}
    for message in node._tile_publisher.messages:
        key = tile_index_of(message.info.pose.position.x, message.info.pose.position.y)
        by_key.setdefault(key, []).append(message)

    stale_keys = {key for key in by_key if key not in {(0, 0), (1, 0)}}
    assert stale_keys                                  # the (10, 10) area was touched
    for key in stale_keys:
        for message in by_key[key]:
            assert np.isnan(np.asarray(message.data[0].data)).all()

    for key in [(0, 0), (1, 0)]:
        assert key in by_key
        assert any(np.isfinite(np.asarray(m.data[0].data)).any() for m in by_key[key])


def test_clear_empties_the_grid_and_sends_every_published_tile_once_more_as_nan(node):
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=0.0)
    node._tile_publisher.messages.clear()
    node._on_command(String(data='{"action":"clear"}'))
    node._tick(now=1.0)                                # the blanks are paced by _tick
    messages = node._tile_publisher.messages
    assert len(messages) == 2
    assert all(np.isnan(np.asarray(m.data[0].data)).all() for m in messages)
    node._publish_status()
    status = json.loads(node._status_publisher.messages[-1].data)
    assert status['cells_seen'] == 0 and status['tiles'] == 0


def test_the_node_talks_on_the_map_and_obstacle_topics(node):
    names = {name for name, _ in node.get_topic_names_and_types()}
    assert {MAP_TILE_TOPIC, OBSTACLE_TILE_TOPIC, MAP_COMMAND_TOPIC, MAP_STATUS_TOPIC} <= names
    assert '/localization/map' not in names


def test_the_node_subscribes_the_rover_pose(node):
    names = {name for name, _ in node.get_topic_names_and_types()}
    assert POSE_TOPIC in names


def _only_finite_value(message):
    values = np.asarray(message.data[0].data)
    finite = values[np.isfinite(values)]
    assert finite.size == 1
    return float(finite[0])


def test_no_clamp_before_the_first_pose(node):
    node._on_cloud(cloud([[0.1, 0.1, 5.0]]))
    node._tick(now=0.0)

    assert len(node._tile_publisher.messages) == 1
    assert _only_finite_value(node._tile_publisher.messages[0]) == pytest.approx(5.0)
    assert node._grid.snapshot().top[0, 0] == pytest.approx(5.0)


def test_a_pose_hides_published_cells_above_the_rover_band_but_never_top(node):
    # _on_pose only remembers z_rover - it does not re-offer (see
    # test_pose_alone_does_not_reoffer_tiles) - so the clamp is exercised
    # here through the _offer that _on_cloud already triggers, with the
    # pose set first.
    odometry = Odometry()
    odometry.pose.pose.position.z = 0.0
    node._on_pose(odometry)          # z_rover = 0.0; default clamp_above = 0.5

    # One ground cell beside one cell 5 m up (a wall). The wall cell is
    # published as unseen, not as a 0.5 m plateau; the ground cell stays.
    node._on_cloud(cloud([[0.1, 0.1, 5.0], [0.16, 0.1, 0.0]]))
    node._tick(now=0.0)

    assert len(node._tile_publisher.messages) == 1
    values = np.asarray(node._tile_publisher.messages[0].data[0].data, dtype=np.float32)
    finite = values[np.isfinite(values)]
    assert list(finite) == pytest.approx([0.0])
    # top is never clamped, even though the published cell now is.
    assert node._grid.snapshot().top[0, 0] == pytest.approx(5.0)


def test_a_pose_also_clamps_cells_below_the_rover_height(node):
    odometry = Odometry()
    odometry.pose.pose.position.z = 0.0
    node._on_pose(odometry)          # z_rover = 0.0; default clamp_below = 1.0

    node._on_cloud(cloud([[0.1, 0.1, -5.0]]))
    node._tick(now=0.0)

    assert len(node._tile_publisher.messages) == 1
    assert _only_finite_value(node._tile_publisher.messages[0]) == pytest.approx(-1.0)
    assert node._grid.snapshot().top[0, 0] == pytest.approx(-5.0)


def test_pose_alone_does_not_reoffer_tiles(node, monkeypatch):
    # _on_pose must be cheap: it only remembers z_rover. Re-cutting the
    # grid into tiles costs ~30 ms at the 60 m cap, and pose arrives at
    # 15 Hz while the cloud - the thing that actually changes what there
    # is to publish - arrives at ~1 Hz, so _on_pose calling tiles_of_snapshot
    # would burn ~half a core on the Jetson for nothing.
    import navi_localization.elevation_mapper as elevation_mapper_module

    node._on_cloud(cloud([[0.1, 0.1, 5.0]]))
    node._tick(now=0.0)
    published_before = list(node._tile_publisher.messages)
    node._tile_publisher.messages.clear()

    calls = []
    real_tiles_of_snapshot = elevation_mapper_module.tiles_of_snapshot

    def counting_tiles_of_snapshot(*args, **kwargs):
        calls.append(1)
        return real_tiles_of_snapshot(*args, **kwargs)

    monkeypatch.setattr(elevation_mapper_module, 'tiles_of_snapshot', counting_tiles_of_snapshot)

    for i in range(20):
        odometry = Odometry()
        odometry.pose.pose.position.z = float(i) * 0.1
        node._on_pose(odometry)

    assert calls == []          # _on_pose never re-cuts the grid into tiles

    node._tick(now=5.0)         # past MIN_INTERVAL_S: a keepalive may go out
    if node._tile_publisher.messages:
        # Whatever the keepalive republishes is exactly what the last real
        # _offer (from the cloud, before any pose arrived) computed - the
        # 20 pose messages did not change it.
        assert (_only_finite_value(node._tile_publisher.messages[-1])
                == _only_finite_value(published_before[-1]))


class ExplodingStore:
    """A MapStore whose save fails the way a full disk does."""

    def __init__(self, real):
        self.directory = real.directory
        self._real = real

    def list_names(self):
        return self._real.list_names()

    def save(self, name, state, voxels=None, overwrite=False):
        raise OSError('[Errno 28] No space left on device')

    def load(self, name):
        return self._real.load(name)


def test_a_save_that_fails_with_an_oserror_is_reported_not_fatal(node):
    node._store = ExplodingStore(node._store)
    node._on_cloud(cloud(points_at(0.1, 0.1)))

    node._on_command(String(data='{"action":"save","name":"yard"}'))

    node._publish_status()
    last = json.loads(node._status_publisher.messages[-1].data)['last_command']
    assert last['ok'] is False and 'No space left' in last['error']
    # The node is still mapping: a raised OSError would have come out of
    # rclpy.spin and ended the run.
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=1.0)
    assert node._tile_publisher.messages


def test_loading_a_truncated_map_is_reported_not_fatal(node, tmp_path):
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._on_command(String(data='{"action":"save","name":"yard"}'))
    path = tmp_path / 'yard.npz'
    whole = path.read_bytes()
    path.write_bytes(whole[:len(whole) // 2])

    node._on_command(String(data='{"action":"load","name":"yard"}'))

    node._publish_status()
    last = json.loads(node._status_publisher.messages[-1].data)['last_command']
    assert last['ok'] is False and 'yard' in last['error']
    node._tick(now=1.0)


def test_loading_a_map_without_an_elevation_array_is_reported_not_fatal(node, tmp_path):
    np.savez_compressed(str(tmp_path / 'half.npz'),
                        count=np.ones((4, 4), dtype=np.int32),
                        origin_ix=np.int64(0), origin_iy=np.int64(0),
                        resolution=np.float64(0.05))

    node._on_command(String(data='{"action":"load","name":"half"}'))

    node._publish_status()
    last = json.loads(node._status_publisher.messages[-1].data)['last_command']
    assert last['ok'] is False and 'elevation' in last['error']


def test_the_status_tile_count_follows_the_map(node):
    node._publish_status()
    assert json.loads(node._status_publisher.messages[-1].data)['tiles'] == 0
    node._on_cloud(cloud(points_at(0.1, 0.1)))     # tiles (0, 0) and (1, 0)
    node._publish_status()
    assert json.loads(node._status_publisher.messages[-1].data)['tiles'] == 2


def points_over_tiles(n_x, n_y, z=1.0):
    """A cloud covering n_x by n_y whole 2.5 m tiles from the origin."""
    xs = np.arange(0.05, 2.5 * n_x, 0.25)
    ys = np.arange(0.05, 2.5 * n_y, 0.25)
    grid = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
    return [[x, y, z] for x, y in grid]


def _published_keys(node):
    return {tile_index_of(m.info.pose.position.x, m.info.pose.position.y)
            for m in node._tile_publisher.messages}


def test_the_tile_publisher_is_deep_enough_for_a_blanking_burst(ros, tmp_path):
    node = ElevationMapper(map_directory=str(tmp_path))
    try:
        assert node._tile_publisher.qos_profile.depth >= 64
        assert node._obstacle_publisher.qos_profile.depth >= 64
    finally:
        node.destroy_node()


def test_a_clear_paces_the_blanking_tiles_instead_of_bursting_them(node):
    node._on_cloud(cloud(points_over_tiles(7, 6)))
    for tick in range(12):
        node._tick(now=float(tick))
    was_published = _published_keys(node)
    assert len(was_published) >= 40
    node._tile_publisher.messages.clear()

    node._on_command(String(data='{"action":"clear"}'))
    # Nothing at all out of the command callback: 40-plus messages into a
    # depth-64 writer and a bridge queue of the same size is exactly the
    # burst that used to drop tiles on the floor.
    assert node._tile_publisher.messages == []

    node._tick(now=20.0)
    assert len(node._tile_publisher.messages) == 16
    assert all(np.isnan(np.asarray(m.data[0].data)).all()
               for m in node._tile_publisher.messages)

    ticks = 1
    while node._pending_nan:
        ticks += 1
        node._tick(now=20.0 + ticks)
    assert ticks == -(-len(was_published) // 16)          # ceil, 16 a tick
    assert _published_keys(node) == was_published
    assert all(np.isnan(np.asarray(m.data[0].data)).all()
               for m in node._tile_publisher.messages)


def test_a_tile_that_comes_back_before_its_blank_went_out_is_not_blanked(node):
    node._on_cloud(cloud(points_over_tiles(7, 6)))
    for tick in range(12):
        node._tick(now=float(tick))
    node._tile_publisher.messages.clear()

    node._on_command(String(data='{"action":"clear"}'))
    node._on_cloud(cloud(points_at(0.1, 0.1)))            # tiles (0, 0), (1, 0) are live again
    while node._pending_nan:
        node._tick(now=30.0)
    blanked = {tile_index_of(m.info.pose.position.x, m.info.pose.position.y)
               for m in node._tile_publisher.messages
               if np.isnan(np.asarray(m.data[0].data)).all()}
    assert (0, 0) not in blanked and (1, 0) not in blanked


# Obstacle voxel tiles: publish, save/load, clear, and the one case terrain
# tiles never hit - a tile touched by a live update that loses every voxel.


def test_a_wall_publishes_an_obstacle_tile_with_the_right_frame_id_and_centres(node):
    node._on_cloud(wall_cloud())
    node._tick(now=0.0)

    non_empty = [m for m in node._obstacle_publisher.messages if m.width > 0]
    assert len(non_empty) == 1
    message = non_empty[0]
    assert parse_obstacle_frame(message.header.frame_id) == (0, 0)     # cell (2, 2) -> tile (0, 0)
    assert message.point_step == 12 and message.is_dense is True

    points = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(-1, 3)
    assert points.shape == (1, 3)
    wall_voxel = np.array([2, 2, int(np.floor(0.5 / VOXEL))])
    expected_centre = (wall_voxel.astype(np.float64) + 0.5) * VOXEL
    assert points[0] == pytest.approx(expected_centre, abs=1e-5)


def test_terrain_tiles_are_unaffected_by_obstacle_voxels(node):
    # The same cloud that raises an obstacle also still maps the ground
    # underneath it as ordinary terrain.
    node._on_cloud(wall_cloud())
    node._tick(now=0.0)

    assert node._tile_publisher.messages          # terrain still publishes
    assert any(np.isfinite(np.asarray(m.data[0].data)).any()
               for m in node._tile_publisher.messages)


def test_save_load_round_trips_the_obstacle_voxels_and_the_status_count(node):
    node._on_cloud(wall_cloud())
    node._publish_status()
    assert json.loads(node._status_publisher.messages[-1].data)['voxels'] == 1

    node._on_command(String(data='{"action":"save","name":"yard"}'))
    node._on_command(String(data='{"action":"clear"}'))
    node._publish_status()
    assert json.loads(node._status_publisher.messages[-1].data)['voxels'] == 0

    node._on_command(String(data='{"action":"load","name":"yard"}'))
    node._publish_status()
    status = json.loads(node._status_publisher.messages[-1].data)
    assert status['voxels'] == 1 and status['loaded'] == 'yard'

    node._tick(now=1.0)
    non_empty = [m for m in node._obstacle_publisher.messages if m.width > 0]
    assert any(parse_obstacle_frame(m.header.frame_id) == (0, 0) for m in non_empty)


def test_clear_sends_an_empty_obstacle_tile_for_every_published_one(node):
    node._on_cloud(wall_cloud())
    node._tick(now=0.0)
    published_keys = {parse_obstacle_frame(m.header.frame_id)
                       for m in node._obstacle_publisher.messages if m.width > 0}
    assert published_keys                      # sanity: the wall did publish

    node._obstacle_publisher.messages.clear()
    node._on_command(String(data='{"action":"clear"}'))
    assert node._obstacle_publisher.messages == []      # paced by _tick, not bursted

    node._tick(now=1.0)
    keys = {parse_obstacle_frame(m.header.frame_id) for m in node._obstacle_publisher.messages}
    assert keys == published_keys
    assert all(m.width == 0 for m in node._obstacle_publisher.messages)


def test_an_obstacle_tile_that_loses_all_voxels_is_republished_empty(node):
    node._on_cloud(wall_cloud())
    node._tick(now=0.0)
    node._obstacle_publisher.messages.clear()

    # Same cell touched again, but this time with only ground-height
    # points: no obstacle candidate there any more, so the tile ObstacleMap
    # touched loses every voxel and becomes empty.
    node._on_cloud(cloud(points_at(0.1, 0.1, n=4, z=0.0)))
    node._tick(now=2.0)

    empty = [m for m in node._obstacle_publisher.messages if m.width == 0]
    assert empty
    assert parse_obstacle_frame(empty[0].header.frame_id) == (0, 0)
