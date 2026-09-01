"""The two conversions the map hangs on: a PointCloud2 in, tiles and
commands out.

Needs grid_map_msgs and sensor_msgs importable, so:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization python3 -m pytest \
    rover/src/navi_localization/test/test_elevation_mapper.py -q'
"""

import json
import os
import time

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


def obstacle_key(frame_id):
    """(ix, iy) from an obstacle tile frame_id, ignoring the voxel size -
    for the many tests here that only care which tile a message is about."""
    return parse_obstacle_frame(frame_id)[:2]


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

    assert message.header.frame_id == f'map|0|1|{VOXEL}'
    assert parse_obstacle_frame(message.header.frame_id) == (0, 1, VOXEL)
    assert message.height == 1 and message.width == 2
    assert message.is_dense is True
    assert message.point_step == 12
    names = [field.name for field in message.fields]
    offsets = [field.offset for field in message.fields]
    assert names == ['x', 'y', 'z'] and offsets == [0, 4, 8]
    points = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(-1, 3)
    expected = (voxels.astype(np.float64) + 0.5) * VOXEL
    assert points == pytest.approx(expected, abs=1e-6)


def test_build_obstacle_message_carries_a_non_default_voxel_size():
    voxels = np.array([[2, 2, 10]], dtype=np.int32)
    message = build_obstacle_message((0, 1), voxels, Time(), voxel_m=0.10)

    assert message.header.frame_id == 'map|0|1|0.1'
    assert parse_obstacle_frame(message.header.frame_id) == (0, 1, 0.10)
    points = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(-1, 3)
    expected = (voxels.astype(np.float64) + 0.5) * 0.10
    assert points == pytest.approx(expected, abs=1e-6)


def test_build_obstacle_message_with_no_voxels_is_an_empty_but_valid_message():
    message = build_obstacle_message((3, -2), np.zeros((0, 3), dtype=np.int32), Time())
    assert message.width == 0
    assert bytes(message.data) == b''
    assert parse_obstacle_frame(message.header.frame_id) == (3, -2, VOXEL)


def test_parse_obstacle_frame_refuses_a_plain_map_frame():
    with pytest.raises(ValueError):
        parse_obstacle_frame('map')


def test_parse_obstacle_frame_accepts_the_legacy_three_part_frame_id_as_5cm():
    # Backwards compatible: a frame_id built before the obstacle voxel size
    # became a parameter has no 4th part, and every voxel it describes was
    # 5 cm.
    assert parse_obstacle_frame('map|3|-2') == (3, -2, 0.05)


def test_parse_obstacle_frame_reads_the_voxel_size_from_the_fourth_part():
    assert parse_obstacle_frame('map|3|-2|0.1') == (3, -2, 0.10)


def test_parse_obstacle_frame_refuses_a_non_numeric_size():
    with pytest.raises(ValueError):
        parse_obstacle_frame('map|3|-2|not_a_number')


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


def wall_cloud(x0=0.1, y0=0.1, n=4):
    """A small ground patch plus a two-point wall voxel directly above its
    first cell ((x0, y0)'s grid cell) - enough ground for ground_height() to
    answer for that cell, and enough points in one voxel
    (MIN_POINTS_PER_VOXEL) for the wall to register as an obstacle. With the
    defaults, the wall sits at grid cell (2, 2), tile (0, 0)."""
    ground = points_at(x0, y0, n=n, z=0.0)
    wall = [[x0, y0, 0.5], [x0, y0, 0.5]]
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

    def save(self, name, state, voxels=None, voxel_m=None, overwrite=False):
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
    # cell (2, 2) (5 cm) -> tile (0, 0), regardless of the obstacle voxel's
    # own size; the frame_id also carries that size, the node's default.
    assert parse_obstacle_frame(message.header.frame_id) == (0, 0, node._obstacles.voxel_m)
    assert message.point_step == 12 and message.is_dense is True

    points = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(-1, 3)
    assert points.shape == (1, 3)
    voxel_m = node._obstacles.voxel_m
    wall_voxel = np.array([int(np.floor(0.1 / voxel_m)), int(np.floor(0.1 / voxel_m)),
                          int(np.floor(0.5 / voxel_m))])
    expected_centre = (wall_voxel.astype(np.float64) + 0.5) * voxel_m
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
    assert any(obstacle_key(m.header.frame_id) == (0, 0) for m in non_empty)


def test_clear_sends_an_empty_obstacle_tile_for_every_published_one(node):
    node._on_cloud(wall_cloud())
    node._tick(now=0.0)
    published_keys = {obstacle_key(m.header.frame_id)
                       for m in node._obstacle_publisher.messages if m.width > 0}
    assert published_keys                      # sanity: the wall did publish

    node._obstacle_publisher.messages.clear()
    node._on_command(String(data='{"action":"clear"}'))
    assert node._obstacle_publisher.messages == []      # paced by _tick, not bursted

    node._tick(now=1.0)
    keys = {obstacle_key(m.header.frame_id) for m in node._obstacle_publisher.messages}
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
    assert obstacle_key(empty[0].header.frame_id) == (0, 0)


def test_load_sends_empty_obstacle_tile_for_tiles_the_loaded_map_no_longer_covers(node):
    # Mirrors test_load_sends_all_nan_for_tiles_the_loaded_map_no_longer_covers,
    # for obstacle tiles: a wall saved in tile A, a different wall mapped
    # afterwards in tile B, then a load of the saved map must republish A
    # with its voxel and send an empty tile for B, which the loaded map does
    # not cover at all.
    node._on_cloud(wall_cloud(0.1, 0.1))            # wall at cell (2, 2) -> tile (0, 0)
    node._on_command(String(data='{"action":"save","name":"yard"}'))
    node._on_cloud(wall_cloud(10.0, 10.0))          # wall at cell (200, 200) -> tile (4, 4)
    node._tick(now=0.0)
    node._obstacle_publisher.messages.clear()

    node._on_command(String(data='{"action":"load","name":"yard"}'))
    node._tick(now=5.0)

    by_key = {}
    for message in node._obstacle_publisher.messages:
        key = obstacle_key(message.header.frame_id)
        by_key.setdefault(key, []).append(message)

    # Tile B is not in the loaded map at all: it must go out once more as an
    # empty obstacle tile so the sim removes whatever it drew there.
    assert (4, 4) in by_key
    assert all(message.width == 0 for message in by_key[(4, 4)])

    # Tile A is in the loaded map: it must be republished with its voxel.
    assert (0, 0) in by_key
    assert any(message.width > 0 for message in by_key[(0, 0)])


def _tile_keys(messages):
    return [tile_index_of(m.info.pose.position.x, m.info.pose.position.y) for m in messages]


def _all_nan(message):
    return not np.isfinite(np.asarray(message.data[0].data, dtype=np.float32)).any()


def test_a_tile_emptied_by_the_clamp_is_blanked_once_and_never_kept_alive(node):
    # Before the first pose there is no clamp: a desk top 5 m up in tile
    # (0, 0) goes out as ground.
    node._on_cloud(cloud([[0.1, 0.1, 5.0]]))
    node._tick(now=0.0)
    assert _tile_keys(node._tile_publisher.messages) == [(0, 0)]
    assert not _all_nan(node._tile_publisher.messages[0])
    node._tile_publisher.messages.clear()

    # The pose arrives; the next cloud (somewhere else) re-offers with the
    # clamp on, and the desk's tile has no publishable cell any more.
    odometry = Odometry()
    odometry.pose.pose.position.z = 0.0
    node._on_pose(odometry)
    node._on_cloud(cloud([[5.1, 5.1, 0.0]]))       # tile (2, 2), real ground
    node._tick(now=2.0)

    keys = _tile_keys(node._tile_publisher.messages)
    assert (0, 0) in keys and (2, 2) in keys
    blank = node._tile_publisher.messages[keys.index((0, 0))]
    assert _all_nan(blank)                         # the desk is taken off screen

    # ... and never comes back as a keepalive.
    node._tile_publisher.messages.clear()
    for t in range(3, 20):
        node._tick(now=float(t))
    assert (0, 0) not in _tile_keys(node._tile_publisher.messages)
    assert (2, 2) in _tile_keys(node._tile_publisher.messages)


def test_a_loaded_tile_that_is_empty_after_the_clamp_blanks_the_old_one(node, tmp_path):
    # A live tile (0, 0) at 5 m before any pose; saved that way.
    node._on_cloud(cloud([[0.1, 0.1, 5.0]]))
    node._tick(now=0.0)
    node._on_command(String(data='{"action":"save","name":"desk"}'))
    node._tile_publisher.messages.clear()

    # Pose known now; loading the same map must not leave the 5 m tile
    # standing in the sim: it is empty once clamped, so it goes out blank.
    odometry = Odometry()
    odometry.pose.pose.position.z = 0.0
    node._on_pose(odometry)
    node._on_command(String(data='{"action":"load","name":"desk"}'))
    node._tick(now=2.0)

    messages = node._tile_publisher.messages
    assert _tile_keys(messages) == [(0, 0)]
    assert _all_nan(messages[0])


def test_loading_an_old_5cm_map_keeps_the_10cm_parameter_and_coarsens_its_voxels(node, tmp_path):
    # A map written before voxel_m existed: no such key, voxels in 5 cm
    # units. Two 5 cm voxels that share one 10 cm voxel.
    node._on_cloud(cloud(points_at(0.1, 0.1, n=4, z=0.0)))
    state = node._grid.state()
    np.savez(tmp_path / 'old.npz', elevation=state.elevation, count=state.count,
             origin_ix=state.origin_ix, origin_iy=state.origin_iy,
             resolution=state.resolution, saved_at='x',
             voxels=np.array([[2, 2, 10], [3, 2, 10]], dtype=np.int32))

    assert node._obstacles.voxel_m == pytest.approx(0.10)
    node._on_command(String(data='{"action":"load","name":"old"}'))
    node._publish_status()
    status = json.loads(node._status_publisher.messages[-1].data)
    assert status['last_command']['ok'], status['last_command']
    assert node._obstacles.voxel_m == pytest.approx(0.10)          # parameter kept
    assert node._obstacles.state().tolist() == [[1, 1, 5]]         # merged into one 10 cm voxel

    # The next fused cloud keeps voxelising at 10 cm, and so does the message.
    node._on_cloud(wall_cloud())
    node._tick(now=1.0)
    non_empty = [m for m in node._obstacle_publisher.messages if m.width > 0]
    assert all(m.header.frame_id.endswith('|0.1') for m in non_empty), \
        [m.header.frame_id for m in non_empty]


def test_loading_a_map_coarser_than_the_parameter_adopts_its_size(node):
    node._on_cloud(wall_cloud())
    node._on_command(String(data='{"action":"save","name":"coarse"}'))
    node._obstacles.replace(np.zeros((0, 3), dtype=np.int32), voxel_m=0.05)
    node._on_command(String(data='{"action":"load","name":"coarse"}'))
    assert node._obstacles.voxel_m == pytest.approx(0.10)


def test_obstacle_tiles_carry_the_frame_id_parameter_like_terrain_tiles(ros, tmp_path):
    node = ElevationMapper(map_directory=str(tmp_path))
    node._frame_id = 'odom'
    node._tile_publisher = Recorder()
    node._obstacle_publisher = Recorder()
    node._status_publisher = Recorder()
    try:
        node._on_cloud(wall_cloud())
        node._tick(now=0.0)
        frames = {m.header.frame_id.split('|')[0] for m in node._obstacle_publisher.messages}
        assert frames == {'odom'}
        assert {m.header.frame_id for m in node._tile_publisher.messages} == {"odom"}
        assert parse_obstacle_frame(node._obstacle_publisher.messages[0].header.frame_id)[:2] == (0, 0)
    finally:
        node.destroy_node()


def _count_cuts(monkeypatch):
    import navi_localization.elevation_mapper as module
    real = module.tiles_of_snapshot
    calls = []

    def counting(snapshot, only=None):
        calls.append(None if only is None else set(only))
        return real(snapshot, only=only)
    monkeypatch.setattr(module, 'tiles_of_snapshot', counting)
    return calls


def test_a_cloud_cuts_only_the_tiles_it_touched(node, monkeypatch):
    calls = _count_cuts(monkeypatch)
    node._on_cloud(cloud(points_at(0.1, 0.1)))         # tiles (0,0), (1,0)
    node._on_cloud(cloud([[10.1, 10.1, 0.0]]))          # tile (4, 4) only
    assert calls[-1] == {(4, 4)}
    node._tick(now=0.0)
    keys = sorted(_tile_keys(node._tile_publisher.messages))
    assert keys == [(0, 0), (1, 0), (4, 4)]            # the earlier tiles are still scheduled


def test_a_rover_height_change_recuts_every_tile_for_the_clamp(node, monkeypatch):
    calls = _count_cuts(monkeypatch)
    odometry = Odometry()
    odometry.pose.pose.position.z = 0.0
    node._on_pose(odometry)
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._on_cloud(cloud([[10.1, 10.1, 0.0]]))
    assert calls[-1] == {(4, 4)}                       # incremental while z is steady
    odometry.pose.pose.position.z = 0.3                # more than CLAMP_RECUT_M
    node._on_pose(odometry)
    node._on_cloud(cloud([[10.1, 10.1, 0.0]]))
    assert calls[-1] is None                           # a full cut


def test_an_incremental_cut_keeps_the_band_of_the_last_full_cut_so_tiles_agree(node):
    # Full cut at z = 0 (band up to +0.5). The rover then creeps 4 cm - under
    # CLAMP_RECUT_M - and a cloud touches another tile with a cell at 0.52:
    # inside the band at z = 0.04, outside at z = 0. The incremental cut
    # must use the z = 0 band, or the two tiles would disagree at the seam.
    odometry = Odometry()
    odometry.pose.pose.position.z = 0.0
    node._on_pose(odometry)
    node._on_cloud(cloud([[0.1, 0.1, 0.0]]))
    odometry.pose.pose.position.z = 0.04
    node._on_pose(odometry)
    node._on_cloud(cloud([[10.1, 10.1, 0.52]]))
    node._tick(now=0.0)
    keys = _tile_keys(node._tile_publisher.messages)
    assert (4, 4) not in keys                          # entirely above the band: not a tile


def test_clouds_are_not_fused_while_the_localisation_is_not_ok(node):
    node._on_localisation_status(String(data=json.dumps({"state": "SEARCHING", "reason": "pose jump"})))
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=0.0)
    assert node._tile_publisher.messages == []

    node._on_localisation_status(String(data=json.dumps({"state": "OK"})))
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=2.0)
    assert node._tile_publisher.messages != []


def test_an_unreadable_localisation_status_stops_fusing_rather_than_crashing(node):
    node._on_localisation_status(String(data="{not json"))
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=0.0)
    assert node._tile_publisher.messages == []


# -- startup_map --------------------------------------------------------
#
# These construct ElevationMapper directly rather than through the `node`
# fixture, because the behaviour under test happens once, in __init__,
# before there is a node to hand back. None of it ever writes to or
# removes a saved map: 'latest' and a named load both only choose which
# .npz to read, so a bug here can at worst start the rover from the wrong
# map or an empty one - never lose one of the operator's saved files.


def _writer(tmp_path):
    """An ElevationMapper with Recorders in place, for building fixture
    maps on disk exactly the way a live 'save' command would."""
    writer = ElevationMapper(map_directory=str(tmp_path))
    writer._tile_publisher = Recorder()
    writer._obstacle_publisher = Recorder()
    writer._status_publisher = Recorder()
    return writer


def test_with_no_startup_map_the_grid_is_empty_after_construction(node):
    assert node._grid.snapshot() is None
    assert node._loaded is None


def test_a_named_startup_map_is_loaded_into_the_grid(ros, tmp_path):
    writer = _writer(tmp_path)
    writer._on_cloud(cloud(points_at(0.1, 0.1)))
    writer._on_command(String(data='{"action":"save","name":"yard"}'))
    writer.destroy_node()

    node = ElevationMapper(map_directory=str(tmp_path), startup_map='yard')
    try:
        assert node._loaded == 'yard'
        snapshot = node._grid.snapshot()
        assert snapshot is not None
        assert np.isfinite(snapshot.elevation).any()
    finally:
        node.destroy_node()


def test_startup_map_latest_picks_the_most_recently_modified_file_not_the_alphabetically_last(ros, tmp_path):
    writer = _writer(tmp_path)
    writer._on_cloud(cloud(points_at(0.1, 0.1)))
    writer._on_command(String(data='{"action":"save","name":"zzz"}'))
    writer._on_cloud(cloud(points_at(5.0, 5.0)))
    writer._on_command(String(data='{"action":"save","name":"aaa"}'))
    writer.destroy_node()

    # 'aaa' sorts before 'zzz', so a bug that used the alphabetically last
    # name instead of the file's own mtime would pick 'zzz' here - the
    # wrong one, once the timestamps below say otherwise.
    now = time.time()
    os.utime(str(tmp_path / 'zzz.npz'), (now - 100, now - 100))
    os.utime(str(tmp_path / 'aaa.npz'), (now, now))

    node = ElevationMapper(map_directory=str(tmp_path), startup_map='latest')
    try:
        assert node._loaded == 'aaa'
    finally:
        node.destroy_node()


def test_startup_map_latest_with_an_empty_directory_is_an_empty_grid_not_an_error(ros, tmp_path):
    node = ElevationMapper(map_directory=str(tmp_path), startup_map='latest')
    try:
        assert node._grid.snapshot() is None
        assert node._loaded is None
    finally:
        node.destroy_node()


def test_a_startup_map_that_does_not_exist_leaves_the_grid_empty_and_the_node_usable(ros, tmp_path):
    node = ElevationMapper(map_directory=str(tmp_path), startup_map='does_not_exist')
    try:
        assert node._grid.snapshot() is None
        assert node._loaded is None
        # A missing map at start-up must never leave the node half-built:
        # it still maps a live cloud exactly as it would have with no
        # startup_map at all.
        node._tile_publisher = Recorder()
        node._obstacle_publisher = Recorder()
        node._status_publisher = Recorder()
        node._on_cloud(cloud(points_at(0.1, 0.1)))
        node._tick(now=0.0)
        assert node._tile_publisher.messages
    finally:
        node.destroy_node()


def test_a_corrupt_startup_map_leaves_the_grid_empty_and_the_node_usable(ros, tmp_path):
    writer = _writer(tmp_path)
    writer._on_cloud(cloud(points_at(0.1, 0.1)))
    writer._on_command(String(data='{"action":"save","name":"yard"}'))
    writer.destroy_node()
    path = tmp_path / 'yard.npz'
    whole = path.read_bytes()
    path.write_bytes(whole[:len(whole) // 2])

    node = ElevationMapper(map_directory=str(tmp_path), startup_map='yard')
    try:
        assert node._grid.snapshot() is None
        assert node._loaded is None
        node._tile_publisher = Recorder()
        node._obstacle_publisher = Recorder()
        node._status_publisher = Recorder()
        node._on_cloud(cloud(points_at(0.1, 0.1)))
        node._tick(now=0.0)
        assert node._tile_publisher.messages
    finally:
        node.destroy_node()
