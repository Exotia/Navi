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
from sensor_msgs.msg import PointCloud2, PointField

from navi_localization.elevation_mapper import (
    MAP_COMMAND_TOPIC, MAP_STATUS_TOPIC, MAP_TILE_TOPIC, ElevationMapper,
    build_tile_message, points_from_cloud)
from navi_localization.tiles import TILE_SAMPLES, tile_center, tile_index_of
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
    node._status_publisher = Recorder()
    yield node
    node.destroy_node()


def points_at(x0, y0, n=60, z=1.0):
    return [[x0 + 0.05 * i, y0 + 0.05 * j, z] for i in range(n) for j in range(2)]


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
    keys = sorted(tile_index_of(m.info.pose.position.x, m.info.pose.position.y)
                  for m in node._tile_publisher.messages)
    assert keys == [(0, 0), (1, 0)]
    node._publish_status()
    assert json.loads(node._status_publisher.messages[-1].data)['loaded'] == 'yard'


def test_clear_empties_the_grid_and_sends_every_published_tile_once_more_as_nan(node):
    node._on_cloud(cloud(points_at(0.1, 0.1)))
    node._tick(now=0.0)
    node._tile_publisher.messages.clear()
    node._on_command(String(data='{"action":"clear"}'))
    messages = node._tile_publisher.messages
    assert len(messages) == 2
    assert all(np.isnan(np.asarray(m.data[0].data)).all() for m in messages)
    node._publish_status()
    status = json.loads(node._status_publisher.messages[-1].data)
    assert status['cells_seen'] == 0 and status['tiles'] == 0


def test_the_node_talks_on_exactly_the_three_map_topics(node):
    names = {name for name, _ in node.get_topic_names_and_types()}
    assert {MAP_TILE_TOPIC, MAP_COMMAND_TOPIC, MAP_STATUS_TOPIC} <= names
    assert '/localization/map' not in names
