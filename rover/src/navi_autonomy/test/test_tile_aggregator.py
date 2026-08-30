"""tile_aggregator's plumbing, with the publishers replaced by recorders - no
ROS graph, the same shape as test_elevation_mapper.py.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_tile_aggregator.py -q'
"""
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from navi_autonomy.grid_map_io import layer_from_message
from navi_autonomy.tile_aggregator import MAP_TILE_TOPIC, MAP_TOPIC, POSE_TOPIC, TileAggregator
from navi_autonomy.window import WINDOW_CELLS
from navi_localization.elevation_mapper import build_tile_message


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = TileAggregator()
    node._map_publisher = Recorder()
    yield node
    node.destroy_node()


def tile(value=1.0):
    out = np.full((51, 51), np.nan, dtype=np.float32)
    out[:50, :50] = value
    return out


def pose_at(x, y):
    odom = Odometry()
    odom.pose.pose.position.x = float(x)
    odom.pose.pose.position.y = float(y)
    return odom


def test_the_topics_are_the_spec_names():
    assert MAP_TILE_TOPIC == '/localization/map_tile'
    assert POSE_TOPIC == '/localization/pose'
    assert MAP_TOPIC == '/autonomy/map'


def test_the_tile_subscription_is_as_deep_as_the_mappers_publisher(node):
    """A dropped tile is indistinguishable from unseen ground, and the mapper
    bursts up to 25 tiles a tick."""
    from navi_localization.elevation_mapper import TILE_QUEUE_DEPTH
    from navi_autonomy.tile_aggregator import tile_subscription_qos
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    assert node.tile_queue_depth == TILE_QUEUE_DEPTH == 64
    qos = tile_subscription_qos()
    assert qos.depth == 64
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_a_tile_lands_in_the_window_and_goes_out_on_the_next_tick(node):
    node._on_tile(build_tile_message((0, 0), tile(2.0), 'map', Time()))
    node._tick()
    assert len(node._map_publisher.messages) == 1
    message = node._map_publisher.messages[0]
    assert list(message.layers) == ['elevation']
    assert message.header.frame_id == 'map'
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.length_x == pytest.approx(WINDOW_CELLS * 0.05)
    elevation = layer_from_message(message, 'elevation')
    assert elevation.shape == (WINDOW_CELLS, WINDOW_CELLS)
    half = WINDOW_CELLS // 2
    assert elevation[half:half + 50, half:half + 50] == pytest.approx(2.0)
    assert np.isnan(elevation[0, 0])


def test_two_tiles_stitch_into_one_map(node):
    node._on_tile(build_tile_message((0, 0), tile(1.0), 'map', Time()))
    node._on_tile(build_tile_message((1, 0), tile(2.0), 'map', Time()))
    node._tick()
    elevation = layer_from_message(node._map_publisher.messages[0], 'elevation')
    half = WINDOW_CELLS // 2
    assert elevation[half, half] == pytest.approx(1.0)
    assert elevation[half, half + 50] == pytest.approx(2.0)


def test_an_all_nan_tile_erases_what_it_named(node):
    node._on_tile(build_tile_message((0, 0), tile(1.0), 'map', Time()))
    node._on_tile(build_tile_message(
        (0, 0), np.full((51, 51), np.nan, dtype=np.float32), 'map', Time()))
    node._tick()
    elevation = layer_from_message(node._map_publisher.messages[0], 'elevation')
    half = WINDOW_CELLS // 2
    assert not np.isfinite(elevation[half:half + 50, half:half + 50]).any()


def test_a_tile_at_the_wrong_resolution_is_dropped_not_resampled(node):
    message = build_tile_message((0, 0), tile(1.0), 'map', Time())
    message.info.resolution = 0.10
    node._on_tile(message)                     # must not raise out of a callback
    node._tick()
    assert node.rejected_tiles == 1
    assert node.tiles_received == 0
    assert node._map_publisher.messages == []  # nothing was ever seen
    assert not np.isfinite(node.window.elevation).any()


def test_nothing_is_published_before_the_first_tile(node):
    node._tick()
    assert node._map_publisher.messages == []


def test_a_pose_close_to_the_centre_does_not_move_the_window(node):
    node._on_tile(build_tile_message((0, 0), tile(1.0), 'map', Time()))
    before = (node.window.origin_ix, node.window.origin_iy)
    node._on_pose(pose_at(5.0, -5.0))
    node._tick()
    assert (node.window.origin_ix, node.window.origin_iy) == before


def test_a_distant_pose_recentres_the_window_and_keeps_the_ground_where_it_is(node):
    node._on_tile(build_tile_message((0, 0), tile(3.0), 'map', Time()))
    node._on_pose(pose_at(20.0, 0.0))
    node._tick()
    assert node.window.origin_ix > -WINDOW_CELLS // 2
    elevation = layer_from_message(node._map_publisher.messages[0], 'elevation')
    column = 0 - node.window.origin_ix         # lattice cell 0
    row = 0 - node.window.origin_iy
    assert elevation[row, column] == pytest.approx(3.0)
