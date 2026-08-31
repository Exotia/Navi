"""traversability_layer's plumbing and the end of the chain: a pit in, lethal
cells out. Publishers replaced by recorders; no ROS graph.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_traversability_layer.py -q'
"""
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time

from navi_autonomy.grid_map_io import build_grid_map, layer_from_message
from navi_autonomy.traversability import LETHAL, UNKNOWN
from navi_autonomy.traversability_layer import (
    COSTMAP_SEED_TOPIC, MAP_TOPIC, TRAVERSABILITY_TOPIC, TraversabilityLayer)


class Recorder:
    def __init__(self, subscribers=1):
        self.messages = []
        self.subscribers = subscribers

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = TraversabilityLayer()
    node._traversability_publisher = Recorder()
    node._seed_publisher = Recorder()
    node._traversability_subscribers = lambda: 1
    yield node
    node.destroy_node()


def pit_map(depth=0.2, size=6, extent=24, origin_ix=-12, origin_iy=-12):
    grid = np.zeros((extent, extent), dtype=np.float32)
    lo = (extent - size) // 2
    grid[lo:lo + size, lo:lo + size] = -depth
    return build_grid_map({'elevation': grid}, origin_ix, origin_iy, 0.05,
                          'map', Time()), lo


def test_the_topics_are_the_spec_names():
    assert MAP_TOPIC == '/autonomy/map'
    assert TRAVERSABILITY_TOPIC == '/autonomy/traversability'
    assert COSTMAP_SEED_TOPIC == '/autonomy/costmap_seed'


def test_a_pit_publishes_lethal_cells_on_its_rim(node):
    message, lo = pit_map()
    node._on_map(message)
    assert len(node._seed_publisher.messages) == 1
    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[lo - 1, lo - 1] == LETHAL
    assert cost[lo - 1, lo + 2] == LETHAL
    assert cost[2, 2] == 0
    assert (cost == LETHAL).sum() == 48


def test_the_seed_carries_the_maps_geometry(node):
    message, _ = pit_map(origin_ix=-12, origin_iy=40)
    node._on_map(message)
    seed = node._seed_publisher.messages[0]
    assert seed.header.frame_id == 'map'
    assert seed.info.resolution == pytest.approx(0.05)
    assert (seed.info.width, seed.info.height) == (24, 24)
    assert seed.info.origin.position.x == pytest.approx(-0.6)
    assert seed.info.origin.position.y == pytest.approx(2.0)


def test_unseen_ground_is_unknown_in_the_seed(node):
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    node._on_map(build_grid_map({'elevation': grid}, 0, 0, 0.05, 'map', Time()))
    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[5, 5] == UNKNOWN
    assert cost[0, 0] == UNKNOWN


def test_the_traversability_grid_map_carries_all_four_layers(node):
    message, _ = pit_map()
    node._on_map(message)
    published = node._traversability_publisher.messages[0]
    assert list(published.layers) == ['slope', 'step', 'roughness', 'valid']
    assert list(published.basic_layers) == ['valid']
    assert published.info.resolution == pytest.approx(0.05)
    step = layer_from_message(published, 'step')
    assert np.nanmax(step) == pytest.approx(0.2)


def test_the_expensive_grid_map_is_not_built_when_nobody_is_listening(node):
    node._traversability_subscribers = lambda: 0
    message, _ = pit_map()
    node._on_map(message)
    assert node._traversability_publisher.messages == []
    assert len(node._seed_publisher.messages) == 1     # the seed always goes out


def test_a_map_at_the_wrong_resolution_is_refused(node):
    message, _ = pit_map()
    message.info.resolution = 0.10
    node._on_map(message)
    assert node._seed_publisher.messages == []
    assert node.rejected_maps == 1


def test_a_map_without_an_elevation_layer_is_refused(node):
    message, _ = pit_map()
    message.layers = ['colour']
    node._on_map(message)
    assert node._seed_publisher.messages == []
    assert node.rejected_maps == 1
