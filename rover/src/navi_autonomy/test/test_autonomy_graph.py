"""Both nodes on a real ROS graph, on a throwaway domain: tiles in on
/localization/map_tile, a lethal pit rim out on /autonomy/costmap_seed.

  bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_autonomy_graph.py -q'

Domain 91 is a throwaway (spec section 9 and this repo's standing rule);
never domain 0, where the rover and the simulation live. Nothing here
publishes /manual_twist.
"""
import os
import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from navi_autonomy.tile_aggregator import (
    MAP_TILE_TOPIC, TileAggregator, latched_qos, tile_subscription_qos)
from navi_autonomy.traversability import LETHAL
from navi_autonomy.traversability_layer import COSTMAP_SEED_TOPIC, TraversabilityLayer
from navi_localization.elevation_mapper import build_tile_message

assert os.environ.get('ROS_DOMAIN_ID') == '91', \
    "run this file with ROS_DOMAIN_ID=91; never on domain 0"


class Sink(Node):
    def __init__(self):
        super().__init__('graph_test_sink')
        self.seeds = []
        self.create_subscription(OccupancyGrid, COSTMAP_SEED_TOPIC,
                                 self.seeds.append, latched_qos())


class Source(Node):
    def __init__(self):
        super().__init__('graph_test_source')
        self.publisher = self.create_publisher(
            GridMap, MAP_TILE_TOPIC, tile_subscription_qos())


def pit_tile(depth=0.3):
    """Tile (0, 0): flat at z = 0 with a 6 x 6 pit at cells [20, 26).

    0.3 m deep, past the 0.25 m step threshold: this test is about the tile
    reaching the seed as lethal cells, so the pit has to be one the layer
    actually refuses."""
    tile = np.full((51, 51), np.nan, dtype=np.float32)
    tile[:50, :50] = 0.0
    tile[20:26, 20:26] = -depth
    return tile


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


@pytest.fixture
def graph():
    rclpy.init()
    aggregator = TileAggregator()          # 1 Hz publish timer, the default
    layer = TraversabilityLayer()
    source, sink = Source(), Sink()
    executor = SingleThreadedExecutor()
    for node in (aggregator, layer, source, sink):
        executor.add_node(node)
    spin(executor, 1.0)                      # discovery
    yield executor, source, sink, aggregator, layer
    for node in (aggregator, layer, source, sink):
        executor.remove_node(node)
        node.destroy_node()
    rclpy.shutdown()


def test_a_pit_published_as_a_tile_comes_back_as_lethal_cells(graph):
    executor, source, sink, aggregator, layer = graph
    source.publisher.publish(build_tile_message((0, 0), pit_tile(), 'map', Time()))
    spin(executor, 3.0)

    assert aggregator.tiles_received >= 1
    assert layer.maps_processed >= 1
    assert sink.seeds, "no /autonomy/costmap_seed arrived"
    seed = sink.seeds[-1]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost.shape == (960, 960)
    assert seed.info.resolution == pytest.approx(0.05)

    # The pit's rim in map coordinates: lattice cell 19 is the flat ring just
    # outside it, and the window's column 0 is lattice cell origin_ix.
    row = 19 - aggregator.window.origin_iy
    column = 19 - aggregator.window.origin_ix
    assert cost[row, column] == LETHAL
    assert (cost == LETHAL).sum() == 48
    assert cost[row - 5, column - 5] == 0            # flat ground a few cells away
    assert (cost == -1).sum() > 0                    # everything never seen


def test_the_seed_is_latched_for_a_late_subscriber(graph):
    executor, source, sink, aggregator, layer = graph
    source.publisher.publish(build_tile_message((0, 0), pit_tile(), 'map', Time()))
    spin(executor, 3.0)
    late = Sink()
    executor.add_node(late)
    spin(executor, 2.0)
    assert late.seeds, "transient_local did not deliver the seed to a late joiner"
    executor.remove_node(late)
    late.destroy_node()
