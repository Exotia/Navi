"""The two conversions the map hangs on: a PointCloud2 in, a GridMap out.

Needs grid_map_msgs and sensor_msgs importable, so:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization python3 -m pytest \
    rover/src/navi_localization/test/test_elevation_mapper.py -q'
"""

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField

from navi_localization.elevation_grid import GridSnapshot, RESOLUTION
from navi_localization.elevation_mapper import (
    FUSED_CLOUD_TOPIC, LAYER, ElevationMapper, build_grid_map_message,
    points_from_cloud)


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


def snapshot():
    # Two rows (y), three columns (x). Distinct values so a transposed or
    # flipped conversion cannot pass by accident.
    elevation = np.array([[1.0, 2.0, 3.0],
                          [4.0, 5.0, np.nan]], dtype=np.float32)
    return GridSnapshot(elevation=elevation, center_x=10.0, center_y=-5.0,
                        resolution=0.10)


def test_the_message_carries_one_elevation_layer_in_the_map_frame():
    message = build_grid_map_message(snapshot(), 'map', Time())

    assert message.header.frame_id == 'map'
    assert message.layers == [LAYER]
    assert message.basic_layers == [LAYER]
    assert len(message.data) == 1


def test_the_message_geometry_is_the_snapshots():
    message = build_grid_map_message(snapshot(), 'map', Time())

    assert message.info.resolution == pytest.approx(0.10)
    # grid_map's length_x counts the rows of its own matrix, which run along
    # x - three columns of the snapshot, so 0.30 m.
    assert message.info.length_x == pytest.approx(0.30)
    assert message.info.length_y == pytest.approx(0.20)
    assert message.info.pose.position.x == pytest.approx(10.0)
    assert message.info.pose.position.y == pytest.approx(-5.0)
    assert message.info.pose.orientation.w == pytest.approx(1.0)


def test_the_layout_is_the_column_major_one_grid_map_ros_writes():
    layer = build_grid_map_message(snapshot(), 'map', Time()).data[0]

    assert [dimension.label for dimension in layer.layout.dim] == [
        'column_index', 'row_index']
    assert layer.layout.dim[0].size == 2      # columns of the grid_map matrix
    assert layer.layout.dim[0].stride == 6
    assert layer.layout.dim[1].size == 3      # rows of the grid_map matrix
    assert layer.layout.dim[1].stride == 3
    assert len(layer.data) == 6


def test_index_zero_is_the_cell_at_the_largest_x_and_largest_y():
    # grid_map's convention: row index runs in -x, column index in -y, so
    # (0, 0) is the far corner. In the snapshot that is the last column of
    # the last row - the NaN.
    layer = build_grid_map_message(snapshot(), 'map', Time()).data[0]

    assert np.isnan(layer.data[0])
    # Column-major: index 1 is grid_map row 1, column 0 - one cell towards
    # -x from the corner, i.e. snapshot row 1, column 1.
    assert layer.data[1] == pytest.approx(5.0)
    # The last element is grid_map (row 2, column 1): snapshot (0, 0).
    assert layer.data[5] == pytest.approx(1.0)


def test_the_circular_buffer_indices_are_left_at_zero():
    message = build_grid_map_message(snapshot(), 'map', Time())

    assert message.outer_start_index == 0
    assert message.inner_start_index == 0


# Node-level tests: the node is exercised with messages fed straight into its
# callbacks and its publisher replaced with a recorder, the same pattern
# test_localization_status.py uses. No spinning, no executor - the timer is
# ticked by calling _publish_if_changed() directly.


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
def node():
    n = ElevationMapper()
    n._publisher = Recorder()
    yield n
    n.destroy_node()


def test_a_cloud_then_a_tick_publishes_one_grid_map(node):
    node._on_cloud(cloud([(0.05, 0.05, 1.0), (0.25, 0.05, 2.0)]))
    node._publish_if_changed()

    assert len(node._publisher.messages) == 1
    message = node._publisher.messages[0]
    assert message.header.frame_id == 'map'
    assert message.layers == [LAYER]
    assert message.info.resolution == pytest.approx(RESOLUTION)
    layer = message.data[0]
    assert len(layer.data) == layer.layout.dim[0].size * layer.layout.dim[1].size


def test_a_second_tick_without_a_new_cloud_publishes_nothing(node):
    node._on_cloud(cloud([(0.05, 0.05, 1.0), (0.25, 0.05, 2.0)]))
    node._publish_if_changed()

    node._publish_if_changed()

    assert len(node._publisher.messages) == 1


def test_a_changed_cloud_then_a_tick_publishes_again(node):
    node._on_cloud(cloud([(0.05, 0.05, 1.0), (0.25, 0.05, 2.0)]))
    node._publish_if_changed()

    # Same cell, different z - the grid replaces rather than accumulates, so
    # this changes the published mean and should trigger a republish.
    node._on_cloud(cloud([(0.05, 0.05, 9.0), (0.25, 0.05, 2.0)]))
    node._publish_if_changed()

    assert len(node._publisher.messages) == 2


def test_a_malformed_cloud_is_logged_and_does_not_publish(node):
    message = cloud([(1.0, 2.0, 3.0)])
    message.fields[0].name = 'intensity'

    node._on_cloud(message)
    node._publish_if_changed()

    assert node._publisher.messages == []


def test_the_node_subscribes_only_the_fused_cloud_topic(node):
    topics = [s.topic_name for s in node.subscriptions]
    assert topics == [FUSED_CLOUD_TOPIC]


def test_an_unchanged_map_is_resent_once_the_keepalive_has_elapsed(node):
    # A late joiner - terrain_writer starting after the map stopped growing,
    # or a restarted bridge - would otherwise never see the map at all.
    node._on_cloud(cloud([(0.05, 0.05, 1.0), (0.25, 0.05, 2.0)]))
    node._publish_if_changed()
    assert len(node._publisher.messages) == 1
    node._last_publish_time -= 11.0     # pretend 11 s passed (keepalive is 10)

    node._publish_if_changed()

    assert len(node._publisher.messages) == 2
