"""Publishes /localization/map: the ground the rover has actually seen.

Input is the ZED SDK's fused point cloud, which the wrapper publishes in the
map frame - the same frame and the same tracking as /localization/pose, so
the map and the pose cannot drift apart relative to each other. Output is a
grid_map_msgs/GridMap with a single `elevation` layer.

The cloud is read with numpy rather than sensor_msgs_py.point_cloud2:
ZedCamera::callback_pubFusedPc lays every point out as four float32s
(x, y, z, rgb; zed_camera_component.cpp:9550), so the payload is a float32
matrix and a per-point Python loop over ~10^5 points would cost more than
everything else in this node put together. Anything that is not that layout
raises instead of being silently misread.

The wrapper only extracts the fused cloud when the topic has a subscriber
(the count_subscribers guard at the top of that callback), so this node
running is what makes the SDK do the mapping work at all. When it is not
running, the topic is silent and the GPU is not spending anything on it -
that is by design, not a fault.
"""

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from navi_localization.elevation_grid import RESOLUTION, ElevationGrid

FUSED_CLOUD_TOPIC = '/zed_front/zed_node/mapping/fused_cloud'
MAP_TOPIC = '/localization/map'
LAYER = 'elevation'


def points_from_cloud(message: PointCloud2) -> np.ndarray:
    """(N, 3) float64 of the x/y/z columns of a float32 PointCloud2."""
    names = [field.name for field in message.fields]
    offsets = [field.offset for field in message.fields]
    if names[:3] != ['x', 'y', 'z'] or offsets[:3] != [0, 4, 8]:
        raise ValueError(
            f"fused cloud has an unexpected layout: fields={names} "
            f"offsets={offsets}. This node reads x/y/z as the first three "
            f"float32s of every point and will not guess at anything else.")
    if message.is_bigendian:
        raise ValueError("fused cloud is big-endian; this node reads little-endian")
    if message.point_step % 4:
        raise ValueError(f"point_step {message.point_step} is not a whole number of float32s")

    # bytes() copies, which at 0.5 Hz and a couple of MB is free and removes
    # every alignment question np.ndarray.view would raise.
    floats = np.frombuffer(bytes(message.data), dtype=np.float32)
    stride = message.point_step // 4
    return floats.reshape(-1, stride)[:, :3].astype(np.float64)


def build_grid_map_message(snapshot, frame_id: str, stamp) -> GridMap:
    """A GridMap with one `elevation` layer, in grid_map's own index order.

    grid_map's convention is not the obvious one: index (0, 0) is the cell at
    the *maximum* x and maximum y, the row index runs in -x and the column
    index in -y, and the data is column-major - which is what grid_map_ros's
    matrixEigenCopyToMultiArrayMessage produces from an Eigen matrix. The
    snapshot stores the opposite (row 0 at minimum y, column 0 at minimum x),
    so both axes are reversed and the result transposed. Getting this right
    is what lets rviz's grid_map plugin and terrain_writer read the same
    message without either of them needing a special case.
    """
    grid = snapshot.elevation[::-1, ::-1].T
    n_rows, n_cols = grid.shape

    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n_cols,
                            stride=n_rows * n_cols),
        MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
    ]
    layer.layout.data_offset = 0
    layer.data = grid.flatten(order='F').astype(np.float32).tolist()

    message = GridMap()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.info.resolution = float(snapshot.resolution)
    message.info.length_x = float(n_rows * snapshot.resolution)
    message.info.length_y = float(n_cols * snapshot.resolution)
    message.info.pose.position.x = float(snapshot.center_x)
    message.info.pose.position.y = float(snapshot.center_y)
    message.info.pose.position.z = 0.0
    message.info.pose.orientation.w = 1.0
    message.layers = [LAYER]
    message.basic_layers = [LAYER]
    message.data = [layer]
    message.outer_start_index = 0
    message.inner_start_index = 0
    return message


class ElevationMapper(Node):

    def __init__(self) -> None:
        super().__init__('elevation_mapper')
        self.declare_parameter('cloud_topic', FUSED_CLOUD_TOPIC)
        self.declare_parameter('frame_id', 'map')
        # 0.5 Hz, the spec's rate. At the 60 x 60 m ceiling one publish is
        # 600 * 600 * 4 = 1.44 MB, which a wired link carries without
        # noticing; a run's real map is far smaller.
        self.declare_parameter('publish_interval_seconds', 2.0)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._grid = ElevationGrid()
        self._last_published = None
        self._warned_about_cap = False

        # Default QoS on both sides: reliable, volatile, depth 1. Not
        # transient-local, deliberately - sim_bridge relays this topic with
        # a generic subscription, and a durability mismatch there would mean
        # no data at all rather than late data. A late-joining simulation
        # gets the map on the next publish, at most two seconds later.
        self._publisher = self.create_publisher(GridMap, MAP_TOPIC, 1)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('cloud_topic').value),
            self._on_cloud, 1)
        self.create_timer(
            float(self.get_parameter('publish_interval_seconds').value),
            self._publish_if_changed)

        self.get_logger().info(
            f"mapping {self.get_parameter('cloud_topic').value} into "
            f"{MAP_TOPIC} at {RESOLUTION} m cells")

    def _on_cloud(self, message: PointCloud2) -> None:
        try:
            points = points_from_cloud(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self._grid.update(points)
        if self._grid.points_outside_cap and not self._warned_about_cap:
            self._warned_about_cap = True
            self.get_logger().warn(
                f"{self._grid.points_outside_cap} points fell outside the "
                "60 x 60 m map and were dropped; the map stays where it "
                "started rather than sliding and forgetting ground")

    def _publish_if_changed(self) -> None:
        snapshot = self._grid.snapshot()
        if snapshot is None or snapshot.equals(self._last_published):
            return
        if self._last_published is None or \
                snapshot.elevation.shape != self._last_published.elevation.shape:
            rows, cols = snapshot.elevation.shape
            self.get_logger().info(
                f"map is now {cols * snapshot.resolution:.1f} x "
                f"{rows * snapshot.resolution:.1f} m ({cols} x {rows} cells)")
        self._publisher.publish(build_grid_map_message(
            snapshot, self._frame_id, self.get_clock().now().to_msg()))
        self._last_published = snapshot


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
