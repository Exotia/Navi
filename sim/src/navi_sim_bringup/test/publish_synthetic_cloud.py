#!/usr/bin/env python3
"""A fake ZED fused cloud, so the map path runs without a rover.

Publishes a PointCloud2 laid out exactly like the wrapper's fused cloud -
x, y, z, rgb as four float32s, height 1, is_dense false, frame `map` - onto
/zed_front/zed_node/mapping/fused_cloud at 1 Hz, for elevation_mapper to
bin. It is a script rather than an installed node: it belongs to the tests.

  python3 sim/src/navi_sim_bringup/test/publish_synthetic_cloud.py --ramp x
  python3 sim/src/navi_sim_bringup/test/publish_synthetic_cloud.py --ramp y
  python3 sim/src/navi_sim_bringup/test/publish_synthetic_cloud.py --grow

--ramp x   ground rising 1 m over 6 m in +x, flat in y
--ramp y   the same rise in +y. The pair is how a mirrored terrain is
           told from a correct one: run both and the relief must move.
--grow     a patch that widens every publish, to watch the map grow and the
           terrain respawn no faster than once every five seconds.
"""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

TOPIC = '/zed_front/zed_node/mapping/fused_cloud'
# Half the 5 cm grid, offset by a quarter cell so no sample falls exactly on
# a cell boundary: that gives every cell in the patch the same 2 x 2 = 4
# points, uniformly, rather than the patchwork a boundary-aligned lattice
# produces (some cells getting the point, their neighbour getting none).
SPACING = 0.025


def patch(size_m: float, ramp: str) -> np.ndarray:
    axis = np.arange(SPACING / 2.0, size_m, SPACING)
    xs, ys = np.meshgrid(axis, axis, indexing='xy')
    zs = (xs if ramp == 'x' else ys) / 6.0
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def cloud_message(points: np.ndarray, stamp) -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = 'map'
    message.header.stamp = stamp
    message.height = 1
    message.width = points.shape[0]
    message.fields = [
        PointField(name=name, offset=4 * index,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(['x', 'y', 'z', 'rgb'])]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16 * message.width
    message.is_dense = False
    payload = np.zeros((points.shape[0], 4), dtype=np.float32)
    payload[:, :3] = points
    message.data = payload.tobytes()
    return message


class SyntheticCloud(Node):

    def __init__(self, ramp: str, size_m: float, grow: bool):
        super().__init__('synthetic_fused_cloud')
        self._ramp = ramp
        self._size = size_m
        self._grow = grow
        self._publisher = self.create_publisher(PointCloud2, TOPIC, 1)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f"publishing a {ramp}-ramp patch on {TOPIC} at 1 Hz")

    def _publish(self) -> None:
        points = patch(self._size, self._ramp)
        self._publisher.publish(
            cloud_message(points, self.get_clock().now().to_msg()))
        self.get_logger().info(
            f"{points.shape[0]} points over {self._size:.1f} x {self._size:.1f} m")
        if self._grow and self._size < 8.0:
            self._size += 0.5


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ramp', choices=['x', 'y'], default='x')
    parser.add_argument('--size', type=float, default=6.0,
                        help='side of the square patch in metres')
    parser.add_argument('--grow', action='store_true')
    arguments = parser.parse_args()

    rclpy.init()
    node = SyntheticCloud(arguments.ramp, arguments.size, arguments.grow)
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
