"""The fixture seed on the graph, plus - only when asked - the frames and
odometry the ZED would otherwise own.

Two jobs, one node, because they are the same job: standing in for a rover
that has no camera attached.

    ros2 run navi_nav2 fixture_seed_publisher
    ros2 run navi_nav2 fixture_seed_publisher --ros-args -p bench_frames:=true

bench_frames publishes a static map->odom and a 20 Hz odom->base_footprint,
and /localization/odom_local to match.  NEVER run it with the ZED wrapper
up: the wrapper owns map->odom, and base_footprint would get a second
parent - the exact tree split SP6's launch file goes out of its way to
avoid.  It is false by default and start_navi.sh never sets it.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from navi_localization.odom_local import BASE_FRAME, ODOM_FRAME
from navi_nav2 import fixture

SEED_TOPIC = '/autonomy/costmap_seed'
ODOM_TOPIC = '/localization/odom_local'


def latched_qos() -> QoSProfile:
    """Exactly the QoS traversability_layer publishes the seed with; a
    durability mismatch means the costmap gets no data at all."""
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class FixtureSeedPublisher(Node):

    def __init__(self):
        super().__init__('fixture_seed_publisher')
        self.declare_parameter('bench_frames', False)
        self.declare_parameter('robot_x', fixture.START[0])
        self.declare_parameter('robot_y', fixture.START[1])
        self.declare_parameter('republish_period_s', 2.0)

        self._publisher = self.create_publisher(
            OccupancyGrid, SEED_TOPIC, latched_qos())
        self._grid = fixture.occupancy_grid(self.get_clock().now().to_msg())
        self._publish_seed()
        # Latched already, but a costmap that starts late and misses the
        # transient_local sample on a busy domain is a five-minute mystery;
        # a slow republish costs nothing.
        self.create_timer(float(self.get_parameter('republish_period_s').value),
                          self._publish_seed)

        if bool(self.get_parameter('bench_frames').value):
            self.get_logger().warn(
                "bench_frames: faking map->odom, odom->base_footprint and "
                f"{ODOM_TOPIC}. Never run this with the ZED wrapper up.")
            self._static = StaticTransformBroadcaster(self)
            self._static.sendTransform(self._identity('map', ODOM_FRAME))
            self._tf = TransformBroadcaster(self)
            self._odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, 10)
            self.create_timer(0.05, self._publish_pose)

    def _publish_seed(self):
        self._grid.header.stamp = self.get_clock().now().to_msg()
        self._grid.info.map_load_time = self._grid.header.stamp
        self._publisher.publish(self._grid)

    def _identity(self, parent: str, child: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform

    def _publish_pose(self):
        x = float(self.get_parameter('robot_x').value)
        y = float(self.get_parameter('robot_y').value)
        stamp = self.get_clock().now().to_msg()

        transform = self._identity(ODOM_FRAME, BASE_FRAME)
        transform.header.stamp = stamp
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        self._tf.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = ODOM_FRAME
        odometry.child_frame_id = BASE_FRAME
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.w = 1.0
        self._odom_publisher.publish(odometry)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixtureSeedPublisher()
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
