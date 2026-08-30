"""Publishes the rover's pose and the health of its localisation.

Input is the ZED ROS 2 wrapper's positional tracking; output is the pose of
base_footprint in the map frame plus an OK / SEARCHING / OFF status, and the
wrapper's own odometry re-expressed at base_footprint on
/localization/odom_local for Nav2's odom_topic. See tracker.py for the rules,
pose_composition.py for the frame arithmetic and odom_local.py for the odom
stream.

Status goes out as JSON in a std_msgs/String, the convention /video_status
set: the ground station reads it over rosbridge and a custom .msg would cost
an ament_cmake package for one message.

The magnetometer is deliberately not subscribed - a project decision, not an
oversight. Heading comes from the visual-inertial tracking only.

zed_msgs/msg/PosTrackStatus (checked on the Orin: `ros2 interface show
zed_msgs/msg/PosTrackStatus`) defines odometry_status constants OK=0,
UNAVAILABLE=1, LOOP_CLOSED=1, SEARCHING=2, OFF=3. Only OK means the pose is
trustworthy; everything else - including LOOP_CLOSED, which is a
spatial_memory_status value reused here by name collision, not something
odometry_status itself reports - maps to tracking_ok=False.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from zed_msgs.msg import PosTrackStatus

from navi_localization.pose_composition import (
    CAMERA_IN_BASE_FOOTPRINT, MOUNT_OFFSET_VERIFIED, Transform,
    footprint_pose_from_camera_pose)
from navi_localization.odom_local import odom_local_from_camera_odometry
from navi_localization.tracker import LocalizationTracker


class LocalizationStatus(Node):

    def __init__(self) -> None:
        super().__init__('localization_status')
        self.declare_parameter('off_after_seconds', 2.0)
        self.declare_parameter('status_interval_seconds', 0.5)

        self._tracker = LocalizationTracker(
            off_after_seconds=float(self.get_parameter('off_after_seconds').value))
        self._tracking_ok = False
        self._covariance = [0.0] * 36
        self._last_published_state = None

        self._pose_publisher = self.create_publisher(Odometry, '/localization/pose', 10)
        self._status_publisher = self.create_publisher(String, '/localization/status', 10)
        self._odom_local_publisher = self.create_publisher(
            Odometry, '/localization/odom_local', 10)
        self.create_subscription(PoseStamped, '/zed_front/zed_node/pose', self._on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 '/zed_front/zed_node/pose_with_covariance',
                                 self._on_covariance, 10)
        self.create_subscription(PosTrackStatus, '/zed_front/zed_node/pose/status',
                                 self._on_status, 10)
        self.create_subscription(Odometry, '/zed_front/zed_node/odom',
                                 self._on_odom, 10)
        self.create_timer(float(self.get_parameter('status_interval_seconds').value),
                          self._tick)

        c = CAMERA_IN_BASE_FOOTPRINT
        self.get_logger().info(
            f"camera mount offset in base_footprint: ({c.x}, {c.y}, {c.z}) - "
            + ("verified" if MOUNT_OFFSET_VERIFIED else
               "UNVERIFIED - a re-measurement disagreed with the URDF"))
        self._publish_status(force=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_status(self, msg: PosTrackStatus) -> None:
        # Only odometry_status matters here: spatial_memory_status reports
        # the loop-closure side, which being unavailable does not make the
        # pose wrong.
        self._tracking_ok = (msg.odometry_status == PosTrackStatus.OK)

    def _on_covariance(self, msg: PoseWithCovarianceStamped) -> None:
        self._covariance = list(msg.pose.covariance)

    def _on_pose(self, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        camera_in_map = Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._tracker.on_pose(self._now(), footprint_pose_from_camera_pose(camera_in_map),
                              stamp, self._tracking_ok)
        self._publish_pose()
        self._publish_status()

    def _on_odom(self, msg: Odometry) -> None:
        # Deliberately not gated on the tracker, unlike /localization/pose.
        # odom is the continuous, non-jumping frame Nav2's controller reads
        # for velocity feedback; freezing it while the map pose is SEARCHING
        # would say the rover had stopped when it has not. And nothing is
        # repeated on the timer either: a stale twist says the same thing.
        self._odom_local_publisher.publish(odom_local_from_camera_odometry(msg))

    def _tick(self) -> None:
        self._tracker.on_tick(self._now())
        if self._tracker.state == LocalizationTracker.OFF:
            # While OFF no wrapper pose is arriving, so _on_pose - the only
            # other publisher - never runs, and /localization/pose goes
            # silent. tracker.py's rule is that the last good pose stays on
            # offer with its stamp frozen, and a consumer that joins while
            # the wrapper is down can only see it if something repeats it.
            # Nothing is invented: this is the same pose with the same old
            # stamp, and the status alongside says OFF.
            self._publish_pose()
        self._publish_status(force=True)

    def _publish_pose(self) -> None:
        published = self._tracker.pose_to_publish()
        if published is None:
            return
        pose, stamp = published
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp.sec = int(stamp)
        odom.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = pose.z
        odom.pose.pose.orientation.x = pose.qx
        odom.pose.pose.orientation.y = pose.qy
        odom.pose.pose.orientation.z = pose.qz
        odom.pose.pose.orientation.w = pose.qw
        odom.pose.covariance = self._covariance
        self._pose_publisher.publish(odom)

    def _publish_status(self, force: bool = False) -> None:
        state = self._tracker.state
        if not force and state == self._last_published_state:
            return
        if state != self._last_published_state:
            self.get_logger().info(f"localisation {state}")
        self._last_published_state = state
        msg = String()
        msg.data = self._tracker.status_json(self._now())
        self._status_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationStatus()
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
