"""The ZED wrapper's odometry, re-expressed at base_footprint.

Nav2's controller server and velocity smoother read an `odom_topic` for the
rover's current velocity. /localization/pose is the wrong thing for that: it
is in the `map` frame, which jumps whenever the SDK closes a loop, and a
controller that sees a jump reads it as a large velocity. The `odom` frame is
the continuous one, and the wrapper already publishes it - about a frame that
is not the rover.

So this module does to /zed_front/zed_node/odom exactly what
localization_status does to the wrapper's map pose: the same mount constant,
the same arithmetic, one frame lower down. Kept out of localization_status.py
so it can be tested on a machine without zed_msgs installed - that node
imports zed_msgs at module level, this does not.

The covariances are passed through untouched. The mount rotation is identity,
so no rotation of the blocks is called for; the lever arm does couple angular
uncertainty into the linear block, but that term is deliberately not
propagated because nothing reads it (Nav2 reads twist.twist, the collision
monitor reads cmd_vel), and a number nobody checks is worse than an honest
copy.
"""

from nav_msgs.msg import Odometry

from navi_localization.pose_composition import (
    CAMERA_IN_BASE_FOOTPRINT, Transform, footprint_pose_from_camera_pose,
    footprint_twist_from_camera_twist)

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'


def odom_local_from_camera_odometry(
        msg: Odometry,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT) -> Odometry:
    """A new Odometry: the same instant, at base_footprint. `msg` is not
    modified."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    footprint = footprint_pose_from_camera_pose(
        Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w), camera_in_footprint)

    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    linear, angular = footprint_twist_from_camera_twist(
        (v.x, v.y, v.z), (w.x, w.y, w.z), camera_in_footprint)

    out = Odometry()
    out.header.stamp.sec = msg.header.stamp.sec
    out.header.stamp.nanosec = msg.header.stamp.nanosec
    out.header.frame_id = ODOM_FRAME
    out.child_frame_id = BASE_FRAME
    out.pose.pose.position.x = footprint.x
    out.pose.pose.position.y = footprint.y
    out.pose.pose.position.z = footprint.z
    out.pose.pose.orientation.x = footprint.qx
    out.pose.pose.orientation.y = footprint.qy
    out.pose.pose.orientation.z = footprint.qz
    out.pose.pose.orientation.w = footprint.qw
    out.pose.covariance = list(msg.pose.covariance)
    out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = linear
    out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = angular
    out.twist.covariance = list(msg.twist.covariance)
    return out
