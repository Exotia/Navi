"""twist_odometry: dead-reckoned odometry from what the rover was TOLD to do.

The ZED's visual-inertial tracking is the rover's only localisation, and it
is gone the moment the sun saturates the frame (see navi_autonomy.glare and
glare_watch.py). True wheel odometry is planned but gated on another team's
RPC access to the BEMA drive bridge, so this node is the honest interim: it
integrates /chassis_twist - the command AFTER twist_shaper's feasibility
clamp, so it is the closest statement of what the wheels actually attempt -
rather than /rover_twist, the pre-clamp request that may ask for motion the
chassis will refuse. Heading comes from the ZED's own gyro rather than from
the commanded rotation, because yaw error dominates dead reckoning distance
error and a consumer MEMS gyro drifts only a few degrees a minute while
integrating commanded wz drifts with every scrub, stall or slope the wheels
meet that the command never saw. None of this can see wheel slip. At the
rover's 0.2 m/s crawl on firm ground it stays honest for tens of seconds,
which is exactly the sun-blindness horizon this exists to cover, and it is
the odometry input a future EKF will fuse alongside the ZED when both are
available.

This node publishes NO tf. The ZED wrapper's zed_node already owns the
odom -> base_footprint transform (see localization.launch.py); broadcasting
a second one from here would leave two nodes fighting over the same edge of
the tf tree, each silently overwriting the other's latest transform. The
next person wiring this into an EKF will be tempted to add a
TransformBroadcaster "just to see it move" - don't; publish the topic and
let robot_localization or a static remap own tf, not this node.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization:$PWD/rover/src/navi_autonomy \
    python3 -m pytest rover/src/navi_localization/test/test_twist_odometry.py -q'
"""

import math
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

# A consumer MEMS gyro (the ZED's own IMU) drifts on the order of a few
# degrees a minute with nothing to correct it; this is the middle of that
# range, used only to shape the heading covariance's growth, never to
# correct the integration itself - there is no correction source here, that
# is the whole reason an EKF exists downstream.
HEADING_DRIFT_RATE_RAD_S = math.radians(3.0) / 60.0

# Simple linear-in-distance model: at 0.05 m per metre travelled, one metre
# of unslipped driving costs 5 cm of 1-sigma uncertainty. This is not a
# measured slip constant, it is a placeholder honest enough that a future
# EKF sees covariance actually grow instead of a frozen "trust me" diagonal;
# tightening it against real slip data is exactly the kind of follow-up true
# wheel odometry will make possible.
POSITION_DRIFT_PER_METRE = 0.05


class TwistIntegrator:
    """Dead-reckons a 2-D pose from body-frame commanded velocity and a
    separately supplied heading rate.

    This class has no ROS imports and knows nothing about topics, IMUs or
    timeouts beyond command_timeout_s - it only integrates whatever velocity
    and angular rate it is handed, whenever it is handed them, which is what
    keeps it fully testable without a running node.
    """

    def __init__(self, command_timeout_s=0.5):
        self._command_timeout_s = command_timeout_s
        self._vx = 0.0
        self._vy = 0.0
        self._last_command_t = None
        self._last_step_t = None
        self._last_gyro_t = None
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.distance_travelled = 0.0

    def _active_velocity(self, t):
        # The BEMA drive bridge has its own 1 s deadman that stops the
        # wheels the moment commands stop arriving; if this integrator kept
        # sailing on the last twist forever it would report driving through
        # a rover the bridge has already halted. Zeroing here just matches
        # that ground truth rather than outrunning it - command_timeout_s
        # (0.5 s default) is deliberately tighter than the bridge's own 1 s
        # so odometry never claims motion after the bridge would have acted.
        if (self._last_command_t is None
                or t - self._last_command_t > self._command_timeout_s):
            return 0.0, 0.0
        return self._vx, self._vy

    def _advance_heading(self, wz, t):
        """Rectangular (Euler) integration: heading += wz * dt, the rate
        held constant across the interval since the last gyro() call,
        rather than trapezoidal averaging against the previous sample. At
        the 20-25 Hz this runs at on a rover that tops out well under
        1 rad/s of yaw rate, the true rate cannot bend enough within one
        ~40 ms tick for the trapezoid's extra averaging term to move the
        integrated heading by anything the ZED's own gyro noise does not
        already dwarf, and rectangular needs no memory of the previous wz
        sample - one fewer thing to get wrong across a switch between the
        real IMU and the commanded-wz fallback.

        This tracks its own last-call time (_last_gyro_t) rather than
        sharing _last_step_t with _advance_pose: command() and gyro() are
        called back to back at the same t from the node's timer, and if
        heading integration shared that clock a same-timestamp gyro() call
        would see dt == 0 and silently never turn - exactly the bug this
        split avoids.
        """
        if self._last_gyro_t is None:
            self._last_gyro_t = t
            return
        dt = t - self._last_gyro_t
        self._last_gyro_t = t
        if dt <= 0.0:
            return
        self.heading += wz * dt

    def _advance_pose(self, t):
        """Advance x, y and distance_travelled across the time elapsed
        since the previous command() or gyro() call, using whatever
        velocity is currently in force and the heading as of right now -
        so a gyro() call that just turned the heading is reflected in the
        very same step's position update."""
        if self._last_step_t is None:
            self._last_step_t = t
            return
        dt = t - self._last_step_t
        self._last_step_t = t
        if dt <= 0.0:
            # Out-of-order or duplicate timestamps: nothing elapsed to
            # integrate, and a negative dt would run the pose backwards.
            return

        vx, vy = self._active_velocity(t)
        h = self.heading
        dx = (vx * math.cos(h) - vy * math.sin(h)) * dt
        dy = (vx * math.sin(h) + vy * math.cos(h)) * dt
        self.x += dx
        self.y += dy
        self.distance_travelled += math.hypot(dx, dy)

    def command(self, vx, vy, t):
        """Record the body-frame velocity commanded at time t, advancing
        the pose first using whatever velocity was in force up to now."""
        self._advance_pose(t)
        self._vx = vx
        self._vy = vy
        self._last_command_t = t

    def gyro(self, wz, t):
        """Advance heading using the angular rate wz measured (or
        substituted) at time t, then advance the pose with that heading.
        The caller decides where wz comes from - this class has no notion
        of an IMU or a fallback."""
        self._advance_heading(wz, t)
        self._advance_pose(t)


class TwistOdometry(Node):

    def __init__(self, clock=monotonic):
        super().__init__('twist_odometry')
        self.declare_parameter('twist_topic', '/chassis_twist')
        self.declare_parameter('imu_topic', '/zed_front/zed_node/imu/data')
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('imu_timeout_s', 1.0)
        self.declare_parameter('odom_topic', '/odom/twist')
        self.declare_parameter('publish_rate_hz', 20.0)

        # Not self._clock: rclpy.node.Node already owns that name (see
        # glare_watch.py's identical note - overwriting it breaks every
        # timer this node creates, including the publish timer below).
        self._now = clock

        command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self._imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)
        self._integrator = TwistIntegrator(command_timeout_s=command_timeout_s)

        self._start_t = self._now()
        self._last_cmd_wz = 0.0
        self._last_imu_wz = 0.0
        self._last_imu_t = None
        # Set the first tick an outage is discovered, so the warning fires
        # once per outage rather than at the publish rate for as long as
        # the IMU stays silent - reset the moment fresh IMU data returns so
        # the next outage warns again. Same one-shot idiom as
        # glare_watch.py's _warned_bad_encoding.
        self._warned_imu_stale = False

        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter('twist_topic').value), self._on_twist, 10)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu, 10)

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_twist(self, msg: Twist) -> None:
        now = self._now()
        self._last_cmd_wz = msg.angular.z
        self._integrator.command(msg.linear.x, msg.linear.y, now)

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu_t = self._now()
        self._last_imu_wz = msg.angular_velocity.z
        self._warned_imu_stale = False

    def _on_timer(self) -> None:
        now = self._now()
        imu_fresh = (self._last_imu_t is not None
                     and now - self._last_imu_t <= self._imu_timeout_s)
        if imu_fresh:
            wz = self._last_imu_wz
        else:
            # Heading from the commanded rotation is worse than a real
            # gyro - it drifts with every scrub of the wheels - but it is
            # infinitely better than a heading that simply stops moving
            # while the rover keeps driving.
            wz = self._last_cmd_wz
            if not self._warned_imu_stale:
                self._warned_imu_stale = True
                self.get_logger().warn(
                    "ZED IMU silent for over imu_timeout_s; falling back to "
                    "commanded angular velocity for heading")
        self._integrator.gyro(wz, now)
        self._publish_odometry(now)

    def _publish_odometry(self, now: float) -> None:
        integrator = self._integrator
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Not "odom": that name is the ZED wrapper's own frame, and this
        # node publishes no tf connecting the two - a shared frame_id
        # without a shared transform would look connected on paper while
        # being two unrelated pose estimates in practice.
        msg.header.frame_id = 'twist_odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = integrator.x
        msg.pose.pose.position.y = integrator.y
        half = integrator.heading * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)

        msg.twist.twist.linear.x = integrator._vx
        msg.twist.twist.linear.y = integrator._vy
        msg.twist.twist.angular.z = (
            self._last_imu_wz if self._last_imu_t is not None
            and now - self._last_imu_t <= self._imu_timeout_s
            else self._last_cmd_wz)

        # Position uncertainty grows linearly with distance travelled -
        # this has no slip sensing, so every metre driven is a metre this
        # node cannot tell apart from a metre of slip. Heading uncertainty
        # grows with wall-clock time since start-up instead, because the
        # gyro integrates continuously whether or not the rover is moving.
        # Both are placeholders honest enough for a future EKF to see
        # covariance actually widen rather than a frozen diagonal; a real
        # slip and drift characterisation is follow-up work, not this
        # node's job.
        position_sigma = POSITION_DRIFT_PER_METRE * integrator.distance_travelled
        heading_sigma = HEADING_DRIFT_RATE_RAD_S * (now - self._start_t)
        cov = [0.0] * 36
        cov[0] = position_sigma ** 2   # x
        cov[7] = position_sigma ** 2   # y
        cov[35] = heading_sigma ** 2   # yaw
        msg.pose.covariance = cov

        self._odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistOdometry()
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
