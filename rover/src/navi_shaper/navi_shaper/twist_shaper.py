"""The feasibility clamp, as a node.

Subscribes /rover_twist (mode_supervisor is its only publisher - see SP5),
shapes each message with navi_shaper.shaper, and publishes the result on
/chassis_twist, which is what bema_bridge consumes. Diagnostics go out on
/ik_feasibility as JSON in a std_msgs/String, the convention /mode_status,
/drive_status and /localization/status already follow, so the ground station
can read it over rosbridge with no ROS installed.

The shaping happens synchronously in the subscription callback. There is no
timer on the twist path: this node is a transform, not a resampler, and adding
a tick would add up to a tick of latency to the e-stop's zero stream for no
benefit. One message in, one message out, in the same callback.

If this node dies, /chassis_twist goes silent and bema_bridge's own 1 s
deadman stops the rover. That is the intended failure mode and it is why
start_navi.sh starts this before the bridge.
"""
import json
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from navi_shaper.shaper import ShaperConfig, TwistShaper

STATUS_HZ = 2.0


class TwistShaperNode(Node):

    def __init__(self, clock=monotonic, parameter_overrides=None):
        super().__init__("twist_shaper",
                         parameter_overrides=parameter_overrides or [])
        defaults = ShaperConfig()
        self.declare_parameter("min_gain", defaults.min_gain)
        self.declare_parameter("icr_fidelity_tol_rad", defaults.icr_fidelity_tol_rad)
        self.declare_parameter("backstop_max_vx", defaults.backstop_max_vx)
        self.declare_parameter("backstop_max_wz", defaults.backstop_max_wz)
        self.declare_parameter("max_dt_s", defaults.max_dt_s)
        self.declare_parameter("input_topic", "/rover_twist")
        self.declare_parameter("output_topic", "/chassis_twist")

        # NOT self._clock: rclpy.node.Node already owns that name, and
        # create_timer() defaults clock=self._clock - overwriting it makes
        # every timer raise AttributeError on a plain callable. The same trap
        # SP5 documented in mode_supervisor.
        self._now = clock
        self._last_stamp = None
        self._shaper = TwistShaper(self._config())
        self._last_result = None
        self._shaped_count = 0

        out_topic = str(self.get_parameter("output_topic").value)
        self._twist_pub = self.create_publisher(Twist, out_topic, 1)
        self._status_pub = self.create_publisher(String, "/ik_feasibility", 1)
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_twist, 1)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)

        self.get_logger().info(
            f"twist_shaper: {self.get_parameter('input_topic').value} -> {out_topic}; "
            f"backstop {self.get_parameter('backstop_max_vx').value} m/s / "
            f"{self.get_parameter('backstop_max_wz').value} rad/s")

    def _config(self) -> ShaperConfig:
        return ShaperConfig(
            min_gain=float(self.get_parameter("min_gain").value),
            icr_fidelity_tol_rad=float(self.get_parameter("icr_fidelity_tol_rad").value),
            backstop_max_vx=float(self.get_parameter("backstop_max_vx").value),
            backstop_max_wz=float(self.get_parameter("backstop_max_wz").value),
            max_dt_s=float(self.get_parameter("max_dt_s").value),
        )

    def _on_twist(self, msg: Twist):
        # Parameters are read per message so a live `ros2 param set` takes
        # effect at once; the shaper's hold state is carried across, because
        # the chassis does not reset when a parameter does.
        self._shaper.config = self._config()

        now = self._now()
        dt = 0.0 if self._last_stamp is None else max(0.0, now - self._last_stamp)
        self._last_stamp = now

        result = self._shaper.shape(msg.linear.x, msg.linear.y, msg.angular.z, dt)
        self._last_result = result
        if result.gain < 1.0:
            self._shaped_count += 1

        out = Twist()
        out.linear.x = result.vx
        out.linear.y = result.vy
        out.angular.z = result.wz
        # linear.z and angular.x/y are left at zero: the chassis has no such
        # degrees of freedom and mode_supervisor never populates them.
        self._twist_pub.publish(out)

    def _publish_status(self):
        r = self._last_result
        payload = {
            "gain": 1.0 if r is None else round(r.gain, 6),
            "feasible": True if r is None else bool(r.feasible),
            "limited_by": "none" if r is None else r.limited_by,
            # What else was pushing the gain down when the backstop overrode
            # it. "backstop" alone would hide whether the slew policy or the
            # fidelity guard was also active, and ruling 4 justifies
            # limited_by as "the whole content of the diagnostic".
            "also_limited_by": "none" if r is None else r.also_limited_by,
            # The geometry error the shaped output actually carries. The
            # backstop is applied after the fidelity bisection and is allowed
            # to exceed icr_fidelity_tol_rad (see the ordering rule in
            # shaper.py), so this is the only place the true error is visible.
            "fidelity_err_rad": 0.0 if r is None else round(r.fidelity_err_rad, 6),
            "delta_beta_rad": 0.0 if r is None else round(r.delta_beta_rad, 6),
            "hold_remaining_s": 0.0 if r is None else round(r.hold_remaining_s, 3),
            "icr_x": 0.0 if r is None else round(r.icr[0], 4),
            "icr_y": 0.0 if r is None else round(r.icr[1], 4),
            "straight_bias_rad_s": 0.0 if r is None else round(r.straight_bias_rad_s, 6),
            "shaped_count": self._shaped_count,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistShaperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
