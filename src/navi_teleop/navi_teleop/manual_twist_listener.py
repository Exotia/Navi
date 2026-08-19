"""Rover-side consumer of the ground station's manual drive stream.

The ground station publishes gamepad sticks as geometry_msgs/Twist on
/manual_twist over rosbridge. This node is the first thing on the rover
that actually listens to it: it mirrors the ground station's Drive card
(velocities, incoming rate, staleness) in the rover's own log, so a
"the ground station says it is sending" claim can be checked against
what the rover really receives.

It deliberately does NOT republish to /cmd_vel. Deciding whether manual
input or an autonomy source drives the wheels is the later
mode-supervisor's job; keeping that out of here means running this node
can never move the rover.
"""

from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ManualTwistListener(Node):

    def __init__(self) -> None:
        super().__init__('manual_twist_listener')

        self.declare_parameter('topic', '/manual_twist')
        self.declare_parameter('rate_window_seconds', 2.0)
        self.declare_parameter('stale_after_seconds', 1.0)
        self.declare_parameter('report_interval_seconds', 1.0)

        self._topic = self.get_parameter('topic').value
        self._rate_window = float(self.get_parameter('rate_window_seconds').value)
        self._stale_after = float(self.get_parameter('stale_after_seconds').value)
        report_interval = float(self.get_parameter('report_interval_seconds').value)

        self._latest: Twist | None = None
        self._latest_at: float | None = None
        self._timestamps: list[float] = []
        # Starts True so a listener that never receives anything reports the
        # no-data state once, instead of staying silent and looking healthy.
        self._was_stale = True

        # Depth 10, default reliable QoS - matches what rosbridge_server
        # publishes with, so the subscription actually matches.
        self.create_subscription(Twist, self._topic, self._on_twist, 10)
        self.create_timer(report_interval, self._report)

        self.get_logger().info(f"listening for manual Twist on {self._topic}")

    def _on_twist(self, msg: Twist) -> None:
        now = monotonic()
        self._latest = msg
        self._latest_at = now
        self._timestamps.append(now)
        cutoff = now - self._rate_window
        self._timestamps = [t for t in self._timestamps if t >= cutoff]

        if self._was_stale:
            self._was_stale = False
            self.get_logger().info(f"manual Twist stream live on {self._topic}")

    @property
    def rate_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / span

    def _report(self) -> None:
        now = monotonic()
        if self._latest is None or self._latest_at is None:
            return

        age = now - self._latest_at
        if age > self._stale_after:
            if not self._was_stale:
                self._was_stale = True
                self.get_logger().warn(
                    f"no manual Twist for {age:.1f}s on {self._topic} - stream stale"
                )
            return

        # Drop stale timestamps even when no message arrived, so the reported
        # rate decays instead of freezing at its last value.
        self._timestamps = [t for t in self._timestamps if t >= now - self._rate_window]
        self.get_logger().info(
            f"vx={self._latest.linear.x:+.2f} vy={self._latest.linear.y:+.2f} "
            f"wz={self._latest.angular.z:+.2f}  {self.rate_hz:.1f} Hz"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualTwistListener()
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
