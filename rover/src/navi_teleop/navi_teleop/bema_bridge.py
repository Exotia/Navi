"""The rover-side node that turns /manual_twist into real wheel commands.

It owns the timers; bema_session owns the protocol. A twist is forwarded
to the primary's IK at 20 Hz; if the stream stops for deadman_s the wheels
are zeroed and stopped, and kept stopped until a fresh twist arrives.
/drive_command (JSON) drives the coordinator/BEMA buttons the ground
station shows; /drive_status (JSON, 1 Hz) reports what is happening.

Nothing here calls init() or startManual() on its own - the rover only
moves after the operator presses a button.
"""

import json
from math import degrees
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from navi_teleop.bema_session import BemaSession

DRIVE_HZ = 20.0
STATUS_HZ = 1.0


def _default_session_factory(host, bema_port, coordinator_port, clock):
    session = BemaSession(host, bema_port, coordinator_port, clock=clock)
    session.connect()
    return session


class BemaBridge(Node):
    def __init__(self, session_factory=_default_session_factory,
                 clock=monotonic, parameter_overrides=None):
        super().__init__("bema_bridge",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("bema_host", "192.168.178.26")
        self.declare_parameter("bema_port", 21022)
        self.declare_parameter("coordinator_port", 21031)
        self.declare_parameter("deadman_s", 1.0)
        self.declare_parameter("twist_topic", "/manual_twist")

        self._deadman_s = float(self.get_parameter("deadman_s").value)
        self._twist = (0.0, 0.0, 0.0)
        self._twist_at = None
        self._deadman_active = True
        self._last_action = None

        self._session = session_factory(
            self.get_parameter("bema_host").value,
            int(self.get_parameter("bema_port").value),
            int(self.get_parameter("coordinator_port").value),
            clock)

        self.create_subscription(
            Twist, self.get_parameter("twist_topic").value, self._on_twist, 1)
        self.create_subscription(String, "/drive_command", self._on_command, 10)
        self._status_pub = self.create_publisher(String, "/drive_status", 1)
        self.create_timer(1.0 / DRIVE_HZ, self._drive_tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)

        # Store the clock after timers are created so node doesn't use test clock for timers
        self._clock = clock

    def _on_twist(self, msg: Twist):
        # w to the IK is degrees/second; the server negates again so a
        # positive angular.z (CCW) reaches the model as positive u.
        self._twist = (msg.linear.x, msg.linear.y, -degrees(msg.angular.z))
        self._twist_at = self._clock()

    def _drive_tick(self):
        try:
            now = self._clock()
            fresh = (self._twist_at is not None
                     and now - self._twist_at <= self._deadman_s)
            if fresh:
                self._deadman_active = False
                self._session.set_command(*self._twist)
                self._session.tick(now)
            else:
                if not self._deadman_active:
                    self._deadman_active = True
                    self._session.stop()
                self._session.set_command(0.0, 0.0, 0.0)
                self._session.tick(now)
        except Exception as exc:                     # never kill the node
            self.get_logger().error(f"drive tick failed: {exc!r}")

    def _on_command(self, msg: String):
        try:
            action = json.loads(msg.data).get("action")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable drive command: {msg.data!r}")
            return
        self._last_action = action
        table = {
            "stop": self._session.stop,
            "manual": self._session.start_manual,
            "init": self._session.init,
            "reset_encoders": self._session.reset_encoders,
            "reset_odometry": self._session.reset_odometry,
            "drive_mode": self._session.change_drive_mode,
            "drive_state": self._session.change_drive_state,
        }
        handler = table.get(action)
        if handler is None:
            self.get_logger().warn(f"unknown drive action: {action!r}")
            return
        try:
            handler()
        except Exception as exc:
            self.get_logger().error(f"drive action {action} failed: {exc!r}")

    def _status_tick(self):
        now = self._clock()
        status = dict(self._session.status())
        status["twist_age_s"] = (None if self._twist_at is None
                                 else round(now - self._twist_at, 2))
        status["deadman_active"] = self._deadman_active
        status["last_action"] = self._last_action
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._session.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = BemaBridge()
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
