"""The rover-side node that turns /rover_twist into real wheel commands.

It owns the timers; bema_session owns the protocol. A twist is forwarded
to the primary's IK at 20 Hz; if the stream stops for deadman_s the wheels
are zeroed and stopped, and kept stopped until a fresh twist arrives.
/drive_command (JSON) drives the coordinator/BEMA buttons the ground
station shows; /drive_status (JSON, 1 Hz) reports what is happening.
navi_rpc_server uses the same topic to send the coordinator its waypoint
progress, because F8 is unguarded and needs no lease; navi_task/pause_task/
resume_task go the other way, to the coordinator's guarded F0/F4/F5, and
are how an autonomous run is armed at all.

The source is /rover_twist, which mode_supervisor is the only publisher
of - never /manual_twist directly, or the operator's stream would reach
the wheels around the arbitration and the e-stop. `twist_topic` can point
this elsewhere for a bench test; the default is the safe wiring, and
start_navi.sh passes it explicitly anyway so the wiring can be read at the
launch site.

This node's own 1 s deadman is kept even though the supervisor has one:
two in series is deliberate - the supervisor's protects against Nav2
hanging, this one against the whole Orin-side graph dying.

Nothing here calls init() or startManual() on its own - the rover only
moves after the operator presses a button.
"""

import json
from math import degrees, isfinite
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
    _TASK_TAGS = (0x31, 0x32)
    _MAX_WAYPOINTS = 64

    @staticmethod
    def _waypoints(value):
        """[[x, y, w], ...] of finite non-bool numbers, or None.

        The same shape navi_rpc_state.parse_targets enforces on the way in
        from the coordinator, applied here on the way back out to it: this
        is a JSON topic anyone on the graph can publish to, and F0 arms an
        autonomous run.
        """
        if isinstance(value, (str, bytes)) or not isinstance(value, list):
            return None
        if not value or len(value) > BemaBridge._MAX_WAYPOINTS:
            return None
        out = []
        for point in value:
            if isinstance(point, (str, bytes)) or not isinstance(point, list) \
                    or len(point) != 3:
                return None
            row = []
            for component in point:
                if isinstance(component, bool) \
                        or not isinstance(component, (int, float)) \
                        or not isfinite(float(component)):
                    return None
                row.append(float(component))
            out.append(row)
        return out

    def __init__(self, session_factory=_default_session_factory,
                 clock=monotonic, parameter_overrides=None):
        super().__init__("bema_bridge",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("bema_host", "192.168.178.26")
        self.declare_parameter("bema_port", 21022)
        self.declare_parameter("coordinator_port", 21031)
        self.declare_parameter("deadman_s", 1.0)
        self.declare_parameter("twist_topic", "/rover_twist")

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

        self._clock = clock

    def _on_twist(self, msg: Twist):
        try:
            # w to the IK is degrees/second; the server negates again so a
            # positive angular.z (CCW) reaches the model as positive u.
            self._twist = (msg.linear.x, msg.linear.y, -degrees(msg.angular.z))
            self._twist_at = self._clock()
        except Exception as exc:
            self.get_logger().error(f"twist callback failed: {exc!r}")

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
            payload = json.loads(msg.data)
            action = payload.get("action")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable drive command: {msg.data!r}")
            return
        if action == "task_finished":
            # navi_rpc_server's progress, on its way to the coordinator's F8.
            # The tag is whitelisted: TAG_WaypointReached (0x31) and
            # TAG_DestinationReached (0x32) are the only two the coordinator
            # acts on, and an arbitrary one would drive a state machine we do
            # not own.
            tag = payload.get("tag")
            if not isinstance(tag, int) or isinstance(tag, bool) \
                    or tag not in self._TASK_TAGS:
                self.get_logger().warn(f"refusing task_finished tag {tag!r}")
                return
            self._last_action = action
            try:
                self._session.notify_task_finished(tag)
            except Exception as exc:
                self.get_logger().error(f"task_finished failed: {exc!r}")
            return
        if action == "navi_task":
            # The operator's Go, on its way to the coordinator's guarded F0.
            waypoints = self._waypoints(payload.get("waypoints"))
            if waypoints is None:
                self.get_logger().warn(
                    f"refusing navi_task waypoints {payload.get('waypoints')!r}")
                return
            self._last_action = action
            try:
                self._session.start_navi_task(waypoints)
            except Exception as exc:
                self.get_logger().error(f"navi_task failed: {exc!r}")
            return
        table = {
            "stop": self._session.stop,
            "manual": self._session.start_manual,
            "abort": self._session.abort,
            "init": self._session.init,
            "reset_encoders": self._session.reset_encoders,
            "reset_odometry": self._session.reset_odometry,
            "drive_mode": self._session.change_drive_mode,
            "drive_state": self._session.change_drive_state,
            "pause_task": self._session.pause_task,
            "resume_task": self._session.resume_task,
        }
        handler = table.get(action)
        if handler is None:
            self.get_logger().warn(f"unknown drive action: {action!r}")
            return
        # Only recorded once the action is a real one - an unknown action
        # must not be reported on /drive_status as the last command.
        self._last_action = action
        if action == "stop":
            # Latch the GS STOP: clear the retained twist and its timestamp
            # so the deadman re-arms immediately. Without this, a stop sent
            # while a fresh twist is still arriving would be overwritten by
            # the very next _drive_tick (which still sees that twist as
            # fresh) and the rover would resume driving on its own. Zeros
            # keep flowing until a genuinely new twist un-latches it.
            self._twist = (0.0, 0.0, 0.0)
            self._twist_at = None
        try:
            handler()
        except Exception as exc:
            self.get_logger().error(f"drive action {action} failed: {exc!r}")

    def _status_tick(self):
        try:
            now = self._clock()
            status = dict(self._session.status())
            status["twist_age_s"] = (None if self._twist_at is None
                                     else round(now - self._twist_at, 2))
            status["deadman_active"] = self._deadman_active
            status["last_action"] = self._last_action
            msg = String()
            # default=str: an odd (e.g. non-int) F9 value must not blackout
            # the whole of /drive_status just because one field in it isn't
            # natively JSON-serialisable.
            msg.data = json.dumps(status, default=str)
            self._status_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"status tick failed: {exc!r}")

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
