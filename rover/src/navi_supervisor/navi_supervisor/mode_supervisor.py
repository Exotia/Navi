"""The sole publisher of /rover_twist: mode, arbitration and the deadman.

Sources are /manual_twist (in manual and semi_auto) and /autonomy_twist
(in autonomous); /mode_request and /estop_request steer it, and
/mode_status reports it - all JSON in a std_msgs/String, the convention
/drive_status and /localization/status already set, so the ground station
reads them over rosbridge with no custom message type and no ROS.

The rules live in supervisor_state.py, which has no ROS in it. This file
owns the timers, the topics, and the two side-effect channels:
/drive_command, which bema_bridge already owns the RPC session for, and a
Nav2Control that is a stub until SP9. The supervisor deliberately does not
open its own connection to the primary - a second msgpack client would
fight bema_bridge for the same exclusive lease.

Nothing else may publish /rover_twist. bema_bridge subscribes to it and
keeps its own 1 s deadman on top of this one's: two deadmen in series is
deliberate, this one against Nav2 hanging, that one against the whole
Orin-side graph dying.
"""

import json
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from navi_supervisor import supervisor_state as rules
from navi_supervisor.nav2_control import NullNav2Control
from navi_supervisor.supervisor_state import SupervisorState

PUBLISH_HZ = 20.0
STATUS_HZ = 2.0


class ModeSupervisor(Node):

    def __init__(self, clock=monotonic, nav2_control=None,
                 parameter_overrides=None):
        super().__init__("mode_supervisor",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("manual_deadman_s", rules.MANUAL_DEADMAN_S)
        self.declare_parameter("autonomy_deadman_s", rules.AUTONOMY_DEADMAN_S)
        self.declare_parameter("start_mode", rules.MANUAL)

        # NOT self._clock: rclpy.node.Node already owns that name, and
        # create_timer() defaults clock=self._clock - overwriting it makes
        # every timer raise AttributeError on a plain callable.
        self._now = clock
        self._nav2 = (nav2_control if nav2_control is not None
                      else NullNav2Control(self.get_logger()))
        self._state = SupervisorState(
            mode=str(self.get_parameter("start_mode").value),
            manual_deadman_s=float(self.get_parameter("manual_deadman_s").value),
            autonomy_deadman_s=float(self.get_parameter("autonomy_deadman_s").value))

        self._twist_pub = self.create_publisher(Twist, "/rover_twist", 1)
        self._status_pub = self.create_publisher(String, "/mode_status", 1)
        self._command_pub = self.create_publisher(String, "/drive_command", 10)

        self.create_subscription(Twist, "/manual_twist", self._on_manual_twist, 1)
        self.create_subscription(Twist, "/autonomy_twist", self._on_autonomy_twist, 1)
        self.create_subscription(String, "/mode_request", self._on_mode_request, 10)
        self.create_subscription(String, "/estop_request", self._on_estop_request, 10)
        self.create_subscription(String, "/localization/status",
                                 self._on_localization_status, 10)

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)

    def attach_nav2_control(self, nav2_control):
        """Replace the Nav2 hook after construction.

        RosNav2Control needs this node to create its service clients, so it
        cannot be passed to __init__.  The constructor default stays
        NullNav2Control: a supervisor built by a test, or by anything that
        has no Nav2, must still record what it asked for.
        """
        self._nav2 = nav2_control

    # --- inputs ----------------------------------------------------------
    def _on_manual_twist(self, msg: Twist):
        try:
            before = self._state.mode
            self._state.on_manual_twist(self._now(), msg.linear.x,
                                        msg.linear.y, msg.angular.z)
            self._run_actions()
            if self._state.mode != before:
                # A takeover changes the mode, and /mode_status is the
                # ground station's publish gate: waiting for the 2 Hz tick
                # would leave the operator's sticks ignored for up to
                # 500 ms. Contract is "2 Hz and on every change" - hence
                # the guard, so a 20 Hz manual stream does not turn this
                # into a 20 Hz status stream over the field link.
                self._publish_status()
        except Exception as exc:                     # never kill the node
            self.get_logger().error(f"manual twist callback failed: {exc!r}")

    def _on_autonomy_twist(self, msg: Twist):
        try:
            self._state.on_autonomy_twist(self._now(), msg.linear.x,
                                          msg.linear.y, msg.angular.z)
        except Exception as exc:
            self.get_logger().error(f"autonomy twist callback failed: {exc!r}")

    def _on_mode_request(self, msg: String):
        try:
            mode = json.loads(msg.data).get("mode")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable mode request: {msg.data!r}")
            return
        try:
            refusal = self._state.on_mode_request(self._now(), mode)
            if refusal is not None:
                self.get_logger().warn(f"mode request refused: {refusal}")
            self._run_actions()
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"mode request failed: {exc!r}")

    def _on_estop_request(self, msg: String):
        # Deliberately not gated on the parse: an e-stop whose payload will
        # not read is still an e-stop. The JSON is consulted only to
        # recover a reason for /mode_status.
        reason = "e-stop"
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict) and payload.get("reason"):
                reason = str(payload["reason"])
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(
                f"unreadable e-stop payload, stopping anyway: {msg.data!r}")
        try:
            self._state.on_estop_request(self._now(), reason)
            self._run_actions()
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"e-stop failed: {exc!r}")

    def _on_localization_status(self, msg: String):
        # A status that will not parse is left alone rather than treated as
        # a loss: rule 3 names two states, and a garbled message says
        # nothing about which one the localisation is in. The autonomy
        # deadman still covers a Nav2 that reacts to the same fault.
        try:
            state = json.loads(msg.data).get("state")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable localisation status: {msg.data!r}")
            return
        try:
            self._state.on_localization_status(self._now(), state)
            self._run_actions()
            # Same contract as above, and the same urgency: a localisation
            # halt drops autonomous to manual precisely so the operator can
            # drive the rover out, which needs the gate open now, not at
            # the next tick. Rule 3 also names the reason in /mode_status.
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"localisation status failed: {exc!r}")

    # --- outputs ---------------------------------------------------------
    def _run_actions(self):
        actions = self._state.take_actions()
        if not actions:
            return
        reason = self._state.status(self._now())["reason"]
        for action in actions:
            try:
                if action == rules.CANCEL_GOAL:
                    self._nav2.cancel_goal(reason)
                elif action == rules.DEACTIVATE_NAV2:
                    self._nav2.deactivate(reason)
                elif action == rules.COORDINATOR_ABORT:
                    self._send_command("abort")
                elif action == rules.COORDINATOR_MANUAL:
                    self._send_command("manual")
                elif action == rules.CHASSIS_STOP:
                    self._send_command("stop")
                else:
                    self.get_logger().warn(f"unknown supervisor action: {action!r}")
            except Exception as exc:
                self.get_logger().error(f"supervisor action {action} failed: {exc!r}")

    def _send_command(self, action: str):
        msg = String()
        msg.data = json.dumps({"action": action})
        self._command_pub.publish(msg)

    def _publish_tick(self):
        try:
            now = self._now()
            vx, vy, wz = self._state.output(now)
            # output() is what notices the live -> stopped edge, so the
            # chassis stop it queues is drained here, before the zero goes
            # out rather than a tick later.
            self._run_actions()
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(wz)
            self._twist_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"publish tick failed: {exc!r}")

    def _status_tick(self):
        try:
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"status tick failed: {exc!r}")

    def _publish_status(self):
        msg = String()
        # default=str for the same reason /drive_status uses it: one odd
        # field must not black out the whole status the operator reads.
        msg.data = json.dumps(self._state.status(self._now()), default=str)
        self._status_pub.publish(msg)


def main():
    # Imported here, not at module scope: ros_nav2_control pulls in
    # nav2_msgs and action_msgs, and mode_supervisor must stay importable -
    # and runnable with NullNav2Control - on a box that has neither.  It
    # also keeps those two packages out of the import path of all 49
    # existing SP5 tests (11 in test_mode_supervisor.py, 38 in
    # test_supervisor_state.py).
    from navi_supervisor.ros_nav2_control import RosNav2Control

    rclpy.init()
    node = ModeSupervisor()
    # The stub is the constructor default so tests and Nav2-less bringups
    # keep working; a real run talks to a real Nav2.
    node.attach_nav2_control(RosNav2Control(node))
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
