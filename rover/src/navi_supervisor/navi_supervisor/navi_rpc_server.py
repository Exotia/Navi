"""The endpoint the primary's coordinator calls "NaVi".

`CoordinatorImpl::startNaViTask` hands its waypoints to a msgpack-RPC server
at 192.168.178.18:21021, and drops straight back to Idle when it is not
there. This node is that server: the wire and the lease live in
navi_rpc_protocol.py, the contract in navi_rpc_state.py, and this file owns
the socket's lifetime, the timers, and the topics the rest of the graph
sees.

It never publishes /mode_request. The supervisor is the single authority on
mode (SP5): a `manual` request from here would clear a latched e-stop, and
would turn the coordinator's own pause - which calls F6 while it is still in
Autonomous - into a full abort. F6 stops the chassis and bumps `stop_seq` on
/navi_rpc/status instead; SP11's goal_relay watches that and cancels the Nav2
goal, which is what actually stops /autonomy_twist.

It binds 0.0.0.0 rather than the alias itself, so the node still starts (and
is still testable, and still reachable on .33) on a machine where
start_navi.sh could not add 192.168.178.18. The alias is what makes the
coordinator's hard-coded address land here; the bind is what makes it
answerable.

Nothing here starts a run. F4 startNavigation is the coordinator telling us
it has reached Autonomous, which is the arming signal SP11's goal_relay
waits for - runs themselves are operator-initiated (spec 3).

Calls arrive on socket threads, so the state queues actions and this node
drains them on a timer: an rclpy publisher may only be touched from the
executor's thread.
"""

import json
from math import cos, sin
from time import monotonic

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from navi_supervisor import navi_rpc_state as contract
from navi_supervisor.navi_rpc_protocol import RpcServer
from navi_supervisor.navi_rpc_state import (GUARDED_METHODS, NaviRpcState,
                                            navi_method_table)

ACTION_HZ = 20.0
STATUS_HZ = 2.0

# Latched: goal_relay and the ground station may both start after a run has
# been armed, and must not wait for the next setTargets to learn the route.
LATCHED = QoSProfile(depth=1,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def targets_to_path(targets, frame_id, stamp):
    """[(x, y, yaw)] -> nav_msgs/Path. yaw is radians about z, in `frame_id`."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for x, y, yaw in targets:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = sin(float(yaw) / 2.0)
        pose.pose.orientation.w = cos(float(yaw) / 2.0)
        path.poses.append(pose)
    return path


def _default_server_factory(state, host, port, clock, logger):
    server = RpcServer(navi_method_table(state, logger=logger),
                       guarded=GUARDED_METHODS,
                       host=host, port=port, clock=clock, logger=logger)
    server.start()
    return server


class NaviRpcServer(Node):

    def __init__(self, clock=monotonic, server_factory=_default_server_factory,
                 parameter_overrides=None):
        super().__init__("navi_rpc_server",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 21021)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("targets_topic", "/navi_rpc/targets")
        self.declare_parameter("status_topic", "/navi_rpc/status")
        self.declare_parameter("progress_topic", "/navi_rpc/progress")

        # NOT self._clock: rclpy.node.Node already owns that name.
        self._now = clock
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._state = NaviRpcState(clock=clock)

        self._targets_pub = self.create_publisher(
            Path, self.get_parameter("targets_topic").value, LATCHED)
        self._status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 1)
        self._command_pub = self.create_publisher(String, "/drive_command", 10)

        self.create_subscription(
            String, self.get_parameter("progress_topic").value,
            self._on_progress, 10)
        self.create_subscription(String, "/mode_status",
                                 self._on_mode_status, 10)

        # A failure to bind is fatal on purpose: a rover whose NaVi endpoint
        # is missing must not look healthy. The usual cause is a stale server
        # still holding :21021 - see start_navi.sh's cleanup.
        try:
            self._server = server_factory(
                self._state, str(self.get_parameter("bind_host").value),
                int(self.get_parameter("port").value), clock, self.get_logger())
        except OSError as exc:
            self.get_logger().error(
                f"could not bind {self.get_parameter('bind_host').value}:"
                f"{self.get_parameter('port').value} ({exc}); the coordinator "
                f"will report 'NaVi not reachable'")
            raise
        self.get_logger().info(
            f"navi_rpc_server is serving {self.listening}")

        self.create_timer(1.0 / ACTION_HZ, self._action_tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)

    @property
    def port(self):
        return self._server.port

    @property
    def listening(self):
        return f"{self._server.host}:{self._server.port}"

    # --- inputs ----------------------------------------------------------
    def _on_progress(self, msg: String):
        try:
            payload = json.loads(msg.data)
            event = payload.get("event")
            index = payload.get("index")
            reason = payload.get("reason")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable progress: {msg.data!r}")
            return
        try:
            self._state.on_progress(event, index=index, reason=reason)
            self._run_actions()
        except Exception as exc:                     # never kill the node
            self.get_logger().error(f"progress callback failed: {exc!r}")

    def _on_mode_status(self, msg: String):
        try:
            mode = json.loads(msg.data).get("mode")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable mode status: {msg.data!r}")
            return
        try:
            self._state.on_mode(mode)
        except Exception as exc:
            self.get_logger().error(f"mode status callback failed: {exc!r}")

    # --- outputs ---------------------------------------------------------
    def _run_actions(self):
        actions = self._state.take_actions()
        if not actions:
            return
        for action in actions:
            try:
                if action == contract.PUBLISH_TARGETS:
                    self._publish_targets()
                elif action == contract.CHASSIS_STOP:
                    self._send_command({"action": "stop"})
                elif action == contract.NOTIFY_WAYPOINT:
                    self._notify(contract.TAG_WAYPOINT_REACHED)
                elif action == contract.NOTIFY_DESTINATION:
                    self._notify(contract.TAG_DESTINATION_REACHED)
                else:
                    self.get_logger().warn(f"unknown rpc action: {action!r}")
            except Exception as exc:
                self.get_logger().error(f"rpc action {action} failed: {exc!r}")
        self._publish_status()

    def _publish_targets(self):
        targets = self._state.snapshot()["targets"]
        self._targets_pub.publish(
            targets_to_path(targets, self._frame_id, self.get_clock().now().to_msg()))

    def _notify(self, tag):
        # The coordinator's F8 (notifyTaskFinished) is unguarded, so it needs
        # no lease - which is why it can travel on bema_bridge's existing
        # session instead of a second client of our own (SP5's precedent).
        self._send_command({"action": "task_finished", "tag": int(tag)})

    def _send_command(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self._command_pub.publish(msg)

    def _action_tick(self):
        try:
            self._run_actions()
        except Exception as exc:
            self.get_logger().error(f"action tick failed: {exc!r}")

    def _status_tick(self):
        try:
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"status tick failed: {exc!r}")

    def _publish_status(self):
        status = self._state.snapshot()
        status["listening"] = self.listening
        msg = String()
        # default=str for the same reason /drive_status uses it: one odd
        # field must not black out the status the operator reads.
        msg.data = json.dumps(status, default=str)
        self._status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._server.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = NaviRpcServer()
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
