"""goal_relay: the ground station's Go/Pause/Resume/Abort row, driven onto
Nav2 and the primary's coordinator.

nav_run.py owns the rules (spec section 7) and has no ROS in it; this file
owns the topics, the two ports (nav2_goals.py, task_control.py) and the
timers that drive them. Same split as navi_supervisor/mode_supervisor.py
and supervisor_state.py.

Publish contract, the same one mode_supervisor's /mode_status keeps: 2 Hz
baseline (`_status_tick`) and immediately on a change a subscription or a
Nav2 callback made (`take_changed()`), so a refusal or a pause does not
read as a dead button for up to 500 ms. The 5 Hz `tick()` timer is
different: it drives NavRun.tick() (arming, the arm timeout) and polls the
coordinator state every cycle - polling always marks NavRun "changed"
internally (on_coordinator_state does not know whether the value it was
handed is new), so checking take_changed() there would turn the 5 Hz timer
into the status rate. Its side effects still reach Nav2/the coordinator
immediately through _run_actions(); the status catches up at the next 2 Hz
tick, at most one tick later than mode_supervisor's own baseline.

`_on_navi_rpc_status` is the SP8 C1 contract SP8's own plan left for this
node to complete: navi_rpc_server bumps `stop_seq` on `/navi_rpc/status`
for F6 stopNavigation, F7(false) and a failed run alike, and asks for no
mode change in any of the three - this is the one place that watches the
counter and cancels the Nav2 goal, which is what actually stops
`/autonomy_twist`.
"""

import json
import math
from time import monotonic

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from navi_autonomy import nav_run as rules
from navi_autonomy import path_summary
from navi_autonomy.nav_run import NavRun
from navi_autonomy.nav2_goals import ActionClientNav2Goals
from navi_autonomy.task_control import NaviRpcTaskControl

TICK_HZ = 5.0
STATUS_HZ = 2.0
PLAN_HZ = 2.0

# nav_run.py keeps its own copy private (_ACTIVE_STATES): rebuilt here from
# the three public state names rather than reaching into that underscore.
_ACTIVE_STATES = (rules.STARTING, rules.RUNNING, rules.PAUSED)


class GoalRelay(Node):

    def __init__(self, clock=monotonic, nav2_goals=None, task_control=None,
                 parameter_overrides=None):
        super().__init__("goal_relay",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("nav_request_topic", "/nav_request")
        self.declare_parameter("nav_status_topic", "/nav_status")
        self.declare_parameter("nav_path_summary_topic", "/nav_path_summary")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("mode_status_topic", "/mode_status")
        self.declare_parameter("navi_rpc_status_topic", "/navi_rpc/status")
        # NavRun.tick() takes this as a constructor arg (SP11 task 8), so
        # the declared parameter now reaches the timeout actually applied,
        # rather than being informational only.
        self.declare_parameter("arm_timeout_s", rules.ARM_TIMEOUT_S)

        # NOT self._clock: rclpy.node.Node already owns that name (see
        # mode_supervisor.py's identical note).
        self._now = clock
        self._run = NavRun(clock, arm_timeout_s=float(
            self.get_parameter("arm_timeout_s").value))

        self._nav2 = nav2_goals if nav2_goals is not None else ActionClientNav2Goals(self)
        self._task = task_control if task_control is not None else NaviRpcTaskControl(self)

        self._mission_waypoints = []     # [(x, y, yaw_or_None), ...] of the current/last go
        self._plan_points = []           # raw /plan, (x, y) pairs
        self._last_plan_signature = None
        # First observation only sets the baseline (see _on_navi_rpc_status):
        # a value merely cached from before this node subscribed is not
        # evidence the coordinator just stopped something.
        self._last_stop_seq = None

        self._nav_status_pub = self.create_publisher(
            String, str(self.get_parameter("nav_status_topic").value), 1)
        self._nav_path_summary_pub = self.create_publisher(
            String, str(self.get_parameter("nav_path_summary_topic").value), 1)

        self.create_subscription(
            String, str(self.get_parameter("nav_request_topic").value),
            self._on_nav_request, 10)
        self.create_subscription(
            String, str(self.get_parameter("mode_status_topic").value),
            self._on_mode_status, 10)
        self.create_subscription(
            Path, str(self.get_parameter("plan_topic").value), self._on_plan, 1)
        self.create_subscription(
            String, str(self.get_parameter("navi_rpc_status_topic").value),
            self._on_navi_rpc_status, 10)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)
        self.create_timer(1.0 / PLAN_HZ, self._plan_tick)

    # -- inputs --------------------------------------------------------------
    def _on_nav_request(self, msg: String):
        try:
            request = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable nav request: {msg.data!r}")
            return
        try:
            if isinstance(request, dict) and request.get("action") == "go":
                # A fresh mission: the previous run's waypoints and plan
                # must not linger and be drawn under, or used to resolve
                # yaw for, this one. If Go is refused nothing overwrites
                # this again, and the empty plan is exactly what a refused
                # run should show.
                waypoints = request.get("waypoints") or []
                self._mission_waypoints = [
                    (float(w["x"]), float(w["y"]), w.get("yaw"))
                    for w in waypoints if isinstance(w, dict)]
                self._plan_points = []
            self._run.on_request(self._now(), request)
            self._after_run_mutation()
        except Exception as exc:                      # never kill the node
            self.get_logger().error(f"nav request failed: {exc!r}")

    def _on_mode_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable mode status: {msg.data!r}")
            return
        try:
            mode = payload.get("mode") if isinstance(payload, dict) else None
            reason = payload.get("reason", "") if isinstance(payload, dict) else ""
            self._run.on_mode_status(self._now(), mode, reason)
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"mode status callback failed: {exc!r}")

    def _on_plan(self, msg: Path):
        try:
            self._plan_points = [(p.pose.position.x, p.pose.position.y)
                                 for p in msg.poses]
        except Exception as exc:
            self.get_logger().error(f"plan callback failed: {exc!r}")

    def _on_navi_rpc_status(self, msg: String):
        # The SP8 C1 contract: navi_rpc_server bumps stop_seq on F6
        # stopNavigation, F7(false) and a failed run alike, and none of
        # those asks the supervisor for a mode change - completing the
        # stop (cancelling the Nav2 goal, which is what actually stops
        # /autonomy_twist) is this node's job, not the supervisor's.
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable navi_rpc status: {msg.data!r}")
            return
        try:
            stop_seq = payload.get("stop_seq") if isinstance(payload, dict) else None
            if isinstance(stop_seq, bool) or not isinstance(stop_seq, int):
                return
            if self._last_stop_seq is not None and stop_seq != self._last_stop_seq:
                self._run.on_coordinator_stop(self._now())
                self._after_run_mutation()
            self._last_stop_seq = stop_seq
        except Exception as exc:
            self.get_logger().error(f"navi_rpc status callback failed: {exc!r}")

    # -- Nav2 goal callbacks, wired when a SEND_GOAL action is dispatched ----
    def _on_goal_succeeded(self):
        try:
            self._run.on_goal_succeeded(self._now())
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"goal succeeded callback failed: {exc!r}")

    def _on_goal_failed(self, reason):
        try:
            self._run.on_goal_failed(self._now(), reason)
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"goal failed callback failed: {exc!r}")

    def _on_feedback(self, distance_remaining, eta_s):
        try:
            self._run.on_feedback(self._now(), distance_remaining, eta_s)
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"feedback callback failed: {exc!r}")

    # -- timers ---------------------------------------------------------------
    def _tick(self):
        try:
            state = self._task.coordinator_state()
            # Always fed to NavRun, even when unchanged from the last poll:
            # arming (NavRun._arm()) resets its cached coordinator_state to
            # None specifically so a value merely cached from before the
            # request cannot arm it - only a value OBSERVED after arming
            # can, and it can equal what was polled a moment ago (the
            # coordinator really was already Autonomous). Skipping the call
            # here because "nothing changed" would leave that None in place
            # forever and the run would never be released.
            self._run.on_coordinator_state(state)
            self._run.tick(self._now())
            self._run_actions()
        except Exception as exc:
            self.get_logger().error(f"tick failed: {exc!r}")

    def _status_tick(self):
        try:
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"status tick failed: {exc!r}")

    def _plan_tick(self):
        try:
            status = self._run.status(self._now())
            if status["state"] in _ACTIVE_STATES:
                points = self._plan_points
                waypoints = [(x, y) for x, y, _ in self._mission_waypoints]
            else:
                # No active run: clear the drawing rather than leave a
                # finished mission's path shown on the Gazebo mirror.
                points = []
                waypoints = []
            payload = path_summary.summary_payload(
                status["run_id"], points, waypoints, self._now())
            signature = (tuple(map(tuple, payload["points"])), payload["source_points"])
            if signature == self._last_plan_signature:
                return
            self._last_plan_signature = signature
            msg = String()
            msg.data = json.dumps(payload, default=str)
            self._nav_path_summary_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"plan tick failed: {exc!r}")

    # -- side effects -----------------------------------------------------
    def _run_actions(self):
        for action, payload in self._run.take_actions():
            try:
                self._dispatch(action, payload)
            except Exception as exc:
                self.get_logger().error(f"action {action} failed: {exc!r}")

    def _dispatch(self, action, payload):
        if action == rules.START_TASK:
            # The supervisor paused Nav2 when the last run ended (rule 1);
            # wake it back up now, so it is active again by the time the
            # observed-Autonomous transition releases the goal (>= 5 s).
            resume = getattr(self._nav2, "resume", None)
            if resume is not None:
                resume()
            self._task.start_task(payload)
        elif action == rules.SEND_GOAL:
            index, x, y, yaw = payload
            yaw = self._resolve_yaw(index, x, y, yaw)
            self._nav2.send_goal(x, y, yaw, self._on_goal_succeeded,
                                 self._on_goal_failed, self._on_feedback)
        elif action == rules.CANCEL_GOAL:
            self._nav2.cancel(payload)
        elif action == rules.PAUSE_TASK:
            self._task.pause()
        elif action == rules.RESUME_TASK:
            self._task.resume()
        elif action == rules.ABORT_TASK:
            self._task.abort()
        elif action == rules.NOTIFY_WAYPOINT:
            self._task.notify_waypoint_reached(payload)
        elif action == rules.NOTIFY_DESTINATION:
            self._task.notify_destination_reached()
        else:
            self.get_logger().warn(f"unknown goal_relay action: {action!r}")

    def _resolve_yaw(self, index, x, y, yaw):
        if yaw is not None:
            return yaw
        if index == 0 or index > len(self._mission_waypoints):
            return None       # first waypoint: the port faces +x of map
        px, py, _ = self._mission_waypoints[index - 1]
        return math.atan2(y - py, x - px)

    def _after_run_mutation(self):
        self._run_actions()
        if self._run.take_changed():
            self._publish_status()

    def _publish_status(self):
        msg = String()
        # default=str for the same reason /drive_status and /mode_status
        # use it: one odd field must not black out the whole status.
        msg.data = json.dumps(self._run.status(self._now()), default=str)
        self._nav_status_pub.publish(msg)


def main():
    rclpy.init()
    node = GoalRelay()
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
