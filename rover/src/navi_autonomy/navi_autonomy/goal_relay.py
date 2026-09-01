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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from navi_autonomy import nav_run as rules
from navi_autonomy import path_summary
from navi_autonomy.glare import DetourPlanner
from navi_autonomy.nav_run import NavRun
from navi_autonomy.run_log import RunLog
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
        # Where the traversability layer reads the goal it must heal a
        # disc around. Latched, because the layer may start after the
        # goal was sent and a goal nobody heard is a goal not healed.
        self.declare_parameter("active_goal_topic", "/autonomy/active_goal")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("mode_status_topic", "/mode_status")
        self.declare_parameter("navi_rpc_status_topic", "/navi_rpc/status")
        # NavRun.tick() takes this as a constructor arg (SP11 task 8), so
        # the declared parameter now reaches the timeout actually applied,
        # rather than being informational only.
        self.declare_parameter("arm_timeout_s", rules.ARM_TIMEOUT_S)
        # The ride diary: one file, truncated at every accepted Go, holding
        # every decision of the LAST ride (operator request, night session).
        self.declare_parameter("run_log_path", "/tmp/navi_last_ride.log")

        # The glare-aware detour: glare_watch's verdict and the rover's own
        # pose are the two inputs DetourPlanner needs to decide whether the
        # next Nav2 goal is the real waypoint or a tack around the sun. This
        # stays entirely between goal_relay and Nav2 - nav_run is never told
        # about a detour, because a detour "arriving" would otherwise be
        # reported as a waypoint reached, which puts the coordinator in
        # Waiting and forces the operator to press Resume at every tack.
        self.declare_parameter("glare_topic", "/autonomy/glare")
        self.declare_parameter("pose_topic", "/localization/pose")
        self.declare_parameter("glare_detour_enabled", True)
        self.declare_parameter("glare_detour_offset_m", 2.0)
        self.declare_parameter("glare_detour_along_fraction", 0.5)
        self.declare_parameter("glare_detour_max_per_leg", 4)

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

        self._glare_detour_enabled = bool(self.get_parameter("glare_detour_enabled").value)
        self._glare_detour_max_per_leg = int(self.get_parameter("glare_detour_max_per_leg").value)
        self._planner = DetourPlanner(
            offset_m=float(self.get_parameter("glare_detour_offset_m").value),
            along_fraction=float(self.get_parameter("glare_detour_along_fraction").value),
            max_detours=self._glare_detour_max_per_leg)
        # None means "no glare reported yet" - the same value glare_side
        # itself returns for "no side to steer by", so no glare and never
        # having heard from glare_watch are indistinguishable, which is the
        # correct default: drive straight at the waypoint until told
        # otherwise.
        self._glare_side = None
        self._rover_xy = None            # (x, y) from /localization/pose, or None before the first one
        self._real_goal = None           # (index, x, y, yaw) of the waypoint currently in play

        self._runlog = RunLog(str(self.get_parameter("run_log_path").value))
        # What was last written to the diary, so only CHANGES land there:
        # (state, error), (mode, reason), coordinator state.
        self._logged_run = (None, None)
        self._logged_mode = (None, None)
        self._logged_coord = None

        self._nav_status_pub = self.create_publisher(
            String, str(self.get_parameter("nav_status_topic").value), 1)
        self._nav_path_summary_pub = self.create_publisher(
            String, str(self.get_parameter("nav_path_summary_topic").value), 1)
        self._active_goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("active_goal_topic").value),
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

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
        self.create_subscription(
            String, str(self.get_parameter("glare_topic").value),
            self._on_glare, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("pose_topic").value),
            self._on_pose, 10)

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
            if (isinstance(request, dict) and request.get("action") == "go"
                    and self._run.state not in _ACTIVE_STATES):
                # A fresh mission: the previous run's waypoints and plan
                # must not linger and be drawn under, or used to resolve
                # yaw for, this one. Gated on no-active-run with the same
                # check NavRun's own _on_go refuses by: a stray go during a
                # live run is refused there, and the refusal must not have
                # already replaced the live run's waypoint mirror (yaw
                # resolution and the drawn plan both read it).
                waypoints = request.get("waypoints") or []
                self._mission_waypoints = [
                    (float(w["x"]), float(w["y"]), w.get("yaw"))
                    for w in waypoints if isinstance(w, dict)]
                self._plan_points = []
            self._run.on_request(self._now(), request)
            if (isinstance(request, dict) and request.get("action") == "go"
                    and self._run.state == rules.STARTING):
                # An accepted Go is a new ride: the diary is truncated here,
                # so the file always holds the last ride from its first line.
                self._runlog.start(request.get("run_id"), self._mission_waypoints)
                self._runlog.event(
                    "go", f"accepted with {len(self._mission_waypoints)} waypoint(s)")
            elif isinstance(request, dict):
                error = self._run.status(self._now()).get("error") or ""
                self._runlog.event(f"request_{request.get('action')}", error)
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
            if (mode, reason) != self._logged_mode:
                self._logged_mode = (mode, reason)
                self._runlog.event("mode", f"{mode} ({reason})")
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
                self._runlog.event(
                    "coordinator_stop", "the primary stopped navigation "
                    f"(stop_seq {stop_seq}); cancelling the Nav2 goal")
                self._run.on_coordinator_stop(self._now())
                self._after_run_mutation()
            self._last_stop_seq = stop_seq
        except Exception as exc:
            self.get_logger().error(f"navi_rpc status callback failed: {exc!r}")

    def _on_glare(self, msg: String):
        # Parsed strictly, the way this file already parses every wire
        # payload (_on_nav_request, _on_mode_status): a malformed message or
        # an unknown side leaves the last known verdict untouched rather
        # than being read as "no glare" - a glitch on this topic must not
        # silently steer the rover back at the sun.
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable glare payload: {msg.data!r}")
            return
        try:
            side = payload.get("side") if isinstance(payload, dict) else None
            if side not in ("left", "right", "none"):
                self.get_logger().warn(f"unreadable glare payload: {msg.data!r}")
                return
            self._glare_side = None if side == "none" else side
        except Exception as exc:
            self.get_logger().error(f"glare callback failed: {exc!r}")

    def _on_pose(self, msg: Odometry) -> None:
        try:
            p = msg.pose.pose.position
            self._rover_xy = (float(p.x), float(p.y))
        except Exception as exc:
            self.get_logger().error(f"pose callback failed: {exc!r}")

    def _publish_active_goal(self, x: float, y: float) -> None:
        """Tell the traversability layer which point to heal a disc around.

        Published with the goal, not with the plan: the goal has to be
        drivable before a plan can exist, which is the whole reason the
        healing is there.
        """
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        self._active_goal_pub.publish(goal)

    # -- Nav2 goal callbacks, wired when a SEND_GOAL action is dispatched ----
    def _on_goal_succeeded(self):
        try:
            self._runlog.event("nav2_goal", "SUCCEEDED")
            self._run.on_goal_succeeded(self._now())
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"goal succeeded callback failed: {exc!r}")

    def _on_goal_failed(self, reason):
        try:
            self._runlog.event("nav2_goal", f"FAILED - {reason}")
            self._run.on_goal_failed(self._now(), reason)
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"goal failed callback failed: {exc!r}")

    def _on_feedback(self, distance_remaining, eta_s):
        try:
            self._runlog.event(
                "feedback", f"{distance_remaining:.2f} m to the waypoint, "
                            f"eta {eta_s:.0f} s", throttle_s=2.0)
            self._run.on_feedback(self._now(), distance_remaining, eta_s)
            self._after_run_mutation()
        except Exception as exc:
            self.get_logger().error(f"feedback callback failed: {exc!r}")

    # -- the glare detour, entirely between here and Nav2 - nav_run is never
    # told about a detour goal, so it can never put the coordinator into
    # Waiting for one (see the constructor's comment on why). ----------------
    def _advance_toward_real_goal(self):
        """Decide the next Nav2 goal for the leg in `_real_goal`: another
        detour, or - once the planner is out of glare or out of detours -
        the real waypoint itself. Called once when a leg starts and again
        every time a detour goal succeeds, which is what makes successive
        calls tack rather than bow: the rover has moved and the bearing to
        the goal has changed, so the planner is asked fresh each time."""
        index, x, y, _yaw = self._real_goal
        if not self._glare_detour_enabled:
            self._send_real_goal()
            return
        point, is_detour = self._planner.next_target(
            self._rover_xy, (x, y), self._glare_side)
        if not is_detour:
            self._send_real_goal()
            return
        px, py = point
        self._runlog.event(
            "detour_sent",
            f"waypoint {index + 1}/{len(self._mission_waypoints)}: glare on "
            f"the {self._glare_side} half of the frame - steering to "
            f"({px:.2f}, {py:.2f}) [detour {self._planner.detours_taken}/"
            f"{self._glare_detour_max_per_leg}]")
        self._publish_active_goal(px, py)
        self._nav2.send_goal(px, py, None, self._on_detour_succeeded,
                             self._on_detour_failed, self._on_detour_feedback)

    def _send_real_goal(self):
        index, x, y, yaw = self._real_goal
        yaw_txt = "free (+x)" if yaw is None else f"{yaw:.2f}"
        self._runlog.event(
            "goal_sent", f"waypoint {index + 1}/{len(self._mission_waypoints)}"
                         f" -> ({x:.2f}, {y:.2f}) yaw {yaw_txt}")
        self._publish_active_goal(x, y)
        self._nav2.send_goal(x, y, yaw, self._on_goal_succeeded,
                             self._on_goal_failed, self._on_feedback)

    def _on_detour_succeeded(self):
        try:
            if not self._leg_still_live():
                return
            self._runlog.event(
                "detour_result", "SUCCEEDED - re-checking glare before the next leg")
            self._advance_toward_real_goal()
        except Exception as exc:
            self.get_logger().error(f"detour succeeded callback failed: {exc!r}")

    def _on_detour_failed(self, reason):
        # A detour is an optimisation, not a requirement: losing one must
        # never cost the mission, so this falls straight through to the real
        # goal with nav_run's own callbacks rather than aborting the run.
        try:
            if not self._leg_still_live():
                return
            self._runlog.event(
                "detour_result", f"FAILED - {reason} - proceeding to the real waypoint")
            self._send_real_goal()
        except Exception as exc:
            self.get_logger().error(f"detour failed callback failed: {exc!r}")

    def _leg_still_live(self) -> bool:
        """Whether a detour callback may still act on the leg it belongs to.

        nav2_goals.cancel() already clears the stored callbacks, so a
        cancelled detour's result reaches nobody and this should never fire.
        It is here because the two detour callbacks do something no other
        callback in this file does - they DISPATCH A NEW GOAL - and a goal
        sent after the operator aborted would drive a rover that was told to
        stop. The cheap guard is worth more than the argument that the port
        makes it unreachable.
        """
        return self._real_goal is not None and self._run.state == rules.RUNNING

    def _on_detour_feedback(self, distance_remaining, eta_s):
        # Deliberately not forwarded to nav_run: this is progress toward a
        # detour point, not toward the real waypoint, and _run.on_feedback
        # would report a wrong distance/eta on the status line if it were.
        pass

    # -- timers ---------------------------------------------------------------
    def _tick(self):
        try:
            state = self._task.coordinator_state()
            if state != self._logged_coord:
                self._logged_coord = state
                self._runlog.event("coordinator", str(state))
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
            self._log_run_state()
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
            # Waypoints are part of the signature: two runs can share an
            # identical decimated corridor while their clicked waypoints
            # differ, and the mirror must not keep drawing the old markers.
            signature = (tuple(map(tuple, payload["points"])),
                         payload["source_points"], tuple(waypoints))
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
            self._runlog.event("coordinator_task",
                               f"startNaViTask, {len(payload)} waypoint(s); "
                               "Nav2 lifecycle asked to resume")
            resume = getattr(self._nav2, "resume", None)
            if resume is not None:
                resume()
            self._task.start_task(payload)
        elif action == rules.SEND_GOAL:
            index, x, y, yaw = payload
            yaw = self._resolve_yaw(index, x, y, yaw)
            # A new real waypoint: reset the detour counter for it (spec
            # section on DetourPlanner - "a leg" is one real waypoint) and
            # let the planner decide whether the first Nav2 goal is this
            # point or a tack away from the glare.
            self._real_goal = (index, x, y, yaw)
            self._planner.begin_leg()
            self._advance_toward_real_goal()
        elif action == rules.CANCEL_GOAL:
            self._runlog.event("goal_cancelled", str(payload))
            self._nav2.cancel(payload)
        elif action == rules.PAUSE_TASK:
            self._runlog.event("coordinator_pause")
            self._task.pause()
        elif action == rules.RESUME_TASK:
            self._runlog.event("coordinator_resume")
            self._task.resume()
        elif action == rules.ABORT_TASK:
            self._runlog.event("coordinator_abort")
            self._task.abort()
        elif action == rules.NOTIFY_WAYPOINT:
            self._runlog.event("waypoint_reached", f"index {payload}")
            self._task.notify_waypoint_reached(payload)
        elif action == rules.NOTIFY_DESTINATION:
            self._runlog.event("destination_reached")
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
        self._log_run_state()
        if self._run.take_changed():
            self._publish_status()

    def _log_run_state(self):
        # The run machine's own transitions, with the reason it carries -
        # "aborted - Nav2 goal ended with status 6" is the line that answers
        # "why did it break" without any other file.
        status = self._run.status(self._now())
        pair = (status["state"], status["error"])
        if pair == self._logged_run:
            return
        self._logged_run = pair
        state, error = pair
        self._runlog.event("run_state", state + (f" - {error}" if error else ""))

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
