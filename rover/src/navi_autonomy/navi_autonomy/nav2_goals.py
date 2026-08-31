"""What goal_relay is allowed to do to Nav2, and the client that does it.

Deliberately the same shape as navi_supervisor/nav2_control.py: a tiny
interface, a recording double for the tests, and one real implementation.
goal_relay must never block on Nav2 - a rover whose status line stops
updating because an action client is waiting on a server that is not there
is a rover the operator cannot read.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# This build only plans in the map frame - the same constant nav_run.py and
# path_summary.py each keep their own copy of, for the same reason
# task_control.py keeps its own copy of the coordinator state table: this
# port must not import navi_autonomy.nav_run just to read one string.
PLAN_FRAME = "map"

# The operator's Go must not hang: half a second is generous for a Nav2
# that is actually up (discovery is near instant on the loopback-ish field
# link) and short enough that a missing Nav2 is reported well inside one
# 5 Hz tick of goal_relay's own timer.
SERVER_WAIT_S = 0.5


class Nav2Goals:
    """The interface. Do not change the two names."""

    def send_goal(self, x, y, yaw, on_succeeded, on_failed, on_feedback) -> None:
        raise NotImplementedError

    def cancel(self, reason: str) -> None:
        raise NotImplementedError


class RecordingNav2Goals(Nav2Goals):
    """`calls` is the list of (method, payload) pairs, in order - the
    contract the real client is checked against."""

    def __init__(self):
        self.calls = []

    def send_goal(self, x, y, yaw, on_succeeded, on_failed, on_feedback):
        self.calls.append(("send_goal", (x, y, yaw)))

    def cancel(self, reason):
        self.calls.append(("cancel", reason))


class ActionClientNav2Goals(Nav2Goals):
    """NavigateToPose, one waypoint at a time.

    Not FollowWaypoints: its feedback carries no distance_remaining or
    estimated_time_remaining, which is what spec section 7's status line
    asks for, and a waypoint-following goal cannot be resumed from the
    waypoint it was paused at.

    REQUIREMENT, and the one that is easy to get wrong: `cancel()` clears
    the stored goal handle AND the stored callbacks, so the result future
    for a cancelled goal reaches nobody. A late result from a superseded
    goal is dropped, not reported. NavigateToPose's result future fires
    for a cancelled goal too, with STATUS_CANCELED; routed to on_failed it
    would abort the run the operator merely paused, and Resume could never
    be pressed. NavRun guards on state as well, but the port must not send
    the report in the first place - the port is where the identity of the
    goal is known.
    """

    def __init__(self, node, action_name="navigate_to_pose"):
        self._node = node
        self._logger = node.get_logger()
        self._client = ActionClient(node, NavigateToPose, action_name)
        # One slot for the goal in flight: [goal_handle_or_None, on_succeeded,
        # on_failed, on_feedback]. Cleared as a unit by cancel() (and by a
        # goal ending on its own), which is what makes a late callback bound
        # to a superseded slot a no-op - `self._active is not slot` below,
        # checked in every one of the three callbacks.
        self._active = None

    def send_goal(self, x, y, yaw, on_succeeded, on_failed, on_feedback):
        if yaw is None:
            # goal_relay resolves "face the goal" from the previous waypoint
            # before this is called; a None that still reaches here (the
            # very first waypoint, no previous leg to face along) points
            # the rover along +x of the map frame rather than leaving the
            # orientation undefined.
            yaw = 0.0

        slot = [None, on_succeeded, on_failed, on_feedback]
        self._active = slot

        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            if self._active is slot:
                self._active = None
            on_failed("Nav2 is not running")
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)

        def feedback_cb(feedback_msg):
            if self._active is not slot:
                return
            fb = feedback_msg.feedback
            eta_s = (fb.estimated_time_remaining.sec
                     + fb.estimated_time_remaining.nanosec * 1e-9)
            slot[3](fb.distance_remaining, eta_s)

        send_future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)

        def on_goal_response(future):
            if self._active is not slot:
                return
            try:
                goal_handle = future.result()
            except Exception as exc:
                self._active = None
                slot[2](f"Nav2 goal send failed: {exc!r}")
                return
            if goal_handle is None or not goal_handle.accepted:
                self._active = None
                slot[2]("Nav2 rejected the goal")
                return
            slot[0] = goal_handle
            result_future = goal_handle.get_result_async()

            def on_result(result_future):
                if self._active is not slot:
                    # Cancelled (or superseded) before the result arrived:
                    # dropped, not reported - this is the guard the class
                    # docstring is about.
                    return
                self._active = None
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    slot[1]()
                else:
                    slot[2](f"Nav2 goal ended with status {status}")

            result_future.add_done_callback(on_result)

        send_future.add_done_callback(on_goal_response)

    def cancel(self, reason: str) -> None:
        slot, self._active = self._active, None
        if slot is None:
            return
        goal_handle = slot[0]
        if goal_handle is None:
            # Cancelled before Nav2 even accepted it: no handle to cancel
            # with, and clearing self._active already made the eventual
            # goal-response/result callbacks no-ops.
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            self._logger.error(f"cancel_goal_async failed ({reason}): {exc!r}")

    def _pose(self, x, y, yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = PLAN_FRAME
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose
