import pytest

from navi_autonomy import nav_run as rules
from navi_autonomy.nav_run import NavRun


class Clock:
    def __init__(self, t=0.0):
        self.t = t
    def __call__(self):
        return self.t


def go(waypoints=((3.0, -1.5), (8.0, -1.5)), run_id="gs-1"):
    return {"action": "go", "run_id": run_id, "frame_id": "map",
            "waypoints": [{"x": x, "y": y, "yaw": None} for x, y in waypoints]}


@pytest.fixture
def run():
    clock = Clock()
    r = NavRun(clock=clock)
    r.clock = clock
    return r


def armed(run):
    run.on_mode_status(run.clock(), "autonomous", "")
    run.on_coordinator_state("Autonomous")
    return run


def running(run, now=1.0):
    # The full Go path: request, START_TASK out, the coordinator OBSERVED
    # transitioning into Autonomous after the request (a cached value is not
    # evidence - see the arming tests), goal released. Tests whose premise
    # is "a goal is in flight" start here; the two arming tests below keep
    # their explicit step-by-step preambles on purpose.
    armed(run).on_request(0.0, go())
    run.take_actions()
    run.on_coordinator_state("Autonomous")
    run.tick(now)
    run.take_actions()
    return run


# -- the refusals, which are the safety-carrying half ---------------------

def test_go_is_refused_when_the_mode_is_not_autonomous(run):
    run.on_mode_status(0.0, "manual", "")
    run.on_request(0.0, go())
    assert run.state == rules.REFUSED
    assert "autonomous" in run.status(0.0)["error"]
    assert run.take_actions() == []          # nothing was asked of Nav2


def test_go_is_refused_when_no_mode_status_has_ever_arrived(run):
    # No supervisor heard from means no evidence the rover is in
    # autonomous. Refusing is the safe direction - the ground station's
    # own may_publish_manual_twist is permissive in the same situation
    # because publishing a twist nothing consumes is harmless, and
    # starting a mission is not.
    run.on_request(0.0, go())
    assert run.state == rules.REFUSED


def test_go_is_refused_with_no_waypoints(run):
    armed(run).on_request(0.0, {"action": "go", "run_id": "gs-1",
                                "frame_id": "map", "waypoints": []})
    assert run.state == rules.REFUSED and "waypoint" in run.status(0.0)["error"]


def test_go_is_refused_in_a_frame_this_build_does_not_plan_in(run):
    request = go()
    request["frame_id"] = "odom"
    armed(run).on_request(0.0, request)
    assert run.state == rules.REFUSED and "odom" in run.status(0.0)["error"]


def test_an_unknown_action_is_refused_rather_than_ignored(run):
    armed(run).on_request(0.0, {"action": "launch", "run_id": "gs-1"})
    assert run.state == rules.REFUSED and "launch" in run.status(0.0)["error"]


def test_a_pause_for_a_run_that_is_not_the_running_one_is_ignored(run):
    armed(run).on_request(0.0, go())
    run.take_actions()
    run.on_request(1.0, {"action": "pause", "run_id": "gs-OTHER"})
    assert run.take_actions() == []
    assert run.state == rules.STARTING


def test_a_second_go_while_a_run_is_active_is_refused_not_obeyed(run):
    # The row disables Go while a run is active, but the row is not the
    # authority (see "Two gates on Go, deliberately"): a `ros2 topic pub` or
    # a second ground station must be refused here. A go that replaced
    # _waypoints and _index mid-drive would send the rover somewhere nobody
    # asked for while the operator watched the old plan.
    running(run)
    run.on_request(2.0, go(waypoints=((99.0, 0.0),), run_id="gs-2"))
    assert run.state == rules.RUNNING
    assert run.take_actions() == []
    assert run.status(2.0)["run_id"] == "gs-1"


# -- the happy path -------------------------------------------------------

def test_go_asks_the_coordinator_first_and_only_then_nav2(run):
    armed(run).on_request(0.0, go())
    assert run.state == rules.STARTING
    assert run.take_actions() == [(rules.START_TASK, ((3.0, -1.5), (8.0, -1.5)))]
    # Nav2 is not sent a goal until the coordinator says Autonomous: a goal
    # sent while the mission state is still PrepareAutonomous drives a
    # rover the coordinator has not enabled yet.
    run.on_coordinator_state("PrepareAutonomous")
    run.tick(1.0)
    assert run.take_actions() == []
    run.on_coordinator_state("Autonomous")
    run.tick(2.0)
    assert run.take_actions() == [(rules.SEND_GOAL, (0, 3.0, -1.5, None))]
    assert run.state == rules.RUNNING


def test_a_stale_autonomous_does_not_let_the_goal_out_before_the_task_is_armed(run):
    # coordinator_state() is a poll in the node, and startNaViTask drives the
    # coordinator -> PrepareAutonomous -> Autonomous over ~5 s (spec section
    # 3). A value cached from before the START_TASK - the previous run's, or
    # a manual /drive_command's - is not evidence that THIS task is armed.
    # Only a transition observed AFTER the request counts.
    armed(run).on_request(0.0, go())        # coordinator already "Autonomous"
    run.take_actions()
    run.tick(1.0)
    assert run.take_actions() == []         # the cached state is not evidence
    run.on_coordinator_state("Autonomous")
    run.tick(2.0)
    assert run.take_actions() == [(rules.SEND_GOAL, (0, 3.0, -1.5, None))]


def test_arming_that_never_completes_aborts_with_a_reason(run):
    armed(run).on_request(0.0, go())
    run.take_actions()
    run.on_coordinator_state("PrepareAutonomous")
    run.tick(rules.ARM_TIMEOUT_S + 0.1)
    assert run.state == rules.ABORTED
    assert "coordinator" in run.status(0.0)["error"]
    assert (rules.ABORT_TASK, None) in run.take_actions()


def test_a_reached_waypoint_is_notified_and_the_run_holds_for_resume(run):
    # notifyTaskFinished(TAG_WaypointReached) moves the coordinator to
    # Waiting with movement force-disabled until the operator resumes
    # (CoordinatorImpl.cpp:218-236), so the next goal must NOT go out with
    # the notification - a goal driven into a braked chassis is the 45 s
    # progress abort. The run holds in PAUSED; Resume re-arms and only an
    # OBSERVED Autonomous releases the next waypoint's goal.
    running(run)
    run.on_goal_succeeded(2.0)
    assert run.take_actions() == [(rules.NOTIFY_WAYPOINT, 0)]
    assert run.state == rules.PAUSED
    assert "waypoint 1/2 reached" in run.status(2.0)["error"]
    run.on_request(3.0, {"action": "resume", "run_id": "gs-1"})
    assert run.take_actions() == [(rules.RESUME_TASK, None)]
    run.on_coordinator_state("Autonomous")
    run.tick(4.0)
    assert run.take_actions() == [(rules.SEND_GOAL, (1, 8.0, -1.5, None))]
    run.on_goal_succeeded(5.0)
    assert run.take_actions() == [(rules.NOTIFY_WAYPOINT, 1),
                                  (rules.NOTIFY_DESTINATION, None)]
    assert run.state == rules.SUCCEEDED


def test_a_failed_nav2_goal_aborts_the_run_and_the_task(run):
    running(run)
    run.on_goal_failed(2.0, "no valid path")
    assert run.state == rules.ABORTED
    assert run.status(2.0)["error"] == "no valid path"
    assert run.take_actions() == [(rules.ABORT_TASK, None)]


# -- pause, resume, abort -------------------------------------------------

def test_pause_cancels_the_goal_and_keeps_the_waypoint_index(run):
    running(run)
    run.on_goal_succeeded(2.0)
    run.take_actions()
    run.on_request(3.0, {"action": "resume", "run_id": "gs-1"})
    run.on_coordinator_state("Autonomous")
    run.tick(4.0)
    run.take_actions()                       # SEND_GOAL(1, ...) - in flight
    run.on_request(5.0, {"action": "pause", "run_id": "gs-1"})
    assert run.state == rules.PAUSED
    assert run.take_actions() == [(rules.CANCEL_GOAL, "operator paused"),
                                  (rules.PAUSE_TASK, None)]
    assert run.status(5.0)["waypoint_index"] == 1


def test_resume_re_checks_the_coordinator_before_it_re_sends_the_waypoint(run):
    # A pause has no time bound. notifyConnected (F10) must arrive every 2 s
    # or the coordinator drops to Disconnected, and F4 pause may leave it in
    # Waiting (spec section 3). So Resume goes back to STARTING and waits for
    # the same OBSERVED not-Autonomous -> Autonomous transition Go waits for,
    # rather than trusting the state it had before the pause.
    running(run)
    run.on_request(2.0, {"action": "pause", "run_id": "gs-1"})
    run.take_actions()
    run.on_request(3.0, {"action": "resume", "run_id": "gs-1"})
    assert run.state == rules.STARTING
    assert run.take_actions() == [(rules.RESUME_TASK, None)]
    run.on_coordinator_state("Autonomous")
    run.tick(4.0)
    assert run.take_actions() == [(rules.SEND_GOAL, (0, 3.0, -1.5, None))]
    assert run.state == rules.RUNNING


def test_resume_while_the_coordinator_has_dropped_out_aborts_with_a_reason(run):
    running(run)
    run.on_request(2.0, {"action": "pause", "run_id": "gs-1"})
    run.take_actions()
    run.on_coordinator_state("Disconnected")
    run.on_request(3.0, {"action": "resume", "run_id": "gs-1"})
    run.take_actions()
    run.tick(3.0 + rules.ARM_TIMEOUT_S + 0.1)
    assert run.state == rules.ABORTED
    assert "coordinator" in run.status(3.0)["error"]
    assert (rules.ABORT_TASK, None) in run.take_actions()


def test_a_cancel_result_after_a_pause_does_not_abort_the_run(run):
    # NavigateToPose's result future fires for a cancelled goal too, with
    # STATUS_CANCELED. If that reached on_goal_failed, the run the operator
    # merely paused would go straight to ABORTED and Resume could never be
    # pressed. The port drops the callbacks on cancel(); this guard is the
    # second half, for a result already in flight when cancel() ran.
    running(run)
    run.on_request(2.0, {"action": "pause", "run_id": "gs-1"})
    run.take_actions()
    run.on_goal_failed(2.1, "Goal was canceled")
    assert run.state == rules.PAUSED
    assert run.take_actions() == []
    # and the pause is still resumable afterwards
    run.on_request(3.0, {"action": "resume", "run_id": "gs-1"})
    assert run.state == rules.STARTING
    assert run.take_actions() == [(rules.RESUME_TASK, None)]
    run.on_coordinator_state("Autonomous")
    run.tick(4.0)
    assert run.take_actions() == [(rules.SEND_GOAL, (0, 3.0, -1.5, None))]


def test_a_late_result_after_an_abort_does_not_queue_a_second_abort(run):
    running(run)
    run.on_request(2.0, {"action": "abort", "run_id": "gs-1"})
    run.take_actions()
    run.on_goal_failed(2.1, "Goal was canceled")
    assert run.take_actions() == []


def test_abort_cancels_the_goal_then_aborts_the_task_in_that_order(run):
    running(run)
    run.on_request(2.0, {"action": "abort", "run_id": "gs-1"})
    assert run.state == rules.ABORTED
    assert run.take_actions() == [(rules.CANCEL_GOAL, "operator aborted"),
                                  (rules.ABORT_TASK, None)]


def test_abort_does_not_ask_for_a_mode_change(run):
    # The supervisor is the single authority on mode (spec section 4). The
    # way back to the sticks is the DRIVE row's Manual button.
    running(run)
    run.on_request(2.0, {"action": "abort", "run_id": "gs-1"})
    assert all(action != rules.REQUEST_MODE for action, _ in run.take_actions())


def test_a_bumped_stop_seq_cancels_the_goal_and_pauses_the_run(run):
    # SP11 task 8: navi_rpc_server bumps stop_seq for F6, F7(false) and a
    # failed run alike, and asks for no mode change in any of them -
    # cancelling the Nav2 goal here is what actually stops /autonomy_twist.
    running(run)
    run.on_coordinator_stop(3.0)
    assert run.state == rules.PAUSED
    assert run.take_actions() == [(rules.CANCEL_GOAL, "coordinator stop")]
    # No PAUSE_TASK: the coordinator caused this stop, so telling it to
    # pause again would only be a round trip to itself.
    assert run.status(3.0)["error"] == "coordinator stopped navigation"


def test_a_coordinator_stop_with_no_active_run_does_nothing(run):
    run.on_coordinator_stop(0.0)
    assert run.state == rules.IDLE
    assert run.take_actions() == []


def test_a_coordinator_stop_while_already_paused_does_not_double_cancel(run):
    running(run)
    run.on_request(2.0, {"action": "pause", "run_id": "gs-1"})
    run.take_actions()
    run.on_coordinator_stop(3.0)
    assert run.state == rules.PAUSED
    assert run.take_actions() == []


def test_arm_timeout_s_is_configurable():
    # Parked from SP11 task 5: the constructor arg, not the module
    # constant, is what NavRun.tick() actually enforces.
    clock = Clock()
    custom = NavRun(clock=clock, arm_timeout_s=3.0)
    armed(custom).on_request(0.0, go())
    custom.take_actions()
    custom.on_coordinator_state("PrepareAutonomous")
    custom.tick(3.1)
    assert custom.state == rules.ABORTED
    assert "coordinator" in custom.status(3.1)["error"]


def test_arm_timeout_s_defaults_to_the_module_constant():
    clock = Clock()
    default = NavRun(clock=clock)
    armed(default).on_request(0.0, go())
    default.take_actions()
    default.on_coordinator_state("PrepareAutonomous")
    default.tick(rules.ARM_TIMEOUT_S - 0.1)
    assert default.state == rules.STARTING       # not timed out yet
    default.tick(rules.ARM_TIMEOUT_S + 0.1)
    assert default.state == rules.ABORTED


def test_losing_autonomous_mode_aborts_the_run_with_the_supervisors_reason(run):
    running(run)
    run.on_mode_status(2.0, "manual", "operator takeover")
    assert run.state == rules.ABORTED
    assert run.status(2.0)["error"] == "operator takeover"
    # The supervisor has already cancelled the goal and deactivated Nav2 on
    # its own (spec section 4 rule 1); asking again is harmless and makes
    # the goal_relay correct when the mode changed for some other reason.
    assert run.take_actions() == [(rules.CANCEL_GOAL, "operator takeover"),
                                  (rules.ABORT_TASK, None)]


# -- what the status line says --------------------------------------------

def test_distance_and_eta_add_the_legs_that_are_still_ahead(run):
    running(run)
    run.on_feedback(2.0, distance_remaining=2.0, eta_s=40.0)
    status = run.status(2.0)
    # 2.0 m to waypoint 0, plus the 5.0 m straight leg from waypoint 0 to 1.
    assert abs(status["distance_remaining_m"] - 7.0) < 1e-9
    assert abs(status["eta_s"] - (40.0 + 5.0 / rules.NOMINAL_SPEED_MPS)) < 1e-9


def test_status_before_any_feedback_reports_unknown_rather_than_zero(run):
    armed(run).on_request(0.0, go())
    assert run.status(0.0)["distance_remaining_m"] is None
    assert run.status(0.0)["eta_s"] is None


def test_status_carries_every_documented_field(run):
    status = armed(run).status(0.0)
    assert set(status) == {"state", "run_id", "waypoint_index", "waypoint_count",
                           "distance_remaining_m", "eta_s", "error", "mode",
                           "coordinator_state", "stamp_s"}
