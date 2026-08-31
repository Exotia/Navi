"""What the coordinator's calls mean, with time passed in rather than read -
the same fake-clock shape test_supervisor_state.py and test_bema_session.py
use. No sockets and no ROS here; Task 3 puts this behind the wire."""

import pytest

from navi_supervisor.navi_rpc_state import (CHASSIS_STOP, DESTINATION_REACHED,
                                            MAX_TARGETS, NAVIGATION_FAILED,
                                            NOTIFY_DESTINATION,
                                            NOTIFY_WAYPOINT, PUBLISH_TARGETS,
                                            TAG_DESTINATION_REACHED,
                                            TAG_WAYPOINT_REACHED,
                                            WAYPOINT_REACHED, NaviRpcState)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _state(mode=None):
    clock = Clock()
    state = NaviRpcState(clock=clock)
    if mode is not None:
        state.on_mode(mode)
    return state, clock


def test_the_tags_are_the_coordinators_own():
    assert TAG_WAYPOINT_REACHED == 0x31
    assert TAG_DESTINATION_REACHED == 0x32


def test_set_targets_stores_them_and_asks_for_a_path():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], (3.0, 4.0, 1.57)])
    assert state.snapshot()["targets"] == [[1.0, 2.0, 0.0], [3.0, 4.0, 1.57]]
    assert state.take_actions() == [PUBLISH_TARGETS]


def test_set_targets_rejects_an_empty_list():
    state, clock = _state()
    with pytest.raises(ValueError):
        state.set_targets([])


def test_set_targets_rejects_malformed_tuples():
    state, clock = _state()
    for bad in ([[1.0, 2.0]], [[1.0, 2.0, 3.0, 4.0]], [[1.0, 2.0, "x"]],
                [1.0, 2.0, 3.0], "targets", [[1.0, 2.0, float("nan")]],
                [[1.0, 2.0, True]]):
        with pytest.raises(ValueError):
            state.set_targets(bad)


def test_set_targets_rejects_an_absurd_list():
    state, clock = _state()
    with pytest.raises(ValueError):
        state.set_targets([[0.0, 0.0, 0.0]] * (MAX_TARGETS + 1))


def test_set_targets_clears_the_progress_of_the_previous_run():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.on_progress(DESTINATION_REACHED, index=0)
    state.take_actions()
    state.set_targets([[5.0, 6.0, 0.0]])
    snapshot = state.snapshot()
    assert snapshot["target_reached"] is False
    assert snapshot["last_point_reached"] is False
    assert snapshot["navigation_requested"] is False
    assert snapshot["reached_index"] is None


def test_start_navigation_without_targets_is_refused():
    state, clock = _state()
    with pytest.raises(ValueError):
        state.start_navigation()


def test_start_navigation_arms_the_run_and_bumps_the_sequence():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.take_actions()
    clock.t = 12.0
    state.start_navigation()
    snapshot = state.snapshot()
    assert snapshot["navigation_requested"] is True
    assert snapshot["start_seq"] == 1
    assert snapshot["started_at"] == 12.0
    assert state.take_actions() == []          # a run is never started here


def test_is_target_reached_is_a_real_bool_and_follows_progress():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]])
    state.start_navigation()
    assert state.is_target_reached() is False
    state.on_progress(WAYPOINT_REACHED, index=0)
    assert state.is_target_reached() is True
    state.start_navigation()                   # the next leg
    assert state.is_target_reached() is False


def test_a_waypoint_reached_asks_for_the_waypoint_tag():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(WAYPOINT_REACHED, index=0)
    assert state.take_actions() == [NOTIFY_WAYPOINT]
    assert state.snapshot()["reached_index"] == 0
    assert state.snapshot()["navigation_requested"] is True


def test_the_destination_ends_the_run_and_asks_for_the_destination_tag():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(DESTINATION_REACHED, index=0)
    assert state.take_actions() == [NOTIFY_DESTINATION]
    snapshot = state.snapshot()
    assert snapshot["last_point_reached"] is True
    assert snapshot["navigation_requested"] is False


def test_two_waypoints_in_one_batch_are_two_notifies():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0], [5.0, 6.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(WAYPOINT_REACHED, index=0)
    state.on_progress(WAYPOINT_REACHED, index=1)
    assert state.take_actions() == [NOTIFY_WAYPOINT, NOTIFY_WAYPOINT]


def test_a_failure_invents_no_completion():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(NAVIGATION_FAILED, reason="planner gave up")
    assert state.take_actions() == [CHASSIS_STOP]
    snapshot = state.snapshot()
    assert snapshot["navigation_requested"] is False
    assert snapshot["last_error"] == "planner gave up"
    assert snapshot["last_point_reached"] is False
    # The failure path is the SAME stop path as F6 and F7(false): one stop,
    # one bumped counter, whichever of the three fired.
    assert snapshot["stop_seq"] == 1
    assert snapshot["stop_requested"] is True


def test_an_unknown_progress_event_is_recorded_and_ignored():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress("teleported", index=0)
    assert state.take_actions() == []
    assert "teleported" in state.snapshot()["last_error"]
    assert state.snapshot()["navigation_requested"] is True


def test_stop_navigation_stops_the_chassis_and_bumps_the_stop_seq():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.stop_navigation()
    assert state.take_actions() == [CHASSIS_STOP]
    snapshot = state.snapshot()
    assert snapshot["navigation_requested"] is False
    assert snapshot["stop_seq"] == 1
    assert snapshot["stop_requested"] is True


def test_stop_navigation_is_the_same_from_every_mode():
    # No mode ever changes what F6 does, and it never asks for one. Asking
    # for `manual` would clear a latched e-stop (SupervisorState.
    # on_mode_request falls through to _estop_latched = False for a manual
    # request), and from `autonomous` it would turn the coordinator's PAUSE
    # into a full ABORT: CoordinatorImpl::pause() calls F6 while it is still
    # in Autonomous, so the supervisor would answer with COORDINATOR_ABORT
    # and the run would be unrecoverable.
    for mode in ("autonomous", "estop", "manual", None):
        state, clock = _state(mode=mode)
        state.set_targets([[1.0, 2.0, 0.0]])
        state.start_navigation()
        state.take_actions()
        state.stop_navigation()
        assert state.take_actions() == [CHASSIS_STOP], mode
        assert state.snapshot()["stop_seq"] == 1, mode


def test_a_new_run_clears_the_stop_flag_but_never_the_counter():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.stop_navigation()
    state.take_actions()
    assert state.snapshot()["stop_requested"] is True
    state.start_navigation()
    snapshot = state.snapshot()
    assert snapshot["stop_requested"] is False
    # Monotonic: goal_relay dedupes on the counter, so it must never go back.
    assert snapshot["stop_seq"] == 1


def test_a_second_stop_bumps_the_sequence_again():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.stop_navigation()
    assert state.take_actions() == [CHASSIS_STOP]
    state.stop_navigation()
    assert state.take_actions() == [CHASSIS_STOP]
    assert state.snapshot()["stop_seq"] == 2


def test_movement_disabled_stops_the_run_and_movement_enabled_does_not_gate_it():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.take_actions()
    state.start_navigation()                   # never told movement is enabled
    assert state.snapshot()["navigation_requested"] is True
    state.take_actions()
    state.set_movement_enabled(False)
    assert state.take_actions() == [CHASSIS_STOP]
    assert state.snapshot()["navigation_requested"] is False
    assert state.snapshot()["stop_seq"] == 1
    assert state.snapshot()["movement_enabled"] is False
    state.set_movement_enabled(True)
    assert state.take_actions() == []
    assert state.snapshot()["movement_enabled"] is True


def test_init_is_recorded_and_changes_nothing_else():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.take_actions()
    state.init()
    assert state.snapshot()["inited"] is True
    assert state.snapshot()["targets"] == [[1.0, 2.0, 0.0]]
    assert state.take_actions() == []


def test_the_snapshot_records_the_last_call_and_its_age():
    state, clock = _state()
    clock.t = 5.0
    state.init()
    clock.t = 7.5
    snapshot = state.snapshot()
    assert snapshot["last_method"] == "init"
    assert snapshot["last_call_age_s"] == 2.5


def test_the_snapshot_is_json_serialisable():
    import json
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.5]])
    state.start_navigation()
    state.on_progress(WAYPOINT_REACHED, index=0)
    json.dumps(state.snapshot())


# --- the method table, behind the real wire ------------------------------
from fake_coordinator import FakeCoordinator, RpcError            # noqa: E402
from navi_supervisor.navi_rpc_protocol import (ACCESS_DENIED_ERROR,  # noqa: E402
                                               RpcServer)
from navi_supervisor.navi_rpc_state import (GUARDED_METHODS,       # noqa: E402
                                            IDENTITY_POSE,
                                            navi_method_table)


@pytest.fixture
def served():
    clock = Clock()
    state = NaviRpcState(clock=clock)
    server = RpcServer(navi_method_table(state), guarded=GUARDED_METHODS,
                       host="127.0.0.1", port=0, clock=clock)
    server.start()
    client = FakeCoordinator("127.0.0.1", server.port)
    yield state, clock, client
    client.close()
    server.stop()


def test_the_guarded_set_matches_the_idl():
    assert GUARDED_METHODS == frozenset({"F0", "F1", "F3", "F4", "F5",
                                         "F6", "F7"})


def test_the_start_navi_task_sequence_the_coordinator_performs(served):
    state, clock, client = served
    # AutoConnection::getCapability() -> Client::accessServer()
    assert client.access() is True
    # the naviIniter thread's init()
    assert client.call("F0") is None
    # startNaViTask -> setTargets(waypoints)
    assert client.call("F3", [[1.0, 2.0, 0.0], [3.0, 4.0, 1.5]]) is None
    # 5 s later, onAutonomousEntered -> startNavigation_async()
    assert client.call("F4") is None
    assert client.call("F5") is False
    snapshot = state.snapshot()
    assert snapshot["inited"] is True
    assert snapshot["navigation_requested"] is True
    assert snapshot["targets"] == [[1.0, 2.0, 0.0], [3.0, 4.0, 1.5]]


def test_every_guarded_method_is_refused_with_error_one_without_the_lease(served):
    state, clock, client = served
    for method, args in (("F0", ()), ("F1", (1.0, 2.0)),
                         ("F3", ([[1.0, 2.0, 0.0]],)), ("F4", ()),
                         ("F5", ()), ("F6", ()), ("F7", (True,))):
        with pytest.raises(RpcError) as excinfo:
            client.call(method, *args)
        assert excinfo.value.error == ACCESS_DENIED_ERROR, method


def test_the_unguarded_stubs_answer_safely_and_need_no_lease(served):
    state, clock, client = served
    # WeakNaViEP::getPosition/getTofData/takeSnapshot are the three methods
    # whose mStub->call() is NOT wrapped in try/catch (WeakNaViEP.h:28-58),
    # so an error reply becomes an rpc::rpc_error in the caller's own thread
    # - std::terminate if it escapes a thread function. We hold the .18 alias
    # for the whole rover LAN, so anything that used to poll the real NaVi
    # for a pose, a ToF frame or a snapshot now reaches us. Shaped emptiness
    # is readable as "no data"; an exception is not.
    pose = client.call("F2")
    assert [list(row) for row in pose] == IDENTITY_POSE
    assert client.call("F8") == []
    assert client.call("F9", 0) is None


def test_set_position_refuses_even_with_the_lease(served):
    state, clock, client = served
    client.access()
    with pytest.raises(RpcError) as excinfo:
        client.call("F1", 1.0, 2.0)
    assert "not served" in str(excinfo.value.error)


def test_bad_targets_on_the_wire_are_an_error_not_a_dead_server(served):
    state, clock, client = served
    client.access()
    with pytest.raises(RpcError):
        client.call("F3", [[1.0, 2.0]])
    with pytest.raises(RpcError):
        client.call("F3", [])
    assert client.call("F3", [[1.0, 2.0, 0.0]]) is None


def test_is_target_reached_answers_a_bool_over_the_wire(served):
    state, clock, client = served
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    assert client.call("F5") is False
    state.on_progress(DESTINATION_REACHED, index=0)
    assert client.call("F5") is True


def test_set_movement_enabled_takes_a_bool_and_nothing_else(served):
    state, clock, client = served
    client.access()
    assert client.call("F7", False) is None
    with pytest.raises(RpcError):
        client.call("F7", 1)
