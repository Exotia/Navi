"""The supervisor's rules, with time passed in rather than read - the same
fake-clock shape test_bema_session.py uses. No ROS here."""

from navi_supervisor.supervisor_state import (AUTONOMOUS, AUTONOMY_DEADMAN_S,
                                              CANCEL_GOAL, CHASSIS_STOP,
                                              COORDINATOR_ABORT,
                                              COORDINATOR_MANUAL,
                                              DEACTIVATE_NAV2, ESTOP, MANUAL,
                                              MANUAL_DEADMAN_S, SEMI_AUTO,
                                              SupervisorState)


def test_manual_mode_forwards_the_manual_twist():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.04, 0.0, 0.05)
    assert s.output(1.0) == (0.04, 0.0, 0.05)


def test_semi_auto_forwards_the_manual_twist_too():
    s = SupervisorState(mode=SEMI_AUTO)
    s.on_manual_twist(1.0, 0.03, 0.01, 0.0)
    assert s.output(1.0) == (0.03, 0.01, 0.0)


def test_manual_mode_ignores_the_autonomy_twist():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.01, 0.0, 0.0)
    s.on_autonomy_twist(1.0, 0.4, 0.0, 0.0)
    assert s.output(1.0) == (0.01, 0.0, 0.0)


def test_autonomous_mode_forwards_the_autonomy_twist():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.2)
    assert s.output(1.0) == (0.3, 0.0, 0.2)


def test_no_source_yet_is_the_deadman_not_a_free_pass():
    s = SupervisorState(mode=MANUAL)
    assert s.deadman_active(0.0) is True
    assert s.output(0.0) == (0.0, 0.0, 0.0)


def test_manual_deadman_is_one_second():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    assert s.output(1.0 + MANUAL_DEADMAN_S) == (0.05, 0.0, 0.0)
    assert s.deadman_active(1.0 + MANUAL_DEADMAN_S) is False
    assert s.output(1.0 + MANUAL_DEADMAN_S + 0.01) == (0.0, 0.0, 0.0)
    assert s.deadman_active(1.0 + MANUAL_DEADMAN_S + 0.01) is True


def test_autonomy_deadman_is_half_a_second():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    assert s.output(1.0 + AUTONOMY_DEADMAN_S) == (0.3, 0.0, 0.0)
    assert s.output(1.0 + AUTONOMY_DEADMAN_S + 0.01) == (0.0, 0.0, 0.0)


def test_the_manual_stream_does_not_feed_the_autonomy_deadman():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.4, 0.0, 0.0, 0.0)      # a zero stream, below takeover
    assert s.output(1.6) == (0.0, 0.0, 0.0)


def test_the_deadman_edge_queues_one_chassis_stop_not_one_per_tick():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.output(1.0)
    assert s.take_actions() == []
    s.output(3.0)
    assert s.take_actions() == [CHASSIS_STOP]
    s.output(3.05)
    s.output(3.10)
    assert s.take_actions() == []


def test_a_fresh_twist_after_the_deadman_drives_again():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.output(3.0)
    s.take_actions()
    s.on_manual_twist(3.5, 0.02, 0.0, 0.0)
    assert s.output(3.5) == (0.02, 0.0, 0.0)


def test_estop_mode_is_always_zero_and_always_deadman():
    s = SupervisorState(mode=ESTOP)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    assert s.output(1.0) == (0.0, 0.0, 0.0)
    assert s.deadman_active(1.0) is True


def test_status_names_the_mode_the_source_and_the_age():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    status = s.status(1.25)
    assert status["mode"] == AUTONOMOUS
    assert status["source"] == "/autonomy_twist"
    assert status["deadman_active"] is False
    assert status["estop_latched"] is False
    assert status["source_age_s"] == 0.25


def test_estop_zeroes_the_output_and_latches():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_estop_request(1.1, "ground station STOP")
    assert s.mode == ESTOP
    assert s.estop_latched is True
    assert s.output(1.1) == (0.0, 0.0, 0.0)
    # and it stays stopped however much the operator keeps steering
    s.on_manual_twist(1.2, 0.05, 0.0, 0.0)
    s.on_manual_twist(5.0, 0.05, 0.0, 0.0)
    assert s.output(5.0) == (0.0, 0.0, 0.0)
    assert s.mode == ESTOP


def test_estop_stops_the_chassis_and_cancels_any_goal():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_estop_request(1.1, "ground station STOP")
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP]


def test_estop_reason_reaches_the_status():
    s = SupervisorState(mode=MANUAL)
    s.on_estop_request(1.0, "ground station STOP")
    status = s.status(1.0)
    assert status["mode"] == ESTOP
    assert status["reason"] == "ground station STOP"
    assert status["estop_latched"] is True


def test_only_a_manual_mode_request_clears_the_latch():
    s = SupervisorState(mode=MANUAL)
    s.on_estop_request(1.0, "STOP")
    assert s.on_mode_request(2.0, AUTONOMOUS) is not None
    assert s.mode == ESTOP
    assert s.on_mode_request(2.1, SEMI_AUTO) is not None
    assert s.mode == ESTOP
    assert s.on_mode_request(2.2, MANUAL) is None
    assert s.mode == MANUAL
    assert s.estop_latched is False


def test_clearing_the_latch_does_not_replay_the_twist_that_was_held():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_estop_request(1.1, "STOP")
    s.on_manual_twist(1.2, 0.05, 0.0, 0.0)     # operator still on the stick
    s.on_mode_request(1.3, MANUAL)
    assert s.output(1.3) == (0.0, 0.0, 0.0)    # deadman, not the held twist
    s.on_manual_twist(1.4, 0.05, 0.0, 0.0)
    assert s.output(1.4) == (0.05, 0.0, 0.0)   # a genuinely new one drives


def test_an_unknown_mode_is_refused_and_changes_nothing():
    s = SupervisorState(mode=MANUAL)
    assert s.on_mode_request(1.0, "turbo") is not None
    assert s.mode == MANUAL


def test_a_mode_request_of_estop_is_an_estop():
    s = SupervisorState(mode=AUTONOMOUS)
    assert s.on_mode_request(1.0, ESTOP) is None
    assert s.mode == ESTOP
    assert s.estop_latched is True


def test_leaving_autonomous_by_request_cancels_the_goal():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_mode_request(1.0, MANUAL)
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2,
                                COORDINATOR_ABORT, COORDINATOR_MANUAL]
    assert s.mode == MANUAL


def test_entering_autonomous_from_manual_cancels_nothing():
    s = SupervisorState(mode=MANUAL)
    assert s.on_mode_request(1.0, AUTONOMOUS) is None
    assert s.take_actions() == []
