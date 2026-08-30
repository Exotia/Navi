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
    s.on_manual_twist(1.4, 0.001, 0.0, 0.0)    # non-zero, but below takeover
    assert s.output(1.4) == (0.3, 0.0, 0.0)    # the autonomy twist, not manual


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


def test_a_stick_above_the_deadzone_takes_over_from_autonomy():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.05, 0.005, 0.0, 0.0)   # the GS's smallest real output
    assert s.mode == MANUAL
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2,
                                COORDINATOR_ABORT, COORDINATOR_MANUAL]
    assert s.status(1.05)["reason"] == "operator takeover"


def test_the_twist_that_took_over_drives_on_the_same_tick():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.05, 0.05, 0.0, 0.0)
    assert s.output(1.05) == (0.05, 0.0, 0.0)


def test_a_rotation_only_stick_takes_over_too():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.05, 0.0, 0.0, 0.01)    # the GS's smallest real wz
    assert s.mode == MANUAL


def test_a_zero_manual_stream_does_not_take_over():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    for t in (1.05, 1.10, 1.15, 1.20):
        s.on_manual_twist(t, 0.0, 0.0, 0.0)
    assert s.mode == AUTONOMOUS
    assert s.take_actions() == []
    assert s.output(1.20) == (0.3, 0.0, 0.0)


def test_a_stick_while_already_manual_does_not_abort_anything():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    assert s.take_actions() == []


def test_a_stick_while_estopped_does_not_take_over():
    s = SupervisorState(mode=MANUAL)
    s.on_estop_request(1.0, "STOP")
    s.take_actions()
    s.on_manual_twist(1.1, 0.05, 0.0, 0.0)
    assert s.mode == ESTOP
    assert s.take_actions() == []


def test_localisation_searching_halts_autonomy_and_says_why():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_localization_status(1.1, "SEARCHING")
    assert s.mode == MANUAL
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP]
    assert s.output(1.1) == (0.0, 0.0, 0.0)
    status = s.status(1.1)
    assert status["reason"] == "localisation SEARCHING"
    assert status["localization_state"] == "SEARCHING"


def test_localisation_off_halts_autonomy_too():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_localization_status(1.1, "OFF")
    assert s.mode == MANUAL
    assert s.status(1.1)["reason"] == "localisation OFF"


def test_localisation_recovering_does_not_resume_autonomy():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_localization_status(1.1, "OFF")
    s.take_actions()
    s.on_localization_status(2.0, "OK")
    assert s.mode == MANUAL
    assert s.take_actions() == []


def test_localisation_loss_does_not_stop_manual_driving():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_localization_status(1.0, "OFF")
    assert s.mode == MANUAL
    assert s.output(1.0) == (0.05, 0.0, 0.0)
    assert s.status(1.0)["localization_state"] == "OFF"


def test_autonomous_is_refused_while_localisation_is_lost():
    s = SupervisorState(mode=MANUAL)
    s.on_localization_status(1.0, "OFF")
    assert s.on_mode_request(1.1, AUTONOMOUS) is not None
    assert s.mode == MANUAL
    s.on_localization_status(2.0, "OK")
    assert s.on_mode_request(2.1, AUTONOMOUS) is None
    assert s.mode == AUTONOMOUS


def test_a_stateless_localisation_status_does_not_re_permit_autonomy():
    # `{"state": null}` and a status with no state key at all both arrive
    # here as None. None means "nothing has ever arrived", which is the one
    # value the autonomous guard lets through - so the wire must not be
    # able to produce it.
    s = SupervisorState(mode=MANUAL)
    s.on_localization_status(1.0, "OFF")
    s.on_localization_status(1.5, None)
    assert s.on_mode_request(1.6, AUTONOMOUS) is not None


def test_link_loss_does_not_stop_autonomy():
    # Rule 4: the ground station going quiet is what link loss looks like
    # from here, and in autonomous mode it streams nothing anyway - only
    # a deflected stick, which a dead link cannot deliver either.
    s = SupervisorState(mode=AUTONOMOUS)
    t = 1.0
    while t < 11.0:
        s.on_autonomy_twist(t, 0.3, 0.0, 0.0)
        assert s.output(t) == (0.3, 0.0, 0.0)
        t += 0.2
    assert s.mode == AUTONOMOUS


def test_link_loss_stops_manual_via_the_deadman_and_never_clears_the_estop():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_estop_request(1.1, "STOP")
    s.take_actions()
    # ... and then the link dies: nothing arrives at all for a minute
    assert s.output(61.0) == (0.0, 0.0, 0.0)
    assert s.mode == ESTOP
    assert s.estop_latched is True


def test_clearing_the_estop_latch_zeroes_a_stale_autonomy_buffer():
    # Orchestrator ruling (Task 2's checker found this hole): clearing the
    # e-stop latch back to manual must also zero the autonomy buffer, not
    # just the manual one. Nav2 can still be winding down when the latch
    # clears and keep publishing a stale /autonomy_twist in the meantime;
    # that stale twist must not survive to drive the rover the instant the
    # operator re-requests autonomous.
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_estop_request(1.1, "STOP")
    s.take_actions()
    s.on_autonomy_twist(1.2, 0.3, 0.0, 0.0)    # nav2 still winding down
    s.on_mode_request(1.3, MANUAL)
    s.on_mode_request(1.35, AUTONOMOUS)
    assert s.output(1.4) == (0.0, 0.0, 0.0)
