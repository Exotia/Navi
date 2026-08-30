"""The supervisor's rules, with time passed in rather than read - the same
fake-clock shape test_bema_session.py uses. No ROS here."""

from navi_supervisor.supervisor_state import (AUTONOMOUS, AUTONOMY_DEADMAN_S,
                                              CHASSIS_STOP, ESTOP, MANUAL,
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
