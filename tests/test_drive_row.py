from ground_station.models import DriveState
from ground_station.ui.drive_row import DriveRow


def state(**over):
    base = dict(connected=True, lease=True, coordinator_state="Manual",
                deadman_active=False, twist_age_s=0.1, last_action=None,
                last_error=None)
    base.update(over)
    return DriveState(**base)


def test_this_row_does_not_own_stop(qtbot):
    # STOP is the window's header button now: one of it, visible in every
    # view, at a size that needs no aiming. A second STOP here would make
    # an emergency a choice between two buttons.
    row = DriveRow()
    qtbot.addWidget(row)
    assert not hasattr(row, "stop_button")


def test_manual_and_init_are_disabled_with_no_status(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    assert not row.manual_button.isEnabled()
    assert not row.init_button.isEnabled()


def test_the_setup_drawer_starts_closed_and_toggles(qtbot):
    # The five once-per-boot buttons (two of which move the wheels) stay
    # out of the way of the one that is pressed constantly.
    row = DriveRow()
    qtbot.addWidget(row)
    assert not row.setup_panel.isVisible()
    row.setup_toggle.click()
    assert row.setup_panel.isVisibleTo(row)
    assert "▾" in row.setup_toggle.text()
    row.setup_toggle.click()
    assert not row.setup_panel.isVisibleTo(row)


def test_init_asks_for_confirmation_before_emitting(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state())
    emitted = []
    row.init_requested.connect(lambda: emitted.append(True))
    row.confirm_init = lambda: False
    row.init_button.click()
    assert emitted == []
    row.confirm_init = lambda: True
    row.init_button.click()
    assert emitted == [True]


def test_status_line_shows_coordinator_state_and_deadman(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state(coordinator_state="Idle", deadman_active=True))
    assert "idle" in row.state_pill.text().lower()
    assert not row.deadman_pill.isHidden()
    assert "deadman" in row.deadman_pill.text().lower()


def test_manual_shows_arming_while_preparing(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state(coordinator_state="PrepareManual"))
    assert "arming" in row.state_pill.text().lower()


def test_status_line_shows_the_last_action(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state(last_action="stop"))
    assert "last: stop" in row.last_action_label.text().lower()


def test_init_can_only_be_clicked_once(qtbot):
    # BemaServer::init() is one-shot on the rover (a second F0 is ignored),
    # so the button goes dark after the first confirmed click and stays
    # dark even as fresh statuses arrive.
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state())
    emitted = []
    row.init_requested.connect(lambda: emitted.append(True))
    row.confirm_init = lambda: True
    row.init_button.click()
    assert emitted == [True]
    assert not row.init_button.isEnabled()
    row.set_state(state())               # a new status must not re-enable it
    assert not row.init_button.isEnabled()
    row.init_button.click()              # disabled: no dialog, no emit
    assert emitted == [True]


def test_init_re_arms_after_a_connection_cycle(qtbot):
    # m_initialized lives on the rover and resets on a rover power-cycle -
    # after that the operator NEEDS Init again, but a GS-session latch that
    # never clears would leave the button dead. A connection cycle
    # (connected -> not connected/None -> connected again) is the signal:
    # the drive link came back, possibly to a freshly booted rover.
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state())
    row.confirm_init = lambda: True
    row.init_button.click()
    assert not row.init_button.isEnabled()

    row.set_state(None)                  # link drops
    row.set_state(state())               # link comes back (rover may have rebooted)
    assert row.init_button.isEnabled()


def test_repeated_set_state_without_a_gap_keeps_init_disabled(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state())
    row.confirm_init = lambda: True
    row.init_button.click()
    assert not row.init_button.isEnabled()

    row.set_state(state())               # still connected the whole time
    assert not row.init_button.isEnabled()


def test_a_cancelled_init_stays_clickable(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state())
    row.confirm_init = lambda: False
    row.init_button.click()
    assert row.init_button.isEnabled()


def test_every_button_explains_itself(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    for b in (row.manual_button, row.init_button,
              row.reset_enc_button, row.reset_odom_button,
              row.mode_button, row.state_button):
        assert b.toolTip(), b.text()
    # The two wheel-moving buttons say so, unmissably.
    assert "WHEELS WILL MOVE" in row.init_button.toolTip()
    assert "WHEELS WILL MOVE" in row.reset_enc_button.toolTip()


def mode(**over):
    from ground_station.models import ModeState
    base = dict(mode="manual", reason="", source="/manual_twist",
                deadman_active=False, estop_latched=False,
                localization_state="OK", source_age_s=0.05)
    base.update(over)
    return ModeState(**base)


