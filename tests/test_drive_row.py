from ground_station.models import DriveState
from ground_station.ui.drive_row import DriveRow


def state(**over):
    base = dict(connected=True, lease=True, coordinator_state="Manual",
                deadman_active=False, twist_age_s=0.1, last_action=None,
                last_error=None)
    base.update(over)
    return DriveState(**base)


def test_stop_is_always_enabled_even_with_no_status(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(None)
    assert row.stop_button.isEnabled()


def test_stop_emits_without_confirmation(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    with qtbot.waitSignal(row.stop_requested):
        row.stop_button.click()


def test_manual_and_init_are_disabled_with_no_status(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    assert not row.manual_button.isEnabled()
    assert not row.init_button.isEnabled()
    assert row.stop_button.isEnabled()


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
    text = row.status_label.text().lower()
    assert "idle" in text and "deadman" in text


def test_manual_shows_arming_while_preparing(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state(coordinator_state="PrepareManual"))
    assert "arming" in row.status_label.text().lower()
