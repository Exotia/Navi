from ground_station import theme
from ground_station.models import MapState
from ground_station.ui.map_row import MapRow


def state(**overrides):
    base = dict(cells_seen=100, extent_m=(4.0, 3.5), tiles=3, loaded=None,
                maps=["a", "b"], last_command=None)
    base.update(overrides)
    return MapState(**base)


def test_no_state_disables_everything(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(None)
    assert not row.load_button.isEnabled() and not row.save_button.isEnabled()
    assert not row.clear_button.isEnabled()
    assert "no status" in row.status_label.text().lower()


def test_the_dropdown_lists_the_saved_maps_and_keeps_the_selection(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state())
    assert [row.map_combo.itemText(i) for i in range(row.map_combo.count())] == ["a", "b"]
    row.map_combo.setCurrentIndex(1)
    row.set_state(state(maps=["a", "b", "c"]))
    assert row.map_combo.currentText() == "b"
    assert row.load_button.isEnabled()


def test_load_emits_the_selected_name(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state())
    row.map_combo.setCurrentIndex(1)
    with qtbot.waitSignal(row.load_requested) as blocker:
        row.load_button.click()
    assert blocker.args == ["b"]


def test_save_asks_for_a_name_and_refuses_empty_invalid_and_duplicates(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state())
    emitted = []
    row.save_requested.connect(emitted.append)
    for answer in ["", "has space", "a"]:
        row.ask_name = lambda answer=answer: answer
        row.save_button.click()
    assert emitted == []
    assert "refused" in row.status_label.text().lower()
    row.ask_name = lambda: "yard-day1"
    row.save_button.click()
    assert emitted == ["yard-day1"]


def test_clear_needs_confirmation(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state())
    emitted = []
    row.clear_requested.connect(lambda: emitted.append(True))
    row.confirm_clear = lambda: False
    row.clear_button.click()
    assert emitted == []
    row.confirm_clear = lambda: True
    row.clear_button.click()
    assert emitted == [True]


def test_a_stale_save_refusal_notice_clears_once_last_command_changes(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state())
    row.ask_name = lambda: "has space"
    row.save_button.click()
    assert "refused" in row.status_label.text().lower()
    # An unrelated update that repeats the same last_command (still None)
    # must not wipe the refusal notice out from under the operator.
    row.set_state(state())
    assert "refused" in row.status_label.text().lower()
    # A new last_command - even an unrelated one - means the refusal is
    # no longer news, so the stale notice must not linger forever.
    row.set_state(state(last_command={"action": "load", "name": "a", "ok": True}))
    assert "refused" not in row.status_label.text().lower()


def test_the_status_line_shows_size_loaded_map_and_the_last_command_outcome(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state(loaded="a", last_command={"action": "save", "name": "x",
                                                   "ok": False, "error": "exists"}))
    text = row.status_label.text()
    assert "100" in text and "4.0" in text and "a" in text and "exists" in text


class FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def test_the_last_command_outcome_is_shown_for_ten_seconds_then_goes(qtbot):
    clock = FakeClock()
    row = MapRow(clock=clock)
    qtbot.addWidget(row)
    row.set_state(state(last_command={"action": "save", "name": "x", "ok": True}))
    assert "save" in row.status_label.text()

    clock.t = 105.0
    row.refresh()
    assert "save" in row.status_label.text()

    clock.t = 111.0
    row.refresh()
    assert "save" not in row.status_label.text()
    # The rest of the line is untouched.
    assert "100" in row.status_label.text()


def test_the_ten_seconds_are_counted_from_the_change_not_from_the_last_status(qtbot):
    clock = FakeClock()
    row = MapRow(clock=clock)
    qtbot.addWidget(row)
    command = {"action": "save", "name": "x", "ok": True}
    row.set_state(state(last_command=command))
    for tick in range(1, 12):                      # status keeps arriving at 1 Hz
        clock.t = 100.0 + tick
        row.set_state(state(last_command=command))
    assert "save" not in row.status_label.text()


def test_a_failed_outcome_is_shown_in_red_and_an_ok_one_is_not(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state(last_command={"action": "save", "name": "x", "ok": False,
                                      "error": "already exists"}))
    text = row.status_label.text()
    assert "already exists" in text
    assert theme.BAD in text

    row.set_state(state(last_command={"action": "load", "name": "a", "ok": True}))
    assert theme.BAD not in row.status_label.text()


def test_refresh_accepts_the_windows_own_clock_reading(qtbot):
    clock = FakeClock()
    row = MapRow(clock=clock)
    qtbot.addWidget(row)
    row.set_state(state(last_command={"action": "clear", "name": None, "ok": True}))
    row.refresh(now=105.0)
    assert "clear" in row.status_label.text()
    row.refresh(now=111.0)
    assert "clear" not in row.status_label.text()


def test_a_status_drop_does_not_revive_an_old_outcome(qtbot):
    clock = FakeClock()
    row = MapRow(clock=clock)
    qtbot.addWidget(row)
    command = {"action": "save", "name": "x", "ok": False, "error": "disk full"}
    row.set_state(state(last_command=command))
    clock.t = 120.0
    row.refresh()
    assert "save" not in row.status_label.text()      # aged off

    row.set_state(None)                                # 3 s blip / rosbridge drop
    clock.t = 125.0
    row.set_state(state(last_command=command))         # same old command comes back
    assert "save" not in row.status_label.text()
