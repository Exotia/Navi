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


def test_the_status_line_shows_size_loaded_map_and_the_last_command_outcome(qtbot):
    row = MapRow()
    qtbot.addWidget(row)
    row.set_state(state(loaded="a", last_command={"action": "save", "name": "x",
                                                   "ok": False, "error": "exists"}))
    text = row.status_label.text()
    assert "100" in text and "4.0" in text and "a" in text and "exists" in text
