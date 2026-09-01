"""Tests for the TUNING drawer (ground_station/ui/tuning_card.py).

Style follows tests/test_site_card.py: pytest-qt's ``qtbot``, plain
attributes poked directly, behaviour asserted rather than pixels or
stylesheets.
"""

import pytest

from ground_station.ui.tuning_card import TuningCard

FULL_VALUES = {
    "step_lethal_m": 0.25,
    "slope_lethal_deg": 35.0,
    "floating_gap_m": 0.35,
    "wheel_trail_radius_m": 0.40,
    "goal_heal_radius_m": 1.4,
    "startup_clear_radius_m": 0.90,
    "climb_lethal_m": 0.25,
    "drop_lethal_m": 0.14,
    "relative_radius_m": 3.0,
}


def test_fresh_card_shows_no_invented_numbers_and_disables_apply(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)

    assert not card.apply_button.isEnabled()
    assert not card.revert_button.isEnabled()
    for row in card.rows.values():
        assert row.rover_label.text() == "-"
        assert not row.editor.isEnabled()


def test_rover_state_seeds_every_row_and_enables_apply(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)

    card.set_tuning_state(FULL_VALUES)

    for key, value in FULL_VALUES.items():
        row = card.rows[key]
        assert row.editor.isEnabled()
        assert row.editor.value() == pytest.approx(value)
        assert float(row.rover_label.text()) == pytest.approx(value)
    assert card.apply_button.isEnabled()
    assert card.revert_button.isEnabled()


def test_apply_sends_only_the_rows_the_operator_changed(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)
    card.set_tuning_state(FULL_VALUES)
    received = []
    card.values_applied.connect(received.append)

    card.rows["step_lethal_m"].editor.setValue(0.30)
    card.rows["goal_heal_radius_m"].editor.setValue(2.0)
    card.apply_button.click()

    assert len(received) == 1
    sent = received[0]
    assert sent == {"step_lethal_m": pytest.approx(0.30),
                    "goal_heal_radius_m": pytest.approx(2.0)}
    for value in sent.values():
        assert isinstance(value, float)


def test_apply_with_nothing_changed_sends_nothing(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)
    card.set_tuning_state(FULL_VALUES)
    received = []
    card.values_applied.connect(received.append)

    card.apply_button.click()

    assert received == []


def test_revert_puts_the_editors_back_to_the_rovers_values(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)
    card.set_tuning_state(FULL_VALUES)

    card.rows["step_lethal_m"].editor.setValue(0.90)
    card.rows["slope_lethal_deg"].editor.setValue(10.0)
    card.revert_button.click()

    assert card.rows["step_lethal_m"].editor.value() == pytest.approx(0.25)
    assert card.rows["slope_lethal_deg"].editor.value() == pytest.approx(35.0)


def test_editors_do_not_move_to_the_requested_values_after_apply(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)
    card.set_tuning_state(FULL_VALUES)

    card.rows["step_lethal_m"].editor.setValue(0.30)
    card.apply_button.click()

    # Nothing has come back from the rover yet: the editor still shows
    # what was requested, and the rover column has not moved.
    assert card.rows["step_lethal_m"].editor.value() == pytest.approx(0.30)
    assert float(card.rows["step_lethal_m"].rover_label.text()) == pytest.approx(0.25)

    # A later state message is what moves the rover column - the rover
    # accepted the change and republished its state.
    accepted = dict(FULL_VALUES, step_lethal_m=0.30)
    card.set_tuning_state(accepted)

    assert float(card.rows["step_lethal_m"].rover_label.text()) == pytest.approx(0.30)
    # The editor was left exactly where it was - the state message never
    # writes to an editor after the first seed.
    assert card.rows["step_lethal_m"].editor.value() == pytest.approx(0.30)


def test_a_rejected_change_leaves_the_rover_column_disagreeing_with_the_editor(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)
    card.set_tuning_state(FULL_VALUES)

    card.rows["wheel_trail_radius_m"].editor.setValue(0.50)
    card.apply_button.click()

    # The rover refuses the change and republishes exactly what it had.
    card.set_tuning_state(dict(FULL_VALUES))

    assert card.rows["wheel_trail_radius_m"].editor.value() == pytest.approx(0.50)
    assert float(card.rows["wheel_trail_radius_m"].rover_label.text()) == pytest.approx(0.40)


def test_a_state_message_before_any_report_is_ignored_if_none(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)

    card.set_tuning_state(None)

    assert not card.apply_button.isEnabled()
    for row in card.rows.values():
        assert row.rover_label.text() == "-"


def test_step_and_wheel_trail_tooltips_name_the_physical_ceiling(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)

    assert "0.282" in card.rows["step_lethal_m"].editor.toolTip()
    assert "0.445" in card.rows["wheel_trail_radius_m"].editor.toolTip()


def test_every_row_has_a_tooltip(qtbot):
    card = TuningCard()
    qtbot.addWidget(card)

    for row in card.rows.values():
        assert row.editor.toolTip()
