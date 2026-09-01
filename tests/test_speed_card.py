import pytest

from ground_station.gamepad_input import (MAX_LINEAR_SPEED,
                                          MIN_SETTABLE_LINEAR_SPEED,
                                          MAX_SETTABLE_LINEAR_SPEED)
from ground_station.ui.speed_card import SpeedCard


def test_it_opens_at_the_speed_the_rover_drove_at_before_the_slider_existed(qtbot):
    card = SpeedCard()
    qtbot.addWidget(card)

    assert card.speed == pytest.approx(MAX_LINEAR_SPEED)


def test_the_label_reads_in_metres_per_second(qtbot):
    card = SpeedCard()
    qtbot.addWidget(card)

    card.set_speed(0.25)

    assert card.value_label.text() == "0.25 m/s"


def test_moving_the_slider_announces_metres_per_second(qtbot):
    card = SpeedCard()
    qtbot.addWidget(card)
    seen = []
    card.speed_changed.connect(seen.append)

    card.slider.setValue(30)

    assert seen == [pytest.approx(0.30)]


def test_the_slider_cannot_ask_for_more_than_the_drive_train_takes(qtbot):
    card = SpeedCard()
    qtbot.addWidget(card)

    card.set_speed(9.0)

    assert card.speed == pytest.approx(MAX_SETTABLE_LINEAR_SPEED)


def test_the_slider_cannot_ask_for_a_standstill(qtbot):
    # A cap of zero is a rover that will not move with no explanation on
    # screen; the floor is slow enough to park with and still moves.
    card = SpeedCard()
    qtbot.addWidget(card)

    card.set_speed(0.0)

    assert card.speed == pytest.approx(MIN_SETTABLE_LINEAR_SPEED)


def test_every_whole_centimetre_in_the_range_is_reachable_by_dragging(qtbot):
    # Qt sliders are integer-only: the widget works in cm/s precisely so a
    # drag can land on 0.05 exactly rather than 0.0499.
    card = SpeedCard()
    qtbot.addWidget(card)

    for cm in range(int(MIN_SETTABLE_LINEAR_SPEED * 100),
                    int(MAX_SETTABLE_LINEAR_SPEED * 100) + 1):
        card.slider.setValue(cm)
        assert card.speed == pytest.approx(cm / 100.0)
