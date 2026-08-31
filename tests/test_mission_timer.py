"""The mission clock, against a fake clock - the same idiom the rest of
this code base times things with."""

from ground_station.ui.mission_timer import MissionTimer, format_elapsed


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(qtbot):
    clock = Clock()
    timer = MissionTimer(clock=clock)
    qtbot.addWidget(timer)
    return timer, clock


def test_elapsed_formats_as_minutes_then_hours():
    assert format_elapsed(0) == "00:00"
    assert format_elapsed(9.7) == "00:09"
    assert format_elapsed(75) == "01:15"
    assert format_elapsed(3600) == "1:00:00"
    assert format_elapsed(-5) == "00:00"       # never counts backwards


def test_the_clock_runs_only_while_started(qtbot):
    timer, clock = make(qtbot)
    assert timer.elapsed() == 0.0
    clock.t += 10
    assert timer.elapsed() == 0.0              # not started: still zero

    timer.start_button.click()
    assert timer.running
    clock.t += 42
    timer.tick()
    assert timer.elapsed() == 42
    assert timer.time_label.text() == "00:42"


def test_stopping_banks_the_time_and_starting_again_continues_it(qtbot):
    # A mission that was paused is still the same mission; only Reset
    # throws the time away.
    timer, clock = make(qtbot)
    timer.start()
    clock.t += 30
    timer.stop()
    clock.t += 100                              # stopped: this does not count
    assert timer.elapsed() == 30
    timer.start()
    clock.t += 5
    assert timer.elapsed() == 35


def test_reset_is_refused_while_the_clock_is_running(qtbot):
    # One click must not be able to throw away a run that is being timed.
    timer, clock = make(qtbot)
    timer.start()
    clock.t += 20
    timer.reset()
    assert timer.elapsed() == 20
    assert not timer.reset_button.isEnabled()

    timer.stop()
    assert timer.reset_button.isEnabled()
    timer.reset()
    assert timer.elapsed() == 0.0
    assert timer.time_label.text() == "00:00"


def test_the_button_says_what_it_will_do(qtbot):
    timer, _ = make(qtbot)
    assert timer.start_button.text() == "Start"
    timer.start_button.click()
    assert timer.start_button.text() == "Stop"
    timer.start_button.click()
    assert timer.start_button.text() == "Start"
