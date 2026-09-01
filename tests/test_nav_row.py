import math

import pytest

from ground_station import theme
from ground_station.models import ModeState, NavStatus, Waypoint
from ground_station.site_frame import SiteTransform, map_to_site, site_to_map
from ground_station.ui.nav_row import NavRow


def transform(**over):
    base = dict(x=10.0, y=5.0, yaw=math.pi / 2, rms_m=0.01, max_residual_m=0.02,
                worst_id=None, n_points=2, scale_hint=1.0, ids=("51", "52"))
    base.update(over)
    return SiteTransform(**base)


def nav(**over):
    base = dict(state="idle", run_id=None, waypoint_index=None, waypoint_count=0,
                distance_remaining_m=None, eta_s=None, error="", mode="autonomous",
                coordinator_state=None)
    base.update(over)
    return NavStatus(**base)


def mode(**over):
    base = dict(mode="autonomous", reason="", source=None, deadman_active=False,
                estop_latched=False, localization_state="OK", source_age_s=0.1)
    base.update(over)
    return ModeState(**base)


def armed_row(qtbot, **over):
    row = NavRow()
    qtbot.addWidget(row)
    row.set_mode_state(mode(**over))
    row.set_state(nav())
    return row


def test_go_is_disabled_with_no_waypoints(qtbot):
    row = armed_row(qtbot)
    assert not row.go_button.isEnabled()


def test_go_is_disabled_when_the_rover_is_not_in_autonomous(qtbot):
    row = armed_row(qtbot, mode="manual")
    row.waypoints.add(Waypoint(1.0, 1.0))
    row.refresh_waypoints()
    assert not row.go_button.isEnabled()
    assert "autonomous" in row.hint_label.text().lower()


def test_go_is_enabled_with_waypoints_in_autonomous_and_emits_them(qtbot):
    row = armed_row(qtbot)
    row.waypoints.add(Waypoint(3.0, -1.5))
    row.refresh_waypoints()
    assert row.go_button.isEnabled()
    sent = []
    row.go_requested.connect(sent.append)
    row.go_button.click()
    assert sent == [[Waypoint(3.0, -1.5, None)]]


def test_the_autonomous_button_asks_for_the_mode_and_is_confirmed(qtbot):
    row = armed_row(qtbot, mode="manual")
    emitted = []
    row.autonomous_requested.connect(lambda: emitted.append(True))
    row.confirm_autonomous = lambda: False
    row.autonomous_button.click()
    assert emitted == []
    row.confirm_autonomous = lambda: True
    row.autonomous_button.click()
    assert emitted == [True]


def test_the_autonomous_button_is_disabled_once_the_rover_is_autonomous(qtbot):
    assert not armed_row(qtbot).autonomous_button.isEnabled()


def test_add_takes_typed_coordinates(qtbot):
    row = armed_row(qtbot)
    row.x_input.setText("3.0")
    row.y_input.setText("-1.5")
    row.add_button.click()
    assert row.waypoints.items == [Waypoint(3.0, -1.5, None)]
    assert row.waypoint_list.count() == 1


def test_add_refuses_a_bad_coordinate_and_says_why(qtbot):
    row = armed_row(qtbot)
    row.x_input.setText("three")
    row.y_input.setText("0")
    row.add_button.click()
    assert len(row.waypoints) == 0
    assert "not a number" in row.hint_label.text()


def test_a_clicked_map_point_appends_a_waypoint(qtbot):
    row = armed_row(qtbot)
    row.append_world_point(4.0, 2.0)
    assert row.waypoints.items == [Waypoint(4.0, 2.0, None)]


def test_pause_resume_and_abort_follow_the_run_state(qtbot):
    row = armed_row(qtbot)
    row.set_state(nav(state="running", run_id="gs-1", waypoint_index=0,
                      waypoint_count=2))
    assert row.pause_button.isEnabled()
    assert not row.resume_button.isEnabled()
    assert row.abort_button.isEnabled()
    row.set_state(nav(state="paused", run_id="gs-1", waypoint_index=0,
                      waypoint_count=2))
    assert not row.pause_button.isEnabled()
    assert row.resume_button.isEnabled()


def test_go_is_disabled_while_a_run_is_active(qtbot):
    row = armed_row(qtbot)
    row.waypoints.add(Waypoint(1.0, 1.0))
    row.refresh_waypoints()
    row.set_state(nav(state="running", run_id="gs-1", waypoint_count=1))
    assert not row.go_button.isEnabled()


def test_abort_is_a_panic_button_with_no_dialog_in_the_way(qtbot):
    # Handing the rover TO autonomy is the decision worth a confirmation;
    # taking it back is not, and a modal between the operator and stopping
    # a run is a hazard rather than a safeguard.
    row = armed_row(qtbot)
    row.set_state(nav(state="running"))
    assert not hasattr(row, "confirm_abort")
    with qtbot.waitSignal(row.abort_requested):
        row.abort_button.click()


def test_the_status_line_shows_waypoint_distance_and_eta(qtbot):
    row = armed_row(qtbot)
    row.set_state(nav(state="running", run_id="gs-1", waypoint_index=1,
                      waypoint_count=3, distance_remaining_m=12.4, eta_s=248.0))
    assert "2/3" in row.waypoint_pill.text()
    assert "12.4" in row.progress_label.text()
    assert "4:08" in row.progress_label.text()


def test_an_error_is_shown_verbatim_and_never_as_markup(qtbot):
    row = armed_row(qtbot)
    row.set_state(nav(state="refused", error="mode is <manual>, not autonomous"))
    assert row.error_pill.text() == "mode is <manual>, not autonomous"
    # isVisibleTo, not isVisible: a widget added with qtbot.addWidget is
    # never shown, so isVisible() is False however the pill was set. This
    # is the form tests/test_main_window.py:822 already uses.
    assert row.error_pill.isVisibleTo(row)


def test_no_status_leaves_only_the_waypoint_editing_alive(qtbot):
    row = NavRow()
    qtbot.addWidget(row)
    row.set_state(None)
    assert not row.go_button.isEnabled()
    assert not row.pause_button.isEnabled()
    assert not row.abort_button.isEnabled()
    assert row.add_button.isEnabled()          # editing a list needs no rover


def test_clicking_the_canvas_adds_a_waypoint_to_the_row(qtbot):
    row = armed_row(qtbot)
    row.map_view.point_clicked.emit(4.0, 2.0)
    assert row.waypoints.items == [Waypoint(4.0, 2.0, None)]


# -- the map tools ----------------------------------------------------------

def test_the_map_can_be_zoomed_and_unfollowed_from_the_row(qtbot):
    # The canvas had zoom and follow logic that nothing on screen could
    # reach: placing waypoints on a map you cannot scale is working blind.
    row = NavRow()
    qtbot.addWidget(row)
    row.map_view.resize(400, 300)
    before = row.map_view.transform.metres_per_pixel
    row.zoom_in_button.click()
    assert row.map_view.transform.metres_per_pixel < before
    row.zoom_out_button.click()
    assert row.map_view.transform.metres_per_pixel == before

    # Follow starts on, so the view tracks the rover; switching it off pins
    # the view to the ground instead.
    assert row.follow_button.isChecked()
    row.set_pose({"x": 5.0, "y": 0.0, "yaw": 0.0})
    assert row.map_view.transform.centre_x == 5.0
    row.follow_button.click()
    row.set_pose({"x": 9.0, "y": 0.0, "yaw": 0.0})
    assert row.map_view.transform.centre_x == 5.0


# -- the site transform ------------------------------------------------------

def test_with_no_transform_refresh_waypoints_is_unchanged_and_has_no_site_marker(qtbot):
    # The regression assertion: with no transform locked, this is exactly
    # today's behaviour.
    row = armed_row(qtbot)
    row.waypoints.add(Waypoint(3.0, -1.5))
    row.refresh_waypoints()
    assert row.map_view.waypoints == row.waypoints.items
    assert "site" not in row.editor_title.text().lower()
    assert "site" not in row.waypoint_list.item(0).text().lower()


def test_a_locked_transform_converts_the_map_view_waypoints(qtbot):
    row = armed_row(qtbot)
    t = transform()
    row.set_site_transform(t)
    row.waypoints.add(Waypoint(3.0, -1.5))
    row.refresh_waypoints()
    expected_x, expected_y = site_to_map(t, 3.0, -1.5)
    drawn = row.map_view.waypoints[0]
    assert drawn.x == pytest.approx(expected_x)
    assert drawn.y == pytest.approx(expected_y)


def test_go_requested_still_carries_the_site_numbers_under_a_locked_transform(qtbot):
    row = armed_row(qtbot)
    row.set_site_transform(transform())
    row.waypoints.add(Waypoint(3.0, -1.5))
    row.refresh_waypoints()
    sent = []
    row.go_requested.connect(sent.append)
    row.go_button.click()
    assert sent == [[Waypoint(3.0, -1.5, None)]]


def test_set_site_transform_none_restores_todays_behaviour_exactly(qtbot):
    row = armed_row(qtbot)
    row.set_site_transform(transform())
    row.set_site_transform(None)
    row.waypoints.add(Waypoint(3.0, -1.5))
    row.refresh_waypoints()
    assert row.map_view.waypoints == row.waypoints.items
    assert "site" not in row.editor_title.text().lower()
    assert "site" not in row.waypoint_list.item(0).text().lower()


def test_the_site_marker_appears_only_when_a_transform_is_locked(qtbot):
    row = armed_row(qtbot)
    assert "site" not in row.editor_title.text().lower()
    row.set_site_transform(transform())
    assert "site" in row.editor_title.text().lower()
    row.waypoints.add(Waypoint(1.0, 1.0))
    row.refresh_waypoints()
    assert "site" in row.waypoint_list.item(0).text().lower()
    row.set_site_transform(None)
    assert "site" not in row.editor_title.text().lower()


def test_a_canvas_click_round_trips_through_the_locked_transform(qtbot):
    # §3.9: a click on the plan canvas arrives in map frame and must be
    # converted BACK to site here, or the wire conversion at Go would apply
    # the transform a second time and the waypoint would land somewhere the
    # operator never pointed at.
    row = armed_row(qtbot)
    t = transform()
    row.set_site_transform(t)
    mx, my = 4.0, 2.0
    row.map_view.point_clicked.emit(mx, my)
    expected_x, expected_y = map_to_site(t, mx, my)
    stored = row.waypoints.items[0]
    assert stored.x == pytest.approx(expected_x)
    assert stored.y == pytest.approx(expected_y)
    # A click lands where it was clicked: converting the stored site point
    # back to map through the same transform returns the original click.
    drawn = row.map_view.waypoints[0]
    assert drawn.x == pytest.approx(mx, abs=1e-9)
    assert drawn.y == pytest.approx(my, abs=1e-9)


def test_a_canvas_click_with_no_transform_appends_verbatim(qtbot):
    row = armed_row(qtbot)
    row.map_view.point_clicked.emit(4.0, 2.0)
    assert row.waypoints.items == [Waypoint(4.0, 2.0, None)]


def test_resume_is_lit_only_while_the_rover_is_waiting_on_it(qtbot):
    # Resume is pressed at every waypoint now (the coordinator holds in
    # Waiting with movement disabled until it is), so it carries Go's fill -
    # but only when pressing it is the next thing to do.
    row = NavRow()
    qtbot.addWidget(row)
    row.set_state(nav(state="running"))
    assert row.resume_button.styleSheet() == ""
    row.set_state(nav(state="paused"))
    assert row.resume_button.isEnabled()
    assert theme.ACCENT in row.resume_button.styleSheet()
