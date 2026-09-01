"""Tests for the SITE drawer (ground_station/ui/site_card.py).

Style follows tests/test_nav_row.py: pytest-qt's ``qtbot``, plain attributes
poked directly, behaviour asserted rather than pixels or stylesheets.
"""

from PySide6.QtCore import Qt

from ground_station.landmark_table import Landmark, LandmarkTable, MarkerSpec
from ground_station.models import ProbeResult, Sighting, SightingsReport
from ground_station.site_frame import site_to_map
from ground_station.ui.dashboard_page import DashboardPage
from ground_station.ui.site_card import SiteCard


def table(*landmarks, site_name="ERC 2026 marsyard"):
    return LandmarkTable(site_name=site_name, marker=MarkerSpec(),
                          landmarks=tuple(landmarks))


def three_landmark_table(note=""):
    return table(
        Landmark(id="51", x=12.40, y=3.10, note=note),
        Landmark(id="52", x=18.05, y=-2.20),
        Landmark(id="53", x=9.70, y=-6.45),
    )


def _row_widget(card, row_index):
    item = card.landmark_list.item(row_index)
    return card.landmark_list.itemWidget(item)


def _find_row(card, landmark_id):
    for i in range(card.landmark_list.count()):
        w = _row_widget(card, i)
        if w.id == landmark_id:
            return w
    return None


def test_fresh_card_has_no_table_and_disabled_controls(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    assert card.state_pill.text() == "NO TABLE"
    assert not card.solve_button.isEnabled()
    assert not card.probe_button.isEnabled()
    assert not card.lock_button.isEnabled()


def test_set_table_gives_one_ticked_row_per_landmark(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    card.set_table(three_landmark_table())

    assert card.landmark_list.count() == 3
    for i in range(3):
        row = _row_widget(card, i)
        assert row.checkbox.isChecked()
    assert card.state_pill.text() == "0 OF 3 MEASURED"


def test_note_with_markup_is_shown_literally_as_plain_text(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    card.set_table(three_landmark_table(note="<b>x</b>"))
    row = _find_row(card, "51")
    assert "<b>x</b>" in row.label.text()
    assert row.label.textFormat() == Qt.TextFormat.PlainText


def test_one_measurement_is_not_enough_to_solve(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    card.set_table(three_landmark_table())
    card.set_measurement("51", 4.0, 1.0, "good")
    assert card.state_pill.text() == "1 OF 3 MEASURED"
    assert not card.solve_button.isEnabled()


def _rigid_map_points(t_x, t_y, t_yaw, landmarks):
    import math
    c, s = math.cos(t_yaw), math.sin(t_yaw)
    out = {}
    for lm in landmarks:
        mx = c * lm.x - s * lm.y + t_x
        my = s * lm.x + c * lm.y + t_y
        out[lm.id] = (mx, my)
    return out


def test_two_measurements_enable_solve_and_produce_a_transform(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    tbl = three_landmark_table()
    card.set_table(tbl)
    pts = _rigid_map_points(5.0, -2.0, 0.3, tbl.landmarks)
    card.set_measurement("51", *pts["51"], "good")
    card.set_measurement("52", *pts["52"], "good")

    assert card.solve_button.isEnabled()
    card.solve_button.click()
    assert card.transform is not None
    assert "RMS" in card.rms_pill.text()


def test_exact_rigid_image_gives_near_zero_rms(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    tbl = three_landmark_table()
    card.set_table(tbl)
    pts = _rigid_map_points(1.0, 2.0, 0.5, tbl.landmarks)
    for lm in tbl.landmarks:
        card.set_measurement(lm.id, *pts[lm.id], "good")
    card.solve_button.click()
    assert card.transform.rms_m < 1e-6
    assert "0.00" in card.rms_pill.text()


def test_an_outlier_is_named_and_dropping_it_helps(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    # Four landmarks (a square) - with only three, a single least-squares
    # fit can spread the corruption onto a *different* point than the one
    # that was actually wrong, the same reason T1's own outlier test uses
    # four correspondences rather than three.
    tbl = table(
        Landmark(id="51", x=0.0, y=0.0),
        Landmark(id="52", x=10.0, y=0.0),
        Landmark(id="53", x=10.0, y=10.0),
        Landmark(id="54", x=0.0, y=10.0),
    )
    card.set_table(tbl)
    pts = _rigid_map_points(0.0, 0.0, 0.0, tbl.landmarks)
    for lm in tbl.landmarks:
        card.set_measurement(lm.id, *pts[lm.id], "good")
    # 53 is deliberately wrong.
    card.set_measurement("53", pts["53"][0] + 2.0, pts["53"][1], "good")
    card.solve_button.click()
    assert card.transform.worst_id == "53"
    assert "53" in card.detail_label.text()

    bad_rms = card.transform.rms_m
    row = _find_row(card, "53")
    row.checkbox.setChecked(False)
    card.solve_button.click()
    assert card.transform.rms_m < bad_rms


def test_exactly_two_landmarks_shows_the_caveat(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    tbl = table(Landmark(id="51", x=0.0, y=0.0),
                Landmark(id="52", x=10.0, y=0.0))
    card.set_table(tbl)
    pts = _rigid_map_points(0.0, 0.0, 0.0, tbl.landmarks)
    for lm in tbl.landmarks:
        card.set_measurement(lm.id, *pts[lm.id], "good")
    card.solve_button.click()
    assert "third landmark" in card.detail_label.text()


def test_scale_hint_off_by_15_percent_warns(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    tbl = table(Landmark(id="51", x=0.0, y=0.0),
                Landmark(id="52", x=10.0, y=0.0))
    card.set_table(tbl)
    # Measured baseline 11.5 m against a published 10 m -> scale_hint 1.15.
    card.set_measurement("51", 0.0, 0.0, "good")
    card.set_measurement("52", 11.5, 0.0, "good")
    card.solve_button.click()
    assert "spread" in card.detail_label.text()


def test_unknown_id_measurement_is_shown_and_excluded_from_the_fit(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    tbl = three_landmark_table()
    card.set_table(tbl)
    pts = _rigid_map_points(0.0, 0.0, 0.0, tbl.landmarks)
    card.set_measurement("51", *pts["51"], "good")
    card.set_measurement("52", *pts["52"], "good")
    card.set_measurement("99", 1.0, 1.0, "good")

    row = _find_row(card, "99")
    assert row is not None
    assert "not in table" in row.label.text()

    card.solve_button.click()
    assert "99" not in card.transform.ids


def test_lock_button_requires_a_transform_then_toggles(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    tbl = three_landmark_table()
    card.set_table(tbl)
    pts = _rigid_map_points(0.0, 0.0, 0.0, tbl.landmarks)
    card.set_measurement("51", *pts["51"], "good")
    card.set_measurement("52", *pts["52"], "good")

    assert not card.lock_button.isEnabled()
    card.lock_button.setChecked(True)  # forced, bypassing enabled-state
    assert not card.locked  # guarded: no transform yet, ignored

    card.solve_button.click()
    assert card.lock_button.isEnabled()

    seen = []
    card.lock_changed.connect(seen.append)
    card.lock_button.click()
    assert card.locked
    assert seen[-1] is card.transform
    assert not card.solve_button.isEnabled()
    assert not card.probe_button.isEnabled()
    assert not card.anchor_button.isEnabled()
    for i in range(card.landmark_list.count()):
        assert not _row_widget(card, i).checkbox.isEnabled()

    card.lock_button.click()
    assert not card.locked
    assert seen[-1] is None


def test_apply_sightings_produces_measurements_or_an_error(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    card.set_table(three_landmark_table())

    good = SightingsReport(
        phase="running", dictionary="DICT_5X5_100", detector_ok=True,
        error=None,
        sightings=[
            Sighting(id="51", x=4.0, y=1.0, z=0.4, n=60, spread_m=0.02,
                     range_m=4.0, last_seen_s=0.1, quality="good"),
            Sighting(id="52", x=8.0, y=-1.0, z=0.4, n=60, spread_m=0.02,
                     range_m=8.0, last_seen_s=0.1, quality="good"),
        ])
    card.apply_sightings(good)
    assert card.state_pill.text() == "2 OF 3 MEASURED"

    bad = SightingsReport(phase="running", dictionary="DICT_5X5_100",
                          detector_ok=False, error="cv2.aruco not available",
                          sightings=[])
    card.apply_sightings(bad)
    assert "cv2.aruco not available" in card.state_pill.text()


def test_apply_probe_result_ok_and_failure(qtbot):
    card = SiteCard()
    qtbot.addWidget(card)
    card.set_table(three_landmark_table())

    ok = ProbeResult(request_id="p-1-1", ok=True, label="51",
                      x=4.0, y=1.0, z=0.4, range_m=4.0, samples=40,
                      valid_fraction=0.8, error=None)
    card.apply_probe_result(ok)
    assert card.state_pill.text() == "1 OF 3 MEASURED"

    fail = ProbeResult(request_id="p-1-2", ok=False, label="52",
                       x=None, y=None, z=None, range_m=None, samples=0,
                       valid_fraction=0.0, error="no depth image yet")
    card.apply_probe_result(fail)
    assert "no depth image yet" in card.state_pill.text()
    assert card.state_pill.text() != "2 OF 3 MEASURED"


def test_dashboard_page_has_a_hidden_site_card_and_the_stage_did_not_move(qtbot):
    page = DashboardPage()
    qtbot.addWidget(page)
    assert hasattr(page, "site_card")
    assert not page.site_card.isVisible()
    assert page.nav_row is not None
    assert page.video_panel is not None
