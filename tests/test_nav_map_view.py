from PySide6.QtCore import QPoint, Qt
from PySide6.QtGui import QMouseEvent
from PySide6.QtCore import QPointF

from ground_station.models import PathSummary, Waypoint
from ground_station.ui.nav_map_view import NavMapView


def click(view, px, py):
    event = QMouseEvent(QMouseEvent.Type.MouseButtonPress, QPointF(px, py),
                        Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton,
                        Qt.KeyboardModifier.NoModifier)
    view.mousePressEvent(event)


def test_a_click_emits_the_world_coordinate_under_the_cursor(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    view.resize(400, 300)
    view.set_view_centre(0.0, 0.0)
    view.set_metres_per_pixel(0.05)
    points = []
    view.point_clicked.connect(lambda x, y: points.append((x, y)))
    click(view, 200, 150)
    assert points[-1] == (0.0, 0.0)
    click(view, 200, 100)             # 50 px up the screen = +2.5 m in x
    assert abs(points[-1][0] - 2.5) < 1e-9 and abs(points[-1][1]) < 1e-9


def test_a_right_click_does_not_add_a_waypoint(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    points = []
    view.point_clicked.connect(lambda x, y: points.append((x, y)))
    event = QMouseEvent(QMouseEvent.Type.MouseButtonPress, QPointF(10, 10),
                        Qt.MouseButton.RightButton, Qt.MouseButton.RightButton,
                        Qt.KeyboardModifier.NoModifier)
    view.mousePressEvent(event)
    assert points == []


def test_following_the_rover_recentres_the_view_on_the_pose(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    view.resize(400, 300)
    view.set_pose({"x": 10.0, "y": -4.0, "yaw": 0.0})
    assert view.transform.centre_x == 10.0 and view.transform.centre_y == -4.0


def test_pinning_the_view_stops_it_following_the_rover(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    view.resize(400, 300)
    view.set_pose({"x": 10.0, "y": -4.0, "yaw": 0.0})
    view.set_follow(False)
    view.set_pose({"x": 20.0, "y": -8.0, "yaw": 0.0})
    assert view.transform.centre_x == 10.0


def test_the_view_paints_without_a_pose_a_plan_or_waypoints(qtbot):
    # The canvas is on screen before the rover has said anything. A paint
    # that raises there takes the whole window down.
    view = NavMapView()
    qtbot.addWidget(view)
    view.resize(400, 300)
    view.grab()


def test_the_view_paints_a_plan_waypoints_and_a_pose(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    view.resize(400, 300)
    view.set_pose({"x": 0.0, "y": 0.0, "yaw": 0.5})
    view.set_waypoints([Waypoint(3.0, -1.5), Waypoint(8.0, -1.5)])
    view.set_path_summary(PathSummary("gs-1", "map", [(0.0, 0.0), (3.0, -1.5)],
                                      [(3.0, -1.5)], 3.35, 941))
    view.grab()


def test_an_empty_summary_clears_the_drawn_plan(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    view.set_path_summary(PathSummary("gs-1", "map", [(0.0, 0.0)], [], 0.0, 1))
    view.set_path_summary(PathSummary("gs-1", "map", [], [], 0.0, 0))
    assert view.path_points == []


def test_zoom_buttons_change_the_scale_within_bounds(qtbot):
    view = NavMapView()
    qtbot.addWidget(view)
    view.resize(400, 300)
    for _ in range(20):
        view.zoom_in()
    assert view.transform.metres_per_pixel >= NavMapView.MIN_METRES_PER_PIXEL
    for _ in range(40):
        view.zoom_out()
    assert view.transform.metres_per_pixel <= NavMapView.MAX_METRES_PER_PIXEL
