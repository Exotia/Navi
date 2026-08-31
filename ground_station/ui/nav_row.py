"""The NAV row: the operator's waypoint list plus the Autonomous/Go/Pause/
Resume/Abort mission controls, and a status area fed from /nav_status. Built
to ui/drive_row.py's structure exactly - signals out, no ROS in here; the
window routes both the requests and the wire status. STOP is not duplicated
here: it is the window's header button, visible in every view. Abort is,
deliberately, the one control on this row with no confirmation in its
way."""

from time import monotonic

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QLineEdit, QListWidget,
                               QMessageBox, QPushButton, QVBoxLayout, QWidget)

from ground_station import theme
from ground_station.models import (AUTONOMOUS_MODE, NAV_ACTIVE_STATES,
                                   Waypoint, WaypointList,
                                   parse_waypoint_text)
from ground_station.ui.nav_map_view import NavMapView


def _plain(text) -> str:
    """Text as itself. The pills below are pinned to Qt::PlainText (see
    __init__), so a stray '<' from the rover's own error text is shown as a
    literal character, not parsed as markup."""
    return str(text)


class NavRow(QWidget):
    autonomous_requested = Signal()
    go_requested = Signal(list)
    pause_requested = Signal()
    resume_requested = Signal()
    abort_requested = Signal()
    waypoints_changed = Signal(list)

    def __init__(self, parent=None, clock=monotonic):
        super().__init__(parent)
        self._state = None
        self._mode_state = None
        self._clock = clock
        self.waypoints = WaypointList()
        self.confirm_autonomous = self._confirm_autonomous_dialog
        # No confirm_abort: Abort is a panic button. Handing the rover TO
        # autonomy is the decision worth a dialog; taking it back is not,
        # and a modal in the way of stopping a run is a hazard, not a
        # safeguard. (Operator request, night session.)

        self.map_view = NavMapView()
        self.map_view.point_clicked.connect(self.append_world_point)

        self.setObjectName("navRow")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#navRow {{ {theme.card_style()} }}")

        # --- waypoint editing ---------------------------------------------
        self.waypoint_list = QListWidget()
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("x (m)")
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("y (m)")
        self.yaw_input = QLineEdit()
        self.yaw_input.setPlaceholderText("yaw (rad, optional)")
        self.add_button = QPushButton("Add")
        self.remove_button = QPushButton("Remove")
        self.clear_button = QPushButton("Clear")
        self.up_button = QPushButton("↑")
        self.down_button = QPushButton("↓")

        # --- mission controls ----------------------------------------------
        self.autonomous_button = QPushButton("Autonomous")
        self.go_button = QPushButton("Go")
        self.pause_button = QPushButton("Pause")
        self.resume_button = QPushButton("Resume")
        self.abort_button = QPushButton("Abort")

        # Go starts a mission - filled ACCENT, the same visual weight the
        # header gives STOP, just the milder colour: this button starts
        # autonomy, it does not stop the rover.
        self.go_button.setStyleSheet(theme.go_button_style())
        self.go_button.setMinimumHeight(34)
        # Resume is now pressed at EVERY waypoint (the coordinator holds in
        # Waiting with movement disabled until it is), so it earns the same
        # fill as Go - but only while it is actually the next thing to do:
        # _refresh_controls swaps it back to a plain button otherwise, so a
        # lit Resume always means "the rover is waiting for you".
        self.resume_button.setMinimumHeight(34)
        # Abort stops the rover and tears down the coordinator's run, with
        # no dialog in the way - so it is filled BAD like STOP, at the same
        # height. A panic button has to look like one.
        self.abort_button.setStyleSheet(theme.stop_button_style())
        self.abort_button.setMinimumHeight(34)

        # --- status pills ----------------------------------------------
        self.no_status_label = QLabel("NAV: no status")
        self.no_status_label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")
        self.state_pill = QLabel()
        self.waypoint_pill = QLabel()
        self.error_pill = QLabel()
        self.error_pill.setStyleSheet(theme.pill_style(theme.BAD, "white"))
        self.progress_label = QLabel()
        self.progress_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
        self.hint_label = QLabel()
        self.hint_label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")

        # Every chip that ever carries rover-supplied text is pinned to
        # plain text - never auto-detected as rich text - so a stray '<'
        # from the rover is shown as itself instead of being parsed as
        # markup.
        for w in (self.state_pill, self.waypoint_pill, self.error_pill,
                  self.progress_label, self.hint_label):
            w.setTextFormat(Qt.TextFormat.PlainText)

        # --- layout ---------------------------------------------------
        # Three zones, in the order the operator uses them: the map (the
        # instrument you actually watch, so it gets the space), the waypoint
        # editor beside it (a list you touch before a run, not during), and
        # the mission bar underneath spanning both - the buttons you press,
        # on their own line where nothing else competes.
        title = QLabel("NAV")
        title.setStyleSheet(theme.section_title_style())

        self.map_view.setMinimumHeight(320)

        # The canvas has zoom and follow-the-rover logic that nothing on
        # screen could reach: an operator placing waypoints on a map they
        # cannot scale is working blind at whatever scale it booted with.
        self.zoom_in_button = QPushButton("+")
        self.zoom_out_button = QPushButton("−")
        self.follow_button = QPushButton("Follow rover")
        self.follow_button.setCheckable(True)
        self.follow_button.setChecked(True)
        self.zoom_in_button.setToolTip("Zoom in on the plan view.")
        self.zoom_out_button.setToolTip("Zoom out.")
        self.follow_button.setToolTip(
            "Keep the view centred on the rover. Switch off to hold a fixed "
            "patch of ground while the rover drives across it.")
        for b in (self.zoom_in_button, self.zoom_out_button):
            b.setFixedWidth(34)
        self.zoom_in_button.clicked.connect(self.map_view.zoom_in)
        self.zoom_out_button.clicked.connect(self.map_view.zoom_out)
        self.follow_button.toggled.connect(self.map_view.set_follow)

        map_tools = QHBoxLayout()
        map_tools.setSpacing(4)
        map_tools.addWidget(self.zoom_in_button)
        map_tools.addWidget(self.zoom_out_button)
        map_tools.addWidget(self.follow_button)
        map_tools.addStretch()

        editor = QVBoxLayout()
        editor.setSpacing(6)
        editor_title = QLabel("WAYPOINTS")
        editor_title.setStyleSheet(theme.section_title_style())
        editor.addWidget(editor_title)
        self.click_hint = QLabel("click the map to add")
        self.click_hint.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-size: {theme.FONT_SIZE_SMALL}px; "
            f"border: none;")
        editor.addWidget(self.click_hint)
        editor.addWidget(self.waypoint_list, stretch=1)

        # Coordinates on one line, the list verbs on the next: eight widgets
        # in a single row made every one of them too narrow to read.
        coords = QHBoxLayout()
        coords.setSpacing(4)
        for w in (self.x_input, self.y_input, self.yaw_input):
            w.setMinimumWidth(0)
            coords.addWidget(w)
        coords.addWidget(self.add_button)
        editor.addLayout(coords)

        verbs = QHBoxLayout()
        verbs.setSpacing(4)
        for w in (self.remove_button, self.clear_button, self.up_button,
                  self.down_button):
            verbs.addWidget(w)
        editor.addLayout(verbs)

        editor_panel = QWidget()
        editor_panel.setLayout(editor)
        editor_panel.setFixedWidth(300)

        map_column = QVBoxLayout()
        map_column.setSpacing(6)
        map_column.addWidget(self.map_view, stretch=1)
        map_column.addLayout(map_tools)

        upper = QHBoxLayout()
        upper.setSpacing(8)
        upper.addLayout(map_column, stretch=1)
        upper.addWidget(editor_panel)

        mission_layout = QHBoxLayout()
        mission_layout.setSpacing(8)
        mission_layout.addWidget(title)
        mission_layout.addWidget(self.autonomous_button)
        mission_layout.addWidget(self.go_button)
        mission_layout.addWidget(self.pause_button)
        mission_layout.addWidget(self.resume_button)
        mission_layout.addWidget(self.abort_button)
        mission_layout.addSpacing(12)
        # The status chips share the mission line: what the run is doing and
        # what you can do about it belong in one glance, not two rows apart.
        mission_layout.addWidget(self.no_status_label)
        for w in (self.state_pill, self.waypoint_pill, self.progress_label):
            mission_layout.addWidget(w)
        mission_layout.addStretch()

        # The reason line gets its own full-width row: run-ending reasons
        # ("Nav2 goal ended with status 6", "waypoint 2/7 reached - resume to
        # continue") are sentences, and squeezing them into the button row
        # truncated exactly the text that explains the failure.
        reason_layout = QHBoxLayout()
        reason_layout.addWidget(self.error_pill)
        reason_layout.addWidget(self.hint_label)
        reason_layout.addStretch()

        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.addLayout(upper, stretch=1)
        layout.addLayout(mission_layout)
        layout.addLayout(reason_layout)

        self.autonomous_button.setToolTip(
            "Hand the rover to autonomy. Nav2 will drive it. STOP stays live.")
        self.go_button.setToolTip("Send the waypoint list and start the run.")
        self.pause_button.setToolTip("Pause the active run.")
        self.resume_button.setToolTip("Resume a paused run.")
        self.abort_button.setToolTip(
            "Abort the run immediately - no confirmation. The rover stops "
            "and the coordinator's task is torn down.")

        self.add_button.clicked.connect(self._on_add)
        self.remove_button.clicked.connect(self._on_remove)
        self.clear_button.clicked.connect(self._on_clear)
        self.up_button.clicked.connect(self._on_up)
        self.down_button.clicked.connect(self._on_down)
        self.autonomous_button.clicked.connect(self._on_autonomous)
        self.go_button.clicked.connect(self._on_go)
        self.pause_button.clicked.connect(self.pause_requested.emit)
        self.resume_button.clicked.connect(self.resume_requested.emit)
        self.abort_button.clicked.connect(self._on_abort)
        self.waypoint_list.currentRowChanged.connect(lambda _row: self._refresh_controls())

        self.refresh_waypoints()
        self.set_state(None)
        self.set_mode_state(None)

    # --- waypoint editing -------------------------------------------------

    def append_world_point(self, x: float, y: float) -> None:
        """The map view's entry point: a click on the canvas appends a
        waypoint exactly as typing coordinates and pressing Add would."""
        self.waypoints.add(Waypoint(x, y))
        self.refresh_waypoints()

    def refresh_waypoints(self) -> None:
        selected = self.waypoint_list.currentRow()
        self.waypoint_list.clear()
        for i, w in enumerate(self.waypoints.items):
            text = f"{i + 1}. x {w.x:.2f}  y {w.y:.2f}"
            if w.yaw is not None:
                text += f"  yaw {w.yaw:.2f}"
            self.waypoint_list.addItem(text)
        if 0 <= selected < self.waypoint_list.count():
            self.waypoint_list.setCurrentRow(selected)
        self.map_view.set_waypoints(self.waypoints.items)
        self.waypoints_changed.emit(self.waypoints.items)
        self._refresh_controls()

    def _on_add(self):
        wp, reason = parse_waypoint_text(
            self.x_input.text(), self.y_input.text(), self.yaw_input.text())
        if wp is None:
            self.hint_label.setText(_plain(reason))
            return
        self.waypoints.add(wp)
        self.x_input.clear()
        self.y_input.clear()
        self.yaw_input.clear()
        self.refresh_waypoints()

    def _on_remove(self):
        row = self.waypoint_list.currentRow()
        if row < 0:
            return
        self.waypoints.remove(row)
        self.refresh_waypoints()

    def _on_clear(self):
        self.waypoints.clear()
        self.refresh_waypoints()

    def _on_up(self):
        row = self.waypoint_list.currentRow()
        if row <= 0:
            return
        self.waypoints.move_up(row)
        self.refresh_waypoints()
        self.waypoint_list.setCurrentRow(row - 1)

    def _on_down(self):
        row = self.waypoint_list.currentRow()
        if row < 0 or row >= len(self.waypoints) - 1:
            return
        self.waypoints.move_down(row)
        self.refresh_waypoints()
        self.waypoint_list.setCurrentRow(row + 1)

    # --- mission controls ---------------------------------------------

    def _on_autonomous(self):
        if self.confirm_autonomous():
            self.autonomous_requested.emit()

    def _on_go(self):
        self.go_requested.emit(self.waypoints.items)

    def _on_abort(self):
        self.abort_requested.emit()

    # --- state -----------------------------------------------------------

    def set_mode_state(self, state) -> None:
        self._mode_state = state
        self._refresh_controls()
        self._refresh_status()

    def set_state(self, state) -> None:
        self._state = state
        self._refresh_controls()
        self._refresh_status()

    def set_pose(self, pose) -> None:
        """Pass-through to the canvas: the window owns the wire layer, this
        row owns the widget that draws it."""
        self.map_view.set_pose(pose)

    def set_path_summary(self, summary) -> None:
        self.map_view.set_path_summary(summary)

    def refresh(self, now=None) -> None:
        self._refresh_controls()
        self._refresh_status(now)

    def _refresh_controls(self) -> None:
        is_autonomous = (self._mode_state is not None
                         and self._mode_state.mode == AUTONOMOUS_MODE)
        run_state = "" if self._state is None else self._state.state
        active = run_state in NAV_ACTIVE_STATES
        has_waypoints = len(self.waypoints) > 0

        self.go_button.setEnabled(
            is_autonomous and has_waypoints and self._state is not None and not active)
        self.pause_button.setEnabled(run_state == "running")
        waiting_for_resume = run_state == "paused"
        self.resume_button.setEnabled(waiting_for_resume)
        # Lit only while the rover is genuinely waiting on it - see the
        # comment where Resume is created.
        self.resume_button.setStyleSheet(
            theme.go_button_style() if waiting_for_resume else "")
        self.abort_button.setEnabled(active)
        self.autonomous_button.setEnabled(
            self._mode_state is not None and not is_autonomous)

        # Editing a list needs no rover at all.
        self.add_button.setEnabled(True)
        self.clear_button.setEnabled(True)
        has_selection = self.waypoint_list.currentRow() >= 0
        self.remove_button.setEnabled(has_selection)
        self.up_button.setEnabled(has_selection)
        self.down_button.setEnabled(has_selection)

        if self._state is None:
            hint = "no nav status from the rover"
        elif not is_autonomous:
            mode_name = self._mode_state.mode if self._mode_state is not None else "unknown"
            hint = f"press Autonomous first (rover is in {mode_name})"
        elif not has_waypoints:
            hint = "add a waypoint"
        elif active:
            hint = ""
        else:
            hint = ""
        self.hint_label.setText(_plain(hint))

    def _refresh_status(self, now=None) -> None:
        s = self._state
        if s is None:
            self.no_status_label.setVisible(True)
            for w in (self.state_pill, self.waypoint_pill, self.error_pill,
                      self.progress_label):
                w.setVisible(False)
            return
        self.no_status_label.setVisible(False)

        # state_pill: the rover is the authority on its own states - a
        # state this build does not know is still shown, verbatim, rather
        # than guessed at.
        if s.state == "running":
            bg, fg = theme.OK, "#0c1a0e"
        elif s.state in ("starting", "paused"):
            bg, fg = theme.ACCENT, "#2a1600"
        elif s.state in ("refused", "aborted"):
            bg, fg = theme.BAD, "white"
        elif s.state == "succeeded":
            bg, fg = theme.OK, "#0c1a0e"
        elif s.state == "idle":
            bg, fg = theme.OFF, theme.TEXT
        else:
            bg, fg = theme.OFF, theme.TEXT
        self.state_pill.setText(_plain(s.state.upper()))
        self.state_pill.setStyleSheet(theme.pill_style(bg, fg))
        self.state_pill.setVisible(True)

        if s.waypoint_index is not None:
            self.waypoint_pill.setText(
                _plain(f"WP {s.waypoint_index + 1}/{s.waypoint_count}"))
            self.waypoint_pill.setVisible(True)
        else:
            self.waypoint_pill.setVisible(False)

        parts = []
        if s.distance_remaining_m is not None:
            parts.append(f"{s.distance_remaining_m:.1f} m left")
        if s.eta_s is not None:
            minutes = int(s.eta_s // 60)
            seconds = int(s.eta_s % 60)
            parts.append(f"ETA {minutes}:{seconds:02d}")
        if parts:
            self.progress_label.setText(_plain("  ".join(parts)))
            self.progress_label.setVisible(True)
        else:
            self.progress_label.setVisible(False)

        if s.error:
            self.error_pill.setText(_plain(s.error))
            self.error_pill.setVisible(True)
        else:
            self.error_pill.setVisible(False)

    def _confirm_autonomous_dialog(self) -> bool:
        answer = QMessageBox.question(
            self, "Autonomous",
            "Hand the rover to autonomy? Nav2 will drive it. STOP stays live.")
        return answer == QMessageBox.StandardButton.Yes

