"""The NAV row: the operator's waypoint list plus the Autonomous/Go/Pause/
Resume/Abort mission controls, and a status area fed from /nav_status. Built
to ui/drive_row.py's structure exactly - signals out, no ROS in here; the
window routes both the requests and the wire status. STOP is not duplicated
here: it lives on the DRIVE row, which is visible in every mode."""

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
        self.confirm_abort = self._confirm_abort_dialog

        self.map_view = NavMapView()
        self.map_view.setMinimumHeight(220)
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

        # Go starts a mission - filled ACCENT, the same visual weight
        # drive_row.py gives STOP (a full-fill button), just the milder
        # colour: this button starts autonomy, it does not stop the rover.
        self.go_button.setStyleSheet(
            f"QPushButton {{ background-color: {theme.ACCENT}; color: #2a1600; "
            f"font-weight: 700; border: none; border-radius: 6px; "
            f"padding: 6px 14px; }} "
            f"QPushButton:hover {{ background-color: #f0965c; }} "
            f"QPushButton:pressed {{ background-color: #c86a2e; }} "
            f"QPushButton:disabled {{ background-color: {theme.PANEL}; "
            f"color: {theme.TEXT_DIM}; }}"
        )
        # Abort moves the rover to a stop and tears down the coordinator's
        # run - a danger hint, the same border-only weight drive_row.py
        # gives Init/Reset encoders, not a full fill: the button stays
        # readable at a glance, just bordered BAD.
        self.abort_button.setStyleSheet(
            f"QPushButton {{ border: 1px solid {theme.BAD}; }}"
        )

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
        layout = QVBoxLayout(self)
        title = QLabel("NAV")
        title.setStyleSheet(theme.section_title_style())

        entry_layout = QHBoxLayout()
        entry_layout.addWidget(title)
        for w in (self.x_input, self.y_input, self.yaw_input, self.add_button,
                  self.remove_button, self.clear_button, self.up_button,
                  self.down_button):
            entry_layout.addWidget(w)

        mission_layout = QHBoxLayout()
        for b in (self.autonomous_button, self.go_button, self.pause_button,
                  self.resume_button, self.abort_button):
            mission_layout.addWidget(b)
        mission_layout.addStretch()

        status_layout = QHBoxLayout()
        status_layout.addWidget(self.no_status_label)
        for w in (self.state_pill, self.waypoint_pill, self.progress_label,
                  self.error_pill, self.hint_label):
            status_layout.addWidget(w)
        status_layout.addStretch()

        layout.addLayout(entry_layout)
        layout.addWidget(self.waypoint_list)
        layout.addWidget(self.map_view)
        layout.addLayout(mission_layout)
        layout.addLayout(status_layout)

        self.autonomous_button.setToolTip(
            "Hand the rover to autonomy. Nav2 will drive it. STOP stays live.")
        self.go_button.setToolTip("Send the waypoint list and start the run.")
        self.pause_button.setToolTip("Pause the active run.")
        self.resume_button.setToolTip("Resume a paused run.")
        self.abort_button.setToolTip(
            "Abort the run. The rover stops and the coordinator is aborted.")

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
        if self.confirm_abort():
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
        self.resume_button.setEnabled(run_state == "paused")
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

    def _confirm_abort_dialog(self) -> bool:
        answer = QMessageBox.question(
            self, "Abort",
            "Abort the run? The rover stops and the coordinator is aborted. "
            "The mode stays autonomous — use Manual on the DRIVE row to "
            "take back the sticks.")
        return answer == QMessageBox.StandardButton.Yes
