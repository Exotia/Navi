from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                                QRadioButton, QButtonGroup)

from ground_station import theme
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.drive_row import DriveRow
from ground_station.ui.map_row import MapRow
from ground_station.ui.nav_row import NavRow
from ground_station.ui.node_list_widget import NodeListWidget
from ground_station.ui.site_card import SiteCard
from ground_station.ui.speed_card import SpeedCard
from ground_station.ui.tuning_card import TuningCard
from ground_station.ui.video_panel import VideoPanel


class DashboardPage(QWidget):
    #: The bottom card row - waypoint editor and wheels - is a fixed slice,
    #: so every pixel the window gains goes to the camera and the plan grid.
    BOTTOM_CARD_HEIGHT = 190

    drive_details_requested = Signal()
    # Emits "manual", "semi_auto", "autonomous" or "simulation". The switch
    # selects a view source and nothing else - the twist keeps reaching the
    # rover in every mode, so this is never a control-path change wearing a
    # view-change's clothes.
    mode_changed = Signal(str)

    def __init__(self, video_receiver=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {theme.BG}; color: {theme.TEXT};")
        self.drive_card = DriveCard()
        self.node_list = NodeListWidget()
        # Hidden until the header's Nodes button asks for it: a diagnostic
        # read a few times a session should not hold a fixed column of
        # width away from the map and the camera in every view.
        self.node_list.setVisible(False)
        self.site_card = SiteCard()
        # A second right-hand drawer, a twin of node_list: anchoring is a
        # once-per-mission job like reading the node list, and hidden it
        # costs the camera and the plan grid exactly nothing. The header
        # toggle that shows it is wired later (T9) - this page only
        # constructs it and keeps it out of the way until then.
        self.site_card.setVisible(False)
        self.tuning_card = TuningCard()
        # A third right-hand drawer, a twin of node_list and site_card: the
        # operator opens it only when a rock needs a number changed, and
        # hidden it costs the camera and the plan grid nothing. The header
        # toggle that shows it is wired in MainWindow.
        self.tuning_card.setVisible(False)
        self.video_panel = VideoPanel(receiver=video_receiver)
        self.map_row = MapRow()
        self.map_row.setVisible(False)
        self.nav_row = NavRow()
        self.nav_row.setVisible(False)
        self.drive_row = DriveRow()
        self.drive_row.setVisible(False)

        self.drive_card.details_requested.connect(self.drive_details_requested)

        # "VIEW", not "MODE". These radios choose which picture and which
        # controls are on screen; the rover's own mode is the header pill
        # and is changed only by the Manual/Autonomous buttons. Labelling
        # both "mode" is what left an operator pressing Autonomous here and
        # wondering why the rover would not take a goal.
        mode_label = QLabel("VIEW:")
        mode_label.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600;")
        mode_label.setToolTip(
            "Chooses what this window shows. The rover's mode is the pill "
            "in the header - change it with Manual or Autonomous.")
        self.manual_radio = QRadioButton("Camera")
        self.semi_auto_radio = QRadioButton("Semi-autonomous")
        self.autonomous_radio = QRadioButton("Autonomy")
        self.simulation_radio = QRadioButton("Simulation")
        # One list, in display order, rather than four scattered checks: the
        # emitted name and the button are declared together, so adding a
        # mode cannot leave a button that emits nothing.
        self._modes = [
            (self.manual_radio, "manual"),
            (self.semi_auto_radio, "semi_auto"),
            (self.autonomous_radio, "autonomous"),
            (self.simulation_radio, "simulation"),
        ]
        self.manual_radio.setToolTip("The rover's camera, full width.")
        self.semi_auto_radio.setToolTip(
            "The Gazebo mirror placed by the rover's own localisation, plus "
            "the map save/load row.")
        self.autonomous_radio.setToolTip(
            "Show the NAV row and the plan view. The rover's own mode "
            "changes only when you press Autonomous on that row.")
        self.simulation_radio.setToolTip(
            "The local simulation, dead-reckoned from the commanded twist.")

        self._mode_group = QButtonGroup(self)
        mode_row = QHBoxLayout()
        mode_row.setSpacing(12)
        mode_row.addWidget(mode_label)
        for radio, _ in self._modes:
            radio.setStyleSheet(f"color: {theme.TEXT};")
            self._mode_group.addButton(radio)
            radio.toggled.connect(self._on_mode_toggled)
            mode_row.addWidget(radio)
        mode_row.addStretch()
        self.manual_radio.setChecked(True)

        # The stage: the camera and the plan grid SIDE BY SIDE, sharing the
        # whole upper half. Stacked, each got half the height and the
        # operator watched the one that mattered through a letterbox; side
        # by side both are full height, and a hidden nav row gives the
        # camera the entire width in the other views.
        stage = QHBoxLayout()
        stage.setSpacing(8)
        stage.addWidget(self.video_panel, stretch=1)
        stage.addWidget(self.nav_row, stretch=1)

        # The bottom card carries the waypoint editor (built by the NAV row,
        # placed here) beside the wheels: list-editing is a before-the-run
        # job, so it belongs under the stage rather than eating into it.
        self.waypoint_panel = self.nav_row.editor_panel
        self.waypoint_panel.setVisible(False)
        # Capped, and deliberately: the point of moving the editor down here
        # was to give the stage the height, which a list that grows to fill
        # the window would hand straight back.
        self.waypoint_panel.setMaximumHeight(self.BOTTOM_CARD_HEIGHT)
        self.drive_card.setMaximumHeight(self.BOTTOM_CARD_HEIGHT)
        # A box, not a banner: with the waypoint editor hidden (every view
        # but autonomy) an unbounded card stretched the wheels across the
        # whole window with the diagram marooned in the middle.
        self.drive_card.setMaximumWidth(260)
        # Beside the wheels, because it is the other half of the same
        # question: the wheels say which way, this says how fast. Shown in
        # the two hand-driven views only (MainWindow._on_mode_changed) -
        # in autonomy the speed is Nav2's, not a slider's.
        self.speed_card = SpeedCard()
        self.speed_card.setMaximumHeight(self.BOTTOM_CARD_HEIGHT)
        self.speed_card.setMaximumWidth(320)
        bottom = QHBoxLayout()
        bottom.setSpacing(8)
        bottom.addWidget(self.waypoint_panel, stretch=1)
        bottom.addStretch()
        bottom.addWidget(self.speed_card)
        bottom.addWidget(self.drive_card)

        left = QVBoxLayout()
        left.setSpacing(8)
        left.addLayout(mode_row)
        left.addLayout(stage, stretch=1)
        left.addWidget(self.map_row)
        left.addWidget(self.drive_row)
        left.addLayout(bottom)

        layout = QHBoxLayout(self)
        layout.setSpacing(8)
        layout.addLayout(left, stretch=3)
        layout.addWidget(self.node_list)
        layout.addWidget(self.site_card)
        layout.addWidget(self.tuning_card)

    def _on_mode_toggled(self, checked: bool) -> None:
        if not checked:
            # Each radio's toggled fires twice on a switch (the one turning
            # off, then the one turning on) - only the "turning on" edge
            # names the mode we are entering.
            return
        for radio, mode in self._modes:
            if radio.isChecked():
                self.mode_changed.emit(mode)
                return
