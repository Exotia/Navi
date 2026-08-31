from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                                QRadioButton, QButtonGroup)

from ground_station import theme
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.drive_row import DriveRow
from ground_station.ui.map_row import MapRow
from ground_station.ui.nav_row import NavRow
from ground_station.ui.node_list_widget import NodeListWidget
from ground_station.ui.video_panel import VideoPanel


class DashboardPage(QWidget):
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

        # Video goes above the drive card in its own column, so the drive
        # readouts stay visible alongside the camera while driving. An 8px
        # gap between cards, consistent with the card design across the
        # dashboard.
        left = QVBoxLayout()
        left.setSpacing(8)
        left.addLayout(mode_row)
        left.addWidget(self.video_panel, stretch=3)
        # The NAV row takes a share rather than its bare minimum: in the
        # autonomy view the plan map is what the operator watches, and at
        # no stretch it was squeezed to its 320 px floor under a camera
        # that had three times the room. Hidden widgets contribute no
        # stretch, so every other view still gives the camera everything.
        left.addWidget(self.nav_row, stretch=4)
        left.addWidget(self.map_row)
        left.addWidget(self.drive_row)
        left.addWidget(self.drive_card, stretch=1)

        layout = QHBoxLayout(self)
        layout.setSpacing(8)
        layout.addLayout(left, stretch=3)
        layout.addWidget(self.node_list)

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
