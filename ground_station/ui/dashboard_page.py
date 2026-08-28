from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                                QRadioButton, QButtonGroup)

from ground_station import theme
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.node_list_widget import NodeListWidget
from ground_station.ui.video_panel import VideoPanel


class DashboardPage(QWidget):
    drive_details_requested = Signal()
    # Emits "manual" or "semi_auto". The switch selects a view source and
    # nothing else - the twist keeps reaching the rover in both modes, so
    # this is never a control-path change wearing a view-change's clothes.
    mode_changed = Signal(str)

    def __init__(self, video_receiver=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {theme.BG}; color: {theme.TEXT};")
        self.drive_card = DriveCard()
        self.node_list = NodeListWidget()
        self.video_panel = VideoPanel(receiver=video_receiver)

        self.drive_card.details_requested.connect(self.drive_details_requested)

        mode_label = QLabel("MODE:")
        mode_label.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600;")
        self.manual_radio = QRadioButton("Manual")
        self.semi_auto_radio = QRadioButton("Semi-autonomous")
        for radio in (self.manual_radio, self.semi_auto_radio):
            radio.setStyleSheet(f"color: {theme.TEXT};")
        self.manual_radio.setChecked(True)
        self._mode_group = QButtonGroup(self)
        self._mode_group.addButton(self.manual_radio)
        self._mode_group.addButton(self.semi_auto_radio)
        self.manual_radio.toggled.connect(self._on_mode_toggled)
        self.semi_auto_radio.toggled.connect(self._on_mode_toggled)

        mode_row = QHBoxLayout()
        mode_row.addWidget(mode_label)
        mode_row.addWidget(self.manual_radio)
        mode_row.addWidget(self.semi_auto_radio)
        mode_row.addStretch()

        # Video goes above the drive card in its own column, so the drive
        # readouts stay visible alongside the camera while driving.
        left = QVBoxLayout()
        left.addLayout(mode_row)
        left.addWidget(self.video_panel, stretch=3)
        left.addWidget(self.drive_card, stretch=1)

        layout = QHBoxLayout(self)
        layout.addLayout(left, stretch=3)
        layout.addWidget(self.node_list, stretch=1)

    def _on_mode_toggled(self, checked: bool) -> None:
        if not checked:
            # Each radio's toggled fires twice on a switch (the one turning
            # off, then the one turning on) - only the "turning on" edge
            # names the mode we are entering.
            return
        self.mode_changed.emit("semi_auto" if self.semi_auto_radio.isChecked() else "manual")
