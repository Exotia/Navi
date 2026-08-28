from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout

from ground_station import theme
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.node_list_widget import NodeListWidget
from ground_station.ui.video_panel import VideoPanel


class DashboardPage(QWidget):
    drive_details_requested = Signal()

    def __init__(self, video_receiver=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {theme.BG}; color: {theme.TEXT};")
        self.drive_card = DriveCard()
        self.node_list = NodeListWidget()
        self.video_panel = VideoPanel(receiver=video_receiver)

        self.drive_card.details_requested.connect(self.drive_details_requested)

        # Video goes above the drive card in its own column, so the drive
        # readouts stay visible alongside the camera while driving.
        left = QVBoxLayout()
        left.addWidget(self.video_panel, stretch=3)
        left.addWidget(self.drive_card, stretch=1)

        layout = QHBoxLayout(self)
        layout.addLayout(left, stretch=3)
        layout.addWidget(self.node_list, stretch=1)
