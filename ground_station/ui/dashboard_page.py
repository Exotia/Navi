from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QHBoxLayout

from ground_station.ui.drive_card import DriveCard
from ground_station.ui.node_list_widget import NodeListWidget


class DashboardPage(QWidget):
    drive_details_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.drive_card = DriveCard()
        self.node_list = NodeListWidget()

        self.drive_card.details_requested.connect(self.drive_details_requested)

        layout = QHBoxLayout(self)
        layout.addWidget(self.drive_card, stretch=3)
        layout.addWidget(self.node_list, stretch=1)
