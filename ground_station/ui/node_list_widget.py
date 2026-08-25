from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel

from ground_station import theme
from ground_station.models import NodeStatus


class NodeListWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"background-color: {theme.PANEL}; border: 1px solid {theme.BORDER}; border-radius: 6px;"
        )
        self._layout = QVBoxLayout(self)
        title = QLabel("SYSTEM NODES")
        title.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600; border: none;")
        self._layout.addWidget(title)
        self._row_labels: list[QLabel] = []

    def row_count(self) -> int:
        return len(self._row_labels)

    def row_text(self, index: int) -> str:
        return self._row_labels[index].text()

    def update_from(self, statuses: list[NodeStatus]) -> None:
        for label in self._row_labels:
            self._layout.removeWidget(label)
            label.deleteLater()
        self._row_labels = []

        for status in statuses:
            color = theme.OK if status.alive else theme.OFF
            state_word = "up" if status.alive else "down"
            label = QLabel(f"{status.name}  ({state_word})")
            label.setStyleSheet(f"color: {color}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            self._layout.addWidget(label)
            self._row_labels.append(label)
