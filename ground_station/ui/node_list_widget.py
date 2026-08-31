from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel

from ground_station import theme
from ground_station.models import NodeStatus


class NodeListWidget(QWidget):
    # Fixed width for the right-hand column: consistent whatever the video
    # panel and drive cards do with the rest of the window's width.
    FIXED_WIDTH = 240

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("nodeList")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#nodeList {{ {theme.card_style()} }}")
        self.setFixedWidth(self.FIXED_WIDTH)
        self._layout = QVBoxLayout(self)
        title = QLabel("SYSTEM NODES")
        title.setStyleSheet(theme.section_title_style())
        self._layout.addWidget(title)
        self._row_labels: list[QLabel] = []
        # Rows stack from the top under the title. Without this the layout
        # spread its few children down the full column height and left the
        # title floating in the middle of an empty card.
        self._layout.addStretch()

    def _insert_row(self, label: QLabel) -> None:
        """Above the trailing stretch, so rows stay packed under the title."""
        self._layout.insertWidget(self._layout.count() - 1, label)

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
            self._insert_row(label)
            self._row_labels.append(label)
