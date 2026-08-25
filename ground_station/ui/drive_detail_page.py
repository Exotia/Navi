from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPlainTextEdit

from ground_station import theme
from ground_station.models import DriveState


class DriveDetailPage(QWidget):
    back_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {theme.BG}; color: {theme.TEXT};")

        self.back_link = QLabel("← back to dashboard")
        self.back_link.setStyleSheet(f"color: {theme.ACCENT};")
        self.back_link.mousePressEvent = self._on_back_clicked

        header = QLabel("DRIVE / TWIST — DETAIL")
        header.setStyleSheet("font-weight: 600;")

        self.vx_label = QLabel("vx (cmd)  --")
        self.vy_label = QLabel("vy (cmd)  --")
        self.wz_label = QLabel("wz (cmd)  --")
        for label in (self.vx_label, self.vy_label, self.wz_label):
            label.setStyleSheet(f"font-family: {theme.MONO_FONT_FAMILY}; font-size: 16px;")

        self.raw_log = QPlainTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(200)
        self.raw_log.setStyleSheet(
            f"background-color: #0e1014; color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY};"
        )

        top_bar = QHBoxLayout()
        top_bar.addWidget(self.back_link)
        top_bar.addStretch()

        layout = QVBoxLayout(self)
        layout.addLayout(top_bar)
        layout.addWidget(header)
        layout.addWidget(self.vx_label)
        layout.addWidget(self.vy_label)
        layout.addWidget(self.wz_label)
        layout.addWidget(QLabel("RAW MESSAGES"))
        layout.addWidget(self.raw_log)

    def _on_back_clicked(self, event):
        self.back_requested.emit()

    def update_from(self, state: DriveState) -> None:
        if state.latest is None:
            return
        self.vx_label.setText(f"vx (cmd)  {state.latest.linear_x:.2f} m/s")
        self.vy_label.setText(f"vy (cmd)  {state.latest.linear_y:.2f} m/s")
        self.wz_label.setText(f"wz (cmd)  {state.latest.angular_z:.2f} rad/s")

    def append_raw_message(self, text: str) -> None:
        self.raw_log.appendPlainText(text)
