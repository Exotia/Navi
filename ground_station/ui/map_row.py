"""The Map row: which map is loaded, how big it is, save / load / clear.

Shown in semi-autonomous mode only. Everything here talks to the rover
through signals the window routes to RosBridgeClient - no ROS in this file.
"""

import re

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QInputDialog, QLabel, QMessageBox,
                               QPushButton, QWidget)

from ground_station import theme
from ground_station.models import MapState

NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,64}$')


class MapRow(QWidget):
    save_requested = Signal(str)
    load_requested = Signal(str)
    clear_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._state: MapState | None = None
        self._notice = ""
        self.ask_name = self._ask_name_dialog
        self.confirm_clear = self._confirm_clear_dialog

        self.map_combo = QComboBox()
        self.load_button = QPushButton("Load")
        self.save_button = QPushButton("Save as…")
        self.clear_button = QPushButton("Clear")
        self.status_label = QLabel()
        self.status_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY};")

        layout = QHBoxLayout(self)
        layout.addWidget(QLabel("MAP"))
        layout.addWidget(self.map_combo, stretch=1)
        layout.addWidget(self.load_button)
        layout.addWidget(self.save_button)
        layout.addWidget(self.clear_button)
        layout.addWidget(self.status_label, stretch=2)

        self.load_button.clicked.connect(self._on_load)
        self.save_button.clicked.connect(self._on_save)
        self.clear_button.clicked.connect(self._on_clear)
        self.set_state(None)

    def set_state(self, state) -> None:
        self._state = state
        enabled = state is not None
        if state is not None:
            current = self.map_combo.currentText()
            self.map_combo.blockSignals(True)
            self.map_combo.clear()
            self.map_combo.addItems(state.maps)
            if current in state.maps:
                self.map_combo.setCurrentText(current)
            self.map_combo.blockSignals(False)
        self.map_combo.setEnabled(enabled and bool(state.maps))
        self.load_button.setEnabled(enabled and bool(state.maps))
        self.save_button.setEnabled(enabled and state.cells_seen > 0)
        self.clear_button.setEnabled(enabled)
        self._refresh_status()

    def _refresh_status(self) -> None:
        if self._state is None:
            self.status_label.setText("MAP: no status")
            return
        s = self._state
        parts = [f"{s.cells_seen} cells, {s.extent_m[0]:.1f} x {s.extent_m[1]:.1f} m, {s.tiles} tiles"]
        if s.loaded:
            parts.append(f"loaded: {s.loaded}")
        if s.last_command:
            outcome = "ok" if s.last_command.get("ok") else f"FAILED: {s.last_command.get('error')}"
            parts.append(f"{s.last_command.get('action')} {s.last_command.get('name') or ''} {outcome}")
        if self._notice:
            parts.append(self._notice)
        self.status_label.setText(" | ".join(parts))

    def _on_load(self) -> None:
        name = self.map_combo.currentText()
        if name:
            self.load_requested.emit(name)

    def _on_save(self) -> None:
        name = self.ask_name()
        if name is None:
            return
        if not NAME_PATTERN.match(name):
            self._notice = f"save refused: {name!r} is not a valid name (A-Z a-z 0-9 _ -)"
        elif self._state is not None and name in self._state.maps:
            self._notice = f"save refused: {name!r} already exists"
        else:
            self._notice = ""
            self.save_requested.emit(name)
        self._refresh_status()

    def _on_clear(self) -> None:
        if self.confirm_clear():
            self.clear_requested.emit()

    def _ask_name_dialog(self):
        text, ok = QInputDialog.getText(self, "Save map", "Map name:")
        return text.strip() if ok else None

    def _confirm_clear_dialog(self) -> bool:
        answer = QMessageBox.question(self, "Clear map",
                                      "Discard the live map on the rover? Saved maps are kept.")
        return answer == QMessageBox.StandardButton.Yes
