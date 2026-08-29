"""The Map row: which map is loaded, how big it is, save / load / clear.

Shown in semi-autonomous mode only. Everything here talks to the rover
through signals the window routes to RosBridgeClient - no ROS in this file.
"""

import html
import re
from time import monotonic

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QInputDialog, QLabel, QMessageBox,
                               QPushButton, QWidget)

from ground_station import theme
from ground_station.models import MapState

NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,64}$')

# How long the outcome of the last map command stays on the line after it
# changes. The rover repeats `last_command` in every status message for as
# long as it stands, so without this the line would still be reporting a
# save from twenty minutes ago as if it had just happened - and an operator
# who then presses Save and looks at the line has no way to tell the old
# answer from the new one. Ten seconds is long enough to read and short
# enough that anything still showing is about what just happened.
OUTCOME_SECONDS = 10.0


def _plain(text: str) -> str:
    """Text as itself inside the rich-text status line.

    Map names are restricted to A-Z a-z 0-9 _ - but an error string comes
    from the rover verbatim and could contain anything; unescaped, a stray
    '<' would swallow the rest of the line.
    """
    return html.escape(str(text))


class MapRow(QWidget):
    save_requested = Signal(str)
    load_requested = Signal(str)
    clear_requested = Signal()

    def __init__(self, parent=None, clock=monotonic):
        super().__init__(parent)
        self._state: MapState | None = None
        self._notice = ""
        # Injectable so the tests can age an outcome without sleeping, and
        # so the window can hand in the same reading its staleness timer
        # already took.
        self._clock = clock
        self._last_command_at: float | None = None
        self._seen_command = None
        self.ask_name = self._ask_name_dialog
        self.confirm_clear = self._confirm_clear_dialog

        self.map_combo = QComboBox()
        self.load_button = QPushButton("Load")
        self.save_button = QPushButton("Save as…")
        self.clear_button = QPushButton("Clear")
        self.status_label = QLabel()
        # Rich text so a failed outcome can be red while the rest of the
        # line stays chrome-coloured; every value that goes in is escaped.
        self.status_label.setTextFormat(Qt.TextFormat.RichText)
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
        # Compared against the last command *seen*, not the last state: a
        # status drop (state None) must not make the same old outcome look
        # new - and start its 10 s red window again - when status resumes.
        if state is not None and state.last_command != self._seen_command:
            self._seen_command = state.last_command
            self._notice = ""
            self._last_command_at = self._clock() if state.last_command else None
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

    def refresh(self, now: float | None = None) -> None:
        """Re-renders the line, ageing the last-command outcome off it.

        Called by the window's staleness timer: the outcome has to
        disappear ten seconds after it changed even when no new status
        arrives to redraw the line.
        """
        self._refresh_status(now)

    def _refresh_status(self, now: float | None = None) -> None:
        if self._state is None:
            self.status_label.setText("MAP: no status")
            return
        s = self._state
        parts = [_plain(f"{s.cells_seen} cells, {s.extent_m[0]:.1f} x "
                        f"{s.extent_m[1]:.1f} m, {s.tiles} tiles")]
        if s.loaded:
            parts.append(_plain(f"loaded: {s.loaded}"))
        if s.last_command and self._outcome_is_current(now):
            failed = not s.last_command.get("ok")
            outcome = f"FAILED: {s.last_command.get('error')}" if failed else "ok"
            text = _plain(f"{s.last_command.get('action')} "
                          f"{s.last_command.get('name') or ''} {outcome}")
            parts.append(f'<span style="color: {theme.BAD};">{text}</span>' if failed
                         else text)
        if self._notice:
            parts.append(_plain(self._notice))
        self.status_label.setText(" | ".join(parts))

    def _outcome_is_current(self, now: float | None) -> bool:
        if self._last_command_at is None:
            return False
        now = self._clock() if now is None else now
        return now - self._last_command_at <= OUTCOME_SECONDS

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
