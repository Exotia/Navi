"""A mission clock. Start, stop, reset - the ERC runs are timed, and until
now the operator timed them on a phone.

Time is passed in rather than read (the same fake-clock idiom the rest of
this code base uses), so the whole thing is testable without waiting.
"""

from time import monotonic

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget

from ground_station import theme


def format_elapsed(seconds: float) -> str:
    """mm:ss up to an hour, then h:mm:ss. Never negative."""
    total = int(max(0.0, seconds))
    minutes, secs = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


class MissionTimer(QWidget):
    """Elapsed-time readout with one button. `tick()` refreshes the label;
    the widget owns a QTimer that calls it, which tests bypass."""

    def __init__(self, parent=None, clock=monotonic, tick_ms: int = 250):
        super().__init__(parent)
        self._clock = clock
        self._running = False
        # Seconds banked from previous runs, plus the start of the current
        # one: stopping and starting again continues the mission rather
        # than silently restarting it. Reset is the only way back to zero.
        self._banked = 0.0
        self._started_at = None

        self.time_label = QLabel(format_elapsed(0.0))
        self.time_label.setTextFormat(Qt.TextFormat.PlainText)
        self.time_label.setStyleSheet(
            f"color: {theme.TEXT}; font-family: {theme.MONO_FONT_FAMILY}; "
            f"font-size: {theme.FONT_SIZE_TITLE}px; font-weight: 600; "
            f"border: none; background: transparent;")
        self.start_button = QPushButton("Start")
        self.start_button.setFixedWidth(64)
        self.start_button.setToolTip("Start or stop the mission clock.")
        self.reset_button = QPushButton("Reset")
        self.reset_button.setFixedWidth(64)
        self.reset_button.setToolTip("Back to 00:00. Only available when stopped.")

        self.start_button.clicked.connect(self.toggle)
        self.reset_button.clicked.connect(self.reset)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        # No "MISSION" caption: a mm:ss readout beside Start/Reset needs no
        # label, and the header has rover state to fit. The word lives in
        # the tooltip instead.
        self.time_label.setToolTip("Mission elapsed time.")
        layout.addWidget(self.time_label)
        layout.addWidget(self.start_button)
        layout.addWidget(self.reset_button)

        self._timer = QTimer(self)
        self._timer.setInterval(tick_ms)
        self._timer.timeout.connect(self.tick)
        self._timer.start()
        self._refresh()

    # --- state ------------------------------------------------------------
    @property
    def running(self) -> bool:
        return self._running

    def elapsed(self) -> float:
        if self._running and self._started_at is not None:
            return self._banked + (self._clock() - self._started_at)
        return self._banked

    def start(self) -> None:
        if self._running:
            return
        self._started_at = self._clock()
        self._running = True
        self._refresh()

    def stop(self) -> None:
        if not self._running:
            return
        self._banked = self.elapsed()
        self._started_at = None
        self._running = False
        self._refresh()

    def toggle(self) -> None:
        self.stop() if self._running else self.start()

    def reset(self) -> None:
        """Zero, and stop. Deliberately allowed only while stopped - one
        click must not be able to throw away a run that is being timed."""
        if self._running:
            return
        self._banked = 0.0
        self._started_at = None
        self._refresh()

    def tick(self) -> None:
        if self._running:
            self._refresh()

    def _refresh(self) -> None:
        self.time_label.setText(format_elapsed(self.elapsed()))
        self.start_button.setText("Stop" if self._running else "Start")
        self.reset_button.setEnabled(not self._running and self.elapsed() > 0.0)
        self.time_label.setStyleSheet(
            f"color: {theme.ACCENT if self._running else theme.TEXT}; "
            f"font-family: {theme.MONO_FONT_FAMILY}; "
            f"font-size: {theme.FONT_SIZE_TITLE}px; font-weight: 600; "
            f"border: none; background: transparent;")
