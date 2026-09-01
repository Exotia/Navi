"""How fast a full stick deflection drives the rover.

The gamepad's top speed used to be a constant chosen for the first careful
hardware sessions - 0.05 m/s, a tenth of what the drive train takes. That
is the right number for parking against a marker and the wrong number for
crossing a yard, and nothing in the ground station could tell them apart.
This card makes it the operator's choice, in m/s, where they can see it.

It sets a cap, not a speed: the sticks still choose everything below it.
The turning rate rides along in the same proportion the two constants were
written in, so a faster rover is not left turning at the crawl rate that
was picked to match 0.05 m/s.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider

from ground_station import theme
from ground_station.gamepad_input import (MAX_LINEAR_SPEED,
                                          MIN_SETTABLE_LINEAR_SPEED,
                                          MAX_SETTABLE_LINEAR_SPEED)

#: The slider works in centimetres per second, so every step is a whole
#: number the operator can land on exactly. Qt sliders are integer-only and
#: a float scale would make 0.05 unreachable by dragging.
CM_PER_M = 100


def _to_cm(speed_m_s: float) -> int:
    return int(round(speed_m_s * CM_PER_M))


class SpeedCard(QWidget):
    """A slider from the cautious floor to the drive train's ceiling.

    `speed_changed` carries metres per second, the unit the rest of the
    system speaks; the widget converts for its own display only.
    """

    speed_changed = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("speedCard")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#speedCard {{ {theme.card_style()} }}")

        title = QLabel("DRIVE SPEED")
        title.setStyleSheet(theme.section_title_style())

        self.value_label = QLabel()
        self.value_label.setStyleSheet(
            f"color: {theme.TEXT}; border: none; background: transparent; "
            f"font-family: {theme.MONO_FONT_FAMILY}; "
            f"font-size: {theme.FONT_SIZE_TITLE}px; font-weight: 600;")

        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.value_label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(_to_cm(MIN_SETTABLE_LINEAR_SPEED))
        self.slider.setMaximum(_to_cm(MAX_SETTABLE_LINEAR_SPEED))
        self.slider.setValue(_to_cm(MAX_LINEAR_SPEED))
        self.slider.setPageStep(5)
        # Qt's stock slider is a blue that belongs to no other pixel in this
        # window. The filled part uses the accent, the rest the panel's own
        # border colour, and the handle is a grab-sized block rather than
        # the default sliver.
        self.slider.setStyleSheet(f"""
            QSlider {{ border: none; background: transparent; }}
            QSlider::groove:horizontal {{
                height: 4px; border-radius: 2px;
                background: {theme.BORDER};
            }}
            QSlider::sub-page:horizontal {{
                height: 4px; border-radius: 2px;
                background: {theme.ACCENT};
            }}
            QSlider::handle:horizontal {{
                width: 12px; margin: -6px 0; border-radius: 3px;
                background: {theme.TEXT};
            }}
            QSlider::handle:horizontal:hover {{ background: {theme.ACCENT}; }}
        """)
        self.slider.setToolTip(
            "Top speed at full stick deflection. The sticks still choose "
            "everything below it, and the turning rate scales with it.\n"
            "This is a ground-station cap on what is commanded - the rover's "
            "own limits still apply on top.")
        self.slider.valueChanged.connect(self._on_slider_moved)

        ends = QHBoxLayout()
        for text, align in ((f"{MIN_SETTABLE_LINEAR_SPEED:.2f}", Qt.AlignLeft),
                            (f"{MAX_SETTABLE_LINEAR_SPEED:.2f} m/s", Qt.AlignRight)):
            label = QLabel(text)
            label.setStyleSheet(
                f"color: {theme.TEXT_DIM}; border: none; background: transparent; "
                f"font-size: {theme.FONT_SIZE_SMALL}px;")
            if align == Qt.AlignRight:
                ends.addStretch()
            ends.addWidget(label)

        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.addLayout(header)
        layout.addWidget(self.slider)
        layout.addLayout(ends)

        self._refresh_label()

    @property
    def speed(self) -> float:
        """The current cap, in m/s."""
        return self.slider.value() / CM_PER_M

    def set_speed(self, speed_m_s: float) -> None:
        """Move the slider without pretending the operator did it. The
        value is snapped to the slider's own range first, and the signal
        fires only when the value actually CHANGES - Qt suppresses
        valueChanged for a no-op set, so a caller re-asserting the current
        speed produces no wire traffic, which is the behaviour a latched
        consumer wants anyway."""
        self.slider.setValue(_to_cm(speed_m_s))

    def _on_slider_moved(self, _value: int) -> None:
        self._refresh_label()
        self.speed_changed.emit(self.speed)

    def _refresh_label(self) -> None:
        # Two decimals: the slider steps in centimetres, and a third digit
        # would be a zero that changes when nothing moved.
        self.value_label.setText(f"{self.speed:.2f} m/s")
