"""The TUNING drawer: the operator's front end for the six numbers that
decide what the rover will and will not drive over (see the
/autonomy/tuning and /autonomy/tuning_state wire contract in
ground_station/models.py).

A twin of ``site_card.py`` in shape - a hidden-by-default right-hand
drawer the dashboard shows on request - but its own job: show what the
rover currently reports, let the operator type a new value per row, and
send only the rows that actually changed when Apply is pressed.

This widget never touches ROS. It knows nothing about ``ros_client`` or
``rclpy`` - it only emits ``values_applied`` when the operator presses
Apply, and takes plain data (a dict from ``parse_tuning_state``) when told
what the rover reports. The window is the only thing that plumbs that
signal onto the wire and routes the state message back in.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QDoubleSpinBox, QHBoxLayout, QLabel,
                                QPushButton, QVBoxLayout, QWidget)

from ground_station import theme

# key, human name, unit, decimals, step, minimum, maximum, tooltip. The
# tooltip is where the two physical ceilings live (step_lethal_m past
# 0.282 m, wheel_trail_radius_m past 0.445 m) - named there rather than
# enforced as a spin-box maximum, because the operator standing next to
# the rock the rover refuses may have a reason to go past either.
_ROWS = (
    ("step_lethal_m", "Step", "m", 3, 0.01, 0.0, 1.0,
     "Height of a step the rover refuses to drive over. 0.282 m is the "
     "belly clearance and a hard physical ceiling - past it the rover "
     "high-centres."),
    ("slope_lethal_deg", "Slope", "deg", 1, 1.0, 0.0, 90.0,
     "Ground inclination the rover refuses. The chassis tips at about "
     "47 degrees."),
    ("floating_gap_m", "Floating gap", "m", 3, 0.01, 0.0, 2.0,
     "Cells hanging this far above their neighbours with nothing beneath "
     "are dropped as camera noise rather than real terrain."),
    ("wheel_trail_radius_m", "Wheel trail radius", "m", 3, 0.01, 0.0, 1.0,
     "Free disc stamped along ground the wheels have provably covered. "
     "Past 0.445 m - the footprint's inscribed circle - it clears ground "
     "the wheels never actually touched."),
    ("goal_heal_radius_m", "Goal heal radius", "m", 2, 0.1, 0.0, 5.0,
     "Disc around the goal forced free, so a goal buried in phantom "
     "terrain is still reachable."),
    ("startup_clear_radius_m", "Startup clear radius", "m", 2, 0.1, 0.0, 5.0,
     "Disc of unknown ground cleared at the rover's first pose."),
    ("climb_lethal_m", "Climb (from rover)", "m", 3, 0.01, 0.0, 1.0,
     "How far above the rover's OWN ground a nearby cell may sit and "
     "still be mountable - what catches a staircase of small steps that "
     "no single step gives away. The 0.282 m belly ceiling applies here "
     "exactly as it does to Step."),
    ("drop_lethal_m", "Drop (from rover)", "m", 3, 0.01, 0.0, 1.0,
     "How far BELOW the rover's ground a nearby cell may fall. Tighter "
     "than the climb on purpose: a wheel climbs a rock it would fall "
     "into."),
    ("rover_heal_radius_m", "Rover heal radius", "m", 2, 0.1, 0.0, 3.0,
     "Disc around the rover's CURRENT position forced free in the costmap, "
     "measured obstacles included - the rover standing there is the proof. "
     "Cost only; the height layers keep what was measured."),
    ("observation_decay_s", "Observation decay", "s", 0, 5.0, 0.0, 600.0,
     "Ground the camera has not confirmed for this long fades back to "
     "unknown - phantoms heal themselves, real obstacles survive because "
     "they keep being seen. 0 disables and the map never forgets."),
    ("relative_radius_m", "Rover-relative radius", "m", 2, 0.1, 0.0, 10.0,
     "How far around the rover the two limits above are judged. Keep it "
     "small: applied to the whole map, a gentle 5 degree yard rises past "
     "the climb limit within a few metres and the rover walls itself "
     "in. 0 disables the test."),
)

# Shown in a row's rover column until the rover has reported at all. Never
# a number - an invented default here would look like something the rover
# said.
_NO_ROVER_VALUE = "-"


class _TuningRow(QWidget):
    """One row: the parameter's name and unit, what the rover reports, and
    an editable field for a new value. Not exposed outside this module -
    ``TuningCard.rows`` is what tests and the window reach for."""

    def __init__(self, key: str, label_text: str, unit: str, decimals: int,
                step: float, minimum: float, maximum: float, tooltip: str,
                parent=None):
        super().__init__(parent)
        self.key = key
        self.decimals = decimals

        name_label = QLabel(f"{label_text} ({unit})")
        name_label.setStyleSheet(
            f"color: {theme.TEXT}; border: none; background: transparent;")
        name_label.setToolTip(tooltip)

        self.rover_label = QLabel(_NO_ROVER_VALUE)
        self.rover_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; border: none; background: transparent;")
        self.rover_label.setToolTip(tooltip)
        self.rover_label.setMinimumWidth(56)

        self.editor = QDoubleSpinBox()
        self.editor.setDecimals(decimals)
        self.editor.setSingleStep(step)
        self.editor.setRange(minimum, maximum)
        self.editor.setToolTip(tooltip)
        # Disabled until the rover has actually reported - an editable
        # field with no rover value behind it would invite a change built
        # on a number nobody sent.
        self.editor.setEnabled(False)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(name_label, stretch=1)
        layout.addWidget(self.rover_label)
        layout.addWidget(self.editor)

    def rover_text(self, value: float) -> str:
        return f"{value:.{self.decimals}f}"


class TuningCard(QWidget):
    """The TUNING drawer. See the module docstring for the contract."""

    values_applied = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("tuningCard")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#tuningCard {{ {theme.card_style()} }}")

        # None until /autonomy/tuning_state has arrived at least once - the
        # source of truth this card shows and reverts to, never a guess.
        self.rover_values = None

        title = QLabel("TUNING")
        title.setStyleSheet(theme.section_title_style())

        self.status_label = QLabel("waiting for the rover to report its tuning...")
        self.status_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; border: none; background: transparent;")
        self.status_label.setWordWrap(True)

        rover_col = QLabel("rover")
        rover_col.setStyleSheet(
            f"color: {theme.TEXT_DIM}; border: none; background: transparent;")
        new_col = QLabel("new")
        new_col.setStyleSheet(
            f"color: {theme.TEXT_DIM}; border: none; background: transparent;")
        header_row = QHBoxLayout()
        header_row.setSpacing(8)
        header_row.addStretch()
        header_row.addWidget(rover_col)
        header_row.addWidget(new_col)

        self.rows = {}
        rows_layout = QVBoxLayout()
        rows_layout.setSpacing(6)
        for key, label_text, unit, decimals, step, minimum, maximum, tooltip in _ROWS:
            row = _TuningRow(key, label_text, unit, decimals, step,
                            minimum, maximum, tooltip)
            self.rows[key] = row
            rows_layout.addWidget(row)

        self.apply_button = QPushButton("Apply")
        self.apply_button.setEnabled(False)
        self.apply_button.setToolTip(
            "Send every changed row to the rover. Rows left unchanged are "
            "not sent, so this never fights another operator editing the "
            "same values at the same time.")
        self.apply_button.clicked.connect(self._on_apply_clicked)

        self.revert_button = QPushButton("Revert")
        self.revert_button.setEnabled(False)
        self.revert_button.setToolTip(
            "Put every editor back to what the rover currently reports.")
        self.revert_button.clicked.connect(self._on_revert_clicked)

        buttons_row = QHBoxLayout()
        buttons_row.setSpacing(6)
        buttons_row.addWidget(self.apply_button)
        buttons_row.addWidget(self.revert_button)
        buttons_row.addStretch()

        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.addWidget(title)
        layout.addWidget(self.status_label)
        layout.addLayout(header_row)
        layout.addLayout(rows_layout)
        layout.addLayout(buttons_row)
        layout.addStretch()

    def set_tuning_state(self, values: dict | None) -> None:
        """The rover's own account of the tunable values, from
        /autonomy/tuning_state. ``None`` means the message either has not
        arrived yet or would not parse - both read the same to the
        operator: nothing trustworthy to show, so the card is left as it
        was rather than blanked.

        Only the very first arrival seeds the editors. After that, an
        editor is left exactly where the operator (or the last Apply) put
        it - only the rover column moves - so a later state message can be
        compared against what was actually requested, including the case
        where the rover rejected it and nothing changed.
        """
        if values is None:
            return
        first_report = self.rover_values is None
        self.rover_values = values

        self.status_label.setVisible(False)
        for key, row in self.rows.items():
            # A rover value past the editor's range would be silently
            # clamped on seeding, and the clamped number would then read as
            # an operator change on the next Apply - a phantom request the
            # operator never made. The rover's own value is the one thing
            # this card must never argue with, so the range yields to it.
            if values[key] > row.editor.maximum():
                row.editor.setMaximum(values[key])
            if values[key] < row.editor.minimum():
                row.editor.setMinimum(values[key])
            row.rover_label.setText(row.rover_text(values[key]))
            row.editor.setEnabled(True)
            if first_report:
                row.editor.setValue(values[key])

        self.apply_button.setEnabled(True)
        self.revert_button.setEnabled(True)

    def _on_apply_clicked(self) -> None:
        if self.rover_values is None:
            return
        changed = {}
        for key, row in self.rows.items():
            new_value = round(row.editor.value(), row.decimals)
            rover_value = round(self.rover_values[key], row.decimals)
            if new_value != rover_value:
                changed[key] = new_value
        if changed:
            self.values_applied.emit(changed)

    def _on_revert_clicked(self) -> None:
        if self.rover_values is None:
            return
        for key, row in self.rows.items():
            row.editor.setValue(self.rover_values[key])
