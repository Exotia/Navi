"""The SITE drawer: the operator's front end for anchoring the judges' grid
onto the rover's own map (see docs/superpowers/plans/2026-09-01-site-anchor.md).

A twin of ``node_list_widget.py`` in shape - a hidden-by-default right-hand
drawer the dashboard shows on request - but its own job: load the published
landmark table, collect a map-frame measurement for two or more of its
entries (by hand-typed value, a depth probe, or the ArUco accumulator),
solve the rigid site->map fit, and hand a locked ``SiteTransform`` back to
whoever asked (``MainWindow``, wired in a later task).

This widget never touches ROS. It knows nothing about ``ros_client`` or
``rclpy`` - it only emits signals when the operator does something, and
takes plain data (a ``LandmarkTable``, a ``SightingsReport``, a
``ProbeResult``) when told about the world. The window is the only thing
that plumbs those signals to the wire and back.

The residual/solve arithmetic itself lives in ``site_frame.py``; this module
only decides *which* measurements go into a solve (ticked, in the table,
and actually measured) and renders what came back.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QHBoxLayout,
                                QLabel, QListWidget, QListWidgetItem,
                                QPushButton, QVBoxLayout, QWidget)

from ground_station import theme
from ground_station.site_frame import (LandmarkPair, SiteFrameError,
                                       residuals, solve_site_to_map)


def _plain(text) -> str:
    """Text as itself - the pills and labels below are pinned to
    Qt::PlainText, so a wire string that happens to look like markup (an
    operator's note, a rover-supplied error) is shown as a literal
    character, never parsed."""
    return str(text)


def _fg_for(bg: str) -> str:
    """A legible foreground for a pill of this background, expressed only
    in terms of theme constants (plus the two named CSS colours "white"
    and "black") - never a literal hex value, so this file cannot drift out
    of step with a theme palette change."""
    if bg == theme.BAD:
        return "white"
    if bg == theme.OK:
        return theme.BG
    return theme.TEXT


class _LandmarkRow(QWidget):
    """One checkable row in ``landmark_list``: a tick box the operator
    unticks to leave a landmark out of the fit, and a label carrying its
    id, its published site position, its note, and - once measured - its
    quality and residual. The label is explicit Qt::PlainText, matching the
    rule that holds everywhere else in this UI: a wire-sourced string is
    never auto-detected as rich text."""

    def __init__(self, landmark_id: str, parent=None):
        super().__init__(parent)
        self.id = landmark_id
        self.checkbox = QCheckBox()
        self.label = QLabel()
        self.label.setTextFormat(Qt.TextFormat.PlainText)
        self.label.setStyleSheet(f"color: {theme.TEXT}; border: none;")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(8)
        layout.addWidget(self.checkbox)
        layout.addWidget(self.label, stretch=1)


class _Measurement:
    """A map-frame measurement of one landmark, from whichever source
    produced it (hand entry, a probe, or the ArUco accumulator). Not
    exposed outside this module - the row text and the fit are the only
    things that read it."""

    __slots__ = ("x", "y", "quality", "residual_m")

    def __init__(self, x: float, y: float, quality: str):
        self.x = x
        self.y = y
        self.quality = quality
        self.residual_m = None


class SiteCard(QWidget):
    """The SITE drawer. See the module docstring and
    docs/superpowers/plans/2026-09-01-site-anchor.md §3.7 for the contract."""

    table_load_requested = Signal(str)
    probe_requested = Signal(str, str)
    anchor_start_requested = Signal()
    anchor_stop_requested = Signal()
    anchor_reset_requested = Signal()
    solve_requested = Signal()
    lock_changed = Signal(object)
    # The wrapper was relaunched with the rover unmoved since Lock - see
    # §3.10. Only enabled while locked; the window (T9) is what actually
    # re-expresses the transform and hands the result back to this card.
    camera_restarted = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("siteCard")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#siteCard {{ {theme.card_style()} }}")

        self.table = None
        self.transform = None
        self.locked = False
        self._measurements = {}   # id -> _Measurement
        self._rows = {}           # id -> _LandmarkRow

        title = QLabel("SITE")
        title.setStyleSheet(theme.section_title_style())

        self.table_label = QLabel("no table loaded")
        self.table_label.setTextFormat(Qt.TextFormat.PlainText)

        self.load_button = QPushButton("Load table…")
        self.load_button.setToolTip(
            "Load the published landmark table (JSON) for this site.")
        self.load_button.clicked.connect(self._on_load_clicked)

        self.landmark_list = QListWidget()
        self.landmark_list.currentRowChanged.connect(lambda _r: self._refresh_controls())

        target_label = QLabel("target:")
        target_label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")

        self.target_combo = QComboBox()
        self.target_combo.addItem("pole", "pole")
        self.target_combo.addItem("box face", "box_face")
        self.target_combo.setToolTip(
            "What was clicked: the pole itself (no correction) or the "
            "flat marker face (the 0.125 m offset is added).")

        self.probe_button = QPushButton("Probe")
        self.probe_button.setToolTip(
            "Ask the rover for a depth-probe measurement of the selected "
            "landmark, at the point the operator clicks on the camera.")
        self.probe_button.clicked.connect(self._on_probe_clicked)

        self.anchor_button = QPushButton("Anchor")
        self.anchor_button.setCheckable(True)
        self.anchor_button.setToolTip(
            "Start or stop the ArUco anchor accumulator. Commands no "
            "motion - turn the rover with the gamepad if fewer than two "
            "landmarks are visible.")
        self.anchor_button.toggled.connect(self._on_anchor_toggled)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setToolTip("Clear everything the anchor phase has accumulated.")
        self.reset_button.clicked.connect(self.anchor_reset_requested.emit)

        self.solve_button = QPushButton("Solve")
        self.solve_button.setToolTip(
            "Fit the site->map transform from the ticked, measured landmarks.")
        self.solve_button.clicked.connect(self._on_solve_clicked)

        self.lock_button = QPushButton("Lock")
        self.lock_button.setCheckable(True)
        self.lock_button.setToolTip(
            "Lock the solved transform. Waypoints typed in the judges' "
            "grid convert to map coordinates from this moment on.")
        self.lock_button.toggled.connect(self._on_lock_toggled)

        self.camera_restart_button = QPushButton("Camera restarted")
        self.camera_restart_button.setToolTip(
            "Press only if the ZED wrapper was just relaunched with the "
            "rover NOT moved since Lock. Re-expresses the locked transform "
            "in the new map frame.")
        self.camera_restart_button.clicked.connect(self.camera_restarted.emit)

        self.state_pill = QLabel("NO TABLE")
        self.state_pill.setTextFormat(Qt.TextFormat.PlainText)
        self.state_pill.setStyleSheet(theme.pill_style(theme.OFF, _fg_for(theme.OFF)))

        self.rms_pill = QLabel("")
        self.rms_pill.setTextFormat(Qt.TextFormat.PlainText)
        self.rms_pill.setStyleSheet(theme.pill_style(theme.OFF, _fg_for(theme.OFF)))

        self.detail_label = QLabel("")
        self.detail_label.setTextFormat(Qt.TextFormat.PlainText)
        self.detail_label.setWordWrap(True)
        self.detail_label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")

        header = QHBoxLayout()
        header.setSpacing(8)
        header.addWidget(title)
        header.addWidget(self.state_pill)
        header.addStretch()

        top = QHBoxLayout()
        top.setSpacing(8)
        top.addWidget(self.table_label, stretch=1)
        top.addWidget(self.load_button)

        target_row = QHBoxLayout()
        target_row.setSpacing(6)
        target_row.addWidget(target_label)
        target_row.addWidget(self.target_combo)
        target_row.addWidget(self.probe_button)
        target_row.addStretch()

        anchor_row = QHBoxLayout()
        anchor_row.setSpacing(6)
        anchor_row.addWidget(self.anchor_button)
        anchor_row.addWidget(self.reset_button)
        anchor_row.addStretch()

        solve_row = QHBoxLayout()
        solve_row.setSpacing(6)
        solve_row.addWidget(self.solve_button)
        solve_row.addWidget(self.lock_button)
        solve_row.addWidget(self.camera_restart_button)
        solve_row.addStretch()

        pills_row = QHBoxLayout()
        pills_row.setSpacing(6)
        pills_row.addWidget(self.rms_pill)
        pills_row.addStretch()

        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.addLayout(header)
        layout.addLayout(top)
        layout.addWidget(self.landmark_list, stretch=1)
        layout.addLayout(target_row)
        layout.addLayout(anchor_row)
        layout.addLayout(solve_row)
        layout.addLayout(pills_row)
        layout.addWidget(self.detail_label)

        self._refresh_controls()

    # --- loading the table ------------------------------------------------

    def _on_load_clicked(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load site landmark table", "", "JSON (*.json)")
        if path:
            self.table_load_requested.emit(path)

    def set_table(self, table) -> None:
        """Replace the landmark list with ``table``'s entries, all ticked,
        with no measurements yet. Any previous solve is invalidated - a new
        table means the old fit no longer refers to anything real."""
        self.table = table
        self._measurements = {}
        self._rows = {}
        self.transform = None
        self.locked = False
        self.landmark_list.clear()
        self.rms_pill.setText("")
        self.detail_label.setText("")
        for lm in table.landmarks:
            self._add_row(lm.id)
        self.table_label.setText(_plain(f"{table.site_name} ({len(table)} landmarks)"))
        self._refresh_state_pill()
        self._refresh_controls()

    def _add_row(self, landmark_id: str) -> None:
        row = _LandmarkRow(landmark_id)
        row.checkbox.setChecked(True)
        item = QListWidgetItem()
        self.landmark_list.addItem(item)
        self.landmark_list.setItemWidget(item, row)
        item.setSizeHint(row.sizeHint())
        self._rows[landmark_id] = row
        row.checkbox.toggled.connect(lambda _checked: self._refresh_controls())
        self._update_row_text(landmark_id)

    def _update_row_text(self, landmark_id: str) -> None:
        row = self._rows.get(landmark_id)
        if row is None:
            return
        lm = self.table.by_id(landmark_id) if self.table is not None else None
        meas = self._measurements.get(landmark_id)

        parts = []
        if lm is not None:
            parts.append(f"{landmark_id}  site({lm.x:.2f}, {lm.y:.2f})")
            if lm.note:
                parts.append(lm.note)
        else:
            parts.append(f"{landmark_id}  — not in table")
        if meas is not None:
            parts.append(meas.quality)
            if meas.residual_m is not None:
                parts.append(f"residual {meas.residual_m:.2f} m")
        row.label.setText(_plain("   ".join(parts)))

        if lm is None:
            row.label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")
            row.checkbox.setChecked(False)
        else:
            row.label.setStyleSheet(f"color: {theme.TEXT}; border: none;")

    # --- measurements ------------------------------------------------------

    def set_measurement(self, landmark_id: str, x: float, y: float, quality: str) -> None:
        """Record a map-frame measurement for ``landmark_id``. An id that is
        not in the loaded table still gets a row - greyed, marked "not in
        table" - because that is exactly the information an operator needs
        when the dictionary or the id list is wrong; it is never included in
        a fit."""
        self._measurements[landmark_id] = _Measurement(x, y, quality)
        if landmark_id not in self._rows:
            self._add_row(landmark_id)
        else:
            self._update_row_text(landmark_id)
        self._refresh_state_pill()
        self._refresh_controls()

    def apply_sightings(self, report) -> None:
        """A stage-3 ``SightingsReport`` -> one ``set_measurement`` per
        sighting. A detector failure carries no sightings at all - its
        error is shown where the operator can see it and no measurement is
        added; a stage 3 that fails silently is worse than one that visibly
        does not run."""
        if not report.detector_ok:
            message = report.error or "the ArUco detector is not available"
            self.state_pill.setText(_plain(message))
            self.state_pill.setStyleSheet(theme.pill_style(theme.BAD, _fg_for(theme.BAD)))
            return
        for sighting in report.sightings:
            self.set_measurement(sighting.id, sighting.x, sighting.y, sighting.quality)

    def apply_probe_result(self, result) -> None:
        """A stage-2 ``ProbeResult`` -> one ``set_measurement``, quality
        derived from ``valid_fraction`` per the mapping fixed in §3.6 (the
        wire carries no quality of its own). A failed probe adds no
        measurement - its error is shown where the operator can see it."""
        if not result.ok:
            message = result.error or "the probe failed"
            self.state_pill.setText(_plain(message))
            self.state_pill.setStyleSheet(theme.pill_style(theme.BAD, _fg_for(theme.BAD)))
            return
        quality = "good" if result.valid_fraction >= 0.60 else "weak"
        self.set_measurement(result.label, result.x, result.y, quality)

    # --- solving -------------------------------------------------------

    def _build_pairs(self):
        pairs = []
        if self.table is None:
            return pairs
        for lm in self.table.landmarks:
            row = self._rows.get(lm.id)
            if row is None or not row.checkbox.isChecked():
                continue
            meas = self._measurements.get(lm.id)
            if meas is None:
                continue
            pairs.append(LandmarkPair(id=lm.id, site_x=lm.x, site_y=lm.y,
                                      map_x=meas.x, map_y=meas.y))
        return pairs

    def _on_solve_clicked(self) -> None:
        self.solve_requested.emit()
        pairs = self._build_pairs()
        try:
            t = solve_site_to_map(pairs)
        except SiteFrameError as exc:
            self.rms_pill.setText(_plain(str(exc)))
            self.rms_pill.setStyleSheet(theme.pill_style(theme.BAD, _fg_for(theme.BAD)))
            self._refresh_controls()
            return

        for m in self._measurements.values():
            m.residual_m = None
        for landmark_id, residual_m in residuals(t, pairs):
            m = self._measurements.get(landmark_id)
            if m is not None:
                m.residual_m = residual_m
        for landmark_id in list(self._measurements):
            self._update_row_text(landmark_id)

        self.transform = t
        self._refresh_rms_pill(t)
        self._refresh_detail_label(t)
        self._refresh_state_pill()
        self._refresh_controls()

    def _refresh_rms_pill(self, t) -> None:
        bg = theme.BAD if t.rms_m > 0.5 else theme.OK
        self.rms_pill.setText(_plain(f"RMS {t.rms_m:.2f} m"))
        self.rms_pill.setStyleSheet(theme.pill_style(bg, _fg_for(bg)))

    def _refresh_detail_label(self, t) -> None:
        parts = []
        if t.n_points == 2:
            parts.append(
                "2 landmarks: the residual only checks the distance between "
                "them. A third landmark is what catches a mis-identified "
                "marker.")
        if t.worst_id is not None:
            parts.append(f"worst residual {t.max_residual_m:.2f} m at '{t.worst_id}'")
        if not (0.9 <= t.scale_hint <= 1.1):
            pct = t.scale_hint * 100.0
            parts.append(
                f"measured spread is {pct:.0f}% of the published spread "
                "— check the landmark ids")
        self.detail_label.setText(_plain("  ".join(parts)))

    def _refresh_state_pill(self) -> None:
        if self.locked:
            text, bg = "LOCKED", theme.OK
        elif self.table is None:
            text, bg = "NO TABLE", theme.OFF
        elif self.transform is not None:
            text, bg = "SOLVED", theme.OK
        else:
            measured = sum(1 for lm in self.table.landmarks if lm.id in self._measurements)
            text, bg = f"{measured} OF {len(self.table)} MEASURED", theme.OFF
        self.state_pill.setText(_plain(text))
        self.state_pill.setStyleSheet(theme.pill_style(bg, _fg_for(bg)))

    # --- probe / anchor / lock ------------------------------------------

    def _on_probe_clicked(self) -> None:
        row_index = self.landmark_list.currentRow()
        if row_index < 0:
            return
        item = self.landmark_list.item(row_index)
        row = self.landmark_list.itemWidget(item)
        if row is None:
            return
        target = self.target_combo.currentData()
        self.probe_requested.emit(row.id, target)

    def _on_anchor_toggled(self, checked: bool) -> None:
        if checked:
            self.anchor_button.setText("Stop anchor")
            self.anchor_start_requested.emit()
        else:
            self.anchor_button.setText("Anchor")
            self.anchor_stop_requested.emit()

    def _on_lock_toggled(self, checked: bool) -> None:
        if checked:
            if self.transform is None:
                # Guard against a programmatic setChecked(True) with no
                # transform yet - the button is disabled in that state, but
                # setChecked() (unlike click()) bypasses enabled-state.
                self.lock_button.setChecked(False)
                return
            self.locked = True
            self.lock_changed.emit(self.transform)
        else:
            self.locked = False
            self.lock_changed.emit(None)
        self._refresh_state_pill()
        self._refresh_controls()

    # --- control enablement ----------------------------------------------

    def _refresh_controls(self) -> None:
        has_selection = self.landmark_list.currentRow() >= 0
        ticked_measured = 0
        if self.table is not None:
            for lm in self.table.landmarks:
                row = self._rows.get(lm.id)
                if (row is not None and row.checkbox.isChecked()
                        and lm.id in self._measurements):
                    ticked_measured += 1

        if self.locked:
            self.load_button.setEnabled(False)
            self.probe_button.setEnabled(False)
            self.anchor_button.setEnabled(False)
            self.solve_button.setEnabled(False)
            self.camera_restart_button.setEnabled(True)
            for row in self._rows.values():
                row.checkbox.setEnabled(False)
        else:
            self.load_button.setEnabled(True)
            self.probe_button.setEnabled(has_selection)
            self.anchor_button.setEnabled(True)
            self.solve_button.setEnabled(ticked_measured >= 2)
            self.camera_restart_button.setEnabled(False)
            for landmark_id, row in self._rows.items():
                in_table = self.table is not None and self.table.by_id(landmark_id) is not None
                row.checkbox.setEnabled(in_table)

        self.reset_button.setEnabled(True)
        self.lock_button.setEnabled(self.transform is not None or self.locked)
