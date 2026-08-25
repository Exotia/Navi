# Ground Station Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first working slice of the ground station: a cross-platform
Qt desktop app that connects to the rover over rosbridge, shows a live
Drive/Twist monitor (dashboard card + detail subpage), and a generic System
Nodes panel — so every later ROS2 node is verifiable the moment it exists.

**Architecture:** A small pure-Python model layer (`DriveState`,
`NodeRegistry`) with no Qt or ROS dependency, fully unit-testable. A thin
`RosBridgeClient` wraps `roslibpy` and re-emits everything as Qt signals
(`RosSignals`, a `QObject`) so cross-thread delivery from roslibpy's
background thread to the Qt GUI thread is handled by Qt's own queued
connections — no manual locking. The UI layer (`MainWindow` +
`DashboardPage` + `DriveDetailPage` + small widgets) only reads from the
model layer and reacts to signals; it holds no ROS or networking code
itself. Only Drive is wired up end-to-end in this plan — Localization/Mode/
Nav cards are deliberately not built yet (YAGNI; add each when its phase
lands).

**Tech Stack:** Python 3.10+, PySide6 (Qt bindings), roslibpy (rosbridge
websocket client), pytest + pytest-qt for tests (Qt tests run with
`QT_QPA_PLATFORM=offscreen`, no display required).

**Spec:** `docs/superpowers/specs/2026-08-25-nav-rebuild-design.md`

## Global Constraints

- Ground station machine must NOT require ROS2 installed — all rover
  communication goes through `roslibpy` over a rosbridge websocket.
- No custom velocity-vector graphics or other decorative widgets in this
  first slice — numeric readouts only (function over form; visual polish is
  an explicit later add-on, not part of this plan).
- Only the Drive card/detail page and System Nodes panel are built now.
  Localization/Mode/Nav cards are out of scope for this plan.
- Fallback triggers, mode switching, and localization are NOT implemented
  here — this plan is the ground station's Drive-monitoring slice only.

---

## File Structure

```
ground_station/
  __init__.py
  models.py              # DriveState, TwistSample, NodeStatus, NodeRegistry — no Qt/ROS deps
  ros_client.py           # RosSignals (QObject), RosBridgeClient (wraps roslibpy)
  theme.py                # shared color/font constants
  main.py                 # CLI entry point: argparse --host/--port, builds QApplication
  ui/
    __init__.py
    node_list_widget.py   # NodeListWidget: renders NodeRegistry snapshot
    drive_card.py          # DriveCard: compact dashboard card, emits details_requested
    drive_detail_page.py   # DriveDetailPage: full detail view, emits back_requested
    dashboard_page.py      # DashboardPage: assembles DriveCard + NodeListWidget
    main_window.py         # MainWindow: header, QStackedWidget, wires signals + QTimer poll
tests/
  __init__.py
  test_models.py
  test_ros_client.py
  test_ui_widgets.py
  test_main_window.py
pyproject.toml
```

---

### Task 1: Project scaffold

**Files:**
- Create: `pyproject.toml`
- Create: `ground_station/__init__.py`
- Create: `ground_station/theme.py`
- Create: `tests/__init__.py`
- Test: `tests/test_scaffold.py`

**Interfaces:**
- Produces: `ground_station.theme.BG`, `PANEL`, `BORDER`, `TEXT`, `TEXT_DIM`,
  `ACCENT`, `OK`, `OFF` (all `str`, hex colors), `FONT_FAMILY`,
  `MONO_FONT_FAMILY` (both `str`) — used by every widget task below.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_scaffold.py
def test_theme_constants_are_strings():
    from ground_station import theme

    for name in ("BG", "PANEL", "BORDER", "TEXT", "TEXT_DIM", "ACCENT", "OK", "OFF",
                 "FONT_FAMILY", "MONO_FONT_FAMILY"):
        assert isinstance(getattr(theme, name), str)
        assert getattr(theme, name) != ""
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scaffold.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station'`

- [ ] **Step 3: Write the package files**

```toml
# pyproject.toml
[project]
name = "ground-station"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = [
    "PySide6>=6.6",
    "roslibpy>=1.5",
]

[project.optional-dependencies]
dev = ["pytest>=8.0", "pytest-qt>=4.4"]

[tool.pytest.ini_options]
qt_api = "pyside6"
```

```python
# ground_station/__init__.py
```

```python
# ground_station/theme.py
"""Shared visual constants, matching the approved ground-station wireframe."""

BG = "#14171c"
PANEL = "#1c2028"
BORDER = "#2f3542"
TEXT = "#e6e8ec"
TEXT_DIM = "#7d8590"
ACCENT = "#e8823c"
OK = "#3fb950"
OFF = "#545d6b"

FONT_FAMILY = "IBM Plex Sans"
MONO_FONT_FAMILY = "IBM Plex Mono"
```

```python
# tests/__init__.py
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scaffold.py -v`
Expected: PASS

- [ ] **Step 5: Install dependencies and commit**

```bash
pip install -e ".[dev]"
git add pyproject.toml ground_station/__init__.py ground_station/theme.py tests/__init__.py tests/test_scaffold.py
git commit -m "feat(ground-station): project scaffold and theme constants"
```

---

### Task 2: DriveState model

**Files:**
- Create: `ground_station/models.py`
- Test: `tests/test_models.py`

**Interfaces:**
- Consumes: nothing (pure Python, no dependencies).
- Produces: `TwistSample` (dataclass: `linear_x: float, linear_y: float,
  angular_z: float, received_at: float`); `DriveState` with methods
  `ingest(linear_x: float, linear_y: float, angular_z: float, now:
  float | None = None) -> None`, property `latest: TwistSample | None`,
  property `rate_hz: float`, method `seconds_since_last(now: float | None =
  None) -> float | None`. Used by `ui/drive_card.py`, `ui/drive_detail_page.py`,
  and `ui/main_window.py`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_models.py
from ground_station.models import DriveState


def test_ingest_stores_latest_sample():
    state = DriveState()
    state.ingest(0.4, -0.05, 0.1, now=10.0)

    assert state.latest.linear_x == 0.4
    assert state.latest.linear_y == -0.05
    assert state.latest.angular_z == 0.1
    assert state.latest.received_at == 10.0


def test_rate_hz_is_zero_with_fewer_than_two_samples():
    state = DriveState()
    assert state.rate_hz == 0.0
    state.ingest(0.0, 0.0, 0.0, now=10.0)
    assert state.rate_hz == 0.0


def test_rate_hz_computes_from_recent_samples():
    state = DriveState(rate_window_seconds=2.0)
    # 5 samples spaced 0.1s apart -> 10 Hz
    for i in range(5):
        state.ingest(0.0, 0.0, 0.0, now=10.0 + i * 0.1)

    assert state.rate_hz == pytest.approx(10.0, rel=0.05)


def test_rate_hz_drops_samples_outside_window():
    state = DriveState(rate_window_seconds=1.0)
    state.ingest(0.0, 0.0, 0.0, now=0.0)
    state.ingest(0.0, 0.0, 0.0, now=5.0)
    state.ingest(0.0, 0.0, 0.0, now=5.5)

    # only the last two samples (5.0, 5.5) are within the 1s window
    assert state.rate_hz == pytest.approx(2.0, rel=0.05)


def test_seconds_since_last_none_before_any_sample():
    state = DriveState()
    assert state.seconds_since_last(now=10.0) is None


def test_seconds_since_last_computes_elapsed_time():
    state = DriveState()
    state.ingest(0.0, 0.0, 0.0, now=10.0)
    assert state.seconds_since_last(now=10.5) == pytest.approx(0.5)
```

Add `import pytest` at the top of the file.

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_models.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.models'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/models.py
"""Pure-Python state models for the ground station — no Qt or ROS imports."""

from dataclasses import dataclass, field
from time import monotonic


@dataclass
class TwistSample:
    linear_x: float
    linear_y: float
    angular_z: float
    received_at: float


class DriveState:
    """Tracks the latest /cmd_vel Twist message and its incoming rate."""

    def __init__(self, rate_window_seconds: float = 2.0):
        self.rate_window_seconds = rate_window_seconds
        self.latest: TwistSample | None = None
        self._timestamps: list[float] = []

    def ingest(self, linear_x: float, linear_y: float, angular_z: float,
               now: float | None = None) -> None:
        now = monotonic() if now is None else now
        self.latest = TwistSample(linear_x, linear_y, angular_z, now)
        self._timestamps.append(now)
        cutoff = now - self.rate_window_seconds
        self._timestamps = [t for t in self._timestamps if t >= cutoff]

    @property
    def rate_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / span

    def seconds_since_last(self, now: float | None = None) -> float | None:
        if self.latest is None:
            return None
        now = monotonic() if now is None else now
        return now - self.latest.received_at


@dataclass
class NodeStatus:
    name: str
    alive: bool
    last_seen: float


class NodeRegistry:
    """Tracks which ROS2 nodes are currently present, from periodic polls
    of rosbridge's rosapi node list."""

    def __init__(self, stale_after_seconds: float = 5.0):
        self.stale_after_seconds = stale_after_seconds
        self._nodes: dict[str, NodeStatus] = {}

    def update(self, present_node_names: list[str], now: float | None = None) -> None:
        now = monotonic() if now is None else now
        for name in present_node_names:
            self._nodes[name] = NodeStatus(name=name, alive=True, last_seen=now)
        cutoff = now - self.stale_after_seconds
        for status in self._nodes.values():
            if status.last_seen < cutoff:
                status.alive = False

    def snapshot(self) -> list[NodeStatus]:
        return sorted(self._nodes.values(), key=lambda s: s.name)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_models.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add ground_station/models.py tests/test_models.py
git commit -m "feat(ground-station): DriveState and NodeRegistry models"
```

---

### Task 3: NodeRegistry tests

**Files:**
- Modify: `tests/test_models.py` (append; `NodeRegistry` itself was already
  written in Task 2 — this task only adds its test coverage, since it's a
  second, independently-reviewable behavior)

**Interfaces:**
- Consumes: `NodeRegistry` from Task 2 (`ground_station/models.py`).

- [ ] **Step 1: Write the failing tests**

```python
# append to tests/test_models.py
from ground_station.models import NodeRegistry


def test_update_marks_present_nodes_alive():
    registry = NodeRegistry()
    registry.update(["/cmd_vel_bridge", "/rosbridge_websocket"], now=10.0)

    names = [n.name for n in registry.snapshot()]
    assert names == ["/cmd_vel_bridge", "/rosbridge_websocket"]
    assert all(n.alive for n in registry.snapshot())


def test_update_marks_missing_nodes_stale_after_timeout():
    registry = NodeRegistry(stale_after_seconds=1.0)
    registry.update(["/cmd_vel_bridge"], now=0.0)
    # node no longer reported present, and enough time has passed
    registry.update([], now=2.0)

    status = registry.snapshot()[0]
    assert status.name == "/cmd_vel_bridge"
    assert status.alive is False


def test_update_keeps_node_alive_within_stale_window():
    registry = NodeRegistry(stale_after_seconds=5.0)
    registry.update(["/cmd_vel_bridge"], now=0.0)
    registry.update([], now=1.0)  # missing from this poll, but within window

    assert registry.snapshot()[0].alive is True


def test_snapshot_is_sorted_by_name():
    registry = NodeRegistry()
    registry.update(["/zzz_node", "/aaa_node"], now=0.0)

    names = [n.name for n in registry.snapshot()]
    assert names == ["/aaa_node", "/zzz_node"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_models.py -v -k NodeRegistry or update or snapshot`
Expected: These 4 new tests FAIL only if `NodeRegistry` behavior doesn't
match (it should already pass since Task 2 implemented it — if any fail,
fix `NodeRegistry` in `ground_station/models.py` before proceeding).

- [ ] **Step 3: Run full test file to confirm everything passes**

Run: `pytest tests/test_models.py -v`
Expected: PASS (10 tests total)

- [ ] **Step 4: Commit**

```bash
git add tests/test_models.py
git commit -m "test(ground-station): NodeRegistry behavior coverage"
```

---

### Task 4: RosBridgeClient + Qt signal bridge

**Files:**
- Create: `ground_station/ros_client.py`
- Test: `tests/test_ros_client.py`

**Interfaces:**
- Consumes: `roslibpy.Ros`, `roslibpy.Topic` (real classes, injected as
  factories so tests substitute fakes).
- Produces: `RosSignals(QObject)` with signals `twist_received(dict)`,
  `nodes_received(list)`, `connection_changed(bool)`. `RosBridgeClient`
  with constructor `(host: str, port: int = 9090, ros_factory=roslibpy.Ros,
  topic_factory=roslibpy.Topic)`, attribute `signals: RosSignals`, methods
  `connect() -> None`, `close() -> None`, property `is_connected: bool`,
  `subscribe_cmd_vel(topic_name: str = "/cmd_vel") -> None`,
  `poll_nodes() -> None`. Used by `ui/main_window.py`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_ros_client.py
from ground_station.ros_client import RosBridgeClient


class FakeTopic:
    instances = []

    def __init__(self, ros, name, msg_type):
        self.ros = ros
        self.name = name
        self.msg_type = msg_type
        self.callback = None
        FakeTopic.instances.append(self)

    def subscribe(self, callback):
        self.callback = callback


class FakeRos:
    instances = []

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.is_connected = False
        self.ready_callback = None
        self.get_nodes_callback = None
        FakeRos.instances.append(self)

    def on_ready(self, callback):
        self.ready_callback = callback

    def run(self):
        self.is_connected = True
        if self.ready_callback:
            self.ready_callback()

    def close(self):
        self.is_connected = False

    def get_nodes(self, callback, errback=None):
        self.get_nodes_callback = callback


def make_client(qtbot):
    FakeRos.instances.clear()
    FakeTopic.instances.clear()
    return RosBridgeClient(host="localhost", port=9090,
                            ros_factory=FakeRos, topic_factory=FakeTopic)


def test_connect_starts_ros_and_emits_connection_changed(qtbot):
    client = make_client(qtbot)
    with qtbot.waitSignal(client.signals.connection_changed, timeout=1000) as blocker:
        client.connect()

    assert blocker.args == [True]
    assert client.is_connected is True


def test_subscribe_cmd_vel_creates_twist_topic(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_cmd_vel()

    topic = FakeTopic.instances[-1]
    assert topic.name == "/cmd_vel"
    assert topic.msg_type == "geometry_msgs/Twist"


def test_twist_message_emits_twist_received_signal(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_cmd_vel()
    topic = FakeTopic.instances[-1]

    sample_msg = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0},
                  "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}
    with qtbot.waitSignal(client.signals.twist_received, timeout=1000) as blocker:
        topic.callback(sample_msg)

    assert blocker.args == [sample_msg]


def test_poll_nodes_emits_nodes_received_signal(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.poll_nodes()
    ros = FakeRos.instances[-1]

    with qtbot.waitSignal(client.signals.nodes_received, timeout=1000) as blocker:
        ros.get_nodes_callback(["/cmd_vel_bridge", "/rosbridge_websocket"])

    assert blocker.args == [["/cmd_vel_bridge", "/rosbridge_websocket"]]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ros_client.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.ros_client'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/ros_client.py
"""Wraps roslibpy for connecting to rosbridge; re-emits everything as Qt
signals so roslibpy's background-thread callbacks are safely marshaled to
the Qt GUI thread by Qt's own queued-connection mechanism."""

import roslibpy
from PySide6.QtCore import QObject, Signal


class RosSignals(QObject):
    twist_received = Signal(dict)
    nodes_received = Signal(list)
    connection_changed = Signal(bool)


class RosBridgeClient:
    def __init__(self, host: str, port: int = 9090,
                 ros_factory=roslibpy.Ros, topic_factory=roslibpy.Topic):
        self.signals = RosSignals()
        self._topic_factory = topic_factory
        self._ros = ros_factory(host=host, port=port)
        self._cmd_vel_topic = None

    def connect(self) -> None:
        self._ros.on_ready(lambda: self.signals.connection_changed.emit(True))
        self._ros.run()

    def close(self) -> None:
        self._ros.close()
        self.signals.connection_changed.emit(False)

    @property
    def is_connected(self) -> bool:
        return bool(self._ros.is_connected)

    def subscribe_cmd_vel(self, topic_name: str = "/cmd_vel") -> None:
        self._cmd_vel_topic = self._topic_factory(self._ros, topic_name, "geometry_msgs/Twist")
        self._cmd_vel_topic.subscribe(lambda msg: self.signals.twist_received.emit(msg))

    def poll_nodes(self) -> None:
        self._ros.get_nodes(lambda nodes: self.signals.nodes_received.emit(nodes))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ros_client.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add ground_station/ros_client.py tests/test_ros_client.py
git commit -m "feat(ground-station): RosBridgeClient with Qt signal bridge"
```

---

### Task 5: DriveCard and NodeListWidget

**Files:**
- Create: `ground_station/ui/__init__.py`
- Create: `ground_station/ui/drive_card.py`
- Create: `ground_station/ui/node_list_widget.py`
- Test: `tests/test_ui_widgets.py`

**Interfaces:**
- Consumes: `ground_station.models.DriveState`, `NodeStatus`; `ground_station.theme`.
- Produces: `DriveCard(QWidget)` with method `update_from(state: DriveState)
  -> None` and Qt signal `details_requested()` (emitted when its
  "view details" label is clicked). `NodeListWidget(QWidget)` with method
  `update_from(statuses: list[NodeStatus]) -> None`. Used by
  `ui/dashboard_page.py`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_ui_widgets.py
from ground_station.models import DriveState, NodeStatus
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.node_list_widget import NodeListWidget


def test_drive_card_shows_dash_before_any_data(qtbot):
    card = DriveCard()
    qtbot.addWidget(card)
    assert "--" in card.vx_label.text()


def test_drive_card_updates_from_state(qtbot):
    card = DriveCard()
    qtbot.addWidget(card)
    state = DriveState()
    state.ingest(0.42, -0.05, 0.10, now=10.0)

    card.update_from(state)

    assert "0.42" in card.vx_label.text()
    assert "-0.05" in card.vy_label.text()
    assert "0.10" in card.wz_label.text()


def test_drive_card_emits_details_requested_on_click(qtbot):
    card = DriveCard()
    qtbot.addWidget(card)

    with qtbot.waitSignal(card.details_requested, timeout=1000):
        card.details_link.mousePressEvent(None)


def test_node_list_widget_starts_empty(qtbot):
    widget = NodeListWidget()
    qtbot.addWidget(widget)
    assert widget.row_count() == 0


def test_node_list_widget_shows_a_row_per_node(qtbot):
    widget = NodeListWidget()
    qtbot.addWidget(widget)
    statuses = [
        NodeStatus(name="/cmd_vel_bridge", alive=True, last_seen=10.0),
        NodeStatus(name="/amcl", alive=False, last_seen=1.0),
    ]

    widget.update_from(statuses)

    assert widget.row_count() == 2
    assert "cmd_vel_bridge" in widget.row_text(0)
    assert "amcl" in widget.row_text(1)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ui_widgets.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.ui'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/ui/__init__.py
```

```python
# ground_station/ui/drive_card.py
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel

from ground_station import theme
from ground_station.models import DriveState


class DriveCard(QWidget):
    details_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"background-color: {theme.PANEL}; border: 1px solid {theme.BORDER}; border-radius: 6px;"
        )

        title = QLabel("DRIVE / TWIST")
        title.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600; border: none;")

        self.vx_label = QLabel("vx (cmd)  --")
        self.vy_label = QLabel("vy (cmd)  --")
        self.wz_label = QLabel("wz (cmd)  --")
        self.rate_label = QLabel("/cmd_vel  --")
        for label in (self.vx_label, self.vy_label, self.wz_label, self.rate_label):
            label.setStyleSheet(f"color: {theme.TEXT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")

        self.details_link = QLabel("view details →")
        self.details_link.setStyleSheet(f"color: {theme.ACCENT}; border: none;")
        self.details_link.mousePressEvent = self._on_details_clicked

        layout = QVBoxLayout(self)
        layout.addWidget(title)
        for label in (self.vx_label, self.vy_label, self.wz_label, self.rate_label):
            layout.addWidget(label)
        footer = QHBoxLayout()
        footer.addStretch()
        footer.addWidget(self.details_link)
        layout.addLayout(footer)

    def _on_details_clicked(self, event):
        self.details_requested.emit()

    def update_from(self, state: DriveState) -> None:
        if state.latest is None:
            return
        self.vx_label.setText(f"vx (cmd)  {state.latest.linear_x:.2f} m/s")
        self.vy_label.setText(f"vy (cmd)  {state.latest.linear_y:.2f} m/s")
        self.wz_label.setText(f"wz (cmd)  {state.latest.angular_z:.2f} rad/s")
        self.rate_label.setText(f"/cmd_vel  {state.rate_hz:.0f} Hz")
```

```python
# ground_station/ui/node_list_widget.py
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ui_widgets.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
git add ground_station/ui/__init__.py ground_station/ui/drive_card.py ground_station/ui/node_list_widget.py tests/test_ui_widgets.py
git commit -m "feat(ground-station): DriveCard and NodeListWidget"
```

---

### Task 6: DriveDetailPage

**Files:**
- Create: `ground_station/ui/drive_detail_page.py`
- Modify: `tests/test_ui_widgets.py` (append)

**Interfaces:**
- Consumes: `ground_station.models.DriveState`; `ground_station.theme`.
- Produces: `DriveDetailPage(QWidget)` with method `update_from(state:
  DriveState) -> None`, method `append_raw_message(text: str) -> None`, and
  Qt signal `back_requested()`. Used by `ui/main_window.py`.

- [ ] **Step 1: Write the failing tests**

```python
# append to tests/test_ui_widgets.py
from ground_station.ui.drive_detail_page import DriveDetailPage


def test_drive_detail_page_updates_from_state(qtbot):
    page = DriveDetailPage()
    qtbot.addWidget(page)
    state = DriveState()
    state.ingest(0.42, -0.05, 0.10, now=10.0)

    page.update_from(state)

    assert "0.42" in page.vx_label.text()
    assert "0.10" in page.wz_label.text()


def test_drive_detail_page_appends_raw_messages():
    page = DriveDetailPage()
    page.append_raw_message("linear.x=0.42 linear.y=-0.05 angular.z=0.10")

    assert "0.42" in page.raw_log.toPlainText()


def test_drive_detail_page_emits_back_requested(qtbot):
    page = DriveDetailPage()
    qtbot.addWidget(page)

    with qtbot.waitSignal(page.back_requested, timeout=1000):
        page.back_link.mousePressEvent(None)
```

Add `from ground_station.models import DriveState` at the top if not
already imported in this file (it is, from Task 5's tests).

- [ ] **Step 2: Run tests to verify they fail**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ui_widgets.py -v -k DriveDetailPage`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.ui.drive_detail_page'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/ui/drive_detail_page.py
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ui_widgets.py -v`
Expected: PASS (8 tests total)

- [ ] **Step 5: Commit**

```bash
git add ground_station/ui/drive_detail_page.py tests/test_ui_widgets.py
git commit -m "feat(ground-station): DriveDetailPage"
```

---

### Task 7: DashboardPage

**Files:**
- Create: `ground_station/ui/dashboard_page.py`
- Modify: `tests/test_ui_widgets.py` (append)

**Interfaces:**
- Consumes: `DriveCard` (Task 5), `NodeListWidget` (Task 5).
- Produces: `DashboardPage(QWidget)` with attributes `drive_card:
  DriveCard`, `node_list: NodeListWidget`, and Qt signal
  `drive_details_requested()` (re-emitted from `drive_card.details_requested`).
  Used by `ui/main_window.py`.

- [ ] **Step 1: Write the failing test**

```python
# append to tests/test_ui_widgets.py
from ground_station.ui.dashboard_page import DashboardPage


def test_dashboard_page_exposes_drive_card_and_node_list(qtbot):
    page = DashboardPage()
    qtbot.addWidget(page)

    assert isinstance(page.drive_card, DriveCard)
    assert isinstance(page.node_list, NodeListWidget)


def test_dashboard_page_reemits_drive_details_requested(qtbot):
    page = DashboardPage()
    qtbot.addWidget(page)

    with qtbot.waitSignal(page.drive_details_requested, timeout=1000):
        page.drive_card.details_requested.emit()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ui_widgets.py -v -k DashboardPage`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.ui.dashboard_page'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/ui/dashboard_page.py
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_ui_widgets.py -v`
Expected: PASS (10 tests total)

- [ ] **Step 5: Commit**

```bash
git add ground_station/ui/dashboard_page.py tests/test_ui_widgets.py
git commit -m "feat(ground-station): DashboardPage"
```

---

### Task 8: MainWindow and CLI entry point

**Files:**
- Create: `ground_station/ui/main_window.py`
- Create: `ground_station/main.py`
- Test: `tests/test_main_window.py`

**Interfaces:**
- Consumes: `RosBridgeClient`, `RosSignals` (Task 4); `DashboardPage`
  (Task 7); `DriveDetailPage` (Task 6); `DriveState`, `NodeRegistry`
  (Task 2).
- Produces: `MainWindow(QMainWindow)` with constructor
  `(ros_client: RosBridgeClient, node_poll_interval_ms: int = 2000)`,
  attributes `dashboard_page`, `drive_detail_page`, `connection_label`.
  `main.py` provides `build_arg_parser()` and `main()` as the console entry
  point.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_main_window.py
from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


class FakeTopic:
    def __init__(self, ros, name, msg_type):
        self.callback = None

    def subscribe(self, callback):
        self.callback = callback


class FakeRos:
    def __init__(self, host, port):
        self.is_connected = False
        self.ready_callback = None

    def on_ready(self, callback):
        self.ready_callback = callback

    def run(self):
        self.is_connected = True
        self.ready_callback()

    def close(self):
        self.is_connected = False

    def get_nodes(self, callback, errback=None):
        pass


def make_window(qtbot):
    client = RosBridgeClient(host="localhost", ros_factory=FakeRos, topic_factory=FakeTopic)
    window = MainWindow(client)
    qtbot.addWidget(window)
    return window, client


def test_main_window_starts_on_dashboard_page(qtbot):
    window, _ = make_window(qtbot)
    assert window.stacked_widget.currentWidget() is window.dashboard_page


def test_clicking_drive_card_details_shows_detail_page(qtbot):
    window, _ = make_window(qtbot)
    window.dashboard_page.drive_card.details_requested.emit()

    assert window.stacked_widget.currentWidget() is window.drive_detail_page


def test_back_from_detail_returns_to_dashboard(qtbot):
    window, _ = make_window(qtbot)
    window.dashboard_page.drive_card.details_requested.emit()
    window.drive_detail_page.back_requested.emit()

    assert window.stacked_widget.currentWidget() is window.dashboard_page


def test_twist_message_updates_drive_card(qtbot):
    window, client = make_window(qtbot)
    client.connect()
    client.subscribe_cmd_vel()

    # calling the slot directly here: it's a plain method, not a signal to
    # wait on — the signal->slot wiring itself is covered by
    # test_ros_client.py's test_twist_message_emits_twist_received_signal
    msg = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}
    window._on_twist(msg)

    assert "0.40" in window.dashboard_page.drive_card.vx_label.text()


def test_connection_changed_updates_label(qtbot):
    window, client = make_window(qtbot)
    client.connect()

    assert "CONNECTED" in window.connection_label.text()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_main_window.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.ui.main_window'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/ui/main_window.py
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QStackedWidget

from ground_station import theme
from ground_station.models import DriveState, NodeRegistry
from ground_station.ros_client import RosBridgeClient
from ground_station.ui.dashboard_page import DashboardPage
from ground_station.ui.drive_detail_page import DriveDetailPage


class MainWindow(QMainWindow):
    def __init__(self, ros_client: RosBridgeClient, node_poll_interval_ms: int = 2000):
        super().__init__()
        self.setWindowTitle("Asterope Ground Station")
        self.ros_client = ros_client
        self.drive_state = DriveState()
        self.node_registry = NodeRegistry()

        self.connection_label = QLabel("ROSBRIDGE: DISCONNECTED")
        self.connection_label.setStyleSheet(f"color: {theme.TEXT_DIM};")

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.addWidget(QLabel("ASTEROPE GROUND STATION"))
        header_layout.addStretch()
        header_layout.addWidget(self.connection_label)

        self.dashboard_page = DashboardPage()
        self.drive_detail_page = DriveDetailPage()
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.dashboard_page)
        self.stacked_widget.addWidget(self.drive_detail_page)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(header)
        layout.addWidget(self.stacked_widget)
        self.setCentralWidget(central)

        self.dashboard_page.drive_details_requested.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.drive_detail_page)
        )
        self.drive_detail_page.back_requested.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.dashboard_page)
        )

        self.ros_client.signals.twist_received.connect(self._on_twist)
        self.ros_client.signals.nodes_received.connect(self._on_nodes)
        self.ros_client.signals.connection_changed.connect(self._on_connection_changed)

        self._node_poll_timer = QTimer(self)
        self._node_poll_timer.timeout.connect(self.ros_client.poll_nodes)
        self._node_poll_timer.start(node_poll_interval_ms)

    def _on_twist(self, msg: dict) -> None:
        linear = msg.get("linear", {})
        angular = msg.get("angular", {})
        self.drive_state.ingest(linear.get("x", 0.0), linear.get("y", 0.0), angular.get("z", 0.0))
        self.dashboard_page.drive_card.update_from(self.drive_state)
        self.drive_detail_page.update_from(self.drive_state)
        self.drive_detail_page.append_raw_message(
            f"linear.x={linear.get('x', 0.0):.2f} linear.y={linear.get('y', 0.0):.2f} "
            f"angular.z={angular.get('z', 0.0):.2f}"
        )

    def _on_nodes(self, names: list) -> None:
        self.node_registry.update(names)
        self.dashboard_page.node_list.update_from(self.node_registry.snapshot())

    def _on_connection_changed(self, connected: bool) -> None:
        text = "ROSBRIDGE: CONNECTED" if connected else "ROSBRIDGE: DISCONNECTED"
        color = theme.OK if connected else theme.TEXT_DIM
        self.connection_label.setText(text)
        self.connection_label.setStyleSheet(f"color: {color};")
```

```python
# ground_station/main.py
import argparse
import sys

from PySide6.QtWidgets import QApplication

from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Asterope ground station")
    parser.add_argument("--host", required=True, help="rosbridge websocket host, e.g. 192.168.1.50")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge websocket port (default 9090)")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    app = QApplication(sys.argv)
    client = RosBridgeClient(host=args.host, port=args.port)
    window = MainWindow(client)
    window.resize(1440, 900)
    window.show()

    client.connect()
    client.subscribe_cmd_vel()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `QT_QPA_PLATFORM=offscreen pytest tests/test_main_window.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Run the full test suite**

Run: `QT_QPA_PLATFORM=offscreen pytest -v`
Expected: PASS (all tests across every task)

- [ ] **Step 6: Commit**

```bash
git add ground_station/ui/main_window.py ground_station/main.py tests/test_main_window.py
git commit -m "feat(ground-station): MainWindow wiring and CLI entry point"
```

---

### Task 9: Manual verification against a real rosbridge

**Files:**
- Create: `docs/superpowers/plans/2026-08-25-ground-station-manual-verification.md`

**Interfaces:**
- Consumes: `ground_station/main.py` (Task 8).

This task has no automated test — it documents the one thing that
genuinely cannot be verified without ROS2 + rosbridge running somewhere
(the Jetson, per the spec), so it isn't silently skipped.

- [ ] **Step 1: Write the manual verification doc**

```markdown
# Ground Station — Manual Verification

Automated tests (`pytest`) cover all app logic with fake ROS objects. This
step is the one real check that needs an actual rosbridge server, which
isn't available in the dev sandbox this plan was built in.

## Steps (run on/near the Jetson, once ROS2 Humble + rosbridge_suite are set up)

1. On the Jetson: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
2. In another terminal on the Jetson: publish a test Twist repeatedly, e.g.
   `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: -0.05}, angular: {z: 0.1}}"`
3. On the ground station machine: `python -m ground_station.main --host <jetson-ip>`
4. Confirm:
   - Header shows "ROSBRIDGE: CONNECTED" within a couple seconds.
   - The Drive card shows live vx/vy/wz values matching step 2 and a
     nonzero Hz.
   - System Nodes panel lists at least `/rosbridge_websocket`.
   - Clicking "view details" opens the Drive detail page with the same
     live values and a scrolling raw-message log; "back to dashboard"
     returns to the dashboard.
5. Stop the `ros2 topic pub` command and confirm the Drive card's Hz
   reading decays toward 0 (no new samples entering the rate window).
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/plans/2026-08-25-ground-station-manual-verification.md
git commit -m "docs(ground-station): manual verification steps against real rosbridge"
```
