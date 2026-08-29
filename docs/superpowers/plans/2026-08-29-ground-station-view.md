# Ground Station and View (sub-project 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The ground station gets four modes, and in Semi-autonomous the operator drives on a Gazebo view whose rover is placed by the rover's own `/localization/pose` instead of by dead reckoning — with no camera stream, and with the localisation health always on screen.

**Architecture:** `dashboard_page` grows to four radio buttons and `main_window._mode` to four values; `simulation` keeps today's dead-reckoned behaviour verbatim, `semi_auto` becomes a new branch that refuses the rover's camera and shows the localisation marker. `ros_client` gains two rosbridge subscriptions (`/localization/status`, and `/localization/pose` throttled to 5 Hz for a header readout). On the simulation side, `start_sim.sh --mode semi` puts the whole Gazebo launch on its own ROS domain, a `sim_bridge` node carries four topics **one way** from the rover graph into it with two `rclpy` contexts in one process, and `sim_ik_node` gains a `pose_topic` parameter: when set, it replaces its integrated body pose with the received one and moves the Gazebo model through `/gazebo/set_entity_state` at up to 30 Hz, holding still whenever `/localization/status` is not `OK`. `planar_move` is not loaded in that mode, because two writers of one model pose would fight.

**Tech Stack:** PySide6 + roslibpy (ground station, pure venv, no ROS), pytest + pytest-qt, ROS 2 Humble (`rclpy`, `rclcpp`), Gazebo Classic 11 with `gazebo_ros_state` / `gazebo_ros_joint_pose_trajectory`, GTest via `ament_cmake_gtest`, `ament_cmake_pytest`, bash.

**Spec:** `docs/superpowers/specs/2026-08-29-localisation-design.md` — section "Sub-project 2: ground station and view", plus "Architecture", "Constraints found in the environment" and "Error handling". Sub-project 1's plan (`docs/superpowers/plans/2026-08-29-rover-localisation.md`) produces the two topics this one consumes.

## Global Constraints

- **Nothing under `ground_station/` may import `rclpy`.** The venv is built with `include-system-site-packages = false` and cannot. The ground station is a rosbridge client and learns nothing about ROS.
- `/manual_twist` drives the **physical rover**. No test, no fixture and no manual check in this plan may publish to it. Use a scratch topic (`/sim_test_twist`) or a throwaway domain.
- Everything under `sim/src/navi_sim_ik/vendor/` is read-only vendored IK. Do not change a byte of it.
- Interfaces produced by sub-project 1 and consumed here, exactly:
  - `/localization/pose` — `nav_msgs/Odometry`, `frame_id: map`, `child_frame_id: base_footprint`, ~30 Hz. While localisation is `SEARCHING` it keeps publishing the **last good** pose with its stamp **frozen**.
  - `/localization/status` — `std_msgs/String` carrying JSON: `{"state": "OK" | "SEARCHING" | "OFF", "seconds_since_ok": float | null, "source": "zed_vio", "distance_travelled": float, "mount_offset_verified": bool}`, at 2 Hz and on every state change.
  - The ZED wrapper's own topics live under `/zed_front/zed_node/`. Nothing in this sub-project subscribes to them.
- **Nothing flows from the sim domain back to the rover graph.** `/clock`, `/tf`, `/gazebo/*` and the sim's `/robot_description` must never appear on the rover's domain. This is the decision that closes the collision the 2026-08-28 design flagged, and Task 9 makes it structural rather than a rule to remember.
- On this laptop `ros2` and `gazebo` exist only after sourcing, and the default shell is zsh where that fails. Always `bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && ...'`.
- There is no `domain_bridge` package on either machine. Humble's `rclpy` supports several contexts with different `domain_id`s in one process (`ctx = rclpy.Context(); rclpy.init(context=ctx, domain_id=N)`), which is enough. Verified on this laptop.
- `grid_map_msgs` is **not installed on this laptop** (`/opt/ros/humble/share/grid_map_msgs` does not exist); it arrives with sub-project 3. `sim_bridge` must therefore resolve message types by name at runtime and skip, with a warning, any topic whose type will not import — never fail to start.
- The sim domain default is **42**; `--sim-domain` overrides it. Domain 0 is the rover's and is never a valid sim domain.
- Laptop tests: `.venv/bin/pytest tests/ -q` from the repo root. Sim tests: `cd sim && colcon test && colcon test-result --verbose` (inside a sourced `bash -c`).
- Commits: `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit ...` (no identity is configured in this repo). Commit messages are full sentences describing the change.
- **Never use `pkill -f` / `pgrep -f` with a pattern that could match your own shell's command line.** `start_sim.sh` already has the `own_pids` guard; new patterns go through the same `kill_stale` helper.
- Naming: identifiers use the American spelling `localization` (it matches the topic names); operator-facing strings use the British spelling the spec writes (`LOCALISED`, `LOCALISATION OFF`). Do not mix them inside identifiers.

## File structure

```
ground_station/
  ui/dashboard_page.py      # modified: four radios, autonomous disabled, mode_changed
  ui/main_window.py         # modified: four-mode dispatch, semi-auto refusal, header readout
  ui/video_panel.py         # modified: localisation marker, refuse_stream()
  ros_client.py             # modified: /localization/status + /localization/pose (5 Hz)
  models.py                 # modified: yaw_from_quaternion, pose_readout_from_odometry
mock/
  square_walk.py            # new: the pure square the fake pose walks (no deps)
  ros_bridge.py             # modified: fake /localization/pose + /localization/status
  fake_localization.py      # new: the same square on a real DDS domain (rclpy, test fixture)
tests/
  test_ui_widgets.py        # modified: the four radios
  test_main_window.py       # modified: four modes, refusal, marker, header readout
  test_video_panel.py       # modified: marker text and colour, refuse_stream
  test_ros_client.py        # modified: the two new subscriptions and the throttle
  test_models.py            # modified: yaw and pose readout
  test_square_walk.py       # new
sim/src/navi_sim_ik/
  include/navi_sim_ik/external_pose.hpp   # new: localization_state(), ExternalPoseGate
  include/navi_sim_ik/sim_ik_stepper.hpp  # modified: set_pose()
  src/external_pose.cpp                   # new
  src/sim_ik_node.cpp                     # modified: pose_topic, /gazebo/set_entity_state
  test/test_external_pose.cpp             # new
  test/test_sim_ik_stepper.cpp            # modified: set_pose
  CMakeLists.txt, package.xml             # modified: gazebo_msgs, the new library and gtest
sim/src/navi_sim_bringup/
  worlds/site.world                       # modified: libgazebo_ros_state.so
  urdf/asterope_sim.urdf.xacro            # modified: planar_move behind a xacro arg
  launch/sim.launch.py                    # modified: mode / sim_domain / rover_domain
  scripts/sim_bridge.py                   # new: the one-way two-context bridge
  test/test_sim_bridge.py                 # new
  CMakeLists.txt, package.xml             # modified: install the script, run the pytest
start_sim.sh                              # modified: --mode, --sim-domain, --rover-domain
```

---

### Task 1: Four modes in the dashboard and the window

Today there are two radio buttons and `main_window._mode` takes `"manual"` or `"semi_auto"`, where `semi_auto` means "show the dead-reckoned simulation". The spec renames that behaviour to `simulation` and frees `semi_auto` for the localised view. This task does the rename and adds the two new buttons; Tasks 2 and 3 then give `semi_auto` its own behaviour.

**Files:**
- Modify: `ground_station/ui/dashboard_page.py:13-16` (the `mode_changed` docstring), `:29-44` (the radios), `:57-63` (`_on_mode_toggled`)
- Modify: `ground_station/ui/main_window.py:331-365` (`_on_mode_changed`), `:241` (the disconnect guard)
- Test: `tests/test_ui_widgets.py`, `tests/test_main_window.py`

**Interfaces:**
- Produces:
  - `DashboardPage.manual_radio`, `.semi_auto_radio`, `.autonomous_radio`, `.simulation_radio` — `QRadioButton`s. `autonomous_radio.isEnabled()` is `False` and its tooltip is exactly `"not implemented"`.
  - `DashboardPage.mode_changed = Signal(str)` emitting one of `"manual"`, `"semi_auto"`, `"autonomous"`, `"simulation"`.
  - `MainWindow._mode: str` — same four values, `"manual"` at construction.

- [ ] **Step 1: Write the failing widget test**

Append to `tests/test_ui_widgets.py`:

```python
def test_dashboard_page_offers_the_four_modes(qtbot):
    page = DashboardPage()
    qtbot.addWidget(page)

    assert page.manual_radio.text() == "Manual"
    assert page.semi_auto_radio.text() == "Semi-autonomous"
    assert page.autonomous_radio.text() == "Autonomous"
    assert page.simulation_radio.text() == "Simulation"
    assert page.manual_radio.isChecked() is True


def test_autonomous_is_present_but_cannot_be_selected(qtbot):
    # Present so the operator can see the fourth mode exists and is not
    # hidden from them; disabled because nothing behind it is built. A
    # missing button would read as "this project has three modes"; an
    # enabled one would read as "try it".
    page = DashboardPage()
    qtbot.addWidget(page)

    assert page.autonomous_radio.isEnabled() is False
    assert page.autonomous_radio.toolTip() == "not implemented"


def test_each_radio_emits_its_own_mode_name(qtbot):
    page = DashboardPage()
    qtbot.addWidget(page)
    modes = []
    page.mode_changed.connect(modes.append)

    page.semi_auto_radio.setChecked(True)
    page.simulation_radio.setChecked(True)
    page.manual_radio.setChecked(True)

    assert modes == ["semi_auto", "simulation", "manual"]
```

`DashboardPage` is already imported at the top of that file — check with `grep -n "^from\|^import" tests/test_ui_widgets.py` and add `from ground_station.ui.dashboard_page import DashboardPage` only if it is not there.

- [ ] **Step 2: Run the test to verify it fails**

Run: `.venv/bin/pytest tests/test_ui_widgets.py -q -k "four_modes or autonomous or its_own_mode"`
Expected: FAIL with `AttributeError: 'DashboardPage' object has no attribute 'autonomous_radio'`.

- [ ] **Step 3: Rewrite the mode row in `dashboard_page.py`**

Replace lines 13-16 (the `mode_changed` docstring and declaration):

```python
    # Emits "manual", "semi_auto", "autonomous" or "simulation". The switch
    # selects a view source and nothing else - the twist keeps reaching the
    # rover in every mode, so this is never a control-path change wearing a
    # view-change's clothes.
    mode_changed = Signal(str)
```

Replace lines 29-44 (from `self.manual_radio = ...` through the `mode_row.addStretch()`):

```python
        self.manual_radio = QRadioButton("Manual")
        self.semi_auto_radio = QRadioButton("Semi-autonomous")
        self.autonomous_radio = QRadioButton("Autonomous")
        self.simulation_radio = QRadioButton("Simulation")
        # One list, in display order, rather than four scattered checks: the
        # emitted name and the button are declared together, so adding a
        # mode cannot leave a button that emits nothing.
        self._modes = [
            (self.manual_radio, "manual"),
            (self.semi_auto_radio, "semi_auto"),
            (self.autonomous_radio, "autonomous"),
            (self.simulation_radio, "simulation"),
        ]
        # Shown, not hidden: the operator should be able to see that a fourth
        # mode exists and is not built. Disabled and labelled, so nobody
        # discovers that by clicking it mid-drive.
        self.autonomous_radio.setEnabled(False)
        self.autonomous_radio.setToolTip("not implemented")

        self._mode_group = QButtonGroup(self)
        mode_row = QHBoxLayout()
        mode_row.addWidget(mode_label)
        for radio, _ in self._modes:
            radio.setStyleSheet(f"color: {theme.TEXT};")
            self._mode_group.addButton(radio)
            radio.toggled.connect(self._on_mode_toggled)
            mode_row.addWidget(radio)
        mode_row.addStretch()
        self.manual_radio.setChecked(True)
```

Note the ordering: `setChecked(True)` comes **after** the `toggled` connections, so building the page emits `mode_changed("manual")` once. That is harmless (`MainWindow` connects to the signal after the page is constructed, so nothing is listening yet) and it keeps the wiring in one loop.

Replace `_on_mode_toggled` (lines 57-63):

```python
    def _on_mode_toggled(self, checked: bool) -> None:
        if not checked:
            # Each radio's toggled fires twice on a switch (the one turning
            # off, then the one turning on) - only the "turning on" edge
            # names the mode we are entering.
            return
        for radio, mode in self._modes:
            if radio.isChecked():
                self.mode_changed.emit(mode)
                return
```

- [ ] **Step 4: Run the widget test to verify it passes**

Run: `.venv/bin/pytest tests/test_ui_widgets.py -q`
Expected: PASS, all tests in the file.

- [ ] **Step 5: Write the failing window tests for the renamed and new modes**

In `tests/test_main_window.py`, rename the three existing mode tests so they say `simulation` where they mean the dead-reckoned sim, and add the new ones. Replace `test_entering_semi_auto_stops_rover_video_and_switches_port` (line 481) and `test_leaving_semi_auto_returns_to_the_rover_camera` (line 494) with:

```python
def test_entering_simulation_stops_rover_video_and_switches_port(qtbot):
    window, _ = make_window(qtbot)

    window.dashboard_page.mode_changed.emit("simulation")

    assert window._mode == "simulation"
    assert window.dashboard_page.video_panel.receiver.port == 5601
    assert window.dashboard_page.video_panel.dead_reckoning is True
    # The rover keeps being driven - only its camera is turned off, to spare
    # the link while nobody is looking at it.
    request = _last_video_request()
    assert request["enable"] is False


def test_leaving_simulation_returns_to_the_rover_camera(qtbot, monkeypatch):
    # Asserting the port and the marker alone passed for as long as the
    # resume request was missing entirely: the local receiver comes back on
    # 5600 by itself, so the port flipping back proves only that this laptop
    # is listening - not that anything is sending.
    window, _ = make_window(qtbot)
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window._on_stream_requested(True)
    window.dashboard_page.mode_changed.emit("simulation")

    window.dashboard_page.mode_changed.emit("manual")

    assert window.dashboard_page.video_panel.receiver.port == 5600
    assert window.dashboard_page.video_panel.dead_reckoning is False
    request = _last_video_request()
    assert request["enable"] is True
    assert request["port"] == 5600
    assert request["host"] == "10.20.30.40"


def test_semi_auto_shows_the_simulation_without_the_dead_reckoning_marker(qtbot):
    # Semi-autonomous shows the same Gazebo stream on the same port, but the
    # rover in it is placed by localisation, so the DEAD RECKONING warning
    # would be a lie. What replaces it is the localisation marker (Task 3).
    window, _ = make_window(qtbot)

    window.dashboard_page.mode_changed.emit("semi_auto")

    assert window._mode == "semi_auto"
    assert window.dashboard_page.video_panel.receiver.port == 5601
    assert window.dashboard_page.video_panel.dead_reckoning is False
    assert _last_video_request()["enable"] is False


def test_autonomous_changes_nothing_because_nothing_is_built(qtbot):
    # The radio is disabled so this cannot happen from the UI. If the signal
    # is emitted anyway (a test, a future caller), the window must not put
    # the panel into a half-configured state - it leaves the view alone and
    # says so on stderr.
    window, _ = make_window(qtbot)
    port_before = window.dashboard_page.video_panel.receiver.port

    window.dashboard_page.mode_changed.emit("autonomous")

    assert window._mode == "autonomous"
    assert window.dashboard_page.video_panel.receiver.port == port_before
```

Rename the remaining three semi-auto tests the same way, because they are all about the dead-reckoned simulation as it exists today:
- `test_the_video_toggle_does_not_command_the_rover_in_semi_auto` (line 517) → `..._in_simulation`, emitting `"simulation"`.
- `test_a_rosbridge_drop_does_not_tear_down_the_simulation_view` (line 537) → keep the name, emit `"simulation"`.
- `test_entering_semi_auto_shows_receiving_not_a_stale_rover_word` (line 556) → `test_entering_simulation_shows_receiving_not_a_stale_rover_word`, emit `"simulation"`.
- `test_the_twist_still_reaches_the_rover_in_semi_auto` (line 574) → keep the name and the `"semi_auto"` emit: it is exactly as true in the new mode, and it is the mode where an operator might most expect the rover to have been handed over.

Add one more, so the same guarantee is pinned for the renamed mode:

```python
def test_the_twist_still_reaches_the_rover_in_simulation(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.2))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window.dashboard_page.mode_changed.emit("simulation")
    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages[-1] == {
        "linear": {"x": 0.4, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.2},
    }
```

- [ ] **Step 6: Run the window tests to verify they fail**

Run: `.venv/bin/pytest tests/test_main_window.py -q`
Expected: FAIL. `test_entering_simulation_stops_rover_video_and_switches_port` fails because `_on_mode_changed("simulation")` falls into the `else` branch and points the panel at port 5600; `test_semi_auto_shows_the_simulation_without_the_dead_reckoning_marker` fails on `dead_reckoning is False`.

- [ ] **Step 7: Rewrite `_on_mode_changed` in `main_window.py`**

Replace the whole method (lines 331-365):

```python
    def _on_mode_changed(self, mode: str) -> None:
        """Switches the panel's view source only. The twist keeps reaching
        the rover in every mode - driving stays on the gamepad/rosbridge
        path (_poll_gamepad), untouched here - because a mode switch that
        quietly changed what is being driven would be a control change
        wearing a view change's clothes.

        The two simulation modes show the same stream on the same port and
        differ in one thing: where the rover in the picture comes from.
        `simulation` integrates the commanded twist and says so in orange;
        `semi_auto` is placed by the rover's own /localization/pose, so the
        DEAD RECKONING warning would be false and the localisation marker
        takes its place.
        """
        panel = self.dashboard_page.video_panel
        self._mode = mode

        if mode == "autonomous":
            # Unreachable from the UI (the radio is disabled). Reached only
            # if something emits the signal directly, and then the honest
            # thing is to leave the view exactly as it is rather than
            # half-configure a panel for a mode with nothing behind it.
            print("ground_station: autonomous mode is not implemented",
                  file=sys.stderr)
            return

        if mode in ("simulation", "semi_auto"):
            # Stop the rover's camera: nobody is looking at it, and the
            # field link is the scarce resource. The rover keeps being
            # driven.
            self._request_rover_video(False)
            panel.set_source("simulation", SIM_VIDEO_PORT,
                             dead_reckoning=(mode == "simulation"),
                             reports_remote_status=False,
                             show_localization=(mode == "semi_auto"))
            if mode == "semi_auto":
                panel.set_localization_status(self._localization_status)
            return

        panel.set_source("zed front left", self.video_port)
        # Entering a simulation mode told the rover to stop streaming.
        # Leaving must tell it to start again, or the mode is a one-way door
        # for the live camera: set_source restarts the local receiver on 5600
        # so this laptop listens, but the rover was told to stop and never
        # told otherwise, so no frames ever come. What the operator then sees
        # is worse than nothing - stop_receiver has reset the rover state to
        # "stopped", so the panel shows a dim, permanent STOPPED over a black
        # picture, and the "rover says streaming but nothing arrives" branch
        # cannot fire to explain it.
        #
        # Only when the panel is actually receiving: asking the rover to
        # start streaming to a port nothing is listening on is the same waste
        # of the field link that _on_stream_requested guards against. A
        # failed request reports itself on the panel via _request_rover_video
        # rather than doing nothing quietly.
        if panel.streaming:
            self._request_rover_video(True)
```

`set_source`'s `show_localization` keyword and `set_localization_status` land in Task 3; `self._localization_status` lands in Task 4. To keep this task's tests green on their own, add the two stubs now — Task 3 and Task 4 replace them with the real thing:

In `video_panel.py`, extend the `set_source` signature and store the flag (the marker itself is Task 3):

```python
    def set_source(self, name: str, port: int, *, dead_reckoning: bool = False,
                   reports_remote_status: bool = True,
                   show_localization: bool = False) -> None:
```
and inside it, next to `self._dead_reckoning = dead_reckoning`:
```python
        self._show_localization = show_localization
```
with `self._show_localization = False` and `self._localization_status: dict | None = None` added beside `self._dead_reckoning = False` in `__init__` (line 44), and

```python
    def set_localization_status(self, status: dict | None) -> None:
        """The rover's localisation health, as parsed from
        /localization/status. Stored whatever the mode: the panel decides
        whether it is on screen (set_source's show_localization), so the
        window can forward every message without knowing the mode."""
        self._localization_status = status
        self._refresh_title()
```

In `main_window.py.__init__`, next to `self._mode = "manual"` (line 43):

```python
        # The last /localization/status seen, or None if none has arrived.
        # Kept here as well as on the panel so entering semi-autonomous can
        # show the current state immediately instead of a blank marker until
        # the next 2 Hz status message.
        self._localization_status: dict | None = None
```

- [ ] **Step 8: Fix the disconnect guard for the renamed mode**

`_on_connection_changed` (line 241) reads `if self._mode != "semi_auto":`. Both simulation modes show a local sender that has never heard of rosbridge, so both must survive a rosbridge blip. Replace with:

```python
            if self._mode not in ("semi_auto", "simulation"):
```

- [ ] **Step 9: Run the whole suite**

Run: `.venv/bin/pytest tests/ -q`
Expected: PASS. If `test_a_rosbridge_drop_does_not_tear_down_the_simulation_view` fails, Step 8 was skipped.

- [ ] **Step 10: Commit**

```bash
git add ground_station/ui/dashboard_page.py ground_station/ui/main_window.py \
        ground_station/ui/video_panel.py tests/test_ui_widgets.py tests/test_main_window.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Give the dashboard four modes and free semi_auto for the localised view

Renames today's dead-reckoned simulation to the Simulation mode it always
was, adds a disabled Autonomous button so the operator can see the mode
exists and is not built, and points Semi-autonomous at the same stream
without the DEAD RECKONING marker - the rover in that picture is about to
be placed by localisation instead."
```

---

### Task 2: Semi-autonomous refuses the rover's camera, on the panel

The spec: in `semi_auto`, `_on_stream_requested` "does nothing but say `no camera stream in semi-autonomous mode` on the panel". Refusing silently is not an option — the operator pressed a button and something has to answer.

The panel's existing failure line cannot carry this. `apply_status({"state": "failed", ...})` is only rendered by `_refresh_status`, and in the simulation modes the panel is in `reports_remote_status=False`, so `_refresh_local_only_status` runs instead and ignores the rover state entirely. The refusal therefore needs its own path, and it must be **time-boxed**: sticky forever would hide the one fact this mode's operator needs (whether frames are still arriving), and clearing on the next 33 ms poll would make it invisible.

**Files:**
- Modify: `ground_station/ui/video_panel.py:30-34` (constructor), `:114-136` (`set_source`), `:265-331` (`_refresh_status`), `:332-362` (`_refresh_local_only_status`)
- Modify: `ground_station/ui/main_window.py:306-326` (`_on_stream_requested`)
- Test: `tests/test_video_panel.py`, `tests/test_main_window.py`

**Interfaces:**
- Consumes: `MainWindow._mode` from Task 1.
- Produces:
  - `VideoPanel.refuse_stream(reason: str, now: float | None = None) -> None` — puts `reason` verbatim on the status line in `theme.ACCENT` for `VideoPanel.refusal_seconds` seconds, leaves `streaming` and the receiver untouched, and resets the toggle button's text and `_toggle_requested` to match `streaming` so the next click is not a reversal of a request that was never honoured.
  - `VideoPanel(refusal_seconds: float = 5.0)` — new keyword argument, and `self.refusal_seconds` attribute.
  - `MainWindow.SEMI_AUTO_REFUSAL: str = "no camera stream in semi-autonomous mode"`.

- [ ] **Step 1: Write the failing panel tests**

Append to `tests/test_video_panel.py`:

```python
def test_refuse_stream_says_why_on_the_status_line(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    panel.set_source("simulation", 5601, reports_remote_status=False,
                     show_localization=True)

    panel.refuse_stream("no camera stream in semi-autonomous mode", now=100.0)

    assert panel.status_label.text() == "no camera stream in semi-autonomous mode"
    assert theme.ACCENT in panel.status_label.styleSheet()


def test_a_refusal_leaves_the_view_exactly_as_it_was(qtbot):
    # "Does nothing but say so": the simulation stream on screen is not the
    # rover's camera and has nothing to do with the request being refused.
    # Stopping it here would punish the operator for asking a question.
    receiver = FakeReceiver(frame=bytes(4 * 2 * 3))
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)
    panel.set_streaming(True)

    panel.refuse_stream("no camera stream in semi-autonomous mode", now=100.0)

    assert panel.streaming is True
    assert receiver.stopped is False
    # The click flipped the button to "Stop video" on the way in; a refused
    # request must put it back where reality is, or the next click sends the
    # opposite of what the operator means.
    assert panel.toggle_button.text() == "Stop video"


def test_a_refusal_is_shown_for_a_while_and_then_gets_out_of_the_way(qtbot):
    # Sticky forever would bury the one fact this mode's operator needs -
    # whether frames are still arriving. Cleared on the next 33 ms poll it
    # would never be read. Time-boxed is the only honest option.
    receiver = FakeReceiver(frame=bytes(4 * 2 * 3))
    panel = VideoPanel(receiver=receiver, refusal_seconds=5.0)
    qtbot.addWidget(panel)
    panel.set_source("simulation", 5601, reports_remote_status=False)
    panel.set_streaming(True)
    panel.refuse_stream("no camera stream in semi-autonomous mode", now=100.0)

    panel._poll_frame(now=104.9)
    assert panel.status_label.text() == "no camera stream in semi-autonomous mode"

    panel._poll_frame(now=105.1)
    assert panel.status_label.text() == "RECEIVING"


def test_a_mode_change_clears_a_standing_refusal(qtbot):
    # The refusal belongs to the mode that issued it. Carrying it into the
    # manual view, where the rover's camera is exactly what is on screen,
    # would be a lie with a five-second fuse.
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    panel.set_source("simulation", 5601, reports_remote_status=False)
    panel.refuse_stream("no camera stream in semi-autonomous mode", now=100.0)

    panel.set_source("zed front left", 5600)

    assert panel.status_label.text() == "VIDEO OFF"
```

- [ ] **Step 2: Run the panel tests to verify they fail**

Run: `.venv/bin/pytest tests/test_video_panel.py -q -k refus`
Expected: FAIL with `AttributeError: 'VideoPanel' object has no attribute 'refuse_stream'`.

- [ ] **Step 3: Implement the refusal in `video_panel.py`**

Constructor signature (line 30) gains the keyword:

```python
    def __init__(self, receiver=None, parent=None, poll_interval_ms: int = 33,
                 no_frame_after_seconds: float = 2.0,
                 refusal_seconds: float = 5.0):
```

and in the body, next to `self.no_frame_after_seconds = ...`:

```python
        self.refusal_seconds = refusal_seconds
        # A refusal is neither a rover claim nor a local fact, so it does not
        # fit either status path - it is an answer to a button press. Held
        # for refusal_seconds and then dropped, so the ordinary status line
        # comes back on its own.
        self._refusal: str | None = None
        self._refusal_until = 0.0
```

Add the method, next to `apply_status`:

```python
    def refuse_stream(self, reason: str, now: float | None = None) -> None:
        """Answers a stream request that will not be honoured.

        Nothing about the view changes: the source on screen was not what
        was requested, and stopping it would punish the operator for
        asking. Only the button is put back where reality is - the click
        already flipped _toggle_requested, and leaving it flipped would make
        the next click send the opposite of what the operator means.
        """
        now = monotonic() if now is None else now
        self._refusal = reason
        self._refusal_until = now + self.refusal_seconds
        self._toggle_requested = self._streaming
        self.toggle_button.setText("Stop video" if self._streaming else "Start video")
        self._refresh_status(now)
```

`_refresh_status` (line 265) now resolves `now` first and handles the refusal before either status path:

```python
    def _refresh_status(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now

        if self._refusal is not None:
            if now < self._refusal_until:
                self.status_label.setText(self._refusal)
                self.status_label.setStyleSheet(
                    f"color: {theme.ACCENT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
                return
            self._refusal = None

        if not self._reports_remote_status:
            self._refresh_local_only_status(now)
            return
```

The rest of `_refresh_status` is unchanged **except** that its own `now = monotonic() if now is None else now` (line 316) is now redundant — delete that one line. Do the same in `_refresh_local_only_status`: change its signature to `def _refresh_local_only_status(self, now: float) -> None:` and delete its `now = monotonic() if now is None else now` (line 351).

In `set_source`, clear the refusal alongside the other per-source state (next to `self._show_localization = show_localization`):

```python
        self._refusal = None
```

- [ ] **Step 4: Run the panel tests to verify they pass**

Run: `.venv/bin/pytest tests/test_video_panel.py -q`
Expected: PASS, whole file.

- [ ] **Step 5: Write the failing window test**

Append to `tests/test_main_window.py`:

```python
def test_the_video_toggle_is_refused_in_semi_auto_with_the_reason_on_the_panel(qtbot,
                                                                               monkeypatch):
    # Semi-autonomous exists so the operator drives on the Gazebo view and
    # the field link carries no video at all. A toggle here must not command
    # the rover's camera, must not start a local receiver pointed at a
    # camera that is off, and must not be silent about either.
    window, _ = make_window(qtbot)
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/video_request")
    requests_before = len(topic.published_messages)

    window.dashboard_page.video_panel.toggle_button.click()

    assert len(topic.published_messages) == requests_before
    assert window.dashboard_page.video_panel.status_label.text() == (
        "no camera stream in semi-autonomous mode")


def test_the_semi_auto_refusal_does_not_start_a_local_receiver(qtbot):
    # The simulation's own stream is started by the mode switch, not by this
    # button - and in semi-auto the button must move nothing at all.
    window, _ = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    panel = window.dashboard_page.video_panel
    streaming_before = panel.streaming

    window._on_stream_requested(True)

    assert panel.streaming is streaming_before
```

- [ ] **Step 6: Run the window test to verify it fails**

Run: `.venv/bin/pytest tests/test_main_window.py -q -k refus`
Expected: FAIL — the current `_on_stream_requested` treats `semi_auto` as the sim mode and calls `panel.set_streaming(enable)`, so the status line reads `RECEIVING`/`NO FRAMES`, not the refusal.

- [ ] **Step 7: Split `_on_stream_requested` by mode**

In `main_window.py`, above the class, next to `SIM_VIDEO_PORT` (line 18):

```python
# The exact words the panel shows when the operator asks for the rover's
# camera in semi-autonomous mode. One constant, because the test asserts on
# it and the spec fixes the wording.
SEMI_AUTO_REFUSAL = "no camera stream in semi-autonomous mode"
```

and set `MainWindow.SEMI_AUTO_REFUSAL = SEMI_AUTO_REFUSAL` as a class attribute so callers and tests can reach it through the window.

Replace `_on_stream_requested` (lines 306-326):

```python
    def _on_stream_requested(self, enable: bool) -> None:
        panel = self.dashboard_page.video_panel

        if self._mode == "semi_auto":
            # The whole point of this mode is that the field link carries no
            # video: the operator drives on the Gazebo view, placed by
            # localisation. Commanding the rover's camera would undo that;
            # toggling the local simulation receiver would blank the only
            # picture the operator has. So: nothing moves, and the panel
            # says why rather than swallowing the press.
            panel.refuse_stream(SEMI_AUTO_REFUSAL)
            return

        if self._mode == "simulation":
            # The toggle acts on whichever source the panel is actually
            # showing, and here that is the simulation - a local process
            # with no control plane at all, which streams whenever it runs
            # (by design: it is on this same laptop, so there is nothing to
            # ask). Commanding the rover's camera from here would push
            # 800 kbps of H.264 across the field link to port 5600 where
            # nothing is listening, for as long as the mode lasts.
            panel.set_streaming(enable)
            return

        if not self._request_rover_video(enable):
            return
        # The local receiver follows our own intent, not the rover's answer:
        # on disable it must stop regardless of whether the rover ever
        # replies, so a dead link cannot leave a stream pointed at us.
        panel.set_streaming(enable)
```

- [ ] **Step 8: Run the whole suite**

Run: `.venv/bin/pytest tests/ -q`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add ground_station/ui/video_panel.py ground_station/ui/main_window.py \
        tests/test_video_panel.py tests/test_main_window.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Refuse the rover's camera in semi-autonomous mode, on the panel

The mode exists so the field link carries no video. A toggle there now
moves nothing at all and answers with 'no camera stream in semi-autonomous
mode' for five seconds, after which the ordinary status line comes back so
the operator can still see whether the Gazebo stream is arriving."
```

---

### Task 3: The localisation marker on the panel

The spec: in semi-autonomous the panel's marker shows the localisation status — `LOCALISED`, `SEARCHING … 4 s`, or `LOCALISATION OFF` — "in the same colours the panel uses today". Those colours are `theme.OK` for a corroborated good state (`STREAMING`, `RECEIVING`) and `theme.ACCENT` for everything that should stop an operator (`FAILED`, `NO FRAMES`, and the existing `DEAD RECKONING` title marker). It goes in the same place as the dead-reckoning marker — the title — because that marker's whole argument applies here unchanged: it is the operator's only defence against trusting a synthetic view of the machine they are driving, and it has to sit where they are already looking.

**Files:**
- Modify: `ground_station/ui/video_panel.py:137-151` (`_refresh_title`), plus the module-level marker function
- Test: `tests/test_video_panel.py`

**Interfaces:**
- Consumes: `VideoPanel._show_localization` and `VideoPanel.set_localization_status(status)` from Task 1's stubs.
- Produces:
  ```python
  def localization_marker(status: dict | None) -> tuple[str, str]:
      """(marker text, colour) for a /localization/status payload."""
  ```
  in `ground_station/ui/video_panel.py`. `status` is the dict `RosBridgeClient._parse_localization_status` produces (Task 4) or `None`. The mapping, exhaustively:

  | `status` | marker | colour |
  |---|---|---|
  | `None` | `NO LOCALISATION STATUS` | `theme.ACCENT` |
  | `state == "OK"` | `LOCALISED` | `theme.OK` |
  | `state == "SEARCHING"`, `seconds_since_ok` a number | `SEARCHING … 4 s` | `theme.ACCENT` |
  | `state == "SEARCHING"`, `seconds_since_ok` `None` | `SEARCHING` | `theme.ACCENT` |
  | `state == "OFF"` | `LOCALISATION OFF` | `theme.ACCENT` |
  | any other `state` | `LOCALISATION <STATE>` | `theme.ACCENT` |

- [ ] **Step 1: Write the failing marker tests**

Append to `tests/test_video_panel.py`:

```python
from ground_station.ui.video_panel import VideoPanel, localization_marker


def test_the_marker_reads_localised_when_the_rover_knows_where_it_is():
    text, colour = localization_marker(
        {"state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
         "distance_travelled": 12.5, "mount_offset_verified": True, "detail": ""})

    assert text == "LOCALISED"
    assert colour == theme.OK


def test_the_marker_counts_the_seconds_while_searching():
    # The count is the whole content of this state: "searching" alone does
    # not say whether it is a half-second blip over a rut or forty seconds
    # of the rover being lost.
    text, colour = localization_marker(
        {"state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
         "distance_travelled": 12.5, "mount_offset_verified": True, "detail": ""})

    assert text == "SEARCHING … 4 s"
    assert colour == theme.ACCENT


def test_searching_without_a_count_still_says_searching():
    text, _ = localization_marker(
        {"state": "SEARCHING", "seconds_since_ok": None, "source": "zed_vio",
         "distance_travelled": 0.0, "mount_offset_verified": True, "detail": ""})

    assert text == "SEARCHING"


def test_the_marker_says_localisation_is_off():
    text, colour = localization_marker(
        {"state": "OFF", "seconds_since_ok": None, "source": "zed_vio",
         "distance_travelled": 0.0, "mount_offset_verified": True, "detail": ""})

    assert text == "LOCALISATION OFF"
    assert colour == theme.ACCENT


def test_no_status_at_all_is_not_reported_as_localisation_being_off():
    # "The rover told us it is off" and "the rover has told us nothing" are
    # different facts. The second one usually means rosbridge, not the ZED,
    # and sending the operator to the wrong machine costs a rover day.
    text, colour = localization_marker(None)

    assert text == "NO LOCALISATION STATUS"
    assert colour == theme.ACCENT


def test_an_unknown_state_is_shown_rather_than_swallowed():
    text, colour = localization_marker(
        {"state": "wedged", "seconds_since_ok": None, "source": "zed_vio",
         "distance_travelled": 0.0, "mount_offset_verified": False, "detail": ""})

    assert text == "LOCALISATION WEDGED"
    assert colour == theme.ACCENT


def test_the_marker_is_on_the_title_in_semi_auto(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    panel.set_source("simulation", 5601, reports_remote_status=False,
                     show_localization=True)

    panel.set_localization_status(
        {"state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
         "distance_travelled": 1.0, "mount_offset_verified": True, "detail": ""})

    assert panel.title_label.text() == "CAMERA / SIMULATION  -  LOCALISED"
    assert theme.OK in panel.title_label.styleSheet()


def test_the_marker_is_not_shown_for_a_source_that_is_not_localised(qtbot):
    # The dead-reckoned Simulation mode and the rover's own camera have no
    # localisation to report; a marker there would be a claim about a topic
    # that has nothing to do with what is on screen.
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    panel.set_source("simulation", 5601, dead_reckoning=True,
                     reports_remote_status=False)

    panel.set_localization_status(
        {"state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
         "distance_travelled": 1.0, "mount_offset_verified": True, "detail": ""})

    assert panel.title_label.text() == (
        "CAMERA / SIMULATION  -  DEAD RECKONING, NO LOCALISATION")


def test_entering_semi_auto_before_any_status_says_so_rather_than_nothing(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)

    panel.set_source("simulation", 5601, reports_remote_status=False,
                     show_localization=True)

    assert panel.title_label.text() == "CAMERA / SIMULATION  -  NO LOCALISATION STATUS"
```

Replace the file's existing `from ground_station.ui.video_panel import VideoPanel` import (line 2) with the two-name import above rather than adding a second import line.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv/bin/pytest tests/test_video_panel.py -q -k "marker or localis"`
Expected: FAIL with `ImportError: cannot import name 'localization_marker'`.

- [ ] **Step 3: Implement the marker**

In `ground_station/ui/video_panel.py`, after the imports:

```python
def localization_marker(status: dict | None) -> tuple[str, str]:
    """The marker text and colour for a /localization/status payload.

    Colours are the panel's existing two: theme.OK for the one state that
    means the picture can be trusted, theme.ACCENT for every state that
    should stop the operator - which is the same colour FAILED, NO FRAMES
    and the DEAD RECKONING title marker already use, for the same reason.

    None is not the same as OFF and is not shown as OFF: "the rover says
    localisation is off" and "we have heard nothing from the rover" point at
    different machines, and the second one is usually rosbridge.
    """
    if status is None:
        return "NO LOCALISATION STATUS", theme.ACCENT

    state = str(status.get("state", ""))
    if state == "OK":
        return "LOCALISED", theme.OK
    if state == "SEARCHING":
        seconds = status.get("seconds_since_ok")
        if isinstance(seconds, (int, float)):
            # The count is the content: "searching" alone does not say
            # whether this is a blip over a rut or the rover being lost.
            return f"SEARCHING … {seconds:.0f} s", theme.ACCENT
        return "SEARCHING", theme.ACCENT
    if state == "OFF":
        return "LOCALISATION OFF", theme.ACCENT
    # An unrecognised state is shown, not swallowed: a status this panel
    # cannot interpret is itself worth seeing.
    return f"LOCALISATION {state.upper()}", theme.ACCENT
```

Replace `_refresh_title` (lines 137-151):

```python
    def _refresh_title(self) -> None:
        title = f"CAMERA / {self._source_name.upper()}"
        colour = theme.TEXT_DIM
        if self._dead_reckoning:
            # The simulated pose is integrated from commanded twist, so it
            # drifts from the real rover and the picture cannot show it.
            title += "  -  DEAD RECKONING, NO LOCALISATION"
            colour = theme.ACCENT
        elif self._show_localization:
            marker, colour = localization_marker(self._localization_status)
            title += f"  -  {marker}"
        self.title_label.setText(title)
        # This marker is the only defence an operator has against trusting a
        # synthetic view of the real machine they are driving - sitting in
        # the same muted colour as ordinary chrome ("CAMERA / ZED FRONT
        # LEFT") above a moving picture would never win the operator's
        # attention. The failure colours are the same ones FAILED and the
        # blocked-port warning use, for the same reason: these are warnings.
        self.title_label.setStyleSheet(f"color: {colour}; font-weight: 600; border: none;")
```

The two flags are mutually exclusive by construction: `main_window._on_mode_changed` passes `dead_reckoning=True` only for `simulation` and `show_localization=True` only for `semi_auto`. The `elif` records that rather than relying on it.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv/bin/pytest tests/test_video_panel.py -q`
Expected: PASS, whole file.

- [ ] **Step 5: Commit**

```bash
git add ground_station/ui/video_panel.py tests/test_video_panel.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Show the rover's localisation health on the panel in semi-autonomous mode

The marker takes the place the DEAD RECKONING warning holds in Simulation
mode - the operator is driving on this picture, so whether the rover in it
is where the real one is belongs where they are already looking. A status
that has never arrived reads as NO LOCALISATION STATUS rather than OFF: the
two point at different machines."
```

---

### Task 4: `ros_client` subscribes to the two localisation topics, and the header reads out the pose

`/localization/status` is a `std_msgs/String` carrying JSON, exactly like `/video_status`, so rosbridge needs no custom type. `/localization/pose` is a `nav_msgs/Odometry` at ~30 Hz and is wanted only for a header readout, so it is subscribed with rosbridge's own `throttle_rate` — 200 ms, i.e. 5 Hz. Throttling at the server is the point: dropping 25 messages a second here rather than on the laptop is 25 messages a second that never cross the field link.

**Files:**
- Modify: `ground_station/ros_client.py:11-15` (signals), `:26-28` (topic handles), `:73-93` (beside the video-status subscription)
- Modify: `ground_station/models.py` (two pure functions)
- Modify: `ground_station/ui/main_window.py:1-13` (imports), `:57-85` (header), `:176-188` (`_connect_to`), `:251-255` (`_check_staleness`), and two new slots
- Test: `tests/test_ros_client.py`, `tests/test_models.py`, `tests/test_main_window.py`

**Interfaces:**
- Consumes: `VideoPanel.set_localization_status` from Task 1/3; `MainWindow._localization_status` from Task 1.
- Produces:
  ```python
  # ground_station/models.py
  def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float
  def pose_readout_from_odometry(message: dict) -> dict
      # -> {"x": float, "y": float, "yaw": float}   (yaw in radians)

  # ground_station/ros_client.py
  LOCALIZATION_POSE_THROTTLE_MS = 200      # 5 Hz, rosbridge-side

  class RosSignals(QObject):
      localization_status_received = Signal(dict)
      localization_pose_received = Signal(dict)

  class RosBridgeClient:
      def subscribe_localization_status(self, topic_name: str = "/localization/status") -> None
      def subscribe_localization_pose(self, topic_name: str = "/localization/pose",
                                      throttle_ms: int = LOCALIZATION_POSE_THROTTLE_MS) -> None
      @staticmethod
      def _parse_localization_status(payload: object) -> dict
          # -> {"state": str, "seconds_since_ok": float | None, "source": str,
          #     "distance_travelled": float, "mount_offset_verified": bool,
          #     "detail": str}
  ```
  and, in `ground_station/ui/main_window.py`:
  ```python
  LOCALIZATION_STATUS_STALE_AFTER_SECONDS = 3.0

  class MainWindow(QMainWindow):
      localization_label: QLabel
      _localization_status: dict | None          # the last status, or None
      _localization_status_at: float | None      # monotonic() when it arrived
      def _on_localization_status(self, status: dict) -> None
      def _on_localization_pose(self, pose: dict) -> None
      def _check_staleness(self, now: float | None = None) -> None
  ```

- [ ] **Step 1: Write the failing pure-model tests**

Append to `tests/test_models.py`:

```python
import math

from ground_station.models import pose_readout_from_odometry, yaw_from_quaternion


def test_yaw_of_the_identity_quaternion_is_zero():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_yaw_of_a_quarter_turn_about_z():
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

    assert yaw == pytest.approx(math.pi / 2)


def test_yaw_is_signed_so_a_right_turn_reads_negative():
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(-math.pi / 6), math.cos(-math.pi / 6))

    assert yaw == pytest.approx(-math.pi / 3)


def test_pose_readout_pulls_x_y_and_yaw_out_of_an_odometry_message():
    message = {
        "header": {"frame_id": "map"},
        "child_frame_id": "base_footprint",
        "pose": {"pose": {
            "position": {"x": 3.25, "y": -1.5, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)},
        }},
    }

    readout = pose_readout_from_odometry(message)

    assert readout["x"] == pytest.approx(3.25)
    assert readout["y"] == pytest.approx(-1.5)
    assert readout["yaw"] == pytest.approx(math.pi / 2)


def test_pose_readout_of_a_truncated_message_reads_as_the_origin_not_a_crash():
    # rosbridge hands over whatever JSON arrived. A malformed message must
    # not take the GUI thread down mid-drive; a readout at the origin is
    # visibly wrong and recoverable, an exception in a Qt slot is not.
    assert pose_readout_from_odometry({}) == {"x": 0.0, "y": 0.0, "yaw": 0.0}
```

`tests/test_models.py` already imports `pytest`? Check with `head -5 tests/test_models.py`; add `import pytest` if it is missing.

- [ ] **Step 2: Run them to verify they fail**

Run: `.venv/bin/pytest tests/test_models.py -q`
Expected: FAIL with `ImportError: cannot import name 'yaw_from_quaternion'`.

- [ ] **Step 3: Implement the two pure functions**

Add `import math` at the top of `ground_station/models.py` and append:

```python
def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading in radians, from a quaternion.

    The standard ZYX extraction, written out rather than pulled from a
    library: this file is deliberately dependency-free (no Qt, no ROS, no
    numpy), and the alternative is a transforms3d dependency for four
    multiplications.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_readout_from_odometry(message: dict) -> dict:
    """x, y and yaw out of a nav_msgs/Odometry as rosbridge delivers it.

    Every level is defended with a default because the message comes off the
    wire as whatever JSON arrived: a truncated one must produce a visibly
    wrong readout at the origin, not an exception inside a Qt slot while the
    operator is driving.
    """
    pose = (message.get("pose") or {}).get("pose") or {}
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    return {
        "x": float(position.get("x", 0.0)),
        "y": float(position.get("y", 0.0)),
        "yaw": yaw_from_quaternion(
            float(orientation.get("x", 0.0)), float(orientation.get("y", 0.0)),
            float(orientation.get("z", 0.0)), float(orientation.get("w", 1.0))),
    }
```

- [ ] **Step 4: Run them to verify they pass**

Run: `.venv/bin/pytest tests/test_models.py -q`
Expected: PASS.

- [ ] **Step 5: Write the failing `ros_client` tests**

First, `FakeTopic` in **both** `tests/test_ros_client.py` (line 4) and `tests/test_main_window.py` (line 9) must accept the throttle keyword, or `subscribe_localization_pose` raises `TypeError` on every construction. The two classes differ slightly today, so both are written out here.

`tests/test_ros_client.py`, replacing `FakeTopic.__init__` (lines 6-13):

```python
    def __init__(self, ros, name, msg_type, **options):
        self.ros = ros
        self.name = name
        self.msg_type = msg_type
        self.callback = None
        self.published_messages = []
        # Whatever roslibpy keywords the client passed - throttle_rate today.
        # Kept rather than swallowed: a subscription's throttle is part of
        # what the client promises, so a test has to be able to assert on it.
        self.options = options
        FakeTopic.instances.append(self)
```

`tests/test_main_window.py`, replacing `FakeTopic.__init__` (lines 10-16):

```python
    def __init__(self, ros, name, msg_type, **options):
        self.name = name
        self.msg_type = msg_type
        self.callback = None
        self.published_messages = []
        self.options = options
        FakeTopic.instances.append(self)
```

Then append to `tests/test_ros_client.py`:

```python
import json

from ground_station.ros_client import LOCALIZATION_POSE_THROTTLE_MS


def test_subscribe_localization_status_emits_the_parsed_json(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_status()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    assert topic.msg_type == "std_msgs/String"

    with qtbot.waitSignal(client.signals.localization_status_received,
                          timeout=1000) as blocker:
        topic.callback({"data": json.dumps({
            "state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
            "distance_travelled": 12.5, "mount_offset_verified": True})})

    assert blocker.args[0] == {
        "state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
        "distance_travelled": 12.5, "mount_offset_verified": True, "detail": ""}


def test_a_malformed_localization_status_reads_as_off_with_the_reason(qtbot):
    # Same reasoning as the video status: a bad payload must not raise
    # inside roslibpy's background thread. OFF is the right fallback state -
    # it is what the panel shows when nothing can be trusted - and the
    # reason is carried so it is not lost.
    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_status()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")

    with qtbot.waitSignal(client.signals.localization_status_received,
                          timeout=1000) as blocker:
        topic.callback({"data": "{not json"})

    assert blocker.args[0]["state"] == "OFF"
    assert "bad status JSON" in blocker.args[0]["detail"]


def test_subscribe_localization_pose_throttles_at_five_hertz(qtbot):
    # The pose is published at ~30 Hz and is wanted here for one header
    # readout. Throttling at the rosbridge server rather than on this laptop
    # is 25 messages a second that never cross the field link.
    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_pose()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")

    assert topic.msg_type == "nav_msgs/Odometry"
    assert topic.options["throttle_rate"] == LOCALIZATION_POSE_THROTTLE_MS
    assert LOCALIZATION_POSE_THROTTLE_MS == 200


def test_subscribe_localization_pose_emits_x_y_and_yaw(qtbot):
    import math

    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_pose()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")

    with qtbot.waitSignal(client.signals.localization_pose_received,
                          timeout=1000) as blocker:
        topic.callback({"pose": {"pose": {
            "position": {"x": 1.5, "y": 2.5, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}}}})

    assert blocker.args[0]["x"] == 1.5
    assert blocker.args[0]["y"] == 2.5
    assert abs(blocker.args[0]["yaw"] - math.pi / 2) < 1e-9
```

- [ ] **Step 6: Run them to verify they fail**

Run: `.venv/bin/pytest tests/test_ros_client.py -q`
Expected: FAIL with `ImportError: cannot import name 'LOCALIZATION_POSE_THROTTLE_MS'`.

- [ ] **Step 7: Implement the subscriptions**

In `ground_station/ros_client.py`, add below the imports:

```python
from ground_station.models import pose_readout_from_odometry

# /localization/pose is published at the ZED wrapper's ~30 Hz and is wanted
# here for one header readout. rosbridge's own throttle_rate (milliseconds)
# drops the rest at the rover, before they cross the field link.
LOCALIZATION_POSE_THROTTLE_MS = 200      # 5 Hz
```

Extend `RosSignals`:

```python
    localization_status_received = Signal(dict)
    localization_pose_received = Signal(dict)
```

In `__init__`, beside the other topic handles:

```python
        self._localization_status_topic = None
        self._localization_pose_topic = None
```

Append the two subscriptions and the parser:

```python
    def subscribe_localization_status(self, topic_name: str = "/localization/status") -> None:
        """The rover's account of its own localisation: OK, SEARCHING or OFF,
        with the seconds since it was last OK.

        JSON in a std_msgs/String, the same convention as /video_status, so
        rosbridge needs no custom message type to discover. Published at 2 Hz
        and on every state change."""
        topic = self._topic_factory(self._ros, topic_name, "std_msgs/String")
        topic.subscribe(lambda msg: self.signals.localization_status_received.emit(
            self._parse_localization_status(msg.get("data", ""))))
        self._localization_status_topic = topic

    def subscribe_localization_pose(
            self, topic_name: str = "/localization/pose",
            throttle_ms: int = LOCALIZATION_POSE_THROTTLE_MS) -> None:
        """The rover's pose, for the header readout only.

        The Gazebo view gets its pose over DDS through sim_bridge, not
        through here - the ground station has no ROS and is not in that
        path. All this subscription feeds is three numbers in the header, so
        it is throttled server-side to 5 Hz."""
        topic = self._topic_factory(self._ros, topic_name, "nav_msgs/Odometry",
                                    throttle_rate=throttle_ms)
        topic.subscribe(lambda msg: self.signals.localization_pose_received.emit(
            pose_readout_from_odometry(msg)))
        self._localization_pose_topic = topic

    @staticmethod
    def _parse_localization_status(payload: object) -> dict:
        """Every field the status JSON carries, with defaults for all of
        them. A payload that will not parse becomes OFF with the reason
        attached: OFF is what the panel shows when nothing can be trusted,
        and losing the reason would send the operator looking at the ZED
        when the fault is in the link."""
        try:
            status = json.loads(payload)
        except (json.JSONDecodeError, TypeError) as exc:
            return _localization_status_failure(f"bad status JSON: {exc}")
        if not isinstance(status, dict):
            return _localization_status_failure("status was not a JSON object")
        seconds = status.get("seconds_since_ok")
        return {
            "state": str(status.get("state", "OFF")),
            "seconds_since_ok": seconds if isinstance(seconds, (int, float)) else None,
            "source": str(status.get("source", "")),
            "distance_travelled": float(status.get("distance_travelled", 0.0) or 0.0),
            "mount_offset_verified": bool(status.get("mount_offset_verified", False)),
            "detail": "",
        }
```

and the module-level helper, above the class:

```python
def _localization_status_failure(detail: str) -> dict:
    """The same shape as a parsed status, so every consumer can read the
    same keys without asking whether this one came off the wire intact."""
    return {"state": "OFF", "seconds_since_ok": None, "source": "",
            "distance_travelled": 0.0, "mount_offset_verified": False,
            "detail": detail}
```

- [ ] **Step 8: Run the client tests to verify they pass**

Run: `.venv/bin/pytest tests/test_ros_client.py -q`
Expected: PASS, whole file.

- [ ] **Step 9: Write the failing window tests for the wiring and the header**

Append to `tests/test_main_window.py`:

```python
def test_connecting_subscribes_to_both_localisation_topics(qtbot):
    window, _ = make_window(qtbot)

    names = [t.name for t in FakeTopic.instances]
    assert "/localization/status" in names
    assert "/localization/pose" in names


def test_a_localisation_status_reaches_the_panel(qtbot):
    window, client = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")

    topic.callback({"data": json.dumps({
        "state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
        "distance_travelled": 12.5, "mount_offset_verified": True})})

    assert window.dashboard_page.video_panel.title_label.text() == (
        "CAMERA / SIMULATION  -  SEARCHING … 4 s")


def test_the_last_status_is_on_screen_the_moment_semi_auto_is_entered(qtbot):
    # /localization/status arrives at 2 Hz. Waiting half a second with a
    # blank marker after a mode switch is half a second of the operator not
    # knowing whether to trust the picture they just switched to.
    window, _ = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    topic.callback({"data": json.dumps({
        "state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
        "distance_travelled": 1.0, "mount_offset_verified": True})})

    window.dashboard_page.mode_changed.emit("semi_auto")

    assert window.dashboard_page.video_panel.title_label.text() == (
        "CAMERA / SIMULATION  -  LOCALISED")


def test_the_header_reads_out_the_pose(qtbot):
    import math

    window, _ = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")

    topic.callback({"pose": {"pose": {
        "position": {"x": 1.5, "y": -2.25, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0,
                        "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}}}})

    assert window.localization_label.text() == "LOC: x 1.50  y -2.25  yaw 90.0°"


def test_the_header_says_so_before_any_pose_arrives(qtbot):
    window, _ = make_window(qtbot)

    assert window.localization_label.text() == "LOC: NO POSE"
```

- [ ] **Step 10: Run them to verify they fail**

Run: `.venv/bin/pytest tests/test_main_window.py -q -k "localis or localiz or header"`
Expected: FAIL — `/localization/status` is not in `FakeTopic.instances`, and `window.localization_label` does not exist.

- [ ] **Step 11: Wire it into `main_window.py`**

Add `import math` beside `import socket` (line 1).

In `__init__`, after `self.connection_label` is built (line 62), add:

```python
        self.localization_label = QLabel("LOC: NO POSE")
        self.localization_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; background-color: {theme.PANEL}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 6px 12px; "
            f"font-family: {theme.MONO_FONT_FAMILY};"
        )
```

and in the header layout (after line 84, `header_layout.addWidget(self.connect_button)`):

```python
        header_layout.addWidget(self.localization_label)
```

In `_connect_to`, add the two signal connections beside the others (after line 180):

```python
        self.ros_client.signals.localization_status_received.connect(
            self._on_localization_status)
        self.ros_client.signals.localization_pose_received.connect(
            self._on_localization_pose)
```

and the two subscribes inside the `try` (after `self.ros_client.subscribe_video_status()`, line 185):

```python
            self.ros_client.subscribe_localization_status()
            self.ros_client.subscribe_localization_pose()
```

Add the two slots, next to `_on_video_status`:

```python
    def _on_localization_status(self, status: dict) -> None:
        """Kept here as well as handed to the panel: entering semi-autonomous
        must show the current state at once rather than a blank marker until
        the next 2 Hz message."""
        self._localization_status = status
        self.dashboard_page.video_panel.set_localization_status(status)

    def _on_localization_pose(self, pose: dict) -> None:
        """Three numbers in the header, at 5 Hz. This is the whole of what
        the ground station does with the pose - the rover in the Gazebo view
        is placed over DDS by sim_ik_node, not from here, because this
        process has no ROS and is not in that path."""
        self.localization_label.setText(
            f"LOC: x {pose['x']:.2f}  y {pose['y']:.2f}  "
            f"yaw {math.degrees(pose['yaw']):.1f}°")
```

- [ ] **Step 12: Write the failing test for a status that stops arriving**

The spec's error handling: when the rover becomes unreachable, "status goes
stale and the panel says so". `/localization/status` is published at 2 Hz, so
three seconds of silence is six missed messages — that is the rover gone, not
a hiccup, and the marker must stop asserting a health nobody has confirmed
for three seconds.

Append to `tests/test_main_window.py`:

```python
def test_a_localisation_status_that_stops_arriving_stops_being_asserted(qtbot):
    # A marker reading LOCALISED because that is what the rover said before
    # the link died is the worst failure this panel has: the operator is
    # driving on a picture, and the marker is the one thing telling them
    # whether to trust it.
    window, _ = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    topic.callback({"data": json.dumps({
        "state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
        "distance_travelled": 1.0, "mount_offset_verified": True})})
    assert window.dashboard_page.video_panel.title_label.text().endswith("LOCALISED")

    window._check_staleness(now=window._localization_status_at + 3.5)

    assert window.dashboard_page.video_panel.title_label.text() == (
        "CAMERA / SIMULATION  -  NO LOCALISATION STATUS")
    assert window._localization_status is None


def test_a_fresh_status_is_not_called_stale(qtbot):
    window, _ = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    topic.callback({"data": json.dumps({
        "state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
        "distance_travelled": 1.0, "mount_offset_verified": True})})

    window._check_staleness(now=window._localization_status_at + 1.0)

    assert window.dashboard_page.video_panel.title_label.text().endswith("LOCALISED")
```

- [ ] **Step 13: Make the status go stale**

`_check_staleness` already runs every 500 ms (`_staleness_timer`, `main_window.py:113-115`) and already exists to answer exactly this question about the drive readouts. Give it the second question rather than adding a second timer.

Beside the other module constants in `main_window.py`:

```python
# /localization/status arrives at 2 Hz, so three seconds is six missed
# messages: the rover is gone, not hiccuping. After that the marker stops
# asserting a health nobody has confirmed since - it reads NO LOCALISATION
# STATUS, which is what is actually true.
LOCALIZATION_STATUS_STALE_AFTER_SECONDS = 3.0
```

In `__init__`, beside `self._localization_status`:

```python
        # monotonic() when that status arrived, or None if none has.
        self._localization_status_at: float | None = None
```

In `_on_localization_status`, record the arrival:

```python
        self._localization_status_at = monotonic()
```
(add `from time import monotonic` to the imports at the top of `main_window.py`).

Extend `_check_staleness` (line 251):

```python
    def _check_staleness(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        elapsed = self.drive_state.seconds_since_last(now)
        if elapsed is not None and elapsed > self.stale_after_seconds:
            self.dashboard_page.drive_card.mark_stale()
            self.drive_detail_page.mark_stale()

        # The rover unreachable is the case this catches: sim_bridge stops
        # receiving, the Gazebo rover holds still on its own, and this is
        # the panel saying so rather than leaving LOCALISED on screen
        # because that is what the rover said before the link died.
        if (self._localization_status_at is not None
                and now - self._localization_status_at
                > LOCALIZATION_STATUS_STALE_AFTER_SECONDS):
            self._localization_status = None
            self._localization_status_at = None
            self.dashboard_page.video_panel.set_localization_status(None)
```

`DriveState.seconds_since_last` already takes an optional `now` (`models.py:40`), so passing it through changes nothing for the existing tests.

- [ ] **Step 14: Run the whole suite**

Run: `.venv/bin/pytest tests/ -q`
Expected: PASS.

- [ ] **Step 15: Commit**

```bash
git add ground_station/ros_client.py ground_station/models.py \
        ground_station/ui/main_window.py tests/test_ros_client.py \
        tests/test_models.py tests/test_main_window.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Subscribe to the rover's localisation status and pose over rosbridge

The status drives the panel's marker; the pose feeds one header readout and
is throttled to 5 Hz at the rosbridge server, so the other 25 messages a
second never cross the field link. A status that stops arriving for three
seconds - six missed messages at 2 Hz - stops being asserted, so an
unreachable rover cannot leave LOCALISED on screen over a picture nobody
has confirmed. Nothing here imports rclpy: the Gazebo view gets its pose
over DDS, not through this process."
```

---

### Task 5: A fake pose that walks a square — in the mock rosbridge and on a real domain

The spec's end-to-end check is "`mock/ros_bridge.py` gains a fake `/localization/pose` that walks a square; Gazebo's rover walks the same square". Those are two different transports: the mock rosbridge is a websocket server and puts nothing on a DDS graph, while `sim_bridge` reads DDS. "The same square" is therefore only true if both read the same code, so the walk is one pure function in `mock/square_walk.py` and both fixtures import it.

**Files:**
- Create: `mock/square_walk.py`, `mock/fake_localization.py`, `tests/test_square_walk.py`
- Modify: `mock/ros_bridge.py:18-24` (imports and constants), `:57-63` (`_handle_publish`), `:83-99` (factory and `main`)

**Interfaces:**
- Produces:
  ```python
  # mock/square_walk.py   (stdlib only - imported from the venv and from ROS's python3)
  def square_pose(elapsed: float, side: float = 4.0,
                  speed: float = 0.5) -> tuple[float, float, float]
      # -> (x, y, yaw) in metres and radians, anticlockwise from the origin

  def odometry_message(x: float, y: float, yaw: float) -> dict
      # a nav_msgs/Odometry shaped as rosbridge delivers it

  def status_payload(state: str, seconds_since_ok: float | None,
                     distance_travelled: float) -> str
      # the /localization/status JSON string
  ```
  `mock/ros_bridge.py --localization-state OK|SEARCHING|OFF` (default `OK`).
  `mock/fake_localization.py --state OK|SEARCHING|OFF --side 4.0 --speed 0.5` — an `rclpy` node publishing the same square on the current `ROS_DOMAIN_ID`.

- [ ] **Step 1: Write the failing walk tests**

`tests/test_square_walk.py`:

```python
"""The square the fake localisation walks.

Loaded by path rather than imported: mock/ is not a package, and a plain
`import square_walk` would depend on the working directory.
"""

import importlib.util
import math
import pathlib

import pytest

_PATH = pathlib.Path(__file__).resolve().parent.parent / "mock" / "square_walk.py"
_spec = importlib.util.spec_from_file_location("square_walk", _PATH)
square_walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(square_walk)


def test_the_walk_starts_at_the_origin_facing_along_x():
    assert square_walk.square_pose(0.0) == (0.0, 0.0, 0.0)


def test_the_first_leg_runs_along_x_at_the_given_speed():
    x, y, yaw = square_walk.square_pose(4.0, side=4.0, speed=0.5)

    assert x == pytest.approx(2.0)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_each_corner_is_a_quarter_turn():
    # side 4 m at 0.5 m/s is 8 s a leg.
    assert square_walk.square_pose(8.0) == pytest.approx((4.0, 0.0, math.pi / 2))
    assert square_walk.square_pose(16.0) == pytest.approx((4.0, 4.0, math.pi))
    assert square_walk.square_pose(24.0) == pytest.approx((0.0, 4.0, -math.pi / 2))


def test_the_square_closes_and_repeats():
    # A closed loop is what makes this fixture useful: whatever is watching
    # the far end can be left running and must keep coming back to the
    # origin, so a drift or an accumulating offset shows up as the picture
    # walking away rather than as a number someone has to read.
    assert square_walk.square_pose(32.0) == pytest.approx((0.0, 0.0, 0.0))
    assert square_walk.square_pose(36.0) == pytest.approx(square_walk.square_pose(4.0))


def test_a_shorter_faster_square_scales_both_ways():
    x, y, yaw = square_walk.square_pose(1.0, side=2.0, speed=1.0)

    assert (x, y, yaw) == pytest.approx((1.0, 0.0, 0.0))


def test_the_odometry_message_is_shaped_the_way_the_ground_station_reads_it():
    from ground_station.models import pose_readout_from_odometry

    message = square_walk.odometry_message(1.5, -2.25, math.pi / 2)

    assert message["header"]["frame_id"] == "map"
    assert message["child_frame_id"] == "base_footprint"
    readout = pose_readout_from_odometry(message)
    assert readout["x"] == pytest.approx(1.5)
    assert readout["y"] == pytest.approx(-2.25)
    assert readout["yaw"] == pytest.approx(math.pi / 2)


def test_the_status_payload_is_what_ros_client_parses():
    import json

    from ground_station.ros_client import RosBridgeClient

    parsed = RosBridgeClient._parse_localization_status(
        square_walk.status_payload("SEARCHING", 4.2, 12.5))

    assert parsed["state"] == "SEARCHING"
    assert parsed["seconds_since_ok"] == 4.2
    assert parsed["source"] == "zed_vio"
    assert parsed["distance_travelled"] == 12.5
    assert parsed["mount_offset_verified"] is True
    assert json.loads(square_walk.status_payload("OK", 0.0, 0.0))["state"] == "OK"
```

- [ ] **Step 2: Run them to verify they fail**

Run: `.venv/bin/pytest tests/test_square_walk.py -q`
Expected: FAIL — `mock/square_walk.py` does not exist, so `spec_from_file_location` returns a loader that raises `FileNotFoundError`.

- [ ] **Step 3: Write `mock/square_walk.py`**

```python
"""A fake rover pose that walks a closed square.

Used by two fixtures that share nothing else: mock/ros_bridge.py serves it
to the ground station over websockets, and mock/fake_localization.py
publishes it on a real DDS domain for sim_bridge and Gazebo. "Gazebo walks
the same square as the panel" is only a real check if both ends read the
same code, which is why this is its own module rather than two similar
loops.

Standard library only, on purpose: the ground station's venv cannot import
ROS, and ROS's python3 is not the venv. This module has to load in both.
"""

import json
import math


def square_pose(elapsed: float, side: float = 4.0,
                speed: float = 0.5) -> tuple[float, float, float]:
    """Where the fake rover is at `elapsed` seconds, and which way it faces.

    Anticlockwise from the origin: +X, +Y, -X, -Y, turning instantly at each
    corner. The turn is not simulated because nothing downstream learns
    anything from it - what this fixture is for is a path that closes, so a
    drift anywhere in the chain shows up as the picture walking away from
    the square instead of as a number somebody has to read.

    Repeats forever: a fixture that stops after one lap is a fixture that
    has to be restarted every 32 seconds during a manual check.
    """
    leg_seconds = side / speed
    lap_seconds = 4.0 * leg_seconds
    t = elapsed % lap_seconds
    leg = int(t // leg_seconds)
    along = (t - leg * leg_seconds) * speed
    if leg == 0:
        return along, 0.0, 0.0
    if leg == 1:
        return side, along, math.pi / 2.0
    if leg == 2:
        return side - along, side, math.pi
    return 0.0, side - along, -math.pi / 2.0


def odometry_message(x: float, y: float, yaw: float) -> dict:
    """A nav_msgs/Odometry in the JSON shape rosbridge delivers.

    map -> base_footprint, matching what localization_status publishes on
    the rover. The covariance is left out: rosbridge fills absent fields
    with zeros, and this fixture has no uncertainty to claim either way.
    """
    return {
        "header": {"frame_id": "map"},
        "child_frame_id": "base_footprint",
        "pose": {"pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)},
        }},
    }


def status_payload(state: str, seconds_since_ok: float | None,
                   distance_travelled: float) -> str:
    """The /localization/status JSON, exactly as localization_status
    publishes it, so the ground station's parser is exercised for real."""
    return json.dumps({
        "state": state,
        "seconds_since_ok": seconds_since_ok,
        "source": "zed_vio",
        "distance_travelled": distance_travelled,
        "mount_offset_verified": True,
    })
```

- [ ] **Step 4: Run the walk tests to verify they pass**

Run: `.venv/bin/pytest tests/test_square_walk.py -q`
Expected: PASS, 7 tests.

- [ ] **Step 5: Serve it from the mock rosbridge**

In `mock/ros_bridge.py`, extend the imports and constants (lines 18-24):

```python
import argparse
import importlib.util
import json
import pathlib
import time

from autobahn.twisted.websocket import WebSocketServerFactory, WebSocketServerProtocol
from twisted.internet import reactor
from twisted.internet.task import LoopingCall

MOCK_NODE_NAMES = ["/rosbridge_websocket", "/rosapi", "/localization_status"]

# Loaded by path for the same reason tests/test_square_walk.py does it:
# mock/ is not a package, and this file is run as a script from the repo
# root by start_ground_station.sh.
_SQUARE_WALK = pathlib.Path(__file__).resolve().parent / "square_walk.py"
_spec = importlib.util.spec_from_file_location("square_walk", _SQUARE_WALK)
square_walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(square_walk)

POSE_HZ = 10.0      # the ground station throttles to 5 Hz anyway
STATUS_HZ = 2.0     # what localization_status publishes at
```

Replace `_handle_publish` (lines 57-63) so the broadcast lives on the factory and both callers use it:

```python
    def _handle_publish(self, message):
        topic = message.get("topic")
        msg = message.get("msg")
        print(f"[mock-rosbridge] publish {topic}: {msg}")
        self.factory.broadcast(topic, msg)
```

and add to `MockRosBridgeFactory` (line 83):

```python
    def broadcast(self, topic, msg):
        """Sends one message to every connected client. Deliberately not
        routed by subscription: this server has one client (the ground
        station) and pretending otherwise would be more code than the thing
        it stands in for."""
        outgoing = json.dumps({"op": "publish", "topic": topic, "msg": msg}).encode("utf8")
        for client in self.clients:
            client.sendMessage(outgoing)
```

Replace `main` (lines 91-99):

```python
def main() -> None:
    parser = argparse.ArgumentParser(description="Mock rosbridge server for local testing")
    parser.add_argument("--port", type=int, default=9090, help="port to listen on (default 9090)")
    parser.add_argument(
        "--localization-state", choices=["OK", "SEARCHING", "OFF"], default="OK",
        help=(
            "what the fake /localization/status reports (default OK). "
            "SEARCHING freezes the pose and counts the seconds, exactly as the "
            "rover does; OFF publishes no pose at all. Between the three, every "
            "marker the video panel can show is reachable without a rover."))
    parser.add_argument("--square-side", type=float, default=4.0,
                        help="side of the square the fake pose walks, in metres (default 4)")
    parser.add_argument("--square-speed", type=float, default=0.5,
                        help="how fast it walks it, in m/s (default 0.5)")
    args = parser.parse_args()

    factory = MockRosBridgeFactory(f"ws://0.0.0.0:{args.port}")
    reactor.listenTCP(args.port, factory)

    started = time.monotonic()
    # Frozen while SEARCHING, matching the rover: localization_status keeps
    # publishing the last good pose with its stamp frozen rather than
    # extrapolating, and a mock that kept walking would make the panel's
    # SEARCHING marker look like a cosmetic warning over a healthy picture.
    frozen = {"at": None}

    def publish_pose():
        if args.localization_state == "OFF":
            return
        if args.localization_state == "SEARCHING":
            if frozen["at"] is None:
                frozen["at"] = time.monotonic() - started
            elapsed = frozen["at"]
        else:
            elapsed = time.monotonic() - started
        x, y, yaw = square_walk.square_pose(elapsed, args.square_side, args.square_speed)
        factory.broadcast("/localization/pose",
                          square_walk.odometry_message(x, y, yaw))

    def publish_status():
        elapsed = time.monotonic() - started
        seconds_since_ok = elapsed if args.localization_state == "SEARCHING" else (
            0.0 if args.localization_state == "OK" else None)
        factory.broadcast("/localization/status", {"data": square_walk.status_payload(
            args.localization_state, seconds_since_ok,
            args.square_speed * elapsed if args.localization_state == "OK" else 0.0)})

    LoopingCall(publish_pose).start(1.0 / POSE_HZ, now=False)
    LoopingCall(publish_status).start(1.0 / STATUS_HZ, now=False)

    print(f"[mock-rosbridge] listening on ws://localhost:{args.port} - connect the ground station to \"localhost\"")
    print(f"[mock-rosbridge] fake /localization/pose walking a "
          f"{args.square_side} m square at {args.square_speed} m/s, "
          f"status {args.localization_state}")
    reactor.run()
```

- [ ] **Step 6: Run the mock and watch the ground station follow the square**

```bash
./start_ground_station.sh --mock
```
Expected: the header readout `LOC: x … y … yaw …` advances about five times a second, `x` climbing to 4.00 over eight seconds, then `y` climbing while `yaw` reads `90.0°`, and so on around the square, returning to `x 0.00 y 0.00 yaw 0.0°` after 32 s. Switch to **Semi-autonomous**: the panel title reads `CAMERA / SIMULATION  -  LOCALISED`. Press the video button: the status line reads `no camera stream in semi-autonomous mode` and the picture does not change.

To see the other two markers, start the mock directly with the state you want
— `--mock` always starts it with its defaults and has no way to pass this
through — and point the ground station at it:

```bash
.venv/bin/python mock/ros_bridge.py --localization-state SEARCHING
```
in one terminal and
```bash
./start_ground_station.sh localhost 9090
```
in another. Expected: the marker reads `SEARCHING … N s` with N climbing, and the header readout stops moving (the pose is frozen, exactly as the rover freezes it). Repeat with `--localization-state OFF`: the marker reads `LOCALISATION OFF` and the header stays at `LOC: NO POSE`, because no pose is published at all.

- [ ] **Step 7: Write `mock/fake_localization.py`**

```python
#!/usr/bin/env python3
"""The same square as mock/ros_bridge.py, on a real ROS domain.

The mock rosbridge is a websocket server and puts nothing on a DDS graph, so
it can drive the ground station and nothing else. sim_bridge and sim_ik_node
read DDS, so checking that Gazebo's rover walks the square needs a real
publisher - this one. It imports the same square_walk module the mock does,
which is what makes "the same square" a fact rather than a hope.

Run it on a throwaway domain, never on the rover's:

    bash -c 'source /opt/ros/humble/setup.bash && \\
             ROS_DOMAIN_ID=91 python3 mock/fake_localization.py'

This is a test fixture, not a node anybody ships, which is why it lives in
mock/ beside the mock rosbridge rather than in a ROS package.
"""

import argparse
import importlib.util
import math
import os
import pathlib
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

_SQUARE_WALK = pathlib.Path(__file__).resolve().parent / "square_walk.py"
_spec = importlib.util.spec_from_file_location("square_walk", _SQUARE_WALK)
square_walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(square_walk)

POSE_HZ = 10.0
STATUS_HZ = 2.0


class FakeLocalization(Node):
    def __init__(self, state: str, side: float, speed: float):
        super().__init__("fake_localization")
        self.state = state
        self.side = side
        self.speed = speed
        self.started = time.monotonic()
        self.frozen_at = None
        self.pose_publisher = self.create_publisher(Odometry, "/localization/pose", 10)
        self.status_publisher = self.create_publisher(String, "/localization/status", 10)
        self.create_timer(1.0 / POSE_HZ, self.publish_pose)
        self.create_timer(1.0 / STATUS_HZ, self.publish_status)
        self.get_logger().info(
            f"fake localisation: a {side} m square at {speed} m/s, state {state}, "
            f"on domain {os.environ.get('ROS_DOMAIN_ID', '0')}")

    def refuse_if_the_rover_is_already_publishing(self) -> bool:
        """Two publishers on /localization/pose is the one way this fixture
        can do harm: whoever is downstream would see the fake square and the
        real rover interleaved and have no way to tell. Costs one graph
        query at startup."""
        if self.count_publishers("/localization/pose") > 1:
            self.get_logger().error(
                "something else is already publishing /localization/pose on this "
                "domain - refusing to start. Run this on a throwaway domain "
                "(ROS_DOMAIN_ID=91), never on the rover's.")
            return True
        return False

    def publish_pose(self):
        if self.state == "OFF":
            return
        if self.state == "SEARCHING":
            if self.frozen_at is None:
                self.frozen_at = time.monotonic() - self.started
            elapsed = self.frozen_at
        else:
            elapsed = time.monotonic() - self.started
        x, y, yaw = square_walk.square_pose(elapsed, self.side, self.speed)
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = "base_footprint"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_publisher.publish(message)

    def publish_status(self):
        elapsed = time.monotonic() - self.started
        if self.state == "OK":
            seconds_since_ok, distance = 0.0, self.speed * elapsed
        elif self.state == "SEARCHING":
            seconds_since_ok, distance = elapsed, 0.0
        else:
            seconds_since_ok, distance = None, 0.0
        message = String()
        message.data = square_walk.status_payload(self.state, seconds_since_ok, distance)
        self.status_publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", choices=["OK", "SEARCHING", "OFF"], default="OK")
    parser.add_argument("--side", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=0.5)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = FakeLocalization(args.state, args.side, args.speed)
    if node.refuse_if_the_rover_is_already_publishing():
        node.destroy_node()
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

```bash
chmod +x mock/fake_localization.py
```

- [ ] **Step 8: Check the fixture publishes on a throwaway domain**

```bash
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 python3 mock/fake_localization.py' &
sleep 3
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 ros2 topic echo /localization/pose --once'
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 ros2 topic hz /localization/pose --window 20'
```
Expected: an `Odometry` in `map` with `child_frame_id: base_footprint`, and about 10 Hz. Stop it with `kill %1`. Verify it stays off domain 0: `bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'` must not list `/localization/pose` while it runs.

- [ ] **Step 9: Run the whole laptop suite and commit**

Run: `.venv/bin/pytest tests/ -q`
Expected: PASS.

```bash
git add mock/square_walk.py mock/ros_bridge.py mock/fake_localization.py tests/test_square_walk.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Give the mocks a fake localisation that walks a closed square

One pure square_walk module, read by both the websocket mock the ground
station talks to and a real DDS publisher the simulation reads, so 'Gazebo
walks the same square as the panel' is a fact about shared code rather than
two loops that happen to look alike. --localization-state makes every panel
marker reachable without a rover, and SEARCHING freezes the pose the way
the rover freezes it."
```

---

### Task 6: Gazebo gets a way to be told where the rover is, and `planar_move` becomes optional

Two facts, checked on this laptop on 2026-08-29, drive this task:
- `sim/src/navi_sim_bringup/worlds/site.world` loads **no** `libgazebo_ros_state.so`, and the launch passes only `libgazebo_ros_init.so` and `libgazebo_ros_factory.so`. So `/gazebo/set_entity_state` does not exist today and neither does `/gazebo/model_states`, which the end-to-end check reads.
- `libgazebo_ros_state.so` is installed at `/opt/ros/humble/lib/`. Its `gazebo_ros_state.hpp` documents `get_entity_state` and `set_entity_state`; the binary also carries `model_states`, `link_states` and `update_rate`. With `<ros><namespace>/gazebo</namespace></ros>` those become `/gazebo/set_entity_state` and `/gazebo/model_states`.

It is loaded in **both** modes: it only adds services and topics, and `/gazebo/model_states` is the one honest way to ask Gazebo where the model actually is, in either mode.

`planar_move`, on the other hand, must not be loaded in `semi` mode: it and `sim_ik_node`'s `set_entity_state` would both be writing the model's pose every tick, and the picture would jitter between the two — the spec's "two writers of one model pose would fight".

**Files:**
- Modify: `sim/src/navi_sim_bringup/worlds/site.world:4-5`
- Modify: `sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro:7`, `:39-49`

**Interfaces:**
- Produces:
  - The world advertises `/gazebo/set_entity_state` (`gazebo_msgs/srv/SetEntityState`) and publishes `/gazebo/model_states` (`gazebo_msgs/msg/ModelStates`).
  - `asterope_sim.urdf.xacro` takes a `planar_move` argument, `true` by default. `xacro asterope_sim.urdf.xacro planar_move:=false` expands without the `libgazebo_ros_planar_move.so` plugin block and is otherwise byte-identical.

- [ ] **Step 1: Write the failing xacro expansion check**

This one is a shell check rather than a unit test — the artefact is an XML document produced by a tool, and the thing worth asserting is what that document contains.

```bash
cd /home/ole/star/Navi
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  xacro sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro planar_move:=false \
  | grep -c libgazebo_ros_planar_move.so'
```
Expected: fails with `xacro: error: unknown macro parameter` / `Invalid parameter "planar_move"` — the argument does not exist yet.

- [ ] **Step 2: Put `planar_move` behind a xacro argument**

In `sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro`, after the `<robot ...>` line (line 7):

```xml
  <!-- planar_move moves the model from /sim_cmd_vel. In semi-autonomous
       mode the body pose comes from the rover's localisation instead, and
       sim_ik_node writes it through /gazebo/set_entity_state - two writers
       of one model pose fight, and the picture jitters between them. So the
       launch expands this file with planar_move:=false in that mode. -->
  <xacro:arg name="planar_move" default="true"/>
```

and wrap the plugin block (lines 39-49):

```xml
  <!-- Moves the model from /sim_cmd_vel without physics: exactly the
       kinematic body motion this simulation wants. Not loaded in
       semi-autonomous mode - see the argument at the top of this file. -->
  <xacro:if value="$(arg planar_move)">
    <gazebo>
      <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
        <ros><remapping>cmd_vel:=/sim_cmd_vel</remapping></ros>
        <robot_base_frame>base_footprint</robot_base_frame>
        <odometry_frame>odom</odometry_frame>
        <publish_odom>false</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
      </plugin>
    </gazebo>
  </xacro:if>
```

- [ ] **Step 3: Run the expansion check both ways**

```bash
cd /home/ole/star/Navi
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  xacro sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro planar_move:=false \
  | grep -c libgazebo_ros_planar_move.so'
```
Expected: `0` (grep exits 1, which is the pass here — the plugin is gone).

```bash
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  xacro sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro \
  | grep -c libgazebo_ros_planar_move.so'
```
Expected: `1` — the default is unchanged, so Simulation mode is untouched.

```bash
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  diff <(xacro sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro) \
       <(xacro sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro planar_move:=false)'
```
Expected: the only difference is the plugin block. `joint_pose_trajectory`, the chase camera and the eight `<gravity>false</gravity>` blocks appear in both.

- [ ] **Step 4: Add the state plugin to the world**

In `sim/src/navi_sim_bringup/worlds/site.world`, after `<include><uri>model://sun</uri></include>` (line 5):

```xml
    <!-- Lets a ROS node move a model and ask where one is:
         /gazebo/set_entity_state (how sim_ik_node places the rover from the
         rover's own localisation in semi-autonomous mode) and
         /gazebo/model_states (the only honest way to ask Gazebo where the
         model actually ended up, which is what the end-to-end check reads).

         Loaded in both modes: it adds services and a topic and moves
         nothing by itself, and having model_states only in one mode would
         mean the check that Gazebo agrees with its own odometry could only
         be run in that mode.

         The namespace is what makes the names /gazebo/... rather than
         /set_entity_state; sim_ik_node's kSetEntityStateService and the
         end-to-end check both spell them out in full. -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>30.0</update_rate>
    </plugin>
```

- [ ] **Step 5: Check the service and the topic appear**

```bash
./start_sim.sh --twist-topic /sim_test_twist
```
in one terminal, and in another once Gazebo is up:
```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 service list | grep entity_state'
bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /gazebo/model_states --once'
```
Expected: `/gazebo/get_entity_state` and `/gazebo/set_entity_state` are listed, and `model_states` names `ground`, `site_scan` and `asterope` with a pose for each. `--twist-topic /sim_test_twist` is not optional here: the default is `/manual_twist`, which drives the physical rover.

Then check the service actually moves the model:
```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 service call /gazebo/set_entity_state \
  gazebo_msgs/srv/SetEntityState \
  "{state: {name: asterope, pose: {position: {x: 2.0, y: 1.0, z: 0.05}}, reference_frame: world}}"'
```
Expected: `success=True`, and the rover jumps in the Gazebo window. Note that with `planar_move` still loaded (this is Simulation mode) it will drift back on the next `/sim_cmd_vel` — which is exactly the fight Task 8 avoids by not loading the plugin.

Stop the simulation with Ctrl-C.

- [ ] **Step 6: Commit**

```bash
git add sim/src/navi_sim_bringup/worlds/site.world \
        sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Let Gazebo be told where the rover is, and make planar_move optional

libgazebo_ros_state.so gives the world /gazebo/set_entity_state, which is
how the rover's own localisation will place the model, and
/gazebo/model_states, which is how anyone checks where it actually ended
up. planar_move moves behind a xacro argument so the semi-autonomous launch
can leave it out: two writers of one model pose fight."
```

---

### Task 7: The pose gate and the stepper's pose setter — pure, and under gtest

`sim/src/navi_sim_ik/test/` links `navi_sim_ik_stepper` and `navi_sim_ik_model` only; there is no `rclcpp` in the test targets and no node harness. That is a good constraint, not an obstacle: everything the spec asks the pose-override path to decide — *may this pose be applied at all* (status is `OK`), *may it be applied now* (≤ 30 Hz), and *what does the pose become* — is decidable without ROS. This task builds those two pieces and tests them; Task 8 wires them to topics and to Gazebo.

**Files:**
- Create: `sim/src/navi_sim_ik/include/navi_sim_ik/external_pose.hpp`, `src/external_pose.cpp`, `test/test_external_pose.cpp`
- Modify: `sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp:62-66`, `test/test_sim_ik_stepper.cpp`, `CMakeLists.txt:25-43`

**Interfaces:**
- Produces:
  ```cpp
  // navi_sim_ik/external_pose.hpp
  namespace navi_sim_ik
  {
  /// The "state" field of a /localization/status JSON payload; "" if absent
  /// or not a string.
  std::string localization_state(const std::string & status_json);

  class ExternalPoseGate
  {
  public:
    explicit ExternalPoseGate(double max_rate_hz = 30.0);
    void set_state(const std::string & state);
    const std::string & state() const;
    bool ok() const;                        ///< state() == "OK"
    bool accept(double now_seconds);        ///< and records the acceptance
  };
  }

  // navi_sim_ik/sim_ik_stepper.hpp
  void SimIkStepper::set_pose(const Pose2D & pose);
  ```
  and a CMake library target `navi_sim_ik_external_pose`.

- [ ] **Step 1: Write the failing gtests for the gate**

`sim/src/navi_sim_ik/test/test_external_pose.cpp`:

```cpp
#include <gtest/gtest.h>

#include <string>

#include "navi_sim_ik/external_pose.hpp"

using navi_sim_ik::ExternalPoseGate;
using navi_sim_ik::localization_state;

// The exact payload localization_status publishes (sub-project 1). Pinned
// here as a whole string rather than a fragment: this is the wire format
// this node reads, and a change to it should break this file.
constexpr char kOkStatus[] =
  "{\"state\": \"OK\", \"seconds_since_ok\": 0.0, \"source\": \"zed_vio\", "
  "\"distance_travelled\": 12.5, \"mount_offset_verified\": true}";
constexpr char kSearchingStatus[] =
  "{\"state\": \"SEARCHING\", \"seconds_since_ok\": 4.2, \"source\": \"zed_vio\", "
  "\"distance_travelled\": 12.5, \"mount_offset_verified\": true}";

TEST(LocalizationState, ReadsTheStateOutOfARealStatusPayload)
{
  EXPECT_EQ(localization_state(kOkStatus), "OK");
  EXPECT_EQ(localization_state(kSearchingStatus), "SEARCHING");
}

TEST(LocalizationState, EmptyWhenThereIsNoStateToRead)
{
  // Anything that is not a status with a string state must read as "not OK",
  // and "" is not OK. A payload this node cannot understand is exactly when
  // it must not move the model.
  EXPECT_EQ(localization_state(""), "");
  EXPECT_EQ(localization_state("not json at all"), "");
  EXPECT_EQ(localization_state("{\"source\": \"zed_vio\"}"), "");
  // A non-string state must not be answered with the next quoted thing in
  // the payload, which would return "zed_vio" and read as an unknown state.
  EXPECT_EQ(localization_state("{\"state\": 3, \"source\": \"zed_vio\"}"), "");
}

TEST(ExternalPoseGate, RefusesEverythingBeforeAnyStatusArrives)
{
  // Startup order is not guaranteed: a pose can arrive before the first 2 Hz
  // status. Moving the model on a pose whose health is unknown is the one
  // thing this gate exists to prevent.
  ExternalPoseGate gate;
  EXPECT_FALSE(gate.ok());
  EXPECT_FALSE(gate.accept(0.0));
}

TEST(ExternalPoseGate, AcceptsWhileLocalisationIsOk)
{
  ExternalPoseGate gate;
  gate.set_state("OK");
  EXPECT_TRUE(gate.ok());
  EXPECT_TRUE(gate.accept(100.0));
}

TEST(ExternalPoseGate, HoldsStillWhileSearchingAndWhileOff)
{
  ExternalPoseGate gate;
  gate.set_state("OK");
  ASSERT_TRUE(gate.accept(100.0));

  gate.set_state("SEARCHING");
  EXPECT_FALSE(gate.accept(101.0));
  gate.set_state("OFF");
  EXPECT_FALSE(gate.accept(102.0));
}

TEST(ExternalPoseGate, RecoversOnItsOwnWhenLocalisationComesBack)
{
  // Recovery is automatic when the SDK re-acquires - nothing restarts, and
  // nothing has to be pressed.
  ExternalPoseGate gate;
  gate.set_state("SEARCHING");
  EXPECT_FALSE(gate.accept(100.0));

  gate.set_state("OK");
  EXPECT_TRUE(gate.accept(101.0));
}

TEST(ExternalPoseGate, CapsTheRateAtThirtyHertz)
{
  // /localization/pose arrives at the wrapper's 30 Hz today, but nothing
  // guarantees that: a faster publisher would put a service call and a
  // physics-thread write on Gazebo for every message. The cap is on this
  // side because it is the side that knows.
  ExternalPoseGate gate(30.0);
  gate.set_state("OK");
  ASSERT_TRUE(gate.accept(100.0));

  EXPECT_FALSE(gate.accept(100.010));    // 100 Hz worth of poses
  EXPECT_FALSE(gate.accept(100.030));    // still inside 1/30 s
  EXPECT_TRUE(gate.accept(100.034));     // 1/30 s = 0.0333 s has passed
}

TEST(ExternalPoseGate, ARefusedPoseDoesNotRestartTheRateWindow)
{
  // If a refusal moved the window, a fast publisher would starve the gate
  // forever: every message would land inside a window its predecessor just
  // reset, and the model would never move at all.
  ExternalPoseGate gate(30.0);
  gate.set_state("OK");
  ASSERT_TRUE(gate.accept(100.0));
  for (double t = 100.005; t < 100.033; t += 0.005) {
    EXPECT_FALSE(gate.accept(t));
  }
  EXPECT_TRUE(gate.accept(100.034));
}

TEST(ExternalPoseGate, TheFirstPoseIsNeverRateLimited)
{
  // Whatever "now" happens to be at startup, the first accepted pose must
  // go through: a steady clock starts wherever it starts, and comparing it
  // against a zero-initialised last-applied time would either pass by luck
  // or block for as long as the machine has been up.
  ExternalPoseGate gate(30.0);
  gate.set_state("OK");
  EXPECT_TRUE(gate.accept(1.0e6));
}
```

- [ ] **Step 2: Write the failing gtest for the stepper's setter**

Append to `sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp`:

```cpp
TEST(SimIkStepper, SetPoseReplacesTheIntegratedPoseOutright)
{
  // In semi-autonomous mode the body pose comes from the rover's
  // localisation and the integration is not a second opinion to be blended
  // with it - it is dead reckoning, which is what localisation exists to
  // replace.
  SimIkStepper stepper;
  for (int i = 0; i < 50; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  ASSERT_GT(stepper.pose().x, 0.5);

  stepper.set_pose(navi_sim_ik::Pose2D{3.0, -1.0, 0.5});

  EXPECT_DOUBLE_EQ(stepper.pose().x, 3.0);
  EXPECT_DOUBLE_EQ(stepper.pose().y, -1.0);
  EXPECT_DOUBLE_EQ(stepper.pose().yaw, 0.5);
}

TEST(SimIkStepper, SteppingAfterSetPoseCarriesOnFromTheNewPose)
{
  // The wheels keep turning from /manual_twist while the body pose comes
  // from outside, so the next step must integrate from where localisation
  // put the rover, not from where dead reckoning had it.
  SimIkStepper stepper;
  for (int i = 0; i < 50; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  stepper.set_pose(navi_sim_ik::Pose2D{10.0, 0.0, 0.0});

  stepper.step(0.5, 0.0, 0.0);

  EXPECT_GT(stepper.pose().x, 10.0);
  EXPECT_LT(stepper.pose().x, 10.1);   // one 0.06 s tick at ~0.5 m/s
}
```

- [ ] **Step 3: Run the gtests to verify they fail**

Run: `bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik'`
Expected: FAIL at compile — `fatal error: navi_sim_ik/external_pose.hpp: No such file or directory` and `'class navi_sim_ik::SimIkStepper' has no member named 'set_pose'`.

- [ ] **Step 4: Write `external_pose.hpp`**

```cpp
#ifndef NAVI_SIM_IK__EXTERNAL_POSE_HPP_
#define NAVI_SIM_IK__EXTERNAL_POSE_HPP_

#include <string>

namespace navi_sim_ik
{

/// The "state" field of a /localization/status payload, or "" if the payload
/// does not carry one as a string.
///
/// Scanned rather than parsed. The alternative is a JSON dependency in this
/// package for one string field out of a document this project's own
/// localization_status node writes with json.dumps - and a parser that
/// throws on a malformed payload would need the same "anything unreadable is
/// not OK" fallback this returns directly.
std::string localization_state(const std::string & status_json);

/// Decides whether a pose from outside may be written into the simulation.
///
/// Two rules, both of them the spec's:
///  - the model holds still whenever /localization/status is not OK, because
///    a rover drawn where it was 40 seconds ago, moving, is worse than one
///    that visibly stops;
///  - poses are applied at no more than max_rate_hz, because each one costs
///    a service call and a write on Gazebo's physics thread.
///
/// Deliberately knows nothing about ROS or Gazebo: everything it decides is
/// decidable from a state string and a clock reading, which is what lets it
/// be tested exhaustively without a node.
class ExternalPoseGate
{
public:
  explicit ExternalPoseGate(double max_rate_hz = 30.0);

  /// The latest state from /localization/status. "" means none has arrived.
  void set_state(const std::string & state);
  const std::string & state() const {return state_;}
  bool ok() const {return state_ == "OK";}

  /// True if a pose seen at now_seconds may be applied, and records that it
  /// was. A refusal does not move the rate window: if it did, a publisher
  /// faster than max_rate_hz would starve the gate forever.
  bool accept(double now_seconds);

private:
  double min_interval_;
  std::string state_;
  double last_applied_{0.0};
  bool ever_applied_{false};
};

}  // namespace navi_sim_ik

#endif  // NAVI_SIM_IK__EXTERNAL_POSE_HPP_
```

- [ ] **Step 5: Write `src/external_pose.cpp`**

```cpp
#include "navi_sim_ik/external_pose.hpp"

#include <string>

namespace navi_sim_ik
{

std::string localization_state(const std::string & status_json)
{
  const std::string key = "\"state\"";
  const auto key_at = status_json.find(key);
  if (key_at == std::string::npos) {
    return "";
  }
  const auto colon = status_json.find(':', key_at + key.size());
  if (colon == std::string::npos) {
    return "";
  }
  // The value must be a quoted string belonging to THIS key. Without the
  // end bound, {"state": 3, "source": "zed_vio"} would answer "zed_vio" -
  // a state that looks unrecognised rather than absent, which is a
  // different and more confusing failure.
  const auto value_end = status_json.find_first_of(",}", colon + 1);
  const auto open = status_json.find('"', colon + 1);
  if (open == std::string::npos ||
    (value_end != std::string::npos && open > value_end))
  {
    return "";
  }
  const auto close = status_json.find('"', open + 1);
  if (close == std::string::npos) {
    return "";
  }
  return status_json.substr(open + 1, close - open - 1);
}

ExternalPoseGate::ExternalPoseGate(double max_rate_hz)
: min_interval_(max_rate_hz > 0.0 ? 1.0 / max_rate_hz : 0.0)
{
}

void ExternalPoseGate::set_state(const std::string & state)
{
  state_ = state;
}

bool ExternalPoseGate::accept(double now_seconds)
{
  if (!ok()) {
    return false;
  }
  // ever_applied_ rather than a sentinel time: a steady clock starts
  // wherever it starts, so comparing an unset last_applied_ of 0.0 against
  // it would either pass by luck or block for as long as the machine has
  // been up.
  if (ever_applied_ && now_seconds - last_applied_ < min_interval_) {
    return false;
  }
  last_applied_ = now_seconds;
  ever_applied_ = true;
  return true;
}

}  // namespace navi_sim_ik
```

- [ ] **Step 6: Add `set_pose` to the stepper**

In `sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp`, after `const Pose2D & pose() const {return pose_;}` (line 63):

```cpp
  /// Replaces the integrated pose with one from outside (localisation).
  ///
  /// Not a blend and not a correction: what this replaces is dead
  /// reckoning, which is the thing localisation exists to stop trusting.
  /// The wheel and steering state is untouched, so the wheels keep turning
  /// from /manual_twist while the body goes where the rover really is.
  void set_pose(const Pose2D & pose) {pose_ = pose;}
```

- [ ] **Step 7: Build the new library and gtest**

In `sim/src/navi_sim_ik/CMakeLists.txt`, after the `navi_sim_ik_stepper` library block (line 28):

```cmake
# Pure: no rclcpp, no Gazebo. Everything the pose-override path decides is
# decidable from a status string and a clock reading, so it is decided here
# and tested exhaustively without a node harness.
add_library(navi_sim_ik_external_pose STATIC src/external_pose.cpp)
target_include_directories(navi_sim_ik_external_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
```

link it into the node (line 31):

```cmake
target_link_libraries(sim_ik_node navi_sim_ik_stepper navi_sim_ik_external_pose)
```

and add the gtest inside the `BUILD_TESTING` block (after line 42):

```cmake
  ament_add_gtest(test_external_pose test/test_external_pose.cpp)
  target_link_libraries(test_external_pose navi_sim_ik_external_pose)
```

- [ ] **Step 8: Run the gtests to verify they pass**

Run:
```bash
bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && \
  colcon build --packages-select navi_sim_ik && \
  colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
```
Expected: PASS. `test_external_pose` runs 9 tests, `test_sim_ik_stepper` now runs 11.

- [ ] **Step 9: Commit**

```bash
git add sim/src/navi_sim_ik/include/navi_sim_ik/external_pose.hpp \
        sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp \
        sim/src/navi_sim_ik/src/external_pose.cpp \
        sim/src/navi_sim_ik/test/test_external_pose.cpp \
        sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp \
        sim/src/navi_sim_ik/CMakeLists.txt
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Add the pose gate and the stepper's pose setter, with no ROS attached

Whether a pose from the rover may be written into the simulation is decided
by a status string and a clock reading, so it is decided in a class the
existing gtest harness can exercise exhaustively - the model holds still
whenever localisation is not OK, and no more than 30 poses a second reach
Gazebo. Wiring it to topics is the next commit."
```

---

### Task 8: `sim_ik_node` takes its body pose from `/localization/pose`

The spec: "`sim_ik_node` gains a `pose_topic` parameter (default empty). When set, it subscribes `nav_msgs/Odometry` there and, on each message, **replaces** its integrated pose with the received one before publishing the joint trajectory and setting the model. The IK keeps driving the wheel and steering joints from `/manual_twist` … only the body pose comes from outside."

The pose is applied inside `tick()`, between `stepper_.step(...)` and the publishes — not in the subscription callback. A pose written before the step would be integrated away by that same tick, and `/sim_odom` would report the localised pose plus one tick of dead reckoning. Applied where this task puts it, `/sim_odom` is exactly what localisation said, which is the assertion the spec's test asks for.

**Files:**
- Modify: `sim/src/navi_sim_ik/src/sim_ik_node.cpp` — includes (1-13), the anonymous namespace (15-77), the constructor (88-138), `tick()` (141-200), `publish_motion()` (283-317), the members (333-367)
- Modify: `sim/src/navi_sim_ik/CMakeLists.txt:8-13,32-33`, `package.xml:10-14`

**Interfaces:**
- Consumes: `navi_sim_ik::ExternalPoseGate`, `navi_sim_ik::localization_state`, `SimIkStepper::set_pose` from Task 7. `/gazebo/set_entity_state` from Task 6.
- Produces: `sim_ik_node` parameters
  - `pose_topic` (string, default `""`) — empty keeps today's dead-reckoned behaviour byte for byte.
  - `status_topic` (string, default `"/localization/status"`)
  - `model_name` (string, default `"asterope"`) — must match `spawn_entity.py -entity` in `sim.launch.py:116`.
  - `pose_z_offset` (double, default `0.05`) — the same 0.05 m `spawn_entity.py -z` uses.
  - `max_pose_rate_hz` (double, default `30.0`)

  and `/sim_odom`'s `header.frame_id`, which becomes `map` when `pose_topic` is set and stays `odom` when it is not.

- [ ] **Step 1: Add the build dependencies**

`sim/src/navi_sim_ik/CMakeLists.txt`, after `find_package(geometry_msgs REQUIRED)` (line 10):

```cmake
find_package(gazebo_msgs REQUIRED)
```
and in `ament_target_dependencies(sim_ik_node ...)` (line 32-33):

```cmake
ament_target_dependencies(sim_ik_node
  rclcpp geometry_msgs gazebo_msgs nav_msgs std_msgs trajectory_msgs)
```

`sim/src/navi_sim_ik/package.xml`, after `<depend>geometry_msgs</depend>` (line 11):

```xml
  <depend>gazebo_msgs</depend>
```

Verified present: `/opt/ros/humble/include/gazebo_msgs/gazebo_msgs/srv/set_entity_state.hpp` and `/opt/ros/humble/share/gazebo_msgs/cmake`.

- [ ] **Step 2: Add the includes and the two new constants**

In `sim_ik_node.cpp`, add to the includes (lines 1-13):

```cpp
#include <optional>

#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "navi_sim_ik/external_pose.hpp"
```

and inside the anonymous namespace, after `kDeadReckoningVariance` (line 76):

```cpp
// gazebo_ros_state's set_entity_state, under the /gazebo namespace the world
// file gives it. This is how the model is moved in semi-autonomous mode;
// planar_move is not loaded there, because two writers of one model pose
// fight and the picture jitters between them.
constexpr char kSetEntityStateService[] = "/gazebo/set_entity_state";

/// Heading out of a quaternion. The pose on /localization/pose is planar in
/// everything this simulation shows, so one angle is the whole of it.
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
```

- [ ] **Step 3: Subscribe, in the constructor**

Add at the end of the `SimIkNode` constructor, before the `timer_ = ...` block (line 135):

```cpp
    pose_topic_ = declare_parameter<std::string>("pose_topic", "");
    status_topic_ = declare_parameter<std::string>("status_topic", "/localization/status");
    model_name_ = declare_parameter<std::string>("model_name", "asterope");
    pose_z_offset_ = declare_parameter<double>("pose_z_offset", 0.05);
    const double max_pose_rate_hz = declare_parameter<double>("max_pose_rate_hz", 30.0);
    gate_ = navi_sim_ik::ExternalPoseGate(max_pose_rate_hz);

    // Empty by default, so Simulation mode is byte for byte what it was:
    // no subscriptions, no service client, no behaviour to regress.
    if (!pose_topic_.empty()) {
      pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        pose_topic_, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          // Stored, not applied. Applying here would put the pose in
          // before this tick's step() integrates over it, and /sim_odom
          // would report localisation plus one tick of dead reckoning.
          // See apply_external_pose().
          pending_odom_ = msg;
        });
      status_sub_ = create_subscription<std_msgs::msg::String>(
        status_topic_, 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          gate_.set_state(navi_sim_ik::localization_state(msg->data));
        });
      set_state_client_ =
        create_client<gazebo_msgs::srv::SetEntityState>(kSetEntityStateService);
      RCLCPP_INFO(
        get_logger(),
        "external pose mode: body pose from %s, gated on %s, model '%s' placed "
        "through %s at up to %.1f Hz (z offset %.3f m). planar_move must NOT be "
        "loaded in this mode - two writers of one model pose fight.",
        pose_topic_.c_str(), status_topic_.c_str(), model_name_.c_str(),
        kSetEntityStateService, max_pose_rate_hz, pose_z_offset_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "dead-reckoning mode: the pose is integrated from /manual_twist and "
        "drifts without bound. Set pose_topic to place the model from the "
        "rover's own localisation instead.");
    }
```

- [ ] **Step 4: Apply it in `tick()`**

In `tick()`, replace the three lines after `check_tick_rate();` (lines 196-199) with:

```cpp
    stepper_.step(vx_, vy_, yaw_rate_);
    apply_external_pose();
    publish_joints();
    publish_motion();
    publish_debug(stale);
```

and add the two methods after `check_tick_rate()` (line 258):

```cpp
  /// Replaces the integrated body pose with the localised one, and moves the
  /// Gazebo model to match.
  ///
  /// Called between step() and the publishes, on purpose. In the
  /// subscription callback the pose would land before this tick's step()
  /// integrated over it and /sim_odom would report the localised pose plus
  /// one tick of dead reckoning - a small, permanent, plausible-looking
  /// error, which is the worst kind. Here, /sim_odom is exactly what
  /// localisation said.
  ///
  /// The wheels and steering are untouched: they keep coming from
  /// /manual_twist through the IK, so the picture still shows a rover
  /// steering and rolling rather than sliding. Only the body pose is
  /// external.
  void apply_external_pose()
  {
    if (!pending_odom_) {
      return;
    }
    // Steady, not the simulation clock, for the same reason the twist
    // staleness guard uses it: the 30 Hz cap protects a service and a
    // physics thread, which are real-time resources and do not slow down
    // when the real-time factor does.
    const double now = steady_clock_.now().seconds();
    if (!gate_.accept(now)) {
      if (!gate_.ok() && (!holding_ || holding_state_ != gate_.state())) {
        RCLCPP_WARN(
          get_logger(),
          "localisation reports '%s', not OK - holding the model still and "
          "ignoring poses on %s until it recovers.",
          gate_.state().empty() ? "(nothing yet)" : gate_.state().c_str(),
          pose_topic_.c_str());
        holding_ = true;
        holding_state_ = gate_.state();
      }
      return;
    }
    if (holding_) {
      RCLCPP_INFO(
        get_logger(), "localisation is OK again - the model follows the pose");
      holding_ = false;
      holding_state_.clear();
    }

    const auto odom = pending_odom_;
    pending_odom_.reset();
    const navi_sim_ik::Pose2D pose{
      odom->pose.pose.position.x, odom->pose.pose.position.y,
      yaw_of(odom->pose.pose.orientation)};
    stepper_.set_pose(pose);
    applied_odom_ = odom;
    send_entity_state(pose);
  }

  void send_entity_state(const navi_sim_ik::Pose2D & pose)
  {
    if (!set_state_client_->service_is_ready()) {
      // Warned once and then again on the next outage, rather than at the
      // pose rate: at 30 Hz this would be 30 identical lines a second, and
      // the launch log is where the reason has to be findable.
      if (!set_state_missing_) {
        RCLCPP_WARN(
          get_logger(),
          "%s is not available - is libgazebo_ros_state.so loaded in the world "
          "file? The joints will move and the body will not.",
          kSetEntityStateService);
        set_state_missing_ = true;
      }
      return;
    }
    if (set_state_missing_) {
      RCLCPP_INFO(get_logger(), "%s is available again", kSetEntityStateService);
      set_state_missing_ = false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = model_name_;
    request->state.reference_frame = "world";
    request->state.pose.position.x = pose.x;
    request->state.pose.position.y = pose.y;
    // /localization/pose is map -> base_footprint, so its z is the ground.
    // The model was spawned 0.05 m up (spawn_entity.py -z 0.05) and putting
    // it back at 0 here would sink it into the ground plane by that much on
    // the first pose.
    request->state.pose.position.z = pose_z_offset_;
    request->state.pose.orientation.z = std::sin(pose.yaw / 2.0);
    request->state.pose.orientation.w = std::cos(pose.yaw / 2.0);

    // Fire and forget. This node spins single-threaded, so waiting on the
    // future here would deadlock the very spin that delivers the response.
    // The callback exists only so a refusal is not silent.
    set_state_client_->async_send_request(
      request,
      [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture future) {
        if (!future.get()->success) {
          RCLCPP_WARN_ONCE(
            get_logger(),
            "%s refused to move '%s' - is that the entity's name in the world? "
            "It must match spawn_entity.py's -entity argument.",
            kSetEntityStateService, model_name_.c_str());
        }
      });
  }
```

- [ ] **Step 5: Tell the truth on `/sim_odom`**

In `publish_motion()`, replace the odometry frame and covariance block (lines 300-315):

```cpp
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    // The frame this pose is expressed in changes with where it comes from.
    // Publishing a localised, map-frame pose under frame_id "odom" would
    // quietly claim it is a drifting local estimate.
    odom.header.frame_id = applied_odom_ ? "map" : "odom";
    odom.child_frame_id = kBaseFootprintFrameId;
    odom.pose.pose.position.x = stepper_.pose().x;
    odom.pose.pose.position.y = stepper_.pose().y;
    odom.pose.pose.orientation.z = std::sin(stepper_.pose().yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(stepper_.pose().yaw / 2.0);
    // See kDeadReckoningVariance: an all-zero covariance asserts certainty,
    // and a dead-reckoned pose has none to assert. The 6x6 row-major
    // diagonal is (x, y, z, roll, pitch, yaw). The twist covariance is set
    // that way unconditionally, because odom.twist is left at zero either
    // way and a consumer must not read that unpopulated zero as a
    // confidently measured standstill.
    for (int i = 0; i < 6; ++i) {
      odom.twist.covariance[i * 6 + i] = kDeadReckoningVariance;
    }
    if (applied_odom_) {
      // The pose is localisation's and so is its uncertainty. Stamping
      // kDeadReckoningVariance onto a measured pose is the same kind of lie
      // as an all-zero covariance on a dead-reckoned one, pointing the
      // other way.
      odom.pose.covariance = applied_odom_->pose.covariance;
    } else {
      for (int i = 0; i < 6; ++i) {
        odom.pose.covariance[i * 6 + i] = kDeadReckoningVariance;
      }
    }
    odom_pub_->publish(odom);
```

- [ ] **Step 6: Add the members**

In the private member block, after `bool twist_ever_arrived_{false};` (line 342):

```cpp
  // External pose (semi-autonomous mode). All of it is inert while
  // pose_topic_ is empty: no subscriptions are created and pending_odom_ is
  // never set, so tick() costs one null check more than it did.
  std::string pose_topic_;
  std::string status_topic_;
  std::string model_name_;
  double pose_z_offset_{0.05};
  navi_sim_ik::ExternalPoseGate gate_{30.0};
  nav_msgs::msg::Odometry::SharedPtr pending_odom_;
  nav_msgs::msg::Odometry::SharedPtr applied_odom_;
  // Two fields rather than one: holding_state_ can legitimately be "" (no
  // status has arrived at all), so an empty string cannot also mean "not
  // holding" without swallowing the first and most important warning.
  bool holding_{false};
  std::string holding_state_;
  bool set_state_missing_{false};
```

and beside the other subscriptions and publishers (line 362):

```cpp
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;
```

- [ ] **Step 7: Build and run the existing tests**

Run:
```bash
bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && \
  colcon build --packages-select navi_sim_ik && \
  colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
```
Expected: builds clean, all gtests pass. The gtests do not link `rclcpp` and are unaffected — they cover the decisions, and Step 8 checks the wiring against a live Gazebo.

- [ ] **Step 8: Check the wiring against a live Gazebo**

The node-level claim — "given a pose message, the next published pose equals it" — needs a running Gazebo and a running node, so it is checked here rather than in a gtest (the gtest targets link no `rclcpp`, by design). `start_sim.sh` cannot do this in one flag yet; that is Task 10. Everything below runs on throwaway domain 91, so neither `/manual_twist` nor the rover's domain is touched.

Build first:
```bash
bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && \
  colcon build --packages-select navi_sim_ik navi_sim_bringup'
```

Terminal 1 — the fake rover pose:
```bash
bash -c 'source /opt/ros/humble/setup.bash && \
  ROS_DOMAIN_ID=91 python3 mock/fake_localization.py'
```

Terminal 2 — the simulation as it launches today, on the same domain, driven from a scratch topic nobody publishes to:
```bash
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  ROS_DOMAIN_ID=91 ros2 launch navi_sim_bringup sim.launch.py \
  twist_topic:=/sim_test_twist map_mesh:=$HOME/star/Navi/Model3D_mesh2.obj'
```
`planar_move` is still loaded here (Task 10 is what stops loading it), and that is survivable for this check only because nothing publishes on `/sim_test_twist`: `sim_ik_node` then publishes an all-zero `/sim_cmd_vel`, `planar_move` holds the model at zero velocity, and `set_entity_state` is the only thing moving it. Do not publish a twist during this step — that is the fight, and seeing it is Task 10's job, not this one's.

Terminal 3 — replace the launch's `sim_ik_node` with one that has `pose_topic` set:
```bash
pkill -x sim_ik_node
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  ROS_DOMAIN_ID=91 ros2 run navi_sim_ik sim_ik_node --ros-args \
  -p use_sim_time:=true -p pose_topic:=/localization/pose \
  -r /manual_twist:=/sim_test_twist'
```
Expected in its first lines: `external pose mode: body pose from /localization/pose, gated on /localization/status, model 'asterope' placed through /gazebo/set_entity_state at up to 30.0 Hz (z offset 0.050).`

Terminal 4 — the checks:
```bash
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
  ros2 topic echo /localization/pose --once --field pose.pose.position'
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
  ros2 topic echo /sim_odom --once'
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
  ros2 topic echo /gazebo/model_states --once'
```
Expected: `/sim_odom`'s `header.frame_id` is `map` (not `odom`), its position matches `/localization/pose`'s, and `/gazebo/model_states` has `asterope` at that same x/y with z 0.05. The two echoes are not simultaneous and the square advances at 0.5 m/s, so allow the distance one command's worth of time buys — run them back to back and compare the trend over three samples rather than demanding equality.

Then the failure path. Stop terminal 1 and restart it as:
```bash
bash -c 'source /opt/ros/humble/setup.bash && \
  ROS_DOMAIN_ID=91 python3 mock/fake_localization.py --state SEARCHING'
```
Expected: `sim_ik_node` logs `localisation reports 'SEARCHING', not OK - holding the model still and ignoring poses on /localization/pose until it recovers.` **once**, not once per pose, and `/gazebo/model_states` stops changing while `/localization/pose` keeps being published. Restart the fixture with `--state OK` and the log reads `localisation is OK again - the model follows the pose`.

Record the observed `/sim_odom` and `model_states` numbers in the commit message.

- [ ] **Step 9: Commit**

```bash
git add sim/src/navi_sim_ik/src/sim_ik_node.cpp sim/src/navi_sim_ik/CMakeLists.txt \
        sim/src/navi_sim_ik/package.xml
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Place the simulated rover from the rover's own localisation

With pose_topic set, the body pose comes from /localization/pose and the
model is moved through /gazebo/set_entity_state; the IK keeps driving the
wheels and steering from /manual_twist, so the picture still shows a rover
that steers rather than slides. The pose is applied between the step and
the publishes, so /sim_odom is exactly what localisation said rather than
that plus one tick of dead reckoning, and its frame_id and covariance stop
claiming to be dead reckoning too. Empty pose_topic is the default and
leaves Simulation mode untouched."
```

---

### Task 9: `sim_bridge` — one way, from the rover's domain into the simulation's

Neither machine has `domain_bridge`. Humble's `rclpy` supports several contexts with different `domain_id`s in one process, which is enough, and this node is that: two contexts, two nodes, one direction.

The direction is a property of the object graph, not a rule someone has to remember. The **sim-side node has no subscriptions and no executor at all** — only publishers. There is no code path that could carry a message the other way, so `/clock`, `/tf`, `/gazebo/*` and the sim's `/robot_description` cannot reach the rover's graph even by accident. That is the spec's "nothing goes the other way", made structural.

`grid_map_msgs` is not installed on this laptop (it arrives with sub-project 3), so message types are resolved by name at runtime and a topic whose type will not import is skipped with a warning rather than taking the bridge down.

**Files:**
- Create: `sim/src/navi_sim_bringup/scripts/sim_bridge.py`, `sim/src/navi_sim_bringup/test/test_sim_bridge.py`
- Modify: `sim/src/navi_sim_bringup/CMakeLists.txt:12-13`, `package.xml:8-13`

**Interfaces:**
- Produces:
  ```python
  DEFAULT_TOPICS: list[str]      # "<topic>:<pkg>/msg/<Type>" specs

  def parse_topic_spec(spec: str) -> tuple[str, str]      # (topic, type name)

  class SimBridge:
      def __init__(self, specs: list[str], rover_domain_id: int, sim_domain_id: int)
      bridged: list[str]         # topics actually wired up
      skipped: list[str]         # topics whose message type would not import
      def spin(self) -> None
      def shutdown(self) -> None

  def build_arg_parser() -> argparse.ArgumentParser
  def main(argv: list[str] | None = None) -> int
  ```
  Installed as `lib/navi_sim_bringup/sim_bridge.py`, i.e. `ros2 run navi_sim_bringup sim_bridge.py`. Command line: `--rover-domain N` (default 0), `--sim-domain N` (default 42), `--topic SPEC` (repeatable; defaults to `DEFAULT_TOPICS`).

- [ ] **Step 1: Write the failing tests**

`sim/src/navi_sim_bringup/test/test_sim_bridge.py`:

```python
"""Tests for the one-way domain bridge.

The bridge is a script, not a module in an installed Python package
(navi_sim_bringup is ament_cmake), so it is loaded by path.

Domains 91 and 92 are throwaways used by these tests only. Domain 0 is the
rover's and 42 is the simulation's default; neither appears here, and
neither does /manual_twist - the topic that drives the physical rover.
"""

import importlib.util
import pathlib
import threading
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

_PATH = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "sim_bridge.py"
_spec = importlib.util.spec_from_file_location("sim_bridge", _PATH)
sim_bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim_bridge)

ROVER_DOMAIN = 91
SIM_DOMAIN = 92
TOPIC = "/bridge_test"
SPEC = f"{TOPIC}:std_msgs/msg/String"


def test_parse_topic_spec_splits_the_topic_from_the_type():
    assert sim_bridge.parse_topic_spec("/localization/pose:nav_msgs/msg/Odometry") == (
        "/localization/pose", "nav_msgs/msg/Odometry")


def test_parse_topic_spec_refuses_something_that_is_not_a_spec():
    with pytest.raises(ValueError):
        sim_bridge.parse_topic_spec("/localization/pose")
    with pytest.raises(ValueError):
        sim_bridge.parse_topic_spec(":nav_msgs/msg/Odometry")


def test_a_topic_whose_type_is_not_installed_is_skipped_not_fatal():
    # grid_map_msgs arrives with sub-project 3. Until then the bridge has to
    # come up and carry everything else, or semi-autonomous mode cannot be
    # run at all on a laptop that does not have it yet.
    bridge = sim_bridge.SimBridge(
        [SPEC, "/localization/map:definitely_not_a_package/msg/Nothing"],
        ROVER_DOMAIN, SIM_DOMAIN)
    try:
        assert bridge.bridged == [TOPIC]
        assert bridge.skipped == ["/localization/map"]
    finally:
        bridge.shutdown()


def test_the_two_domains_must_differ():
    with pytest.raises(ValueError):
        sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, ROVER_DOMAIN)


def _node_on(domain_id, name):
    """A node on its own context and domain, plus that context, so the
    caller can shut it down. Each test builds its own: a context is not
    reusable after shutdown."""
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    return Node(name, context=context), context


def _spin_until(node, context, predicate, seconds=5.0):
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if predicate():
            break
    executor.remove_node(node)
    return predicate()


def test_a_message_on_the_rover_domain_appears_on_the_sim_domain():
    bridge = sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, SIM_DOMAIN)
    thread = threading.Thread(target=bridge.spin, daemon=True)
    thread.start()

    publisher_node, publisher_context = _node_on(ROVER_DOMAIN, "bridge_test_publisher")
    listener_node, listener_context = _node_on(SIM_DOMAIN, "bridge_test_listener")
    received = []
    listener_node.create_subscription(String, TOPIC, received.append, 10)
    publisher = publisher_node.create_publisher(String, TOPIC, 10)

    try:
        # Discovery is not instant and the bridge's subscription has to find
        # the publisher before anything is carried, so this publishes
        # repeatedly rather than once and waits for the first arrival.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not received:
            publisher.publish(String(data="hello"))
            _spin_until(listener_node, listener_context, lambda: bool(received), seconds=0.5)

        assert received, "nothing crossed from the rover domain to the sim domain"
        assert received[0].data == "hello"
    finally:
        publisher_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown(context=publisher_context)
        rclpy.shutdown(context=listener_context)
        bridge.shutdown()


def test_nothing_crosses_back_from_the_sim_domain():
    # This is the decision the whole two-domain design exists for: /clock,
    # /tf and the sim's /robot_description must never reach the rover's
    # graph. Asserting it with the bridge's own topic is the strongest form
    # of the check - if even the topic it is wired for does not come back,
    # nothing does.
    bridge = sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, SIM_DOMAIN)
    thread = threading.Thread(target=bridge.spin, daemon=True)
    thread.start()

    publisher_node, publisher_context = _node_on(SIM_DOMAIN, "backflow_publisher")
    listener_node, listener_context = _node_on(ROVER_DOMAIN, "backflow_listener")
    received = []
    listener_node.create_subscription(String, TOPIC, received.append, 10)
    publisher = publisher_node.create_publisher(String, TOPIC, 10)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            publisher.publish(String(data="should not cross"))
            _spin_until(listener_node, listener_context, lambda: bool(received), seconds=0.2)

        assert received == [], (
            "a message published on the simulation's domain reached the rover's")
    finally:
        publisher_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown(context=publisher_context)
        rclpy.shutdown(context=listener_context)
        bridge.shutdown()


def test_the_sim_side_node_has_no_subscriptions_at_all():
    # The structural half of the test above: the reverse direction is not
    # forbidden by a rule, it is absent from the object graph.
    bridge = sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, SIM_DOMAIN)
    try:
        # Asked of the nodes themselves, not of the graph:
        # get_subscriptions_info_by_topic reports every subscription anywhere,
        # including the bridge's own rover-side one on the other domain.
        assert len(bridge.sim_node.subscriptions) == 0
        assert len(bridge.rover_node.publishers) == 0
    finally:
        bridge.shutdown()
```

- [ ] **Step 2: Wire the test into the build**

`sim/src/navi_sim_bringup/CMakeLists.txt`, replacing lines 12-13:

```cmake
install(DIRECTORY urdf worlds launch DESTINATION share/${PROJECT_NAME})

# sim_bridge is a script rather than a module: this package is ament_cmake
# (it exists for a world file, a xacro and a launch file), and adding an
# ament_python package for one node would mean two packages where the launch
# file and the node it launches could drift apart.
install(PROGRAMS scripts/sim_bridge.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_bridge test/test_sim_bridge.py TIMEOUT 120)
endif()

ament_package()
```

`sim/src/navi_sim_bringup/package.xml`, adding to the dependency block:

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <test_depend>ament_cmake_pytest</test_depend>
```

`grid_map_msgs` is deliberately **not** declared: it is not installed here, `rosdep` would fail on it, and the bridge is written to work without it.

- [ ] **Step 3: Run the tests to verify they fail**

Run:
```bash
bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && \
  colcon build --packages-select navi_sim_bringup && \
  colcon test --packages-select navi_sim_bringup && colcon test-result --verbose'
```
Expected: FAIL — `scripts/sim_bridge.py` does not exist, so the module load raises at import time and every test errors.

- [ ] **Step 4: Write `scripts/sim_bridge.py`**

```python
#!/usr/bin/env python3
"""Carries the rover's topics onto the simulation's ROS domain. One way.

Why two domains at all: the simulation publishes /clock at 100 Hz, a /tf
tree, /gazebo/* and its own /robot_description. On one shared domain those
land on the rover's graph, where a node that ever sets use_sim_time would
have its clock driven by a laptop's Gazebo and where two /robot_description
publishers would disagree. The 2026-08-28 design flagged that collision; the
simulation moving to its own domain closes it.

Why not domain_bridge: neither machine has it. Humble's rclpy takes a
domain_id per context, and several contexts live happily in one process.

Why one way, structurally: the sim-side node here has publishers and nothing
else - no subscriptions, no executor. There is no code path that could carry
a message back to the rover, so the guarantee does not depend on anybody
remembering it. The only spinning executor is the rover-side one.

Types are resolved by name at runtime because grid_map_msgs (which
/localization/map needs) arrives with sub-project 3 and is not installed on
this laptop yet. A topic whose type will not import is skipped with a
warning: a bridge that refused to start would take semi-autonomous mode with
it, for a topic nothing is publishing yet anyway.

    ros2 run navi_sim_bringup sim_bridge.py --rover-domain 0 --sim-domain 42
"""

import argparse
import sys

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

#: Everything semi-autonomous mode needs out of the rover's graph.
#: /manual_twist so the IK still steers the wheels in the picture; the two
#: localisation topics so the model is placed and gated; /localization/map so
#: sub-project 3's terrain_writer has something to write. Nothing else, and
#: nothing in the other direction.
DEFAULT_TOPICS = [
    "/manual_twist:geometry_msgs/msg/Twist",
    "/localization/pose:nav_msgs/msg/Odometry",
    "/localization/status:std_msgs/msg/String",
    "/localization/map:grid_map_msgs/msg/GridMap",
]

QUEUE_DEPTH = 10


def parse_topic_spec(spec: str) -> tuple[str, str]:
    """Splits "<topic>:<pkg>/msg/<Type>".

    rpartition, not split: a topic name never contains a colon but this way
    the type - which is the part with the fixed shape - is what anchors the
    split.
    """
    topic, separator, type_name = spec.rpartition(":")
    if not separator or not topic or not type_name:
        raise ValueError(
            f"bad topic spec {spec!r} - expected '<topic>:<pkg>/msg/<Type>', "
            "e.g. '/localization/pose:nav_msgs/msg/Odometry'")
    return topic, type_name


class SimBridge:
    """Two rclpy contexts in one process, and traffic in one direction."""

    def __init__(self, specs, rover_domain_id: int, sim_domain_id: int):
        if rover_domain_id == sim_domain_id:
            raise ValueError(
                f"the rover and simulation domains must differ (both are "
                f"{rover_domain_id}). On one domain there is nothing to bridge "
                "and the simulation's /clock lands on the rover's graph.")

        self.rover_context = Context()
        rclpy.init(context=self.rover_context, domain_id=rover_domain_id)
        self.sim_context = Context()
        rclpy.init(context=self.sim_context, domain_id=sim_domain_id)

        self.rover_node = Node("sim_bridge_rover_side", context=self.rover_context)
        self.sim_node = Node("sim_bridge_sim_side", context=self.sim_context)
        self.bridged: list[str] = []
        self.skipped: list[str] = []

        for spec in specs:
            topic, type_name = parse_topic_spec(spec)
            try:
                message_type = get_message(type_name)
            except Exception as exc:      # noqa: BLE001 - see below
                # Broad on purpose: get_message raises ModuleNotFoundError,
                # AttributeError or ValueError depending on which half of
                # the name is unresolvable, and every one of them means the
                # same thing here - this machine cannot carry this topic.
                self.rover_node.get_logger().warn(
                    f"not bridging {topic}: cannot resolve {type_name} on this "
                    f"machine ({type(exc).__name__}: {exc}). Install the package "
                    "that defines it and restart the simulation.")
                self.skipped.append(topic)
                continue
            publisher = self.sim_node.create_publisher(message_type, topic, QUEUE_DEPTH)
            self.rover_node.create_subscription(
                message_type, topic,
                # Bound as a default argument: a late-bound closure over the
                # loop variable would make every subscription publish on the
                # last topic's publisher.
                lambda message, publisher=publisher: publisher.publish(message),
                QUEUE_DEPTH)
            self.bridged.append(topic)

        # One executor, for the rover side only. The sim-side node is never
        # spun because it has nothing to spin: publishing does not need an
        # executor, and giving it one would be the first step towards it
        # having a subscription.
        self.executor = SingleThreadedExecutor(context=self.rover_context)
        self.executor.add_node(self.rover_node)

    def spin(self) -> None:
        self.executor.spin()

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.rover_node.destroy_node()
        self.sim_node.destroy_node()
        rclpy.shutdown(context=self.rover_context)
        rclpy.shutdown(context=self.sim_context)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="One-way ROS domain bridge")
    parser.add_argument("--rover-domain", type=int, default=0,
                        help="the domain the rover's graph is on (default 0)")
    parser.add_argument("--sim-domain", type=int, default=42,
                        help="the domain the simulation runs on (default 42)")
    parser.add_argument("--topic", action="append", metavar="TOPIC:TYPE",
                        help=("a topic to carry, repeatable. Defaults to "
                              + ", ".join(DEFAULT_TOPICS)))
    return parser


def main(argv=None) -> int:
    # parse_known_args, because ros2 launch appends --ros-args to every node
    # it starts and argparse would reject them.
    args, _ = build_arg_parser().parse_known_args(
        sys.argv[1:] if argv is None else argv)

    bridge = SimBridge(args.topic or DEFAULT_TOPICS,
                       args.rover_domain, args.sim_domain)
    print(f"[sim_bridge] domain {args.rover_domain} -> domain {args.sim_domain}, "
          f"one way: {', '.join(bridge.bridged) or '(nothing)'}", flush=True)
    if bridge.skipped:
        print(f"[sim_bridge] not carried: {', '.join(bridge.skipped)}", flush=True)
    try:
        bridge.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

```bash
chmod +x sim/src/navi_sim_bringup/scripts/sim_bridge.py
```

- [ ] **Step 5: Run the tests to verify they pass**

Run:
```bash
bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && \
  colcon build --packages-select navi_sim_bringup && \
  colcon test --packages-select navi_sim_bringup && colcon test-result --verbose'
```
Expected: PASS, 7 tests. `test_a_message_on_the_rover_domain_appears_on_the_sim_domain` is the slowest (DDS discovery); if it times out, raise its 10 s deadline before suspecting the bridge — but check first that `bridge.bridged == ["/bridge_test"]`, because a skipped topic would produce exactly the same silence.

- [ ] **Step 6: Commit**

```bash
git add sim/src/navi_sim_bringup/scripts/sim_bridge.py \
        sim/src/navi_sim_bringup/test/test_sim_bridge.py \
        sim/src/navi_sim_bringup/CMakeLists.txt sim/src/navi_sim_bringup/package.xml
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Bridge the rover's topics onto the simulation's domain, one way

Two rclpy contexts in one process, since neither machine has domain_bridge.
The direction is structural rather than a rule: the sim-side node has
publishers and nothing else - no subscriptions and no executor - so the
simulation's /clock, /tf and /robot_description cannot reach the rover's
graph even by accident. Message types are resolved at runtime so the bridge
still comes up on a laptop without grid_map_msgs."
```

---

### Task 10: `--mode semi` — one flag that puts it all together

`start_sim.sh --mode semi` must: put the whole launch on the simulation's own `ROS_DOMAIN_ID`, expand the xacro without `planar_move`, give `sim_ik_node` its `pose_topic`, and start `sim_bridge`. `--mode simulation` (the default) must produce exactly what runs today.

**Files:**
- Modify: `sim/src/navi_sim_bringup/launch/sim.launch.py:61-81` (`_robot_description` and the head of `_world_with_mesh`), `:82-131` (the returned actions), `:134-149` (`generate_launch_description`)
- Modify: `start_sim.sh:1-16` (usage), `:24-35` (arguments), `:95-115` (cleanup), `:117-127` (build and launch)

**Interfaces:**
- Consumes: the xacro's `planar_move` argument (Task 6), `sim_ik_node`'s `pose_topic` (Task 8), `sim_bridge.py` (Task 9).
- Produces:
  - `sim.launch.py` arguments `mode` (`simulation` | `semi`, default `simulation`), `sim_domain` (default `42`), `rover_domain` (default `0`), alongside the existing `map_mesh` and `twist_topic`.
  - `./start_sim.sh [--mode semi|simulation] [--sim-domain N] [--rover-domain N] [--twist-topic T] [--map-mesh P] [--keep-stale]`.

- [ ] **Step 1: Teach the launch file about the two modes**

In `sim/src/navi_sim_bringup/launch/sim.launch.py`, change `_robot_description`'s signature and command (lines 20 and 34):

```python
def _robot_description(xacro_path, planar_move: bool):
```
(the existing docstring, from `"""Expands the xacro into a URDF string, or raises.` down to `got none."""`, stays exactly as it is), and then the command on line 34:

```python
    # planar_move:= is passed explicitly in both modes rather than relying on
    # the xacro default, so the expansion this function produces is a
    # function of its arguments alone and reading the caller is enough to
    # know which plugins are in the model.
    command = ["xacro", xacro_path, f"planar_move:={'true' if planar_move else 'false'}"]
```
The three failure branches below it (`FileNotFoundError`, `CalledProcessError`, the empty-description check) are unchanged.

Replace the head of `_world_with_mesh` (lines 62-80):

```python
    share = get_package_share_directory("navi_sim_bringup")
    mesh = LaunchConfiguration("map_mesh").perform(context)
    twist_topic = LaunchConfiguration("twist_topic").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    sim_domain = int(LaunchConfiguration("sim_domain").perform(context))
    rover_domain = int(LaunchConfiguration("rover_domain").perform(context))

    if mode not in ("simulation", "semi"):
        raise RuntimeError(
            f"mode must be 'simulation' or 'semi', not {mode!r}.\n"
            "  simulation - the IK-driven, dead-reckoned rover on whatever "
            "domain this process is on. What has always run here.\n"
            "  semi       - the body pose comes from the rover's own "
            "/localization/pose, bridged in from domain "
            f"{rover_domain}. Run it on its own ROS_DOMAIN_ID.")

    # The one place the two modes differ, named once. Everything below reads
    # this rather than re-testing the string, so a third mode cannot be
    # half-added.
    external_pose = mode == "semi"

    if not os.path.exists(mesh):
        raise RuntimeError(
            f"map mesh not found: {mesh}\n"
            "Pass map_mesh:=/path/to/Model3D_mesh2.obj - the mesh is "
            "gitignored, so it is not in the repository.")

    source = os.path.join(share, "worlds", "site.world")
    with open(source) as handle:
        world = handle.read().replace("MAP_MESH_PATH", mesh)

    generated = os.path.join(tempfile.mkdtemp(prefix="navi_sim_world_"), "site.world")
    with open(generated, "w") as handle:
        handle.write(world)

    robot = os.path.join(share, "urdf", "asterope_sim.urdf.xacro")
    # planar_move and sim_ik_node's set_entity_state would both be writing
    # the model's pose every tick. Two writers of one pose fight.
    description = _robot_description(robot, planar_move=not external_pose)
```

Replace the returned list (lines 82-131) — the Gazebo, `robot_state_publisher` and `spawn_entity.py` actions keep their bodies and their comments verbatim, so only the assembly changes:

```python
    actions = [
        ExecuteProcess(
            # Keep the existing 24-line comment block here verbatim (the one
            # beginning "publish_rate is libgazebo_ros_init's /clock rate",
            # sim.launch.py:84-106). It explains why 100 Hz and why the long
            # --param, and both are still exactly as load-bearing.
            cmd=["gazebo", "--verbose", generated,
                 "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so",
                 "--ros-args", "--param", "publish_rate:=100.0"],
            output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}],
             output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", "asterope",
                        "-z", "0.05"],
             output="screen"),
    ]

    # Keep the existing 9-line comment block here verbatim (the one beginning
    # "use_sim_time matters here beyond timestamp cosmetics",
    # sim.launch.py:119-126) - joint_pose_trajectory still compares each
    # trajectory's stamp against the simulation clock, in both modes.
    ik_parameters = {"use_sim_time": True}
    if external_pose:
        ik_parameters.update({
            "pose_topic": "/localization/pose",
            "status_topic": "/localization/status",
            # The name spawn_entity.py above gives the model. Set together
            # with it or set_entity_state addresses a model that is not there.
            "model_name": "asterope",
            "pose_z_offset": 0.05,
        })
    actions.append(
        Node(package="navi_sim_ik", executable="sim_ik_node", output="screen",
             parameters=[ik_parameters],
             remappings=[("/manual_twist", twist_topic)]))
    actions.append(
        Node(package="navi_sim_video", executable="sim_video_sender", output="screen"))

    if external_pose:
        # The twist entry follows twist_topic: with --twist-topic pointed at
        # a scratch topic, the bridge has to carry that one or the wheels in
        # the picture never move.
        bridged = [f"{twist_topic}:geometry_msgs/msg/Twist",
                   "/localization/pose:nav_msgs/msg/Odometry",
                   "/localization/status:std_msgs/msg/String",
                   "/localization/map:grid_map_msgs/msg/GridMap"]
        arguments = ["--rover-domain", str(rover_domain),
                     "--sim-domain", str(sim_domain)]
        for spec in bridged:
            arguments += ["--topic", spec]
        # Arguments rather than parameters: the bridge builds its two
        # contexts before it could own a node to read parameters from, and
        # a node that has to exist before it can be configured is the wrong
        # shape for this.
        actions.append(
            Node(package="navi_sim_bringup", executable="sim_bridge.py",
                 name="sim_bridge", arguments=arguments, output="screen"))

    return actions
```

and add the three arguments in `generate_launch_description` (after the `twist_topic` declaration, line 147):

```python
        DeclareLaunchArgument(
            "mode",
            default_value="simulation",
            description=(
                "'simulation' (default): the IK-driven, dead-reckoned rover, "
                "moved by planar_move - what has always run here. 'semi': the "
                "body pose comes from the rover's own /localization/pose, "
                "bridged in from the rover's domain, planar_move is not "
                "loaded, and the model is placed through "
                "/gazebo/set_entity_state.")),
        DeclareLaunchArgument(
            "sim_domain",
            default_value="42",
            description=(
                "The ROS domain this simulation is expected to be running on, "
                "for sim_bridge's sim-side context. Set ROS_DOMAIN_ID to the "
                "same value for the launch itself - start_sim.sh does both.")),
        DeclareLaunchArgument(
            "rover_domain",
            default_value="0",
            description=(
                "The ROS domain the rover's graph is on, for sim_bridge's "
                "rover-side context. 0 in the field; a throwaway domain when "
                "testing against mock/fake_localization.py.")),
```

- [ ] **Step 2: Check both modes launch**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && \
  colcon build --packages-select navi_sim_bringup'
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  ros2 launch navi_sim_bringup sim.launch.py --show-args'
```
Expected: five arguments listed — `map_mesh`, `twist_topic`, `mode`, `sim_domain`, `rover_domain` — with the defaults above.

```bash
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && \
  ros2 launch navi_sim_bringup sim.launch.py mode:=nonsense \
  map_mesh:=$HOME/star/Navi/Model3D_mesh2.obj'
```
Expected: the launch fails immediately with the three-line "mode must be 'simulation' or 'semi'" message. It must not come up with a half-configured simulation.

- [ ] **Step 3: Add the flags to `start_sim.sh`**

Replace the usage block (lines 1-16):

```bash
#!/usr/bin/env bash
# Launch the Gazebo rover simulation.
#
#   ./start_sim.sh                              build if needed, then launch
#   ./start_sim.sh --mode semi                  place the rover from the rover's
#                                               own /localization/pose, on its
#                                               own ROS domain (default 42)
#   ./start_sim.sh --mode semi --sim-domain 7   ... on domain 7 instead
#   ./start_sim.sh --mode semi --rover-domain 91  read the rover's topics off
#                                               domain 91 instead of 0 (for
#                                               mock/fake_localization.py)
#   ./start_sim.sh --keep-stale                 don't clean up a previous run first
#   ./start_sim.sh --twist-topic /sim_test_twist   drive the sim from a scratch
#                                                topic instead of /manual_twist,
#                                                which drives the physical rover
#   ./start_sim.sh --map-mesh /path/to/mesh.obj
#
# Two modes, and the difference is where the rover in the picture comes from:
#
#   simulation (default) - the pose is integrated from the commanded twist.
#       No localisation, drifts without bound, runs on this process's domain.
#       The ground station's Simulation mode, marked DEAD RECKONING.
#   semi - the pose is the rover's own, read from /localization/pose on the
#       rover's domain and carried across by sim_bridge. The simulation runs
#       on its own ROS domain so its /clock, /tf and /robot_description never
#       land on the rover's graph, and nothing goes back the other way.
#       The ground station's Semi-autonomous mode.
#
# Streams its chase camera to the ground station over UDP 5601 - a different
# port than the rover's own 5600, so the two senders can never contend and
# decode each other's late packets as garbage.
#
# The ground station counterpart is ./start_ground_station.sh.
```

Replace the argument block (lines 24-35):

```bash
TWIST_TOPIC="${TWIST_TOPIC:-/manual_twist}"
MAP_MESH="${MAP_MESH:-$REPO_DIR/Model3D_mesh2.obj}"
MODE="${MODE:-simulation}"
SIM_DOMAIN="${SIM_DOMAIN:-42}"
ROVER_DOMAIN="${ROVER_DOMAIN:-0}"

CLEAN_STALE=1
while true; do
    case "${1:-}" in
        --keep-stale) CLEAN_STALE=0; shift ;;
        --twist-topic) TWIST_TOPIC="$2"; shift 2 ;;
        --map-mesh) MAP_MESH="$2"; shift 2 ;;
        --mode) MODE="$2"; shift 2 ;;
        --sim-domain) SIM_DOMAIN="$2"; shift 2 ;;
        --rover-domain) ROVER_DOMAIN="$2"; shift 2 ;;
        *) break ;;
    esac
done

case "$MODE" in
    simulation|semi) ;;
    *) echo "error: --mode must be 'simulation' or 'semi', not '$MODE'" >&2; exit 2 ;;
esac

for pair in "sim domain:$SIM_DOMAIN" "rover domain:$ROVER_DOMAIN"; do
    value="${pair#*:}"
    if ! [[ "$value" =~ ^[0-9]+$ ]] || [ "$value" -gt 232 ]; then
        echo "error: ${pair%%:*} must be a whole number from 0 to 232, not '$value'" >&2
        exit 2
    fi
done

if [ "$MODE" = "semi" ] && [ "$SIM_DOMAIN" = "$ROVER_DOMAIN" ]; then
    # The whole point of semi mode is that the simulation is not on the
    # rover's domain: /clock at 100 Hz, a /tf tree and a second
    # /robot_description would all land there.
    echo "error: --sim-domain and --rover-domain are both $SIM_DOMAIN;" >&2
    echo "       in semi mode the simulation must have a domain of its own." >&2
    exit 2
fi
```

- [ ] **Step 4: Clean up the bridge too, and launch with the domain set**

In the cleanup block, after the `sim_video_sender` line (line 108):

```bash
    # A stale bridge holds two DDS contexts and would keep republishing the
    # rover's topics onto the sim domain alongside the new one's.
    kill_stale "simulation domain bridges" pattern "sim_bridge\.py"
```

Replace the launch block (lines 121-127):

```bash
echo "sim -> Gazebo world with the rover in the scanned site (mode: $MODE, twist topic: $TWIST_TOPIC)"
if [ "$MODE" = "semi" ]; then
    echo "     -> rover placed from /localization/pose, read off domain $ROVER_DOMAIN"
    echo "     -> simulation on ROS_DOMAIN_ID=$SIM_DOMAIN; nothing goes back the other way"
else
    echo "     -> rover placed by dead reckoning from the commanded twist"
fi
echo "     -> chase camera streaming to UDP 5601 once the ground station shows it"

LAUNCH="ros2 launch navi_sim_bringup sim.launch.py \
  map_mesh:='$MAP_MESH' twist_topic:='$TWIST_TOPIC' mode:='$MODE' \
  sim_domain:='$SIM_DOMAIN' rover_domain:='$ROVER_DOMAIN'"

# The domain is exported for the launch, not for the build: colcon does not
# care, and exporting it earlier would only widen the window in which a
# stray ros2 command in this script talked to the wrong graph. In simulation
# mode the environment is left exactly as it was, so that mode is unchanged.
#
# Not exec until here on purpose is fine: nothing else was started in this
# script that a trap would need to tear down - ros2 launch is the last and
# only long-running process, so replacing this shell with it costs nothing.
if [ "$MODE" = "semi" ]; then
    exec bash -c "source '$ROS_SETUP' && source '$SIM_DIR/install/setup.bash' && \
      export ROS_DOMAIN_ID='$SIM_DOMAIN' && $LAUNCH"
else
    exec bash -c "source '$ROS_SETUP' && source '$SIM_DIR/install/setup.bash' && $LAUNCH"
fi
```

- [ ] **Step 5: Check the argument handling without launching anything**

```bash
cd /home/ole/star/Navi
./start_sim.sh --mode nonsense                       # expect: exit 2, "must be 'simulation' or 'semi'"
./start_sim.sh --mode semi --sim-domain 999          # expect: exit 2, "0 to 232"
./start_sim.sh --mode semi --sim-domain 0            # expect: exit 2, "a domain of its own"
```
Expected: each exits 2 with its own message and starts no build and no Gazebo. `echo $?` after each to confirm.

- [ ] **Step 6: Check that Simulation mode is unchanged**

```bash
./start_sim.sh --twist-topic /sim_test_twist
```
Expected: exactly today's behaviour — Gazebo comes up, `sim_ik_node` logs `dead-reckoning mode: ...` and the wheel mapping, `/sim_odom` has `frame_id: odom`, and in another terminal:
```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list | grep -c localization'
bash -c 'source /opt/ros/humble/setup.bash && ros2 node list | grep sim_bridge'
```
Expected: `0` localisation topics and no `sim_bridge` node. Then drive it and watch it move:
```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 topic pub -r 10 /sim_test_twist \
  geometry_msgs/msg/Twist "{linear: {x: 0.4}}"'
```
Expected: the rover drives forward in the Gazebo window, as before. Ctrl-C both.

- [ ] **Step 7: Commit**

```bash
git add sim/src/navi_sim_bringup/launch/sim.launch.py start_sim.sh
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Add --mode semi: the simulation on its own domain, placed by the rover

One flag now does the four things semi-autonomous mode needs - the launch
on its own ROS_DOMAIN_ID, the xacro expanded without planar_move,
sim_ik_node given its pose_topic, and sim_bridge started - and refuses
loudly rather than half-configuring itself when the domains collide or the
mode is misspelt. --mode simulation is the default and is unchanged."
```

---

### Task 11: End to end on the laptop — the rover in Gazebo walks the square

The spec's last test for this sub-project: "`mock/ros_bridge.py` gains a fake `/localization/pose` that walks a square; Gazebo's rover walks the same square, checked by reading `/gazebo/model_states`." Everything needed exists after Tasks 5-10; this task runs it, checks it, and writes down what was measured.

It runs entirely on throwaway domains — 91 for "the rover", 92 for the simulation. Domain 0 is never touched, and `/manual_twist` is never published to.

**Files:**
- Modify: `PROJECT_SUMMARY.md` (the modes and how to run them)
- No new code. If a check here fails, the fix belongs in the task that owns the code, not in a workaround here.

**Interfaces:**
- Consumes: every task above.
- Produces: no code. A recorded closure error, and a documented way to run the whole thing.

- [ ] **Step 1: Build everything and run every test**

```bash
cd /home/ole/star/Navi
.venv/bin/pytest tests/ -q
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build && \
  colcon test && colcon test-result --verbose'
```
Expected: both green. Do not continue past a failure here — a live check on top of a failing unit test tells you nothing about which of the two is wrong.

- [ ] **Step 2: Start the fake rover**

```bash
bash -c 'source /opt/ros/humble/setup.bash && \
  ROS_DOMAIN_ID=91 python3 mock/fake_localization.py'
```
Expected: `fake localisation: a 4.0 m square at 0.5 m/s, state OK, on domain 91`. Leave it running.

- [ ] **Step 3: Start the simulation in semi mode against it**

```bash
./start_sim.sh --mode semi --sim-domain 92 --rover-domain 91 \
               --twist-topic /sim_test_twist
```
Expected, in the launch output:
- `sim -> ... (mode: semi, twist topic: /sim_test_twist)` and the two domain lines.
- `[sim_bridge] domain 91 -> domain 92, one way: /sim_test_twist, /localization/pose, /localization/status`
- `[sim_bridge] not carried: /localization/map` (grid_map_msgs is not installed until sub-project 3 — this line is expected, not a fault).
- `sim_ik_node`: `external pose mode: body pose from /localization/pose, gated on /localization/status, model 'asterope' placed through /gazebo/set_entity_state at up to 30.0 Hz (z offset 0.050). planar_move must NOT be loaded ...`

and in the Gazebo window, the rover walking a square.

- [ ] **Step 4: Check Gazebo's model against the pose it was given**

```bash
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
  ros2 topic echo /localization/pose --once --field pose.pose.position'
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=92 \
  ros2 topic echo /gazebo/model_states --once'
```
Expected: `model_states` names `ground`, `site_scan` and `asterope`; `asterope`'s x and y match `/localization/pose`'s to within the distance the square advances between the two commands (0.5 m/s, so a second apart is 0.5 m — run them back to back, or run each three times and compare the trend). `asterope`'s z is `0.05`.

Take four samples, one near each corner (the lap is 32 s, so roughly every 8 s), and record them. The x/y pairs must approach `(0,0)`, `(4,0)`, `(4,4)`, `(0,4)`.

- [ ] **Step 5: Check that nothing came back**

```bash
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 ros2 topic list'
```
Expected: `/localization/pose`, `/localization/status`, `/sim_test_twist`, `/parameter_events`, `/rosout` — and **none** of `/clock`, `/tf`, `/tf_static`, `/robot_description`, `/gazebo/model_states`, `/sim_odom`, `/set_joint_trajectory`. This is the spec's decision, verified. If any of them is there, the bridge has grown a subscription on its sim-side node and Task 9's `test_the_sim_side_node_has_no_subscriptions_at_all` should have caught it — fix it there.

```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'
```
Expected (domain 0, the rover's): whatever was there before this test, and nothing from either side of it.

- [ ] **Step 6: Check the wheels still turn and the body still comes from outside**

```bash
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 ros2 topic pub -r 10 \
  /sim_test_twist geometry_msgs/msg/Twist "{linear: {x: 0.4}}"'
```
Expected: the wheels in the Gazebo window turn and the steering swings, but the body keeps walking the square and does **not** drive off along it — the twist reaches the IK through the bridge and drives the joints, while the body pose stays the rover's. That split is the whole design of the mode. Ctrl-C the publisher.

- [ ] **Step 7: Check the model holds still while localisation is lost**

Stop `mock/fake_localization.py` and restart it as:
```bash
bash -c 'source /opt/ros/humble/setup.bash && \
  ROS_DOMAIN_ID=91 python3 mock/fake_localization.py --state SEARCHING'
```
Expected: `sim_ik_node` logs `localisation reports 'SEARCHING', not OK - holding the model still and ignoring poses on /localization/pose until it recovers.` — once, not once per pose — and `/gazebo/model_states` stops changing. Restart the fixture with `--state OK`: the log says `localisation is OK again - the model follows the pose` and the rover resumes.

- [ ] **Step 8: Check the ground station's half against the same square**

In two more terminals:
```bash
.venv/bin/python mock/ros_bridge.py --port 9090
```
```bash
./start_ground_station.sh localhost 9090
```
Expected, in **Semi-autonomous**:
- the header reads `LOC: x … y … yaw …`, updating about five times a second and walking the same square;
- the panel title reads `CAMERA / SIMULATION  -  LOCALISED`;
- pressing the video button leaves the picture alone and puts `no camera stream in semi-autonomous mode` on the status line for five seconds;
- switching to **Simulation** puts the `DEAD RECKONING, NO LOCALISATION` marker back;
- switching to **Manual** points the panel at port 5600 and asks the rover for video (which fails against the mock, and says so — that is correct);
- **Autonomous** cannot be clicked and its tooltip reads `not implemented`.

Note that the mock rosbridge's square and `fake_localization.py`'s square are two independent clocks — they walk the same shape, not the same phase. Being in step is not part of this check.

- [ ] **Step 9: Record what was measured, and how to run it**

Add to `PROJECT_SUMMARY.md`, in the section that describes the ground station's modes:

```markdown
### Modes

| Mode | Panel | Rover in the picture | Rover's camera |
|---|---|---|---|
| Manual | rover camera, UDP 5600 | — | streaming on request |
| Semi-autonomous | Gazebo chase camera, UDP 5601 | `/localization/pose` from the rover | stopped, and requests refused |
| Autonomous | — | — | — (not implemented) |
| Simulation | Gazebo chase camera, UDP 5601 | dead reckoning from the commanded twist | stopped |

The twist reaches the rover in every mode. A mode switch changes what is on
screen and nothing else.

Semi-autonomous needs the simulation running in its matching mode:

    ./start_sim.sh --mode semi            # rover on domain 0, sim on domain 42
    ./start_ground_station.sh

and, with no rover, the whole thing runs on this laptop on throwaway domains:

    bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 python3 mock/fake_localization.py'
    ./start_sim.sh --mode semi --sim-domain 92 --rover-domain 91 --twist-topic /sim_test_twist
    .venv/bin/python mock/ros_bridge.py
    ./start_ground_station.sh localhost 9090
```

- [ ] **Step 10: Commit**

```bash
git add PROJECT_SUMMARY.md
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Write down the four modes and how to run the whole chain on this laptop

Checked end to end on throwaway domains 91 and 92: the fake rover walks a
4 m square, Gazebo's rover walks the same square (verified against
/gazebo/model_states), the wheels keep turning from the bridged twist while
the body comes from localisation, the model holds still while the status
says SEARCHING, and domain 91 sees nothing of the simulation - no /clock,
no /tf, no /robot_description."
```

---

## Notes for whoever runs this

- **Do not touch `/manual_twist`.** Every check in this plan uses `--twist-topic /sim_test_twist` or a throwaway domain. `start_sim.sh`'s default is `/manual_twist` because in the field that is right; on a desk it is a rover moving in a workshop.
- **`sim_bridge` skipping `/localization/map` is expected** until sub-project 3 installs `grid_map_msgs`. The warning line is the design working, not a fault.
- **Semi-autonomous mode has no camera by design.** If an operator asks why the picture is Gazebo and not the ZED, the answer is on the panel.
- The `/sim_odom` topic still exists in semi mode and is now the localised pose in the `map` frame. It is a diagnostic, not an input to anything.
- `/sim_cmd_vel` also still exists in semi mode with nothing subscribed (no `planar_move`). Left alone deliberately: it is the IK's own account of the body velocity it achieved, and it is worth being able to plot against the localised pose.
