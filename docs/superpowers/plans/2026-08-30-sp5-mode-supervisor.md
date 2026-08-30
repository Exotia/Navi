# SP5: Mode Supervisor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One node on the Orin — `mode_supervisor` — becomes the only thing that publishes to the chassis, owns the mode (`manual` / `semi_auto` / `autonomous` / `estop`), owns the deadman, and latches an e-stop that survives the field link dying; and the ground station stops streaming `/manual_twist` in modes where that stream would drown out autonomy.

**Architecture:** A new ament_python package `rover/src/navi_supervisor` holds a pure state machine (`supervisor_state.py`, no ROS, fake-clock testable) and the rclpy node around it (`mode_supervisor.py`), which subscribes `/manual_twist`, `/autonomy_twist`, `/mode_request`, `/estop_request` and `/localization/status`, publishes `/rover_twist` at 20 Hz and `/mode_status` at 2 Hz, and reaches the primary only through the command topic `bema_bridge` already owns (`/drive_command`) plus a `Nav2Control` stub that SP9 fills. `bema_bridge` is re-pointed from `/manual_twist` to `/rover_twist` and otherwise unchanged, keeping its own 1 s deadman as the second one in the series. On the ground station, a `/mode_status` subscription gates the `/manual_twist` publish and STOP gains a second, latching destination on `/estop_request`.

**Tech Stack:** Python 3.10, rclpy (ROS 2 Humble), colcon, `msgpack`, PySide6, roslibpy, pytest.

**Spec:** `docs/superpowers/specs/autonomy-plan.md`

## Global Constraints

- **`/manual_twist` drives the physical rover. No test may ever publish to `/manual_twist`.** Node tests set a throwaway `ROS_DOMAIN_ID` in the module before importing rclpy (91 for `navi_supervisor`, 93 is already taken by `test_bema_bridge.py`); a bench harness that needs a twist on the wire uses `/sim_test_twist`.
- Nothing under `ground_station/` imports `rclpy`. `ground_station/models.py` imports neither Qt nor ROS — every new parse/format helper goes there.
- Ground-station speed caps stay exactly `MAX_LINEAR_SPEED = 0.05` m/s and `MAX_ANGULAR_SPEED = 0.1` rad/s in `ground_station/gamepad_input.py`. This plan does not touch that file.
- `sim/src/navi_sim_ik/vendor/` is read-only. This plan touches nothing under `sim/`.
- `bema_bridge` keeps its own 1 s deadman. Two deadmen in series is deliberate (spec §4, last paragraph): the supervisor protects against Nav2 hanging, the bridge against the whole Orin-side graph dying.
- Only `mode_supervisor` publishes `/rover_twist`. There is no `twist_mux`.
- Nothing on `a_primary` or the ZED workspace is created, modified, started or stopped. This plan edits files under `rover/`, `ground_station/` and `tests/` only.
- The full ground-station suite must stay green: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/ -q` — **249 tests collected** at the time of writing (the spec brief's "233" predates the video and map rows; use the collected count, and it must only ever go up in this plan).
- Every task ends green and commits. Commits use
  `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`
  with the trailer:
  ```
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
  ```
  Never push.

### Design decisions (made here, with the reason)

1. **`mode_supervisor` lives in a new package `rover/src/navi_supervisor/`,** not as a module in `navi_teleop`. Reason: `navi_teleop` is the teleoperation bridge and its `package.xml` says so; the arbiter that decides what reaches the wheels must be startable, stoppable and testable without dragging in the msgpack RPC stack, and SP8/SP9 will add to it.
2. **Every new topic is JSON in a `std_msgs/String`,** matching `/drive_status`, `/drive_command`, `/localization/status`, `/map_status` and `/video_request`. Reason: the ground station reads these over rosbridge with no ROS installed, and a custom `.msg` would cost an `ament_cmake` package per message. `/rover_twist`, `/manual_twist` and `/autonomy_twist` stay `geometry_msgs/Twist` — they are rover-internal and already typed.
3. **`bema_bridge` is re-pointed by configuration and by default.** Its existing `twist_topic` parameter's default changes from `/manual_twist` to `/rover_twist`, *and* `rover/start_navi.sh` passes `-p twist_topic:=/rover_twist` explicitly. Reason: the default must be the safe wiring so that a bridge started by hand cannot be driven around the supervisor, and the launcher must still state the wiring out loud so it can be read without opening the node.
4. **The Nav2 hook is `navi_supervisor/nav2_control.py`, class `Nav2Control` with `cancel_goal(reason)` and `deactivate(reason)`, and the stub `NullNav2Control` that records calls in `.calls` and logs.** SP9 replaces the stub, not the interface. Reason: the *decision* to cancel is the safety-critical half and is finished and tested now; the two methods return nothing, because the supervisor must never wait on Nav2 in order to stop the rover.
5. **The ground station learns the mode by subscribing `/mode_status` over rosbridge, and the publish gate sits in `MainWindow._poll_gamepad`,** using the pure predicate `ground_station.models.may_publish_manual_twist(state)`. Reason: `_poll_gamepad` is the single place `/manual_twist` is published, and putting the rule in `models.py` keeps it testable without Qt.

### Rulings on spec ambiguities (binding for this plan)

- **Localisation loss halts autonomy only.** Spec rule 3 zeroes the twist on `SEARCHING`/`OFF`; it does not say manual driving stops. It must not: the operator's way of recovering a rover whose VIO died is to drive it by hand. So in `manual`/`semi_auto` the localisation state is reported in `/mode_status` and the twist passes through unchanged.
- **Localisation loss drops the mode to `manual`, not to a muted `autonomous`.** "No automatic resume" plus rule 5 (the GS does not stream `/manual_twist` in `autonomous`) means a rover left in `autonomous` after a halt is a rover the operator cannot drive at all. Dropping to `manual` is what re-opens the manual stream. The retained manual twist is cleared at the same moment so the rover does not lurch on a stale command.
- **The autonomy deadman does not change the mode.** The table says 0.5 s of Nav2 silence gives "zero + stop", not "end the run". A Nav2 hiccup zeroes the output and recovers by itself when Nav2 speaks again; only localisation loss, an e-stop or a takeover latch a mode change.
- **In `autonomous` the ground station publishes `/manual_twist` only above the deadzone, not nothing at all.** Spec rule 5 says the ground station publishes nothing in `autonomous` "so rule 1 is a real signal rather than a constant stream" — the intent is that the constant zero stream dies, not that takeover becomes unreachable. Full silence would make rule 1 dead code: the ground station is the only publisher of `/manual_twist`, so nothing above the deadzone could ever arrive while autonomous. So: in `manual`/`semi_auto` the stream is continuous as today; in `autonomous` a deflected stick is published and a centred one is not; in `estop` and in a mode this build does not know, nothing is published. "Above the deadzone" is not a new threshold — `GamepadReader.read_twist()` has already run every axis through `_apply_deadzone(gamepad_input.DEADZONE = 0.1)` before scaling, so a centred stick is exactly `(0.0, 0.0, 0.0)` and the smallest deflection it can report is `DEADZONE * MAX_LINEAR_SPEED` = 0.005 m/s (0.01 rad/s), comfortably above the supervisor's `TAKEOVER_LINEAR_MPS` of 0.002. A non-zero component *is* past the deadzone.
- **An `/estop_request` whose payload will not parse still stops the rover.** The JSON is parsed only to recover a reason string; a parse failure logs and stops anyway.
- **`/rover_twist` is published continuously, zeros included, and the chassis stop is edge-triggered.** The bridge's deadman therefore never fires while the supervisor is alive — which is exactly what spec §4 assigns it ("the bridge against the whole Orin-side graph dying"). To actually stop the wheels the supervisor sends `{"action": "stop"}` on `/drive_command` once, on the transition into a stopped state. Note the consequence: the bridge's own STOP latch (`_on_command` clearing `_twist_at`) is un-latched by the supervisor's very next zero, one tick later. That is harmless — the wheels have already had `F2`, and the supervisor's own latch is what holds them at zero.

---

## File structure

- Create `rover/src/navi_supervisor/package.xml`, `setup.py`, `setup.cfg`, `resource/navi_supervisor`, `navi_supervisor/__init__.py`
- Create `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py` — the rules, no ROS.
- Create `rover/src/navi_supervisor/navi_supervisor/nav2_control.py` — the SP9 hook and its stub.
- Create `rover/src/navi_supervisor/navi_supervisor/mode_supervisor.py` — the rclpy node.
- Create `rover/src/navi_supervisor/test/test_supervisor_state.py`, `test/test_mode_supervisor.py`
- Modify `rover/src/navi_teleop/navi_teleop/bema_session.py` — add `abort()`.
- Modify `rover/src/navi_teleop/navi_teleop/bema_bridge.py` — `abort` action, `twist_topic` default.
- Modify `rover/src/navi_teleop/test/fake_bema_server.py` — coordinator `F7` = abort → Idle.
- Modify `rover/src/navi_teleop/test/test_bema_bridge.py`, `rover/src/navi_teleop/package.xml`
- Modify `rover/start_navi.sh`
- Modify `ground_station/models.py`, `ground_station/ros_client.py`, `ground_station/ui/drive_row.py`, `ground_station/ui/main_window.py`
- Modify `tests/test_models.py`, `tests/test_ros_client.py`, `tests/test_drive_row.py`, `tests/test_main_window.py`

---

## Task 1: The supervisor package and its source/deadman core

**Files:**
- Create: `rover/src/navi_supervisor/package.xml`, `rover/src/navi_supervisor/setup.py`, `rover/src/navi_supervisor/setup.cfg`, `rover/src/navi_supervisor/resource/navi_supervisor`, `rover/src/navi_supervisor/navi_supervisor/__init__.py`, `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py`
- Test: `rover/src/navi_supervisor/test/test_supervisor_state.py`

**Interfaces:**
- Produces: `navi_supervisor.supervisor_state.SupervisorState` with `on_manual_twist(now, vx, vy, wz)`, `on_autonomy_twist(now, vx, vy, wz)`, `output(now) -> (vx, vy, wz)`, `deadman_active(now) -> bool`, `status(now) -> dict`, `take_actions() -> list[str]`, and the module constants `MANUAL`, `SEMI_AUTO`, `AUTONOMOUS`, `ESTOP`, `MANUAL_DEADMAN_S`, `AUTONOMY_DEADMAN_S`, `CHASSIS_STOP`.
- Consumes: nothing. No ROS, no project imports.

### Steps

- [ ] Write the failing test at `rover/src/navi_supervisor/test/test_supervisor_state.py`:

```python
"""The supervisor's rules, with time passed in rather than read - the same
fake-clock shape test_bema_session.py uses. No ROS here."""

from navi_supervisor.supervisor_state import (AUTONOMOUS, AUTONOMY_DEADMAN_S,
                                              CHASSIS_STOP, ESTOP, MANUAL,
                                              MANUAL_DEADMAN_S, SEMI_AUTO,
                                              SupervisorState)


def test_manual_mode_forwards_the_manual_twist():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.04, 0.0, 0.05)
    assert s.output(1.0) == (0.04, 0.0, 0.05)


def test_semi_auto_forwards_the_manual_twist_too():
    s = SupervisorState(mode=SEMI_AUTO)
    s.on_manual_twist(1.0, 0.03, 0.01, 0.0)
    assert s.output(1.0) == (0.03, 0.01, 0.0)


def test_manual_mode_ignores_the_autonomy_twist():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.01, 0.0, 0.0)
    s.on_autonomy_twist(1.0, 0.4, 0.0, 0.0)
    assert s.output(1.0) == (0.01, 0.0, 0.0)


def test_autonomous_mode_forwards_the_autonomy_twist():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.2)
    assert s.output(1.0) == (0.3, 0.0, 0.2)


def test_no_source_yet_is_the_deadman_not_a_free_pass():
    s = SupervisorState(mode=MANUAL)
    assert s.deadman_active(0.0) is True
    assert s.output(0.0) == (0.0, 0.0, 0.0)


def test_manual_deadman_is_one_second():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    assert s.output(1.0 + MANUAL_DEADMAN_S) == (0.05, 0.0, 0.0)
    assert s.deadman_active(1.0 + MANUAL_DEADMAN_S) is False
    assert s.output(1.0 + MANUAL_DEADMAN_S + 0.01) == (0.0, 0.0, 0.0)
    assert s.deadman_active(1.0 + MANUAL_DEADMAN_S + 0.01) is True


def test_autonomy_deadman_is_half_a_second():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    assert s.output(1.0 + AUTONOMY_DEADMAN_S) == (0.3, 0.0, 0.0)
    assert s.output(1.0 + AUTONOMY_DEADMAN_S + 0.01) == (0.0, 0.0, 0.0)


def test_the_manual_stream_does_not_feed_the_autonomy_deadman():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.4, 0.0, 0.0, 0.0)      # a zero stream, below takeover
    assert s.output(1.6) == (0.0, 0.0, 0.0)


def test_the_deadman_edge_queues_one_chassis_stop_not_one_per_tick():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.output(1.0)
    assert s.take_actions() == []
    s.output(3.0)
    assert s.take_actions() == [CHASSIS_STOP]
    s.output(3.05)
    s.output(3.10)
    assert s.take_actions() == []


def test_a_fresh_twist_after_the_deadman_drives_again():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.output(3.0)
    s.take_actions()
    s.on_manual_twist(3.5, 0.02, 0.0, 0.0)
    assert s.output(3.5) == (0.02, 0.0, 0.0)


def test_estop_mode_is_always_zero_and_always_deadman():
    s = SupervisorState(mode=ESTOP)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    assert s.output(1.0) == (0.0, 0.0, 0.0)
    assert s.deadman_active(1.0) is True


def test_status_names_the_mode_the_source_and_the_age():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    status = s.status(1.25)
    assert status["mode"] == AUTONOMOUS
    assert status["source"] == "/autonomy_twist"
    assert status["deadman_active"] is False
    assert status["estop_latched"] is False
    assert status["source_age_s"] == 0.25
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_supervisor_state.py -q'`
      Expected: collection error, `ModuleNotFoundError: No module named 'navi_supervisor'`.

- [ ] Create `rover/src/navi_supervisor/resource/navi_supervisor` as an empty file:

```
```

- [ ] Create `rover/src/navi_supervisor/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://relaxng.org/ns/structure/1.0"?>
<package format="3">
  <name>navi_supervisor</name>
  <version>0.1.0</version>
  <description>The Asterope rover's mode supervisor: the sole publisher of
  /rover_twist, and the sole owner of the arbitration between the manual and
  autonomy twist streams, of the deadman, and of the latched e-stop.</description>
  <maintainer email="oxe.pxs@gmail.com">star</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] Create `rover/src/navi_supervisor/setup.cfg`:

```
[develop]
script_dir=$base/lib/navi_supervisor
[install]
install_scripts=$base/lib/navi_supervisor
```

- [ ] Create `rover/src/navi_supervisor/setup.py`:

```python
from setuptools import setup

package_name = 'navi_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description="Mode arbitration, deadman and e-stop for the Asterope rover.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_supervisor = navi_supervisor.mode_supervisor:main',
        ],
    },
)
```

- [ ] Create `rover/src/navi_supervisor/navi_supervisor/__init__.py` as an empty file:

```
```

- [ ] Create `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py`:

```python
"""The mode supervisor's rules, with no ROS in them.

Modes are manual, semi_auto, autonomous and estop. This class decides
which twist reaches the chassis, when the deadman fires, when the e-stop
latches, and which side effects the node owes as a result - cancelling the
Nav2 goal, telling the coordinator to abort and re-enter Manual, stopping
the wheels. Time is passed in rather than read, so the whole thing runs
against a fake clock the way bema_session does.

mode_supervisor.py owns the timers and the topics; this file owns the
rules, and is the only place they are written down.
"""

MANUAL = "manual"
SEMI_AUTO = "semi_auto"
AUTONOMOUS = "autonomous"
ESTOP = "estop"

MODES = (MANUAL, SEMI_AUTO, AUTONOMOUS, ESTOP)
MANUAL_MODES = (MANUAL, SEMI_AUTO)

MANUAL_DEADMAN_S = 1.0
AUTONOMY_DEADMAN_S = 0.5

# Above these a /manual_twist counts as the operator taking over. The
# ground station's smallest non-zero output is its gamepad deadzone times
# its speed cap - 0.1 * 0.05 = 0.005 m/s and 0.1 * 0.1 = 0.01 rad/s - so
# these sit a factor of two below any real stick deflection and far above
# the float noise around a zero.
TAKEOVER_LINEAR_MPS = 0.002
TAKEOVER_ANGULAR_RPS = 0.004

ZERO = (0.0, 0.0, 0.0)

# The side effects the node performs on this class's behalf, drained by
# take_actions() in the order they were queued.
CANCEL_GOAL = "cancel_goal"                 # Nav2Control.cancel_goal()
DEACTIVATE_NAV2 = "deactivate_nav2"         # Nav2Control.deactivate()
COORDINATOR_ABORT = "coordinator_abort"     # {"action": "abort"} on /drive_command
COORDINATOR_MANUAL = "coordinator_manual"   # {"action": "manual"}
CHASSIS_STOP = "chassis_stop"               # {"action": "stop"}


class SupervisorState:

    def __init__(self, mode=MANUAL, manual_deadman_s=MANUAL_DEADMAN_S,
                 autonomy_deadman_s=AUTONOMY_DEADMAN_S):
        self._mode = mode if mode in MODES else MANUAL
        self._manual_deadman_s = float(manual_deadman_s)
        self._autonomy_deadman_s = float(autonomy_deadman_s)
        self._manual = ZERO
        self._manual_at = None
        self._autonomy = ZERO
        self._autonomy_at = None
        self._localization = None          # None means none has arrived yet
        self._reason = "startup"
        self._estop_latched = False
        # Whether the chassis has already been told to stop. Starts True so
        # that coming up with no source yet does not send a stop to a
        # chassis that has never been asked to move.
        self._stopped = True
        self._actions = []

    @property
    def mode(self):
        return self._mode

    @property
    def estop_latched(self):
        return self._estop_latched

    # --- inputs ----------------------------------------------------------
    def on_manual_twist(self, now, vx, vy, wz):
        self._manual = (float(vx), float(vy), float(wz))
        self._manual_at = now

    def on_autonomy_twist(self, now, vx, vy, wz):
        self._autonomy = (float(vx), float(vy), float(wz))
        self._autonomy_at = now

    # --- output ----------------------------------------------------------
    def _source_age(self, now):
        """(age of the mode's source, the deadman it is judged against).

        estop has no source: it is not "silent", it is stopped, so it
        returns no age at all and deadman_active answers True directly.
        """
        if self._mode in MANUAL_MODES:
            at, limit = self._manual_at, self._manual_deadman_s
        elif self._mode == AUTONOMOUS:
            at, limit = self._autonomy_at, self._autonomy_deadman_s
        else:
            return None, 0.0
        return (None if at is None else now - at), limit

    def deadman_active(self, now):
        if self._mode == ESTOP:
            return True
        age, limit = self._source_age(now)
        return age is None or age > limit

    def output(self, now):
        """The twist to publish on /rover_twist this tick.

        Called at the publish rate. The live -> stopped edge is where the
        chassis stop is queued: /rover_twist keeps carrying zeros so the
        bridge's own deadman never fires while this node lives, which means
        the wheels only really stop if something says so - once, on the
        edge, rather than twenty times a second.
        """
        if self.deadman_active(now):
            if not self._stopped:
                self._stopped = True
                self._queue(CHASSIS_STOP)
            return ZERO
        self._stopped = False
        if self._mode == AUTONOMOUS:
            return self._autonomy
        return self._manual

    def status(self, now):
        age, _ = self._source_age(now)
        return {
            "mode": self._mode,
            "reason": self._reason,
            "source": (None if self._mode == ESTOP
                       else "/autonomy_twist" if self._mode == AUTONOMOUS
                       else "/manual_twist"),
            "deadman_active": self.deadman_active(now),
            "estop_latched": self._estop_latched,
            "localization_state": self._localization,
            "source_age_s": None if age is None else round(age, 2),
        }

    # --- side effects ----------------------------------------------------
    def _queue(self, *actions):
        # Deduplicated within a batch: two localisation-loss messages in
        # one tick must not abort the coordinator twice.
        for action in actions:
            if action not in self._actions:
                self._actions.append(action)

    def take_actions(self):
        """The side effects owed since the last call, in order, drained."""
        actions, self._actions = self._actions, []
        return actions
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_supervisor_state.py -q'`
      Expected: 12 passed.

- [ ] Commit: `git add rover/src/navi_supervisor && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
navi_supervisor: the package and the source/deadman core

A pure state machine, no ROS: which stream owns /rover_twist in each
mode, the 1 s manual and 0.5 s autonomy deadmen, and the one chassis
stop queued on the live -> stopped edge rather than every tick.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 2: The latched e-stop and the mode requests

**Files:**
- Modify: `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py`
- Test: `rover/src/navi_supervisor/test/test_supervisor_state.py`

**Interfaces:**
- Produces: `SupervisorState.on_estop_request(now, reason)`, `SupervisorState.on_mode_request(now, mode) -> str | None` (the refusal reason, or `None` when honoured).
- Consumes: the module constants from Task 1.

### Steps

- [ ] Append the failing tests to `rover/src/navi_supervisor/test/test_supervisor_state.py` (and extend the import line at the top of the file to
      `from navi_supervisor.supervisor_state import (AUTONOMOUS, AUTONOMY_DEADMAN_S, CANCEL_GOAL, CHASSIS_STOP, COORDINATOR_ABORT, COORDINATOR_MANUAL, DEACTIVATE_NAV2, ESTOP, MANUAL, MANUAL_DEADMAN_S, SEMI_AUTO, SupervisorState)`):

```python
def test_estop_zeroes_the_output_and_latches():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_estop_request(1.1, "ground station STOP")
    assert s.mode == ESTOP
    assert s.estop_latched is True
    assert s.output(1.1) == (0.0, 0.0, 0.0)
    # and it stays stopped however much the operator keeps steering
    s.on_manual_twist(1.2, 0.05, 0.0, 0.0)
    s.on_manual_twist(5.0, 0.05, 0.0, 0.0)
    assert s.output(5.0) == (0.0, 0.0, 0.0)
    assert s.mode == ESTOP


def test_estop_stops_the_chassis_and_cancels_any_goal():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_estop_request(1.1, "ground station STOP")
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP]


def test_estop_reason_reaches_the_status():
    s = SupervisorState(mode=MANUAL)
    s.on_estop_request(1.0, "ground station STOP")
    status = s.status(1.0)
    assert status["mode"] == ESTOP
    assert status["reason"] == "ground station STOP"
    assert status["estop_latched"] is True


def test_only_a_manual_mode_request_clears_the_latch():
    s = SupervisorState(mode=MANUAL)
    s.on_estop_request(1.0, "STOP")
    assert s.on_mode_request(2.0, AUTONOMOUS) is not None
    assert s.mode == ESTOP
    assert s.on_mode_request(2.1, SEMI_AUTO) is not None
    assert s.mode == ESTOP
    assert s.on_mode_request(2.2, MANUAL) is None
    assert s.mode == MANUAL
    assert s.estop_latched is False


def test_clearing_the_latch_does_not_replay_the_twist_that_was_held():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_estop_request(1.1, "STOP")
    s.on_manual_twist(1.2, 0.05, 0.0, 0.0)     # operator still on the stick
    s.on_mode_request(1.3, MANUAL)
    assert s.output(1.3) == (0.0, 0.0, 0.0)    # deadman, not the held twist
    s.on_manual_twist(1.4, 0.05, 0.0, 0.0)
    assert s.output(1.4) == (0.05, 0.0, 0.0)   # a genuinely new one drives


def test_an_unknown_mode_is_refused_and_changes_nothing():
    s = SupervisorState(mode=MANUAL)
    assert s.on_mode_request(1.0, "turbo") is not None
    assert s.mode == MANUAL


def test_a_mode_request_of_estop_is_an_estop():
    s = SupervisorState(mode=AUTONOMOUS)
    assert s.on_mode_request(1.0, ESTOP) is None
    assert s.mode == ESTOP
    assert s.estop_latched is True


def test_leaving_autonomous_by_request_cancels_the_goal():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_mode_request(1.0, MANUAL)
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2,
                                COORDINATOR_ABORT, COORDINATOR_MANUAL]
    assert s.mode == MANUAL


def test_entering_autonomous_from_manual_cancels_nothing():
    s = SupervisorState(mode=MANUAL)
    assert s.on_mode_request(1.0, AUTONOMOUS) is None
    assert s.take_actions() == []
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_supervisor_state.py -q'`
      Expected: 9 failures, `AttributeError: 'SupervisorState' object has no attribute 'on_estop_request'`.

- [ ] In `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py`, insert these two methods in the `# --- inputs ---` section, directly after `on_autonomy_twist`:

```python
    def on_estop_request(self, now, reason=""):
        """Rule 2: STOP is latched and local.

        Nothing here consults the mode: an e-stop from any state is an
        e-stop, and the goal cancel goes out unconditionally rather than
        only when this class believes it is autonomous - a Nav2 that was
        still winding down from a takeover must not survive the stop
        because of a bookkeeping disagreement. The stub cancel is a no-op,
        so the unconditional call costs nothing.
        """
        self._estop_latched = True
        self._mode = ESTOP
        self._reason = str(reason) if reason else "e-stop"
        self._manual = ZERO
        self._manual_at = None
        self._autonomy = ZERO
        self._autonomy_at = None
        self._stopped = True
        self._queue(CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP)

    def on_mode_request(self, now, mode):
        """Honour a /mode_request, or return the reason it was refused."""
        if mode not in MODES:
            return f"unknown mode {mode!r}"
        if mode == ESTOP:
            self.on_estop_request(now, "mode request")
            return None
        if self._estop_latched and mode != MANUAL:
            # Rule 2 again: an explicit request back to manual is the only
            # thing that clears the latch. Anything else leaves the rover
            # stopped, and says so on /mode_status rather than silently.
            self._reason = f"e-stop latched, {mode} refused"
            return self._reason
        was_autonomous = self._mode == AUTONOMOUS
        self._estop_latched = False
        self._mode = mode
        self._reason = "mode request"
        if was_autonomous and mode != AUTONOMOUS:
            # Spec rule 1's ordering, on the path the operator actually
            # uses: the coordinator must be aborted out of the autonomous
            # task before anything asks it for Manual, or ERC's mission
            # state machine is left claiming a run that is over. Same
            # sequence as _take_over(), so the DRIVE row's Manual button
            # and a deflected stick leave the coordinator in one state.
            self._queue(CANCEL_GOAL, DEACTIVATE_NAV2, COORDINATOR_ABORT)
            if mode in MANUAL_MODES:
                self._queue(COORDINATOR_MANUAL)
        if mode in MANUAL_MODES:
            # Whatever manual twist is still retained predates this
            # request - it may be the stick position from before an e-stop.
            # It must not become the first thing the rover does on the way
            # back; the deadman holds zero until a genuinely new one lands.
            self._manual = ZERO
            self._manual_at = None
        return None
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_supervisor_state.py -q'`
      Expected: 21 passed.

- [ ] Commit: `git add rover/src/navi_supervisor && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
navi_supervisor: the latched e-stop and the mode requests

STOP latches until an explicit /mode_request back to manual; a request
that leaves autonomous cancels the goal; the twist held across a latch is
cleared so the rover does not lurch on the way back.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 3: Takeover, localisation loss, and link loss

**Files:**
- Modify: `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py`
- Test: `rover/src/navi_supervisor/test/test_supervisor_state.py`

**Interfaces:**
- Produces: `SupervisorState.on_localization_status(now, state)`; takeover behaviour inside `on_manual_twist`.
- Consumes: `/localization/status`'s `state` field, which `navi_localization/tracker.py` publishes as exactly `"OK"`, `"SEARCHING"` or `"OFF"`.

### Steps

- [ ] Append the failing tests to `rover/src/navi_supervisor/test/test_supervisor_state.py`:

```python
def test_a_stick_above_the_deadzone_takes_over_from_autonomy():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.05, 0.005, 0.0, 0.0)   # the GS's smallest real output
    assert s.mode == MANUAL
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2,
                                COORDINATOR_ABORT, COORDINATOR_MANUAL]
    assert s.status(1.05)["reason"] == "operator takeover"


def test_the_twist_that_took_over_drives_on_the_same_tick():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.05, 0.05, 0.0, 0.0)
    assert s.output(1.05) == (0.05, 0.0, 0.0)


def test_a_rotation_only_stick_takes_over_too():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_manual_twist(1.05, 0.0, 0.0, 0.01)    # the GS's smallest real wz
    assert s.mode == MANUAL


def test_a_zero_manual_stream_does_not_take_over():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    for t in (1.05, 1.10, 1.15, 1.20):
        s.on_manual_twist(t, 0.0, 0.0, 0.0)
    assert s.mode == AUTONOMOUS
    assert s.take_actions() == []
    assert s.output(1.20) == (0.3, 0.0, 0.0)


def test_a_stick_while_already_manual_does_not_abort_anything():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    assert s.take_actions() == []


def test_a_stick_while_estopped_does_not_take_over():
    s = SupervisorState(mode=MANUAL)
    s.on_estop_request(1.0, "STOP")
    s.take_actions()
    s.on_manual_twist(1.1, 0.05, 0.0, 0.0)
    assert s.mode == ESTOP
    assert s.take_actions() == []


def test_localisation_searching_halts_autonomy_and_says_why():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_localization_status(1.1, "SEARCHING")
    assert s.mode == MANUAL
    assert s.take_actions() == [CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP]
    assert s.output(1.1) == (0.0, 0.0, 0.0)
    status = s.status(1.1)
    assert status["reason"] == "localisation SEARCHING"
    assert status["localization_state"] == "SEARCHING"


def test_localisation_off_halts_autonomy_too():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_localization_status(1.1, "OFF")
    assert s.mode == MANUAL
    assert s.status(1.1)["reason"] == "localisation OFF"


def test_localisation_recovering_does_not_resume_autonomy():
    s = SupervisorState(mode=AUTONOMOUS)
    s.on_autonomy_twist(1.0, 0.3, 0.0, 0.0)
    s.on_localization_status(1.1, "OFF")
    s.take_actions()
    s.on_localization_status(2.0, "OK")
    assert s.mode == MANUAL
    assert s.take_actions() == []


def test_localisation_loss_does_not_stop_manual_driving():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_localization_status(1.0, "OFF")
    assert s.mode == MANUAL
    assert s.output(1.0) == (0.05, 0.0, 0.0)
    assert s.status(1.0)["localization_state"] == "OFF"


def test_autonomous_is_refused_while_localisation_is_lost():
    s = SupervisorState(mode=MANUAL)
    s.on_localization_status(1.0, "OFF")
    assert s.on_mode_request(1.1, AUTONOMOUS) is not None
    assert s.mode == MANUAL
    s.on_localization_status(2.0, "OK")
    assert s.on_mode_request(2.1, AUTONOMOUS) is None
    assert s.mode == AUTONOMOUS


def test_a_stateless_localisation_status_does_not_re_permit_autonomy():
    # `{"state": null}` and a status with no state key at all both arrive
    # here as None. None means "nothing has ever arrived", which is the one
    # value the autonomous guard lets through - so the wire must not be
    # able to produce it.
    s = SupervisorState(mode=MANUAL)
    s.on_localization_status(1.0, "OFF")
    s.on_localization_status(1.5, None)
    assert s.on_mode_request(1.6, AUTONOMOUS) is not None


def test_link_loss_does_not_stop_autonomy():
    # Rule 4: the ground station going quiet is what link loss looks like
    # from here, and in autonomous mode it streams nothing anyway - only
    # a deflected stick, which a dead link cannot deliver either.
    s = SupervisorState(mode=AUTONOMOUS)
    t = 1.0
    while t < 11.0:
        s.on_autonomy_twist(t, 0.3, 0.0, 0.0)
        assert s.output(t) == (0.3, 0.0, 0.0)
        t += 0.2
    assert s.mode == AUTONOMOUS


def test_link_loss_stops_manual_via_the_deadman_and_never_clears_the_estop():
    s = SupervisorState(mode=MANUAL)
    s.on_manual_twist(1.0, 0.05, 0.0, 0.0)
    s.on_estop_request(1.1, "STOP")
    s.take_actions()
    # ... and then the link dies: nothing arrives at all for a minute
    assert s.output(61.0) == (0.0, 0.0, 0.0)
    assert s.mode == ESTOP
    assert s.estop_latched is True
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_supervisor_state.py -q'`
      Expected: 9 failures — the takeover assertions fail with `assert 'autonomous' == 'manual'`, and the localisation ones with `AttributeError: 'SupervisorState' object has no attribute 'on_localization_status'`.

- [ ] In `rover/src/navi_supervisor/navi_supervisor/supervisor_state.py`, replace `on_manual_twist` with:

```python
    def on_manual_twist(self, now, vx, vy, wz):
        self._manual = (float(vx), float(vy), float(wz))
        self._manual_at = now
        if self._mode == AUTONOMOUS and self._is_takeover(self._manual):
            self._take_over()
```

- [ ] In the same file, add these three methods immediately after `on_autonomy_twist` (before `on_estop_request`):

```python
    @staticmethod
    def _is_takeover(twist):
        vx, vy, wz = twist
        return (abs(vx) > TAKEOVER_LINEAR_MPS or abs(vy) > TAKEOVER_LINEAR_MPS
                or abs(wz) > TAKEOVER_ANGULAR_RPS)

    def _take_over(self):
        """Rule 1: takeover wins instantly.

        The order matters. Nav2 is stopped first, because a still-running
        Nav2 that regains the output the moment the operator lets go is the
        dangerous case - muting it is not enough. Only then is the
        coordinator's mission state told the truth: abort out of the
        autonomous task, then startManual, which is what ERC judging reads.

        No chassis stop is queued: the twist that caused this takeover is
        the command now, and stopping the wheels for one tick before
        obeying the stick would be a stutter the operator feels as a fault.
        """
        self._mode = MANUAL
        self._reason = "operator takeover"
        self._queue(CANCEL_GOAL, DEACTIVATE_NAV2, COORDINATOR_ABORT,
                    COORDINATOR_MANUAL)

    def on_localization_status(self, now, state):
        """Rule 3: localisation loss halts autonomy, and does not resume.

        The halt drops the mode to manual rather than leaving a muted
        autonomous, for two reasons: the operator re-issues Go by hand
        anyway, and rule 5 keeps the ground station from streaming while
        the mode is autonomous - a deflected stick still gets through, but
        staying there would leave the operator with no continuous control
        to drive the rover out of wherever it stopped.

        Manual driving is deliberately untouched by this. Driving home by
        hand is how a rover whose VIO died gets recovered; taking the
        sticks away at that moment would be the opposite of safe. The state
        still reaches /mode_status in every mode.
        """
        if not isinstance(state, str):
            # A status with no usable state is not evidence that
            # localisation recovered. `None` here would mean "nothing has
            # ever arrived", which is the one value the autonomous guard
            # lets through - so it must not be reachable from the wire.
            return
        self._localization = state
        if state in ("SEARCHING", "OFF") and self._mode == AUTONOMOUS:
            self._mode = MANUAL
            self._reason = f"localisation {state}"
            self._manual = ZERO
            self._manual_at = None
            self._stopped = True
            self._queue(CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP)
```

- [ ] In the same file, in `on_mode_request`, insert this check directly after the `if self._estop_latched and mode != MANUAL:` block and before `was_autonomous = ...`:

```python
        if mode == AUTONOMOUS and self._localization not in (None, "OK"):
            # A run must not start on a pose that is already wrong. `None`
            # - no status has ever arrived - is allowed through on purpose,
            # so a bench with no ZED can still exercise the mode; rule 3
            # then halts the run the moment a real SEARCHING or OFF lands.
            self._reason = f"localisation {self._localization}, autonomous refused"
            return self._reason
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_supervisor_state.py -q'`
      Expected: 35 passed.

- [ ] Commit: `git add rover/src/navi_supervisor && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
navi_supervisor: takeover, localisation loss, link loss

Rule 1: a stick above the ground station's own smallest output cancels
the goal, deactivates Nav2, aborts the coordinator and re-enters Manual,
and drives on the same tick. Rule 3: SEARCHING or OFF halts autonomy,
drops to manual, and never resumes by itself - manual driving keeps
working, because that is how a rover with dead VIO gets recovered.
Rule 4 is covered by tests: autonomy does not care that the link died.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 4: The `mode_supervisor` node and the Nav2 hook

**Files:**
- Create: `rover/src/navi_supervisor/navi_supervisor/nav2_control.py`, `rover/src/navi_supervisor/navi_supervisor/mode_supervisor.py`
- Test: `rover/src/navi_supervisor/test/test_mode_supervisor.py`

**Interfaces:**
- Consumes: `/manual_twist` (`geometry_msgs/Twist`), `/autonomy_twist` (`geometry_msgs/Twist`), `/mode_request` (`std_msgs/String`, JSON `{"mode": "manual"}`), `/estop_request` (`std_msgs/String`, JSON `{"reason": "..."}`), `/localization/status` (`std_msgs/String`, JSON with a `state` field).
- Produces: `/rover_twist` (`geometry_msgs/Twist`, 20 Hz), `/mode_status` (`std_msgs/String`, JSON, 2 Hz and on every change), `/drive_command` (`std_msgs/String`, JSON `{"action": "stop" | "abort" | "manual"}`); `navi_supervisor.nav2_control.Nav2Control` / `NullNav2Control`.

### Steps

- [ ] Write the failing test at `rover/src/navi_supervisor/test/test_mode_supervisor.py`:

```python
import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "91")   # throwaway; never the rover's

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from navi_supervisor.mode_supervisor import ModeSupervisor
from navi_supervisor.nav2_control import NullNav2Control


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    clock = Clock()
    nav2 = NullNav2Control()
    supervisor = ModeSupervisor(clock=clock, nav2_control=nav2)
    twists, commands, statuses = [], [], []
    supervisor._twist_pub.publish = lambda msg: twists.append(
        (msg.linear.x, msg.linear.y, msg.angular.z))
    supervisor._command_pub.publish = lambda msg: commands.append(
        json.loads(msg.data)["action"])
    supervisor._status_pub.publish = lambda msg: statuses.append(json.loads(msg.data))
    yield supervisor, clock, nav2, twists, commands, statuses
    supervisor.destroy_node()


def _twist(x, y, wz):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, wz
    return t


def _string(payload):
    m = String()
    m.data = json.dumps(payload)
    return m


def test_a_manual_twist_is_republished_on_rover_twist(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.1))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.05, 0.0, 0.1))


def test_the_deadman_zeroes_and_sends_exactly_one_chassis_stop(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    supervisor._publish_tick()
    clock.t = 2.0
    supervisor._publish_tick()
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert commands == ["stop"]


def test_an_estop_request_stops_and_latches(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    supervisor._on_estop_request(_string({"reason": "ground station STOP"}))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert "stop" in commands
    assert statuses[-1]["mode"] == "estop"
    assert statuses[-1]["reason"] == "ground station STOP"


def test_an_unparseable_estop_request_still_stops(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    bad = String()
    bad.data = "{not json"
    supervisor._on_estop_request(bad)
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert statuses[-1]["mode"] == "estop"


def test_a_mode_request_back_to_manual_clears_the_latch(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_estop_request(_string({"reason": "STOP"}))
    supervisor._on_mode_request(_string({"mode": "manual"}))
    assert statuses[-1]["mode"] == "manual"
    assert statuses[-1]["estop_latched"] is False


def test_an_unreadable_mode_request_changes_nothing(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    bad = String()
    bad.data = "{not json"
    supervisor._on_mode_request(bad)
    supervisor._status_tick()
    assert statuses[-1]["mode"] == "manual"


def test_a_takeover_cancels_nav2_and_drives_the_coordinator(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_mode_request(_string({"mode": "autonomous"}))
    supervisor._on_autonomy_twist(_twist(0.3, 0.0, 0.0))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.3, 0.0, 0.0))

    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    assert nav2.calls == [("cancel_goal", "operator takeover"),
                          ("deactivate", "operator takeover")]
    assert commands == ["abort", "manual"]
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.05, 0.0, 0.0))
    # published by the callback itself, not by a status tick: the ground
    # station's gate is driven by this topic.
    assert statuses[-1]["mode"] == "manual"


def test_localisation_loss_halts_autonomy_with_the_reason_in_the_status(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_mode_request(_string({"mode": "autonomous"}))
    supervisor._on_autonomy_twist(_twist(0.3, 0.0, 0.0))
    supervisor._publish_tick()
    supervisor._on_localization_status(_string({"state": "SEARCHING"}))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert nav2.calls[0] == ("cancel_goal", "localisation SEARCHING")
    assert "stop" in commands
    # no _status_tick() here: the callback publishes the change itself
    assert statuses[-1]["mode"] == "manual"
    assert statuses[-1]["reason"] == "localisation SEARCHING"
    assert statuses[-1]["localization_state"] == "SEARCHING"


def test_an_unreadable_localisation_status_does_not_kill_the_node(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    bad = String()
    bad.data = "{not json"
    supervisor._on_localization_status(bad)
    supervisor._publish_tick()      # no exception = pass


def test_the_status_tick_publishes_json_with_every_field(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._status_tick()
    status = statuses[-1]
    for key in ("mode", "reason", "source", "deadman_active", "estop_latched",
                "localization_state", "source_age_s"):
        assert key in status


def test_the_node_publishes_rover_twist_and_never_manual_twist(node):
    # Read off the publisher itself rather than the ROS graph: the graph
    # cache is populated by discovery and a test that asks it immediately
    # is a flake waiting to happen.
    supervisor, clock, nav2, twists, commands, statuses = node
    assert supervisor._twist_pub.topic_name == "/rover_twist"
    assert supervisor._status_pub.topic_name == "/mode_status"
    assert supervisor._command_pub.topic_name == "/drive_command"
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_mode_supervisor.py -q'`
      Expected: collection error, `ModuleNotFoundError: No module named 'navi_supervisor.mode_supervisor'`.

- [ ] Create `rover/src/navi_supervisor/navi_supervisor/nav2_control.py`:

```python
"""What the supervisor is allowed to do to Nav2, and the stub that stands
in until Nav2 exists.

SP9 brings Nav2 up and replaces NullNav2Control with an implementation
that cancels the running NavigateToPose goal and deactivates the
navigation lifecycle. Until then the supervisor still decides when both
must happen, and still records that it asked: the decision is the
safety-critical half, and it is finished and tested now rather than
arriving late with Nav2.

The interface is deliberately two methods that return nothing. The
supervisor must never wait on Nav2 in order to stop the rover - it has
already published a zero twist by the time either of these is called.
"""


class Nav2Control:
    """The interface SP9 implements. Do not change the two names."""

    def cancel_goal(self, reason: str) -> None:
        raise NotImplementedError

    def deactivate(self, reason: str) -> None:
        raise NotImplementedError


class NullNav2Control(Nav2Control):
    """No Nav2 on the graph yet: log the request and record it.

    `calls` is the list of (method, reason) pairs, in order. It is what the
    tests assert on, and it is the contract SP9's real implementation is
    checked against: the sequence the supervisor asks for must not change
    when the stub stops being a stub.
    """

    def __init__(self, logger=None):
        self._logger = logger
        self.calls = []

    def cancel_goal(self, reason: str) -> None:
        self.calls.append(("cancel_goal", reason))
        if self._logger is not None:
            self._logger.info(f"nav2 goal cancel requested ({reason}); no Nav2 yet")

    def deactivate(self, reason: str) -> None:
        self.calls.append(("deactivate", reason))
        if self._logger is not None:
            self._logger.info(f"nav2 deactivate requested ({reason}); no Nav2 yet")
```

- [ ] Create `rover/src/navi_supervisor/navi_supervisor/mode_supervisor.py`:

```python
"""The sole publisher of /rover_twist: mode, arbitration and the deadman.

Sources are /manual_twist (in manual and semi_auto) and /autonomy_twist
(in autonomous); /mode_request and /estop_request steer it, and
/mode_status reports it - all JSON in a std_msgs/String, the convention
/drive_status and /localization/status already set, so the ground station
reads them over rosbridge with no custom message type and no ROS.

The rules live in supervisor_state.py, which has no ROS in it. This file
owns the timers, the topics, and the two side-effect channels:
/drive_command, which bema_bridge already owns the RPC session for, and a
Nav2Control that is a stub until SP9. The supervisor deliberately does not
open its own connection to the primary - a second msgpack client would
fight bema_bridge for the same exclusive lease.

Nothing else may publish /rover_twist. bema_bridge subscribes to it and
keeps its own 1 s deadman on top of this one's: two deadmen in series is
deliberate, this one against Nav2 hanging, that one against the whole
Orin-side graph dying.
"""

import json
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from navi_supervisor import supervisor_state as rules
from navi_supervisor.nav2_control import NullNav2Control
from navi_supervisor.supervisor_state import SupervisorState

PUBLISH_HZ = 20.0
STATUS_HZ = 2.0


class ModeSupervisor(Node):

    def __init__(self, clock=monotonic, nav2_control=None,
                 parameter_overrides=None):
        super().__init__("mode_supervisor",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("manual_deadman_s", rules.MANUAL_DEADMAN_S)
        self.declare_parameter("autonomy_deadman_s", rules.AUTONOMY_DEADMAN_S)
        self.declare_parameter("start_mode", rules.MANUAL)

        # NOT self._clock: rclpy.node.Node already owns that name, and
        # create_timer() defaults clock=self._clock - overwriting it makes
        # every timer raise AttributeError on a plain callable.
        self._now = clock
        self._nav2 = (nav2_control if nav2_control is not None
                      else NullNav2Control(self.get_logger()))
        self._state = SupervisorState(
            mode=str(self.get_parameter("start_mode").value),
            manual_deadman_s=float(self.get_parameter("manual_deadman_s").value),
            autonomy_deadman_s=float(self.get_parameter("autonomy_deadman_s").value))

        self._twist_pub = self.create_publisher(Twist, "/rover_twist", 1)
        self._status_pub = self.create_publisher(String, "/mode_status", 1)
        self._command_pub = self.create_publisher(String, "/drive_command", 10)

        self.create_subscription(Twist, "/manual_twist", self._on_manual_twist, 1)
        self.create_subscription(Twist, "/autonomy_twist", self._on_autonomy_twist, 1)
        self.create_subscription(String, "/mode_request", self._on_mode_request, 10)
        self.create_subscription(String, "/estop_request", self._on_estop_request, 10)
        self.create_subscription(String, "/localization/status",
                                 self._on_localization_status, 10)

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)

    # --- inputs ----------------------------------------------------------
    def _on_manual_twist(self, msg: Twist):
        try:
            before = self._state.mode
            self._state.on_manual_twist(self._now(), msg.linear.x,
                                        msg.linear.y, msg.angular.z)
            self._run_actions()
            if self._state.mode != before:
                # A takeover changes the mode, and /mode_status is the
                # ground station's publish gate: waiting for the 2 Hz tick
                # would leave the operator's sticks ignored for up to
                # 500 ms. Contract is "2 Hz and on every change" - hence
                # the guard, so a 20 Hz manual stream does not turn this
                # into a 20 Hz status stream over the field link.
                self._publish_status()
        except Exception as exc:                     # never kill the node
            self.get_logger().error(f"manual twist callback failed: {exc!r}")

    def _on_autonomy_twist(self, msg: Twist):
        try:
            self._state.on_autonomy_twist(self._now(), msg.linear.x,
                                          msg.linear.y, msg.angular.z)
        except Exception as exc:
            self.get_logger().error(f"autonomy twist callback failed: {exc!r}")

    def _on_mode_request(self, msg: String):
        try:
            mode = json.loads(msg.data).get("mode")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable mode request: {msg.data!r}")
            return
        try:
            refusal = self._state.on_mode_request(self._now(), mode)
            if refusal is not None:
                self.get_logger().warn(f"mode request refused: {refusal}")
            self._run_actions()
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"mode request failed: {exc!r}")

    def _on_estop_request(self, msg: String):
        # Deliberately not gated on the parse: an e-stop whose payload will
        # not read is still an e-stop. The JSON is consulted only to
        # recover a reason for /mode_status.
        reason = "e-stop"
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict) and payload.get("reason"):
                reason = str(payload["reason"])
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(
                f"unreadable e-stop payload, stopping anyway: {msg.data!r}")
        try:
            self._state.on_estop_request(self._now(), reason)
            self._run_actions()
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"e-stop failed: {exc!r}")

    def _on_localization_status(self, msg: String):
        # A status that will not parse is left alone rather than treated as
        # a loss: rule 3 names two states, and a garbled message says
        # nothing about which one the localisation is in. The autonomy
        # deadman still covers a Nav2 that reacts to the same fault.
        try:
            state = json.loads(msg.data).get("state")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable localisation status: {msg.data!r}")
            return
        try:
            self._state.on_localization_status(self._now(), state)
            self._run_actions()
            # Same contract as above, and the same urgency: a localisation
            # halt drops autonomous to manual precisely so the operator can
            # drive the rover out, which needs the gate open now, not at
            # the next tick. Rule 3 also names the reason in /mode_status.
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"localisation status failed: {exc!r}")

    # --- outputs ---------------------------------------------------------
    def _run_actions(self):
        actions = self._state.take_actions()
        if not actions:
            return
        reason = self._state.status(self._now())["reason"]
        for action in actions:
            try:
                if action == rules.CANCEL_GOAL:
                    self._nav2.cancel_goal(reason)
                elif action == rules.DEACTIVATE_NAV2:
                    self._nav2.deactivate(reason)
                elif action == rules.COORDINATOR_ABORT:
                    self._send_command("abort")
                elif action == rules.COORDINATOR_MANUAL:
                    self._send_command("manual")
                elif action == rules.CHASSIS_STOP:
                    self._send_command("stop")
                else:
                    self.get_logger().warn(f"unknown supervisor action: {action!r}")
            except Exception as exc:
                self.get_logger().error(f"supervisor action {action} failed: {exc!r}")

    def _send_command(self, action: str):
        msg = String()
        msg.data = json.dumps({"action": action})
        self._command_pub.publish(msg)

    def _publish_tick(self):
        try:
            now = self._now()
            vx, vy, wz = self._state.output(now)
            # output() is what notices the live -> stopped edge, so the
            # chassis stop it queues is drained here, before the zero goes
            # out rather than a tick later.
            self._run_actions()
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(wz)
            self._twist_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"publish tick failed: {exc!r}")

    def _status_tick(self):
        try:
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"status tick failed: {exc!r}")

    def _publish_status(self):
        msg = String()
        # default=str for the same reason /drive_status uses it: one odd
        # field must not black out the whole status the operator reads.
        msg.data = json.dumps(self._state.status(self._now()), default=str)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = ModeSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/ -q'`
      Expected: 46 passed (35 state + 11 node).

- [ ] Run the build so the entry point and the package marker are real: `bash -c 'cd /home/ole/star/Navi/rover && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select navi_supervisor'`
      Expected: `Summary: 1 package finished`.

- [ ] Commit: `git add rover/src/navi_supervisor && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
navi_supervisor: the mode_supervisor node and the SP9 Nav2 hook

The node owns the timers and the topics: /rover_twist at 20 Hz,
/mode_status at 2 Hz and on every change, and side effects through
/drive_command - which bema_bridge already holds the primary's lease for,
so the supervisor never opens a second msgpack client to fight it.
Nav2Control is two methods and a recording stub; SP9 replaces the stub.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 5: `abort` reaches the coordinator through the bridge

**Files:**
- Modify: `rover/src/navi_teleop/navi_teleop/bema_session.py`, `rover/src/navi_teleop/navi_teleop/bema_bridge.py`, `rover/src/navi_teleop/test/fake_bema_server.py`
- Test: `rover/src/navi_teleop/test/test_bema_bridge.py`, `rover/src/navi_teleop/test/test_bema_session.py`

**Interfaces:**
- Consumes: `/drive_command` JSON `{"action": "abort"}` (published by `mode_supervisor`).
- Produces: coordinator RPC `F7` (`abort`) under the coordinator's own `__sam__` lease.

### Steps

- [ ] Append the failing tests to `rover/src/navi_teleop/test/test_bema_session.py`:

```python
def test_abort_takes_the_coordinator_lease_then_calls_f7(server):
    # Coordinator F7 is abort, and like F6 it sits behind checkAccess() on
    # the coordinator's own lease - the same trap start_manual fell into.
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    server.set_state(5)                          # Autonomous, so 1 proves F7
    sess.abort()
    assert ("coord", "__sam__request", []) in server.calls
    assert ("coord", "F7", []) in server.calls
    assert server.state == 1                     # back to Idle


def test_abort_without_a_coordinator_is_a_no_op(server):
    clock = Clock()
    sess = _session(server, clock)
    sess._coord = None
    sess.abort()          # no exception = pass
```

  (`server`, `Clock` and `_session` are the fixture and helpers
  `test_bema_session.py` already defines — do not add new ones.)

- [ ] Append the failing test to `rover/src/navi_teleop/test/test_bema_bridge.py`:

```python
def test_abort_command_calls_the_coordinator(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "abort"}))
    assert ("coord", "F7", []) in server.calls
    assert node._last_action == "abort"
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/test_bema_session.py rover/src/navi_teleop/test/test_bema_bridge.py -q'`
      Expected: 3 failures, `AttributeError: 'BemaSession' object has no attribute 'abort'` and, for the bridge, `unknown drive action: 'abort'` leaving `_last_action` as `None`.

- [ ] In `rover/src/navi_teleop/navi_teleop/bema_session.py`, add this method immediately after `start_manual`:

```python
    def abort(self):
        # F7 on the COORDINATOR is abort - not BEMA's F7, which is
        # setMovementEnabled(bool) on a different server on a different
        # port. Like startManual it sits behind CoordinatorProxy's
        # checkAccess() on the coordinator's own ServerAccessManager, so
        # the lease is acquired just-in-time here too; it lapses by itself
        # after 4 s of guarded-call silence.
        if self._coord is None:
            return
        try:
            if not self._coord.call("__sam__request"):
                self._last_error = "coordinator refused the lease for abort"
                return
            self._coord.call("F7")
        except RpcError as exc:
            self._mark_refused(exc, "abort")
        except (RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)
```

- [ ] In `rover/src/navi_teleop/navi_teleop/bema_bridge.py`, add the `abort` entry to the dispatch table in `_on_command` (after `"manual"`):

```python
            "abort": self._session.abort,
```

- [ ] In `rover/src/navi_teleop/test/fake_bema_server.py`, in `_coord_dispatch`, insert this immediately before the `if method == "F6":` branch:

```python
        if method == "F7":             # abort - back to Idle
            self.state = 1
            return None, None
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/ -q'`
      Expected: all pass, 3 more than before.

- [ ] Commit: `git add rover/src/navi_teleop && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
bema_bridge: an abort action, so the supervisor can drive the coordinator

The supervisor's takeover sequence is abort then startManual on the
coordinator. Both go through the bridge, which already holds both leases -
a second msgpack client would fight it for the exclusive one. Coordinator
F7 is abort; BEMA's F7 is setMovementEnabled, a different server.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 6: `bema_bridge` is fed from `/rover_twist`, and the launcher starts the supervisor

**Files:**
- Modify: `rover/src/navi_teleop/navi_teleop/bema_bridge.py`, `rover/src/navi_teleop/package.xml`, `rover/start_navi.sh`
- Test: `rover/src/navi_teleop/test/test_bema_bridge.py`, `rover/test/test_start_navi_gate.sh`

**Interfaces:**
- Consumes: `/rover_twist` (`geometry_msgs/Twist`) — the bridge's new default source.
- Produces: nothing new. `ros2 run navi_supervisor mode_supervisor` is added to the rover bring-up.

### Steps

- [ ] Append the failing test to `rover/src/navi_teleop/test/test_bema_bridge.py`:

```python
def test_the_bridge_defaults_to_rover_twist_not_manual_twist(bridge):
    # The default is what matters: a bridge started by hand must not be
    # drivable around the supervisor's arbitration and e-stop.
    node, server, clock = bridge
    assert node.get_parameter("twist_topic").value == "/rover_twist"
    subscribed = {s.topic_name for s in node.subscriptions}
    assert "/rover_twist" in subscribed
    assert "/manual_twist" not in subscribed
```

  (The existing `bridge` fixture's `parameter_overrides` sets `deadman_s`
  only. Do **not** add a `twist_topic` override to it "for clarity" - the
  un-overridden parameter is exactly what this test proves.)

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/test_bema_bridge.py -q'`
      Expected: 1 failure, `assert '/manual_twist' == '/rover_twist'`.

- [ ] In `rover/src/navi_teleop/navi_teleop/bema_bridge.py`, change the parameter default:

```python
        self.declare_parameter("twist_topic", "/rover_twist")
```

- [ ] In `rover/src/navi_teleop/navi_teleop/bema_bridge.py`, replace the module docstring's first paragraph with:

```python
"""The rover-side node that turns /rover_twist into real wheel commands.

It owns the timers; bema_session owns the protocol. A twist is forwarded
to the primary's IK at 20 Hz; if the stream stops for deadman_s the wheels
are zeroed and stopped, and kept stopped until a fresh twist arrives.
/drive_command (JSON) drives the coordinator/BEMA buttons the ground
station shows; /drive_status (JSON, 1 Hz) reports what is happening.

The source is /rover_twist, which mode_supervisor is the only publisher
of - never /manual_twist directly, or the operator's stream would reach
the wheels around the arbitration and the e-stop. `twist_topic` can point
this elsewhere for a bench test; the default is the safe wiring, and
start_navi.sh passes it explicitly anyway so the wiring can be read at the
launch site.

This node's own 1 s deadman is kept even though the supervisor has one:
two in series is deliberate - the supervisor's protects against Nav2
hanging, this one against the whole Orin-side graph dying.

Nothing here calls init() or startManual() on its own - the rover only
moves after the operator presses a button.
"""
```

- [ ] In `rover/src/navi_teleop/package.xml`, replace the `<description>` element with:

```xml
  <description>Teleoperation bridge nodes for the Asterope rover: the drive
  bridge consumes /rover_twist from navi_supervisor, and the video and
  listener nodes serve the ground station.</description>
```

- [ ] In `rover/start_navi.sh`, replace the numbered list in the header comment (lines 2-11 of the file) with:

```bash
# Bring up the rover side of the manual-drive link:
#   1. rosbridge_server  - the websocket the ground station connects to
#   2. manual_twist_listener - logs the /manual_twist stream the ground
#      station publishes, so what the rover receives is visible here
#   3. video_sender - waits on /video_request and streams the ZED 2i to
#      whichever address the ground station asks for
#   4. mode_supervisor - the only publisher of /rover_twist: mode
#      arbitration, the deadman, and the latched e-stop
#   5. bema_bridge - the BEMA drive bridge, fed from /rover_twist, idle
#      until the ground station drives
#   6. localization.launch.py - the ZED 2i wrapper with positional tracking,
#      plus localization_status publishing /localization/pose and
#      /localization/status
```

- [ ] In `rover/start_navi.sh`, add the new flag to the usage comment, directly after the `--no-drive-bridge` line:

```bash
#   ./start_navi.sh --no-supervisor  no mode_supervisor (nothing publishes /rover_twist, so the rover cannot be driven)
```

- [ ] In `rover/start_navi.sh`, add the flag's default beside the others (after `START_DRIVE_BRIDGE=1`):

```bash
START_SUPERVISOR=1
```

- [ ] In `rover/start_navi.sh`, add the flag to the argument loop, directly after the `--no-drive-bridge` case:

```bash
        --no-supervisor) START_SUPERVISOR=0; shift ;;
```

- [ ] In `rover/start_navi.sh`, extend the stale-node cleanup. Replace the two `kill_stale` lines for the teleop nodes with:

```bash
    kill_stale "navi_teleop nodes" "navi_teleop/(manual_twist_listener|video_sender|bema_bridge)"
    kill_stale "navi_supervisor nodes" "navi_supervisor/mode_supervisor"
    kill_stale "ros2 run wrappers" "ros2 run navi_(teleop|supervisor)"
```

- [ ] In `rover/start_navi.sh`, insert the supervisor launch immediately before the `if [ "$START_DRIVE_BRIDGE" -eq 1 ]; then` block, and re-point the bridge in the same edit — replace that whole block with:

```bash
if [ "$START_SUPERVISOR" -eq 1 ]; then
    # Before the bridge: the bridge's source must have a publisher by the
    # time it subscribes, and this is the node that owns the e-stop.
    echo "starting mode_supervisor (sole publisher of /rover_twist)"
    ros2 run navi_supervisor mode_supervisor &
    BACKGROUND_PIDS+=("$!")
fi

if [ "$START_DRIVE_BRIDGE" -eq 1 ]; then
    # twist_topic is already the default; it is passed here as well so the
    # wiring can be read at the launch site rather than only in the node.
    echo "starting bema_bridge on /rover_twist (idle until the ground station drives)"
    ros2 run navi_teleop bema_bridge --ros-args -p twist_topic:=/rover_twist &
    BACKGROUND_PIDS+=("$!")
fi
```

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/ -q'`
      Expected: all pass, 1 more than after Task 5.

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && bash rover/test/test_start_navi_gate.sh'`
      Expected: `all start_navi.sh readiness-gate checks passed`, exit 0 — the new flag and variable must not disturb the `NAVI_FUNCTIONS_ONLY` source path.

- [ ] Run: `bash -c 'cd /home/ole/star/Navi && bash -n rover/start_navi.sh && echo "syntax ok"'`
      Expected: `syntax ok`.

- [ ] Commit: `git add rover && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
The chassis is fed from /rover_twist, and start_navi.sh starts the supervisor

The bridge's twist_topic default moves to /rover_twist so a bridge started
by hand cannot be driven around the arbitration, and start_navi.sh passes
it explicitly so the wiring reads at the launch site. mode_supervisor
starts before the bridge, has its own --no-supervisor flag, and joins the
stale-node cleanup.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 7: The ground station learns the mode (pure model + rosbridge client)

**Files:**
- Modify: `ground_station/models.py`, `ground_station/ros_client.py`
- Test: `tests/test_models.py`, `tests/test_ros_client.py`

**Interfaces:**
- Consumes: `/mode_status` (`std_msgs/String`, JSON) over rosbridge.
- Produces: `/mode_request` and `/estop_request` (`std_msgs/String`, JSON) over rosbridge; `ground_station.models.ModeState`, `parse_mode_status`, `may_publish_manual_twist`, `may_publish_takeover_twist`, `is_stick_deflected`, `mode_request_json`, `estop_request_json`; `RosSignals.mode_status_received`; `RosBridgeClient.subscribe_mode_status`, `send_mode_request`, `send_estop_request`.

### Steps

- [ ] Append the failing tests to `tests/test_models.py`. `tests/test_models.py` currently starts with `import pytest` and `from ground_station.models import DriveCommandTracker, NodeRegistry`; replace those two lines with:

```python
import json

import pytest
from ground_station.models import (DriveCommandTracker, ModeState, NodeRegistry,
                                   estop_request_json, is_stick_deflected,
                                   may_publish_manual_twist,
                                   may_publish_takeover_twist,
                                   mode_request_json, parse_mode_status)
```

  then append:

```python
def test_parse_mode_status_reads_every_field():
    state = parse_mode_status(json.dumps({
        "mode": "autonomous", "reason": "mode request",
        "source": "/autonomy_twist", "deadman_active": False,
        "estop_latched": False, "localization_state": "OK",
        "source_age_s": 0.12}))
    assert state == ModeState(mode="autonomous", reason="mode request",
                              source="/autonomy_twist", deadman_active=False,
                              estop_latched=False, localization_state="OK",
                              source_age_s=0.12)


def test_parse_mode_status_defaults_every_missing_field():
    state = parse_mode_status("{}")
    assert state.mode == ""
    assert state.reason == ""
    assert state.source is None
    assert state.deadman_active is False
    assert state.estop_latched is False
    assert state.localization_state is None
    assert state.source_age_s is None


def test_parse_mode_status_returns_none_for_rubbish():
    assert parse_mode_status("{not json") is None
    assert parse_mode_status("[1, 2]") is None


def test_parse_mode_status_ignores_fields_of_the_wrong_type():
    state = parse_mode_status(json.dumps({
        "mode": "manual", "source": 7, "source_age_s": "soon",
        "localization_state": ["OK"]}))
    assert state.source is None
    assert state.source_age_s is None
    assert state.localization_state is None


def test_manual_twist_is_published_in_manual_and_semi_auto_only():
    def at(mode):
        return parse_mode_status(json.dumps({"mode": mode}))
    assert may_publish_manual_twist(at("manual")) is True
    assert may_publish_manual_twist(at("semi_auto")) is True
    assert may_publish_manual_twist(at("autonomous")) is False
    assert may_publish_manual_twist(at("estop")) is False


def test_an_unknown_mode_is_treated_as_not_driving():
    assert may_publish_manual_twist(parse_mode_status('{"mode": "turbo"}')) is False


def test_no_mode_status_at_all_still_publishes():
    # No supervisor on that rover: publishing is what it needs, and a rover
    # that has one answers within half a second of the subscription.
    assert may_publish_manual_twist(None) is True


def test_a_deflected_stick_may_still_be_published_in_autonomous_only():
    def at(mode):
        return parse_mode_status(json.dumps({"mode": mode}))
    assert may_publish_takeover_twist(at("autonomous")) is True
    assert may_publish_takeover_twist(at("estop")) is False
    assert may_publish_takeover_twist(at("manual")) is False
    assert may_publish_takeover_twist(parse_mode_status('{"mode": "turbo"}')) is False
    assert may_publish_takeover_twist(None) is False


def test_a_centred_stick_is_not_deflected_but_the_smallest_output_is():
    # gamepad_input.DEADZONE (0.1) has already been applied per axis before
    # scaling, so a centred stick is exactly zero and the smallest thing
    # the reader can emit is DEADZONE * MAX_LINEAR_SPEED = 0.005.
    assert is_stick_deflected((0.0, 0.0, 0.0)) is False
    assert is_stick_deflected((0.005, 0.0, 0.0)) is True
    assert is_stick_deflected((0.0, 0.0, 0.01)) is True


def test_mode_and_estop_request_json():
    assert json.loads(mode_request_json("manual")) == {"mode": "manual"}
    assert json.loads(estop_request_json("ground station STOP")) == {
        "reason": "ground station STOP"}
```

- [ ] Append the failing tests to `tests/test_ros_client.py`:

```python
def test_subscribe_mode_status_emits_a_parsed_state():
    FakeTopic.instances.clear()
    client = RosBridgeClient("localhost", ros_factory=FakeRos, topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    received = []
    client.signals.mode_status_received.connect(received.append)
    client.subscribe_mode_status()
    topic = next(t for t in FakeTopic.instances if t.name == "/mode_status")
    assert topic.msg_type == "std_msgs/String"
    topic.callback({"data": json.dumps({"mode": "autonomous", "reason": "go"})})
    assert received[-1].mode == "autonomous"


def test_send_mode_request_publishes_json_when_connected():
    FakeTopic.instances.clear()
    client = RosBridgeClient("localhost", ros_factory=FakeRos, topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    client.connect()
    client.send_mode_request("manual")
    topic = next(t for t in FakeTopic.instances if t.name == "/mode_request")
    assert json.loads(topic.published_messages[-1]["data"]) == {"mode": "manual"}


def test_send_estop_request_publishes_json_when_connected():
    FakeTopic.instances.clear()
    client = RosBridgeClient("localhost", ros_factory=FakeRos, topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    client.connect()
    client.send_estop_request("ground station STOP")
    topic = next(t for t in FakeTopic.instances if t.name == "/estop_request")
    assert json.loads(topic.published_messages[-1]["data"]) == {
        "reason": "ground station STOP"}


def test_requests_are_dropped_when_not_connected():
    FakeTopic.instances.clear()
    client = RosBridgeClient("localhost", ros_factory=FakeRos, topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    client.send_mode_request("manual")
    client.send_estop_request("STOP")
    assert [t for t in FakeTopic.instances
            if t.name in ("/mode_request", "/estop_request")] == []
```

  (`tests/test_ros_client.py` already has a module-level `import json`, and
  `FakeTopic`, `FakeRos` and `fake_message_factory` are defined at the top of
  that file. Add nothing to its imports.)

- [ ] Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_models.py tests/test_ros_client.py -q`
      Expected: collection error, `ImportError: cannot import name 'parse_mode_status' from 'ground_station.models'`.

- [ ] In `ground_station/models.py`, append at the end of the file:

```python
# The modes in which the ground station streams /manual_twist continuously.
# In autonomous it streams nothing - only a deflected stick is published,
# see may_publish_takeover_twist - so a stick that does move is a real
# takeover signal rather than one message in a 20 Hz stream of zeros; in
# estop it publishes nothing at all, because there is nothing to say.
DRIVING_MODES = ("manual", "semi_auto")
AUTONOMOUS_MODE = "autonomous"


@dataclass
class ModeState:
    """/mode_status as the DRIVE row's mode chip shows it."""
    mode: str
    reason: str
    source: str | None
    deadman_active: bool
    estop_latched: bool
    localization_state: str | None
    source_age_s: float | None


def parse_mode_status(payload: str):
    """A best-effort ModeState from /mode_status's JSON.

    The same defensive stance as parse_drive_status: a payload that will
    not parse, or a field of the wrong type, falls back to that field's
    default rather than raising inside a Qt slot while someone is driving.
    """
    try:
        status = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(status, dict):
        return None
    age = status.get("source_age_s")
    source = status.get("source")
    localization = status.get("localization_state")
    return ModeState(
        mode=str(status.get("mode", "")),
        reason=str(status.get("reason") or ""),
        source=source if isinstance(source, str) else None,
        deadman_active=status.get("deadman_active") is True,
        estop_latched=status.get("estop_latched") is True,
        localization_state=localization if isinstance(localization, str) else None,
        source_age_s=(age if isinstance(age, (int, float))
                      and not isinstance(age, bool) else None))


def may_publish_manual_twist(state) -> bool:
    """Spec rule 5: stream /manual_twist only in manual and semi_auto.

    This is the *stream* gate. In autonomous it is False, and the takeover
    path (may_publish_takeover_twist + is_stick_deflected) is what still
    lets a deflected stick through - rule 5's own justification is that
    rule 1 becomes "a real signal rather than a constant stream", not that
    rule 1 becomes unreachable.

    No mode status at all means no supervisor is running on that rover -
    an older build, or one brought up with --no-supervisor. Then this
    returns True, because publishing is the behaviour that rover needs and
    the stream reaches nothing that can act on it anyway; a rover that does
    have a supervisor answers within half a second of the subscription
    being made, so the permissive window is short.

    An unknown mode name is the other way round: a supervisor exists and
    is in a state this ground station does not know about, and the
    supervisor is the authority on what its own modes mean. Guessing in
    the permissive direction there is how a takeover gets faked.
    """
    if state is None:
        return True
    return state.mode in DRIVING_MODES


def may_publish_takeover_twist(state) -> bool:
    """Whether a *deflected* stick may still be published in this mode.

    Only autonomous: that is where rule 1 lives. In estop nothing may be
    published, and in a mode this build does not know the supervisor is
    the authority on what its own modes mean - guessing permissively there
    is how a takeover gets faked. With no mode status at all the stream
    gate is already open, so this never has to decide.
    """
    return state is not None and state.mode == AUTONOMOUS_MODE


def is_stick_deflected(twist) -> bool:
    """"Above the deadzone", in the ground station's own terms.

    No new threshold: gamepad_input.GamepadReader.read_twist() has already
    run every axis through _apply_deadzone(DEADZONE = 0.1) before scaling,
    so a centred stick is exactly (0.0, 0.0, 0.0) and the smallest
    deflection it can report is DEADZONE * MAX_LINEAR_SPEED = 0.005 m/s
    (0.01 rad/s) - well above the supervisor's TAKEOVER_LINEAR_MPS of
    0.002. A non-zero component is therefore past the deadzone by
    construction, and gamepad_input.py stays untouched.
    """
    return any(float(v) != 0.0 for v in twist)


def mode_request_json(mode: str) -> str:
    return json.dumps({"mode": mode})


def estop_request_json(reason: str) -> str:
    return json.dumps({"reason": reason})
```

- [ ] In `ground_station/ros_client.py`, extend the import from `ground_station.models` to:

```python
from ground_station.models import (drive_command_json, estop_request_json,
                                    map_command_json, mode_request_json,
                                    parse_drive_status, parse_map_status,
                                    parse_mode_status, pose_readout_from_odometry)
```

- [ ] In `ground_station/ros_client.py`, add the signal to `RosSignals`, after `drive_status_received`:

```python
    mode_status_received = Signal(object)
```

- [ ] In `ground_station/ros_client.py`, add the three topic handles to `RosBridgeClient.__init__`, after `self._drive_command_topic = None`:

```python
        self._mode_status_topic = None
        self._mode_request_topic = None
        self._estop_request_topic = None
```

- [ ] In `ground_station/ros_client.py`, append these three methods at the end of the class:

```python
    def subscribe_mode_status(self, topic_name: str = "/mode_status") -> None:
        """The supervisor's account of itself: which mode owns /rover_twist,
        whether the deadman has fired, whether the e-stop is latched, and
        why. JSON in a std_msgs/String at 2 Hz and on every change, the same
        convention as /drive_status - and the thing that decides whether
        this ground station may publish /manual_twist at all."""
        topic = self._topic_factory(self._ros, topic_name, "std_msgs/String")
        topic.subscribe(lambda msg: self.signals.mode_status_received.emit(
            parse_mode_status(msg.get("data", ""))))
        self._mode_status_topic = topic

    def send_mode_request(self, mode: str,
                          topic_name: str = "/mode_request") -> None:
        """manual / semi_auto / autonomous / estop, as the supervisor reads.
        A request for manual is also the only thing that clears a latched
        e-stop, which is why the DRIVE row's Manual button sends one."""
        if not self.is_connected:
            print("ground_station: not connected, mode request dropped", file=sys.stderr)
            return
        if self._mode_request_topic is None:
            self._mode_request_topic = self._topic_factory(
                self._ros, topic_name, "std_msgs/String")
        self._mode_request_topic.publish(self._message_factory(
            {"data": mode_request_json(mode)}))

    def send_estop_request(self, reason: str = "ground station STOP",
                           topic_name: str = "/estop_request") -> None:
        """The latched, rover-local stop.

        Sent in every mode, alongside the chassis stop on /drive_command,
        because the two do different things: this one latches on the Orin
        and survives this link dying, that one stops the wheels through the
        primary right now. Neither replaces the other."""
        if not self.is_connected:
            print("ground_station: not connected, e-stop request dropped", file=sys.stderr)
            return
        if self._estop_request_topic is None:
            self._estop_request_topic = self._topic_factory(
                self._ros, topic_name, "std_msgs/String")
        self._estop_request_topic.publish(self._message_factory(
            {"data": estop_request_json(reason)}))
```

- [ ] Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_models.py tests/test_ros_client.py -q`
      Expected: all pass, 14 more than before.

- [ ] Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/ -q`
      Expected: 263 passed (249 + 14).

- [ ] Commit: `git add ground_station tests && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
ground station: /mode_status, /mode_request and /estop_request

The parse and the publish rule live in models.py, which imports neither
Qt nor ROS, so rule 5 is testable on its own. No mode status at all means
no supervisor on that rover and publishing continues; an unknown mode
means a supervisor this build does not understand, and it does not.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Task 8: The publish gate, the STOP path, and the mode chip

**Files:**
- Modify: `ground_station/ui/main_window.py`, `ground_station/ui/drive_row.py`
- Test: `tests/test_main_window.py`, `tests/test_drive_row.py`

**Interfaces:**
- Consumes: `RosSignals.mode_status_received`, `ground_station.models.may_publish_manual_twist`, `may_publish_takeover_twist`, `is_stick_deflected`, `ModeState`.
- Note: `main_window.py` already has an unrelated `_on_mode_changed(self, mode: str)` at line 515 - that is the *video/view* selector (its values include `simulation`), a different axis from the rover mode. The new names here (`_on_mode_status`, `self._mode_state`) do not collide with it; keep both exactly as written.
- Produces: `/manual_twist` gated by mode; `/estop_request` on every STOP; `/mode_request` `{"mode": "manual"}` on the DRIVE row's Manual button; `DriveRow.set_mode_state(state)` and its `mode_pill`.

### Steps

- [ ] Append the failing tests to `tests/test_drive_row.py`:

```python
def mode(**over):
    from ground_station.models import ModeState
    base = dict(mode="manual", reason="", source="/manual_twist",
                deadman_active=False, estop_latched=False,
                localization_state="OK", source_age_s=0.05)
    base.update(over)
    return ModeState(**base)


def test_the_mode_pill_is_hidden_until_a_mode_status_arrives(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    assert not row.mode_pill.isVisibleTo(row)


def test_the_mode_pill_names_the_mode(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_mode_state(mode(mode="semi_auto"))
    assert row.mode_pill.text() == "SEMI_AUTO"
    row.set_mode_state(mode(mode="autonomous"))
    assert row.mode_pill.text() == "AUTONOMOUS"


def test_a_latched_estop_is_what_the_mode_pill_says(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_mode_state(mode(mode="estop", estop_latched=True, source=None))
    assert row.mode_pill.text() == "E-STOP LATCHED"


def test_the_mode_pill_survives_a_drive_status_going_away(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_mode_state(mode(mode="autonomous"))
    row.set_state(None)                # the bridge went quiet, not the supervisor
    assert row.mode_pill.text() == "AUTONOMOUS"
    assert row.mode_pill.isVisibleTo(row)
```

- [ ] Append the failing tests to `tests/test_main_window.py`:

```python
def _mode_status(window, mode):
    from ground_station.models import parse_mode_status
    window._on_mode_status(parse_mode_status(json.dumps({"mode": mode})))


def test_manual_twist_is_published_in_manual_mode(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.04, 0.0, 0.05))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "manual")

    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert len(topic.published_messages) == 1


def test_manual_twist_is_published_in_semi_auto_mode(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.04, 0.0, 0.05))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "semi_auto")

    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert len(topic.published_messages) == 1


def test_a_centred_stick_publishes_nothing_in_autonomous_mode(qtbot):
    # The constant zero stream is what rule 5 kills.
    gamepad = FakeGamepadReader(connected=True, twist=(0.0, 0.0, 0.0))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "autonomous")

    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages == []


def test_a_deflected_stick_is_published_in_autonomous_mode(qtbot):
    # ... and this is what keeps rule 1 reachable: the supervisor reads
    # 0.04 m/s as a takeover, aborts the coordinator and re-enters manual.
    gamepad = FakeGamepadReader(connected=True, twist=(0.04, 0.0, 0.05))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "autonomous")

    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert len(topic.published_messages) == 1
    # the local display still shows what the sticks are doing
    assert "0.04" in window.dashboard_page.drive_card.vx_label.text()


def test_manual_twist_is_not_published_while_estopped(qtbot):
    # Not even a deflected stick: estop has no takeover path.
    gamepad = FakeGamepadReader(connected=True, twist=(0.04, 0.0, 0.05))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "estop")

    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages == []


def test_the_gamepad_disconnect_zero_is_gated_by_the_mode_too(qtbot):
    # A zero is never a takeover, so the fail-safe zero stays gated by the
    # stream gate alone. Centred sticks throughout, so nothing else can
    # account for a published message.
    gamepad = FakeGamepadReader(connected=True, twist=(0.0, 0.0, 0.0))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "autonomous")

    window._poll_gamepad()
    gamepad.set_connected(False)
    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages == []


def test_a_mode_status_that_stops_arriving_does_not_reopen_the_stream(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.0, 0.0, 0.0))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "autonomous")
    window._on_connection_changed(False)
    window._on_connection_changed(True)

    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages == []


def test_an_unreadable_mode_status_does_not_reopen_the_stream(qtbot):
    # A garbled /mode_status frame parses to None. Ignoring it keeps the
    # last known mode; storing it would reopen the zero stream mid-run.
    gamepad = FakeGamepadReader(connected=True, twist=(0.0, 0.0, 0.0))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    _mode_status(window, "autonomous")
    window._on_mode_status(None)
    window._poll_gamepad()
    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages == []


def test_stop_sends_both_the_estop_request_and_the_chassis_stop(qtbot):
    window, _ = make_window(qtbot, initial_host="localhost")

    window.dashboard_page.drive_row.stop_requested.emit()

    estop = next(t for t in FakeTopic.instances if t.name == "/estop_request")
    assert json.loads(estop.published_messages[-1]["data"])["reason"]
    command = next(t for t in FakeTopic.instances if t.name == "/drive_command")
    assert json.loads(command.published_messages[-1]["data"]) == {"action": "stop"}


def test_stop_sends_the_estop_request_in_autonomous_mode_too(qtbot):
    window, _ = make_window(qtbot, initial_host="localhost")
    _mode_status(window, "autonomous")

    window.dashboard_page.drive_row.stop_requested.emit()

    estop = next(t for t in FakeTopic.instances if t.name == "/estop_request")
    assert len(estop.published_messages) == 1


def test_manual_asks_for_the_mode_before_the_coordinator(qtbot):
    window, _ = make_window(qtbot, initial_host="localhost")

    window.dashboard_page.drive_row.manual_requested.emit()

    request = next(t for t in FakeTopic.instances if t.name == "/mode_request")
    assert json.loads(request.published_messages[-1]["data"]) == {"mode": "manual"}
    command = next(t for t in FakeTopic.instances if t.name == "/drive_command")
    assert json.loads(command.published_messages[-1]["data"]) == {"action": "manual"}


def test_the_mode_status_reaches_the_drive_row(qtbot):
    window, _ = make_window(qtbot, initial_host="localhost")
    _mode_status(window, "autonomous")
    assert window.dashboard_page.drive_row.mode_pill.text() == "AUTONOMOUS"


def test_the_window_subscribes_to_mode_status_on_connect(qtbot):
    make_window(qtbot, initial_host="localhost")
    assert any(t.name == "/mode_status" for t in FakeTopic.instances)
```

- [ ] Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_drive_row.py tests/test_main_window.py -q`
      Expected: failures — `AttributeError: 'DriveRow' object has no attribute 'mode_pill'` and `AttributeError: 'MainWindow' object has no attribute '_on_mode_status'`.

- [ ] In `ground_station/ui/drive_row.py`, add the state field in `__init__`, directly after `self._state = None`:

```python
        self._mode_state = None
```

- [ ] In `ground_station/ui/drive_row.py`, create the pill in the status-chips block, directly after `self.no_status_label.setStyleSheet(...)`:

```python
        # The supervisor's mode, not the bridge's status - a separate rover
        # node, so a separate pill with its own setter. It goes first
        # because it is the chip that explains why the sticks are or are
        # not reaching the wheels.
        self.mode_pill = QLabel()
```

- [ ] In `ground_station/ui/drive_row.py`, add `self.mode_pill` to the plain-text loop:

```python
        for w in (self.mode_pill, self.lease_pill, self.state_pill,
                  self.twist_age_label, self.last_action_label, self.error_pill):
            w.setTextFormat(Qt.TextFormat.PlainText)
```

- [ ] In `ground_station/ui/drive_row.py`, add the pill to the status layout — replace the two lines that build `status_layout` with:

```python
        self.status_layout = QHBoxLayout()
        self.status_layout.addWidget(self.mode_pill)
        self.status_layout.addWidget(self.no_status_label)
```

- [ ] In `ground_station/ui/drive_row.py`, replace the last line of `__init__` (`self.set_state(None)`) with:

```python
        self.set_state(None)
        self.set_mode_state(None)
```

- [ ] In `ground_station/ui/drive_row.py`, add these two methods immediately after `set_state`:

```python
    def set_mode_state(self, state) -> None:
        """/mode_status, which is a different rover node's business from
        /drive_status - the supervisor's, not the bridge's. Hence its own
        setter and its own pill: either source can go quiet without
        blanking the other, and the mode is exactly what the operator needs
        when the bridge is the thing that has gone quiet."""
        self._mode_state = state
        self._refresh_mode()

    def _refresh_mode(self) -> None:
        s = self._mode_state
        if s is None:
            self.mode_pill.setVisible(False)
            return
        if s.estop_latched or s.mode == "estop":
            self.mode_pill.setText("E-STOP LATCHED")
            self.mode_pill.setStyleSheet(theme.pill_style(theme.BAD, "white"))
        elif s.mode == "autonomous":
            self.mode_pill.setText("AUTONOMOUS")
            self.mode_pill.setStyleSheet(theme.pill_style(theme.ACCENT, "#2a1600"))
        elif s.mode in ("manual", "semi_auto"):
            self.mode_pill.setText(s.mode.upper())
            self.mode_pill.setStyleSheet(theme.pill_style(theme.OK, "#0c1a0e"))
        else:
            # A mode this build does not know. Shown verbatim rather than
            # guessed at, and through _plain because it came off the wire.
            self.mode_pill.setText(_plain(f"MODE {s.mode}"))
            self.mode_pill.setStyleSheet(theme.pill_style(theme.OFF, theme.TEXT))
        self.mode_pill.setVisible(True)
```

- [ ] In `ground_station/ui/main_window.py`, replace line 12, `from ground_station.models import DriveCommandTracker, NodeRegistry`, with:

```python
from ground_station.models import (DriveCommandTracker, NodeRegistry,
                                   is_stick_deflected, may_publish_manual_twist,
                                   may_publish_takeover_twist)
```

- [ ] In `ground_station/ui/main_window.py`, add the field in `__init__`, directly after `self._drive_status_at: float | None = None`:

```python
        # The last /mode_status seen, or None if none has. Deliberately
        # never expired and never cleared on a rosbridge drop: the mode is
        # the rover's, not this link's, and forgetting it would reopen the
        # /manual_twist stream into an autonomous run the moment the link
        # hiccuped - which rule 1 would then read as a takeover.
        self._mode_state = None
```

- [ ] In `ground_station/ui/main_window.py`, replace the two DRIVE-row connections for stop and manual with:

```python
        drive_row.stop_requested.connect(self._on_stop_requested)
        drive_row.manual_requested.connect(self._on_manual_requested)
```

- [ ] In `ground_station/ui/main_window.py`, in `_connect_to`, add the signal connection after the `drive_status_received` one:

```python
        self.ros_client.signals.mode_status_received.connect(self._on_mode_status)
```

- [ ] In `ground_station/ui/main_window.py`, in `_connect_to`, add the subscription after `self.ros_client.subscribe_drive_status()`:

```python
            self.ros_client.subscribe_mode_status()
```

- [ ] In `ground_station/ui/main_window.py`, replace the body of `_poll_gamepad` (keeping its docstring, updated) with:

```python
    def _poll_gamepad(self) -> None:
        """Shows the gamepad's current stick-derived Twist on the Drive
        card/detail page unconditionally - this display never depends on a
        rosbridge connection, or on the mode.

        Publishing is gated. The continuous stream goes out only when a
        gamepad and a rosbridge connection are both present AND the rover's
        own /mode_status says manual or semi_auto (spec rule 5).

        In autonomous the stream stops but the operator is not locked out:
        a stick past the gamepad deadzone is still published, a centred one
        is not. That is what rule 5's own justification asks for - rule 1
        becomes "a real signal rather than a constant stream" - and it is
        what makes the supervisor's takeover reachable at all, since this
        is the only publisher of /manual_twist. No new threshold is
        invented: read_twist() has already applied gamepad_input.DEADZONE
        (0.1) per axis, so a non-zero component is past it by construction.

        On gamepad disconnect it publishes one zero-velocity Twist as a
        fail-safe stop - subject to the *stream* gate only, because a zero
        is never a takeover and a zero stream is exactly what the gate
        exists to silence - then stops publishing until the gamepad
        returns."""
        connected = self.gamepad_reader.poll()
        rosbridge_ready = self.ros_client is not None and self.ros_client.is_connected
        may_stream = rosbridge_ready and may_publish_manual_twist(self._mode_state)
        may_take_over = rosbridge_ready and may_publish_takeover_twist(self._mode_state)

        if connected:
            self._gamepad_was_connected = True
            linear_x, linear_y, angular_z = self.gamepad_reader.read_twist()
            self._update_drive_display(linear_x, linear_y, angular_z)
            if may_stream or (may_take_over and is_stick_deflected(
                    (linear_x, linear_y, angular_z))):
                self.ros_client.publish_manual_twist(linear_x, linear_y, angular_z)
        elif self._gamepad_was_connected:
            self._gamepad_was_connected = False
            self._update_drive_display(0.0, 0.0, 0.0)
            if may_stream:
                self.ros_client.publish_manual_twist(0.0, 0.0, 0.0)
```

- [ ] In `ground_station/ui/main_window.py`, add these three methods immediately after `_on_drive_status`:

```python
    def _on_mode_status(self, state) -> None:
        # A status that would not parse says nothing about the rover's
        # mode. Keeping the last known one is the safe direction:
        # forgetting it would reopen the /manual_twist stream into a
        # running autonomous mission for the same reason a rosbridge blip
        # must not (see __init__).
        if state is None:
            return
        self._mode_state = state
        self.dashboard_page.drive_row.set_mode_state(state)

    def _on_stop_requested(self) -> None:
        """STOP does two different things, in this order.

        /estop_request latches on the Orin and survives this link dying -
        it is the one that must go first, because it is the one that keeps
        working when the next message does not arrive. The chassis stop on
        /drive_command is what puts F2 on the wire through the primary
        right now. Neither replaces the other, and both are sent in every
        mode."""
        if self.ros_client is None:
            return
        self.ros_client.send_estop_request("ground station STOP")
        self.ros_client.send_drive_command("stop")

    def _on_manual_requested(self) -> None:
        """The DRIVE row's Manual button, which is also the only way back
        from a latched e-stop: /mode_request manual clears the supervisor's
        latch, then the coordinator is asked for Manual as before. In that
        order - asking the coordinator to arm while the supervisor is still
        latched would arm a rover that is still being held at zero."""
        if self.ros_client is None:
            return
        self.ros_client.send_mode_request("manual")
        self.ros_client.send_drive_command("manual")
```

- [ ] Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_drive_row.py tests/test_main_window.py -q`
      Expected: all pass, 17 more than before.

- [ ] Run the whole ground-station suite: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/ -q`
      Expected: 280 passed (249 + 14 from Task 7 + 17 here). If `test_gamepad_publishes_manual_twist_when_gamepad_and_rosbridge_both_present`, `test_gamepad_disconnect_sends_one_zero_velocity_stop_then_stays_quiet` or any other pre-existing gamepad test now fails, it is because it never sets a mode status — and `may_publish_manual_twist(None)` returns True, so it must still pass. A failure there is a real regression in the gate, not a test to update.

- [ ] Run the rover suites once more so the whole SP is green together:
      `bash -c 'cd /home/ole/star/Navi && source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/ -q && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/ -q'`
      Expected: both suites pass.

- [ ] Commit: `git add ground_station tests && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "$(cat <<'EOF'
ground station: the publish gate, the STOP path, and a mode chip

/manual_twist now streams only while the rover says manual or semi_auto.
In autonomous the zero stream stops but a stick past the gamepad deadzone
is still published, so a moved stick is a takeover signal instead of one
message in a stream of zeros - and rule 1 stays reachable. STOP sends /estop_request first - the latch that
survives this link - then the chassis stop as before. Manual sends
/mode_request first, which is the only way back from a latched e-stop.
The mode is never forgotten on a rosbridge drop: forgetting it would
reopen the stream into a running autonomous mission.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
EOF
)"`

---

## Done means

- `mode_supervisor` is the only publisher of `/rover_twist`, and `bema_bridge` subscribes to nothing else by default.
- All five of spec §4's rules exist as named, passing tests:
  1. takeover — `test_a_stick_above_the_deadzone_takes_over_from_autonomy`, `test_the_twist_that_took_over_drives_on_the_same_tick`, `test_a_takeover_cancels_nav2_and_drives_the_coordinator`, and reachable end to end because `test_a_deflected_stick_is_published_in_autonomous_mode` shows the ground station still publishes a deflected stick while autonomous
  1b. the operator's other exit from autonomous, the DRIVE row's Manual button, carries the same coordinator ordering — `abort` then `startManual` — via `test_leaving_autonomous_by_request_cancels_the_goal`
  2. latched local STOP — `test_estop_zeroes_the_output_and_latches`, `test_only_a_manual_mode_request_clears_the_latch`, `test_an_unparseable_estop_request_still_stops`, `test_stop_sends_both_the_estop_request_and_the_chassis_stop`
  3. localisation loss — `test_localisation_searching_halts_autonomy_and_says_why`, `test_localisation_recovering_does_not_resume_autonomy`, `test_localisation_loss_does_not_stop_manual_driving`
  4. link loss — `test_link_loss_does_not_stop_autonomy`, `test_link_loss_stops_manual_via_the_deadman_and_never_clears_the_estop`
  5. GS publish policy — `test_a_centred_stick_publishes_nothing_in_autonomous_mode`, `test_a_deflected_stick_is_published_in_autonomous_mode`, `test_manual_twist_is_not_published_while_estopped`, `test_an_unreadable_mode_status_does_not_reopen_the_stream` and the rest beside them. The zero stream dies in `autonomous`; a stick past `gamepad_input.DEADZONE` still goes out, which is what rule 1 needs.
- `docs/superpowers/specs/autonomy-plan.md` §8's SP5 row is satisfied; SP8 and SP11 can now depend on it.
- Nothing in SP5 can *request* `autonomous` or `semi_auto`: `send_mode_request` exists but only the DRIVE row's Manual button calls it, because the NAV row is SP11. To exercise the autonomous path end to end before SP11, put the supervisor there by hand with `ros2 topic pub --once /mode_request std_msgs/String '{data: "{\"mode\": \"autonomous\"}"}'`. The mode is not dead; it has no button yet.
- Not in this SP, by design: Nav2 itself (SP9 fills `Nav2Control`), the NAV row and `goal_relay` (SP11), `/nav_status` and `/nav_path_summary` (SP11), `twist_shaper` between the supervisor and the bridge (SP10).
