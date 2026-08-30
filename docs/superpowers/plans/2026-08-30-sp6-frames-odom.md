# SP6: Static Frames and odom_local Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Nav2 can resolve `base_footprint` and `base_link` in TF against the ZED's single tree, and has an `odom`-frame `nav_msgs/Odometry` stream (`/localization/odom_local`) to read the rover's velocity from. Nothing else on the rover changes behaviour.

**Architecture:** Two `tf2_ros static_transform_publisher` entries in `localization.launch.py` hang `base_footprint` under the ZED's `zed_front_camera_link` (the inverse of the mount constant) and `base_link` under `base_footprint`, so the wrapper keeps sole ownership of `map → odom → zed_front_camera_link` and the tree stays single-rooted. `localization_status` gains one subscription to the wrapper's `/zed_front/zed_node/odom` and republishes it at `base_footprint` as `/localization/odom_local`, through a pure `odom_local.py` builder that reuses the same `pose_composition` arithmetic `/localization/pose` already uses, with the rigid-body lever-arm correction applied to the twist.

**Tech Stack:** Python 3.10, ROS 2 Humble (`rclpy`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `launch_ros`), colcon, pytest.

**Spec:** `docs/superpowers/specs/autonomy-plan.md` §6 (binding), §2 and §8 SP6 row for context; background in `docs/research/2026-08-29-autonomy-research.md` §4 and `docs/superpowers/specs/2026-08-29-localisation-design.md`.

---

## Global Constraints

- **The ZED 2i magnetometer must never be used.** Heading comes from the visual-inertial tracking only; no new subscription may match `mag`.
- **The ZED wrapper remains the sole owner of `map → odom → zed_front_camera_link`.** Nothing added here may publish those transforms, and nothing may make `zed_front_camera_link` the *child* of a transform we publish.
- **No `robot_state_publisher` on the Orin.** It would give `zed_front_camera_link` a second parent and break the tree. Only `static_transform_publisher` entries, and only for frames *below* the camera.
- **Never publish to `/manual_twist` in tests.** Tests that need a live ROS graph use a throwaway `ROS_DOMAIN_ID` of 91, 92 or 93 — never domain 0. No task in this plan needs a live graph.
- Commits: `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`, explicit `git add <paths>`, never push, never `git add -A`; on `index.lock` wait 2 s and retry (other agents work in the same tree on other files — SP4/SP5/SP7 plans and sources are theirs, do not touch them).
- **Laptop test command** (the gate for every task in this plan):
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  ```
  Baseline before this plan: **164 passed**. `test_localization_status.py` is excluded because `zed_msgs` is not installed on the laptop (verified 2026-08-30: `ModuleNotFoundError: No module named 'zed_msgs'`) — that file imports `localization_status`, which imports `zed_msgs.msg`. Everything this plan adds is therefore designed to be testable *without* `zed_msgs`, in pure modules; the node itself keeps only a one-line adapter.
- **URDF test command** (Task 1 only): `python3 -m pytest tests/test_mount_offset_agrees_with_urdf.py tests/test_urdf.py -q -p no:cacheprovider`. Baseline: **27 passed**. (The whole `tests/` directory does not collect on a machine without PySide6; run only these two files.)
- Build check after Task 2 and Task 4: `cd /home/ole/star/Navi/rover && colcon build --packages-select navi_localization`.

## Verified numbers (the code is ground truth, and the spec agrees)

| Thing | Code / URDF | Spec §6 | Verdict |
|---|---|---|---|
| `CAMERA_IN_BASE_FOOTPRINT` | `Transform(0.345, 0.0, 0.548, 0,0,0,1)` in `pose_composition.py` | — | — |
| `zed_front_camera_link → base_footprint` | inverse of the above = `(−0.345, 0, −0.548)`, identity rotation | `(−0.345, 0, −0.548)` | **matches** |
| `base_footprint → base_link` | URDF `base_footprint_joint` origin `xyz="0 0 0.409"` | `(0, 0, 0.409)` | **matches** |
| Derivation of `0.548` | URDF `zed_front_camera_joint` `xyz="0.345 0 0.139"` + `0.409` = `0.548` | — | consistent |

No discrepancy to record. `tests/test_mount_offset_agrees_with_urdf.py` already pins the camera constant to the URDF; Task 1 extends it to pin the `base_link` constant too.

## Design decisions

1. **`static_transform_publisher` in the launch file, not a broadcaster node.** Two fixed transforms need no state, no callback and no package code; a `tf2_ros` node already latches `/tf_static` correctly, and putting them in `localization.launch.py` means they start exactly where the ZED wrapper starts and die with it — a broadcaster node would be ~60 lines and an entry point to keep alive for zero added capability. The numbers are still "one number in one place": the launch file imports `CAMERA_IN_BASE_FOOTPRINT` from `pose_composition` and inverts it at launch time rather than retyping it.
2. **`/localization/odom_local` comes from the wrapper's `/zed_front/zed_node/odom`, re-expressed.** Not from TF (the package deliberately has no `tf2` dependency — `pose_composition.py`'s docstring makes that a design rule — and a TF lookup would add a wait-for-transform failure mode to a node that has none), and not from the pose callback (that is the `map` frame, which is the wrong frame for Nav2's `odom_topic` and jumps on loop closure). The wrapper's odom message is already `nav_msgs/Odometry` in `odom → zed_front_camera_link` at the wrapper's publish rate, so re-expressing it through the *same* `footprint_pose_from_camera_pose` the `map` pose uses is the smallest, most consistent change.
3. **Covariance is passed through unchanged, pose and twist both.** The mount rotation is identity, so no rotation of the covariance blocks is needed; the lever arm does couple angular uncertainty into the linear block, but that term is not propagated because nothing reads it — Nav2's controller server and velocity smoother read `twist.twist`, the collision monitor reads `cmd_vel`, and inventing a propagated covariance would be a number nobody checks. Documented in `odom_local.py` so the next reader does not think it was forgotten.
4. **`odom_local` is not gated by the localisation tracker, and is published only on receipt.** `/localization/pose` freezes the last good pose while SEARCHING because the ground station needs a position to draw. `odom` is the opposite contract: it is the continuous, non-jumping frame Nav2 reads for velocity feedback, and repeating a stale twist would tell the controller the rover had stopped when it has not. When the wrapper goes silent the topic goes silent, which Nav2's odometry smoother already handles.
5. **The twist is corrected for the lever arm, not merely copied.** `base_footprint` sits 0.345 m behind and 0.548 m below the camera, so at a yaw rate of 0.5 rad/s the two points differ by 0.17 m/s laterally. `v_footprint = R·v_camera + (R·ω_camera) × (−p)` with `p = CAMERA_IN_BASE_FOOTPRINT`'s translation — six lines of pure arithmetic in `pose_composition.py`, tested there. This assumes the wrapper's twist is expressed in `child_frame_id` (the camera body), per its own convention; that assumption is unchecked until the Task 4 rover verification confirms it.

---

### Task 1: The static-frame constants, pure

**Files:**
- Modify `rover/src/navi_localization/navi_localization/pose_composition.py`
- Modify (test) `rover/src/navi_localization/test/test_pose_composition.py`
- Modify (test) `tests/test_mount_offset_agrees_with_urdf.py`

**Interfaces:**
- Consumes: `CAMERA_IN_BASE_FOOTPRINT: Transform`, `inverse()` (both already in `pose_composition`); the URDF `asterope_iiI.urdf` joint `base_footprint_joint`.
- Produces: `pose_composition.BASE_LINK_IN_BASE_FOOTPRINT: Transform`; `pose_composition.STATIC_FRAMES: tuple[tuple[str, str, Transform], ...]` — `(parent_frame, child_frame, transform)` triples, parent-to-child, ready for a static transform publisher.

Steps:

- [ ] **Failing test.** Append to `rover/src/navi_localization/test/test_pose_composition.py`:

```python
def test_base_link_sits_where_the_urdf_base_footprint_joint_puts_it():
    assert BASE_LINK_IN_BASE_FOOTPRINT == Transform(0.0, 0.0, 0.409, 0.0, 0.0, 0.0, 1.0)


def test_the_static_frames_are_the_two_the_wrapper_does_not_own():
    assert [(parent, child) for parent, child, _ in STATIC_FRAMES] == [
        ("zed_front_camera_link", "base_footprint"),
        ("base_footprint", "base_link"),
    ]


def test_the_camera_static_frame_is_the_mount_constant_inverted():
    _, _, camera_to_footprint = STATIC_FRAMES[0]
    approx(camera_to_footprint, -0.345, 0.0, -0.548, 0.0)
    # Composing the two directions has to land back on nothing: this is the
    # test that catches a sign flip, which reads plausibly either way.
    approx(compose(CAMERA_IN_BASE_FOOTPRINT, camera_to_footprint), 0.0, 0.0, 0.0, 0.0)


def test_no_static_frame_gives_the_wrappers_link_a_second_parent():
    # The ZED wrapper owns map -> odom -> zed_front_camera_link. A transform
    # of ours whose *child* is that link would give it two parents and split
    # the tree - the single failure this whole arrangement exists to avoid.
    children = [child for _, child, _ in STATIC_FRAMES]
    assert "zed_front_camera_link" not in children
    assert len(children) == len(set(children)), "a frame may have only one parent"
```

  and extend the import at the top of that file to:

```python
from navi_localization.pose_composition import (
    BASE_LINK_IN_BASE_FOOTPRINT, CAMERA_IN_BASE_FOOTPRINT, IDENTITY,
    STATIC_FRAMES, Transform, compose, footprint_pose_from_camera_pose,
    inverse, translation_distance, yaw_of)
```

  Append to `tests/test_mount_offset_agrees_with_urdf.py`:

```python
def test_the_base_link_constant_matches_the_urdf_base_footprint_joint():
    robot = ET.parse(URDF).getroot()
    base = origin_of(robot, "base_footprint_joint")
    t = load_pose_composition().BASE_LINK_IN_BASE_FOOTPRINT
    assert (t.x, t.y, t.z) == pytest.approx((base[0], base[1], base[2]))
    assert (t.qx, t.qy, t.qz, t.qw) == (0.0, 0.0, 0.0, 1.0)
```

- [ ] **Run — expect failure.**
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  ```
  Expected: a collection error in `test_pose_composition.py` — `ImportError: cannot import name 'BASE_LINK_IN_BASE_FOOTPRINT' from 'navi_localization.pose_composition'`.
  ```
  python3 -m pytest tests/test_mount_offset_agrees_with_urdf.py tests/test_urdf.py -q -p no:cacheprovider
  ```
  Expected: `1 failed, 27 passed` — `AttributeError: module 'pose_composition' has no attribute 'BASE_LINK_IN_BASE_FOOTPRINT'`.

- [ ] **Implement.** In `rover/src/navi_localization/navi_localization/pose_composition.py`, immediately after the `MOUNT_OFFSET_VERIFIED = True` line, insert:

```python
# base_link above base_footprint: the URDF's base_footprint_joint, which is
# the wheel axle at -0.284 plus the 0.125 m wheel radius. Nav2 needs the link
# to exist in TF; nothing on the rover computes with it.
# tests/test_mount_offset_agrees_with_urdf.py keeps this equal to the URDF.
BASE_LINK_IN_BASE_FOOTPRINT = Transform(0.0, 0.0, 0.409, 0.0, 0.0, 0.0, 1.0)
```

  and at the end of the file (after `translation_distance`), append:

```python
# The only two transforms the Orin publishes on /tf_static, parent to child.
#
# The ZED wrapper owns map -> odom -> zed_front_camera_link and must stay its
# only owner, so base_footprint is hung *below* the camera rather than above
# it: topologically upside down relative to the URDF, but one tree with one
# root, which is what Nav2's costmap lookups need. Both entries add children,
# never a second parent - that is the invariant, and test_pose_composition.py
# asserts it. A robot_state_publisher on the Orin would violate it.
# (The simulation's own bringup does run one, with the full URDF, publishing
# these two edges the other way up. That is fine only while the sim and the
# rover never share a ROS domain.)
STATIC_FRAMES = (
    ('zed_front_camera_link', 'base_footprint', inverse(CAMERA_IN_BASE_FOOTPRINT)),
    ('base_footprint', 'base_link', BASE_LINK_IN_BASE_FOOTPRINT),
)
```

- [ ] **Run — expect PASS.** Both commands above. Expected: `168 passed` for the package suite (164 + 4), `28 passed` for the URDF pair.

- [ ] **Commit.**
  ```
  git add rover/src/navi_localization/navi_localization/pose_composition.py rover/src/navi_localization/test/test_pose_composition.py tests/test_mount_offset_agrees_with_urdf.py
  git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Name the two static frames the ZED wrapper does not own, from the mount constant and the URDF"
  ```

---

### Task 2: The static transforms in the launch file

**Files:**
- Modify `rover/src/navi_localization/launch/localization.launch.py`
- Modify `rover/src/navi_localization/package.xml`
- Modify (test) `rover/src/navi_localization/test/test_localization_launch.py`

**Interfaces:**
- Consumes: `pose_composition.STATIC_FRAMES`.
- Produces: `/tf_static` gains `zed_front_camera_link → base_footprint` and `base_footprint → base_link` (latched, from two `tf2_ros/static_transform_publisher` nodes named `camera_to_base_footprint_tf` and `base_footprint_to_base_link_tf`). New launch-module API: `static_transform_arguments(transform, frame_id, child_frame_id) -> list[str]` and `static_frame_arguments() -> list[list[str]]`.

Note for the implementer: the launch test may not call `generate_launch_description()` — it calls `get_package_share_directory('zed_wrapper')`, which raises `PackageNotFoundError` on the laptop. That is why the argument lists are built by a separate pure function, which the test *can* call, and the wiring into the `LaunchDescription` is asserted by reading the source, the idiom `test_the_launch_file_starts_the_elevation_mapper` already uses in this file.

Steps:

- [ ] **Failing test.** Append to `rover/src/navi_localization/test/test_localization_launch.py`:

```python
def _named(arguments):
    return {arguments[i]: arguments[i + 1] for i in range(0, len(arguments), 2)}


def test_the_static_transforms_are_the_two_from_pose_composition():
    argument_lists = _load_launch_module().static_frame_arguments()

    assert len(argument_lists) == 2
    camera, base_link = (_named(a) for a in argument_lists)

    assert camera['--frame-id'] == 'zed_front_camera_link'
    assert camera['--child-frame-id'] == 'base_footprint'
    assert float(camera['--x']) == pytest.approx(-0.345)
    assert float(camera['--y']) == pytest.approx(0.0)
    assert float(camera['--z']) == pytest.approx(-0.548)
    assert [float(camera[k]) for k in ('--qx', '--qy', '--qz', '--qw')] == [0.0, 0.0, 0.0, 1.0]

    assert base_link['--frame-id'] == 'base_footprint'
    assert base_link['--child-frame-id'] == 'base_link'
    assert float(base_link['--z']) == pytest.approx(0.409)


def test_negative_zero_never_reaches_the_command_line():
    # inverse() of a transform with a zero component produces -0.0, which
    # would be published correctly but read as a typo in `ros2 node info`.
    for arguments in _load_launch_module().static_frame_arguments():
        assert '-0.0' not in arguments


def test_the_launch_file_starts_the_static_transform_publishers():
    # The nodes are built in a loop over static_frame_arguments(), so the
    # count that matters is the argument lists', asserted above; what this
    # checks is that they are wired into the LaunchDescription at all.
    source = LAUNCH_FILE.read_text()

    assert "executable='static_transform_publisher'" in source
    assert "package='tf2_ros'" in source
    assert 'LaunchDescription([zed, status, mapper] + static_frame_nodes())' in source


def test_the_orin_never_starts_a_robot_state_publisher():
    # A robot_state_publisher here would publish the URDF root and re-parent
    # zed_front_camera_link, giving it a second parent and splitting the tree.
    # This only guards against this file starting one directly: with
    # publish_urdf: 'true', the wrapper's included zed_camera.launch.py does
    # start its own robot_state_publisher for the camera's xacro, rooted at
    # zed_front_camera_link (harmless, since it only adds children) — this
    # test proves less than its name suggests. The real evidence for "no
    # robot_state_publisher on the Orin" is the rover check elsewhere in this
    # plan.
    assert 'robot_state_publisher' not in LAUNCH_FILE.read_text()


def test_tf2_ros_is_declared_as_a_dependency():
    package_xml = (PACKAGE_ROOT / 'package.xml').read_text()

    assert '<exec_depend>tf2_ros</exec_depend>' in package_xml
```

  and add `import pytest` to that file's imports (it currently has none).

- [ ] **Run — expect failure.**
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  ```
  Expected: `4 failed, 169 passed` — the first two with `AttributeError: module
  'localization_launch' has no attribute 'static_frame_arguments'`, and
  `test_the_launch_file_starts_the_static_transform_publishers` and
  `test_tf2_ros_is_declared_as_a_dependency` with plain assertion failures.
  `test_the_orin_never_starts_a_robot_state_publisher` is green already: it is a
  regression guard, not a red test.

- [ ] **Implement.** In `rover/src/navi_localization/launch/localization.launch.py`:

  add to the imports, after `from launch_ros.actions import Node`:

```python
from navi_localization.pose_composition import STATIC_FRAMES
```

  add these three functions immediately above `def generate_launch_description():`:

```python
def _number(value: float) -> str:
    """A float for a command line. `+ 0.0` folds -0.0 (which inverse()
    produces from a zero component) back to 0.0, so the arguments read like
    the constant they came from."""
    return repr(value + 0.0)


def static_transform_arguments(transform, frame_id: str, child_frame_id: str) -> list:
    """tf2_ros static_transform_publisher's named arguments (Humble). The
    positional form is deprecated and silently reorders roll/pitch/yaw."""
    return [
        '--x', _number(transform.x),
        '--y', _number(transform.y),
        '--z', _number(transform.z),
        '--qx', _number(transform.qx),
        '--qy', _number(transform.qy),
        '--qz', _number(transform.qz),
        '--qw', _number(transform.qw),
        '--frame-id', frame_id,
        '--child-frame-id', child_frame_id,
    ]


def static_frame_arguments() -> list:
    """One argument list per entry in pose_composition.STATIC_FRAMES.

    Separated from static_frame_nodes() so the test can call it: this launch
    file's generate_launch_description() needs the zed_wrapper share
    directory, which does not exist on the laptop.
    """
    return [static_transform_arguments(transform, parent, child)
            for parent, child, transform in STATIC_FRAMES]


def static_frame_nodes() -> list:
    """base_footprint and base_link, hung under the frame the ZED wrapper
    owns.

    Two static_transform_publisher processes rather than a broadcaster node
    of our own: the transforms are fixed, tf2_ros already latches /tf_static
    correctly, and starting them here means they live and die with the
    wrapper they hang from. The numbers are not retyped - they are
    pose_composition's constants, the same ones localization_status uses to
    re-express the pose, so a re-measured mount is still one number in one
    place.

    Deliberately NOT a robot_state_publisher: it would publish the URDF root
    and make zed_front_camera_link a child of base_link, giving that link a
    second parent and splitting the tree the wrapper owns.
    """
    names = ['camera_to_base_footprint_tf', 'base_footprint_to_base_link_tf']
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=arguments,
            output='screen',
        )
        for name, arguments in zip(names, static_frame_arguments())
    ]
```

  and change the final line of `generate_launch_description()` from

```python
    return LaunchDescription([zed, status, mapper])
```

  to

```python
    return LaunchDescription([zed, status, mapper] + static_frame_nodes())
```

  In `rover/src/navi_localization/package.xml`, add after the `<exec_depend>zed_wrapper</exec_depend>` line:

```xml
  <!-- localization.launch.py runs two static_transform_publisher nodes. -->
  <exec_depend>tf2_ros</exec_depend>
```

- [ ] **Run — expect PASS.**
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  cd /home/ole/star/Navi/rover && colcon build --packages-select navi_localization
  ```
  Expected: `173 passed`, and a clean build (`Summary: 1 package finished`).

- [ ] **Rover verification — record the output, not a gate for the commit** (the ZED wrapper is not installed on this laptop, so this runs in the next Orin session). After `./start_navi.sh`:
  - `ros2 run tf2_ros tf2_echo map base_footprint` prints a transform within 2 s.
  - `ros2 run tf2_ros tf2_echo base_footprint base_link` prints `Translation: [0.000, 0.000, 0.409]`.
  - `ros2 run tf2_tools view_frames` shows **one** tree rooted at `map`, with `zed_front_camera_link` having exactly one parent (`odom`).
  - `ros2 topic echo /tf_static --once` after both publishers are up.
  If the wrapper's own `publish_urdf: true` robot_state_publisher turns out to publish a `base_link → zed_front_camera_link` joint (it should not — wrapper 4.2's xacro roots at `<camera_name>_camera_link`), that is a second parent and the fix is `publish_urdf: 'false'` in the launch arguments, not a change here. Note the outcome in the plan file.

- [ ] **Commit.**
  ```
  git add rover/src/navi_localization/launch/localization.launch.py rover/src/navi_localization/package.xml rover/src/navi_localization/test/test_localization_launch.py
  git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Hang base_footprint and base_link under the ZED's frame with two static transform publishers, so the tree stays single-rooted"
  ```

---

### Task 3: The twist at base_footprint, pure

**Files:**
- Modify `rover/src/navi_localization/navi_localization/pose_composition.py`
- Modify (test) `rover/src/navi_localization/test/test_pose_composition.py`

**Interfaces:**
- Consumes: `CAMERA_IN_BASE_FOOTPRINT`, `_rotate` (both already in `pose_composition`).
- Produces: `pose_composition.footprint_twist_from_camera_twist(linear, angular, camera_in_footprint=CAMERA_IN_BASE_FOOTPRINT) -> tuple[tuple[float, float, float], tuple[float, float, float]]` — the camera's body twist, expressed at `base_footprint` in `base_footprint`'s axes.

Steps:

- [ ] **Failing test.** Append to `rover/src/navi_localization/test/test_pose_composition.py`:

```python
def test_a_pure_translation_is_the_same_at_both_points():
    linear, angular = footprint_twist_from_camera_twist((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert linear == pytest.approx((1.0, 0.0, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 0.0))


def test_a_yaw_rate_moves_the_footprint_sideways():
    # base_footprint is 0.345 m behind the camera, so turning left at
    # 1 rad/s about the camera drags it 0.345 m/s to its own right.
    linear, angular = footprint_twist_from_camera_twist((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    assert linear == pytest.approx((0.0, -0.345, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 1.0))


def test_a_pitch_rate_lifts_and_pushes_the_footprint():
    # 0.548 m below and 0.345 m behind: pitching nose-up about the camera
    # swings the footprint backwards and upwards.
    linear, angular = footprint_twist_from_camera_twist((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    assert linear == pytest.approx((-0.548, 0.0, 0.345))


def test_a_roll_rate_swings_the_footprint_sideways():
    linear, angular = footprint_twist_from_camera_twist((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert linear == pytest.approx((0.0, 0.548, 0.0))


def test_translation_and_rotation_add():
    linear, _ = footprint_twist_from_camera_twist((0.5, 0.0, 0.0), (0.0, 0.0, 2.0))
    assert linear == pytest.approx((0.5, -0.690, 0.0))


def test_a_mount_offset_of_zero_changes_nothing():
    linear, angular = footprint_twist_from_camera_twist(
        (1.0, 2.0, 3.0), (0.1, 0.2, 0.3), IDENTITY)
    assert linear == pytest.approx((1.0, 2.0, 3.0))
    assert angular == pytest.approx((0.1, 0.2, 0.3))


def test_a_rotated_mount_rotates_the_twist_into_the_footprints_axes():
    # A camera mounted yawed 90 deg at the footprint's own origin: its +x is
    # the footprint's +y, so a forward twist reads sideways at the rover.
    offset = Transform(0.0, 0.0, 0.0, *quat_z(math.pi / 2))
    linear, angular = footprint_twist_from_camera_twist((1.0, 0.0, 0.0), (0.0, 0.0, 0.5), offset)
    assert linear == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert angular == pytest.approx((0.0, 0.0, 0.5), abs=1e-9)
```

  and add `footprint_twist_from_camera_twist` to that file's `from navi_localization.pose_composition import (...)` list.

- [ ] **Run — expect failure.**
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  ```
  Expected: a collection error in `test_pose_composition.py` — `ImportError: cannot import name 'footprint_twist_from_camera_twist' from 'navi_localization.pose_composition'`.

- [ ] **Implement.** In `rover/src/navi_localization/navi_localization/pose_composition.py`, add after `_rotate`:

```python
def _cross(a, b):
    ax, ay, az = a
    bx, by, bz = b
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)
```

  and add after `footprint_pose_from_camera_pose`:

```python
def footprint_twist_from_camera_twist(
        linear, angular,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT):
    """The camera's body twist, re-expressed at base_footprint.

    Both arguments are (x, y, z) in the camera's own axes - the ROS
    convention for nav_msgs/Odometry.twist, which is expressed in
    child_frame_id. The result is the same for base_footprint.

    Two points on one rigid body do not share a linear velocity: with R and p
    the mount's rotation and translation (camera in footprint),

        omega_footprint = R * omega_camera
        v_footprint     = R * v_camera + omega_footprint x (-p)

    Not an optional refinement. The camera sits 0.345 m ahead of and 0.548 m
    above base_footprint, so at a 0.5 rad/s yaw the two points differ by
    0.17 m/s sideways, which is the size of the speeds the controller works
    with.
    """
    q = (camera_in_footprint.qx, camera_in_footprint.qy,
         camera_in_footprint.qz, camera_in_footprint.qw)
    vx, vy, vz = _rotate(q, tuple(linear))
    omega = _rotate(q, tuple(angular))
    lx, ly, lz = _cross(omega, (-camera_in_footprint.x,
                                -camera_in_footprint.y,
                                -camera_in_footprint.z))
    return (vx + lx, vy + ly, vz + lz), omega
```

- [ ] **Run — expect PASS.** Same command. Expected: `180 passed` (173 + 7).

- [ ] **Commit.**
  ```
  git add rover/src/navi_localization/navi_localization/pose_composition.py rover/src/navi_localization/test/test_pose_composition.py
  git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Re-express the camera's twist at base_footprint, lever arm included, so a turn does not read as sideways drift"
  ```

---

### Task 4: `/localization/odom_local`

**Files:**
- Create `rover/src/navi_localization/navi_localization/odom_local.py`
- Create (test) `rover/src/navi_localization/test/test_odom_local.py`
- Modify `rover/src/navi_localization/navi_localization/localization_status.py`
- Modify (test) `rover/src/navi_localization/test/test_localization_status.py` (Orin-only; see the note below)

**Interfaces:**
- Consumes: `/zed_front/zed_node/odom` (`nav_msgs/Odometry`, `frame_id: odom`, `child_frame_id: zed_front_camera_link`, published by the ZED wrapper at `pub_frame_rate`, currently 15 Hz).
- Produces: `/localization/odom_local` (`nav_msgs/Odometry`, `frame_id: odom`, `child_frame_id: base_footprint`), for Nav2's `odom_topic` in `controller_server` and `velocity_smoother` (SP9 wires it).
- New module API: `odom_local.odom_local_from_camera_odometry(msg, camera_in_footprint=CAMERA_IN_BASE_FOOTPRINT) -> Odometry`, `odom_local.ODOM_FRAME`, `odom_local.BASE_FRAME`.

Note for the implementer: the message building lives in `odom_local.py`, which imports `nav_msgs` but not `zed_msgs`, so `test_odom_local.py` runs on the laptop and is a real gate. `localization_status.py` keeps only a one-line adapter; the node-level tests added to `test_localization_status.py` cannot run here (no `zed_msgs`) and are for the Orin — write them, do not expect them to run in this session.

Steps:

- [ ] **Failing test.** Create `rover/src/navi_localization/test/test_odom_local.py`:

```python
"""The wrapper's odom message, re-expressed at base_footprint.

Pure message arithmetic, no node: this is what /localization/odom_local
carries, and it is the frame and the twist Nav2's controller reads.
"""

import math

import pytest
from nav_msgs.msg import Odometry

from navi_localization.odom_local import (
    BASE_FRAME, ODOM_FRAME, odom_local_from_camera_odometry)


def camera_odometry(x=0.345, y=0.0, z=0.548, yaw=0.0, stamp_sec=100.0,
                    linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9))
    msg.child_frame_id = "zed_front_camera_link"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = linear
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
    return msg


def test_the_frames_are_the_ones_nav2_is_configured_for():
    out = odom_local_from_camera_odometry(camera_odometry())

    assert out.header.frame_id == ODOM_FRAME == "odom"
    assert out.child_frame_id == BASE_FRAME == "base_footprint"


def test_the_camera_pose_becomes_the_footprint_pose():
    # The camera exactly at its mount offset means the footprint is at the
    # odom origin.
    out = odom_local_from_camera_odometry(camera_odometry())

    assert out.pose.pose.position.x == pytest.approx(0.0, abs=1e-9)
    assert out.pose.pose.position.y == pytest.approx(0.0, abs=1e-9)
    assert out.pose.pose.position.z == pytest.approx(0.0, abs=1e-9)
    assert out.pose.pose.orientation.w == pytest.approx(1.0)


def test_the_offset_rotates_with_the_camera():
    out = odom_local_from_camera_odometry(
        camera_odometry(x=10.0, y=5.0, yaw=math.pi / 2))

    assert out.pose.pose.position.x == pytest.approx(10.0, abs=1e-9)
    assert out.pose.pose.position.y == pytest.approx(5.0 - 0.345, abs=1e-9)
    assert out.pose.pose.position.z == pytest.approx(0.0, abs=1e-9)


def test_the_stamp_is_the_wrappers_own():
    out = odom_local_from_camera_odometry(camera_odometry(stamp_sec=1234.5))

    assert out.header.stamp.sec == 1234
    assert out.header.stamp.nanosec == pytest.approx(500000000, abs=2)


def test_a_straight_line_twist_passes_through():
    out = odom_local_from_camera_odometry(camera_odometry(linear=(0.4, 0.0, 0.0)))

    assert out.twist.twist.linear.x == pytest.approx(0.4)
    assert out.twist.twist.linear.y == pytest.approx(0.0)
    assert out.twist.twist.angular.z == pytest.approx(0.0)


def test_a_yaw_rate_gets_the_lever_arm_correction():
    # A copied twist would say the rover is turning in place; the footprint
    # is 0.345 m behind the camera and is actually swinging sideways.
    out = odom_local_from_camera_odometry(camera_odometry(angular=(0.0, 0.0, 0.5)))

    assert out.twist.twist.linear.y == pytest.approx(-0.1725)
    assert out.twist.twist.angular.z == pytest.approx(0.5)


def test_both_covariances_are_passed_through():
    msg = camera_odometry()
    msg.pose.covariance[0] = 0.25
    msg.twist.covariance[7] = 0.5

    out = odom_local_from_camera_odometry(msg)

    assert out.pose.covariance[0] == pytest.approx(0.25)
    assert out.twist.covariance[7] == pytest.approx(0.5)


def test_the_wrappers_message_is_not_modified():
    msg = camera_odometry(linear=(0.4, 0.0, 0.0), angular=(0.0, 0.0, 0.5))

    odom_local_from_camera_odometry(msg)

    assert msg.child_frame_id == "zed_front_camera_link"
    assert msg.pose.pose.position.x == pytest.approx(0.345)
    assert msg.twist.twist.linear.y == pytest.approx(0.0)
```

  and append to `rover/src/navi_localization/test/test_localization_status.py` (Orin-only):

```python
def test_the_wrappers_odom_is_republished_at_base_footprint(node):
    from nav_msgs.msg import Odometry

    node._odom_local_publisher = Recorder()
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.header.stamp.sec = 7
    msg.child_frame_id = "zed_front_camera_link"
    msg.pose.pose.position.x = 0.345
    msg.pose.pose.position.z = 0.548
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.angular.z = 1.0

    node._on_odom(msg)

    assert len(node._odom_local_publisher.messages) == 1
    out = node._odom_local_publisher.messages[0]
    assert out.header.frame_id == "odom"
    assert out.child_frame_id == "base_footprint"
    assert out.header.stamp.sec == 7
    assert out.pose.pose.position.x == pytest.approx(0.0, abs=1e-9)
    assert out.twist.twist.linear.y == pytest.approx(-0.345)


def test_odom_local_is_not_gated_by_the_tracker(node):
    # odom is the continuous frame Nav2 reads for velocity feedback. Holding
    # it back while the map pose is SEARCHING would tell the controller the
    # rover had stopped when it has not.
    from nav_msgs.msg import Odometry

    node._odom_local_publisher = Recorder()
    node._on_status(status(False))
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.3

    node._on_odom(msg)

    assert len(node._odom_local_publisher.messages) == 1
    assert node._odom_local_publisher.messages[-1].twist.twist.linear.x == pytest.approx(0.3)


def test_the_odom_subscription_is_the_wrappers_and_not_the_magnetometers(node):
    topics = [s.topic_name for s in node.subscriptions]
    assert "/zed_front/zed_node/odom" in topics
    assert not any("mag" in t for t in topics)
```

- [ ] **Run — expect failure.**
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  ```
  Expected: a collection error in `test_odom_local.py` — `ModuleNotFoundError: No module named 'navi_localization.odom_local'`.

- [ ] **Implement.** Create `rover/src/navi_localization/navi_localization/odom_local.py`:

```python
"""The ZED wrapper's odometry, re-expressed at base_footprint.

Nav2's controller server and velocity smoother read an `odom_topic` for the
rover's current velocity. /localization/pose is the wrong thing for that: it
is in the `map` frame, which jumps whenever the SDK closes a loop, and a
controller that sees a jump reads it as a large velocity. The `odom` frame is
the continuous one, and the wrapper already publishes it - about a frame that
is not the rover.

So this module does to /zed_front/zed_node/odom exactly what
localization_status does to the wrapper's map pose: the same mount constant,
the same arithmetic, one frame lower down. Kept out of localization_status.py
so it can be tested on a machine without zed_msgs installed - that node
imports zed_msgs at module level, this does not.

The covariances are passed through untouched. The mount rotation is identity,
so no rotation of the blocks is called for; the lever arm does couple angular
uncertainty into the linear block, but that term is deliberately not
propagated because nothing reads it (Nav2 reads twist.twist, the collision
monitor reads cmd_vel), and a number nobody checks is worse than an honest
copy.
"""

from nav_msgs.msg import Odometry

from navi_localization.pose_composition import (
    CAMERA_IN_BASE_FOOTPRINT, Transform, footprint_pose_from_camera_pose,
    footprint_twist_from_camera_twist)

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'


def odom_local_from_camera_odometry(
        msg: Odometry,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT) -> Odometry:
    """A new Odometry: the same instant, at base_footprint. `msg` is not
    modified."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    footprint = footprint_pose_from_camera_pose(
        Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w), camera_in_footprint)

    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    linear, angular = footprint_twist_from_camera_twist(
        (v.x, v.y, v.z), (w.x, w.y, w.z), camera_in_footprint)

    out = Odometry()
    out.header.stamp.sec = msg.header.stamp.sec
    out.header.stamp.nanosec = msg.header.stamp.nanosec
    out.header.frame_id = ODOM_FRAME
    out.child_frame_id = BASE_FRAME
    out.pose.pose.position.x = footprint.x
    out.pose.pose.position.y = footprint.y
    out.pose.pose.position.z = footprint.z
    out.pose.pose.orientation.x = footprint.qx
    out.pose.pose.orientation.y = footprint.qy
    out.pose.pose.orientation.z = footprint.qz
    out.pose.pose.orientation.w = footprint.qw
    out.pose.covariance = list(msg.pose.covariance)
    out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = linear
    out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = angular
    out.twist.covariance = list(msg.twist.covariance)
    return out
```

  In `rover/src/navi_localization/navi_localization/localization_status.py`:

  add to the imports, after the `from navi_localization.pose_composition import (...)` block:

```python
from navi_localization.odom_local import odom_local_from_camera_odometry
```

  add in `__init__`, immediately after the `self._status_publisher = ...` line:

```python
        self._odom_local_publisher = self.create_publisher(
            Odometry, '/localization/odom_local', 10)
```

  add in `__init__`, immediately after the `PosTrackStatus` subscription:

```python
        self.create_subscription(Odometry, '/zed_front/zed_node/odom',
                                 self._on_odom, 10)
```

  add this method immediately after `_on_pose`:

```python
    def _on_odom(self, msg: Odometry) -> None:
        # Deliberately not gated on the tracker, unlike /localization/pose.
        # odom is the continuous, non-jumping frame Nav2's controller reads
        # for velocity feedback; freezing it while the map pose is SEARCHING
        # would say the rover had stopped when it has not. And nothing is
        # repeated on the timer either: a stale twist says the same thing.
        self._odom_local_publisher.publish(odom_local_from_camera_odometry(msg))
```

  and extend the module docstring's second paragraph so the node's own header says what it publishes — replace

```python
Input is the ZED ROS 2 wrapper's positional tracking; output is the pose of
base_footprint in the map frame plus an OK / SEARCHING / OFF status. See
tracker.py for the rules and pose_composition.py for the frame arithmetic.
```

  with

```python
Input is the ZED ROS 2 wrapper's positional tracking; output is the pose of
base_footprint in the map frame plus an OK / SEARCHING / OFF status, and the
wrapper's own odometry re-expressed at base_footprint on
/localization/odom_local for Nav2's odom_topic. See tracker.py for the rules,
pose_composition.py for the frame arithmetic and odom_local.py for the odom
stream.
```

- [ ] **Run — expect PASS.**
  ```
  bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'
  cd /home/ole/star/Navi/rover && colcon build --packages-select navi_localization
  ```
  Expected: `188 passed` (180 + 8), clean build.

- [ ] **Rover verification — record the output, not a gate for the commit** (next Orin session):
  - `ros2 topic echo /zed_front/zed_node/odom --once` — **confirm `twist.twist` is populated**. If wrapper 4.2 publishes zeros there, `/localization/odom_local` carries zeros too, and SP9 must know before it configures `velocity_smoother` to trust the feedback. Record the answer here.
  - **MUST-DO before Nav2 consumes this topic — confirm the wrapper's twist is in the child frame, not the odom frame.** Yaw the rover roughly 90 deg away from its start heading, drive slowly forward, and echo `/zed_front/zed_node/odom`: `twist.twist.linear.x` must be the forward speed and `linear.y` near zero. If instead `linear.y` carries the speed, the wrapper is publishing a world-frame twist, `footprint_twist_from_camera_twist` is being fed the wrong frame, and the fix is to rotate the twist into the camera frame by the inverse of the message's own `pose.pose.orientation` before calling it. Record the answer here.
  - `ros2 topic hz /localization/odom_local` — expect ~15 Hz, matching `pub_frame_rate`.
  - `ros2 topic echo /localization/odom_local --once` — `frame_id: odom`, `child_frame_id: base_footprint`, position within a few cm of `tf2_echo odom base_footprint`.
  - Drive a slow point turn by hand and check `twist.twist.linear.y` is non-zero with the expected sign.

- [ ] **Commit.**
  ```
  git add rover/src/navi_localization/navi_localization/odom_local.py rover/src/navi_localization/navi_localization/localization_status.py rover/src/navi_localization/test/test_odom_local.py rover/src/navi_localization/test/test_localization_status.py
  git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Publish /localization/odom_local so Nav2 has a continuous odom-frame estimate at base_footprint"
  ```

---

## Out of scope, deliberately

- **A `frames` gate in `start_navi.sh`.** An earlier draft (`docs/superpowers/plans/2026-08-29-autonomy.md`, Task 6) proposed a `tf2_echo map base_footprint` check in the launcher. It is not in spec §6, and nothing on the rover needs the tree until Nav2 exists; SP9 (Nav2 bringup) is where a missing transform first becomes fatal, so the gate belongs there, next to the lifecycle manager it would protect. `rover/test/test_start_navi_gate.sh` already has the fake-`ros2` harness to test it when SP9 wants it.
- **Nav2 configuration.** `odom_topic: /localization/odom_local` goes in `config/nav2.yaml`, which is SP9's file.
- **`/localization/pose`.** Unchanged. It stays the ground station's and the simulation's interface; Nav2 never reads it.
- **Covariance propagation through the lever arm.** See design decision 3.

## Self-review

- **Spec coverage.** §6 has four claims: the camera→footprint static at `(−0.345, 0, −0.548)` (Task 2, from Task 1's constant); `base_footprint → base_link` at `(0, 0, 0.409)` (Task 2, from Task 1's constant); the wrapper stays sole owner of `map → odom → zed_front_camera_link` and no `robot_state_publisher` (asserted by `test_no_static_frame_gives_the_wrappers_link_a_second_parent` and `test_the_orin_never_starts_a_robot_state_publisher`, and by the rover check in Task 2); `localization_status` gains `/localization/odom_local` in the `odom` frame (Task 4). §8's SP6 row has no dependencies, and this plan takes none.
- **Placeholder scan.** No `TODO`, no `...`, no "implement here". Every code block above is complete and can be pasted as-is; the only prose-only steps are the two rover verifications, which are explicitly not commit gates.
- **Type consistency.** `STATIC_FRAMES` is `tuple[tuple[str, str, Transform], ...]` and is consumed by `static_frame_arguments()` as `for parent, child, transform in ...` — order matches. `footprint_twist_from_camera_twist` takes and returns 3-tuples of `float`; `odom_local.py` unpacks them into `Vector3` fields, and `test_pose_composition.py` compares them with `pytest.approx` on tuples, which works elementwise. `odom_local_from_camera_odometry` takes and returns `nav_msgs/Odometry`, matching both the wrapper's publication and the node's publisher type. `list(msg.pose.covariance)` converts the `numpy.ndarray` rclpy hands back into the `list` the existing `_publish_pose` already assigns, so both publishers agree.
- **Test-run arithmetic.** 164 baseline → 168 (Task 1) → 173 (Task 2) → 180 (Task 3) → 188 (Task 4), plus 27 → 28 in the URDF pair. The three `test_localization_status.py` tests are additional and Orin-only.
