# Rover Localisation (sub-project 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Orin publishes the rover's pose (`/localization/pose`) and its health (`/localization/status`) from the front ZED 2i's visual-inertial tracking, while manual-mode video keeps working from the same camera.

**Architecture:** `zed_wrapper` owns the camera permanently and runs positional tracking; a pure-Python `localization_status` node re-expresses the wrapper's `map -> zed_front_camera_link` pose at `base_footprint` and publishes a JSON status; `video_sender` gains a `zed_topic` source that pipes the wrapper's RGB image into the same gst front end the simulation sender uses. The Orin's git repository joins this one as `rover/` and is deployed by rsync + remote `colcon build`.

**Tech Stack:** ROS 2 Humble, `zed-ros2-wrapper humble-v4.2.5` (ZED SDK 4.2.5, prebuilt in `~/workspaces/isaac_ros-dev` on the Orin), `rclpy`, `ament_python`, pytest, GStreamer (`gst-launch-1.0`), bash.

**Spec:** `docs/superpowers/specs/2026-08-29-localisation-design.md` — sections "Repository layout" and "Sub-project 1".

## Global Constraints

- The ZED 2i's magnetometer is never subscribed or fused. Nothing reads `/zed_front/zed_node/imu/mag`.
- No wheel odometry is invented from the commanded twist.
- Nothing under `ground_station/` imports `rclpy` (unchanged; this plan does not touch it).
- `/manual_twist` drives the physical rover. Never publish to it during testing.
- Everything under `sim/src/navi_sim_ik/vendor/` is read-only.
- Commits: `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit ...` (no identity configured). Commit messages are full sentences describing the change.
- The Orin is `star@a_navi` (192.168.178.33), reachable with `ssh star@a_navi` (key and config already set up on this laptop; ignore the "remote port forwarding failed for listen port 1080" warning — it is harmless). Its ROS environment is entered with `source /opt/ros/humble/setup.bash && source ~/workspaces/isaac_ros-dev/install/setup.bash && source ~/navi/install/local_setup.bash`. The default shell there is bash.
- On this laptop `ros2` is only on PATH after `source /opt/ros/humble/setup.bash`, and the default shell is zsh where that fails — always `bash -c 'source /opt/ros/humble/setup.bash && ...'`.
- **Never use `pkill -f` or `pgrep -f` with a pattern that could match your own shell's command line** (e.g. over ssh, the pattern appears in the ssh command). Use `pkill -x <exact process name>` — e.g. `pkill -x component_container_isolated`.
- The wrapper takes ~90 s to publish `/zed_front/zed_node/odom` with its default configuration (Task 5 fixes this). Poll for the topic; do not assume a fixed sleep.
- Laptop tests: `.venv/bin/pytest tests/` from the repo root (pure Python, no ROS). Orin-side package tests run on the Orin: `cd ~/navi && colcon test --packages-select <pkg> && colcon test-result --verbose`, or faster, `python3 -m pytest src/<pkg>/test -q` from `~/navi` after sourcing.

## File structure

```
rover/                                   # git subtree of the Orin's ~/navi
  start_navi.sh                          # modified: --no-localization, cleanup, readiness
  src/navi_teleop/
    navi_teleop/video_sender.py          # modified: `source` parameter, zed_topic path
    navi_teleop/image_pipe.py            # new: pipe front end + frame-size check (pure)
    test/test_video_sender.py            # modified
    test/test_image_pipe.py              # new
  src/navi_localization/
    package.xml, setup.py, setup.cfg, resource/navi_localization
    navi_localization/__init__.py
    navi_localization/pose_composition.py   # new: rigid-transform maths (pure)
    navi_localization/tracker.py            # new: OK/SEARCHING/OFF state machine (pure)
    navi_localization/localization_status.py # new: the ROS node
    config/zed_front.yaml                   # new: wrapper overrides
    launch/localization.launch.py           # new
    test/test_pose_composition.py
    test/test_tracker.py
    test/test_localization_status.py
deploy_rover.sh                          # new: rsync + remote colcon build + tests
asterope_iiI.urdf                        # modified: zed_front_camera_link, zed_rear_camera_link
tests/test_urdf.py                       # modified
tests/test_mount_offset_agrees_with_urdf.py  # new: URDF joint == Python constant
```

---

### Task 1: Bring the Orin repository in as `rover/` and add `deploy_rover.sh`

**Files:**
- Create: `rover/` (subtree), `deploy_rover.sh`
- Modify: `.gitignore`

**Interfaces:**
- Produces: `./deploy_rover.sh [--no-build] [--test]` — rsyncs `rover/` to `star@a_navi:~/navi/`, runs `colcon build --symlink-install` there, and with `--test` runs the Orin-side pytest suites. Exit status is the remote command's.

- [ ] **Step 1: Add the Orin as a git remote and fetch its history**

```bash
cd /home/ole/star/Navi
git remote add orin ssh://star@a_navi/home/star/navi
git fetch orin master
git log --oneline orin/master | head -3
```
Expected: the three most recent Orin commits, the top one `45b846b Clear a previous run's leftovers when start_navi.sh starts`.

- [ ] **Step 2: Add it as a subtree under `rover/`**

```bash
git subtree add --prefix=rover orin master -m "Bring the Orin's navi repository in as rover/ so both sides of the link live in one tree"
ls rover rover/src
```
Expected: `rover/start_navi.sh`, `rover/src/navi_teleop`. `git log --oneline -1` shows the merge commit.

- [ ] **Step 3: Write `deploy_rover.sh`**

```bash
#!/usr/bin/env bash
# Deploy rover/ to the Orin and build it there.
#
#   ./deploy_rover.sh            rsync, then colcon build on the Orin
#   ./deploy_rover.sh --test     ... and run the Orin-side test suites
#   ./deploy_rover.sh --no-build rsync only
#
# ~/navi on the Orin is a deploy target, not a development tree: edit under
# rover/ here, commit here, deploy with this. The Orin's build/ install/ log/
# are left alone (they are gitignored on both sides).

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_SSH="${ROVER_SSH:-star@a_navi}"
ROVER_DIR="${ROVER_DIR:-navi}"

BUILD=1
TEST=0
while [ $# -gt 0 ]; do
    case "$1" in
        --no-build) BUILD=0; shift ;;
        --test) TEST=1; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# --delete so a file removed here disappears there; the excludes keep the
# Orin's build products and its own .git out of the deletion.
rsync -az --delete \
    --exclude '.git' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
    --exclude '__pycache__/' --exclude '.pytest_cache/' \
    "$REPO_DIR/rover/" "$ROVER_SSH:$ROVER_DIR/"
echo "synced rover/ -> $ROVER_SSH:$ROVER_DIR/"

[ "$BUILD" -eq 1 ] || exit 0

# One remote shell, one script: sourcing ROS is per-shell state.
REMOTE='set -eo pipefail
source /opt/ros/humble/setup.bash
[ -f ~/workspaces/isaac_ros-dev/install/setup.bash ] && source ~/workspaces/isaac_ros-dev/install/setup.bash
cd ~/'"$ROVER_DIR"'
colcon build --symlink-install
'
if [ "$TEST" -eq 1 ]; then
    REMOTE+='source install/local_setup.bash
for pkg in src/*/; do
    [ -d "$pkg/test" ] || continue
    echo "== pytest $pkg"
    (cd "$pkg" && python3 -m pytest test -q)
done
'
fi
ssh "$ROVER_SSH" "$REMOTE"
```

```bash
chmod +x deploy_rover.sh
```

- [ ] **Step 4: Run it with tests**

Run: `./deploy_rover.sh --test`
Expected: rsync line, `colcon build` summary with `navi_teleop` finished, then `== pytest src/navi_teleop/` and a pytest pass line. If pytest reports import errors for `rclpy`, the ROS sourcing in `REMOTE` is wrong — fix that, do not skip tests.

- [ ] **Step 5: Ignore Orin build products in this repo too**

`.gitignore` already has `build/`, `install/`, `log/`. Confirm with `git status --short rover/` that nothing under `rover/` shows as untracked after the deploy (the deploy touches only the Orin). Nothing to add if clean.

- [ ] **Step 6: Commit**

```bash
git add deploy_rover.sh
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Add deploy_rover.sh: rsync rover/ to the Orin and build it there"
```

---

### Task 2: Pin the measured camera frames in the URDF

**Files:**
- Modify: `tests/test_urdf.py`
- `asterope_iiI.urdf`: **no change** — it already carries the frames.

**Interfaces:**
- Produces: nothing new. The URDF already has, from the hardware team, a full
  ZED frame tree for both cameras, laid out exactly as
  `zed_wrapper/urdf/zed_macro.urdf.xacro` does and "placed from the measured
  LEFT optical centres": `zed_front_camera_joint` (fixed, parent `base_link`,
  child `zed_front_camera_link`, origin `0.345 0 0.139`, rpy `0 0 0`) and
  `zed_rear_camera_joint` (child `zed_rear_camera_link`, origin
  `-0.345 0 0.139`, rpy `0 0 3.141592654`), each with `*_camera_center`
  (+0.015 z), left/right frames and optical frames beneath. `camera_link` is
  the 1/4" mounting screw, which is also what the ZED wrapper tracks. Task 3's
  `CAMERA_IN_BASE_FOOTPRINT` is this front origin plus `base_footprint_joint`'s
  z (0.409): `(0.345, 0, 0.548)`. The wrapper is launched with
  `camera_name: zed_front` so its hard-coded base frame name,
  `<camera_name>_camera_link`, is the URDF's `zed_front_camera_link`.

- [ ] **Step 1: Write the tests that pin those frames**

Append to `tests/test_urdf.py`:

```python
def test_the_front_zed_mount_is_where_the_hardware_team_measured_it(robot):
    # The frame the ZED wrapper tracks is <camera_name>_camera_link, the 1/4"
    # mounting screw. These numbers were placed from the measured left
    # optical centres (see the comment block in the URDF); navi_localization's
    # CAMERA_IN_BASE_FOOTPRINT is derived from them, so they are pinned here.
    j = joint(robot, "zed_front_camera_joint")
    assert j.get("type") == "fixed"
    assert j.find("parent").get("link") == "base_link"
    assert j.find("child").get("link") == "zed_front_camera_link"
    assert j.find("origin").get("xyz") == "0.345 0 0.139"
    assert j.find("origin").get("rpy") == "0 0 0"


def test_the_rear_zed_faces_backwards(robot):
    j = joint(robot, "zed_rear_camera_joint")
    assert j.get("type") == "fixed"
    assert j.find("parent").get("link") == "base_link"
    assert j.find("child").get("link") == "zed_rear_camera_link"
    assert j.find("origin").get("xyz") == "-0.345 0 0.139"
    assert j.find("origin").get("rpy") == "0 0 3.141592654"


def test_each_zed_carries_the_wrapper_frame_layout(robot):
    # camera_center sits 15 mm above the mounting screw, the two eyes 60 mm
    # either side of it and 10 mm back - the wrapper's own macro layout.
    for cam in ("zed_front", "zed_rear"):
        assert joint(robot, f"{cam}_camera_center_joint").find("origin").get("xyz") == "0 0 0.015"
        assert joint(robot, f"{cam}_left_camera_joint").find("origin").get("xyz") == "-0.01 0.06 0"
        assert joint(robot, f"{cam}_right_camera_joint").find("origin").get("xyz") == "-0.01 -0.06 0"
```

- [ ] **Step 2: Run them**

Run: `.venv/bin/pytest tests/test_urdf.py -q`
Expected: all pass against the unchanged URDF. If any fails, the URDF has been changed from what the hardware team committed — do not "fix" the URDF; report it.

- [ ] **Step 3: Run the whole laptop suite and the xacro conversion**

Run: `.venv/bin/pytest tests/ -q` and `bash -c 'source /opt/ros/humble/setup.bash && xacro sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro > /dev/null && echo ok'`
Expected: all pass; `ok`.

- [ ] **Step 4: Commit**

```bash
git add tests/test_urdf.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Pin the hardware team's measured ZED frames in the URDF tests, since navi_localization derives its mount offset from them"
```

---

### Task 3: `navi_localization` package with the pose composition module

**Files:**
- Create: `rover/src/navi_localization/package.xml`, `setup.py`, `setup.cfg`, `resource/navi_localization`, `navi_localization/__init__.py`, `navi_localization/pose_composition.py`, `test/test_pose_composition.py`
- Create: `tests/test_mount_offset_agrees_with_urdf.py`

**Interfaces:**
- Produces:
  ```python
  @dataclass(frozen=True)
  class Transform:
      x: float; y: float; z: float
      qx: float; qy: float; qz: float; qw: float   # unit quaternion
  IDENTITY = Transform(0, 0, 0, 0, 0, 0, 1)
  CAMERA_IN_BASE_FOOTPRINT = Transform(0.345, 0.0, 0.548, 0.0, 0.0, 0.0, 1.0)
  MOUNT_OFFSET_VERIFIED = True
  def compose(a: Transform, b: Transform) -> Transform          # a * b
  def inverse(t: Transform) -> Transform
  def footprint_pose_from_camera_pose(camera_in_map: Transform,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT) -> Transform
  def yaw_of(t: Transform) -> float                            # radians
  def translation_distance(a: Transform, b: Transform) -> float
  ```

- [ ] **Step 1: Package skeleton**

`rover/src/navi_localization/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://relaxng.org/ns/structure/1.0"?>
<package format="3">
  <name>navi_localization</name>
  <version>0.1.0</version>
  <description>Rover pose and localisation health from the front ZED 2i's
  visual-inertial tracking. Re-expresses the ZED wrapper's camera pose at
  base_footprint and publishes an OK / SEARCHING / OFF status.</description>
  <maintainer email="oxe.pxs@gmail.com">star</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>zed_msgs</depend>
  <exec_depend>zed_wrapper</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`rover/src/navi_localization/setup.py`:
```python
import os
from glob import glob

from setuptools import setup

package_name = 'navi_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description='Rover pose and localisation health from the front ZED 2i.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_status = navi_localization.localization_status:main',
        ],
    },
)
```

`rover/src/navi_localization/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/navi_localization
[install]
install_scripts=$base/lib/navi_localization
```

`rover/src/navi_localization/resource/navi_localization`: empty file. `navi_localization/__init__.py`: empty. Create empty `launch/` and `config/` directories with a `.gitkeep` each (glob on an absent directory is fine, but the directories are filled in Task 5).

- [ ] **Step 2: Write the failing tests**

`rover/src/navi_localization/test/test_pose_composition.py`:
```python
import math

import pytest

from navi_localization.pose_composition import (
    CAMERA_IN_BASE_FOOTPRINT, IDENTITY, Transform, compose,
    footprint_pose_from_camera_pose, inverse, translation_distance, yaw_of)


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def approx(t: Transform, x, y, z, yaw):
    assert t.x == pytest.approx(x, abs=1e-9)
    assert t.y == pytest.approx(y, abs=1e-9)
    assert t.z == pytest.approx(z, abs=1e-9)
    assert yaw_of(t) == pytest.approx(yaw, abs=1e-9)


def test_compose_with_identity_is_the_same_transform():
    t = Transform(1.0, 2.0, 3.0, *quat_z(0.4))
    assert compose(t, IDENTITY) == pytest.approx(t)
    assert compose(IDENTITY, t) == pytest.approx(t)


def test_compose_applies_the_left_rotation_to_the_right_translation():
    # a = rotate 90 deg about z; b = one metre along x. a*b puts b's origin
    # at (0, 1) in a's parent frame.
    a = Transform(0, 0, 0, *quat_z(math.pi / 2))
    b = Transform(1, 0, 0, 0, 0, 0, 1)
    approx(compose(a, b), 0.0, 1.0, 0.0, math.pi / 2)


def test_inverse_undoes_the_transform():
    t = Transform(1.5, -2.0, 0.7, *quat_z(-1.1))
    approx(compose(t, inverse(t)), 0.0, 0.0, 0.0, 0.0)
    approx(compose(inverse(t), t), 0.0, 0.0, 0.0, 0.0)


def test_footprint_is_behind_and_below_the_camera_when_facing_x():
    # Camera at map origin facing +x: the footprint origin is 0.345 m behind
    # it and 0.548 m below it.
    cam = IDENTITY
    approx(footprint_pose_from_camera_pose(cam), -0.345, 0.0, -0.548, 0.0)


def test_footprint_offset_rotates_with_the_camera():
    # Camera facing +y (yaw 90 deg) at (10, 5, 0.548): "behind" is now -y.
    cam = Transform(10.0, 5.0, 0.548, *quat_z(math.pi / 2))
    approx(footprint_pose_from_camera_pose(cam), 10.0, 5.0 - 0.345, 0.0, math.pi / 2)


def test_a_custom_mount_offset_is_honoured():
    cam = IDENTITY
    offset = Transform(1.0, 0.0, 2.0, 0, 0, 0, 1)
    approx(footprint_pose_from_camera_pose(cam, offset), -1.0, 0.0, -2.0, 0.0)


def test_the_default_mount_offset_is_the_front_camera_box():
    assert CAMERA_IN_BASE_FOOTPRINT == Transform(0.345, 0.0, 0.548, 0.0, 0.0, 0.0, 1.0)


def test_translation_distance_ignores_rotation():
    a = Transform(0, 0, 0, *quat_z(1.0))
    b = Transform(3, 4, 0, *quat_z(-2.0))
    assert translation_distance(a, b) == pytest.approx(5.0)
```

- [ ] **Step 3: Run them to see them fail**

Run (on the laptop; the module is pure Python): `cd rover/src/navi_localization && python3 -m pytest test/test_pose_composition.py -q`
Expected: `ModuleNotFoundError: No module named 'navi_localization.pose_composition'`.

- [ ] **Step 4: Implement `pose_composition.py`**

```python
"""Rigid-transform arithmetic for re-expressing the ZED's pose at the rover.

The ZED ROS 2 wrapper (4.2) tracks a frame it insists on calling
`<camera_name>_camera_link`, and it publishes the TF for it. The rover's
own frame, base_footprint, cannot be hung *above* that in TF without giving
zed_front_camera_link two parents, so instead the pose is re-expressed here in
plain arithmetic and published as its own message. Pure Python on purpose:
no tf2, no numpy, so it is testable on any machine and the mount offset is
one constant in one file.

Quaternions are (x, y, z, w), ROS order.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Transform:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


IDENTITY = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

# Front ZED 2i body centre in base_footprint: the URDF's zed_camera_joint
# (zed_front_camera_joint: 0.345, 0, 0.139 in base_link, the 1/4" mounting screw) plus base_footprint_joint's 0.409 m of
# height. tests/test_mount_offset_agrees_with_urdf.py in the Navi repository
# keeps the two in step.
CAMERA_IN_BASE_FOOTPRINT = Transform(0.345, 0.0, 0.548, 0.0, 0.0, 0.0, 1.0)

# The URDF block these numbers come from says they were placed from the
# measured left optical centres. Set False if a re-measurement disagrees.
MOUNT_OFFSET_VERIFIED = True


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q, v):
    """Rotate vector v by unit quaternion q: q * v * q^-1."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w * t + cross(q.xyz, t)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def compose(a: Transform, b: Transform) -> Transform:
    """a * b: apply b, then a. If a is X-in-W and b is Y-in-X, the result
    is Y-in-W."""
    qa = (a.qx, a.qy, a.qz, a.qw)
    rx, ry, rz = _rotate(qa, (b.x, b.y, b.z))
    qx, qy, qz, qw = _quat_multiply(qa, (b.qx, b.qy, b.qz, b.qw))
    return Transform(a.x + rx, a.y + ry, a.z + rz, qx, qy, qz, qw)


def inverse(t: Transform) -> Transform:
    q_inv = (-t.qx, -t.qy, -t.qz, t.qw)
    x, y, z = _rotate(q_inv, (-t.x, -t.y, -t.z))
    return Transform(x, y, z, *q_inv)


def footprint_pose_from_camera_pose(
        camera_in_map: Transform,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT) -> Transform:
    """T_map_footprint = T_map_camera * inverse(T_footprint_camera)."""
    return compose(camera_in_map, inverse(camera_in_footprint))


def yaw_of(t: Transform) -> float:
    siny_cosp = 2.0 * (t.qw * t.qz + t.qx * t.qy)
    cosy_cosp = 1.0 - 2.0 * (t.qy * t.qy + t.qz * t.qz)
    return math.atan2(siny_cosp, cosy_cosp)


def translation_distance(a: Transform, b: Transform) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
```

- [ ] **Step 5: Run the tests**

Run: `cd rover/src/navi_localization && python3 -m pytest test/test_pose_composition.py -q`
Expected: 8 passed. (`pytest.approx` on a dataclass: if `compose(t, IDENTITY) == pytest.approx(t)` fails to compare, change those two asserts to compare `astuple(...)` — `from dataclasses import astuple` — `astuple(compose(t, IDENTITY)) == pytest.approx(astuple(t))`.)

- [ ] **Step 6: The URDF-agreement test on the laptop**

`tests/test_mount_offset_agrees_with_urdf.py`:
```python
"""The mount offset lives in two places by necessity - the URDF for the
laptop and navi_localization for the Orin - so a test keeps them equal."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / "asterope_iiI.urdf"
MODULE = ROOT / "rover/src/navi_localization/navi_localization/pose_composition.py"


def load_pose_composition():
    spec = importlib.util.spec_from_file_location("pose_composition", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def origin_of(robot, joint_name):
    j = [j for j in robot.findall("joint") if j.get("name") == joint_name][0]
    return [float(v) for v in j.find("origin").get("xyz").split()]


def test_the_python_constant_matches_the_urdf_front_camera_joint():
    robot = ET.parse(URDF).getroot()
    cam = origin_of(robot, "zed_front_camera_joint")
    base = origin_of(robot, "base_footprint_joint")
    pc = load_pose_composition()
    t = pc.CAMERA_IN_BASE_FOOTPRINT
    assert (t.x, t.y, t.z) == pytest.approx((cam[0] + base[0], cam[1] + base[1], cam[2] + base[2]))
    assert (t.qx, t.qy, t.qz, t.qw) == (0.0, 0.0, 0.0, 1.0)
```

Run: `.venv/bin/pytest tests/test_mount_offset_agrees_with_urdf.py -q`
Expected: 1 passed.

- [ ] **Step 7: Build on the Orin**

Run: `./deploy_rover.sh --test`
Expected: `navi_localization` in the colcon summary, and its pytest run passing.

- [ ] **Step 8: Commit**

```bash
git add rover/src/navi_localization tests/test_mount_offset_agrees_with_urdf.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Add navi_localization with the pose re-expression from zed_front_camera_link to base_footprint"
```

---

### Task 4: The localisation tracker (state machine) and the node

**Files:**
- Create: `rover/src/navi_localization/navi_localization/tracker.py`, `test/test_tracker.py`, `navi_localization/localization_status.py`, `test/test_localization_status.py`

**Interfaces:**
- Consumes: `Transform`, `footprint_pose_from_camera_pose`, `translation_distance`, `MOUNT_OFFSET_VERIFIED` from Task 3.
- Produces:
  ```python
  class LocalizationTracker:
      OK = "OK"; SEARCHING = "SEARCHING"; OFF = "OFF"
      def __init__(self, off_after_seconds: float = 2.0): ...
      def on_pose(self, now: float, pose: Transform, stamp: float, tracking_ok: bool) -> None
      def on_tick(self, now: float) -> None
      state: str                              # property
      def seconds_since_ok(self, now: float) -> float
      distance_travelled: float               # property, metres, OK poses only
      def pose_to_publish(self) -> tuple[Transform, float] | None   # (pose, stamp); stamp frozen while not OK
      def status_json(self, now: float) -> str
  ```
  Node `localization_status` (executable `localization_status`): subscribes `/zed_front/zed_node/pose` (`geometry_msgs/PoseStamped`), `/zed_front/zed_node/pose_with_covariance` (`geometry_msgs/PoseWithCovarianceStamped`), `/zed_front/zed_node/pose/status` (`zed_msgs/PosTrackStatus`); publishes `/localization/pose` (`nav_msgs/Odometry`) and `/localization/status` (`std_msgs/String`, JSON).

- [ ] **Step 1: Find out what the ZED status message looks like**

Run on the Orin: `ssh star@a_navi 'source /opt/ros/humble/setup.bash && source ~/workspaces/isaac_ros-dev/install/setup.bash && ros2 interface show zed_msgs/msg/PosTrackStatus'`
Expected: fields `odometry_status` and `spatial_memory_status` (`int8`) with constants such as `OK=0`, `UNAVAILABLE=1`, `SEARCHING=...`. Record the exact names and values — the node maps `odometry_status == OK` to `tracking_ok=True`, anything else to `False`. If the constants differ from `OK`, use what the interface actually shows and name it in the node's comment.

- [ ] **Step 2: Write the failing tracker tests**

`rover/src/navi_localization/test/test_tracker.py`:
```python
import json

import pytest

from navi_localization.pose_composition import IDENTITY, Transform
from navi_localization.tracker import LocalizationTracker


def at(x):
    return Transform(x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_starts_off_with_nothing_to_publish():
    t = LocalizationTracker()
    assert t.state == LocalizationTracker.OFF
    assert t.pose_to_publish() is None
    assert t.distance_travelled == 0.0


def test_a_good_pose_makes_it_ok_and_publishes_that_pose():
    t = LocalizationTracker()
    t.on_pose(now=10.0, pose=at(1.0), stamp=9.99, tracking_ok=True)
    assert t.state == LocalizationTracker.OK
    assert t.pose_to_publish() == (at(1.0), 9.99)
    assert t.seconds_since_ok(now=10.5) == pytest.approx(0.5)


def test_distance_accumulates_over_good_poses_only():
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    t.on_pose(10.1, at(3.0), 10.1, True)
    t.on_pose(10.2, at(7.0), 10.2, True)
    assert t.distance_travelled == pytest.approx(7.0)
    t.on_pose(10.3, at(100.0), 10.3, False)   # searching: a jump is not travel
    assert t.distance_travelled == pytest.approx(7.0)


def test_searching_freezes_the_last_good_pose_and_its_stamp():
    t = LocalizationTracker()
    t.on_pose(10.0, at(2.0), 10.0, True)
    t.on_pose(10.1, at(2.5), 10.1, False)
    assert t.state == LocalizationTracker.SEARCHING
    assert t.pose_to_publish() == (at(2.0), 10.0)
    assert t.seconds_since_ok(now=13.0) == pytest.approx(3.0)


def test_recovery_returns_to_ok_and_continues_from_the_new_pose():
    t = LocalizationTracker()
    t.on_pose(10.0, at(2.0), 10.0, True)
    t.on_pose(10.1, at(9.0), 10.1, False)
    t.on_pose(10.2, at(2.2), 10.2, True)
    assert t.state == LocalizationTracker.OK
    assert t.pose_to_publish() == (at(2.2), 10.2)
    # The 2.0 -> 2.2 hop while OK counts; the searching jump did not.
    assert t.distance_travelled == pytest.approx(0.2)


def test_silence_becomes_off_after_the_timeout_and_keeps_the_last_pose():
    t = LocalizationTracker(off_after_seconds=2.0)
    t.on_pose(10.0, at(1.0), 10.0, True)
    t.on_tick(now=11.9)
    assert t.state == LocalizationTracker.OK
    t.on_tick(now=12.1)
    assert t.state == LocalizationTracker.OFF
    assert t.pose_to_publish() == (at(1.0), 10.0)


def test_searching_also_times_out_to_off():
    t = LocalizationTracker(off_after_seconds=2.0)
    t.on_pose(10.0, at(1.0), 10.0, True)
    t.on_pose(10.5, at(1.0), 10.5, False)
    t.on_tick(now=13.0)
    assert t.state == LocalizationTracker.OFF


def test_status_json_carries_the_fields_the_ground_station_reads():
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    t.on_pose(10.1, at(4.0), 10.1, True)
    data = json.loads(t.status_json(now=10.6))
    assert data == {
        "state": "OK",
        "seconds_since_ok": pytest.approx(0.5),
        "source": "zed_vio",
        "distance_travelled": pytest.approx(4.0),
        "mount_offset_verified": True,
    }


def test_status_json_before_any_pose_reports_off_with_no_age():
    data = json.loads(LocalizationTracker().status_json(now=5.0))
    assert data["state"] == "OFF"
    assert data["seconds_since_ok"] is None
```

- [ ] **Step 3: Run them to see them fail**

Run: `cd rover/src/navi_localization && python3 -m pytest test/test_tracker.py -q`
Expected: `ModuleNotFoundError: No module named 'navi_localization.tracker'`.

- [ ] **Step 4: Implement `tracker.py`**

```python
"""The OK / SEARCHING / OFF state of the localisation, kept apart from ROS.

Three rules, all in the name of never inventing a position:

- While the ZED reports tracking OK, the latest pose is the pose.
- While it reports anything else (SEARCHING), the last good pose is
  republished with its *original* stamp, so any consumer that looks at the
  stamp sees a pose that stopped ageing. Nothing is extrapolated.
- When no pose at all has arrived for `off_after_seconds`, the state is OFF:
  the wrapper is dead or not started. The last good pose is still offered,
  still with its old stamp, for the same reason.

Distance travelled is summed over consecutive OK poses only; a jump during a
search, or the jump when tracking re-acquires, is not driving.
"""

import json

from navi_localization.pose_composition import (
    MOUNT_OFFSET_VERIFIED, Transform, translation_distance)


class LocalizationTracker:
    OK = "OK"
    SEARCHING = "SEARCHING"
    OFF = "OFF"

    def __init__(self, off_after_seconds: float = 2.0) -> None:
        self._off_after = off_after_seconds
        self._state = self.OFF
        self._last_message_at: float | None = None
        self._last_ok_at: float | None = None
        self._good_pose: tuple[Transform, float] | None = None
        self._previous_ok_pose: Transform | None = None
        self._distance = 0.0

    @property
    def state(self) -> str:
        return self._state

    @property
    def distance_travelled(self) -> float:
        return self._distance

    def on_pose(self, now: float, pose: Transform, stamp: float, tracking_ok: bool) -> None:
        self._last_message_at = now
        if tracking_ok:
            if self._state == self.OK and self._previous_ok_pose is not None:
                self._distance += translation_distance(self._previous_ok_pose, pose)
            self._previous_ok_pose = pose
            self._good_pose = (pose, stamp)
            self._last_ok_at = now
            self._state = self.OK
        else:
            self._previous_ok_pose = None
            self._state = self.SEARCHING

    def on_tick(self, now: float) -> None:
        if self._last_message_at is None:
            return
        if now - self._last_message_at > self._off_after:
            self._state = self.OFF
            self._previous_ok_pose = None

    def seconds_since_ok(self, now: float) -> float | None:
        if self._last_ok_at is None:
            return None
        return now - self._last_ok_at

    def pose_to_publish(self) -> tuple[Transform, float] | None:
        return self._good_pose

    def status_json(self, now: float) -> str:
        return json.dumps({
            "state": self._state,
            "seconds_since_ok": self.seconds_since_ok(now),
            "source": "zed_vio",
            "distance_travelled": self._distance,
            "mount_offset_verified": MOUNT_OFFSET_VERIFIED,
        })
```

- [ ] **Step 5: Run the tracker tests**

Run: `cd rover/src/navi_localization && python3 -m pytest test/test_tracker.py -q`
Expected: 9 passed.

- [ ] **Step 6: Write the failing node tests**

These need `rclpy`, so they run on the Orin (or on the laptop under `bash -c 'source /opt/ros/humble/setup.bash && ...'` — `zed_msgs` is **not** installed on the laptop, so the Orin it is). `rover/src/navi_localization/test/test_localization_status.py`:

```python
"""Node-level tests: the node is exercised with messages fed straight into
its callbacks, and its publishers are replaced with recorders. No spinning,
no executor - the ROS plumbing is the wrapper's problem, the mapping from
ZED messages to ours is this node's."""

import json
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from zed_msgs.msg import PosTrackStatus

from navi_localization.localization_status import LocalizationStatus


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = LocalizationStatus()
    n._pose_publisher = Recorder()
    n._status_publisher = Recorder()
    yield n
    n.destroy_node()


def camera_pose(x, y, z, yaw, stamp_sec):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def status(ok: bool):
    msg = PosTrackStatus()
    msg.odometry_status = PosTrackStatus.OK if ok else PosTrackStatus.SEARCHING
    return msg


def test_a_camera_pose_is_republished_at_base_footprint(node):
    node._on_status(status(True))
    node._on_pose(camera_pose(0.345, 0.0, 0.548, 0.0, 100.0))

    assert len(node._pose_publisher.messages) == 1
    odom = node._pose_publisher.messages[0]
    assert odom.header.frame_id == "map"
    assert odom.child_frame_id == "base_footprint"
    assert odom.header.stamp.sec == 100
    assert odom.pose.pose.position.x == pytest.approx(0.0, abs=1e-9)
    assert odom.pose.pose.position.z == pytest.approx(0.0, abs=1e-9)


def test_searching_republishes_the_last_good_pose_with_its_old_stamp(node):
    node._on_status(status(True))
    node._on_pose(camera_pose(1.345, 0.0, 0.548, 0.0, 100.0))
    node._on_status(status(False))
    node._on_pose(camera_pose(5.0, 5.0, 0.548, 0.0, 101.0))

    last = node._pose_publisher.messages[-1]
    assert last.pose.pose.position.x == pytest.approx(1.0, abs=1e-9)
    assert last.header.stamp.sec == 100
    assert json.loads(node._status_publisher.messages[-1].data)["state"] == "SEARCHING"


def test_covariance_is_copied_from_the_wrapper(node):
    cov = PoseWithCovarianceStamped()
    cov.pose.covariance[0] = 0.25
    node._on_covariance(cov)
    node._on_status(status(True))
    node._on_pose(camera_pose(0.0, 0.0, 0.0, 0.0, 1.0))

    assert node._pose_publisher.messages[-1].pose.covariance[0] == pytest.approx(0.25)


def test_status_is_published_on_every_state_change(node):
    node._on_status(status(True))
    node._on_pose(camera_pose(0.0, 0.0, 0.0, 0.0, 1.0))
    node._on_status(status(False))
    node._on_pose(camera_pose(0.0, 0.0, 0.0, 0.0, 1.1))

    states = [json.loads(m.data)["state"] for m in node._status_publisher.messages]
    assert "OK" in states
    assert states[-1] == "SEARCHING"


def test_the_magnetometer_is_never_subscribed(node):
    topics = [s.topic_name for s in node.subscriptions]
    assert not any("mag" in t for t in topics)
    assert "/zed_front/zed_node/pose" in topics
```

Adjust `PosTrackStatus.OK` / `PosTrackStatus.SEARCHING` to the constant names Step 1 found.

- [ ] **Step 7: Implement the node**

`rover/src/navi_localization/navi_localization/localization_status.py`:
```python
"""Publishes the rover's pose and the health of its localisation.

Input is the ZED ROS 2 wrapper's positional tracking; output is the pose of
base_footprint in the map frame plus an OK / SEARCHING / OFF status. See
tracker.py for the rules and pose_composition.py for the frame arithmetic.

Status goes out as JSON in a std_msgs/String, the convention /video_status
set: the ground station reads it over rosbridge and a custom .msg would cost
an ament_cmake package for one message.

The magnetometer is deliberately not subscribed - a project decision, not an
oversight. Heading comes from the visual-inertial tracking only.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from zed_msgs.msg import PosTrackStatus

from navi_localization.pose_composition import (
    CAMERA_IN_BASE_FOOTPRINT, MOUNT_OFFSET_VERIFIED, Transform,
    footprint_pose_from_camera_pose)
from navi_localization.tracker import LocalizationTracker


class LocalizationStatus(Node):

    def __init__(self) -> None:
        super().__init__('localization_status')
        self.declare_parameter('off_after_seconds', 2.0)
        self.declare_parameter('status_interval_seconds', 0.5)

        self._tracker = LocalizationTracker(
            off_after_seconds=float(self.get_parameter('off_after_seconds').value))
        self._tracking_ok = False
        self._covariance = [0.0] * 36
        self._last_published_state = None

        self._pose_publisher = self.create_publisher(Odometry, '/localization/pose', 10)
        self._status_publisher = self.create_publisher(String, '/localization/status', 10)
        self.create_subscription(PoseStamped, '/zed_front/zed_node/pose', self._on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 '/zed_front/zed_node/pose_with_covariance',
                                 self._on_covariance, 10)
        self.create_subscription(PosTrackStatus, '/zed_front/zed_node/pose/status',
                                 self._on_status, 10)
        self.create_timer(float(self.get_parameter('status_interval_seconds').value),
                          self._tick)

        c = CAMERA_IN_BASE_FOOTPRINT
        self.get_logger().info(
            f"camera mount offset in base_footprint: ({c.x}, {c.y}, {c.z}) - "
            + ("verified" if MOUNT_OFFSET_VERIFIED else
               "UNVERIFIED - a re-measurement disagreed with the URDF"))
        self._publish_status(force=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_status(self, msg: PosTrackStatus) -> None:
        # Only odometry_status matters here: spatial_memory_status reports
        # the loop-closure side, which being unavailable does not make the
        # pose wrong.
        self._tracking_ok = (msg.odometry_status == PosTrackStatus.OK)

    def _on_covariance(self, msg: PoseWithCovarianceStamped) -> None:
        self._covariance = list(msg.pose.covariance)

    def _on_pose(self, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        camera_in_map = Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._tracker.on_pose(self._now(), footprint_pose_from_camera_pose(camera_in_map),
                              stamp, self._tracking_ok)
        self._publish_pose()
        self._publish_status()

    def _tick(self) -> None:
        self._tracker.on_tick(self._now())
        self._publish_status(force=True)

    def _publish_pose(self) -> None:
        published = self._tracker.pose_to_publish()
        if published is None:
            return
        pose, stamp = published
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp.sec = int(stamp)
        odom.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = pose.z
        odom.pose.pose.orientation.x = pose.qx
        odom.pose.pose.orientation.y = pose.qy
        odom.pose.pose.orientation.z = pose.qz
        odom.pose.pose.orientation.w = pose.qw
        odom.pose.covariance = self._covariance
        self._pose_publisher.publish(odom)

    def _publish_status(self, force: bool = False) -> None:
        state = self._tracker.state
        if not force and state == self._last_published_state:
            return
        if state != self._last_published_state:
            self.get_logger().info(f"localisation {state}")
        self._last_published_state = state
        msg = String()
        msg.data = self._tracker.status_json(self._now())
        self._status_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 8: Deploy and run all package tests on the Orin**

Run: `./deploy_rover.sh --test`
Expected: `navi_localization` tests all pass (8 + 9 + 5 = 22). If `test_the_magnetometer_is_never_subscribed` fails on `node.subscriptions` not existing in this rclpy, use `[s.topic_name for s in node._subscriptions]`; if that fails too, keep a list of subscribed topic names on the node in `__init__` (`self.subscribed_topics`) and assert on it.

- [ ] **Step 9: Commit**

```bash
git add rover/src/navi_localization
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Add localization_status: the ZED pose at base_footprint plus an OK/SEARCHING/OFF status that never invents a position"
```

---

### Task 5: The wrapper configuration and launch file, with the startup time measured

**Files:**
- Create: `rover/src/navi_localization/config/zed_front.yaml`, `rover/src/navi_localization/launch/localization.launch.py`
- Delete: the two `.gitkeep` files from Task 3

**Interfaces:**
- Produces: `ros2 launch navi_localization localization.launch.py` starts the wrapper (camera `zed`, node `zed_node`) and `localization_status`. Within **20 s** of launch `/localization/status` is being published; `/localization/pose` publishes at the wrapper's grab rate once tracking starts. Task 7's `start_navi.sh` calls this launch file.

- [ ] **Step 1: Measure the baseline startup once, so the improvement is a number**

Run (this takes ~2 minutes):
```bash
ssh star@a_navi 'source /opt/ros/humble/setup.bash && source ~/workspaces/isaac_ros-dev/install/setup.bash
start=$(date +%s)
timeout 240 ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i > /tmp/zed_baseline.log 2>&1 &
for i in $(seq 1 48); do sleep 5; ros2 topic list 2>/dev/null | grep -q "zed_node/odom$" && break; done
echo "baseline: odom advertised after $(( $(date +%s) - start )) s"
pkill -x component_container_isolated; sleep 3'
```
Expected: about 90 s. Write the number down for the launch file docstring.

- [ ] **Step 2: Write the override configuration**

`rover/src/navi_localization/config/zed_front.yaml`:
```yaml
# Overrides applied on top of the wrapper's own common_stereo.yaml + zed2i.yaml
# (zed_camera.launch.py ros_params_override_path). Only what differs from
# the wrapper's defaults is here, so a wrapper upgrade changes as little as
# possible underneath us.
/**:
    ros__parameters:
        general:
            camera_name: 'zed_front'
            grab_resolution: 'HD720'
            grab_frame_rate: 30
            pub_resolution: 'CUSTOM'
            pub_downscale_factor: 2.0     # 640 x 360 images: the video sender is their only consumer
            pub_frame_rate: 15.0
        video:
            # nothing: defaults
        depth:
            depth_mode: 'PERFORMANCE'
            min_depth: 0.3
            max_depth: 10.0
            point_cloud_freq: 5.0         # nobody consumes it in sub-project 1; kept cheap, not off, so sub-project 3 can turn it up
        pos_tracking:
            pos_tracking_enabled: true
            pos_tracking_mode: 'GEN_1'
            imu_fusion: true
            publish_tf: true
            publish_map_tf: true
            area_memory: true             # the SDK's loop closure
            reset_odom_with_loop_closure: true
            set_gravity_as_origin: true
            two_d_mode: false
            floor_alignment: false
        sensors:
            publish_imu_tf: true
            sensors_pub_rate: 100.0
        mapping:
            mapping_enabled: false        # sub-project 3
        object_detection:
            od_enabled: false
        body_tracking:
            bt_enabled: false
        stream_server:
            stream_enabled: false
        # Image transport plugins. Every image topic the wrapper advertises
        # initialises one publisher per plugin, and the ffmpeg plugin's
        # libx264 setup costs seconds each - that is where the 90 s
        # startup went. Raw only.
        rgb.image_rect_color:
            enable_pub_plugins: ['image_transport/raw']
```

The last block is the part to verify. image_transport (Humble) reads, per
image topic, a parameter named `<base topic with '/' replaced by '.'>.enable_pub_plugins`
(confirmed: `strings /opt/ros/humble/lib/libimage_transport.so | grep enable_pub_plugins`
shows `.enable_pub_plugins`). The wrapper advertises ~17 image topics, so there
are two ways in, and Step 3 uses the first unless the check below rules it out:

1. **Parameters in the YAML.** YAML keys may contain dots, so
   `rgb.image_rect_color.enable_pub_plugins: ['image_transport/raw']` under
   `/**: ros__parameters:` reaches the node through the override file. Add
   one such line per topic in `IMAGE_TOPICS` (Step 3) — generate them with
   `python3 -c "print('\n'.join(f\"        {t.replace('/', '.')}.enable_pub_plugins: ['image_transport/raw']\" for t in [...]))"`
   and paste them into `zed_front.yaml` in place of the single `rgb.image_rect_color` block.
2. **If the wrapper builds its publishers before parameters apply** (Step 5's
   measurement still shows ~90 s and `ros2 topic list | grep ffmpeg` is
   non-empty), check whether the component reads its own list:
   `grep -n "enable_pub_plugins\|disable_pub_plugins" ~/workspaces/isaac_ros-dev/src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp`
   on the Orin, and use whatever parameter it names. If it names none, the
   remaining lever is to hide the ffmpeg plugin from this process: launch the
   wrapper with `AMENT_PREFIX_PATH` minus the prefix that provides
   `ffmpeg_image_transport` (`ros2 pkg prefix ffmpeg_image_transport`). If that
   prefix is `/opt/ros/humble` itself this cannot work, and the answer is to
   report the finding rather than to hack around it — stop and say so.

List the topics from the baseline log:
`grep -o "Advertised on topic: /zed_front/zed_node/[a-z_/]*" /tmp/zed_baseline.log | sed 's|.*/zed_node/||' | sort -u`
and compare with `IMAGE_TOPICS` in Step 3; use the log's list, not the plan's, if they differ.

- [ ] **Step 3: Write the launch file**

`rover/src/navi_localization/launch/localization.launch.py`:
```python
"""Front ZED 2i positional tracking plus the localisation status node.

Includes the wrapper's own zed_camera.launch.py so its container, its
URDF publisher and its parameter loading stay the wrapper's business; we
add one override file and one node.

Startup, measured 2026-08-29 on the Orin, time from launch to
/zed_front/zed_node/odom being advertised:
    wrapper defaults: <baseline from Task 5 step 1> s
    this launch file: <after from Task 5 step 5> s
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Every image topic the wrapper advertises, relative to the node. Each gets
# its image_transport plugin list pinned to raw: the default set includes
# ffmpeg, whose libx264 initialisation per topic is what made the wrapper
# take 90 s to reach odometry. Regenerate from a wrapper log with:
#   grep -o "Advertised on topic: /zed_front/zed_node/[a-z_/]*image[a-z_]*" log | sort -u
IMAGE_TOPICS = [
    'rgb/image_rect_color',
    'rgb_gray/image_rect_gray',
    'rgb_raw/image_raw_color',
    'rgb_raw_gray/image_raw_gray',
    'left/image_rect_color',
    'left_gray/image_rect_gray',
    'left_raw/image_raw_color',
    'left_raw_gray/image_raw_gray',
    'right/image_rect_color',
    'right_gray/image_rect_gray',
    'right_raw/image_raw_color',
    'right_raw_gray/image_raw_gray',
    'stereo/image_rect_color',
    'stereo_raw/image_raw_color',
    'depth/depth_registered',
    'confidence/confidence_map',
    'disparity/disparity_image',
]


def raw_only_plugin_parameters() -> dict:
    return {f"{topic.replace('/', '.')}.enable_pub_plugins": ['image_transport/raw']
            for topic in IMAGE_TOPICS}


def generate_launch_description():
    share = get_package_share_directory('navi_localization')
    wrapper_share = get_package_share_directory('zed_wrapper')

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrapper_share, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed_front',
            'ros_params_override_path': os.path.join(share, 'config', 'zed_front.yaml'),
            'publish_tf': 'true',
            'publish_map_tf': 'true',
            'publish_urdf': 'true',
        }.items(),
    )

    status = Node(
        package='navi_localization',
        executable='localization_status',
        name='localization_status',
        output='screen',
    )

    return LaunchDescription([zed, status])
```

`IMAGE_TOPICS` and `raw_only_plugin_parameters()` document the list and generate the YAML lines; the wrapper's included launch does not accept extra node parameters, so the parameters themselves live in `zed_front.yaml` (Step 2, way 1). Keep the two in step: a test is not worth it for a static list, a comment pointing each at the other is.

- [ ] **Step 4: Remove the `.gitkeep` files, deploy**

```bash
git rm -q rover/src/navi_localization/config/.gitkeep rover/src/navi_localization/launch/.gitkeep
./deploy_rover.sh
```

- [ ] **Step 5: Measure the startup with this launch file, and confirm the outputs**

```bash
ssh star@a_navi 'source /opt/ros/humble/setup.bash && source ~/workspaces/isaac_ros-dev/install/setup.bash && source ~/navi/install/local_setup.bash
start=$(date +%s)
timeout 200 ros2 launch navi_localization localization.launch.py > /tmp/loc_launch.log 2>&1 &
for i in $(seq 1 40); do sleep 2; ros2 topic list 2>/dev/null | grep -q "zed_node/odom$" && break; done
echo "with overrides: odom advertised after $(( $(date +%s) - start )) s"
for i in $(seq 1 30); do sleep 2; timeout 3 ros2 topic echo /localization/status --once >/dev/null 2>&1 && break; done
echo "status received after $(( $(date +%s) - start )) s"
echo "== status =="; timeout 5 ros2 topic echo /localization/status --once
echo "== pose hz =="; timeout 15 ros2 topic hz /localization/pose --window 30 2>&1 | grep average | tail -1
echo "== pose frames =="; timeout 5 ros2 topic echo /localization/pose --once | grep -E "frame_id|child_frame_id"
echo "== mag subscribers (must be 0) =="; ros2 topic info /zed_front/zed_node/imu/mag | grep "Subscription count"
echo "== plugin topics (ffmpeg must be absent) =="; ros2 topic list | grep -c ffmpeg || true
pkill -x component_container_isolated; pkill -x localization_status; sleep 3
grep -iE "\[ERROR\]|\[WARN\]" /tmp/loc_launch.log | grep -viE "debug" | head -5'
```
Expected: odom advertised in **≤ 20 s**; status received; status JSON with `"state": "OK"` (the camera is static on the bench, tracking initialises fine while stationary); pose ~30 Hz; `frame_id: map`, `child_frame_id: base_footprint`; mag subscription count 0; ffmpeg topic count 0. If the startup is still ~90 s, the plugin parameter did not apply — go back to Step 2's investigation, and do not proceed with the number unmet: it is the acceptance criterion of this task.

- [ ] **Step 6: Put both numbers in the launch file docstring, run the tests, commit**

Replace the two placeholders in the docstring with the measured values.

```bash
./deploy_rover.sh --test
git add rover/src/navi_localization
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Launch the front ZED with tracking and raw-only image transport, cutting startup from 90 s to under 20 s"
```
(Put the real numbers in the message.)

---

### Task 6: `video_sender` streams from the wrapper's image topic

**Files:**
- Create: `rover/src/navi_teleop/navi_teleop/image_pipe.py`, `rover/src/navi_teleop/test/test_image_pipe.py`
- Modify: `rover/src/navi_teleop/navi_teleop/video_sender.py`, `rover/src/navi_teleop/test/test_video_sender.py`, `rover/src/navi_teleop/package.xml` (add `<depend>sensor_msgs</depend>`)

**Interfaces:**
- Consumes: `VideoRequest`, `parse_request`, `InvalidRequest` from `navi_teleop.video_request` (unchanged).
- Produces:
  ```python
  # image_pipe.py
  def build_pipe_pipeline(host: str, port: int, width: int, height: int,
                          fps: int, bitrate_kbps: int, encoding: str) -> list[str]
  # encoding: the sensor_msgs/Image encoding, "bgra8" (the wrapper's default) or "rgb8" / "bgr8"
  def gst_format_for(encoding: str) -> str          # "bgra8" -> "bgra", "rgb8" -> "rgb", "bgr8" -> "bgr"; ValueError otherwise
  def bytes_per_pixel(encoding: str) -> int         # 4 / 3 / 3
  def frame_matches(msg_width, msg_height, msg_encoding, msg_len, request: VideoRequest) -> str | None
  #   None when the image can be sent for this request; else a one-line reason.
  ```
  `VideoSender` gains parameter `source` (`'zed_topic'` default, or `'v4l2'`) and `image_topic` (`'/zed_front/zed_node/rgb/image_rect_color'`).

- [ ] **Step 1: Look at what the wrapper publishes**

On the Orin with the Task 5 launch running (start it, wait for `/localization/status`, then):
`ros2 topic echo /zed_front/zed_node/rgb/image_rect_color --once --no-arr | grep -E "width|height|encoding|step"`
Expected: `width: 640`, `height: 360`, `encoding: bgra8`, `step: 2560`. Then stop it (`pkill -x component_container_isolated; pkill -x localization_status`). Record the values; the tests below assume them.

- [ ] **Step 2: Write the failing `image_pipe` tests**

`rover/src/navi_teleop/test/test_image_pipe.py`:
```python
import pytest

from navi_teleop.image_pipe import (
    build_pipe_pipeline, bytes_per_pixel, frame_matches, gst_format_for)
from navi_teleop.video_request import VideoRequest

REQUEST = VideoRequest(enable=True, host="192.168.178.101", port=5600,
                       width=640, height=360, fps=15, bitrate_kbps=800,
                       device="/dev/video0")


def test_the_pipeline_reads_whole_frames_from_stdin():
    argv = build_pipe_pipeline("192.168.178.101", 5600, 640, 360, 15, 800, "bgra8")
    assert argv[:2] == ["gst-launch-1.0", "-q"]
    assert "fdsrc" in argv and "fd=0" in argv
    # One blocksize = one frame, 640*360*4 bytes for bgra8.
    assert f"blocksize={640 * 360 * 4}" in argv


def test_the_pipeline_frames_the_byte_stream_with_rawvideoparse():
    # A capsfilter only asserts; rawvideoparse is what cuts a byte stream
    # into frames. Without it the decoder shows solid green.
    argv = build_pipe_pipeline("h", 5600, 640, 360, 15, 800, "bgra8")
    i = argv.index("rawvideoparse")
    assert argv[i + 1:i + 5] == ["width=640", "height=360", "format=bgra", "framerate=15/1"]


def test_the_pipeline_keeps_the_rover_encoder_settings():
    argv = build_pipe_pipeline("10.0.0.5", 5600, 640, 360, 15, 800, "bgra8")
    assert "x264enc" in argv
    assert "tune=zerolatency" in argv
    assert "bitrate=800" in argv
    assert "key-int-max=30" in argv
    assert "config-interval=1" in argv
    assert "host=10.0.0.5" in argv and "port=5600" in argv


@pytest.mark.parametrize("encoding,fmt,bpp", [("bgra8", "bgra", 4), ("rgb8", "rgb", 3), ("bgr8", "bgr", 3)])
def test_encodings_map_to_gstreamer_formats(encoding, fmt, bpp):
    assert gst_format_for(encoding) == fmt
    assert bytes_per_pixel(encoding) == bpp


def test_an_unknown_encoding_is_refused_not_guessed():
    with pytest.raises(ValueError):
        gst_format_for("mono16")


def test_a_matching_frame_is_accepted():
    assert frame_matches(640, 360, "bgra8", 640 * 360 * 4, REQUEST) is None


def test_a_geometry_mismatch_names_both_sizes():
    reason = frame_matches(1280, 720, "bgra8", 1280 * 720 * 4, REQUEST)
    assert "1280x720" in reason and "640x360" in reason


def test_a_short_frame_is_refused():
    reason = frame_matches(640, 360, "bgra8", 10, REQUEST)
    assert "bytes" in reason
```

- [ ] **Step 3: Run them to see them fail**

Run: `cd rover/src/navi_teleop && python3 -m pytest test/test_image_pipe.py -q`
Expected: `ModuleNotFoundError: No module named 'navi_teleop.image_pipe'`.

- [ ] **Step 4: Implement `image_pipe.py`**

```python
"""The stdin-fed half of video_sender: frames arrive as sensor_msgs/Image
from the ZED wrapper and go into gst-launch through a pipe.

This is the front end the simulation's sender uses, carried over because it
was the one verified to frame a byte stream correctly: fdsrc emits
fixed-size chunks with no idea where a frame ends, blocksize makes each
chunk one frame, and rawvideoparse is the element that actually cuts the
stream - a capsfilter only asserts. Pure functions, so the argv and the
frame check are testable without a camera or ROS.
"""

from navi_teleop.video_request import VideoRequest

_FORMATS = {"bgra8": ("bgra", 4), "rgb8": ("rgb", 3), "bgr8": ("bgr", 3)}


def gst_format_for(encoding: str) -> str:
    try:
        return _FORMATS[encoding][0]
    except KeyError:
        raise ValueError(f"unsupported image encoding {encoding!r}; "
                         f"expected one of {sorted(_FORMATS)}") from None


def bytes_per_pixel(encoding: str) -> int:
    gst_format_for(encoding)
    return _FORMATS[encoding][1]


def build_pipe_pipeline(host: str, port: int, width: int, height: int,
                        fps: int, bitrate_kbps: int, encoding: str) -> list[str]:
    frame_bytes = width * height * bytes_per_pixel(encoding)
    return [
        "gst-launch-1.0", "-q",
        "fdsrc", "fd=0", f"blocksize={frame_bytes}",
        "!", "rawvideoparse", f"width={width}", f"height={height}",
        f"format={gst_format_for(encoding)}", f"framerate={fps}/1",
        "!", "videoconvert",
        "!", "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
        f"bitrate={bitrate_kbps}", "key-int-max=30",
        "!", "rtph264pay", "config-interval=1", "pt=96",
        "!", "udpsink", f"host={host}", f"port={port}",
    ]


def frame_matches(msg_width: int, msg_height: int, msg_encoding: str,
                  msg_len: int, request: VideoRequest) -> str | None:
    """None if this image can go into a pipeline built for `request`;
    otherwise the reason it cannot. Rescaling here would hide a
    misconfiguration behind a picture, so a mismatch is refused."""
    if (msg_width, msg_height) != (request.width, request.height):
        return (f"image is {msg_width}x{msg_height} but the request is for "
                f"{request.width}x{request.height}")
    try:
        expected = msg_width * msg_height * bytes_per_pixel(msg_encoding)
    except ValueError as exc:
        return str(exc)
    if msg_len != expected:
        return f"image has {msg_len} bytes, expected {expected} for {msg_encoding}"
    return None
```

- [ ] **Step 5: Run the image_pipe tests**

Run: `cd rover/src/navi_teleop && python3 -m pytest test/test_image_pipe.py -q`
Expected: 10 passed.

- [ ] **Step 6: Write the failing `video_sender` tests for the new source**

Append to `rover/src/navi_teleop/test/test_video_sender.py` (it already imports `rclpy`, `String`, `VideoSender`; the existing tests stay as they are):

```python
from sensor_msgs.msg import Image

from navi_teleop.video_request import DEFAULT_PORT


class FakeProcess:
    def __init__(self):
        self.stdin = self
        self.written = []
        self.terminated = False

    def write(self, data):
        self.written.append(bytes(data))

    def flush(self):
        pass

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        return 0

    def kill(self):
        pass


class FakeLauncher:
    def __init__(self):
        self.calls = []
        self.process = FakeProcess()

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        return self.process


@pytest.fixture(scope="module", autouse=True)
def ros():
    if not rclpy.ok():
        rclpy.init()
    yield


def make_node(source="zed_topic"):
    launcher = FakeLauncher()
    node = VideoSender(launcher=launcher, parameter_overrides=[
        rclpy.parameter.Parameter("source", value=source)])
    return node, launcher


def request(width=640, height=360):
    msg = String()
    msg.data = json.dumps({"enable": True, "host": "192.168.178.101", "port": DEFAULT_PORT,
                           "width": width, "height": height, "fps": 15, "bitrate_kbps": 800})
    return msg


def image(width=640, height=360, encoding="bgra8"):
    msg = Image()
    msg.width, msg.height, msg.encoding = width, height, encoding
    msg.step = width * 4
    msg.data = bytes(width * height * 4)
    return msg


def test_zed_topic_source_starts_a_stdin_pipeline_on_the_first_frame():
    node, launcher = make_node()
    try:
        node._on_request(request())
        assert launcher.calls == []                # nothing to encode yet
        assert node._state == "starting"
        node._on_image(image())
        argv, kwargs = launcher.calls[0]
        assert "fdsrc" in argv and "format=bgra" in argv
        assert kwargs["stdin"] is not None
        assert node._state == "streaming"
        assert launcher.process.written == [bytes(640 * 360 * 4)]
    finally:
        node.destroy_node()


def test_zed_topic_source_refuses_a_frame_of_the_wrong_size_with_the_reason():
    node, launcher = make_node()
    try:
        node._on_request(request(width=1280, height=720))
        node._on_image(image(640, 360))
        assert launcher.calls == []
        assert node._state == "failed"
        assert "640x360" in node._detail and "1280x720" in node._detail
    finally:
        node.destroy_node()


def test_zed_topic_source_ignores_frames_when_not_streaming():
    node, launcher = make_node()
    try:
        node._on_image(image())
        assert launcher.calls == []
        assert node._state == "stopped"
    finally:
        node.destroy_node()


def test_v4l2_source_keeps_the_old_capture_pipeline():
    node, launcher = make_node(source="v4l2")
    try:
        node._on_request(request(width=1344, height=376))
        argv, _ = launcher.calls[0]
        assert "v4l2src" in argv
        assert node._state == "streaming"
    finally:
        node.destroy_node()


def test_stopping_terminates_the_stdin_pipeline():
    node, launcher = make_node()
    try:
        node._on_request(request())
        node._on_image(image())
        stop = String()
        stop.data = json.dumps({"enable": False})
        node._on_request(stop)
        assert launcher.process.terminated
        assert node._state == "stopped"
    finally:
        node.destroy_node()
```

Check the top of the existing test file: if it already has a module-scoped `rclpy.init()` fixture, do not add a second one — reuse it.

- [ ] **Step 7: Run to see them fail**

Run: `./deploy_rover.sh --test`
Expected: the five new tests fail (`unexpected keyword argument 'parameter_overrides'` or `'source'` unknown / `_on_image` missing). The existing ones still pass.

- [ ] **Step 8: Modify `video_sender.py`**

Changes, in order:

1. Imports: add `from sensor_msgs.msg import Image` and `from navi_teleop.image_pipe import build_pipe_pipeline, frame_matches`.
2. Docstring: replace the paragraph beginning "The camera is read as a plain UVC device" with:
   ```
   Two sources. `zed_topic` (the default) takes frames from the ZED ROS 2
   wrapper's rectified RGB topic and pipes them into the encoder through
   stdin: the wrapper owns the camera for localisation, and the ZED SDK
   opens it exclusively, so nothing else can. `v4l2` is the old path -
   the camera as a plain UVC device - for a run without localisation
   (start_navi.sh --no-localization), when the wrapper is not there.
   ```
3. `__init__(self, launcher=subprocess.Popen, **node_kwargs)` → `super().__init__('video_sender', **node_kwargs)`. Declare `source` (`'zed_topic'`) and `image_topic` (`'/zed_front/zed_node/rgb/image_rect_color'`). Add `self._pending: VideoRequest | None = None` (a request waiting for its first frame) and `self._frame_bytes = 0`. If `source == 'zed_topic'`, `self.create_subscription(Image, image_topic, self._on_image, 1)` — depth 1: a late frame is worthless.
4. `_on_request`: unchanged except the enable branch becomes:
   ```python
        self._stop_stream()
        if not request.enable:
            self._set_state('stopped', '')
            return
        if self._source() == 'v4l2':
            self._start_stream(request)
        else:
            # The pipeline is built from the first frame, whose encoding
            # decides the raw format - so until one arrives there is
            # nothing to start.
            self._pending = request
            self._set_state('starting', f"waiting for {self._image_topic()} "
                                        f"({request.width}x{request.height})")
   ```
   with small helpers `_source()` and `_image_topic()` reading the parameters.
5. New `_on_image`:
   ```python
    def _on_image(self, msg: Image) -> None:
        if self._pending is not None:
            request, self._pending = self._pending, None
            reason = frame_matches(msg.width, msg.height, msg.encoding, len(msg.data), request)
            if reason is not None:
                self.get_logger().warn(f"refusing video request: {reason}")
                self._set_state('failed', reason)
                return
            self._start_pipe_stream(request, msg.encoding)
        if self._state != 'streaming' or self._process is None:
            return
        if len(msg.data) != self._frame_bytes:
            # A torn frame would desynchronise every frame after it.
            self.get_logger().warn(f"dropping frame: {len(msg.data)} bytes, expected {self._frame_bytes}")
            return
        try:
            self._process.stdin.write(bytes(msg.data))
        except (BrokenPipeError, OSError):
            detail = self._stderr_tail()
            self._stop_stream()
            self._set_state('failed', detail if detail else "encoder exited")
   ```
6. New `_start_pipe_stream(self, request, encoding)`: same shape as `_start_stream` (gst-launch check, stderr temp file, `_set_state('starting', ...)`), but `argv = build_pipe_pipeline(request.host, request.port, request.width, request.height, request.fps, request.bitrate_kbps, encoding)` and the launch is `self._launcher(argv, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=stderr_file)`; set `self._frame_bytes = len-of-one-frame` (`request.width * request.height * bytes_per_pixel(encoding)` — import `bytes_per_pixel` too), then `_set_state('streaming', f"{request.host}:{request.port}")`.
7. `_stop_stream`: also `self._pending = None`, and before `terminate()`, close stdin if present:
   ```python
        if getattr(self._process, 'stdin', None) is not None:
            try:
                self._process.stdin.close()
            except OSError:
                pass
   ```
   (With the `FakeProcess` in the tests, `stdin` is the fake itself and has no `close` — add a no-op `close()` to `FakeProcess`.)
8. `package.xml`: add `<depend>sensor_msgs</depend>`.

Keep `_publish_status_tick`'s death detection as it is: it works for both sources.

- [ ] **Step 9: Run all navi_teleop tests on the Orin**

Run: `./deploy_rover.sh --test`
Expected: all pass, including the six original `build_pipeline` tests untouched.

- [ ] **Step 10: Prove it end to end with a real stream to this laptop**

Terminal A (Orin, in one ssh): start the localisation launch and `video_sender`:
```bash
ssh star@a_navi 'source /opt/ros/humble/setup.bash && source ~/workspaces/isaac_ros-dev/install/setup.bash && source ~/navi/install/local_setup.bash
timeout 120 ros2 launch navi_localization localization.launch.py > /tmp/loc.log 2>&1 &
timeout 120 ros2 run navi_teleop video_sender > /tmp/vs.log 2>&1 &
for i in $(seq 1 30); do sleep 2; timeout 3 ros2 topic echo /localization/status --once >/dev/null 2>&1 && break; done
LAPTOP=$(echo $SSH_CONNECTION | cut -d" " -f1)
ros2 topic pub --once /video_request std_msgs/String "{data: \"{\\\"enable\\\": true, \\\"host\\\": \\\"$LAPTOP\\\", \\\"port\\\": 5600, \\\"width\\\": 640, \\\"height\\\": 360, \\\"fps\\\": 15, \\\"bitrate_kbps\\\": 800}\"}"
sleep 20
timeout 3 ros2 topic echo /video_status --once
pkill -x component_container_isolated; pkill -x localization_status; pkill -x video_sender; sleep 2' &
```
Terminal B (laptop), during those 20 s, count decoded frames with the project's receive pipeline:
```bash
timeout 15 gst-launch-1.0 -q udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtpjitterbuffer latency=50 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB ! fdsink fd=1 | wc -c
```
Expected: `/video_status` says `streaming`, and the byte count divided by `640*360*3 = 691200` is on the order of 100+ frames (15 fps × ~10 s of overlap). Zero means the packets never arrived — check the laptop's firewall and the host address the Orin derived. Also check the picture is not green: replace `wc -c` with `head -c 691200 | python3 -c "import sys; d=sys.stdin.buffer.read(); print(sum(d[0::3])/len(d[0::3]), sum(d[1::3])/len(d[1::3]), sum(d[2::3])/len(d[2::3]))"` after a 3 s `sleep` so it is not the first frame — three similar channel means, not `0 131 0`.

- [ ] **Step 11: Commit**

```bash
git add rover/src/navi_teleop
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "video_sender streams from the ZED wrapper's image topic, since the SDK now owns the camera; v4l2 stays for runs without localisation"
```

---

### Task 7: `start_navi.sh` launches localisation, cleans up after it, and waits for it

**Files:**
- Modify: `rover/start_navi.sh`

**Interfaces:**
- Consumes: `ros2 launch navi_localization localization.launch.py` (Task 5); `video_sender`'s `source` parameter (Task 6).
- Produces: `./start_navi.sh [--no-localization]`. With localisation, the script does not report ready until `/localization/status` has been received; without it, `video_sender` runs with `source:=v4l2`.

- [ ] **Step 1: Header and flag**

In the header comment add the line `#   ./start_navi.sh --no-localization  no ZED tracking; video from the camera as a UVC device` after `--no-video`, and in the numbered list add `4. localization.launch.py - the ZED 2i wrapper with positional tracking, plus localization_status publishing /localization/pose and /localization/status`. Add `START_LOCALIZATION=1` next to `START_VIDEO=1` and `--no-localization) START_LOCALIZATION=0; shift ;;` in the `case`.

- [ ] **Step 2: Cleanup**

After the `kill_stale "video pipelines" ...` line add:
```bash
    # The wrapper runs in a composable-node container; a killed run leaves
    # it holding the camera, and the next wrapper then fails to open it.
    kill_stale "ZED wrapper containers" "component_container_isolated.*zed"
    kill_stale "localisation launches" "ros2 launch navi_localization"
    kill_stale "localization_status nodes" "navi_localization/localization_status"
    # The stdin-fed encode pipeline video_sender's zed_topic source spawns.
    kill_stale "video pipe pipelines" "gst-launch-1\.0.*fdsrc.*udpsink"
```
(`kill_stale` already skips this script's own ancestry, so an ssh command line containing these words is safe.)

- [ ] **Step 3: Source the wrapper workspace**

After `source "$WS_DIR/install/local_setup.bash"` add:
```bash
ZED_WS="${ZED_WS:-$HOME/workspaces/isaac_ros-dev}"
if [ "$START_LOCALIZATION" -eq 1 ]; then
    if [ ! -f "$ZED_WS/install/setup.bash" ]; then
        echo "error: ZED wrapper workspace not found at $ZED_WS" >&2
        echo "       set ZED_WS, or run with --no-localization" >&2
        exit 1
    fi
    source "$ZED_WS/install/setup.bash"
    # After, not before, the wrapper's setup: that setup resets the overlay
    # and this workspace would otherwise fall out of the path.
    source "$WS_DIR/install/local_setup.bash"
fi
```

- [ ] **Step 4: Launch and readiness**

Before the `if [ "$START_VIDEO" -eq 1 ]` block add:
```bash
if [ "$START_LOCALIZATION" -eq 1 ]; then
    echo "starting localisation (ZED 2i tracking)"
    ros2 launch navi_localization localization.launch.py &
    LOC_PID=$!
    BACKGROUND_PIDS+=("$LOC_PID")

    # Ready means a status message has actually been received, not that
    # the launch process exists: the wrapper spends its first seconds
    # opening the camera and advertising, and a camera it cannot open
    # leaves the launch alive with nothing behind it.
    LOC_UP=0
    for _ in $(seq 1 30); do
        if ! kill -0 "$LOC_PID" 2>/dev/null; then
            echo "error: localisation launch exited during startup" >&2
            exit 1
        fi
        if timeout 3 ros2 topic echo /localization/status --once >/dev/null 2>&1; then
            LOC_UP=1
            break
        fi
    done
    if [ "$LOC_UP" -ne 1 ]; then
        echo "error: /localization/status never arrived within 90 s" >&2
        echo "       is the ZED 2i plugged in? see the wrapper output above" >&2
        exit 1
    fi
    echo "localisation is publishing /localization/status"
fi
```

- [ ] **Step 5: Video source follows the flag**

In the `START_VIDEO` block, the `/dev/video0` check only makes sense for the v4l2 source — wrap it in `if [ "$START_LOCALIZATION" -eq 0 ]`. Where `video_sender` is started (`ros2 run navi_teleop video_sender ...`), append `--ros-args -p source:=$( [ "$START_LOCALIZATION" -eq 1 ] && echo zed_topic || echo v4l2 )`. Read the existing line first: if it already passes `--ros-args`, add the `-p` to it rather than a second `--ros-args`.

- [ ] **Step 6: Deploy and run the whole rover side, then clean up**

```bash
./deploy_rover.sh
ssh star@a_navi 'cd ~/navi && timeout 150 ./start_navi.sh > /tmp/start_navi.log 2>&1; echo "exit $?"; grep -E "^(starting|rosbridge|localisation|error|warning|cleaning)" /tmp/start_navi.log'
```
Expected (after ~2.5 minutes, when `timeout` ends it): the greps show `starting rosbridge_server`, `rosbridge_server is serving port 9090`, `starting localisation`, `localisation is publishing /localization/status`, no `error:` lines. Then run it **again** the same way: the second run must show `cleaning up stale ZED wrapper containers: <pid>` (or nothing stale, if `timeout`'s SIGTERM let the trap clean everything — either is correct; what must not happen is an error about the camera being in use). Finally `ssh star@a_navi 'pgrep -x component_container_isolated || echo none left'` → `none left`.

- [ ] **Step 7: The `--no-localization` path still works**

```bash
ssh star@a_navi 'cd ~/navi && timeout 40 ./start_navi.sh --no-localization > /tmp/start_navi2.log 2>&1; grep -E "^(starting|rosbridge|error|warning)" /tmp/start_navi2.log'
```
Expected: rosbridge lines, no localisation lines, no errors.

- [ ] **Step 8: Commit**

```bash
git add rover/start_navi.sh
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "start_navi.sh brings up ZED localisation, cleans its container up, and waits for /localization/status before reporting ready"
```

---

## Rover-day checklist (not tasks — recorded for the operator)

- Drive a closed loop of ~20 m; `ros2 topic echo /localization/pose` at start and at the end back on the start mark; report the closure error in metres and degrees.
- If the mounting-screw position is ever re-measured and differs from `(0.345, 0, 0.139)`, change `zed_front_camera_joint` in the URDF and `CAMERA_IN_BASE_FOOTPRINT` in `pose_composition.py` together — `tests/test_mount_offset_agrees_with_urdf.py` enforces that.
- Watch `/localization/status` while driving over the sandiest part of the yard; note how often it goes `SEARCHING` and for how long.
- Start the rover *before* any simulation on ROS domain 0. Measured during implementation: the ZED wrapper's start-up grows from ~5 s on a quiet domain to ~35 s with a sim running, because DDS discovers the simulation's participants first. Nothing fails - it just looks like the rover has hung, and `start_navi.sh` sits in its readiness wait for half a minute.
