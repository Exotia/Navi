# Rover Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Gazebo simulation of the Asterope rover, its wheels driven by the bema controller's own inverse kinematics, shown in the ground station in place of the camera when the operator switches to semi-autonomous mode.

**Architecture:** Everything runs on the laptop. `navi_sim_ik` vendors the Simulink-generated kinematics model unchanged and subscribes to `/manual_twist` directly over DDS from the rover's ROS graph — the ground station needs no new connection. The IK's outputs drive the URDF's eight new joints through stock `gazebo_ros` plugins, and the body pose is integrated from the IK's own constrained velocity rather than produced by physics. A chase camera in Gazebo is encoded to H.264/RTP on port 5601, which the ground station's existing `VideoReceiver` decodes into the existing `VideoPanel`.

**Tech Stack:** ROS 2 Humble, Gazebo Classic 11, `gazebo_ros` plugins (`planar_move`, `joint_pose_trajectory`, `camera`), xacro, ament_cmake + ament_cmake_gtest, GStreamer, PySide6.

**Spec:** `docs/superpowers/specs/2026-08-28-rover-simulation-design.md`

## Global Constraints

- The ground station never imports `rclpy`. Its venv is built with `include-system-site-packages = false`. Nothing under `ground_station/` may gain a ROS dependency.
- The simulation is kinematic. No `ros2_control`, no contact physics, no friction, no inertia tuning. This is a decision, not a stage.
- The vendored Simulink sources are never edited. They are regenerated upstream.
- `TS = 0.06` seconds, matching the rover.
- Rover video uses UDP 5600; simulation video uses UDP 5601. Never the same port.
- Wheel index to physical corner mapping is unverified. It lives in exactly one named constant and is logged at startup.
- All new ROS packages live under `sim/src/`. `build/`, `install/` and `log/` are already gitignored.
- Commits use `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de"`, as this repo has no git identity configured.

## File Structure

| File | Responsibility |
|---|---|
| `asterope_iiI.urdf` | The real robot. Gains steering and wheel joints; stays free of anything fictional. |
| `tests/test_urdf.py` | Asserts the URDF's structure with pure XML parsing, so it runs in the existing venv with no ROS. |
| `sim/src/navi_sim_ik/vendor/` | The Simulink sources, copied verbatim, with provenance recorded. |
| `sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp` | Twist in, joint targets and pose out. No ROS, so it is unit-testable. |
| `sim/src/navi_sim_ik/src/sim_ik_node.cpp` | Thin ROS wrapper: subscriptions, publishers, a 60 ms timer. |
| `sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro` | Includes the real URDF and adds the chase camera and Gazebo plugins. Keeps fiction out of the robot. |
| `sim/src/navi_sim_bringup/worlds/site.world` | The scanned map as visual-only scenery. |
| `sim/src/navi_sim_video/navi_sim_video/sender.py` | Chase camera frames to H.264/RTP on 5601. |
| `ground_station/ui/video_panel.py` | Gains a source label and the dead reckoning marker. |
| `ground_station/ui/main_window.py` | Owns the mode switch: which port to receive on, and stopping rover video. |
| `start_sim.sh` | Brings the simulation up, with the same stale-process cleanup the other launchers have. |

---

### Task 1: Articulate the wheels in the URDF

The four wheels are currently on `fixed` joints. Each corner becomes a steering link rotating about Z, carrying the wheel rotating about Y.

**Files:**
- Modify: `asterope_iiI.urdf`
- Test: `tests/test_urdf.py` (create)

**Interfaces:**
- Consumes: nothing.
- Produces: joint names `steer_<corner>_joint` (revolute, axis `0 0 1`) and `wheel_<corner>_joint` (continuous, axis `0 1 0`), for `corner` in `front_left, front_right, rear_left, rear_right`. Link names `steer_<corner>` and `wheel_<corner>`. Every later task refers to these exact names.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_urdf.py
"""Structural checks on the robot description.

Pure XML parsing on purpose: the ground station's venv has no ROS, and a
test that needs one would not run in this suite.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

URDF = Path(__file__).resolve().parent.parent / "asterope_iiI.urdf"
CORNERS = ["front_left", "front_right", "rear_left", "rear_right"]


@pytest.fixture(scope="module")
def robot():
    return ET.parse(URDF).getroot()


def joint(robot, name):
    found = [j for j in robot.findall("joint") if j.get("name") == name]
    assert len(found) == 1, f"expected exactly one joint {name}, got {len(found)}"
    return found[0]


def test_the_urdf_parses(robot):
    assert robot.get("name") == "asterope"


@pytest.mark.parametrize("corner", CORNERS)
def test_each_corner_steers_about_z(robot, corner):
    j = joint(robot, f"steer_{corner}_joint")
    assert j.get("type") == "revolute"
    assert j.find("axis").get("xyz") == "0 0 1"
    assert j.find("parent").get("link") == "base_link"
    assert j.find("child").get("link") == f"steer_{corner}"


@pytest.mark.parametrize("corner", CORNERS)
def test_each_wheel_rolls_about_y_under_its_steering_link(robot, corner):
    j = joint(robot, f"wheel_{corner}_joint")
    assert j.get("type") == "continuous"
    assert j.find("axis").get("xyz") == "0 1 0"
    # Under the steering link, not base_link: steering has to carry the
    # wheel around with it, or the rover steers without the wheel turning.
    assert j.find("parent").get("link") == f"steer_{corner}"
    assert j.find("child").get("link") == f"wheel_{corner}"


@pytest.mark.parametrize("corner", CORNERS)
def test_steering_is_limited_to_one_turn_either_way(robot, corner):
    # Revolute needs limits. The cabling on a steer module does not permit
    # unbounded rotation, so this is not merely a formality.
    limit = joint(robot, f"steer_{corner}_joint").find("limit")
    assert limit is not None
    assert float(limit.get("lower")) == pytest.approx(-3.14159265, abs=1e-5)
    assert float(limit.get("upper")) == pytest.approx(3.14159265, abs=1e-5)
    assert float(limit.get("effort")) > 0
    assert float(limit.get("velocity")) > 0


def test_the_wheels_keep_their_measured_geometry(robot):
    # 250 mm diameter, 200 mm tread. Regression guard: these came from the
    # hardware, not from the mock geometry they replaced.
    for corner in CORNERS:
        link = [l for l in robot.findall("link") if l.get("name") == f"wheel_{corner}"][0]
        cylinder = link.find("visual/geometry/cylinder")
        assert float(cylinder.get("radius")) == pytest.approx(0.125)
        assert float(cylinder.get("length")) == pytest.approx(0.200)


def test_the_wheels_sit_at_the_910mm_square(robot):
    seen = set()
    for corner in CORNERS:
        origin = joint(robot, f"steer_{corner}_joint").find("origin")
        x, y, z = (float(v) for v in origin.get("xyz").split())
        assert abs(x) == pytest.approx(0.455)
        assert abs(y) == pytest.approx(0.455)
        assert z == pytest.approx(-0.284)
        seen.add((x > 0, y > 0))
    assert len(seen) == 4, "the four corners must be four distinct corners"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv/bin/python -m pytest tests/test_urdf.py -v`
Expected: FAIL — `expected exactly one joint steer_front_left_joint, got 0`

- [ ] **Step 3: Restructure the four corners**

Replace each existing `wheel_<corner>` link and its fixed joint with a steering link and a wheel link. The steering joint carries the corner offset; the wheel joint sits at the steering link's origin.

```xml
    <link name="steer_front_left">
        <visual>
            <origin xyz="0 0 0.08" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.06" length="0.16"/>
            </geometry>
            <material name="silver"/>
        </visual>
    </link>

    <joint name="steer_front_left_joint" type="revolute">
        <parent link="base_link"/>
        <child link="steer_front_left"/>
        <origin xyz="0.455 0.455 -0.284" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <!-- One turn either way: the steer module is cabled, not a slip ring. -->
        <limit lower="-3.14159265" upper="3.14159265" effort="100" velocity="6.0"/>
    </joint>

    <link name="wheel_front_left">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707963 0 0"/>
            <geometry>
                <cylinder radius="0.125" length="0.200"/>
            </geometry>
            <material name="black"/>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="1.5707963 0 0"/>
            <geometry>
                <cylinder radius="0.125" length="0.200"/>
            </geometry>
        </collision>
    </link>

    <joint name="wheel_front_left_joint" type="continuous">
        <parent link="steer_front_left"/>
        <child link="wheel_front_left"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>
```

Repeat for `front_right` at `0.455 -0.455 -0.284`, `rear_left` at `-0.455 0.455 -0.284`, `rear_right` at `-0.455 -0.455 -0.284`. Write each out in full — do not factor them, this is plain URDF with no macros.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m pytest tests/test_urdf.py -v`
Expected: all PASS

- [ ] **Step 5: Look at it**

```bash
bash -c 'source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher /home/ole/star/Navi/asterope_iiI.urdf &
sleep 2
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
rviz2'
```
Drag the joint sliders. Each steering slider must swing its wheel about the vertical; each wheel slider must spin it about the axle. If a wheel orbits the rover instead of turning in place, the joint origins are wrong.

- [ ] **Step 6: Commit**

```bash
git add asterope_iiI.urdf tests/test_urdf.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Give each wheel a steering joint and a rolling joint

The four wheels were on fixed joints, so nothing could steer or roll.
Each corner is now a steering link revolute about Z carrying a wheel
continuous about Y. Steering is limited to one turn either way because
the steer module is cabled rather than on a slip ring.

Tested by parsing the URDF rather than through ROS, so the checks run in
the ground station's venv, which has no ROS by design."
```

---

### Task 2: Vendor the IK and prove it steps

Copy the Simulink model into the package and get it building and stepping, with no ROS involved yet.

**Files:**
- Create: `sim/src/navi_sim_ik/package.xml`, `sim/src/navi_sim_ik/CMakeLists.txt`
- Create: `sim/src/navi_sim_ik/vendor/` (copied from `bemacontroller/src/`)
- Create: `sim/src/navi_sim_ik/vendor/VENDOR.md`
- Test: `sim/src/navi_sim_ik/test/test_vendored_model.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: a CMake target `navi_sim_ik_model` (static library) exposing `kinematics` and `IkController` from the vendored headers.

- [ ] **Step 1: Copy the sources and record where they came from**

```bash
mkdir -p sim/src/navi_sim_ik/vendor/ert_rtw
cp bemacontroller/src/ert_rtw/*.h bemacontroller/src/ert_rtw/*.cpp sim/src/navi_sim_ik/vendor/ert_rtw/
rm -f sim/src/navi_sim_ik/vendor/ert_rtw/ert_main.cpp   # a standalone main we do not want
cp bemacontroller/src/IkController.h bemacontroller/src/IkController.cpp sim/src/navi_sim_ik/vendor/
```

```markdown
<!-- sim/src/navi_sim_ik/vendor/VENDOR.md -->
# Vendored kinematics

Copied verbatim from the bema controller. Do not edit these files: they are
Simulink-generated and are replaced by regenerating the model upstream.

- Origin: ssh://git@gitlab.star-dresden.de:11022/star-projekte/merope/bema/bemacontroller.git
- Commit: bd216fa
- Copied: 2026-08-28
- Paths: `src/ert_rtw/*` (less `ert_main.cpp`), `src/IkController.{h,cpp}`

Copied rather than referenced across the working tree because
`bemacontroller/` is an untracked nested repository — a clone of this project
would not have it, and the build must not depend on a directory that may not
exist.

The reason for vendoring rather than reimplementing: the simulation must run
the same arithmetic as the rover, so that a disagreement between them is a
real disagreement and not a porting artifact.
```

- [ ] **Step 2: Write the failing test**

```cpp
// sim/src/navi_sim_ik/test/test_vendored_model.cpp
#include <gtest/gtest.h>

#include <cmath>

#include "kinematics.h"

// Proves the vendored model builds, initialises and steps. Not a test of the
// control law, which belongs to whoever regenerates it - this is the guard
// that says the copy is complete and usable.
TEST(VendoredModel, DrivingStraightAheadPointsEveryWheelForward)
{
  kinematics model;
  model.initialize();

  kinematics::ExternalInputs in{};
  in.TS = 0.06;
  in.VX_out = 0.5;   // straight ahead
  in.VY_out = 0.0;
  in.U_p = 0.0;      // no yaw

  for (int i = 0; i < 100; ++i) {
    model.setExternalInputs(&in);
    model.step();
    const auto & out = model.getExternalOutputs();
    for (int w = 0; w < 4; ++w) {
      in.beta_hat[w] = out.beta_next[w];
      in.beta_dot_hat[w] = out.Beta_dot[w];
    }
  }

  const auto & out = model.getExternalOutputs();
  for (int w = 0; w < 4; ++w) {
    EXPECT_NEAR(out.beta_next[w], 0.0, 0.05) << "wheel " << w << " is not straight";
    EXPECT_GT(out.omega[w], 0.0) << "wheel " << w << " is not driving forward";
  }
}

TEST(VendoredModel, TurningInPlaceSpreadsTheWheelAngles)
{
  kinematics model;
  model.initialize();

  kinematics::ExternalInputs in{};
  in.TS = 0.06;
  in.VX_out = 0.0;
  in.VY_out = 0.0;
  in.U_p = 0.4;      // yaw only

  for (int i = 0; i < 200; ++i) {
    model.setExternalInputs(&in);
    model.step();
    const auto & out = model.getExternalOutputs();
    for (int w = 0; w < 4; ++w) {
      in.beta_hat[w] = out.beta_next[w];
      in.beta_dot_hat[w] = out.Beta_dot[w];
    }
  }

  const auto & out = model.getExternalOutputs();
  double lo = out.beta_next[0], hi = out.beta_next[0];
  for (int w = 1; w < 4; ++w) {
    lo = std::min(lo, out.beta_next[w]);
    hi = std::max(hi, out.beta_next[w]);
  }
  // Spinning about a point puts the four wheels on tangents to a circle, so
  // they cannot all point the same way.
  EXPECT_GT(hi - lo, 0.2) << "all four wheels point the same way while turning in place";
}
```

- [ ] **Step 3: Write package.xml and CMakeLists.txt**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>navi_sim_ik</name>
  <version>0.1.0</version>
  <description>The rover's own inverse kinematics, driving the simulation.</description>
  <maintainer email="ole.peters@star-dresden.de">Ole Peters</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>trajectory_msgs</depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```cmake
cmake_minimum_required(VERSION 3.8)
project(navi_sim_ik)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Simulink-generated code, compiled as found. Its warnings are not ours to
# fix and would drown anything we do need to see.
file(GLOB VENDOR_SOURCES vendor/ert_rtw/*.cpp vendor/IkController.cpp)
add_library(navi_sim_ik_model STATIC ${VENDOR_SOURCES})
target_include_directories(navi_sim_ik_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/ert_rtw>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(navi_sim_ik_model PRIVATE -w)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vendored_model test/test_vendored_model.cpp)
  target_link_libraries(test_vendored_model navi_sim_ik_model)
endif()

ament_package()
```

- [ ] **Step 4: Build and run the test, expecting it to pass**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
```
Expected: both tests PASS. If `TurningInPlaceSpreadsTheWheelAngles` fails, do not adjust the test to fit — the vendored copy is incomplete or `U_p` is not the yaw rate, and both are worth stopping for.

- [ ] **Step 5: Commit**

```bash
git add sim/src/navi_sim_ik
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Vendor the rover's kinematics model into the simulation

Copied verbatim from bemacontroller bd216fa rather than referenced in
place: that directory is an untracked nested repository, so a clone of
this project would not have it and the build would depend on a path that
may not exist. Copied rather than reimplemented so the simulation runs
the rover's arithmetic - a disagreement between them should be a real
disagreement, not a porting artifact.

Two tests, both about the copy being usable rather than about the
control law: driving straight points every wheel forward, and turning in
place does not."
```

---

### Task 3: The stepper — twist in, joint targets and pose out

All the logic, none of the ROS, so it can be tested.

**Files:**
- Create: `sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp`, `sim/src/navi_sim_ik/src/sim_ik_stepper.cpp`
- Modify: `sim/src/navi_sim_ik/CMakeLists.txt`
- Test: `sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp`

**Interfaces:**
- Consumes: `navi_sim_ik_model` from Task 2.
- Produces:
```cpp
namespace navi_sim_ik {
struct Pose2D { double x, y, yaw; };
struct WheelTargets { std::array<double, 4> steer; std::array<double, 4> spin; };
// WHEEL_CORNERS[i] names the physical corner of model index i.
extern const std::array<const char *, 4> WHEEL_CORNERS;
class SimIkStepper {
public:
  explicit SimIkStepper(double ts = 0.06);
  void step(double vx, double vy, double yaw_rate);
  const WheelTargets & targets() const;
  const Pose2D & pose() const;
  bool indirect_mode() const;
  std::array<double, 2> feasible_icr() const;
};
}
```

- [ ] **Step 1: Write the failing test**

```cpp
// sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp
#include <gtest/gtest.h>

#include <cmath>

#include "navi_sim_ik/sim_ik_stepper.hpp"

using navi_sim_ik::SimIkStepper;

TEST(SimIkStepper, StartsAtTheOrigin)
{
  SimIkStepper stepper;
  EXPECT_DOUBLE_EQ(stepper.pose().x, 0.0);
  EXPECT_DOUBLE_EQ(stepper.pose().y, 0.0);
  EXPECT_DOUBLE_EQ(stepper.pose().yaw, 0.0);
}

TEST(SimIkStepper, DrivingForwardMovesAlongXAndNotAcross)
{
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {   // 6 seconds
    stepper.step(0.5, 0.0, 0.0);
  }
  EXPECT_GT(stepper.pose().x, 1.0);
  EXPECT_NEAR(stepper.pose().y, 0.0, 0.05);
  EXPECT_NEAR(stepper.pose().yaw, 0.0, 0.05);
}

TEST(SimIkStepper, TurningInPlaceChangesYawWithoutTravelling)
{
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.0, 0.0, 0.4);
  }
  EXPECT_GT(std::abs(stepper.pose().yaw), 0.5);
  EXPECT_NEAR(stepper.pose().x, 0.0, 0.05);
  EXPECT_NEAR(stepper.pose().y, 0.0, 0.05);
}

TEST(SimIkStepper, YawIsAppliedInTheBodyFrame)
{
  // Turn a quarter circle, then drive: the rover must go sideways in world
  // terms. Integrating in the world frame instead would send it along +X.
  SimIkStepper stepper;
  while (stepper.pose().yaw < M_PI / 2) {
    stepper.step(0.0, 0.0, 0.6);
  }
  const double x_before = stepper.pose().x;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  EXPECT_GT(stepper.pose().y, 1.0);
  EXPECT_NEAR(stepper.pose().x, x_before, 0.2);
}

TEST(SimIkStepper, WheelSpinIsReportedForEveryWheelWhenDriving)
{
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  for (int w = 0; w < 4; ++w) {
    EXPECT_GT(stepper.targets().spin[w], 0.0) << "wheel " << w;
    EXPECT_NEAR(stepper.targets().steer[w], 0.0, 0.05) << "wheel " << w;
  }
}

TEST(SimIkStepper, StandingStillDoesNotDrift)
{
  // The pose comes from the IK's constrained velocity, so a zero command
  // must integrate to exactly nothing - not to a slow creep that would look
  // like localisation drift later.
  SimIkStepper stepper;
  for (int i = 0; i < 200; ++i) {
    stepper.step(0.0, 0.0, 0.0);
  }
  EXPECT_NEAR(stepper.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(stepper.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(stepper.pose().yaw, 0.0, 1e-9);
}

TEST(SimIkStepper, TheCornerNamesAreTheOnesTheUrdfUses)
{
  const std::array<const char *, 4> expected{
    "front_left", "front_right", "rear_right", "rear_left"};
  for (int i = 0; i < 4; ++i) {
    EXPECT_STREQ(navi_sim_ik::WHEEL_CORNERS[i], expected[i]);
  }
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build --packages-select navi_sim_ik'
```
Expected: FAIL — `navi_sim_ik/sim_ik_stepper.hpp: No such file or directory`

- [ ] **Step 3: Write the header**

```cpp
// sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp
#ifndef NAVI_SIM_IK__SIM_IK_STEPPER_HPP_
#define NAVI_SIM_IK__SIM_IK_STEPPER_HPP_

#include <array>

#include "kinematics.h"

namespace navi_sim_ik
{

/// Which physical corner each index of the model's beta/omega arrays refers to.
///
/// UNVERIFIED. The model indexes wheels 1-4, matching Steer Module 1-4 and
/// Drive Module 1-4 on the I2C bus, but nothing in this project records which
/// corner each module is bolted to - that is wiring knowledge. Getting it
/// wrong steers the wrong corners while looking entirely plausible, which is
/// why it is one named constant, logged at startup, rather than four
/// scattered indices.
extern const std::array<const char *, 4> WHEEL_CORNERS;

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct WheelTargets
{
  std::array<double, 4> steer{{0.0, 0.0, 0.0, 0.0}};   ///< radians
  std::array<double, 4> spin{{0.0, 0.0, 0.0, 0.0}};    ///< radians/second
};

/// The rover's kinematics with no ROS attached.
///
/// A kinematic simulation has no encoders, so the measured steering angle and
/// rate fed back into the model are its own previous outputs. That closes the
/// loop the way the rover's hardware would when it tracks perfectly, which is
/// the assumption this whole simulation rests on.
class SimIkStepper
{
public:
  explicit SimIkStepper(double ts = 0.06);

  /// One tick of `ts` seconds against a body-frame velocity command.
  void step(double vx, double vy, double yaw_rate);

  const WheelTargets & targets() const {return targets_;}
  const Pose2D & pose() const {return pose_;}
  bool indirect_mode() const {return indirect_mode_;}
  std::array<double, 2> feasible_icr() const {return feasible_icr_;}

private:
  double ts_;
  kinematics model_;
  kinematics::ExternalInputs in_{};
  WheelTargets targets_{};
  Pose2D pose_{};
  bool indirect_mode_{false};
  std::array<double, 2> feasible_icr_{{0.0, 0.0}};
};

}  // namespace navi_sim_ik

#endif  // NAVI_SIM_IK__SIM_IK_STEPPER_HPP_
```

- [ ] **Step 4: Write the implementation**

```cpp
// sim/src/navi_sim_ik/src/sim_ik_stepper.cpp
#include "navi_sim_ik/sim_ik_stepper.hpp"

#include <cmath>

namespace navi_sim_ik
{

const std::array<const char *, 4> WHEEL_CORNERS{
  {"front_left", "front_right", "rear_right", "rear_left"}};

SimIkStepper::SimIkStepper(double ts)
: ts_(ts)
{
  model_.initialize();
  in_.TS = ts_;
}

void SimIkStepper::step(double vx, double vy, double yaw_rate)
{
  in_.VX_out = vx;
  in_.VY_out = vy;
  in_.U_p = yaw_rate;

  model_.setExternalInputs(&in_);
  model_.step();
  const auto & out = model_.getExternalOutputs();

  for (int w = 0; w < 4; ++w) {
    targets_.steer[w] = out.beta_next[w];
    targets_.spin[w] = out.omega[w];
    // No encoders in a kinematic simulation: the model's own output is what
    // it measures next tick.
    in_.beta_hat[w] = out.beta_next[w];
    in_.beta_dot_hat[w] = out.Beta_dot[w];
  }

  indirect_mode_ = out.indirect_mode;
  feasible_icr_ = {out.feasable_ICR[0], out.feasable_ICR[1]};

  // Integrate what the controller could actually deliver, not what was asked
  // for. eta_dot_constrained is the request after the ICR feasibility limits
  // have been applied, so a command the geometry cannot satisfy moves the
  // rover the way the rover would move rather than the way it was told to.
  const double achieved_vx = out.eta_dot_constrained[0];
  const double achieved_vy = out.eta_dot_constrained[1];
  const double achieved_yaw_rate = out.eta_dot_constrained[2];

  const double cos_yaw = std::cos(pose_.yaw);
  const double sin_yaw = std::sin(pose_.yaw);
  pose_.x += (achieved_vx * cos_yaw - achieved_vy * sin_yaw) * ts_;
  pose_.y += (achieved_vx * sin_yaw + achieved_vy * cos_yaw) * ts_;
  pose_.yaw += achieved_yaw_rate * ts_;
}

}  // namespace navi_sim_ik
```

Add to `CMakeLists.txt`, before the `BUILD_TESTING` block:

```cmake
add_library(navi_sim_ik_stepper STATIC src/sim_ik_stepper.cpp)
target_link_libraries(navi_sim_ik_stepper navi_sim_ik_model)
target_include_directories(navi_sim_ik_stepper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
```

and inside `BUILD_TESTING`:

```cmake
  ament_add_gtest(test_sim_ik_stepper test/test_sim_ik_stepper.cpp)
  target_link_libraries(test_sim_ik_stepper navi_sim_ik_stepper)
```

- [ ] **Step 5: Run the tests**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
```
Expected: all PASS.

If `StandingStillDoesNotDrift` fails, `eta_dot_constrained` is not the constrained body velocity and the assumption in `step()` is wrong. Stop and find out what it is rather than switching to integrating the raw command — the difference is the whole reason for using the real IK.

- [ ] **Step 6: Commit**

```bash
git add sim/src/navi_sim_ik
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Add the simulation's kinematic stepper

Twist in, four steering angles, four wheel speeds and an integrated pose
out, with no ROS attached so it can be tested directly.

Two choices worth stating. The pose integrates eta_dot_constrained
rather than the raw command, so a motion the geometry cannot satisfy
moves the rover the way the rover would move - integrating the request
would make the simulation agree with itself and disagree with reality.
And the model's measured steering feedback is its own previous output,
because a kinematic simulation has no encoders.

The wheel index to corner mapping is unverified wiring knowledge, so it
is one named constant rather than four scattered indices."
```

---

### Task 4: The ROS node

**Files:**
- Create: `sim/src/navi_sim_ik/src/sim_ik_node.cpp`
- Modify: `sim/src/navi_sim_ik/CMakeLists.txt`

**Interfaces:**
- Consumes: `SimIkStepper` from Task 3.
- Produces: node `sim_ik`. Subscribes `geometry_msgs/Twist` on `/manual_twist`. Publishes `trajectory_msgs/JointTrajectory` on `/set_joint_trajectory`, `geometry_msgs/Twist` on `/sim_cmd_vel`, `nav_msgs/Odometry` on `/sim_odom`, `std_msgs/String` (JSON) on `/sim_ik_debug`.

- [ ] **Step 1: Write the node**

```cpp
// sim/src/navi_sim_ik/src/sim_ik_node.cpp
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navi_sim_ik/sim_ik_stepper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

/// Drives the simulated rover from the same /manual_twist the real one gets.
///
/// The twist is read straight off the rover's ROS graph over DDS rather than
/// forwarded by the ground station: the ground station has no ROS and would
/// have needed a second rosbridge purely to repeat a message that is already
/// on the network.
class SimIkNode : public rclcpp::Node
{
public:
  SimIkNode()
  : Node("sim_ik"), stepper_(0.06)
  {
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/manual_twist", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        yaw_rate_ = msg->angular.z;
        last_twist_ = now();
      });

    joints_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/set_joint_trajectory", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/sim_cmd_vel", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim_odom", 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>("/sim_ik_debug", 10);

    // Logged every run because it is unverified and silent when wrong.
    RCLCPP_INFO(
      get_logger(), "wheel index mapping (UNVERIFIED): 0=%s 1=%s 2=%s 3=%s",
      navi_sim_ik::WHEEL_CORNERS[0], navi_sim_ik::WHEEL_CORNERS[1],
      navi_sim_ik::WHEEL_CORNERS[2], navi_sim_ik::WHEEL_CORNERS[3]);

    timer_ = create_wall_timer(60ms, [this] {tick();});
  }

private:
  void tick()
  {
    // A twist that stops arriving means the link to the rover is gone. Coast
    // to a stop rather than continuing to drive on a stale command: a
    // simulation that keeps moving after the rover stopped talking is worse
    // than one that freezes, because it looks alive.
    const bool stale = (now() - last_twist_) > rclcpp::Duration::from_seconds(1.0);
    if (stale) {
      if (!reported_stale_) {
        RCLCPP_WARN(get_logger(), "/manual_twist is stale - is the rover reachable?");
        reported_stale_ = true;
      }
      vx_ = vy_ = yaw_rate_ = 0.0;
    } else if (reported_stale_) {
      RCLCPP_INFO(get_logger(), "/manual_twist is flowing again");
      reported_stale_ = false;
    }

    stepper_.step(vx_, vy_, yaw_rate_);
    publish_joints();
    publish_motion();
    publish_debug(stale);
  }

  void publish_joints()
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = now();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < 4; ++i) {
      const std::string corner = navi_sim_ik::WHEEL_CORNERS[i];
      msg.joint_names.push_back("steer_" + corner + "_joint");
      point.positions.push_back(stepper_.targets().steer[i]);

      // The rolling angle is integrated here rather than commanded as a
      // velocity: joint_pose_trajectory sets positions, and a wheel that
      // never rotates makes a moving rover look like it is sliding.
      wheel_angle_[i] += stepper_.targets().spin[i] * 0.06;
      msg.joint_names.push_back("wheel_" + corner + "_joint");
      point.positions.push_back(wheel_angle_[i]);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(0.06);
    msg.points.push_back(point);
    joints_pub_->publish(msg);
  }

  void publish_motion()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = (stepper_.pose().x - last_pose_.x) / 0.06;
    cmd.linear.y = (stepper_.pose().y - last_pose_.y) / 0.06;
    cmd.angular.z = (stepper_.pose().yaw - last_pose_.yaw) / 0.06;
    last_pose_ = stepper_.pose();
    cmd_vel_pub_->publish(cmd);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = stepper_.pose().x;
    odom.pose.pose.position.y = stepper_.pose().y;
    odom.pose.pose.orientation.z = std::sin(stepper_.pose().yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(stepper_.pose().yaw / 2.0);
    odom_pub_->publish(odom);
  }

  void publish_debug(bool stale)
  {
    std::ostringstream json;
    json << "{\"indirect_mode\":" << (stepper_.indirect_mode() ? "true" : "false")
         << ",\"feasible_icr\":[" << stepper_.feasible_icr()[0] << ","
         << stepper_.feasible_icr()[1] << "]"
         << ",\"pose\":[" << stepper_.pose().x << "," << stepper_.pose().y << ","
         << stepper_.pose().yaw << "]"
         << ",\"twist_stale\":" << (stale ? "true" : "false") << "}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    debug_pub_->publish(msg);
  }

  navi_sim_ik::SimIkStepper stepper_;
  double vx_{0.0}, vy_{0.0}, yaw_rate_{0.0};
  std::array<double, 4> wheel_angle_{{0.0, 0.0, 0.0, 0.0}};
  navi_sim_ik::Pose2D last_pose_{};
  rclcpp::Time last_twist_{0, 0, RCL_ROS_TIME};
  bool reported_stale_{true};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimIkNode>());
  rclcpp::shutdown();
  return 0;
}
```

Add to `CMakeLists.txt`:

```cmake
add_executable(sim_ik_node src/sim_ik_node.cpp)
target_link_libraries(sim_ik_node navi_sim_ik_stepper)
ament_target_dependencies(sim_ik_node
  rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs trajectory_msgs)
install(TARGETS sim_ik_node DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 2: Build**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build --packages-select navi_sim_ik'
```
Expected: builds clean.

- [ ] **Step 3: Run it against the real rover**

With `~/navi/start_navi.sh` running on the Orin:

```bash
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && ros2 run navi_sim_ik sim_ik_node'
```

In another terminal, drive the gamepad through the ground station and watch:

```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /sim_ik_debug'
```
Expected: `pose` moves while you drive and holds still when you stop; `twist_stale` is `false`. Note the logged wheel mapping line.

- [ ] **Step 4: Commit**

```bash
git add sim/src/navi_sim_ik
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Add the simulation's IK node

Reads /manual_twist off the rover's ROS graph directly, steps the
kinematics every 60 ms and publishes joint positions, the achieved body
velocity, odometry and the ICR diagnostics.

A twist that stops arriving is treated as the rover being unreachable
and the rover coasts to a stop, because a simulation that keeps driving
on a stale command looks alive when it is not. The rolling angle is
integrated here rather than commanded as a velocity, since the Gazebo
plugin that will consume this sets positions - and a moving rover with
motionless wheels reads as sliding."
```

---

### Task 5: The world, the robot in it, and the chase camera

**Files:**
- Create: `sim/src/navi_sim_bringup/package.xml`, `CMakeLists.txt`
- Create: `sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro`
- Create: `sim/src/navi_sim_bringup/worlds/site.world`
- Create: `sim/src/navi_sim_bringup/launch/sim.launch.py`

**Interfaces:**
- Consumes: joint names from Task 1; `/set_joint_trajectory` and `/sim_cmd_vel` from Task 4.
- Produces: Gazebo running the rover in the scanned world, publishing camera images on `/sim_chase_camera/image_raw` at 672x376.

- [ ] **Step 1: Write the world**

```xml
<?xml version="1.0"?>
<!-- sim/src/navi_sim_bringup/worlds/site.world -->
<sdf version="1.6">
  <world name="site">
    <include><uri>model://sun</uri></include>

    <!-- The rover drives on this, not on the scan. The scan is scenery:
         with no physics there is nothing to stand on, and its 1.68 M faces
         would be a punishing collision mesh for no benefit. -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.25 0.25 0.28 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- The scanned site. MAP_MESH_PATH is substituted by the launch file:
         the .obj is gitignored and 161 MB, so it is not in this repository
         and its location cannot be hardcoded. -->
    <model name="site_scan">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>file://MAP_MESH_PATH</uri></mesh></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 2: Write the xacro that adds the simulation-only parts**

```xml
<?xml version="1.0"?>
<!-- sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro

     The real robot plus the parts that exist only in simulation. Kept
     separate so asterope_iiI.urdf stays a description of the rover that
     exists, with no fictional camera on it. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="asterope">
  <xacro:include filename="$(find navi_sim_bringup)/urdf/asterope_base.urdf"/>

  <!-- Chase camera: rigidly behind and above the rover, looking down at it.
       Attached to the robot rather than free-flying, which makes following
       the rover automatic and needs no control channel back from the
       ground station - a video stream is one-way. -->
  <link name="chase_camera_link"/>
  <joint name="chase_camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chase_camera_link"/>
    <origin xyz="-3.0 0 1.8" rpy="0 0.35 0"/>
  </joint>

  <gazebo reference="chase_camera_link">
    <sensor type="camera" name="chase_camera">
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.2</horizontal_fov>
        <!-- 672x376 matches what the rover streams, so the ground station
             panel gets the same frame size from either source. -->
        <image><width>672</width><height>376</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>200</far></clip>
      </camera>
      <plugin name="chase_camera_controller" filename="libgazebo_ros_camera.so">
        <ros><namespace>/sim_chase_camera</namespace></ros>
        <camera_name>chase</camera_name>
        <frame_name>chase_camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Moves the model from /sim_cmd_vel without physics: exactly the
       kinematic body motion this simulation wants. -->
  <gazebo>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros><remapping>cmd_vel:=/sim_cmd_vel</remapping></ros>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odometry_frame>odom</odometry_frame>
      <publish_odom>false</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

  <!-- Sets joint positions from /set_joint_trajectory. -->
  <gazebo>
    <plugin name="joint_pose_trajectory" filename="libgazebo_ros_joint_pose_trajectory.so">
      <update_rate>30</update_rate>
    </plugin>
  </gazebo>
</robot>
```

- [ ] **Step 3: Write the launch file**

```python
# sim/src/navi_sim_bringup/launch/sim.launch.py
"""Bring up Gazebo with the rover in the scanned site.

The map mesh path is a launch argument rather than a fixed path: the .obj is
gitignored and 161 MB, so it is not part of this repository and cannot be
assumed to sit anywhere in particular.
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world_with_mesh(context, *args, **kwargs):
    share = get_package_share_directory("navi_sim_bringup")
    mesh = LaunchConfiguration("map_mesh").perform(context)
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
    description = os.popen(f"xacro {robot}").read()

    return [
        ExecuteProcess(
            cmd=["gazebo", "--verbose", generated,
                 "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so"],
            output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", "asterope",
                        "-z", "0.05"],
             output="screen"),
        Node(package="navi_sim_ik", executable="sim_ik_node", output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_mesh",
            default_value=os.path.expanduser("~/star/Navi/Model3D_mesh2.obj"),
            description="Path to the scanned site mesh (.obj)"),
        OpaqueFunction(function=_world_with_mesh),
    ])
```

- [ ] **Step 4: Write package.xml and CMakeLists.txt**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>navi_sim_bringup</name>
  <version>0.1.0</version>
  <description>World, robot description and launch for the rover simulation.</description>
  <maintainer email="ole.peters@star-dresden.de">Ole Peters</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>navi_sim_ik</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

```cmake
cmake_minimum_required(VERSION 3.8)
project(navi_sim_bringup)
find_package(ament_cmake REQUIRED)

# The real URDF is copied in at build time rather than referenced from the
# working tree, so an installed package does not depend on the source
# checkout still being there.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../../asterope_iiI.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/asterope_base.urdf COPYONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/asterope_base.urdf
        DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY urdf worlds launch DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 5: Build and run it**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build && source install/setup.bash &&
ros2 launch navi_sim_bringup sim.launch.py map_mesh:=/home/ole/star/Navi/Model3D_mesh2.obj'
```
Expected: Gazebo opens with the rover on the plane and the scan around it. Loading 1.68 M faces takes a while; wait before deciding it has hung.

With the rover running and the gamepad driving, the rover should move and its wheels should steer and spin. Check the camera:

```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /sim_chase_camera/chase/image_raw'
```
Expected: about 30 Hz.

- [ ] **Step 6: Commit**

```bash
git add sim/src/navi_sim_bringup
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Add the Gazebo world, robot description and launch

The rover drives on a flat plane with the scanned site as visual-only
scenery: with no physics there is nothing to stand on, and 1.68 M faces
would be a punishing collision mesh for no benefit. The mesh path is a
launch argument because the .obj is gitignored and 161 MB, so it is not
in this repository and cannot be assumed to sit anywhere.

Simulation-only parts live in a xacro that includes the real URDF,
keeping asterope_iiI.urdf a description of the rover that exists rather
than one with a fictional chase camera bolted on. That camera is rigidly
attached to the rover, which makes following it automatic and needs no
control channel back from the ground station - a video stream is
one-way."
```

---

### Task 6: Stream the chase camera to the ground station

**Files:**
- Create: `sim/src/navi_sim_video/package.xml`, `setup.py`, `navi_sim_video/__init__.py`, `navi_sim_video/sender.py`
- Test: `sim/src/navi_sim_video/test/test_sender.py`

**Interfaces:**
- Consumes: `/sim_chase_camera/chase/image_raw` from Task 5.
- Produces: H.264/RTP on UDP 5601, decodable by `ground_station.video_receiver.VideoReceiver(port=5601)`.

- [ ] **Step 1: Install the encoder**

```bash
sudo apt install gstreamer1.0-plugins-ugly
gst-inspect-1.0 x264enc >/dev/null && echo "x264enc present"
```
Expected: `x264enc present`. Nothing in this task works without it.

- [ ] **Step 2: Write the failing test**

```python
# sim/src/navi_sim_video/test/test_sender.py
from navi_sim_video.sender import build_send_pipeline


def test_the_pipeline_reads_raw_frames_from_stdin():
    # Frames arrive as ROS messages, so gst cannot fetch them itself - the
    # node writes them in. Same shape as the rover's sender, which pipes
    # rather than linking GStreamer into the process.
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert argv[0] == "gst-launch-1.0"
    assert "fdsrc" in argv
    assert any("width=672" in part and "height=376" in part for part in argv)


def test_the_pipeline_encodes_for_low_latency():
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert "x264enc" in argv
    assert any("tune=zerolatency" in part for part in argv)
    # Repeats SPS/PPS so a receiver joining late can start decoding without
    # the stream being restarted.
    assert any("config-interval=1" in part for part in argv)


def test_the_pipeline_targets_the_simulation_port():
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert "port=5601" in argv
    assert "host=127.0.0.1" in argv


def test_the_bitrate_is_carried_through():
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 1500)
    assert any("bitrate=1500" in part for part in argv)
```

- [ ] **Step 3: Run to verify it fails**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build --packages-select navi_sim_video && colcon test --packages-select navi_sim_video && colcon test-result --verbose'
```
Expected: FAIL — `ModuleNotFoundError: No module named 'navi_sim_video.sender'`

- [ ] **Step 4: Write the sender**

```python
# sim/src/navi_sim_video/navi_sim_video/sender.py
"""Streams the simulation's chase camera to the ground station.

The same protocol, frame size and encoder settings as the rover's
video_sender, so the ground station decodes either source with the same
receiver and the same panel. Only the port differs: 5601 rather than 5600,
which is what lets the mode switch change source without an ordering hazard
between two senders on one port.

Frames go into gst-launch through a pipe rather than an in-process pipeline,
following the rover's sender: it needs no PyGObject, and a decoder crash on a
corrupt stream kills a child process rather than this node.
"""

import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def build_send_pipeline(host: str, port: int, width: int, height: int,
                        fps: int, bitrate_kbps: int) -> list[str]:
    return [
        "gst-launch-1.0", "-q",
        "fdsrc", "fd=0",
        "!", f"video/x-raw,format=RGB,width={width},height={height},"
             f"framerate={fps}/1",
        "!", "videoconvert",
        "!", "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
        f"bitrate={bitrate_kbps}", "key-int-max=30",
        "!", "rtph264pay", "config-interval=1", "pt=96",
        "!", "udpsink", f"host={host}", f"port={port}",
    ]


class SimVideoSender(Node):
    def __init__(self):
        super().__init__("sim_video_sender")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5601)
        self.declare_parameter("width", 672)
        self.declare_parameter("height", 376)
        self.declare_parameter("fps", 30)
        self.declare_parameter("bitrate_kbps", 800)

        argv = build_send_pipeline(
            self.get_parameter("host").value,
            self.get_parameter("port").value,
            self.get_parameter("width").value,
            self.get_parameter("height").value,
            self.get_parameter("fps").value,
            self.get_parameter("bitrate_kbps").value,
        )
        # stderr to a file, never a pipe: nothing here would drain a pipe,
        # and at about 64 KB the encoder would block in write() forever.
        self._stderr = tempfile.NamedTemporaryFile(
            mode="w", prefix="sim_video_sender_stderr_", delete=False)
        self._process = subprocess.Popen(
            argv, stdin=subprocess.PIPE, stderr=self._stderr)

        self._expected = (self.get_parameter("width").value
                          * self.get_parameter("height").value * 3)
        self.create_subscription(Image, "/sim_chase_camera/chase/image_raw",
                                 self._on_image, 1)
        self.get_logger().info(
            f"streaming to {self.get_parameter('host').value}:"
            f"{self.get_parameter('port').value}")

    def _on_image(self, msg: Image) -> None:
        if len(msg.data) != self._expected:
            # A size mismatch would be sliced into torn, progressively
            # desynchronised frames by the receiver rather than failing.
            self.get_logger().warn(
                f"dropping frame: got {len(msg.data)} bytes, "
                f"expected {self._expected}")
            return
        try:
            self._process.stdin.write(bytes(msg.data))
        except BrokenPipeError:
            self.get_logger().error("encoder exited - see " + self._stderr.name)
            raise SystemExit(1)

    def destroy_node(self):
        if self._process.poll() is None:
            self._process.terminate()
            self._process.wait(timeout=3)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SimVideoSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

`setup.py`:

```python
from setuptools import setup

package_name = "navi_sim_video"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ole Peters",
    maintainer_email="ole.peters@star-dresden.de",
    description="Streams the simulation's chase camera to the ground station.",
    license="Proprietary",
    entry_points={"console_scripts": [
        "sim_video_sender = navi_sim_video.sender:main"]},
)
```

`package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>navi_sim_video</name>
  <version>0.1.0</version>
  <description>Streams the simulation's chase camera to the ground station.</description>
  <maintainer email="ole.peters@star-dresden.de">Ole Peters</maintainer>
  <license>Proprietary</license>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <test_depend>ament_copyright</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

Create the empty marker file `sim/src/navi_sim_video/resource/navi_sim_video` and `sim/src/navi_sim_video/navi_sim_video/__init__.py`.

- [ ] **Step 5: Run the tests**

```bash
bash -c 'source /opt/ros/humble/setup.bash && cd sim && colcon build --packages-select navi_sim_video && colcon test --packages-select navi_sim_video && colcon test-result --verbose'
```
Expected: all PASS.

- [ ] **Step 6: Prove a frame crosses the wire**

With the simulation running from Task 5:

```bash
bash -c 'source /opt/ros/humble/setup.bash && source sim/install/setup.bash && ros2 run navi_sim_video sim_video_sender' &
cd /home/ole/star/Navi && timeout -s INT 15 .venv/bin/python -m ground_station.video_receiver --port 5601
```
Expected: a frame count in the hundreds. Zero means the encoder is missing or the camera topic name is wrong — check the sender's stderr temp file, whose path it logs.

- [ ] **Step 7: Add the sender to the launch file**

In `sim.launch.py`, add to the returned list:

```python
        Node(package="navi_sim_video", executable="sim_video_sender", output="screen"),
```

- [ ] **Step 8: Commit**

```bash
git add sim/src/navi_sim_video sim/src/navi_sim_bringup/launch/sim.launch.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Stream the simulation's chase camera to the ground station

Same protocol, frame size and encoder settings as the rover's sender, so
the ground station decodes either source with the same receiver and the
same panel. Only the port differs - 5601 rather than 5600 - which lets
the mode switch change source without two senders ever contending for
one port and decoding each other's late packets as garbage.

No /video_request control plane here. The rover needs one because it is
a different machine that has to be told to start encoding; this runs
beside the ground station, so it simply streams while the simulation
does."
```

---

### Task 7: The mode switch in the ground station

**Files:**
- Modify: `ground_station/ui/video_panel.py`
- Modify: `ground_station/ui/dashboard_page.py`
- Modify: `ground_station/ui/main_window.py`
- Create: `start_sim.sh`
- Test: `tests/test_video_panel.py`, `tests/test_main_window.py`

**Interfaces:**
- Consumes: the sim stream on 5601 from Task 6.
- Produces: `VideoPanel.set_source(name: str, port: int)`, `VideoPanel.dead_reckoning: bool`, `DashboardPage.mode_changed = Signal(str)` emitting `"manual"` or `"semi_auto"`.

- [ ] **Step 1: Write the failing tests**

```python
# append to tests/test_video_panel.py

def test_switching_source_restarts_the_receiver_on_the_new_port(qtbot):
    receiver = FakeReceiver()
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)
    panel.set_streaming(True)

    panel.set_source("simulation", 5601)

    assert receiver.port == 5601
    assert receiver.stopped is True


def test_the_sim_source_is_marked_as_dead_reckoning(qtbot):
    # The pose is integrated from commanded twist, so it drifts from the real
    # rover and the picture cannot show that. Saying so is the only defence.
    receiver = FakeReceiver()
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)

    panel.set_source("simulation", 5601, dead_reckoning=True)

    assert "DEAD RECKONING" in panel.title_label.text().upper()


def test_the_rover_source_is_not_marked_as_dead_reckoning(qtbot):
    receiver = FakeReceiver()
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)

    panel.set_source("rover", 5600, dead_reckoning=False)

    assert "DEAD RECKONING" not in panel.title_label.text().upper()
```

```python
# append to tests/test_main_window.py

def test_entering_semi_auto_stops_rover_video_and_switches_port(qtbot, window):
    window.dashboard_page.mode_changed.emit("semi_auto")

    assert window.dashboard_page.video_panel.receiver.port == 5601
    # The rover keeps being driven - only its camera is turned off, to spare
    # the link while nobody is looking at it.
    assert window.ros_client.published_video_requests[-1]["enable"] is False


def test_leaving_semi_auto_returns_to_the_rover_camera(qtbot, window):
    window.dashboard_page.mode_changed.emit("semi_auto")
    window.dashboard_page.mode_changed.emit("manual")

    assert window.dashboard_page.video_panel.receiver.port == 5600


def test_the_twist_still_reaches_the_rover_in_semi_auto(qtbot, window):
    # The rover is driven in both modes. Anything else would make the mode
    # switch a control change disguised as a view change.
    window.dashboard_page.mode_changed.emit("semi_auto")

    window._update_drive_display(0.4, 0.0, 0.2)

    assert window.ros_client.published_twists[-1] == (0.4, 0.0, 0.2)
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_video_panel.py tests/test_main_window.py -v -k "source or dead_reckoning or semi_auto"`
Expected: FAIL — `AttributeError: 'VideoPanel' object has no attribute 'set_source'`

- [ ] **Step 3: Add `set_source` and the marker to `VideoPanel`**

Give the panel a `title_label` (the existing `title` local becomes an attribute), then:

```python
    def set_source(self, name: str, port: int, *, dead_reckoning: bool = False) -> None:
        """Points the panel at a different sender.

        The receiver is stopped and re-pointed rather than a second one being
        created: two receivers would both be bound, and whichever the panel
        was not reading would silently fill its socket buffer.
        """
        was_streaming = self._streaming
        self.stop_receiver()
        self.receiver.port = port
        self._source_name = name
        self._dead_reckoning = dead_reckoning
        self._refresh_title()
        if was_streaming:
            self.set_streaming(True)

    def _refresh_title(self) -> None:
        title = f"CAMERA / {self._source_name.upper()}"
        if self._dead_reckoning:
            # The simulated pose is integrated from commanded twist, so it
            # drifts from the real rover and the picture cannot show it.
            title += "  -  DEAD RECKONING, NO LOCALISATION"
        self.title_label.setText(title)
```

Initialise `self._source_name = "zed front left"` and `self._dead_reckoning = False` in `__init__`, and call `_refresh_title()` there.

- [ ] **Step 4: Add the switch to `DashboardPage`**

```python
    mode_changed = Signal(str)
```

with two radio buttons, "Manual" and "Semi-autonomous", emitting `"manual"` and `"semi_auto"`.

- [ ] **Step 5: Wire it in `MainWindow`**

```python
ROVER_VIDEO_PORT = 5600
SIM_VIDEO_PORT = 5601

    def _on_mode_changed(self, mode: str) -> None:
        panel = self.dashboard_page.video_panel
        if mode == "semi_auto":
            # Stop the rover's camera: nobody is looking at it, and the field
            # link is the scarce resource. The rover keeps being driven.
            if self.ros_client is not None:
                self.ros_client.publish_video_request(enable=False)
            panel.set_source("simulation", SIM_VIDEO_PORT, dead_reckoning=True)
        else:
            panel.set_source("zed front left", ROVER_VIDEO_PORT)
```

connected in `__init__` with `self.dashboard_page.mode_changed.connect(self._on_mode_changed)`.

- [ ] **Step 6: Run the whole suite**

Run: `.venv/bin/python -m pytest tests/ -q`
Expected: all PASS, including the 109 that already existed.

- [ ] **Step 7: Write `start_sim.sh`**

Follow `start_navi.sh`: `own_pids`/`kill_stale` guarded against self-matching, no pattern for the script's own name, and `|| true` on any pipeline whose grep may legitimately find nothing. Clean up stale `gzserver`, `gzclient`, `sim_ik_node`, `sim_video_sender` and any `gst-launch` matching `fdsrc.*udpsink`. Then build if needed and launch.

- [ ] **Step 8: Run it end to end**

With the rover up and the ground station running: `./start_sim.sh`, switch the ground station to Semi-autonomous, and drive. The panel shows the rover in the scanned site, moving as commanded, wheels steering and rolling, titled `DEAD RECKONING, NO LOCALISATION`.

- [ ] **Step 9: Commit**

```bash
git add ground_station tests start_sim.sh
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
  commit -m "Show the simulation in place of the camera in semi-autonomous mode

The switch selects a view source and nothing else: the twist still goes
to the rover in both modes, because a mode switch that quietly changed
what is being driven would be a control change wearing a view change's
clothes. The rover's camera is stopped on entry to spare the field link,
which is the one thing the rover does differently.

The panel re-points its existing receiver at 5601 rather than holding a
second one - two bound receivers would mean the unread one silently
filling its socket buffer.

The simulated view is labelled DEAD RECKONING, NO LOCALISATION. Its pose
is integrated from commanded twist, so it drifts from the real rover and
nothing in the picture would ever show that."
```

---

## Self-Review

**Spec coverage.** Goal → Tasks 1-7. Kinematic-only → Tasks 3, 5. IK vendored unchanged → Task 2. DDS twist → Task 4. Video on 5601 through the existing receiver → Tasks 6, 7. No sim control plane → Task 6. Two ports → Tasks 6, 7. Dead reckoning marker → Task 7. Wheel index mapping in one constant, logged → Tasks 3, 4. Map as visual-only scenery, flat plane → Task 5. Error handling: stale twist → Task 4; dead receiver → existing panel behaviour, unchanged; infeasible commands surfaced → Task 4's `/sim_ik_debug`. Testing section → the test steps of Tasks 1, 2, 3, 6, 7. Prerequisite `gstreamer1.0-plugins-ugly` → Task 6 Step 1.

**Gaps accepted deliberately.** The spec's "Deferred" items are not planned, by definition. Gazebo itself is verified by running it, as the spec says, so Tasks 5 and 7 end in manual checks rather than assertions.

**Type consistency.** `WHEEL_CORNERS`, `SimIkStepper`, `Pose2D`, `WheelTargets` are defined in Task 3 and used in Task 4 under those names. Joint names `steer_<corner>_joint` and `wheel_<corner>_joint` are fixed in Task 1 and used in Tasks 3-5. `build_send_pipeline` is defined and tested in Task 6. `set_source`, `mode_changed`, `ROVER_VIDEO_PORT`, `SIM_VIDEO_PORT` are defined and used within Task 7.

**Known risk carried into execution.** Task 3 assumes `eta_dot_constrained` is `(vx, vy, yaw_rate)` after feasibility limiting, and Task 2 assumes `U_p` is the yaw rate. Both are inferred from the Simulink signal names, not from documentation. Each has a test whose failure means the assumption is wrong, and each says so at the point of failure rather than being quietly worked around.
