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


@pytest.mark.parametrize("corner", CORNERS)
@pytest.mark.parametrize("kind", ["steer", "wheel"])
def test_steer_and_wheel_links_have_a_positive_mass_inertial(robot, kind, corner):
    # Gazebo's URDF->SDF conversion silently drops any link on a non-fixed
    # joint (steer is revolute, wheel is continuous) if it has no
    # <inertial> - no error, no warning, the link just vanishes from the
    # spawned model. That failure mode is invisible to anything except
    # actually looking at Gazebo, so it needs this guard instead.
    name = f"{kind}_{corner}"
    link = [l for l in robot.findall("link") if l.get("name") == name][0]
    inertial = link.find("inertial")
    assert inertial is not None, f"{name} has no <inertial> - Gazebo will drop it"
    mass = inertial.find("mass")
    assert mass is not None
    assert float(mass.get("value")) > 0


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
