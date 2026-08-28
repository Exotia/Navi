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
