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


def test_the_base_link_constant_matches_the_urdf_base_footprint_joint():
    robot = ET.parse(URDF).getroot()
    base = origin_of(robot, "base_footprint_joint")
    t = load_pose_composition().BASE_LINK_IN_BASE_FOOTPRINT
    assert (t.x, t.y, t.z) == pytest.approx((base[0], base[1], base[2]))
    assert (t.qx, t.qy, t.qz, t.qw) == (0.0, 0.0, 0.0, 1.0)
