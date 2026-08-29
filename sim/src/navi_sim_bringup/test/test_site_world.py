"""What is in the world in each mode.

The rule from the design: in semi-autonomous mode the ground under the rover
is the ground the rover has seen, so the organisers' static scan is left out
entirely - but the ground plane at z = 0 stays in both modes, because
without it a rover whose map has not arrived yet is in the void.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from navi_sim_bringup.world_composition import (
    MESH_PLACEHOLDER, SEMI_GROUND_Z, SITE_SCAN_MARKER, compose_world, site_scan_required)

WORLDS = Path(__file__).resolve().parent.parent / "worlds"
WORLD = (WORLDS / "site.world").read_text()
SCAN = (WORLDS / "site_scan.model").read_text()


def model_names(world_text):
    return [model.get("name")
            for model in ET.fromstring(world_text).find("world").findall("model")]


def test_the_world_file_itself_carries_no_terrain():
    assert SITE_SCAN_MARKER in WORLD
    assert "site_scan" not in WORLD


@pytest.mark.parametrize("mode", ["semi", "simulation"])
def test_the_ground_plane_is_there_in_every_mode(mode):
    assert "ground" in model_names(compose_world(WORLD, SCAN, mode, "/tmp/mesh.obj"))


@pytest.mark.parametrize("mode", ["semi", "simulation"])
def test_the_composed_world_is_valid_xml(mode):
    ET.fromstring(compose_world(WORLD, SCAN, mode, "/tmp/mesh.obj"))


def test_semi_mode_leaves_the_organisers_scan_out():
    world = compose_world(WORLD, SCAN, "semi", "/tmp/mesh.obj")

    assert "site_scan" not in model_names(world)


def test_simulation_mode_still_gets_the_scan_with_its_mesh_path():
    world = compose_world(WORLD, SCAN, "simulation", "/tmp/mesh.obj")

    assert "site_scan" in model_names(world)
    assert "file:///tmp/mesh.obj" in world
    assert MESH_PLACEHOLDER not in world


def test_the_mesh_is_only_required_when_the_scan_is():
    # The .obj is gitignored and 161 MB; demanding it for a mode that does
    # not display it would make the rover's own map unusable without it.
    assert site_scan_required("simulation") is True
    assert site_scan_required("semi") is False


def test_a_world_that_lost_the_marker_fails_loudly():
    with pytest.raises(RuntimeError):
        compose_world("<sdf><world name='x'></world></sdf>", SCAN, "semi", "")


def ground_pose_z(world_text):
    world = ET.fromstring(world_text).find("world")
    ground = next(m for m in world.findall("model") if m.get("name") == "ground")
    pose = ground.find("pose")
    return 0.0 if pose is None else float(pose.text.split()[2])


def test_semi_mode_lowers_the_ground_plane_under_the_map():
    # The localised z on flat ground is about -0.55 (the ZED's origin is
    # where it started, 0.55 m up); the plane at 0 would hide the rover
    # and every terrain tile.
    assert ground_pose_z(compose_world(WORLD, SCAN, "semi", "")) == SEMI_GROUND_Z
    assert SEMI_GROUND_Z < -1.0


def test_simulation_mode_keeps_the_ground_plane_at_zero():
    assert ground_pose_z(compose_world(WORLD, SCAN, "simulation", "/tmp/mesh.obj")) == 0.0
