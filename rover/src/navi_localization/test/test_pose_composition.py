import math

import pytest

from navi_localization.pose_composition import (
    BASE_LINK_IN_BASE_FOOTPRINT, CAMERA_IN_BASE_FOOTPRINT, IDENTITY,
    STATIC_FRAMES, Transform, compose, footprint_pose_from_camera_pose,
    inverse, translation_distance, yaw_of)


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
