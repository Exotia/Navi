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
