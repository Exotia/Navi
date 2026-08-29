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


def test_the_last_good_pose_is_still_republished_while_off(node):
    # OFF means no wrapper pose is arriving, so _on_pose stops running and
    # /localization/pose would go silent - a ground station that connects
    # while the wrapper is down would then see no position at all, though
    # the last known one is right there. tracker.py promises it stays on
    # offer, stamp frozen; the timer is the only thing left to say it.
    node._on_status(status(True))
    node._on_pose(camera_pose(2.345, 0.0, 0.548, 0.0, 100.0))
    published = len(node._pose_publisher.messages)
    node._tracker._last_message_at = node._now() - 60.0

    node._tick()

    assert json.loads(node._status_publisher.messages[-1].data)["state"] == "OFF"
    assert len(node._pose_publisher.messages) == published + 1
    last = node._pose_publisher.messages[-1]
    assert last.pose.pose.position.x == pytest.approx(2.0, abs=1e-9)
    assert last.header.stamp.sec == 100


def test_nothing_is_published_while_off_before_any_pose_arrived(node):
    # A node that has never seen a pose has none to offer, and must not
    # publish an empty one that reads as "the rover is at the origin".
    node._tick()

    assert node._pose_publisher.messages == []
    assert json.loads(node._status_publisher.messages[-1].data)["state"] == "OFF"


def test_the_magnetometer_is_never_subscribed(node):
    topics = [s.topic_name for s in node.subscriptions]
    assert not any("mag" in t for t in topics)
    assert "/zed_front/zed_node/pose" in topics
