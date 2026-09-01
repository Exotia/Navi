"""Node-level tests for site_probe.py: the rover answers one depth-probe
click at a time. Every callback is driven directly with hand-built
messages, the way test_localization_status.py does - no graph, no
executor, and (unlike that file) no zed_msgs: this node only needs
sensor_msgs, nav_msgs and std_msgs, so it is laptop-safe with ROS sourced.
"""

import json
import math
import struct

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from navi_localization import landmark_geometry as lg
from navi_localization.pose_composition import IDENTITY, Transform
from navi_localization.site_probe import SiteProbe


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = SiteProbe()
    n._result_publisher = Recorder()
    yield n
    n.destroy_node()


def depth_image(width, height, values, encoding="32FC1", is_bigendian=False):
    """A synthetic sensor_msgs/Image of 32-bit floats, `values` a flat
    row-major list of length width*height."""
    assert len(values) == width * height
    fmt = (">" if is_bigendian else "<") + f"{len(values)}f"
    msg = Image()
    msg.encoding = encoding
    msg.width = width
    msg.height = height
    msg.step = width * 4
    msg.is_bigendian = is_bigendian
    msg.data = bytearray(struct.pack(fmt, *values))
    return msg


def camera_info(fx, fy, cx, cy, width, height):
    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return msg


def odometry(x, y, z, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def status_string(state):
    msg = String()
    msg.data = json.dumps({"state": state, "reason": "", "seconds_since_ok": 0.0,
                           "source": "zed_vio", "distance_travelled": 0.0,
                           "mount_offset_verified": True})
    return msg


def request_string(payload: dict):
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def make_request(request_id="p-1.000-0", label="51", u=320, v=240,
                 width=640, height=480, target="pole", patch_px=11):
    return {"request_id": request_id, "label": label, "u": u, "v": v,
            "width": width, "height": height, "target": target,
            "patch_px": patch_px}


def ready(node, width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
         depth=4.0, pose=None):
    """Feeds the node a depth image, matching camera info, an OK status
    and a pose, so a request only fails for the reason a test injects."""
    node._on_camera_info(camera_info(fx, fy, cx, cy, width, height))
    node._on_depth(depth_image(width, height, [depth] * (width * height)))
    node._on_status(status_string("OK"))
    node._on_pose(odometry(*(pose or (0.0, 0.0, 0.0))))


def test_centre_pixel_pole_matches_the_pure_function(node):
    ready(node)
    node._on_request(request_string(make_request(target="pole")))

    assert len(node._result_publisher.messages) == 1
    result = node._result_publisher.messages[0]
    assert result["ok"] is True
    assert result["request_id"] == "p-1.000-0"
    assert result["label"] == "51"
    assert result["frame_id"] == "map"

    intr = lg.Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    expected = lg.landmark_point_in_map(320, 240, 4.0, intr, IDENTITY, offset_m=0.0)
    assert result["x"] == pytest.approx(expected[0], abs=1e-9)
    assert result["y"] == pytest.approx(expected[1], abs=1e-9)
    assert result["z"] == pytest.approx(expected[2], abs=1e-9)


def test_box_face_target_adds_the_pure_function_offset(node):
    ready(node)
    node._on_request(request_string(make_request(target="box_face")))

    result = node._result_publisher.messages[0]
    intr = lg.Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    expected = lg.landmark_point_in_map(320, 240, 4.0, intr, IDENTITY, offset_m=0.125)
    assert result["x"] == pytest.approx(expected[0], abs=1e-9)
    assert result["y"] == pytest.approx(expected[1], abs=1e-9)
    assert result["z"] == pytest.approx(expected[2], abs=1e-9)


def test_a_downscaled_click_resolves_to_the_same_point_as_full_res(node):
    ready(node, width=1280, height=720)
    node._on_request(request_string(make_request(
        u=320, v=180, width=640, height=360)))
    small = node._result_publisher.messages[-1]

    node._on_request(request_string(make_request(
        request_id="p-1.000-1", u=640, v=360, width=1280, height=720)))
    full = node._result_publisher.messages[-1]

    assert small["ok"] is True and full["ok"] is True
    assert small["x"] == pytest.approx(full["x"], abs=1e-6)
    assert small["y"] == pytest.approx(full["y"], abs=1e-6)
    assert small["z"] == pytest.approx(full["z"], abs=1e-6)


def test_all_invalid_patch_fails_with_no_valid_depth(node):
    node._on_camera_info(camera_info(500.0, 500.0, 320.0, 240.0, 640, 480))
    node._on_depth(depth_image(640, 480, [float("nan")] * (640 * 480)))
    node._on_status(status_string("OK"))
    node._on_pose(odometry(0.0, 0.0, 0.0))
    node._on_request(request_string(make_request()))

    result = node._result_publisher.messages[0]
    assert result["ok"] is False
    assert result["error"] == "no valid depth at that pixel"
    assert result["x"] is None and result["y"] is None
    assert result["z"] is None and result["range_m"] is None


def test_valid_fraction_below_threshold_fails_above_succeeds(node):
    width, height = 5, 5
    node._on_camera_info(camera_info(500.0, 500.0, 2.0, 2.0, width, height))
    node._on_status(status_string("OK"))
    node._on_pose(odometry(0.0, 0.0, 0.0))
    node.set_parameters([rclpy.parameter.Parameter(
        "min_valid_fraction", rclpy.Parameter.Type.DOUBLE, 0.25)])

    values = [float("nan")] * (width * height)
    for i in range(5):
        values[i] = 4.0
    node._on_depth(depth_image(width, height, values))
    node._on_request(request_string(make_request(
        u=2, v=2, width=width, height=height, patch_px=width)))
    below = node._result_publisher.messages[-1]
    assert below["ok"] is False
    assert below["error"] == "no valid depth at that pixel"

    values = [float("nan")] * (width * height)
    for i in range(9):
        values[i] = 4.0
    node._on_depth(depth_image(width, height, values))
    node._on_request(request_string(make_request(
        request_id="p-2.000-0", u=2, v=2, width=width, height=height,
        patch_px=width)))
    above = node._result_publisher.messages[-1]
    assert above["ok"] is True


def test_request_before_any_depth_image_fails(node):
    node._on_status(status_string("OK"))
    node._on_pose(odometry(0.0, 0.0, 0.0))
    node._on_request(request_string(make_request()))

    result = node._result_publisher.messages[0]
    assert result["ok"] is False
    assert result["error"] == "no depth image yet"


def test_request_before_any_pose_fails(node):
    node._on_camera_info(camera_info(500.0, 500.0, 320.0, 240.0, 640, 480))
    node._on_depth(depth_image(640, 480, [4.0] * (640 * 480)))
    node._on_status(status_string("OK"))
    node._on_request(request_string(make_request()))

    result = node._result_publisher.messages[0]
    assert result["ok"] is False
    assert result["error"] == "no rover pose yet"


def test_localisation_not_ok_fails_when_required(node):
    ready(node)
    node._on_status(status_string("SEARCHING"))
    node._on_request(request_string(make_request()))

    result = node._result_publisher.messages[0]
    assert result["ok"] is False
    assert result["error"] == "localisation is not OK"


def test_localisation_not_ok_is_ignored_when_not_required(node):
    ready(node)
    node._on_status(status_string("SEARCHING"))
    node.set_parameters([rclpy.parameter.Parameter(
        "require_localisation_ok", rclpy.Parameter.Type.BOOL, False)])
    node._on_request(request_string(make_request()))

    result = node._result_publisher.messages[0]
    assert result["ok"] is True


def test_pixel_outside_the_image_fails(node):
    ready(node)
    node._on_request(request_string(make_request(u=-5, v=240)))
    result = node._result_publisher.messages[0]
    assert result["ok"] is False
    assert result["error"] == "pixel is outside the image"


def test_unsupported_encoding_names_it(node):
    node._on_camera_info(camera_info(500.0, 500.0, 320.0, 240.0, 640, 480))
    node._on_depth(depth_image(640, 480, [4.0] * (640 * 480), encoding="mono16"))
    node._on_status(status_string("OK"))
    node._on_pose(odometry(0.0, 0.0, 0.0))
    node._on_request(request_string(make_request()))

    result = node._result_publisher.messages[0]
    assert result["ok"] is False
    assert result["error"] == "unsupported depth encoding 'mono16'"


@pytest.mark.parametrize("payload", [
    "not json at all",
    "[1, 2, 3]",
    json.dumps({"request_id": "p-1.000-0", "label": "51"}),
])
def test_garbage_requests_publish_nothing_and_do_not_raise(node, payload):
    ready(node)
    msg = String()
    msg.data = payload
    node._on_request(msg)

    assert node._result_publisher.messages == []
    # The node is still alive and answers a well-formed request afterwards.
    node._on_request(request_string(make_request()))
    assert len(node._result_publisher.messages) == 1


def test_two_requests_in_a_row_each_get_their_own_result(node):
    ready(node)
    node._on_request(request_string(make_request(request_id="p-1.000-0")))
    node._on_request(request_string(make_request(request_id="p-1.000-1")))

    ids = [m["request_id"] for m in node._result_publisher.messages]
    assert ids == ["p-1.000-0", "p-1.000-1"]
    assert all(m["ok"] for m in node._result_publisher.messages)
