"""Node-level tests for the stage-3 ArUco anchor phase (§3.5). `_detect` is
the only method that touches cv2, and every test here replaces it with a
stub, so nothing below needs OpenCV installed - matching
test_localization_status.py's pattern of feeding messages straight into
callbacks with recorders standing in for the real publishers."""

import json
import math

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from navi_localization.landmark_geometry import Intrinsics, landmark_point_in_map
from navi_localization.pose_composition import Transform
from navi_localization.site_anchor import SiteAnchor

WIDTH, HEIGHT = 100, 100
FX = FY = 500.0
CX = CY = 50.0
IDENTITY = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


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
    n = SiteAnchor()
    n._sightings_publisher = Recorder()
    yield n
    n.destroy_node()


def camera_info_msg():
    msg = CameraInfo()
    msg.width = WIDTH
    msg.height = HEIGHT
    msg.k[0] = FX
    msg.k[4] = FY
    msg.k[2] = CX
    msg.k[5] = CY
    return msg


def depth_image_msg(fill_value=4.0, nan_patch_center=None, patch_radius=3):
    arr = np.full((HEIGHT, WIDTH), fill_value, dtype=np.float32)
    if nan_patch_center is not None:
        ui, vi = nan_patch_center
        u0, u1 = max(0, ui - patch_radius), min(WIDTH, ui + patch_radius + 1)
        v0, v1 = max(0, vi - patch_radius), min(HEIGHT, vi + patch_radius + 1)
        arr[v0:v1, u0:u1] = float('nan')
    msg = Image()
    msg.height = HEIGHT
    msg.width = WIDTH
    msg.encoding = '32FC1'
    msg.step = WIDTH * 4
    msg.data = arr.tobytes()
    return msg


def pose_msg(x=0.0, y=0.0, z=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0
    return msg


def command(action):
    msg = String()
    msg.data = json.dumps({"action": action})
    return msg


def prepared(node, fill_value=4.0, nan_patch_center=None):
    """Feeds camera info, a constant (or partly-NaN) depth image, and an
    identity pose into the node, so `_process_detection` has everything it
    needs. A fake `now` clock the test fully controls stands in for
    `get_clock()`, advancing exactly one `detect_interval_s` per tick so the
    rate limiter never rejects a tick."""
    node._on_camera_info(camera_info_msg())
    node._on_depth(depth_image_msg(fill_value, nan_patch_center))
    node._on_pose(pose_msg())
    t = [1000.0]
    node._now = lambda: t[0]
    return t


def tick(node, t, detections):
    t[0] += node._detect_interval_s
    node._detect = lambda msg: detections
    node._on_image(object())


def last_report(node):
    # The wire cadence (§3.5) is 1 Hz while running plus once per phase
    # transition, decoupled from how often detections are processed - so a
    # test that just ticked the detector needs to force the same periodic
    # publish real time would eventually deliver, the way
    # test_localization_status.py calls `node._tick()` to force a status
    # publish rather than waiting on a real timer.
    node._on_report_timer()
    return json.loads(node._sightings_publisher.messages[-1].data)


def expected_point(depth_m, offset_m=0.125, u=CX, v=CY):
    intr = Intrinsics(FX, FY, CX, CY, WIDTH, HEIGHT)
    return landmark_point_in_map(u, v, depth_m, intr, IDENTITY, offset_m=offset_m)


def test_start_moves_to_running_and_publishes_immediately(node):
    node._on_command(command("start"))

    assert node._phase == "running"
    assert len(node._sightings_publisher.messages) == 1
    assert last_report(node)["phase"] == "running"


def test_sixty_ticks_of_one_marker_give_n_60_good_quality_and_the_shared_arithmetic(node):
    t = prepared(node, fill_value=4.0)
    node._on_command(command("start"))

    for _ in range(60):
        tick(node, t, [("51", CX, CY)])

    report = last_report(node)
    assert len(report["sightings"]) == 1
    sighting = report["sightings"][0]
    assert sighting["id"] == "51"
    assert sighting["n"] == 60
    assert sighting["quality"] == "good"
    expected = expected_point(4.0)
    assert sighting["x"] == pytest.approx(expected[0], abs=1e-6)
    assert sighting["y"] == pytest.approx(expected[1], abs=1e-6)
    assert sighting["z"] == pytest.approx(expected[2], abs=1e-6)


def test_thirty_ticks_is_weak(node):
    t = prepared(node)
    node._on_command(command("start"))

    for _ in range(30):
        tick(node, t, [("51", CX, CY)])

    assert last_report(node)["sightings"][0]["quality"] == "weak"


def test_scattered_detections_are_noisy_but_the_median_stays_near_truth(node):
    node._on_camera_info(camera_info_msg())
    node._on_pose(pose_msg())
    t = [1000.0]
    node._now = lambda: t[0]
    node._on_command(command("start"))

    for i in range(60):
        depth = 3.8 if i % 2 == 0 else 4.2
        node._on_depth(depth_image_msg(fill_value=depth))
        tick(node, t, [("51", CX, CY)])

    sighting = last_report(node)["sightings"][0]
    assert sighting["quality"] == "noisy"
    expected = expected_point(4.0)
    assert sighting["x"] == pytest.approx(expected[0], abs=1e-6)


def test_two_markers_give_two_string_id_sightings(node):
    t = prepared(node)
    node._on_command(command("start"))

    tick(node, t, [("51", 30, 30), ("52", 70, 70)])

    report = last_report(node)
    ids = sorted(s["id"] for s in report["sightings"])
    assert ids == ["51", "52"]
    assert all(isinstance(i, str) for i in ids)


def test_stop_freezes_the_phase_and_accumulation(node):
    t = prepared(node)
    node._on_command(command("start"))
    for _ in range(10):
        tick(node, t, [("51", CX, CY)])
    node._on_command(command("stop"))
    n_at_stop = last_report(node)["sightings"][0]["n"]

    tick(node, t, [("51", CX, CY)])

    assert node._phase == "stopped"
    assert last_report(node)["sightings"][0]["n"] == n_at_stop


def test_reset_empties_the_accumulator(node):
    t = prepared(node)
    node._on_command(command("start"))
    for _ in range(10):
        tick(node, t, [("51", CX, CY)])

    node._on_command(command("reset"))

    assert last_report(node)["sightings"] == []


def test_a_detection_with_no_valid_depth_is_skipped_not_guessed(node):
    t = prepared(node, nan_patch_center=(int(CX), int(CY)))
    node._on_command(command("start"))

    tick(node, t, [("51", CX, CY)])

    assert last_report(node)["sightings"] == []


def test_max_samples_caps_the_ring_buffer(node):
    t = prepared(node)
    node._on_command(command("start"))

    for _ in range(node._max_samples + 20):
        tick(node, t, [("51", CX, CY)])

    assert last_report(node)["sightings"][0]["n"] == node._max_samples


def test_unknown_dictionary_leaves_detector_ok_false_but_reports_keep_coming(node, monkeypatch):
    monkeypatch.setattr(node, "_dictionary_name", "DICT_NOT_A_REAL_DICTIONARY")
    node._setup_detector()

    assert node._detector_ok is False
    assert "DICT_NOT_A_REAL_DICTIONARY" in node._detector_error

    node._on_command(command("start"))
    report = last_report(node)
    assert report["detector_ok"] is False
    assert report["error"] == node._detector_error


def test_no_motion_publisher_exists_after_start_stop_reset_and_a_hundred_ticks(node):
    t = prepared(node)
    node._on_command(command("start"))
    for _ in range(100):
        tick(node, t, [("51", CX, CY)])
    node._on_command(command("stop"))
    node._on_command(command("reset"))

    topics = {p.topic_name for p in node.publishers}
    forbidden = {"/rover_twist", "/manual_twist", "/autonomy_twist",
                 "/mode_request", "/drive_command"}
    assert topics.isdisjoint(forbidden)
    assert "/site/landmark_sightings" in topics


def test_garbage_on_the_command_topic_is_ignored(node):
    node._on_command(command("start"))
    assert node._phase == "running"

    bad = String()
    bad.data = "not json at all"
    node._on_command(bad)
    assert node._phase == "running"

    node._on_command(command("sideways"))
    assert node._phase == "running"


# --- the colour-image decode `_detect` feeds to cv2 ---------------------
#
# `_detect` itself needs OpenCV, but the buffer arithmetic in front of it
# does not - and that arithmetic is where a review found the whole of
# stage 3 broken: a bgra8 frame was reshaped to (height, step // channels),
# which is a numpy size error on every multi-channel image, so detectMarkers
# was never reached at all. These tests own that arithmetic directly.


def colour_image_msg(encoding, width, height, pad_bytes=0, fill=None):
    channels = {'mono8': 1, 'bgr8': 3, 'bgra8': 4, 'rgba8': 4}[encoding]
    step = width * channels + pad_bytes
    rows = []
    for row in range(height):
        body = np.arange(row * width * channels,
                          (row + 1) * width * channels, dtype=np.uint8)
        if fill is not None:
            body = np.full(width * channels, fill, dtype=np.uint8)
        rows.append(np.concatenate([body, np.zeros(pad_bytes, dtype=np.uint8)]))
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    msg.step = step
    msg.data = np.concatenate(rows).tobytes()
    return msg


@pytest.mark.parametrize("encoding,channels",
                         [("mono8", 1), ("bgr8", 3), ("bgra8", 4), ("rgba8", 4)])
def test_image_to_array_shapes_every_supported_encoding(encoding, channels):
    from navi_localization.site_anchor import image_to_array

    msg = colour_image_msg(encoding, 7, 5)
    arr, found = image_to_array(msg)
    assert found == encoding
    assert arr.shape == (5, 7, channels)
    # Row 2, pixel 3, first channel: the byte the header says it is.
    assert arr[2, 3, 0] == np.uint8((2 * 7 * channels) + 3 * channels)


def test_image_to_array_honours_a_padded_row_stride():
    from navi_localization.site_anchor import image_to_array

    msg = colour_image_msg("bgr8", 4, 3, pad_bytes=5)
    arr, _ = image_to_array(msg)
    assert arr.shape == (3, 4, 3)
    # With the padding mistaken for picture, row 1 would start 5 bytes early.
    assert arr[1, 0, 0] == np.uint8(1 * 4 * 3)


def test_image_to_array_refuses_an_encoding_it_cannot_convert():
    from navi_localization.site_anchor import image_to_array

    msg = colour_image_msg("bgr8", 4, 3)
    msg.encoding = "16UC1"
    assert image_to_array(msg) is None


def test_image_to_array_refuses_a_truncated_buffer():
    from navi_localization.site_anchor import image_to_array

    msg = colour_image_msg("bgra8", 4, 3)
    msg.data = msg.data[:-8]
    assert image_to_array(msg) is None


def test_detect_returns_nothing_without_a_detector_rather_than_raising(node):
    node._detector_ok = False
    node._aruco_dict = None
    assert node._detect(colour_image_msg("bgra8", 8, 6)) == []
