"""glare_watch's ROS plumbing: rate limiting, encoding handling and the
change-or-heartbeat publish rule. The glare arithmetic itself (saturated
fractions, which side, the margin) is pure numpy and already covered by
test_glare.py; this file only exercises what a fake clock and a fake
publisher can reach that a pure-function test cannot.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_glare_watch.py -q'
"""
import json

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Image

from navi_autonomy.glare_watch import HEARTBEAT_S, GlareWatch


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(msg)


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def node(ros, clock):
    node = GlareWatch(clock=clock)
    node._glare_pub = Recorder()
    yield node
    node.destroy_node()


def image_msg(encoding, width, height, array):
    """A sensor_msgs/Image with no row padding (step == width * channels),
    the shape the live ZED wrapper actually publishes at 640x360 bgra8."""
    msg = Image()
    msg.encoding = encoding
    msg.width = width
    msg.height = height
    channels = array.shape[-1] if array.ndim == 3 else 1
    msg.step = width * channels
    msg.data = array.astype(np.uint8).tobytes()
    return msg


def bgra_frame(width=10, height=10, left=0, right=0):
    """A bgra8 frame, opaque, saturated to `left`/`right` on each half - the
    same shape saturated_fractions splits (width // 2 on both sides)."""
    array = np.zeros((height, width, 4), dtype=np.uint8)
    array[..., 3] = 255
    half = width // 2
    array[:, :half, :3] = left
    array[:, half:, :3] = right
    return image_msg('bgra8', width, height, array)


def test_a_left_saturated_frame_publishes_left(node):
    node._on_image(bgra_frame(left=255, right=0))

    assert len(node._glare_pub.messages) == 1
    payload = json.loads(node._glare_pub.messages[0].data)
    assert payload["side"] == "left"


def test_an_unsaturated_frame_publishes_none(node):
    node._on_image(bgra_frame(left=0, right=0))

    assert len(node._glare_pub.messages) == 1
    payload = json.loads(node._glare_pub.messages[0].data)
    assert payload["side"] == "none"


def test_an_undecodable_encoding_is_ignored_and_warns_only_once(node):
    fake_logger = FakeLogger()
    node.get_logger = lambda: fake_logger

    bad = Image()
    bad.encoding = "yuv422"
    bad.width, bad.height, bad.step = 10, 10, 20
    bad.data = bytes(200)

    node._on_image(bad)
    node._on_image(bad)

    assert node._glare_pub.messages == []
    assert len(fake_logger.warnings) == 1


def test_the_rate_limit_actually_drops_frames(node, clock):
    node._on_image(bgra_frame(left=255, right=0))
    assert len(node._glare_pub.messages) == 1

    # max_rate_hz defaults to 2.0 Hz, a 0.5 s minimum period - 0.1 s later is
    # well inside it, so this (oppositely glaring) frame must be dropped
    # before its glare side is ever computed.
    clock.t = 0.1
    node._on_image(bgra_frame(left=0, right=255))

    assert len(node._glare_pub.messages) == 1
    payload = json.loads(node._glare_pub.messages[0].data)
    assert payload["side"] == "left"       # the dropped frame never counted


def test_an_unchanged_verdict_is_not_republished_before_the_heartbeat_interval(node, clock):
    node._on_image(bgra_frame(left=0, right=0))
    assert len(node._glare_pub.messages) == 1

    # Past the rate limit's minimum period, but short of HEARTBEAT_S: the
    # verdict has not changed, so this must not republish.
    clock.t = HEARTBEAT_S - 0.6
    node._on_image(bgra_frame(left=0, right=0))
    assert len(node._glare_pub.messages) == 1

    # Now past HEARTBEAT_S since the one and only publish: the heartbeat
    # fires even though the verdict is still unchanged.
    clock.t = HEARTBEAT_S + 0.6
    node._on_image(bgra_frame(left=0, right=0))
    assert len(node._glare_pub.messages) == 2
