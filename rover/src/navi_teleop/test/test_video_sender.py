import json
import os

import pytest
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from navi_teleop.video_request import DEFAULT_PORT, VideoRequest
from navi_teleop.video_sender import VideoSender, build_pipeline

REQUEST = VideoRequest(enable=True, host="192.168.178.101", port=5600,
                       width=1344, height=376, fps=30, bitrate_kbps=800,
                       device="/dev/video0")


def test_pipeline_starts_with_gst_launch_and_the_requested_device():
    argv = build_pipeline(REQUEST)

    assert argv[0] == "gst-launch-1.0"
    assert "device=/dev/video0" in argv


def test_pipeline_requests_the_capture_caps_from_the_request():
    argv = build_pipeline(REQUEST)

    assert "video/x-raw,width=1344,height=376,framerate=30/1" in argv


def test_pipeline_crops_away_the_right_eye():
    # The ZED delivers side-by-side stereo; cropping half the width off the
    # right leaves the left eye at its native size.
    argv = build_pipeline(REQUEST)

    assert "videocrop" in argv
    assert "right=672" in argv


def test_pipeline_carries_bitrate_and_sends_to_the_requesting_host():
    argv = build_pipeline(REQUEST)

    assert "bitrate=800" in argv
    assert "host=192.168.178.101" in argv
    assert "port=5600" in argv


def test_pipeline_bounds_recovery_after_packet_loss():
    # A keyframe at least every second, and repeated SPS/PPS, are what let a
    # receiver recover from loss without the stream being restarted.
    argv = build_pipeline(REQUEST)

    assert "key-int-max=30" in argv
    assert "config-interval=1" in argv


def test_pipeline_crop_follows_the_capture_width():
    request = VideoRequest(enable=True, host="10.0.0.5", port=5600, width=2560,
                           height=720, fps=30, bitrate_kbps=2000,
                           device="/dev/video0")

    assert "right=1280" in build_pipeline(request)


def test_pipeline_stages_run_capture_to_sink_in_order():
    # Membership alone lets videocrop and videoconvert swap, or the caps
    # move after the crop, and every assertion above would still pass while
    # the pipeline sent nothing real.
    argv = build_pipeline(REQUEST)

    caps = argv.index("video/x-raw,width=1344,height=376,framerate=30/1")
    crop = argv.index("videocrop")
    convert = argv.index("videoconvert")
    encoder = argv.index("x264enc")
    pay = argv.index("rtph264pay")
    sink = argv.index("udpsink")

    assert caps < crop < convert < encoder < pay < sink


class FakeProcess:
    """Stands in for subprocess.Popen: alive until told otherwise, and
    records whether the node ever asked it to stop. Also stands in for its
    own stdin, so the zed_topic path's `process.stdin.write(...)` and the
    shutdown path's `process.stdin.close()` land on this same fake."""

    def __init__(self) -> None:
        self.returncode = None
        self.terminated = False
        self.stdin = self
        self.written: list[bytes] = []

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def kill(self):
        self.terminated = True
        self.returncode = -9

    def wait(self, timeout=None):
        return self.returncode

    def write(self, data):
        self.written.append(bytes(data))

    def flush(self):
        pass

    def close(self):
        pass


class FakeLauncher:
    """Records every argv (and kwargs) it was asked to launch and hands back
    one shared FakeProcess, so a test can assert on the call and drive the
    process's lifecycle (alive, dies, gets terminated)."""

    def __init__(self) -> None:
        self.calls: list[tuple[list[str], dict]] = []
        self.process = FakeProcess()

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        return self.process


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def sender():
    # These pre-date the zed_topic source and describe the node's generic
    # request/lifecycle handling (enable, disable, malformed requests,
    # process death, stderr cleanup) - none of it is specific to how a
    # stream is produced. They exercise it through the v4l2 path, which
    # still starts synchronously from _on_request like they expect; the
    # zed_topic default now waits for a first frame before that happens.
    launcher = FakeLauncher()
    node = VideoSender(launcher=launcher, parameter_overrides=[
        rclpy.parameter.Parameter("source", value="v4l2")])
    yield node, launcher
    if node._stderr_path and os.path.exists(node._stderr_path):
        os.remove(node._stderr_path)
    node.destroy_node()


def _request(payload: dict) -> String:
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def test_enable_request_starts_streaming_and_calls_launcher_once(sender):
    node, launcher = sender

    node._on_request(_request({"enable": True, "host": "127.0.0.1", "port": 5600}))

    assert node._state == 'streaming'
    assert len(launcher.calls) == 1


def test_disable_request_stops_the_stream_and_terminates_the_process(sender):
    node, launcher = sender
    node._on_request(_request({"enable": True, "host": "127.0.0.1", "port": 5600}))

    node._on_request(_request({"enable": False}))

    assert node._state == 'stopped'
    assert launcher.process.terminated
    assert node._process is None


def test_malformed_request_while_stopped_fails_without_ever_launching(sender):
    node, launcher = sender

    node._on_request(_request({"enable": True, "host": "127.0.0.1", "bitrate_kbps": 99000}))

    assert node._state == 'failed'
    assert "bitrate_kbps" in node._detail
    assert launcher.calls == []


def test_malformed_request_while_streaming_leaves_the_stream_running(sender):
    # Regression test: a refusal must not disturb a healthy stream, and
    # must not lie about it by reporting failed while video keeps flowing.
    node, launcher = sender
    node._on_request(_request({"enable": True, "host": "127.0.0.1", "port": 5600}))

    node._on_request(_request({"enable": True, "host": "127.0.0.1", "bitrate_kbps": 99000}))

    assert node._state == 'streaming'
    assert len(launcher.calls) == 1
    assert launcher.process.terminated is False
    assert node._process is launcher.process


def test_process_death_while_streaming_reports_failed_with_stderr_tail(sender):
    # Regression test: stderr goes to a temp file, not a pipe that only
    # gets read after death is detected, so this tail must still be
    # readable once the process is gone.
    node, launcher = sender
    node._on_request(_request({"enable": True, "host": "127.0.0.1", "port": 5600}))
    stderr_path = node._stderr_path
    with open(stderr_path, 'w') as f:
        f.write("warning: dropped frame\nERROR: v4l2src0: Internal data flow error\n")
    launcher.process.returncode = 1

    node._publish_status_tick()

    assert node._state == 'failed'
    assert "Internal data flow error" in node._detail
    # The tail must be read before the file is removed, not instead of it -
    # a stderr temp file per death should not outlive the death it explains.
    assert not os.path.exists(stderr_path)
    assert node._stderr_path is None


def test_disable_removes_the_stderr_temp_file(sender):
    # A stream that ends leaves nothing behind: every enable creates a new
    # stderr temp file, so an operator toggling video on and off across a
    # session must not accumulate one orphaned file per enable in /tmp.
    node, launcher = sender
    node._on_request(_request({"enable": True, "host": "127.0.0.1", "port": 5600}))
    stderr_path = node._stderr_path
    assert os.path.exists(stderr_path)

    node._on_request(_request({"enable": False}))

    assert not os.path.exists(stderr_path)
    assert node._stderr_path is None


def make_node(source="zed_topic"):
    launcher = FakeLauncher()
    node = VideoSender(launcher=launcher, parameter_overrides=[
        rclpy.parameter.Parameter("source", value=source)])
    return node, launcher


def request(width=640, height=360):
    msg = String()
    msg.data = json.dumps({"enable": True, "host": "192.168.178.101", "port": DEFAULT_PORT,
                           "width": width, "height": height, "fps": 15, "bitrate_kbps": 800})
    return msg


def image(width=640, height=360, encoding="bgra8"):
    msg = Image()
    msg.width, msg.height, msg.encoding = width, height, encoding
    msg.step = width * 4
    msg.data = bytes(width * height * 4)
    return msg


def test_zed_topic_source_starts_a_stdin_pipeline_on_the_first_frame():
    node, launcher = make_node()
    try:
        node._on_request(request())
        assert launcher.calls == []                # nothing to encode yet
        assert node._state == "starting"
        node._on_image(image())
        argv, kwargs = launcher.calls[0]
        assert "fdsrc" in argv and "format=bgra" in argv
        assert kwargs["stdin"] is not None
        assert node._state == "streaming"
        assert node._detail == f"192.168.178.101:{DEFAULT_PORT} 640x360"
        assert launcher.process.written == [bytes(640 * 360 * 4)]
    finally:
        node.destroy_node()


def test_malformed_request_while_pending_leaves_the_pending_request_alone():
    # Regression test: a malformed request must not disturb a request still
    # waiting on its first frame any more than it disturbs an already
    # healthy stream - otherwise /video_status briefly lies 'failed' while
    # the pending request is still live, and a matching frame right after
    # would start streaming anyway, contradicting the status it just sent.
    node, launcher = make_node()
    try:
        node._on_request(request())
        assert node._state == "starting"

        bad = String()
        bad.data = json.dumps({"enable": True, "host": "127.0.0.1", "bitrate_kbps": 99000})
        node._on_request(bad)

        assert node._state == "starting"
        assert node._pending is not None
        assert launcher.calls == []

        node._on_image(image())
        assert node._state == "streaming"
    finally:
        node.destroy_node()


def test_zed_topic_source_adopts_the_image_size_and_says_so():
    # The ground station asks for 1344x376 (the UVC capture size) while the
    # wrapper publishes 640x360. The wrapper owns the camera, so its size
    # wins and the request's geometry is advisory: the pipeline is built for
    # what actually arrives, and /video_status carries that geometry so the
    # operator sees what they got rather than what they asked for.
    node, launcher = make_node()
    try:
        node._on_request(request(width=1344, height=376))
        node._on_image(image(640, 360))

        argv, _ = launcher.calls[0]
        assert "width=640" in argv and "height=360" in argv
        assert f"blocksize={640 * 360 * 4}" in argv
        assert node._state == "streaming"
        assert "640x360" in node._detail
        assert launcher.process.written == [bytes(640 * 360 * 4)]
    finally:
        node.destroy_node()


def test_zed_topic_source_refuses_an_encoding_it_cannot_build_a_pipeline_for():
    node, launcher = make_node()
    try:
        node._on_request(request())
        bad = image()
        bad.encoding = "mono16"
        node._on_image(bad)

        assert launcher.calls == []
        assert node._state == "failed"
        assert "mono16" in node._detail
    finally:
        node.destroy_node()


def image_topics(node):
    return [s.topic_name for s in node.subscriptions
            if s.topic_name == node._image_topic()]


def test_no_image_subscription_exists_until_video_is_asked_for():
    # The topic carries ~27 MB/s of bgra8 and every message on a subscribed
    # topic is deserialised whether anyone wants it or not. In
    # semi-autonomous mode video is forbidden, so that cost must not be
    # paid just because the node is running.
    node, _ = make_node()
    try:
        assert image_topics(node) == []
        assert node._image_subscription is None
    finally:
        node.destroy_node()


def test_a_zed_topic_request_subscribes_and_stopping_unsubscribes():
    node, _ = make_node()
    try:
        node._on_request(request())
        assert len(image_topics(node)) == 1
        assert node._image_subscription is not None

        stop = String()
        stop.data = json.dumps({"enable": False})
        node._on_request(stop)

        assert image_topics(node) == []
        assert node._image_subscription is None
    finally:
        node.destroy_node()


def test_a_refused_frame_leaves_no_subscription_behind():
    # A request that cannot be served is not a request that is still
    # wanted: leaving the topic subscribed would go on deserialising
    # ~27 MB/s into a node that is reporting failed.
    node, launcher = make_node()
    try:
        node._on_request(request())
        bad = image()
        bad.encoding = "mono16"
        node._on_image(bad)

        assert node._state == "failed"
        assert node._image_subscription is None
        assert image_topics(node) == []
    finally:
        node.destroy_node()


def test_a_dead_pipeline_leaves_no_subscription_behind():
    node, launcher = make_node()
    try:
        node._on_request(request())
        node._on_image(image())
        assert node._image_subscription is not None
        launcher.process.returncode = 1

        node._publish_status_tick()

        assert node._state == "failed"
        assert node._image_subscription is None
    finally:
        if node._stderr_path and os.path.exists(node._stderr_path):
            os.remove(node._stderr_path)
        node.destroy_node()


def test_a_second_request_does_not_stack_up_subscriptions():
    node, _ = make_node()
    try:
        node._on_request(request())
        node._on_request(request())
        assert len(image_topics(node)) == 1
    finally:
        node.destroy_node()


def test_the_v4l2_source_never_subscribes_to_images():
    node, _ = make_node(source="v4l2")
    try:
        node._on_request(request(width=1344, height=376))
        assert node._image_subscription is None
        assert image_topics(node) == []
    finally:
        node.destroy_node()


def test_zed_topic_source_ignores_frames_when_not_streaming():
    node, launcher = make_node()
    try:
        node._on_image(image())
        assert launcher.calls == []
        assert node._state == "stopped"
    finally:
        node.destroy_node()


def test_v4l2_source_keeps_the_old_capture_pipeline():
    node, launcher = make_node(source="v4l2")
    try:
        node._on_request(request(width=1344, height=376))
        argv, _ = launcher.calls[0]
        assert "v4l2src" in argv
        assert node._state == "streaming"
    finally:
        node.destroy_node()


def test_stopping_terminates_the_stdin_pipeline():
    node, launcher = make_node()
    try:
        node._on_request(request())
        node._on_image(image())
        stop = String()
        stop.data = json.dumps({"enable": False})
        node._on_request(stop)
        assert launcher.process.terminated
        assert node._state == "stopped"
    finally:
        node.destroy_node()


def test_the_v4l2_streaming_detail_names_the_frame_size_like_the_zed_path(sender):
    # The ground station re-pins its decoder from the 'streaming' detail;
    # a v4l2 rover after a zed_topic session (640x360) must reset it to
    # its own size or the pipeline never negotiates.
    node, _ = sender
    node._on_request(_request({"enable": True, "host": "127.0.0.1", "port": 5600,
                               "width": 1344, "height": 376}))
    assert node._state == 'streaming'
    assert node._detail.endswith(" 672x376"), node._detail
