import json
import os

import pytest
import rclpy
from std_msgs.msg import String

from navi_teleop.video_request import VideoRequest
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
    records whether the node ever asked it to stop."""

    def __init__(self) -> None:
        self.returncode = None
        self.terminated = False

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


class FakeLauncher:
    """Records every argv it was asked to launch and hands back one shared
    FakeProcess, so a test can assert on the call and drive the process's
    lifecycle (alive, dies, gets terminated)."""

    def __init__(self) -> None:
        self.calls: list[list[str]] = []
        self.process = FakeProcess()

    def __call__(self, argv, **kwargs):
        self.calls.append(argv)
        return self.process


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def sender():
    launcher = FakeLauncher()
    node = VideoSender(launcher=launcher)
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
