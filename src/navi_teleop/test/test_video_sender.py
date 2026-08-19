from navi_teleop.video_request import VideoRequest
from navi_teleop.video_sender import build_pipeline

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
