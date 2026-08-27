import io
import shutil
import subprocess

import pytest

from ground_station.video_receiver import VideoReceiver, build_receive_pipeline


class FakeProcess:
    def __init__(self, frames: bytes):
        self.stdout = io.BytesIO(frames)
        self.stderr = io.BytesIO(b"")
        self.terminated = False
        self._returncode = None

    def poll(self):
        return self._returncode

    def terminate(self):
        self.terminated = True
        self._returncode = 0

    def wait(self, timeout=None):
        self._returncode = 0
        return 0

    def kill(self):
        self._returncode = -9


def make_receiver(frames=b"", width=4, height=2):
    process = FakeProcess(frames)
    receiver = VideoReceiver(port=5600, width=width, height=height,
                             launcher=lambda *a, **k: process)
    return receiver, process


def test_pipeline_listens_on_the_given_port():
    argv = build_receive_pipeline(5600, 672, 376)

    assert argv[0] == "gst-launch-1.0"
    assert "port=5600" in argv


def test_pipeline_declares_h264_rtp_caps():
    argv = build_receive_pipeline(5600, 672, 376)

    assert any("encoding-name=H264" in part for part in argv)
    assert "rtph264depay" in argv
    assert "avdec_h264" in argv


def test_pipeline_emits_raw_rgb_to_stdout():
    # The panel reads width * height * 3 bytes per frame off the pipe, so the
    # sink must be raw RGB on fd 1 and nothing else.
    argv = build_receive_pipeline(5600, 672, 376)

    caps_tokens = [part for part in argv if part.startswith("video/x-raw,format=RGB")]
    assert len(caps_tokens) == 1
    assert "width=672" in caps_tokens[0]
    assert "height=376" in caps_tokens[0]
    assert "fdsink" in argv
    assert "fd=1" in argv


def test_read_frame_returns_exactly_one_frame_of_bytes():
    frame = bytes(range(4 * 2 * 3))
    receiver, _ = make_receiver(frames=frame)
    receiver.start()

    assert receiver.read_frame() == frame


def test_read_frame_returns_none_when_no_full_frame_available():
    receiver, _ = make_receiver(frames=b"\x00\x01\x02")
    receiver.start()

    assert receiver.read_frame() is None


def test_read_frame_returns_none_before_start():
    receiver, _ = make_receiver(frames=b"")

    assert receiver.read_frame() is None


def test_stop_terminates_the_pipeline_process():
    receiver, process = make_receiver()
    receiver.start()

    receiver.stop()

    assert process.terminated
    assert receiver.is_running is False


def test_start_is_idempotent():
    receiver, _ = make_receiver()
    receiver.start()
    first = receiver._process

    receiver.start()

    assert receiver._process is first


def test_is_running_is_false_after_the_process_dies():
    receiver, process = make_receiver()
    receiver.start()
    process._returncode = 1

    assert receiver.is_running is False


@pytest.mark.skipif(shutil.which("gst-launch-1.0") is None,
                     reason="requires a real gst-launch-1.0 binary")
def test_read_frame_against_a_real_gstreamer_subprocess():
    """Codec-free loopback: this laptop's GStreamer is missing avdec_h264
    and x264enc (needs gstreamer1.0-libav / gstreamer1.0-plugins-ugly, both
    requiring sudo we don't have), so H.264 can't be exercised here. This
    proves everything else end to end against a *real* subprocess instead of
    FakeProcess: videotestsrc stands in for the decoded output, feeding raw
    RGB through a real pipe into the non-blocking read/buffer/frame-slicing
    logic. The H.264 decode leg itself is proven in Task 7 against the rover.
    """
    width, height, num_buffers = 32, 24, 5
    frame_size = width * height * 3
    argv = [
        "gst-launch-1.0", "-q",
        "videotestsrc", f"num-buffers={num_buffers}",
        "!", f"video/x-raw,width={width},height={height},format=RGB",
        "!", "videoconvert",
        "!", "fdsink", "fd=1",
    ]

    def real_launcher(_pipeline_argv, **kwargs):
        return subprocess.Popen(argv, **kwargs)

    receiver = VideoReceiver(port=5600, width=width, height=height,
                             launcher=real_launcher)
    receiver.start()
    try:
        import time
        frames = []
        deadline = 10.0
        start_time = time.monotonic()
        while len(frames) < num_buffers and time.monotonic() - start_time < deadline:
            frame = receiver.read_frame()
            if frame is not None:
                frames.append(frame)
            elif not receiver.is_running:
                break
            else:
                time.sleep(0.01)
        assert len(frames) == num_buffers
        for frame in frames:
            assert len(frame) == frame_size
    finally:
        receiver.stop()
