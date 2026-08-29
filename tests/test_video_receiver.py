import io
import os
import shutil
import subprocess
import threading
import time

import pytest

from ground_station import video_receiver
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


def test_stop_removes_the_stderr_temp_file():
    # Critical 1: stderr is redirected to a temp file (not subprocess.PIPE,
    # which nothing ever drains and which fills and stalls the pipeline).
    # The node itself must remove that file when the stream ends - not a
    # test fixture cleaning up after it.
    receiver, _ = make_receiver()
    receiver.start()
    stderr_path = receiver._stderr_path
    assert stderr_path is not None
    assert os.path.exists(stderr_path)

    receiver.stop()

    assert not os.path.exists(stderr_path)
    assert receiver._stderr_path is None


def test_start_after_a_stop_does_not_reuse_or_leak_the_previous_stderr_file():
    receiver, _ = make_receiver()
    receiver.start()
    first_path = receiver._stderr_path
    receiver.stop()

    receiver.start()

    assert receiver._stderr_path is not None
    assert receiver._stderr_path != first_path
    assert os.path.exists(receiver._stderr_path)
    receiver.stop()


def test_start_removes_the_stderr_temp_file_when_the_launcher_raises(monkeypatch):
    # If the launcher raises (e.g. FileNotFoundError for a missing
    # gst-launch-1.0 - the exact scenario the panel's own guard comment
    # names), _process never gets set, so stop()'s cleanup path is never
    # reached. The temp file must not be left behind, and the exception
    # must still propagate so the panel's guard can catch it.
    created_paths = []
    real_named_temp_file = video_receiver.tempfile.NamedTemporaryFile

    def spying_named_temp_file(*args, **kwargs):
        temp_file = real_named_temp_file(*args, **kwargs)
        created_paths.append(temp_file.name)
        return temp_file

    monkeypatch.setattr(video_receiver.tempfile, "NamedTemporaryFile",
                        spying_named_temp_file)

    def raising_launcher(*_args, **_kwargs):
        raise FileNotFoundError("gst-launch-1.0 not found")

    receiver = VideoReceiver(port=5600, width=4, height=2,
                             launcher=raising_launcher)

    with pytest.raises(FileNotFoundError):
        receiver.start()

    assert receiver._stderr_path is None
    assert receiver._process is None
    assert len(created_paths) == 1
    assert not os.path.exists(created_paths[0])


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
        frames = []
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            frame = receiver.read_frame()
            if frame is not None:
                frames.append(frame)
            elif not receiver.is_running and frames:
                break
            else:
                time.sleep(0.01)
        # Not num_buffers: videotestsrc writes all five faster than this loop
        # polls, and the receiver keeps only the newest frame by design. What
        # this proves is that real bytes crossed a real pipe and came back out
        # correctly framed.
        assert frames
        assert len(frames) <= num_buffers
        for frame in frames:
            assert len(frame) == frame_size
    finally:
        receiver.stop()


class PipeProcess:
    """A fake process whose stdout is a real OS pipe, so the 64 KB pipe
    capacity that the reader has to cope with is real rather than simulated
    by a BytesIO that never fills."""

    def __init__(self):
        read_fd, self.write_fd = os.pipe()
        self.stdout = os.fdopen(read_fd, "rb", buffering=0)
        self._returncode = None

    def poll(self):
        return self._returncode

    def terminate(self):
        self._returncode = 0
        try:
            os.close(self.write_fd)
        except OSError:
            pass

    def wait(self, timeout=None):
        self._returncode = 0
        return 0

    def kill(self):
        self._returncode = -9


def wait_for_frame(receiver, timeout=5.0):
    """Frames are produced by the receiver's own reader thread, so a test
    that wants one has to wait for it rather than assume it has arrived."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = receiver.read_frame()
        if frame is not None:
            return frame
        time.sleep(0.005)
    return None


def test_the_pipe_is_drained_even_while_nobody_calls_read_frame():
    """The regression test for the stall that made the stream lag.

    A 672x376 RGB frame is 758,016 bytes - about twelve times a Linux
    pipe's 64 KB capacity. When the pipe was only read from the GUI's
    33 ms timer, one tick could consume at most one pipe-full, so the
    decoder spent nearly all its time blocked in write() and the backlog
    grew in the jitter buffer and the kernel's UDP socket buffer instead.
    Measured against the rover, that capped the panel at 2.2 fps while a
    greedy reader on the same pipeline got ~27 fps.

    So: the receiver must keep the pipe empty on its own schedule, and a
    writer must be able to push several frames through without a single
    read_frame() call.
    """
    width, height = 672, 376
    frame_size = width * height * 3
    process = PipeProcess()
    receiver = VideoReceiver(port=5600, width=width, height=height,
                             launcher=lambda *a, **k: process)
    receiver.start()
    try:
        payload = bytes(frame_size * 3)
        finished = []

        def writer():
            os.write(process.write_fd, payload)
            finished.append(True)

        thread = threading.Thread(target=writer, daemon=True)
        thread.start()
        thread.join(timeout=5.0)

        assert finished == [True], (
            "the writer is still blocked in write(): nothing is draining the "
            "pipe until the caller asks for a frame")
    finally:
        receiver.stop()


def test_read_frame_returns_the_newest_frame_and_drops_the_backlog():
    """Live video means the newest frame, not the oldest queued one. If the
    GUI is late, the frames it missed are stale by definition - handing them
    over would replay the backlog at 30 fps instead of catching up."""
    width, height = 4, 2
    frame_size = width * height * 3
    oldest = b"\x01" * frame_size
    middle = b"\x02" * frame_size
    newest = b"\x03" * frame_size
    receiver, _ = make_receiver(frames=oldest + middle + newest,
                                width=width, height=height)
    receiver.start()
    try:
        assert wait_for_frame(receiver) == newest
        assert receiver.read_frame() is None
    finally:
        receiver.stop()


def test_stop_joins_the_reader_thread():
    receiver, _ = make_receiver(frames=b"")
    receiver.start()

    receiver.stop()

    assert receiver._reader is None or not receiver._reader.is_alive()


def test_parse_geometry_reads_the_size_the_rover_reports():
    from ground_station.video_receiver import parse_geometry

    assert parse_geometry("192.168.178.101:5600 640x360") == (640, 360)
    assert parse_geometry("640x360") == (640, 360)


def test_parse_geometry_is_none_when_the_detail_carries_no_size():
    from ground_station.video_receiver import parse_geometry

    assert parse_geometry("") is None
    assert parse_geometry("192.168.178.101:5600") is None
    assert parse_geometry("not connected to rosbridge") is None
