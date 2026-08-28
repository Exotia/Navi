# Ground Station Video Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream the rover's ZED 2i left camera to the ground station as live H.264 video, started and stopped from a panel in the UI.

**Architecture:** Two planes that share no transport. Control (an enable request and a status reply) rides the existing rosbridge websocket as JSON in `std_msgs/String`. Media is H.264 over RTP/UDP, sent by a GStreamer pipeline on the Orin straight to the laptop, never touching rosbridge. On the laptop the decode pipeline runs as a `gst-launch-1.0` subprocess writing raw RGB frames to stdout, which the panel reads and paints.

**Tech Stack:** ROS 2 Humble + rclpy (Orin), GStreamer 1.20 (both machines), PySide6 6.11 + roslibpy (laptop), pytest / pytest-qt.

**Spec:** `docs/superpowers/specs/2026-08-27-ground-station-video-design.md`

## Global Constraints

- The laptop has **no ROS 2** and never will. Everything laptop-side talks rosbridge via roslibpy, or plain sockets. Never import `rclpy` in `ground_station/`.
- The Orin package is `navi_teleop` at `~/navi/src/navi_teleop`, an **ament_python** package. It already contains `manual_twist_listener.py`. Do not add message definitions — that would force an `ament_cmake` package.
- Control messages are `std_msgs/String` carrying JSON. Topics: `/video_request` (ground station to rover), `/video_status` (rover to ground station).
- Capture defaults: `1344x376@30` from `/dev/video0`, cropped to a `672x376` left eye, `800` kbit/s.
- Request ceilings, enforced on the rover: `2560x720` maximum resolution, `4000` kbit/s maximum bitrate, `/dev/video0` the only allowed device.
- Default UDP media port: `5600`.
- Status values are exactly: `stopped`, `starting`, `streaming`, `failed`.
- The laptop needs `gstreamer1.0-libav` (`avdec_h264`) installed. This is a sudo prerequisite, not a task.
- Python 3.10 on both machines. Use `X | None` syntax, match the existing code style: module docstrings explaining *why*, no comments restating *what*.
- Rover-side commands run over `ssh star@a_navi`. Source `/opt/ros/humble/setup.bash` and `~/navi/install/local_setup.bash` first.

---

### Task 1: Video request validation on the rover

Pure logic, no ROS and no GStreamer, so it is testable anywhere and the node that uses it stays thin.

**Files:**
- Create: `~/navi/src/navi_teleop/navi_teleop/video_request.py` (on the Orin)
- Test: `~/navi/src/navi_teleop/test/test_video_request.py` (on the Orin)

**Interfaces:**
- Consumes: nothing.
- Produces: `parse_request(payload: str, max_width: int, max_height: int, max_bitrate_kbps: int, allowed_device: str) -> VideoRequest`, raising `InvalidRequest` (a `ValueError` subclass). `VideoRequest` is a frozen dataclass with fields `enable: bool`, `host: str`, `port: int`, `width: int`, `height: int`, `fps: int`, `bitrate_kbps: int`, `device: str`.

- [ ] **Step 1: Write the failing tests**

```python
# ~/navi/src/navi_teleop/test/test_video_request.py
import pytest

from navi_teleop.video_request import InvalidRequest, VideoRequest, parse_request

LIMITS = dict(max_width=2560, max_height=720, max_bitrate_kbps=4000,
              allowed_device="/dev/video0")


def test_parse_request_fills_defaults():
    request = parse_request('{"enable": true, "host": "192.168.178.101"}', **LIMITS)

    assert request == VideoRequest(enable=True, host="192.168.178.101", port=5600,
                                   width=1344, height=376, fps=30,
                                   bitrate_kbps=800, device="/dev/video0")


def test_parse_request_accepts_explicit_values():
    payload = ('{"enable": true, "host": "10.0.0.5", "port": 5601, "width": 2560, '
               '"height": 720, "fps": 15, "bitrate_kbps": 2500}')

    request = parse_request(payload, **LIMITS)

    assert (request.port, request.width, request.height) == (5601, 2560, 720)
    assert (request.fps, request.bitrate_kbps) == (15, 2500)


def test_parse_request_rejects_resolution_above_ceiling():
    payload = '{"enable": true, "host": "10.0.0.5", "width": 3840, "height": 1080}'

    with pytest.raises(InvalidRequest, match="width"):
        parse_request(payload, **LIMITS)


def test_parse_request_rejects_bitrate_above_ceiling():
    payload = '{"enable": true, "host": "10.0.0.5", "bitrate_kbps": 9000}'

    with pytest.raises(InvalidRequest, match="bitrate_kbps"):
        parse_request(payload, **LIMITS)


def test_parse_request_rejects_foreign_device():
    payload = '{"enable": true, "host": "10.0.0.5", "device": "/dev/video9"}'

    with pytest.raises(InvalidRequest, match="device"):
        parse_request(payload, **LIMITS)


def test_parse_request_rejects_malformed_json():
    with pytest.raises(InvalidRequest, match="JSON"):
        parse_request("{not json", **LIMITS)


def test_parse_request_requires_host_when_enabling():
    with pytest.raises(InvalidRequest, match="host"):
        parse_request('{"enable": true}', **LIMITS)


def test_parse_request_allows_disable_without_host():
    request = parse_request('{"enable": false}', **LIMITS)

    assert request.enable is False
    assert request.host == ""


def test_parse_request_rejects_port_outside_user_range():
    payload = '{"enable": true, "host": "10.0.0.5", "port": 80}'

    with pytest.raises(InvalidRequest, match="port"):
        parse_request(payload, **LIMITS)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `ssh star@a_navi 'cd ~/navi/src/navi_teleop && python3 -m pytest test/test_video_request.py -v'`
Expected: FAIL, `ModuleNotFoundError: No module named 'navi_teleop.video_request'`

- [ ] **Step 3: Write the implementation**

```python
# ~/navi/src/navi_teleop/navi_teleop/video_request.py
"""Parsing and bounds-checking for the ground station's video requests.

The request arrives as JSON in a std_msgs/String rather than a typed
message, because a custom .msg would force this pure-Python package into
an ament_cmake build for the sake of two messages. That trade puts the
burden of validation here: nothing downstream may assume a field exists,
has the right type, or is within range.

The ceilings are the rover's, not the ground station's. An operator can
ask for more than the link can carry, and the answer is a refusal rather
than a stream that saturates the WiFi and takes manual driving with it.
"""

import json
from dataclasses import dataclass

DEFAULT_PORT = 5600
DEFAULT_WIDTH = 1344
DEFAULT_HEIGHT = 376
DEFAULT_FPS = 30
DEFAULT_BITRATE_KBPS = 800


class InvalidRequest(ValueError):
    """Raised when a request is malformed or asks for more than allowed."""


@dataclass(frozen=True)
class VideoRequest:
    enable: bool
    host: str
    port: int
    width: int
    height: int
    fps: int
    bitrate_kbps: int
    device: str


def _int_field(data: dict, name: str, default: int) -> int:
    value = data.get(name, default)
    # bool is an int subclass in Python; accepting it here would let
    # {"port": true} through as port 1.
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidRequest(f"{name} must be an integer, got {value!r}")
    return value


def parse_request(payload: str, max_width: int, max_height: int,
                  max_bitrate_kbps: int, allowed_device: str) -> VideoRequest:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise InvalidRequest(f"payload is not valid JSON: {exc}") from exc

    if not isinstance(data, dict):
        raise InvalidRequest(f"payload must be a JSON object, got {type(data).__name__}")

    enable = data.get("enable", False)
    if not isinstance(enable, bool):
        raise InvalidRequest(f"enable must be a boolean, got {enable!r}")

    host = data.get("host", "")
    if not isinstance(host, str):
        raise InvalidRequest(f"host must be a string, got {host!r}")
    if enable and not host:
        raise InvalidRequest("host is required when enabling the stream")

    port = _int_field(data, "port", DEFAULT_PORT)
    if not 1024 <= port <= 65535:
        raise InvalidRequest(f"port must be between 1024 and 65535, got {port}")

    width = _int_field(data, "width", DEFAULT_WIDTH)
    height = _int_field(data, "height", DEFAULT_HEIGHT)
    if width <= 0 or width > max_width:
        raise InvalidRequest(f"width must be between 1 and {max_width}, got {width}")
    if height <= 0 or height > max_height:
        raise InvalidRequest(f"height must be between 1 and {max_height}, got {height}")

    fps = _int_field(data, "fps", DEFAULT_FPS)
    if not 1 <= fps <= 60:
        raise InvalidRequest(f"fps must be between 1 and 60, got {fps}")

    bitrate_kbps = _int_field(data, "bitrate_kbps", DEFAULT_BITRATE_KBPS)
    if bitrate_kbps <= 0 or bitrate_kbps > max_bitrate_kbps:
        raise InvalidRequest(
            f"bitrate_kbps must be between 1 and {max_bitrate_kbps}, got {bitrate_kbps}")

    device = data.get("device", allowed_device)
    if device != allowed_device:
        raise InvalidRequest(f"device must be {allowed_device}, got {device!r}")

    return VideoRequest(enable=enable, host=host, port=port, width=width,
                        height=height, fps=fps, bitrate_kbps=bitrate_kbps,
                        device=device)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `ssh star@a_navi 'cd ~/navi/src/navi_teleop && python3 -m pytest test/test_video_request.py -v'`
Expected: PASS, 9 tests.

- [ ] **Step 5: Commit**

The Orin workspace is not a git repository yet. Initialize it once, here, so rover-side work is tracked from this point on.

```bash
ssh star@a_navi 'cd ~/navi && \
  ([ -d .git ] || (git init -q && printf "build/\ninstall/\nlog/\n__pycache__/\n" > .gitignore)) && \
  git add .gitignore src && \
  git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
      commit -q -m "Add video request parsing and bounds checking"'
```

---

### Task 2: The `video_sender` node

**Files:**
- Create: `~/navi/src/navi_teleop/navi_teleop/video_sender.py` (on the Orin)
- Test: `~/navi/src/navi_teleop/test/test_video_sender.py` (on the Orin)
- Modify: `~/navi/src/navi_teleop/setup.py` — add the console script entry point

**Interfaces:**
- Consumes: `parse_request`, `InvalidRequest`, `VideoRequest` from Task 1.
- Produces: `build_pipeline(request: VideoRequest) -> list[str]` (the `gst-launch-1.0` argv), and the `video_sender` executable.

- [ ] **Step 1: Write the failing tests**

The pipeline is built by a pure function so it can be asserted on without a camera. The node itself is exercised through a fake process launcher.

```python
# ~/navi/src/navi_teleop/test/test_video_sender.py
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


def test_pipeline_crop_halves_an_odd_capture_width_downwards():
    request = VideoRequest(enable=True, host="10.0.0.5", port=5600, width=2560,
                           height=720, fps=30, bitrate_kbps=2000,
                           device="/dev/video0")

    assert "right=1280" in build_pipeline(request)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `ssh star@a_navi 'cd ~/navi/src/navi_teleop && python3 -m pytest test/test_video_sender.py -v'`
Expected: FAIL, `ModuleNotFoundError: No module named 'navi_teleop.video_sender'`

- [ ] **Step 3: Write the implementation**

```python
# ~/navi/src/navi_teleop/navi_teleop/video_sender.py
"""Streams the ZED's left eye to the ground station on request.

Control arrives over rosbridge (/video_request) and status goes back the
same way, but the video itself never touches rosbridge: it is H.264 over
RTP/UDP straight to the operator's machine. On the long-range field link
that difference is the whole point - UDP loses packets and shows
artifacts, where a TCP-framed stream stalls and then bursts.

The camera is read as a plain UVC device, not through the ZED SDK or
zed_ros2_wrapper, so this node shares no state with the localization
stack and can run whether or not that stack is up.

The pipeline runs as a gst-launch-1.0 subprocess rather than through
PyGObject: it needs no Python bindings on either machine, and a pipeline
that dies on a bad frame kills a subprocess instead of this node.
"""

import json
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from navi_teleop.video_request import InvalidRequest, VideoRequest, parse_request


def build_pipeline(request: VideoRequest) -> list[str]:
    """gst-launch-1.0 argv for one stream. Pure, so it is testable without
    a camera, a network, or ROS."""
    # The ZED presents both eyes in one frame, side by side. Cropping the
    # right half away leaves the left eye at exactly half the capture width.
    crop_right = request.width // 2
    return [
        "gst-launch-1.0", "-q",
        "v4l2src", f"device={request.device}",
        "!", f"video/x-raw,width={request.width},height={request.height},"
             f"framerate={request.fps}/1",
        "!", "videocrop", f"right={crop_right}",
        "!", "videoconvert",
        "!", "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
        f"bitrate={request.bitrate_kbps}", "key-int-max=30",
        "!", "rtph264pay", "config-interval=1", "pt=96",
        "!", "udpsink", f"host={request.host}", f"port={request.port}",
    ]


class VideoSender(Node):

    def __init__(self, launcher=subprocess.Popen) -> None:
        super().__init__('video_sender')

        self.declare_parameter('max_width', 2560)
        self.declare_parameter('max_height', 720)
        self.declare_parameter('max_bitrate_kbps', 4000)
        self.declare_parameter('allowed_device', '/dev/video0')
        self.declare_parameter('status_interval_seconds', 1.0)

        self._launcher = launcher
        self._process: subprocess.Popen | None = None

        self._status_publisher = self.create_publisher(String, '/video_status', 10)
        self.create_subscription(String, '/video_request', self._on_request, 10)
        self.create_timer(
            float(self.get_parameter('status_interval_seconds').value),
            self._publish_status_tick,
        )

        self._state = 'stopped'
        self._detail = ''
        self._publish_status()
        self.get_logger().info("video_sender ready, waiting for /video_request")

    def _limits(self) -> dict:
        return dict(
            max_width=int(self.get_parameter('max_width').value),
            max_height=int(self.get_parameter('max_height').value),
            max_bitrate_kbps=int(self.get_parameter('max_bitrate_kbps').value),
            allowed_device=str(self.get_parameter('allowed_device').value),
        )

    def _on_request(self, msg: String) -> None:
        try:
            request = parse_request(msg.data, **self._limits())
        except InvalidRequest as exc:
            self._set_state('failed', str(exc))
            self.get_logger().warn(f"rejected video request: {exc}")
            return

        self._stop_stream()
        if request.enable:
            self._start_stream(request)
        else:
            self._set_state('stopped', '')

    def _start_stream(self, request: VideoRequest) -> None:
        if shutil.which("gst-launch-1.0") is None:
            self._set_state('failed', 'gst-launch-1.0 not installed')
            return

        self._set_state('starting', f"{request.width // 2}x{request.height} "
                                    f"@{request.fps} {request.bitrate_kbps}kbps")
        argv = build_pipeline(request)
        try:
            self._process = self._launcher(argv, stdout=subprocess.DEVNULL,
                                           stderr=subprocess.PIPE)
        except OSError as exc:
            self._set_state('failed', f"could not start pipeline: {exc}")
            return

        self.get_logger().info(f"streaming to {request.host}:{request.port}")
        self._set_state('streaming', f"{request.host}:{request.port}")

    def _stop_stream(self) -> None:
        if self._process is None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        self._process = None

    def _publish_status_tick(self) -> None:
        # A pipeline can die long after it started - a camera unplugged, an
        # encoder error. Nothing reports that except the exit code, so the
        # heartbeat is also where death is noticed.
        if self._state == 'streaming' and self._process is not None:
            code = self._process.poll()
            if code is not None:
                stderr = b''
                if self._process.stderr is not None:
                    stderr = self._process.stderr.read() or b''
                detail = stderr.decode(errors='replace').strip().splitlines()
                self._process = None
                self._set_state('failed', detail[-1] if detail else f"pipeline exited ({code})")
                return
        self._publish_status()

    def _set_state(self, state: str, detail: str) -> None:
        self._state = state
        self._detail = detail
        self._publish_status()

    def _publish_status(self) -> None:
        # json.dumps, not an f-string: detail carries GStreamer errors, which
        # contain quotes often enough to produce unparseable status messages.
        message = String()
        message.data = json.dumps({"state": self._state, "detail": self._detail})
        self._status_publisher.publish(message)

    def destroy_node(self) -> bool:
        self._stop_stream()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Add the entry point**

In `~/navi/src/navi_teleop/setup.py`, extend `console_scripts`:

```python
        'console_scripts': [
            'manual_twist_listener = navi_teleop.manual_twist_listener:main',
            'video_sender = navi_teleop.video_sender:main',
        ],
```

- [ ] **Step 5: Run the tests and build**

Run: `ssh star@a_navi 'cd ~/navi/src/navi_teleop && python3 -m pytest test/ -v'`
Expected: PASS, 15 tests (9 from Task 1, 6 here).

Run: `ssh star@a_navi 'bash -lc "source /opt/ros/humble/setup.bash && cd ~/navi && colcon build --symlink-install"'`
Expected: `Finished <<< navi_teleop`

- [ ] **Step 6: Verify the node against real ROS, without the ground station**

Nested quoting through ssh, bash, and YAML is where this kind of check usually dies, so put the request in a file on the Orin first and publish it with `-f`.

```bash
ssh star@a_navi "cat > /tmp/video_on.yaml" <<'EOF'
data: '{"enable": true, "host": "127.0.0.1", "port": 5600}'
EOF

ssh star@a_navi 'bash -lc "
source /opt/ros/humble/setup.bash && source ~/navi/install/local_setup.bash
ros2 run navi_teleop video_sender > /tmp/sender.log 2>&1 &
sleep 3
echo --- before ---
timeout 3 ros2 topic echo /video_status --once
ros2 topic pub --once /video_request std_msgs/msg/String -f /tmp/video_on.yaml
sleep 3
echo --- after ---
timeout 3 ros2 topic echo /video_status --once
pkill -f video_sender
"'
```

Expected: the status before is `stopped`, the status after is `streaming`. `/tmp/sender.log` should show `streaming to 127.0.0.1:5600`.

- [ ] **Step 7: Verify a bad request is refused**

```bash
ssh star@a_navi "cat > /tmp/video_bad.yaml" <<'EOF'
data: '{"enable": true, "host": "127.0.0.1", "bitrate_kbps": 99000}'
EOF
```

Publish it the same way as Step 6.
Expected: status becomes `failed` with a detail naming `bitrate_kbps`, and no `gst-launch-1.0` process appears in `ps` on the Orin.

- [ ] **Step 8: Commit**

```bash
ssh star@a_navi 'cd ~/navi && git add src && \
  git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" \
      commit -q -m "Add video_sender node streaming the ZED left eye over RTP"'
```

---

### Task 3: The laptop-side receiver

**Files:**
- Create: `ground_station/video_receiver.py`
- Test: `tests/test_video_receiver.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `build_receive_pipeline(port: int, width: int, height: int) -> list[str]`, and `VideoReceiver(port: int = 5600, width: int = 672, height: int = 376, launcher=subprocess.Popen)` with methods `start()`, `stop()`, `read_frame() -> bytes | None`, and property `is_running: bool`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_video_receiver.py
import io
import subprocess

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

    assert "video/x-raw,format=RGB" in argv
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_video_receiver.py -v`
Expected: FAIL, `ModuleNotFoundError: No module named 'ground_station.video_receiver'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/video_receiver.py
"""Receives the rover's H.264 stream and hands out raw frames.

The decode pipeline is a gst-launch-1.0 subprocess writing raw RGB to its
stdout, not an in-process GStreamer pipeline. PyGObject is absent from
this project's virtualenv (built with include-system-site-packages =
false), so in-process would mean either compiling PyGObject or opening
the venv to system packages. The subprocess needs neither, and it
isolates the decoder: a corrupt stream that kills the pipeline kills a
child process, not the ground station.

No ROS here. The laptop has no ROS 2 installed - the rover is asked to
start and stop the stream over rosbridge, and this module only listens on
a UDP port.
"""

import argparse
import subprocess


def build_receive_pipeline(port: int, width: int, height: int) -> list[str]:
    return [
        "gst-launch-1.0", "-q",
        "udpsrc", f"port={port}",
        "caps=application/x-rtp,media=video,encoding-name=H264,payload=96",
        "!", "rtpjitterbuffer", "latency=100",
        "!", "rtph264depay",
        "!", "avdec_h264",
        "!", "videoconvert",
        "!", f"video/x-raw,format=RGB,width={width},height={height}",
        "!", "fdsink", "fd=1",
    ]


class VideoReceiver:
    """Owns the decode subprocess. Frames are pulled, not pushed, so the
    GUI reads on its own timer instead of being driven by the network."""

    def __init__(self, port: int = 5600, width: int = 672, height: int = 376,
                 launcher=subprocess.Popen):
        self.port = port
        self.width = width
        self.height = height
        self._launcher = launcher
        self._process = None

    @property
    def frame_size(self) -> int:
        return self.width * self.height * 3

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self) -> None:
        if self._process is not None:
            return
        self._process = self._launcher(
            build_receive_pipeline(self.port, self.width, self.height),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )

    def stop(self) -> None:
        if self._process is None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        self._process = None

    def read_frame(self) -> bytes | None:
        """One complete frame, or None if a whole frame is not available.
        Never returns a partial frame - a short read would tear the image
        and desynchronize every frame after it."""
        if self._process is None or self._process.stdout is None:
            return None
        data = self._process.stdout.read(self.frame_size)
        if data is None or len(data) < self.frame_size:
            return None
        return data


def main() -> None:
    """Standalone debugging: decode a stream with no rover and no GUI.

        python -m ground_station.video_receiver --port 5600
    """
    parser = argparse.ArgumentParser(description="Receive the rover video stream")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--width", type=int, default=672)
    parser.add_argument("--height", type=int, default=376)
    parser.add_argument("--frames", type=int, default=0,
                        help="stop after N frames (0 = run until interrupted)")
    args = parser.parse_args()

    receiver = VideoReceiver(port=args.port, width=args.width, height=args.height)
    receiver.start()
    print(f"listening for H.264/RTP on udp/{args.port}, "
          f"expecting {args.width}x{args.height}")
    count = 0
    try:
        while args.frames == 0 or count < args.frames:
            if receiver.read_frame() is None:
                if not receiver.is_running:
                    print("pipeline exited")
                    break
                continue
            count += 1
            print(f"\rframes: {count}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        print(f"\nreceived {count} frames")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv/bin/python -m pytest tests/test_video_receiver.py -v`
Expected: PASS, 9 tests.

- [ ] **Step 5: Prove the receive path against a real stream, no rover**

Requires `gstreamer1.0-libav`. In one terminal, send a test pattern to yourself:

```bash
gst-launch-1.0 -q videotestsrc pattern=ball ! video/x-raw,width=672,height=376,framerate=30/1 \
  ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=800 key-int-max=30 \
  ! rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5600
```

In another:

```bash
.venv/bin/python -m ground_station.video_receiver --port 5600 --frames 60
```

Expected: `received 60 frames`. If it hangs at 0 frames, `avdec_h264` is missing — install `gstreamer1.0-libav`.

- [ ] **Step 6: Commit**

```bash
git add ground_station/video_receiver.py tests/test_video_receiver.py
git commit -m "Add the laptop-side H.264 receiver as a gst-launch subprocess"
```

---

### Task 4: Video request and status on the rosbridge client

**Files:**
- Modify: `ground_station/ros_client.py`
- Test: `tests/test_ros_client.py` (append)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: on `RosBridgeClient` — `subscribe_video_status(topic_name: str = "/video_status") -> None`, `publish_video_request(enable: bool, host: str, port: int, width: int, height: int, fps: int, bitrate_kbps: int) -> None`; on `RosSignals` — `video_status_received = Signal(dict)`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_ros_client.py`, reusing the `FakeRos`, `FakeTopic`, and `fake_message_factory` already defined in that file.

```python
def test_subscribe_video_status_emits_parsed_json():
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    received = []
    client.signals.video_status_received.connect(received.append)

    client.subscribe_video_status()
    topic = FakeTopic.instances[-1]
    topic.callback({"data": '{"state": "streaming", "detail": "10.0.0.5:5600"}'})

    assert received == [{"state": "streaming", "detail": "10.0.0.5:5600"}]


def test_subscribe_video_status_reports_malformed_payloads_instead_of_raising():
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    received = []
    client.signals.video_status_received.connect(received.append)

    client.subscribe_video_status()
    FakeTopic.instances[-1].callback({"data": "{not json"})

    assert received[0]["state"] == "failed"
    assert "JSON" in received[0]["detail"]


def test_publish_video_request_sends_json_on_the_request_topic():
    import json

    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)

    client.publish_video_request(enable=True, host="192.168.178.101", port=5600,
                                 width=1344, height=376, fps=30, bitrate_kbps=800)

    topic = FakeTopic.instances[-1]
    assert topic.name == "/video_request"
    assert topic.msg_type == "std_msgs/String"
    payload = json.loads(topic.published_messages[-1]["data"])
    assert payload == {"enable": True, "host": "192.168.178.101", "port": 5600,
                       "width": 1344, "height": 376, "fps": 30, "bitrate_kbps": 800}


def test_publish_video_request_reuses_one_topic_across_calls():
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)

    client.publish_video_request(enable=True, host="10.0.0.5", port=5600,
                                 width=1344, height=376, fps=30, bitrate_kbps=800)
    client.publish_video_request(enable=False, host="10.0.0.5", port=5600,
                                 width=1344, height=376, fps=30, bitrate_kbps=800)

    request_topics = [t for t in FakeTopic.instances if t.name == "/video_request"]
    assert len(request_topics) == 1
    assert len(request_topics[0].published_messages) == 2
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_ros_client.py -v`
Expected: FAIL, `AttributeError: 'RosSignals' object has no attribute 'video_status_received'`

- [ ] **Step 3: Write the implementation**

In `ground_station/ros_client.py`, add `import json` at the top, add the signal, and add the two methods following the existing `/manual_twist` pattern.

```python
class RosSignals(QObject):
    twist_received = Signal(dict)
    nodes_received = Signal(list)
    connection_changed = Signal(bool)
    video_status_received = Signal(dict)
```

In `RosBridgeClient.__init__`, alongside `self._manual_twist_topic = None`:

```python
        self._video_request_topic = None
```

And the methods:

```python
    def subscribe_video_status(self, topic_name: str = "/video_status") -> None:
        """The rover's own account of the stream: stopped, starting,
        streaming, or failed. Distinct from whether frames are actually
        arriving, which only the receiver can tell - a rover reporting
        'streaming' while no packets land is the signature of a blocked
        UDP port."""
        topic = self._topic_factory(self._ros, topic_name, "std_msgs/String")
        topic.subscribe(lambda msg: self.signals.video_status_received.emit(
            self._parse_status(msg.get("data", ""))))
        self._video_status_topic = topic

    @staticmethod
    def _parse_status(payload: str) -> dict:
        try:
            status = json.loads(payload)
        except json.JSONDecodeError as exc:
            return {"state": "failed", "detail": f"bad status JSON: {exc}"}
        if not isinstance(status, dict):
            return {"state": "failed", "detail": "status was not a JSON object"}
        return {"state": status.get("state", "failed"),
                "detail": status.get("detail", "")}

    def publish_video_request(self, enable: bool, host: str, port: int, width: int,
                              height: int, fps: int, bitrate_kbps: int) -> None:
        """Asks the rover to start or stop streaming to host:port. The host
        is ours, not the rover's: the rover is the server side of rosbridge
        and has no other way to learn where we are."""
        if self._video_request_topic is None:
            self._video_request_topic = self._topic_factory(
                self._ros, "/video_request", "std_msgs/String")
        self._video_request_topic.publish(self._message_factory({
            "data": json.dumps({
                "enable": enable, "host": host, "port": port, "width": width,
                "height": height, "fps": fps, "bitrate_kbps": bitrate_kbps,
            }),
        }))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv/bin/python -m pytest tests/test_ros_client.py -v`
Expected: PASS, all existing tests plus the 4 new ones.

- [ ] **Step 5: Commit**

```bash
git add ground_station/ros_client.py tests/test_ros_client.py
git commit -m "Add video request and status to the rosbridge client"
```

---

### Task 5: The video panel

**Files:**
- Create: `ground_station/ui/video_panel.py`
- Test: `tests/test_video_panel.py`

**Interfaces:**
- Consumes: `VideoReceiver` from Task 3.
- Produces: `VideoPanel(receiver=None, parent=None)` — a `QWidget` with `stream_requested = Signal(bool)`, methods `apply_status(status: dict) -> None` and `set_streaming(enabled: bool) -> None`, and attributes `toggle_button`, `status_label`, `image_label`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_video_panel.py
from ground_station.ui.video_panel import VideoPanel


class FakeReceiver:
    def __init__(self, frame=None):
        self.width = 4
        self.height = 2
        self.started = False
        self.stopped = False
        self.is_running = True
        self._frame = frame

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def read_frame(self):
        return self._frame


def test_panel_starts_idle(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)

    assert "OFF" in panel.status_label.text().upper()


def test_toggle_emits_stream_requested_true_then_false(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    requests = []
    panel.stream_requested.connect(requests.append)

    panel.toggle_button.click()
    panel.toggle_button.click()

    assert requests == [True, False]


def test_set_streaming_starts_and_stops_the_receiver(qtbot):
    receiver = FakeReceiver()
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)

    panel.set_streaming(True)
    assert receiver.started

    panel.set_streaming(False)
    assert receiver.stopped


def test_apply_status_shows_the_rover_reported_state(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)

    panel.apply_status({"state": "failed", "detail": "camera busy"})

    assert "FAILED" in panel.status_label.text().upper()
    assert "camera busy" in panel.status_label.text()


def test_panel_reports_no_frames_while_rover_claims_streaming(qtbot):
    # The rover saying 'streaming' while nothing arrives is exactly the
    # blocked-UDP-port case, and it must read differently from 'off'.
    panel = VideoPanel(receiver=FakeReceiver(frame=None), no_frame_after_seconds=0.0)
    qtbot.addWidget(panel)
    panel.apply_status({"state": "streaming", "detail": "10.0.0.5:5600"})

    panel.set_streaming(True)
    panel._poll_frame(now=100.0)
    panel._poll_frame(now=101.0)

    assert "NO FRAMES" in panel.status_label.text().upper()


def test_panel_clears_no_frames_once_a_frame_arrives(qtbot):
    receiver = FakeReceiver(frame=bytes(4 * 2 * 3))
    panel = VideoPanel(receiver=receiver, no_frame_after_seconds=0.0)
    qtbot.addWidget(panel)
    panel.apply_status({"state": "streaming", "detail": "10.0.0.5:5600"})
    panel.set_streaming(True)

    panel._poll_frame(now=100.0)

    assert "NO FRAMES" not in panel.status_label.text().upper()
    assert panel.image_label.pixmap() is not None
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_video_panel.py -v`
Expected: FAIL, `ModuleNotFoundError: No module named 'ground_station.ui.video_panel'`

- [ ] **Step 3: Write the implementation**

```python
# ground_station/ui/video_panel.py
"""Live camera view with its own on/off control.

Two independent facts are shown, never conflated: what the rover says
about the stream (/video_status) and whether frames are actually
arriving here. A rover reporting 'streaming' while nothing lands is the
signature of a blocked UDP port, and collapsing the two would hide
exactly that case.
"""

from time import monotonic

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget

from ground_station import theme
from ground_station.video_receiver import VideoReceiver


class VideoPanel(QWidget):
    stream_requested = Signal(bool)

    def __init__(self, receiver=None, parent=None, poll_interval_ms: int = 33,
                 no_frame_after_seconds: float = 2.0):
        super().__init__(parent)
        self.receiver = receiver if receiver is not None else VideoReceiver()
        self.no_frame_after_seconds = no_frame_after_seconds
        self._streaming = False
        self._last_frame_at: float | None = None
        self._rover_state = "stopped"
        self._rover_detail = ""

        self.setStyleSheet(
            f"background-color: {theme.PANEL}; border: 1px solid {theme.BORDER}; "
            f"border-radius: 6px;"
        )

        title = QLabel("CAMERA / ZED FRONT LEFT")
        title.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600; border: none;")

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(self.receiver.width, self.receiver.height)
        self.image_label.setStyleSheet(f"background-color: {theme.BG}; border: none;")

        self.toggle_button = QPushButton("Start video")
        self.toggle_button.setStyleSheet(
            f"QPushButton {{ background-color: {theme.PANEL}; color: {theme.TEXT}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 4px 14px; }} "
            f"QPushButton:hover {{ border-color: {theme.ACCENT}; }}"
        )
        self.toggle_button.clicked.connect(self._on_toggle_clicked)

        self.status_label = QLabel("VIDEO OFF")
        self.status_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;"
        )

        layout = QVBoxLayout(self)
        layout.addWidget(title)
        layout.addWidget(self.image_label, stretch=1)
        footer = QHBoxLayout()
        footer.addWidget(self.toggle_button)
        footer.addWidget(self.status_label)
        footer.addStretch()
        layout.addLayout(footer)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_frame)
        self._poll_timer.start(poll_interval_ms)

    def _on_toggle_clicked(self) -> None:
        self.stream_requested.emit(not self._streaming)

    def set_streaming(self, enabled: bool) -> None:
        """Drives the local receiver. Called by the window after it has sent
        the request, and on disconnect - the receiver is stopped whether or
        not the rover ever answers."""
        self._streaming = enabled
        self.toggle_button.setText("Stop video" if enabled else "Start video")
        if enabled:
            self._last_frame_at = None
            self.receiver.start()
        else:
            self.receiver.stop()
            self.image_label.clear()
            self._rover_state = "stopped"
            self._rover_detail = ""
        self._refresh_status()

    def apply_status(self, status: dict) -> None:
        self._rover_state = status.get("state", "failed")
        self._rover_detail = status.get("detail", "")
        self._refresh_status()

    def _poll_frame(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        if not self._streaming:
            return
        frame = self.receiver.read_frame()
        if frame is not None:
            self._last_frame_at = now
            image = QImage(frame, self.receiver.width, self.receiver.height,
                           self.receiver.width * 3, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(image))
        elif self._last_frame_at is None:
            self._last_frame_at = now
        self._refresh_status(now)

    def _refresh_status(self, now: float | None = None) -> None:
        if not self._streaming:
            self.status_label.setText("VIDEO OFF")
            self.status_label.setStyleSheet(
                f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        now = monotonic() if now is None else now
        starving = (self._last_frame_at is not None
                    and now - self._last_frame_at > self.no_frame_after_seconds)
        if starving and self._rover_state == "streaming":
            text = "NO FRAMES - rover streaming, nothing arriving (UDP blocked?)"
            color = theme.ACCENT
        elif self._rover_state == "failed":
            text = f"FAILED - {self._rover_detail}"
            color = theme.ACCENT
        elif self._rover_state == "streaming":
            text = f"STREAMING {self._rover_detail}"
            color = theme.OK
        else:
            text = self._rover_state.upper()
            color = theme.TEXT_DIM
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"color: {color}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv/bin/python -m pytest tests/test_video_panel.py -v`
Expected: PASS, 6 tests.

- [ ] **Step 5: Commit**

```bash
git add ground_station/ui/video_panel.py tests/test_video_panel.py
git commit -m "Add the video panel with separate rover and wire status"
```

---

### Task 6: Wire the panel into the window

**Files:**
- Modify: `ground_station/ui/dashboard_page.py`
- Modify: `ground_station/ui/main_window.py`
- Test: `tests/test_main_window.py` (append)

**Interfaces:**
- Consumes: `VideoPanel` (Task 5), `publish_video_request` and `video_status_received` (Task 4).
- Produces: `MainWindow.local_address_for(host: str, port: int) -> str`, and `MainWindow._on_stream_requested(enable: bool) -> None`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_main_window.py`. The existing `FakeRos`/`make_window` helpers in that file already cover construction; these tests use the same style.

```python
def test_dashboard_has_a_video_panel(qtbot):
    window = make_window(qtbot)

    assert window.dashboard_page.video_panel is not None


def test_enabling_video_publishes_a_request_with_our_own_address(qtbot):
    window = make_window(qtbot)
    window._connect_to("192.168.178.33", 9090)

    window._on_stream_requested(True)

    request = window.ros_client.video_requests[-1]
    assert request["enable"] is True
    assert request["port"] == 5600
    assert request["host"]


def test_disabling_video_stops_the_receiver_even_if_the_rover_never_answers(qtbot):
    window = make_window(qtbot)
    window._connect_to("192.168.178.33", 9090)
    window._on_stream_requested(True)

    window._on_stream_requested(False)

    assert window.dashboard_page.video_panel.receiver.stopped


def test_video_status_reaches_the_panel(qtbot):
    window = make_window(qtbot)
    window._connect_to("192.168.178.33", 9090)

    window._on_video_status({"state": "streaming", "detail": "10.0.0.5:5600"})

    assert "STREAMING" in window.dashboard_page.video_panel.status_label.text().upper()


def test_requesting_video_without_a_connection_is_ignored(qtbot):
    window = make_window(qtbot)

    window._on_stream_requested(True)

    assert window.dashboard_page.video_panel._streaming is False


def test_local_address_is_the_interface_that_reaches_the_rover(qtbot):
    window = make_window(qtbot)

    address = window.local_address_for("192.168.178.33", 9090)

    assert address.count(".") == 3
```

The existing fake ROS client in `tests/test_main_window.py` needs two additions so these tests can observe requests. Extend `FakeRos` (the fake client class used by `make_fake_client_factory`) with:

```python
    def publish_video_request(self, enable, host, port, width, height, fps, bitrate_kbps):
        self.video_requests.append({
            "enable": enable, "host": host, "port": port, "width": width,
            "height": height, "fps": fps, "bitrate_kbps": bitrate_kbps,
        })

    def subscribe_video_status(self):
        self.video_status_subscribed = True
```

and initialize `self.video_requests = []` plus `self.video_status_subscribed = False` in its `__init__`. Also give the fake receiver used by the panel a `stopped` flag — reuse `FakeReceiver` from `tests/test_video_panel.py` by importing it.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_main_window.py -v`
Expected: FAIL, `AttributeError: 'DashboardPage' object has no attribute 'video_panel'`

- [ ] **Step 3: Add the panel to the dashboard**

In `ground_station/ui/dashboard_page.py`, import `VideoPanel`, construct it, and place it. The dashboard is currently a single row of drive card and node list; video goes beside them in a column so the drive readouts stay visible while driving.

```python
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout

from ground_station import theme
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.node_list_widget import NodeListWidget
from ground_station.ui.video_panel import VideoPanel


class DashboardPage(QWidget):
    drive_details_requested = Signal()

    def __init__(self, video_receiver=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {theme.BG}; color: {theme.TEXT};")
        self.drive_card = DriveCard()
        self.node_list = NodeListWidget()
        self.video_panel = VideoPanel(receiver=video_receiver)

        self.drive_card.details_requested.connect(self.drive_details_requested)

        left = QVBoxLayout()
        left.addWidget(self.video_panel, stretch=3)
        left.addWidget(self.drive_card, stretch=1)

        layout = QHBoxLayout(self)
        layout.addLayout(left, stretch=3)
        layout.addWidget(self.node_list, stretch=1)
```

- [ ] **Step 4: Wire the window**

In `ground_station/ui/main_window.py`, add `import socket` at the top, accept a `video_receiver` parameter, pass it to `DashboardPage`, and add the handlers.

Constructor signature gains `video_receiver=None`, and the dashboard construction becomes:

```python
        self.dashboard_page = DashboardPage(video_receiver=video_receiver)
```

After the existing `drive_detail_page.back_requested` connection, add:

```python
        self.dashboard_page.video_panel.stream_requested.connect(self._on_stream_requested)
```

In `_connect_to`, alongside `self.ros_client.subscribe_manual_twist()`:

```python
            self.ros_client.subscribe_video_status()
```

and connect the signal where the others are connected:

```python
        self.ros_client.signals.video_status_received.connect(self._on_video_status)
```

Then the methods:

```python
    def local_address_for(self, host: str, port: int) -> str:
        """Our own address on the interface that reaches the rover. The rover
        cannot discover this: it is the server side of rosbridge, and a
        hardcoded laptop address breaks on every network change."""
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # UDP connect assigns a local address without sending anything.
            probe.connect((host, port))
            return probe.getsockname()[0]
        except OSError:
            return ""
        finally:
            probe.close()

    def _on_stream_requested(self, enable: bool) -> None:
        panel = self.dashboard_page.video_panel
        if self.ros_client is None or not self.ros_client.is_connected:
            panel.apply_status({"state": "failed", "detail": "not connected to rosbridge"})
            return

        if enable:
            address = self.local_address_for(self.host_input.text().strip() or "127.0.0.1",
                                             self.video_port)
            if not address:
                panel.apply_status({"state": "failed", "detail": "no route to the rover"})
                return
            self.ros_client.publish_video_request(
                enable=True, host=address, port=self.video_port,
                width=1344, height=376, fps=30, bitrate_kbps=800)
        else:
            self.ros_client.publish_video_request(
                enable=False, host="", port=self.video_port,
                width=1344, height=376, fps=30, bitrate_kbps=800)
        # The local receiver follows our intent, not the rover's answer, so a
        # dead link cannot leave a stream pointed at us.
        panel.set_streaming(enable)

    def _on_video_status(self, status: dict) -> None:
        self.dashboard_page.video_panel.apply_status(status)
```

Add `self.video_port = video_port` in the constructor with a `video_port: int = 5600` parameter.

- [ ] **Step 5: Run the whole suite**

Run: `.venv/bin/python -m pytest -v`
Expected: PASS, every test including the pre-existing ones.

- [ ] **Step 6: Commit**

```bash
git add ground_station/ui/dashboard_page.py ground_station/ui/main_window.py tests/test_main_window.py
git commit -m "Wire the video panel into the dashboard and rosbridge client"
```

---

### Task 7: End-to-end verification against the rover

No new code. This is the check that the two halves meet, and the only place the real latency number can be measured.

**Files:**
- Create: `docs/superpowers/plans/2026-08-27-ground-station-video-verification.md`
- Modify: `docs/superpowers/specs/2026-08-27-ground-station-video-design.md` — fill in the measured latency

- [ ] **Step 1: Start the rover side**

```bash
ssh star@a_navi 'bash -lc "
source /opt/ros/humble/setup.bash && source ~/navi/install/local_setup.bash
ros2 run navi_teleop video_sender
"'
```

- [ ] **Step 2: Start the ground station and connect**

```bash
./start_ground_station.sh
```

Connect to `192.168.178.33:9090`, then press **Start video**.

- [ ] **Step 3: Confirm the happy path**

Expected: status reads `STREAMING <your-ip>:5600` and the camera image appears within about two seconds.

- [ ] **Step 4: Measure glass-to-glass latency**

Point the camera at a phone stopwatch, screenshot the ground station showing both the panel and the stopwatch, and subtract. Record the number.

- [ ] **Step 5: Confirm the failure paths read correctly**

- Stop `video_sender` while streaming. Expected: the panel shows `NO FRAMES - rover streaming, nothing arriving` rather than an unchanged picture.
- Press **Start video** while disconnected. Expected: `FAILED - not connected to rosbridge`.
- Unplug the ZED and press **Start video**. Expected: `FAILED` with the GStreamer error, within a couple of seconds.

- [ ] **Step 6: Confirm video does not disturb driving**

With video streaming and a gamepad connected, confirm `manual_twist_listener` on the Orin still reports the expected rate, and that killing `video_sender` leaves it untouched.

- [ ] **Step 7: Write the verification doc and record the latency**

Write what was run, what was observed, and the measured latency into `docs/superpowers/plans/2026-08-27-ground-station-video-verification.md`, following the shape of `2026-08-25-ground-station-manual-verification.md`. Replace the placeholder sentence in the spec's Testing section with the measured number.

- [ ] **Step 8: Commit**

```bash
git add docs/superpowers/
git commit -m "Record the video end-to-end verification and measured latency"
```

---

## Notes for the executor

- Rover-side and laptop-side changes live in **two different git repositories**: `~/navi` on the Orin (initialized in Task 1) and this repo on the laptop. Commit in the right one.
- Tasks 1, 2, and 7 run against the Orin over ssh. Tasks 3 through 6 are laptop-only and need no rover.
- `gstreamer1.0-libav` on the laptop is a sudo prerequisite. Task 3's Step 5 is the first step that needs it; everything before it passes without.
- If `x264enc` proves too slow on the Orin at 30 fps, drop `fps` to 15 in the request rather than reaching for the hardware encoder — `nvv4l2h264enc` is known broken on this machine and diagnosing it is out of scope here.
