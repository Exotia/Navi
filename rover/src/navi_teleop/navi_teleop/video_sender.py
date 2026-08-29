"""Streams the ZED's left eye to the ground station on request.

Control arrives over rosbridge (/video_request) and status goes back the
same way, but the video itself never touches rosbridge: it is H.264 over
RTP/UDP straight to the operator's machine. On the long-range field link
that difference is the whole point - UDP loses packets and shows
artifacts, where a TCP-framed stream stalls and then bursts.

Two sources. `zed_topic` (the default) takes frames from the ZED ROS 2
wrapper's rectified RGB topic and pipes them into the encoder through
stdin: the wrapper owns the camera for localisation, and the ZED SDK
opens it exclusively, so nothing else can. `v4l2` is the old path -
the camera as a plain UVC device - for a run without localisation
(start_navi.sh --no-localization), when the wrapper is not there.

The pipeline runs as a gst-launch-1.0 subprocess rather than through
PyGObject: it needs no Python bindings on either machine, and a pipeline
that dies on a bad frame kills a subprocess instead of this node.

A malformed request never tears down a stream that is already healthy:
the ceilings exist so a bad ask gets refused, not so a refusal can make
the operator believe video stopped when it did not. Status only becomes
failed when nothing is actually running.

The child's stderr goes to a temp file rather than a pipe. Nothing reads
a pipe until the process is already found dead, and a flaky UVC device
can fill the OS pipe buffer with warnings faster than that - the child
then blocks inside write() and the stream silently stalls while poll()
still reports it alive. A temp file gives the same diagnostic value
(read its tail once death is detected) without that failure mode, and
without needing a reader thread in a node that has no concurrency today.
Every path that ends a stream removes that file - a toggle-heavy operator
session must not leave one orphaned file per enable in /tmp forever.
"""

import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from navi_teleop.image_pipe import build_pipe_pipeline, bytes_per_pixel, frame_matches
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

    def __init__(self, launcher=subprocess.Popen, **node_kwargs) -> None:
        super().__init__('video_sender', **node_kwargs)

        self.declare_parameter('max_width', 2560)
        self.declare_parameter('max_height', 720)
        self.declare_parameter('max_bitrate_kbps', 4000)
        self.declare_parameter('allowed_device', '/dev/video0')
        self.declare_parameter('status_interval_seconds', 1.0)
        self.declare_parameter('source', 'zed_topic')
        self.declare_parameter('image_topic', '/zed_front/zed_node/rgb/image_rect_color')

        self._launcher = launcher
        self._process: subprocess.Popen | None = None
        self._stderr_path: str | None = None
        # A request waiting for its first frame: the pipeline for the
        # zed_topic source cannot be built until a frame's encoding is
        # known, so enabling it only records the request.
        self._pending: VideoRequest | None = None
        self._frame_bytes = 0

        self._status_publisher = self.create_publisher(String, '/video_status', 10)
        self.create_subscription(String, '/video_request', self._on_request, 10)
        if self._source() == 'zed_topic':
            # Depth 1: a late frame is worthless - only the newest one
            # matters to a live stream.
            self.create_subscription(Image, self._image_topic(), self._on_image, 1)
        self.create_timer(
            float(self.get_parameter('status_interval_seconds').value),
            self._publish_status_tick,
        )

        self._state = 'stopped'
        self._detail = ''
        self._publish_status()
        self.get_logger().info("video_sender ready, waiting for /video_request")

    def _source(self) -> str:
        return str(self.get_parameter('source').value)

    def _image_topic(self) -> str:
        return str(self.get_parameter('image_topic').value)

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
            self.get_logger().warn(f"rejected video request: {exc}")
            # A healthy stream keeps reporting streaming through a bad
            # request that arrives later - refusing it must not make the
            # operator believe video stopped when it did not.
            if self._state != 'streaming':
                self._set_state('failed', str(exc))
            return

        self._stop_stream()
        if not request.enable:
            self._set_state('stopped', '')
            return
        if self._source() == 'v4l2':
            self._start_stream(request)
        else:
            # The pipeline is built from the first frame, whose encoding
            # decides the raw format - so until one arrives there is
            # nothing to start.
            self._pending = request
            self._set_state('starting', f"waiting for {self._image_topic()} "
                                        f"({request.width}x{request.height})")

    def _start_stream(self, request: VideoRequest) -> None:
        if shutil.which("gst-launch-1.0") is None:
            self._set_state('failed', 'gst-launch-1.0 not installed')
            return

        self._set_state('starting', f"{request.width // 2}x{request.height} "
                                    f"@{request.fps} {request.bitrate_kbps}kbps")
        argv = build_pipeline(request)
        try:
            # A stale path should never survive to here - _on_request always
            # runs _stop_stream first - but a leftover file is worse than a
            # defensive check, so a path that slipped through gets removed
            # rather than silently orphaned.
            self._remove_stderr_file()
            stderr_file = tempfile.NamedTemporaryFile(
                mode='w', prefix='video_sender_stderr_', delete=False)
            self._stderr_path = stderr_file.name
            with stderr_file:
                self._process = self._launcher(argv, stdout=subprocess.DEVNULL,
                                               stderr=stderr_file)
        except OSError as exc:
            self._set_state('failed', f"could not start pipeline: {exc}")
            return

        self.get_logger().info(f"streaming to {request.host}:{request.port}")
        self._set_state('streaming', f"{request.host}:{request.port}")

    def _on_image(self, msg: Image) -> None:
        if self._pending is not None:
            request, self._pending = self._pending, None
            reason = frame_matches(msg.width, msg.height, msg.encoding, len(msg.data), request)
            if reason is not None:
                self.get_logger().warn(f"refusing video request: {reason}")
                self._set_state('failed', reason)
                return
            self._start_pipe_stream(request, msg.encoding)
        if self._state != 'streaming' or self._process is None:
            return
        if len(msg.data) != self._frame_bytes:
            # A torn frame would desynchronise every frame after it.
            self.get_logger().warn(
                f"dropping frame: {len(msg.data)} bytes, expected {self._frame_bytes}")
            return
        try:
            self._process.stdin.write(bytes(msg.data))
        except (BrokenPipeError, OSError):
            detail = self._stderr_tail()
            self._stop_stream()
            self._set_state('failed', detail if detail else "encoder exited")

    def _start_pipe_stream(self, request: VideoRequest, encoding: str) -> None:
        if shutil.which("gst-launch-1.0") is None:
            self._set_state('failed', 'gst-launch-1.0 not installed')
            return

        self._set_state('starting', f"{request.width}x{request.height} "
                                    f"@{request.fps} {request.bitrate_kbps}kbps")
        argv = build_pipe_pipeline(request.host, request.port, request.width,
                                   request.height, request.fps,
                                   request.bitrate_kbps, encoding)
        try:
            # A stale path should never survive to here - _on_request always
            # runs _stop_stream first - but a leftover file is worse than a
            # defensive check, so a path that slipped through gets removed
            # rather than silently orphaned.
            self._remove_stderr_file()
            stderr_file = tempfile.NamedTemporaryFile(
                mode='w', prefix='video_sender_stderr_', delete=False)
            self._stderr_path = stderr_file.name
            with stderr_file:
                self._process = self._launcher(argv, stdin=subprocess.PIPE,
                                               stdout=subprocess.DEVNULL,
                                               stderr=stderr_file)
        except OSError as exc:
            self._set_state('failed', f"could not start pipeline: {exc}")
            return

        self._frame_bytes = request.width * request.height * bytes_per_pixel(encoding)
        self.get_logger().info(f"streaming to {request.host}:{request.port}")
        self._set_state('streaming', f"{request.host}:{request.port}")

    def _stop_stream(self) -> None:
        self._pending = None
        if self._process is None:
            return
        if getattr(self._process, 'stdin', None) is not None:
            try:
                self._process.stdin.close()
            except OSError:
                pass
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        self._process = None
        self._remove_stderr_file()

    def _publish_status_tick(self) -> None:
        # A pipeline can die long after it started - a camera unplugged, an
        # encoder error. Nothing reports that except the exit code, so the
        # heartbeat is also where death is noticed.
        if self._state == 'streaming' and self._process is not None:
            code = self._process.poll()
            if code is not None:
                detail = self._stderr_tail()
                self._remove_stderr_file()
                self._process = None
                self._set_state('failed', detail if detail else f"pipeline exited ({code})")
                return
        self._publish_status()

    def _stderr_tail(self) -> str:
        if self._stderr_path is None:
            return ''
        try:
            with open(self._stderr_path, 'r', errors='replace') as f:
                lines = f.read().strip().splitlines()
        except OSError:
            return ''
        return lines[-1] if lines else ''

    def _remove_stderr_file(self) -> None:
        if self._stderr_path is None:
            return
        Path(self._stderr_path).unlink(missing_ok=True)
        self._stderr_path = None

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
