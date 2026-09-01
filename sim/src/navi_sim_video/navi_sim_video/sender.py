"""Streams the simulation's chase camera to the ground station.

The same protocol, frame size and encoder settings as the rover's
video_sender, so the ground station decodes either source with the same
receiver and the same panel. Only the port differs: 5601 rather than 5600,
which is what lets the mode switch change source without an ordering hazard
between two senders on one port.

Frames go into gst-launch through a pipe rather than an in-process pipeline,
following the rover's sender: it needs no PyGObject, and a decoder crash on a
corrupt stream kills a child process rather than this node.
"""

import subprocess
import tempfile

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


def build_send_pipeline(host: str, port: int, width: int, height: int,
                        fps: int, bitrate_kbps: int) -> list[str]:
    return [
        "gst-launch-1.0", "-q",
        # fdsrc reads a pipe, not a camera: it emits fixed-size chunks with
        # no notion of where a frame ends, and blocksize makes those chunks
        # whole frames rather than 186 fragments of one.
        "fdsrc", "fd=0", f"blocksize={width * height * 3}",
        # rawvideoparse, not a capsfilter. A capsfilter chunks nothing - it
        # only asserts that each buffer already is a frame of this size, and
        # when that is false the encoder compresses malformed buffers and the
        # decoder emits its zeroed output buffer, which renders as solid
        # green. rawvideoparse is what actually cuts the stream into frames.
        "!", "rawvideoparse", f"width={width}", f"height={height}",
        "format=rgb", f"framerate={fps}/1",
        "!", "videoconvert",
        "!", "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
        f"bitrate={bitrate_kbps}", "key-int-max=30",
        "!", "rtph264pay", "config-interval=1", "pt=96",
        "!", "udpsink", f"host={host}", f"port={port}",
    ]


class SimVideoSender(Node):
    def __init__(self):
        super().__init__("sim_video_sender")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5601)
        self.declare_parameter("width", 672)
        self.declare_parameter("height", 376)
        # Keep this equal to the chase camera's update_rate in
        # asterope_sim.urdf.xacro: rawvideoparse is told this number and
        # every timestamp in the stream drifts if the camera disagrees.
        self.declare_parameter("fps", 7)
        self.declare_parameter("bitrate_kbps", 800)

        argv = build_send_pipeline(
            self.get_parameter("host").value,
            self.get_parameter("port").value,
            self.get_parameter("width").value,
            self.get_parameter("height").value,
            self.get_parameter("fps").value,
            self.get_parameter("bitrate_kbps").value,
        )
        # stderr to a file, never a pipe: nothing here would drain a pipe,
        # and at about 64 KB the encoder would block in write() forever.
        self._stderr = tempfile.NamedTemporaryFile(
            mode="w", prefix="sim_video_sender_stderr_", delete=False)
        self._process = subprocess.Popen(
            argv, stdin=subprocess.PIPE, stderr=self._stderr)

        self._expected = (self.get_parameter("width").value
                          * self.get_parameter("height").value * 3)
        self.create_subscription(Image, "/sim_chase_camera/chase/image_raw",
                                 self._on_image, 1)
        self.get_logger().info(
            f"streaming to {self.get_parameter('host').value}:"
            f"{self.get_parameter('port').value}")

    def _on_image(self, msg: Image) -> None:
        if len(msg.data) != self._expected:
            # A size mismatch would be sliced into torn, progressively
            # desynchronised frames by the receiver rather than failing.
            self.get_logger().warn(
                f"dropping frame: got {len(msg.data)} bytes, "
                f"expected {self._expected}")
            return
        try:
            self._process.stdin.write(bytes(msg.data))
        except BrokenPipeError:
            self.get_logger().error("encoder exited - see " + self._stderr.name)
            raise SystemExit(1)

    def destroy_node(self):
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                # A pipeline stuck in a blocking write ignores SIGTERM;
                # raising here would skip rclpy's shutdown and leave the
                # encoder bound to the port.
                self._process.kill()
                self._process.wait(timeout=3)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SimVideoSender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how ros2 launch's SIGINT reaches a
        # spinning node; uncaught, every launch teardown logged this node
        # as "process has died, exit code 1" - a real crash looked the same.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
