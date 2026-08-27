"""Receives the rover's H.264 stream and hands out raw frames.

The decode pipeline is a gst-launch-1.0 subprocess writing raw RGB to its
stdout, not an in-process GStreamer pipeline. PyGObject is absent from
this project's virtualenv (built with include-system-site-packages =
false), so in-process would mean either compiling PyGObject or opening
the venv to system packages. The subprocess needs neither, and it
isolates the decoder: a corrupt stream that kills the pipeline kills a
child process, not the ground station.

read_frame() is non-blocking. A plain `stdout.read(frame_size)` blocks
until the full frame arrives - on a stalled or slow stream that would
freeze the GUI's event loop, which is exactly the "no frames arriving"
case the UI exists to surface. The child's stdout is put in non-blocking
mode instead, and partial reads accumulate in a buffer until a whole
frame is available; a short read is never handed back, since that would
tear the image and desynchronize every frame after it.

No ROS here. The laptop has no ROS 2 installed - the rover is asked to
start and stop the stream over rosbridge, and this module only listens on
a UDP port.
"""

import argparse
import io
import os
import subprocess


def build_receive_pipeline(port: int, width: int, height: int) -> list[str]:
    """width/height are pinned into the caps filter, not left open. The
    read side slices exactly width * height * 3 bytes per frame off the
    pipe using the numbers passed to VideoReceiver's constructor, not
    numbers observed from the stream - so a sender that emits a different
    size needs to fail the negotiation loudly (pipeline dies, is_running
    goes false, the panel reports a dead stream) rather than emit frames
    at the wrong stride, which read_frame would silently slice into torn,
    progressively desynchronized images. A refused pipeline is
    diagnosable; a corrupted picture over a lossy field link is not."""
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
        self._buffer = bytearray()

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
        try:
            os.set_blocking(self._process.stdout.fileno(), False)
        except (AttributeError, OSError, io.UnsupportedOperation):
            pass

    def stop(self) -> None:
        if self._process is None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        self._process = None
        self._buffer.clear()

    def read_frame(self) -> bytes | None:
        """One complete frame, or None if a whole frame is not available
        yet. Never returns a partial frame - a short read would tear the
        image and desynchronize every frame after it."""
        if self._process is None or self._process.stdout is None:
            return None
        chunk = self._process.stdout.read(self.frame_size - len(self._buffer))
        if chunk:
            self._buffer.extend(chunk)
        if len(self._buffer) < self.frame_size:
            return None
        frame = bytes(self._buffer[:self.frame_size])
        del self._buffer[:self.frame_size]
        return frame


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
