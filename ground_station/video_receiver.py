"""Receives the rover's H.264 stream and hands out raw frames.

The decode pipeline is a gst-launch-1.0 subprocess writing raw RGB to its
stdout, not an in-process GStreamer pipeline. PyGObject is absent from
this project's virtualenv (built with include-system-site-packages =
false), so in-process would mean either compiling PyGObject or opening
the venv to system packages. The subprocess needs neither, and it
isolates the decoder: a corrupt stream that kills the pipeline kills a
child process, not the ground station.

A reader thread owns the pipe, and read_frame() only takes what that
thread has already put down. The pipe is the reason: a 672x376 RGB frame
is 758,016 bytes, roughly twelve times a Linux pipe's 64 KB capacity.
Reading it from the GUI's 33 ms timer meant one tick could consume at
most one pipe-full, so gst-launch spent nearly all its time blocked in
write() and the backlog accumulated upstream in the jitter buffer and
the kernel's UDP socket buffer. Measured against the rover that capped
the panel at 2.2 fps, with latency that grew for as long as the stream
ran, while a greedy reader on the same pipeline managed ~27 fps.

The thread therefore reads as fast as the decoder produces, keeping the
pipe empty, and keeps only the newest completed frame. Frames the GUI
was too late to collect are dropped rather than queued: on a live view a
stale frame has no value, and handing the backlog over would replay it
at 30 fps instead of catching up. read_frame() stays non-blocking, so a
stalled stream still shows up as "no frames arriving" rather than
freezing the event loop, and a short read is never handed back, which
would tear the image and desynchronize every frame after it.

No ROS here. The laptop has no ROS 2 installed - the rover is asked to
start and stop the stream over rosbridge, and this module only listens on
a UDP port.
"""

import argparse
import re
import subprocess
import tempfile
import threading
import time
from pathlib import Path


_GEOMETRY = re.compile(r"(?<![0-9])(\d{2,5})x(\d{2,5})(?![0-9])")


def parse_geometry(detail: str) -> tuple[int, int] | None:
    """The width and height the rover names in its /video_status detail,
    e.g. "192.168.178.101:5600 640x360" -> (640, 360), or None when the
    detail carries no size. The rover's zed_topic source streams whatever
    the ZED wrapper publishes and reports that size here, and a receiver
    pinned to any other size never frames a single picture - so this is
    the one place the ground station learns what to decode."""
    match = _GEOMETRY.search(detail or "")
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


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
    """Owns the decode subprocess and the thread that drains it. The GUI
    reads on its own timer and always gets the newest frame available,
    never a queue of old ones."""

    def __init__(self, port: int = 5600, width: int = 672, height: int = 376,
                 launcher=subprocess.Popen):
        self.port = port
        self.width = width
        self.height = height
        self._launcher = launcher
        self._process = None
        self._stderr_path: str | None = None
        self._reader: threading.Thread | None = None
        self._stop_reading = threading.Event()
        self._lock = threading.Lock()
        self._latest: bytes | None = None

    @property
    def frame_size(self) -> int:
        return self.width * self.height * 3

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self) -> None:
        if self._process is not None:
            return
        # Defensive: a previous run's stderr temp file should already have
        # been removed by stop(), but if start() is ever called without a
        # matching stop() first (e.g. after the process died on its own),
        # don't leak a stale file on top of the new one.
        self._remove_stderr_file()
        # stderr goes to a temp file, not subprocess.PIPE: gst-launch-1.0 -q
        # still prints bus WARNING/ERROR text (-q only suppresses progress),
        # and rtph264depay/avdec_h264 warn on every loss burst - the normal
        # steady state of the lossy link this feature exists for. Nothing
        # ever read a PIPE here, so at ~64 KB the pipe would fill and the
        # streaming thread would block in write(), stalling the pipeline
        # permanently. A temp file needs no reader thread and keeps the text
        # diagnosable on disk instead.
        stderr_file = tempfile.NamedTemporaryFile(
            mode="w", prefix="video_receiver_stderr_", delete=False)
        self._stderr_path = stderr_file.name
        try:
            self._process = self._launcher(
                build_receive_pipeline(self.port, self.width, self.height),
                stdout=subprocess.PIPE, stderr=stderr_file,
            )
        except Exception:
            stderr_file.close()
            self._remove_stderr_file()
            raise
        else:
            stderr_file.close()
        # Blocking reads on purpose: the thread exists to sit in read() and
        # keep the pipe empty, which is what stops the decoder stalling.
        self._stop_reading.clear()
        self._latest = None
        self._reader = threading.Thread(
            target=self._read_loop, args=(self._process.stdout,), daemon=True)
        self._reader.start()

    def _remove_stderr_file(self) -> None:
        if self._stderr_path is not None:
            try:
                Path(self._stderr_path).unlink(missing_ok=True)
            except OSError:
                pass
            self._stderr_path = None

    def stop(self) -> None:
        if self._process is None:
            return
        # Terminate first: that closes the write end, so a reader thread
        # parked in read() gets EOF and returns instead of being joined
        # while it still waits for bytes that will never come.
        self._stop_reading.set()
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        if self._reader is not None:
            self._reader.join(timeout=3)
            self._reader = None
        self._process = None
        with self._lock:
            self._latest = None
        self._remove_stderr_file()

    def _read_loop(self, stdout) -> None:
        """Runs on the reader thread. Assembles whole frames and keeps only
        the newest, so the pipe never backs up behind a busy GUI."""
        buffer = bytearray()
        frame_size = self.frame_size
        while not self._stop_reading.is_set():
            try:
                chunk = stdout.read(frame_size - len(buffer))
            except (OSError, ValueError):
                # The pipe was closed under us by stop().
                break
            if not chunk:
                break
            buffer.extend(chunk)
            if len(buffer) < frame_size:
                continue
            with self._lock:
                self._latest = bytes(buffer)
            buffer.clear()

    def read_frame(self) -> bytes | None:
        """The newest complete frame, or None if none has arrived since the
        last call. Never a partial frame - a short read would tear the image
        and desynchronize every frame after it - and never an old one, since
        a queued frame on a live view is only latency."""
        with self._lock:
            frame, self._latest = self._latest, None
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
                # The reader thread owns the pipe now, so there is nothing
                # to block on here - without this sleep the loop is a bare
                # spin on one core, and it would steal time from the thread
                # that actually matters.
                time.sleep(0.005)
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
