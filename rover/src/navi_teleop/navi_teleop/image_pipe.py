"""The stdin-fed half of video_sender: frames arrive as sensor_msgs/Image
from the ZED wrapper and go into gst-launch through a pipe.

This is the front end the simulation's sender uses, carried over because it
was the one verified to frame a byte stream correctly: fdsrc emits
fixed-size chunks with no idea where a frame ends, blocksize makes each
chunk one frame, and rawvideoparse is the element that actually cuts the
stream - a capsfilter only asserts. Pure functions, so the argv and the
frame check are testable without a camera or ROS.
"""

from navi_teleop.video_request import VideoRequest

_FORMATS = {"bgra8": ("bgra", 4), "rgb8": ("rgb", 3), "bgr8": ("bgr", 3)}


def gst_format_for(encoding: str) -> str:
    try:
        return _FORMATS[encoding][0]
    except KeyError:
        raise ValueError(f"unsupported image encoding {encoding!r}; "
                         f"expected one of {sorted(_FORMATS)}") from None


def bytes_per_pixel(encoding: str) -> int:
    gst_format_for(encoding)
    return _FORMATS[encoding][1]


def build_pipe_pipeline(host: str, port: int, width: int, height: int,
                        fps: int, bitrate_kbps: int, encoding: str) -> list[str]:
    frame_bytes = width * height * bytes_per_pixel(encoding)
    return [
        "gst-launch-1.0", "-q",
        "fdsrc", "fd=0", f"blocksize={frame_bytes}",
        "!", "rawvideoparse", f"width={width}", f"height={height}",
        f"format={gst_format_for(encoding)}", f"framerate={fps}/1",
        "!", "videoconvert",
        "!", "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
        f"bitrate={bitrate_kbps}", "key-int-max=30",
        "!", "rtph264pay", "config-interval=1", "pt=96",
        "!", "udpsink", f"host={host}", f"port={port}",
    ]


def frame_matches(msg_width: int, msg_height: int, msg_encoding: str,
                  msg_len: int, request: VideoRequest) -> str | None:
    """None if this image can go into a pipeline built for `request`;
    otherwise the reason it cannot. Rescaling here would hide a
    misconfiguration behind a picture, so a mismatch is refused."""
    if (msg_width, msg_height) != (request.width, request.height):
        return (f"image is {msg_width}x{msg_height} but the request is for "
                f"{request.width}x{request.height}")
    try:
        expected = msg_width * msg_height * bytes_per_pixel(msg_encoding)
    except ValueError as exc:
        return str(exc)
    if msg_len != expected:
        return f"image has {msg_len} bytes, expected {expected} for {msg_encoding}"
    return None
