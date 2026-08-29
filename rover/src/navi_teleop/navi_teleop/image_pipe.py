"""The stdin-fed half of video_sender: frames arrive as sensor_msgs/Image
from the ZED wrapper and go into gst-launch through a pipe.

This is the front end the simulation's sender uses, carried over because it
was the one verified to frame a byte stream correctly: fdsrc emits
fixed-size chunks with no idea where a frame ends, blocksize makes each
chunk one frame, and rawvideoparse is the element that actually cuts the
stream - a capsfilter only asserts. Pure functions, so the argv and the
frame check are testable without a camera or ROS.
"""

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


def unsupported_frame_reason(msg_width: int, msg_height: int,
                             msg_encoding: str, msg_len: int) -> str | None:
    """None if a pipeline can be built for this image; otherwise the reason
    it cannot.

    Geometry is deliberately not compared against the request. In
    `zed_topic` mode the wrapper owns the camera and its published size is
    the only size there is - the ground station cannot change it by asking,
    so the stream adopts what arrives and the request's width/height are
    advisory. What is still refused is an encoding this module has no
    GStreamer format for (there is no pipeline to build), and a byte count
    that disagrees with the frame's own header - a torn frame would
    desynchronise every frame after it.
    """
    try:
        expected = msg_width * msg_height * bytes_per_pixel(msg_encoding)
    except ValueError as exc:
        return str(exc)
    if msg_len != expected:
        return f"image has {msg_len} bytes, expected {expected} for {msg_encoding}"
    return None
