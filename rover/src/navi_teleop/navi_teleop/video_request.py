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
