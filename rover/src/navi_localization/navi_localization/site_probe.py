"""Stage 2 of site anchor: the rover answers one operator click at a time.

The ground station sends a pixel it wants ranged (`/site/probe_request`,
SS3.4); this node reads the latest depth image at that pixel, corrects the
range for the pole-axis offset of D4 and publishes the map-frame point
(`/site/probe_result`). All of the geometry lives in
`navi_localization.landmark_geometry` (pure, no ROS); this file is the thin
rclpy shell around it, the same split `localization_status.py` uses.

Deliberately does not lean on OpenCV's ROS image-conversion bridge package:
the ZED's depth image is 32FC1 (32-bit float, one channel), and decoding
that is a strided read of a float buffer with `struct.unpack_from`,
honouring the message's own `step` and `is_bigendian` - not a reason to
pull in a heavyweight bridge dependency that would also make this node
untestable without it. No vendor-specific camera messages either: the
depth image, camera info, pose and status messages are all stock ROS types
(`sensor_msgs`, `nav_msgs`, `std_msgs`), which is what keeps this module
laptop-safe with nothing but ROS sourced - see test_site_probe.py.

`min_depth_m` / `max_depth_m` mirror zed_front.yaml's `depth.min_depth` /
`depth.max_depth`: the SDK returns nothing outside that range, so a wider
filter here buys nothing and a narrower one silently throws away good
returns. If that config changes, these parameters change with it.

A note on `/localization/status`'s `state` field: `tracker.py` publishes
the literal string `"OK"` (uppercase - `LocalizationTracker.OK = "OK"`),
not the lower-case `"ok"` an earlier draft of this plan assumed. The check
below compares case-insensitively so it is correct either way the wire
value is cased.
"""

import json
import struct

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from navi_localization.landmark_geometry import (
    Intrinsics, apply_face_offset, depth_to_range, landmark_point_in_map,
    median_depth, rescale_pixel)
from navi_localization.pose_composition import Transform

_SUPPORTED_DEPTH_ENCODING = "32FC1"


def _safe_str(value):
    return value if isinstance(value, str) and value else None


def _safe_number(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _safe_int(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return None


def _clamp_patch_px(value) -> int:
    """Odd, in [1, 51] - the same clamp `probe_request_json` applies at the
    wire boundary on the ground-station side, applied again here because
    this node must not trust a client's arithmetic with an index into its
    own depth buffer."""
    try:
        n = int(value)
    except (TypeError, ValueError):
        n = 11
    n = max(1, min(51, n))
    if n % 2 == 0:
        n = min(51, n + 1)
    return n


def parse_probe_request(payload: str):
    """A strict `/site/probe_request` (SS3.4): `request_id`, `label`, `u`,
    `v`, `width` and `height` are all required, and any one of them missing
    or wrong-typed fails the WHOLE payload (returns `None`) rather than
    reading a pixel nobody clicked. `target` and `patch_px` are optional
    and take the documented defaults; an unrecognised `target` is treated
    as the default `"pole"` rather than failing the payload, since the
    request still names a real pixel to range."""
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(data, dict):
        return None

    request_id = _safe_str(data.get("request_id"))
    label = _safe_str(data.get("label"))
    u = _safe_number(data.get("u"))
    v = _safe_number(data.get("v"))
    width = _safe_int(data.get("width"))
    height = _safe_int(data.get("height"))
    if None in (request_id, label, u, v, width, height):
        return None
    if width <= 0 or height <= 0:
        return None

    target = data.get("target", "pole")
    if target not in ("pole", "box_face"):
        target = "pole"
    patch_px = _clamp_patch_px(data.get("patch_px", 11))

    return {
        "request_id": request_id, "label": label, "u": u, "v": v,
        "width": width, "height": height, "target": target,
        "patch_px": patch_px,
    }


def intrinsics_from_camera_info(msg: CameraInfo) -> Intrinsics:
    k = msg.k
    return Intrinsics(fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                      width=msg.width, height=msg.height)


def pose_transform_from_odometry(msg: Odometry) -> Transform:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def decode_depth_patch(image: Image, u: float, v: float, patch_px: int):
    """The `patch_px` x `patch_px` window of z-depth values (metres)
    centred on pixel (u, v) of `image`, clipped to the image bounds. Only
    `32FC1` (the ZED's native depth encoding) is supported; anything else
    raises `ValueError` naming the encoding it actually got."""
    if image.encoding != _SUPPORTED_DEPTH_ENCODING:
        raise ValueError(f"unsupported depth encoding '{image.encoding}'")
    fmt = ">f" if image.is_bigendian else "<f"
    half = patch_px // 2
    u0 = max(0, int(round(u)) - half)
    u1 = min(image.width, int(round(u)) + half + 1)
    v0 = max(0, int(round(v)) - half)
    v1 = min(image.height, int(round(v)) + half + 1)
    values = []
    for row in range(v0, v1):
        base = row * image.step
        for col in range(u0, u1):
            offset = base + col * 4
            values.append(struct.unpack_from(fmt, image.data, offset)[0])
    return values


class SiteProbe(Node):
    """Answers `/site/probe_request` with exactly one `/site/probe_result`
    each - success or failure, request_id echoed - by reading the latest
    depth image at the requested pixel. Stateless between requests beyond
    "what was the most recent depth image / camera info / pose / status":
    the rover is stationary during anchoring, so there is no queue and no
    time synchronisation across the four inputs."""

    def __init__(self) -> None:
        super().__init__('site_probe')
        self.declare_parameter(
            'depth_topic', '/zed_front/zed_node/depth/depth_registered')
        self.declare_parameter(
            'camera_info_topic', '/zed_front/zed_node/depth/camera_info')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('status_topic', '/localization/status')
        self.declare_parameter('request_topic', '/site/probe_request')
        self.declare_parameter('result_topic', '/site/probe_result')
        # Mirrors zed_front.yaml's depth.min_depth / depth.max_depth - the
        # SDK returns nothing outside them, so keep these two in step with
        # that config if it ever changes.
        self.declare_parameter('min_depth_m', 0.3)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('min_valid_fraction', 0.25)
        self.declare_parameter('face_offset_m', 0.125)
        self.declare_parameter('require_localisation_ok', True)

        self._depth_msg = None
        self._camera_info = None
        self._pose = None
        self._localisation_ok = None

        self._result_publisher = self.create_publisher(
            String, self.get_parameter('result_topic').value, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, 10)
        self.create_subscription(
            Odometry, self.get_parameter('pose_topic').value, self._on_pose, 10)
        self.create_subscription(
            String, self.get_parameter('status_topic').value, self._on_status, 10)
        self.create_subscription(
            String, self.get_parameter('request_topic').value, self._on_request, 10)

    def _on_depth(self, msg: Image) -> None:
        self._depth_msg = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_pose(self, msg: Odometry) -> None:
        self._pose = pose_transform_from_odometry(msg)

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(data, dict):
            return
        state = data.get("state")
        if isinstance(state, str):
            self._localisation_ok = state.strip().upper() == "OK"

    def _on_request(self, msg: String) -> None:
        request = parse_probe_request(msg.data)
        if request is None:
            # The only silent case: without a trustworthy request_id there
            # is nothing to reply to, and a half-parsed pixel is worse than
            # no reply at all.
            self.get_logger().warning(
                "dropped an unreadable /site/probe_request")
            return
        self._handle_request(request)

    def _fail(self, request_id, label, error, samples=0, valid_fraction=0.0):
        self._publish(request_id, False, label, error=error,
                      samples=samples, valid_fraction=valid_fraction)

    def _handle_request(self, request: dict) -> None:
        request_id = request["request_id"]
        label = request["label"]

        if self._depth_msg is None or self._camera_info is None:
            self._fail(request_id, label, "no depth image yet")
            return
        if self._pose is None:
            self._fail(request_id, label, "no rover pose yet")
            return
        require_ok = bool(self.get_parameter('require_localisation_ok').value)
        if require_ok and self._localisation_ok is not True:
            self._fail(request_id, label, "localisation is not OK")
            return

        u, v = request["u"], request["v"]
        width, height = request["width"], request["height"]
        if not (0 <= u < width and 0 <= v < height):
            self._fail(request_id, label, "pixel is outside the image")
            return

        depth_msg = self._depth_msg
        u_d, v_d = rescale_pixel(u, v, (width, height),
                                 (depth_msg.width, depth_msg.height))
        try:
            patch = decode_depth_patch(depth_msg, u_d, v_d, request["patch_px"])
        except ValueError as exc:
            self._fail(request_id, label, str(exc))
            return

        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        median, samples, valid_fraction = median_depth(patch, min_depth, max_depth)
        min_valid_fraction = float(self.get_parameter('min_valid_fraction').value)
        if median is None or valid_fraction < min_valid_fraction:
            self._fail(request_id, label, "no valid depth at that pixel",
                      samples=samples, valid_fraction=valid_fraction)
            return

        intr = intrinsics_from_camera_info(self._camera_info)
        if (intr.width, intr.height) != (depth_msg.width, depth_msg.height):
            intr = intr.scaled_to(depth_msg.width, depth_msg.height)

        target = request["target"]
        offset_m = (float(self.get_parameter('face_offset_m').value)
                    if target == "box_face" else 0.0)
        x, y, z = landmark_point_in_map(u_d, v_d, median, intr, self._pose,
                                        offset_m=offset_m)
        range_m = apply_face_offset(depth_to_range(u_d, v_d, median, intr), offset_m)

        self._publish(request_id, True, label, x=x, y=y, z=z, range_m=range_m,
                     samples=samples, valid_fraction=valid_fraction)

    def _publish(self, request_id, ok, label, x=None, y=None, z=None,
                range_m=None, samples=0, valid_fraction=0.0, error=None) -> None:
        payload = {
            "request_id": request_id,
            "ok": ok,
            "label": label,
            "x": x, "y": y, "z": z,
            "frame_id": "map",
            "range_m": range_m,
            "samples": samples,
            "valid_fraction": valid_fraction,
            "stamp_s": self.get_clock().now().nanoseconds / 1e9,
            "error": error,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._result_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SiteProbe()
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
