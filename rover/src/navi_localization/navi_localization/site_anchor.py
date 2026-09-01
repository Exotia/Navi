"""Stage 3 of site-anchor: the ArUco anchor phase (§3.5 of the site-anchor plan).

The operator points the front ZED at two or more ERC landmarks (each a
250x250x310 mm box carrying the same 5x5 ArUco face on all four sides) and
this node accumulates map-frame landmark positions while the ground station
watches the running median and its spread. It never estimates the marker's
orientation (D3) and never drives anything: `mode_supervisor` is the sole
publisher of `/rover_twist` in this repo, and the "optional slow point-turn
sweep" the brief imagined is an operator action with the gamepad, not a
motion command from here (correction 3). This node creates no publisher on
any twist or mode/drive command topic - test_site_anchor.py enumerates its
publishers and asserts that.

The pixel + depth arithmetic - ray casting, the z-depth-to-range conversion,
and the 0.125 m pole-axis offset that D4/correction 4 collapse into a range
correction along the camera-to-marker ray - is shared with stage 2's manual
probe and lives in `navi_localization.landmark_geometry`, along with the
per-id accumulator that produces the median position, the robust spread
(`spread_m`) and the good/weak/noisy quality call. This module's own job is
the phase machine, the wiring from ROS messages into that arithmetic, and
the JSON report.

`cv2` is only needed to actually find markers in an image, so it is imported
lazily in `__init__`, wrapped in a `try`: a laptop with no OpenCV, or a
dictionary name `cv2.aruco` does not define, still constructs this node and
still publishes reports - with `detector_ok: false` and a sentence in
`error` - rather than crashing the process that would otherwise tell the
operator why nothing is being seen. `_detect` is the only method that
touches cv2; tests replace it with a stub and never need OpenCV installed.
"""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from navi_localization.landmark_geometry import (
    Intrinsics, SightingAccumulator, apply_face_offset, depth_to_range,
    landmark_point_in_map, median_depth)
from navi_localization.pose_composition import Transform

IDLE = "idle"
RUNNING = "running"
STOPPED = "stopped"

# Half-width, in pixels, of the square patch of depth samples read around a
# detected marker's centroid. A single pixel of a stereo depth map is noisy
# enough on its own that median_depth's job - reject NaN/inf/out-of-range
# and take the median of what is left - needs more than one sample to work.
DEPTH_PATCH_RADIUS = 3

# sensor_msgs/Image encodings this node can hand to the ArUco detector, and
# how many bytes each pixel takes. The channel count is read from the
# encoding rather than inferred from the buffer length: a row-padded image
# (step > width * channels, which the ZED wrapper does not send today but
# any republisher may) makes `len(data) // (width * height)` a wrong answer
# that silently reshapes the picture into diagonal garbage.
_IMAGE_CHANNELS = {
    'mono8': 1, '8UC1': 1,
    'bgr8': 3, 'rgb8': 3,
    'bgra8': 4, 'rgba8': 4,
}
_MONO_ENCODINGS = ('mono8', '8UC1')
# Looked up lazily against the cv2 module so importing this file never
# needs OpenCV (the detector-unavailable path has to keep publishing).
_GRAY_CONVERSIONS = {
    'bgr8': lambda cv2: cv2.COLOR_BGR2GRAY,
    'rgb8': lambda cv2: cv2.COLOR_RGB2GRAY,
    'bgra8': lambda cv2: cv2.COLOR_BGRA2GRAY,
    'rgba8': lambda cv2: cv2.COLOR_RGBA2GRAY,
}


def image_to_array(image_msg):
    """`(height, width, channels)` uint8 view of a sensor_msgs/Image, plus
    its encoding - or None if the encoding is not one this node handles or
    the buffer does not match the header.

    The row stride is `step` BYTES, so the buffer is reshaped to
    `(height, step)` and each row trimmed to `width * channels` bytes
    before the channel axis is split out. Reshaping to anything derived
    from `step // channels` is a size error on every multi-channel image -
    which is what this function exists to keep from happening again.
    """
    import numpy as np

    channels = _IMAGE_CHANNELS.get(image_msg.encoding)
    if channels is None:
        return None
    w, h, step = image_msg.width, image_msg.height, image_msg.step
    if w <= 0 or h <= 0 or step < w * channels:
        return None
    buf = np.frombuffer(image_msg.data, dtype=np.uint8)
    if buf.size < h * step:
        return None
    arr = buf[:h * step].reshape(h, step)[:, :w * channels]
    return arr.reshape(h, w, channels), image_msg.encoding


def _median(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2 == 1:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


class SiteAnchor(Node):

    def __init__(self) -> None:
        super().__init__('site_anchor')

        self.declare_parameter('image_topic', '/zed_front/zed_node/left/image_rect_color')
        self.declare_parameter('depth_topic', '/zed_front/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed_front/zed_node/depth/camera_info')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('status_topic', '/localization/status')
        self.declare_parameter('command_topic', '/site/anchor_command')
        self.declare_parameter('sightings_topic', '/site/landmark_sightings')
        self.declare_parameter('dictionary', 'DICT_5X5_100')
        self.declare_parameter('marker_edge_m', 0.150)
        self.declare_parameter('face_offset_m', 0.125)
        self.declare_parameter('max_samples', 150)
        self.declare_parameter('min_samples', 50)
        self.declare_parameter('spread_warn_m', 0.15)
        self.declare_parameter('report_interval_s', 1.0)
        self.declare_parameter('detect_interval_s', 0.2)
        self.declare_parameter('min_depth_m', 0.3)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('range_source', 'depth')

        self._dictionary_name = str(self.get_parameter('dictionary').value)
        self._marker_edge_m = float(self.get_parameter('marker_edge_m').value)
        self._face_offset_m = float(self.get_parameter('face_offset_m').value)
        self._max_samples = int(self.get_parameter('max_samples').value)
        self._min_samples = int(self.get_parameter('min_samples').value)
        self._spread_warn_m = float(self.get_parameter('spread_warn_m').value)
        self._report_interval_s = float(self.get_parameter('report_interval_s').value)
        self._detect_interval_s = float(self.get_parameter('detect_interval_s').value)
        self._min_depth_m = float(self.get_parameter('min_depth_m').value)
        self._max_depth_m = float(self.get_parameter('max_depth_m').value)
        self._range_source = str(self.get_parameter('range_source').value)

        self._phase = IDLE
        self._accumulator = SightingAccumulator(
            max_samples=self._max_samples, min_samples=self._min_samples,
            spread_warn_m=self._spread_warn_m)
        # Parallel to the accumulator, keyed the same way, so the report can
        # carry a representative range_m too - AccumulatedSighting (T3) does
        # not carry one, it is not part of the site-frame fit.
        self._range_samples = {}

        self._intrinsics = None
        self._image_size = (0, 0)
        self._footprint_in_map = None
        self._depth = None
        self._depth_size = (0, 0)
        self._last_detect_t = None

        self._detector_ok = True
        self._detector_error = None
        self._aruco_dict = None
        self._setup_detector()

        image_topic = str(self.get_parameter('image_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        sightings_topic = str(self.get_parameter('sightings_topic').value)

        self._sightings_publisher = self.create_publisher(String, sightings_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(Image, depth_topic, self._on_depth, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Odometry, pose_topic, self._on_pose, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_timer(self._report_interval_s, self._on_report_timer)

        if not self._detector_ok:
            self.get_logger().error(f"ArUco detector unavailable: {self._detector_error}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- detector setup --------------------------------------------------

    def _setup_detector(self) -> None:
        """cv2 is imported here, and only here besides `_detect` - never at
        module import - so this node constructs on a machine with no OpenCV
        at all. Any failure sets detector_ok False with a sentence naming
        why, and never raises: a node that failed to construct tells the
        operator nothing, a report that says detector_ok:false does."""
        try:
            import cv2
        except Exception:
            self._detector_ok = False
            self._detector_error = "cv2.aruco not available"
            return
        if not hasattr(cv2, 'aruco'):
            self._detector_ok = False
            self._detector_error = "cv2.aruco not available"
            return
        if not hasattr(cv2.aruco, self._dictionary_name):
            self._detector_ok = False
            self._detector_error = f"unknown ArUco dictionary '{self._dictionary_name}'"
            return
        try:
            dict_id = getattr(cv2.aruco, self._dictionary_name)
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            self._detector_ok = True
            self._detector_error = None
        except Exception as exc:
            self._detector_ok = False
            self._detector_error = f"unknown ArUco dictionary '{self._dictionary_name}': {exc}"

    def _detect(self, image_msg) -> list:
        """[(id, u, v)] - pixel centroid of each detected marker's four
        corners. The only method that touches cv2; tests replace it with a
        stub returning known ids and pixels, so nothing else in this node
        needs OpenCV installed to be exercised."""
        if not self._detector_ok or self._aruco_dict is None:
            return []
        import cv2

        decoded = image_to_array(image_msg)
        if decoded is None:
            return []
        arr, encoding = decoded
        gray = arr[:, :, 0] if encoding in _MONO_ENCODINGS else cv2.cvtColor(
            arr, _GRAY_CONVERSIONS[encoding](cv2))
        params = cv2.aruco.DetectorParameters_create() if hasattr(
            cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=params)
        out = []
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(-1, 2)
                u = float(pts[:, 0].mean())
                v = float(pts[:, 1].mean())
                out.append((str(int(marker_id)), u, v))
        return out

    # -- subscriptions -----------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        self._intrinsics = Intrinsics(k[0], k[4], k[2], k[5], msg.width, msg.height)
        self._image_size = (msg.width, msg.height)

    def _on_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._footprint_in_map = Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _on_depth(self, msg: Image) -> None:
        import numpy as np
        if msg.encoding not in ('32FC1', '32fc1'):
            return
        row_floats = msg.step // 4
        arr = np.frombuffer(msg.data, dtype=np.float32)
        if row_floats * msg.height != arr.size:
            return
        arr = arr.reshape(msg.height, row_floats)[:, :msg.width]
        self._depth = arr
        self._depth_size = (msg.width, msg.height)

    def _on_image(self, msg: Image) -> None:
        if self._phase != RUNNING:
            return
        now = self._now()
        if (self._last_detect_t is not None
                and now - self._last_detect_t < self._detect_interval_s):
            return
        self._last_detect_t = now
        if self._intrinsics is None or self._footprint_in_map is None or self._depth is None:
            return
        for marker_id, u, v in self._detect(msg):
            self._process_detection(str(marker_id), u, v, now)

    def _on_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            action = data.get('action') if isinstance(data, dict) else None
        except (ValueError, AttributeError):
            return
        if action == 'start':
            self._phase = RUNNING
            self._publish_report()
        elif action == 'stop':
            self._phase = STOPPED
            self._publish_report()
        elif action == 'reset':
            self._accumulator.reset()
            self._range_samples = {}
            self._publish_report()
        # Anything else - unknown action, or well-formed JSON that is not a
        # dict - is ignored outright: garbage on this topic must not change
        # the phase.

    def _on_report_timer(self) -> None:
        if self._phase != IDLE:
            self._publish_report()

    # -- measurement --------------------------------------------------------

    def _depth_patch(self, u: float, v: float) -> list:
        w, h = self._depth_size
        ui, vi = int(round(u)), int(round(v))
        r = DEPTH_PATCH_RADIUS
        u0, u1 = max(0, ui - r), min(w, ui + r + 1)
        v0, v1 = max(0, vi - r), min(h, vi + r + 1)
        if u1 <= u0 or v1 <= v0:
            return []
        return self._depth[v0:v1, u0:u1].flatten().tolist()

    def _process_detection(self, marker_id: str, u: float, v: float, now: float) -> None:
        patch = self._depth_patch(u, v)
        depth_m, _n_valid, _frac = median_depth(patch, self._min_depth_m, self._max_depth_m)
        if depth_m is None:
            # No valid depth under this detection - skipped outright, never
            # guessed at (§Task 8, test 8): a marker with no depth support
            # contributes nothing rather than a fabricated range.
            return
        intr = self._intrinsics
        point = landmark_point_in_map(
            u, v, depth_m, intr, self._footprint_in_map, offset_m=self._face_offset_m)
        range_m = apply_face_offset(depth_to_range(u, v, depth_m, intr), self._face_offset_m)
        self._accumulator.add(marker_id, point[0], point[1], point[2], now)
        samples = self._range_samples.setdefault(marker_id, [])
        samples.append(range_m)
        if len(samples) > self._max_samples:
            del samples[0]

    # -- reporting ------------------------------------------------------

    def _build_report(self) -> dict:
        now = self._now()
        sightings = []
        for s in self._accumulator.snapshot(now):
            sightings.append({
                'id': s.id,
                'x': s.x,
                'y': s.y,
                'z': s.z,
                'n': s.n,
                'spread_m': s.spread_m,
                'range_m': _median(self._range_samples.get(s.id, [])),
                'last_seen_s': s.last_seen_s,
                'quality': s.quality,
            })
        return {
            'stamp_s': now,
            'phase': self._phase,
            'frame_id': 'map',
            'dictionary': self._dictionary_name,
            'image_size': [self._image_size[0], self._image_size[1]],
            'detector_ok': self._detector_ok,
            'error': self._detector_error,
            'sightings': sightings,
        }

    def _publish_report(self) -> None:
        msg = String()
        msg.data = json.dumps(self._build_report())
        self._sightings_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SiteAnchor()
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
