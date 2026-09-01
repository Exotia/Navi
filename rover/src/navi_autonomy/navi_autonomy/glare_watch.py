"""glare_watch: turns ZED frames into a glare verdict for goal_relay's tack.

The ZED's visual odometry loses tracking when the sun saturates the frame
into the auto-exposure death spiral described in navi_autonomy.glare's own
docstring; this node is the ROS wiring around that pure arithmetic. Its job
is threefold: decode whatever encoding the camera happens to be publishing,
keep the comparatively expensive numpy reduction off the camera's real
15 Hz, and publish only what changed - plus an occasional heartbeat, so a
goal_relay or ground station that starts late still learns the current
verdict without waiting for the sun to move.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_glare_watch.py -q'
"""

import json
from time import monotonic

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from navi_autonomy import glare

# A verdict is republished the moment it changes, but at least this often
# regardless of change: the payload here is derived fresh per frame rather
# than retained by the middleware (unlike /autonomy/active_goal's latched
# QoS), so a late subscriber has no way to learn the current side except by
# waiting for the next publish - this bounds that wait.
HEARTBEAT_S = 5.0

# sensor_msgs/Image encodings this node can turn into saturated_fractions'
# input, and how many bytes each pixel takes. This table (and the function
# below) duplicates navi_localization.site_anchor.image_to_array rather than
# importing it: navi_autonomy must not gain a dependency on navi_localization
# for ten lines of buffer arithmetic, and the two nodes decode the same wire
# format for unrelated reasons (ArUco detection there, glare fractions here)
# that have no reason to stay coupled just because they happen to agree
# today.
_IMAGE_CHANNELS = {
    'bgra8': 4, 'bgr8': 3, 'rgb8': 3, 'rgba8': 4, 'mono8': 1,
}


def image_to_array(image_msg):
    """`(height, width, channels)` uint8 view of a sensor_msgs/Image, or None
    if the encoding is not one this node handles or the buffer does not
    match the header.

    The row stride is `step` BYTES, so the buffer is reshaped to
    `(height, step)` and each row trimmed to `width * channels` bytes before
    the channel axis is split out - reshaping from `step // channels`
    instead is a size error on any row-padded image, the same fix
    site_anchor.image_to_array carries for the same reason.
    """
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
    return arr.reshape(h, w, channels)


class GlareWatch(Node):

    def __init__(self, clock=monotonic):
        super().__init__('glare_watch')
        self.declare_parameter(
            'image_topic', '/zed_front/zed_node/left/image_rect_color')
        self.declare_parameter('glare_topic', '/autonomy/glare')
        self.declare_parameter('saturation_level', glare.SATURATION_LEVEL)
        self.declare_parameter('glare_fraction', glare.GLARE_FRACTION)
        self.declare_parameter('glare_margin', glare.GLARE_MARGIN)
        self.declare_parameter('max_rate_hz', 2.0)

        # Not self._clock: rclpy.node.Node already owns that name (see
        # mode_supervisor.py's and goal_relay.py's identical note).
        self._now = clock
        self._saturation_level = int(self.get_parameter('saturation_level').value)
        self._glare_fraction = float(self.get_parameter('glare_fraction').value)
        self._glare_margin = float(self.get_parameter('glare_margin').value)
        max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        # A misconfigured 0 or negative rate means "no limit" rather than a
        # divide-by-zero that would take the whole node down with it.
        self._min_period_s = 1.0 / max_rate_hz if max_rate_hz > 0.0 else 0.0

        self._last_processed_t = None
        self._last_side = None
        self._last_published_t = None
        # Set the first time an encoding cannot be decoded, so the warning
        # fires once per node lifetime rather than at up to 15 Hz - the same
        # idiom as elevation_mapper's _warned_about_cap.
        self._warned_bad_encoding = False

        self._glare_pub = self.create_publisher(
            String, str(self.get_parameter('glare_topic').value), 10)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        now = self._now()
        if (self._last_processed_t is not None
                and now - self._last_processed_t < self._min_period_s):
            # Dropped before touching the pixel data: at the camera's real
            # 15 Hz this callback would otherwise decode and reduce a
            # 640x360 frame seven times for every one glare_side actually
            # needs at the default 2 Hz.
            return
        self._last_processed_t = now

        array = image_to_array(msg)
        if array is None:
            if not self._warned_bad_encoding:
                self._warned_bad_encoding = True
                self.get_logger().warn(
                    f"cannot decode image encoding {msg.encoding!r}; "
                    "ignoring frames on this topic")
            return

        left_fraction, right_fraction = glare.saturated_fractions(
            array, self._saturation_level)
        side = glare.glare_side(
            left_fraction, right_fraction, self._glare_fraction, self._glare_margin)

        changed = side != self._last_side
        heartbeat_due = (self._last_published_t is None
                        or now - self._last_published_t >= HEARTBEAT_S)
        if not changed and not heartbeat_due:
            return

        self._last_side = side
        self._last_published_t = now
        payload = {
            # The wire never carries a JSON null here: the consumer
            # (goal_relay) parses this field strictly, and "none" is a
            # value it can match against without special-casing null.
            "side": side if side is not None else "none",
            "left_fraction": left_fraction,
            "right_fraction": right_fraction,
        }
        out = String()
        out.data = json.dumps(payload)
        self._glare_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GlareWatch()
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
