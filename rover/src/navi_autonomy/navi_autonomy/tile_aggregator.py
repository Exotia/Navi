"""The 2.5 m map tiles, stitched into one 48 m window around the rover.

Spec section 5: "tile_aggregator subscribes /localization/map_tile, stitches
tiles into a rolling window around the rover (48 m, so the 60 m map cap is
never the binding constraint), and publishes a whole-map GridMap for
downstream use. This is the piece the old spec assumed already existed."

Queue depth is the thing to get right here. The mapper can emit 25 tiles in a
single tick (8 dirty + 16 blanks + 1 keepalive), a map load marks all ~576
tiles dirty at once, and the start-of-run burst was measured on the Orin at
714 KB/s settling within ~10 s. A shallow subscription drops tiles silently,
and a dropped tile is indistinguishable from unseen ground in the window it
should have filled - so the depth here is exactly the mapper's own
TILE_QUEUE_DEPTH, imported rather than repeated. Durability must be volatile
to match the publisher; a durability mismatch means no data at all.

/autonomy/map is transient_local so traversability_layer may start after this
node and still get a map immediately, rather than waiting a tick.

The per-tick cost here is build_grid_map on the full 960 x 960 window:
measured 2026-08-31 on the laptop (Intel i7-9750H, 6c/12t) at 26.7 ms,
comfortably inside the 1.0 s publish_period_s default (see
autonomy_perception.launch.py's docstring for the full three-number figure
and the Orin caveat).
"""

import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import ELEVATION_LAYER, build_grid_map, tile_from_message
from navi_autonomy.window import WINDOW_CELLS, RollingWindow
from navi_localization.elevation_grid import RESOLUTION
from navi_localization.elevation_mapper import TILE_QUEUE_DEPTH

MAP_TILE_TOPIC = '/localization/map_tile'
POSE_TOPIC = '/localization/pose'
MAP_TOPIC = '/autonomy/map'


def tile_subscription_qos(depth: int = TILE_QUEUE_DEPTH) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def latched_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class TileAggregator(Node):

    def __init__(self):
        super().__init__('tile_aggregator')
        self.declare_parameter('map_tile_topic', MAP_TILE_TOPIC)
        self.declare_parameter('pose_topic', POSE_TOPIC)
        self.declare_parameter('map_topic', MAP_TOPIC)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('window_cells', WINDOW_CELLS)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('tile_queue_depth', TILE_QUEUE_DEPTH)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self.tile_queue_depth = int(self.get_parameter('tile_queue_depth').value)
        self.window = RollingWindow(cells=int(self.get_parameter('window_cells').value),
                                    resolution=RESOLUTION)
        self.tiles_received = 0
        self.rejected_tiles = 0
        self._pose = None
        self._rejected_logged = False

        self._map_publisher = self.create_publisher(
            GridMap, str(self.get_parameter('map_topic').value), latched_qos())
        self.create_subscription(
            GridMap, str(self.get_parameter('map_tile_topic').value), self._on_tile,
            tile_subscription_qos(self.tile_queue_depth))
        self.create_subscription(
            Odometry, str(self.get_parameter('pose_topic').value), self._on_pose, 1)
        self.create_timer(float(self.get_parameter('publish_period_s').value), self._tick)

    # -- inputs -----------------------------------------------------------

    def _on_tile(self, message: GridMap) -> None:
        try:
            elevation, ix, iy = tile_from_message(message)
        except ValueError as error:
            # A callback that raises takes the executor down with it, and a
            # mapper that changed its contract must be a log line, not a
            # dead node.
            self.rejected_tiles += 1
            if not self._rejected_logged:
                self._rejected_logged = True
                self.get_logger().warn(f"dropping map tiles: {error}")
            return
        self.window.paste_tile(ix, iy, elevation)
        self.tiles_received += 1

    def _on_pose(self, message: Odometry) -> None:
        self._pose = (float(message.pose.pose.position.x),
                      float(message.pose.pose.position.y))

    # -- output -----------------------------------------------------------

    def _tick(self) -> None:
        if self.tiles_received == 0:
            return                      # nothing to say yet; do not publish an empty map
        if self._pose is not None:
            self.window.recentre(*self._pose)
        self._map_publisher.publish(build_grid_map(
            {ELEVATION_LAYER: self.window.elevation},
            self.window.origin_ix, self.window.origin_iy, self.window.resolution,
            self._frame_id, self.get_clock().now().to_msg()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TileAggregator()
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
