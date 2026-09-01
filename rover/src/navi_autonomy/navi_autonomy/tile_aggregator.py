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

Alongside `elevation`, /autonomy/map carries an `age_s` layer once any tile
has ever reported one: seconds since each cell's observation, as of *this*
publish. A tile sits in `_tile_age` between arriving and going out again,
so its own age_s (which was correct only at the moment it was published by
the mapper) needs the time spent waiting here added on top - a tile that
arrived 40 s ago reporting age_s=2 describes an observation that is now 42 s
old, not 2. Backwards compatible by construction, not by a special case:
a tile from a publisher that predates this feature carries no `age_s`
layer at all (see grid_map_io.tile_from_message), is cached as "no age
known" for that tile, and contributes NaN to the composed layer for its
cells - and if *no* tile has ever reported an age, the whole `age_s` layer
is left out of /autonomy/map, exactly as it always was before this
existed.

The per-tick cost here is build_grid_map on the full 960 x 960 window:
measured 2026-08-31 on the laptop (Intel i7-9750H, 6c/12t) at 26.7 ms,
comfortably inside the 1.0 s publish_period_s default (see
autonomy_perception.launch.py's docstring for the full three-number figure
and the Orin caveat).
"""

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import AGE_LAYER, ELEVATION_LAYER, build_grid_map, tile_from_message
from navi_autonomy.window import WINDOW_CELLS, RollingWindow
from navi_localization.elevation_grid import RESOLUTION
from navi_localization.elevation_mapper import TILE_QUEUE_DEPTH
from navi_localization.tiles import TILE_CELLS

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
        # key (tx, ty) -> (own 50x50 float64 age_s at arrival, arrival
        # time) or None ("this tile has never reported an age"). Bounded
        # by the same 60 m / 2.5 m = 24-tiles-per-axis cap
        # navi_localization.elevation_grid enforces on the map itself
        # (about 576 tiles at the cap), so this never grows without limit
        # even over a long-running mission - it is not pruned when a tile
        # leaves the rolling window, since _compose_age only ever reads
        # the entries that still land inside the window's current bounds.
        self._tile_age = {}
        # Once True, stays True: /autonomy/map gains the age_s layer the
        # first time any tile ever reports one, and keeps publishing it
        # from then on rather than having the layer flicker in and out of
        # the message schema tick to tick.
        self._age_layer_active = False

        self._map_publisher = self.create_publisher(
            GridMap, str(self.get_parameter('map_topic').value), latched_qos())
        self.create_subscription(
            GridMap, str(self.get_parameter('map_tile_topic').value), self._on_tile,
            tile_subscription_qos(self.tile_queue_depth))
        self.create_subscription(
            Odometry, str(self.get_parameter('pose_topic').value), self._on_pose, 1)
        self.create_timer(float(self.get_parameter('publish_period_s').value), self._tick)

    # -- inputs -----------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_tile(self, message: GridMap) -> None:
        try:
            elevation, age, ix, iy = tile_from_message(message)
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
        self._paste_age(ix, iy, age)
        self.tiles_received += 1

    def _paste_age(self, ix: int, iy: int, age) -> None:
        """Caches one tile's `age_s`, replaced wholesale like paste_tile
        does its elevation - including a `None` overwriting a previous
        real reading, since a node that reverted to an older build mid-
        mission must not go on reporting a stale "last known" age for a
        tile it no longer says anything about."""
        if age is None:
            self._tile_age[(ix, iy)] = None
            return
        self._age_layer_active = True
        own = np.asarray(age, dtype=np.float64)[:TILE_CELLS, :TILE_CELLS]
        self._tile_age[(ix, iy)] = (own.copy(), self._now())

    def _compose_age(self) -> np.ndarray:
        """The window's `age_s` layer right now: each cached tile's own
        age_s plus however long it has sat in `_tile_age` since arrival -
        see the module docstring for why cache residence has to be added
        back on. Reuses RollingWindow's own `_clip` (rather than
        re-deriving the same corner arithmetic here) so a tile's placement
        in this layer can never quietly drift from where paste_tile puts
        its elevation."""
        out = np.full((self.window.cells, self.window.cells), np.nan, dtype=np.float32)
        now = self._now()
        for (tx, ty), entry in self._tile_age.items():
            if entry is None:
                continue
            own_age, arrival = entry
            x0 = TILE_CELLS * tx - self.window.origin_ix
            y0 = TILE_CELLS * ty - self.window.origin_iy
            got = self.window._clip(y0, x0, own_age)
            if got is None:
                continue
            dst_y, dst_x, src_y, src_x = got
            out[dst_y, dst_x] = (own_age[src_y, src_x] + (now - arrival)).astype(np.float32)
        return out

    def _on_pose(self, message: Odometry) -> None:
        self._pose = (float(message.pose.pose.position.x),
                      float(message.pose.pose.position.y))

    # -- output -----------------------------------------------------------

    def _tick(self) -> None:
        if self.tiles_received == 0:
            return                      # nothing to say yet; do not publish an empty map
        if self._pose is not None:
            self.window.recentre(*self._pose)
        layers = {ELEVATION_LAYER: self.window.elevation}
        if self._age_layer_active:
            layers[AGE_LAYER] = self._compose_age()
        self._map_publisher.publish(build_grid_map(
            layers, self.window.origin_ix, self.window.origin_iy, self.window.resolution,
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
