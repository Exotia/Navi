"""slope, step, roughness and valid from the aggregated map, and the costmap
seed Nav2 plans on.

Spec section 5: "traversability_layer reads that map and derives slope, step,
roughness, valid ... Publishes /autonomy/traversability (GridMap, for the
view - it can also drive the pit colouring in the sim) and
/autonomy/costmap_seed (OccupancyGrid, latched)."

Event-driven, not on a timer: the map arrives at about 1 Hz and there is
nothing to recompute in between. The derive is ~150 ms at 960 x 960 on the
laptop (see traversability.derive).

The four-layer GridMap is 14.7 MB per message and nothing on the rover
subscribes to it - it is for the view and the sim - so it is built only when
someone is listening, the same count_subscribers guard the ZED wrapper uses
for its fused cloud. The 0.92 MB OccupancyGrid seed is what Nav2 reads and
always goes out, latched, so a Nav2 that starts later gets a map instantly
instead of planning on nothing.
"""

import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import (
    ELEVATION_LAYER, build_grid_map, build_occupancy_grid, layer_from_message)
from navi_autonomy.tile_aggregator import MAP_TOPIC, latched_qos
from navi_autonomy.traversability import seed_from_elevation
from navi_localization.elevation_grid import RESOLUTION

TRAVERSABILITY_TOPIC = '/autonomy/traversability'
COSTMAP_SEED_TOPIC = '/autonomy/costmap_seed'
LAYER_ORDER = ('slope', 'step', 'roughness', 'valid')


def view_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class TraversabilityLayer(Node):

    def __init__(self):
        super().__init__('traversability_layer')
        self.declare_parameter('map_topic', MAP_TOPIC)
        self.declare_parameter('traversability_topic', TRAVERSABILITY_TOPIC)
        self.declare_parameter('costmap_seed_topic', COSTMAP_SEED_TOPIC)
        self.declare_parameter('frame_id', 'map')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self.maps_processed = 0
        self.rejected_maps = 0
        self._rejected_logged = False

        self._traversability_topic = str(self.get_parameter('traversability_topic').value)
        self._traversability_publisher = self.create_publisher(
            GridMap, self._traversability_topic, view_qos())
        self._seed_publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('costmap_seed_topic').value),
            latched_qos())
        self.create_subscription(
            GridMap, str(self.get_parameter('map_topic').value), self._on_map,
            latched_qos())

    def _traversability_subscribers(self) -> int:
        return self.count_subscribers(self._traversability_topic)

    def _on_map(self, message: GridMap) -> None:
        resolution = float(message.info.resolution)
        try:
            if abs(resolution - RESOLUTION) > 1e-9:
                raise ValueError(
                    f"map resolution {resolution} is not {RESOLUTION}; this node "
                    "does not resample")
            elevation = layer_from_message(message, ELEVATION_LAYER)
        except ValueError as error:
            self.rejected_maps += 1
            if not self._rejected_logged:
                self._rejected_logged = True
                self.get_logger().warn(f"dropping /autonomy/map: {error}")
            return

        n_y, n_x = elevation.shape
        # grid_map gives the map's centre; the corner cell's lattice index is
        # half a map back, and it is what both output messages are anchored on.
        origin_ix = int(round(float(message.info.pose.position.x) / resolution - n_x / 2.0))
        origin_iy = int(round(float(message.info.pose.position.y) / resolution - n_y / 2.0))

        layers, cost = seed_from_elevation(elevation, resolution)
        stamp = message.header.stamp
        self._seed_publisher.publish(build_occupancy_grid(
            cost, origin_ix, origin_iy, resolution, self._frame_id, stamp))
        if self._traversability_subscribers() > 0:
            grid_map = build_grid_map(
                {name: layers[name] for name in LAYER_ORDER},
                origin_ix, origin_iy, resolution, self._frame_id, stamp)
            grid_map.basic_layers = ['valid']
            self._traversability_publisher.publish(grid_map)
        self.maps_processed += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TraversabilityLayer()
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
