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

"The wheels have been here": the rover starts on ground the camera has never
seen, and unknown = wall (correct, per spec) means Nav2 refuses to move at
all. The ground the rover is already sitting on at start-up is
proof-of-traversable, so this node also subscribes to the rover's pose (the
same source tile_aggregator uses), takes the *first* pose it sees as the
centre of one startup patch, and clears a disc of UNKNOWN cells there on
every published seed thereafter - never touching a measured cell, so a
camera-seen lethal still wins, permanently, the instant it is seen.

This is deliberately one fixed patch, not a disc that follows the rover
around as it drives (a "track"). The disc is wider than the wheels that
would supposedly prove it, so a moving disc would slowly "chip away"
(the operator's words) at unseen ground beside a big obstacle as the rover
drove past it - clearing ground the wheels never actually touched. One
startup patch avoids that: it proves exactly the ground the rover was
demonstrably on, once, at start-up. A node restart re-seeds the patch at
wherever the rover is then, which the wheels prove again.

Only the OccupancyGrid seed is touched; the GridMap layers stay exactly what
the elevation says, because elevation data is never faked.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import (
    ELEVATION_LAYER, build_grid_map, build_occupancy_grid, layer_from_message)
from navi_autonomy.tile_aggregator import MAP_TOPIC, POSE_TOPIC, latched_qos
from navi_autonomy.traversability import (SLOPE_LETHAL_DEG, STEP_LETHAL_M,
                                          clear_startup_patch, heal_goal_patch,
                                          seed_from_elevation, stamp_wheel_trail)
from navi_localization.elevation_grid import RESOLUTION

TRAVERSABILITY_TOPIC = '/autonomy/traversability'
COSTMAP_SEED_TOPIC = '/autonomy/costmap_seed'
LAYER_ORDER = ('slope', 'step', 'roughness', 'valid')

# The costmap's robot_radius (nav2_rover.yaml) is 0.80 m; the operator's
# margin on top of it is 10 cm. 0.90 m is the startup patch's disc radius.
STARTUP_CLEAR_RADIUS_M = 0.90

ACTIVE_GOAL_TOPIC = '/autonomy/active_goal'

# Radius of the free disc forced around the active goal. 1.4 m, the
# operator's number: wide enough to swallow a goal that landed inside a
# phantom wall together with the approach to it, narrow enough that what it
# erases is a patch the operator can see around the waypoint they placed.
GOAL_HEAL_RADIUS_M = 1.4


def view_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class TraversabilityLayer(Node):

    def __init__(self):
        super().__init__('traversability_layer')
        self.declare_parameter('map_topic', MAP_TOPIC)
        self.declare_parameter('pose_topic', POSE_TOPIC)
        self.declare_parameter('traversability_topic', TRAVERSABILITY_TOPIC)
        self.declare_parameter('costmap_seed_topic', COSTMAP_SEED_TOPIC)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('startup_clear_radius_m', STARTUP_CLEAR_RADIUS_M)
        # 0.25 m by default now, not the spec's 0.14: the chassis clears far
        # more than the spec assumed (see traversability.STEP_LETHAL_M for
        # the geometry). Retunable while the node runs - this is the number
        # an operator changes in the yard when the rover refuses a rock it
        # drives over without noticing.
        self.declare_parameter('step_lethal_m', STEP_LETHAL_M)
        # Radius of the free disc stamped along the rover's pose history.
        # MUST stay inside the footprint's inscribed circle (0.445 m half-
        # width) so it can only touch ground the chassis provably covered;
        # 0 disables the trail. See traversability.stamp_wheel_trail.
        self.declare_parameter('wheel_trail_radius_m', 0.40)
        # Cells hanging more than this above their neighbours' floor with
        # nothing beneath are dropped as airborne noise (sun glare, night
        # grain) - the operator confirmed nothing floats in this yard.
        # Bigger than any real drivable step, smaller than blob heights;
        # 0 disables. See traversability.mask_floating_cells.
        self.declare_parameter('floating_gap_m', 0.35)
        # A disc of this radius around the ACTIVE GOAL is forced free,
        # measured LETHAL included. The operator placed the waypoint and
        # says it is reachable; a goal the map calls an obstacle is refused
        # by both planners, so the run ends before it starts. 0 disables.
        # See traversability.heal_goal_patch for the trade this accepts.
        self.declare_parameter('goal_heal_radius_m', GOAL_HEAL_RADIUS_M)
        # 35 degrees by default, not the spec's 25 - see
        # traversability.SLOPE_LETHAL_DEG for the tipping arithmetic behind
        # it. Retunable like the step limit, and for the same reason: the
        # yard decides, not the desk. Degrees on the wire because that is
        # what an operator reads off a slope, radians everywhere inside.
        self.declare_parameter('slope_lethal_deg', SLOPE_LETHAL_DEG)
        self.declare_parameter('active_goal_topic', ACTIVE_GOAL_TOPIC)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._startup_clear_radius_m = float(
            self.get_parameter('startup_clear_radius_m').value)
        self._step_lethal_m = float(self.get_parameter('step_lethal_m').value)
        self._wheel_trail_radius_m = float(
            self.get_parameter('wheel_trail_radius_m').value)
        self._floating_gap_m = float(self.get_parameter('floating_gap_m').value)
        self._goal_heal_radius_m = float(
            self.get_parameter('goal_heal_radius_m').value)
        self._slope_lethal_deg = float(
            self.get_parameter('slope_lethal_deg').value)
        # (x, y) metres of the goal currently being driven to, or None. Kept
        # in metres, converted per tick like the startup patch, because the
        # rolling window's origin moves under it.
        self._active_goal = None
        # The trail: world-lattice cells (metres / RESOLUTION) the rover's
        # centre has visited, deduplicated - a set, so hours of driving in
        # the same yard stay bounded by the yard's area, not by time.
        self._trail = set()
        self.maps_processed = 0
        self.rejected_maps = 0
        self._rejected_logged = False
        # (x, y) metres of the first pose this node ever saw; None until then.
        # Fixed for the node's lifetime - see the module docstring for why a
        # single patch, not a disc that follows the rover around.
        self._startup_pose = None

        self._traversability_topic = str(self.get_parameter('traversability_topic').value)
        self._traversability_publisher = self.create_publisher(
            GridMap, self._traversability_topic, view_qos())
        self._seed_publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('costmap_seed_topic').value),
            latched_qos())
        self.create_subscription(
            GridMap, str(self.get_parameter('map_topic').value), self._on_map,
            latched_qos())
        self.create_subscription(
            Odometry, str(self.get_parameter('pose_topic').value), self._on_pose, 1)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('active_goal_topic').value),
            self._on_active_goal, latched_qos())
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _traversability_subscribers(self) -> int:
        return self.count_subscribers(self._traversability_topic)

    def _on_pose(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        # Every pose feeds the wheel trail (world-lattice cells, deduped);
        # the startup patch below stays fixed at the first pose only.
        self._trail.add((int(round(y / RESOLUTION)), int(round(x / RESOLUTION))))
        if self._startup_pose is not None:
            return          # the patch is fixed at the first pose, permanently
        self._startup_pose = (x, y)

    #: Parameters that may be retuned while the node runs, and the attribute
    #: each one writes. Every one of them is a number an operator wants to
    #: change in the yard with the rover in front of them - "it refuses that
    #: rock, let it through" - and a restart costs the ZED's map, its pose
    #: and the wheel trail. Topic names are deliberately absent: those are
    #: read once when the subscriptions are made, so accepting a new value
    #: would report a change that never happened.
    _LIVE_PARAMETERS = {
        'step_lethal_m': '_step_lethal_m',
        'floating_gap_m': '_floating_gap_m',
        'wheel_trail_radius_m': '_wheel_trail_radius_m',
        'goal_heal_radius_m': '_goal_heal_radius_m',
        'startup_clear_radius_m': '_startup_clear_radius_m',
        'slope_lethal_deg': '_slope_lethal_deg',
    }

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        """Accept a live retune of the numbers above.

        Rejected rather than clamped when a value makes no sense: a negative
        threshold would silently make every cell lethal, and an operator who
        typed it deserves to be told, not to watch the rover refuse the whole
        world. Values are applied only after the whole batch validates, so a
        rejected parameter cannot leave half a batch applied.
        """
        pending = []
        for parameter in parameters:
            attribute = self._LIVE_PARAMETERS.get(parameter.name)
            if attribute is None:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be a number")
            if not math.isfinite(value) or value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be finite and not negative")
            pending.append((attribute, value))
        for attribute, value in pending:
            setattr(self, attribute, value)
        if pending:
            self.get_logger().info(
                "retuned: " + ", ".join(f"{a.lstrip('_')}={v}" for a, v in pending))
        return SetParametersResult(successful=True)

    def _on_active_goal(self, message: PoseStamped) -> None:
        """The goal the rover is driving to now. Replaced, never
        accumulated: healing follows the current goal, so a waypoint list
        does not leave a trail of cleared discs behind it."""
        self._active_goal = (float(message.pose.position.x),
                             float(message.pose.position.y))

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

        layers, cost = seed_from_elevation(
            elevation, resolution,
            step_lethal_m=self._step_lethal_m,
            floating_gap_m=self._floating_gap_m,
            slope_lethal_rad=math.radians(self._slope_lethal_deg))
        if self._startup_pose is not None:
            # The stored pose is metres; the seed's cells are indexed from
            # this tick's origin, which moves as the rolling window
            # recentres - so the conversion is redone every tick, never
            # cached, even though the metres it starts from never change.
            x, y = self._startup_pose
            centre_cell = (int(round(y / resolution)) - origin_iy,
                          int(round(x / resolution)) - origin_ix)
            radius_cells = int(round(self._startup_clear_radius_m / resolution))
            clear_startup_patch(cost, centre_cell, radius_cells)
        if self._active_goal is not None and self._goal_heal_radius_m > 0.0:
            # Same per-tick conversion as the startup patch above: the
            # stored metres never change, the origin they are measured from
            # does.
            gx, gy = self._active_goal
            heal_goal_patch(
                cost,
                (int(round(gy / resolution)) - origin_iy,
                 int(round(gx / resolution)) - origin_ix),
                int(round(self._goal_heal_radius_m / resolution)))
        if self._trail and self._wheel_trail_radius_m > 0.0:
            # World-lattice -> this tick's grid indices, same origin shift
            # as the startup patch above. AFTER the patch and the derive:
            # wheels outrank every camera opinion, phantom or real.
            trail_cells = [(iy - origin_iy, ix - origin_ix)
                           for iy, ix in self._trail]
            stamp_wheel_trail(
                cost, trail_cells,
                int(round(self._wheel_trail_radius_m / resolution)))
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
