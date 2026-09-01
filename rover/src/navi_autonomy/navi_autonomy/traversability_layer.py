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

The ground station's rosbridge link speaks only JSON over std_msgs/String,
never a ROS service, so `ros2 param set` is unreachable from the operator's
screen standing in the yard. /autonomy/tuning (inbound) and
/autonomy/tuning_state (outbound, latched) close that gap for the six
numbers above - see _on_tuning and _publish_tuning_state.
"""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import (
    ELEVATION_LAYER, build_grid_map, build_occupancy_grid, layer_from_message)
from navi_autonomy.tile_aggregator import MAP_TOPIC, POSE_TOPIC, latched_qos
from navi_autonomy.traversability import (CLIMB_LETHAL_M, DROP_LETHAL_M,
                                          RELATIVE_RADIUS_M, SLOPE_LETHAL_DEG,
                                          STEP_LETHAL_M, clear_startup_patch,
                                          ground_under, heal_goal_patch,
                                          seed_from_elevation,
                                          stamp_wheel_trail)
from navi_localization.elevation_grid import RESOLUTION

TRAVERSABILITY_TOPIC = '/autonomy/traversability'
COSTMAP_SEED_TOPIC = '/autonomy/costmap_seed'
LAYER_ORDER = ('slope', 'step', 'roughness', 'valid')

# The costmap's robot_radius (nav2_rover.yaml) is 0.80 m; the operator's
# margin on top of it is 10 cm. 0.90 m is the startup patch's disc radius.
STARTUP_CLEAR_RADIUS_M = 0.90

ACTIVE_GOAL_TOPIC = '/autonomy/active_goal'

# The disc the rover's reference ground is read from: the footprint's
# inscribed circle, the same 0.445 m the wheel trail is bounded by - ground
# the chassis is provably over right now.
FOOTPRINT_RADIUS_M = 0.44

# The disc around the rover's CURRENT position forced free in the seed,
# measured LETHAL included - the operator's standing instruction
# (2026-09-01): "the ground around the rover centre should be healed in a
# 1 m radius". The rover is standing there; ground it is standing on is
# drivable by demonstration, and Nav2 validates the start pose against the
# 0.80 m robot radius, so a phantom-lethal cell in the ring the 0.40 m
# wheel trail does not reach was ending runs with "start pose is an
# obstacle" while the rover sat on perfectly good ground. 1.0 covers the
# footprint and that ring with margin. Cost only, never elevation - the
# height layers stay exactly what was measured, here as everywhere.
ROVER_HEAL_RADIUS_M = 1.0

# Radius of the free disc forced around the active goal. 1.4 m, the
# operator's number: wide enough to swallow a goal that landed inside a
# phantom wall together with the approach to it, narrow enough that what it
# erases is a patch the operator can see around the waypoint they placed.
GOAL_HEAL_RADIUS_M = 1.4

# The wire contract with the ground station's rosbridge link: it can publish
# and subscribe to plain topics but has no way to call a ROS service, so
# these two close the gap `ros2 param set`/`get` leave for that operator.
TUNING_TOPIC = '/autonomy/tuning'
TUNING_STATE_TOPIC = '/autonomy/tuning_state'


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
        # See ROVER_HEAL_RADIUS_M above; 0 disables. Unlike the startup
        # patch this follows the rover, and unlike the wheel trail it
        # clears measured cost - the moving clearing disc rejected in this
        # file's docstring, now ordered deliberately by the same operator
        # with live runs behind the change of mind.
        self.declare_parameter('rover_heal_radius_m', ROVER_HEAL_RADIUS_M)
        # 35 degrees by default, not the spec's 25 - see
        # traversability.SLOPE_LETHAL_DEG for the tipping arithmetic behind
        # it. Retunable like the step limit, and for the same reason: the
        # yard decides, not the desk. Degrees on the wire because that is
        # what an operator reads off a slope, radians everywhere inside.
        self.declare_parameter('slope_lethal_deg', SLOPE_LETHAL_DEG)
        # Rover-relative lethality: a cell more than this above the rover's
        # own current ground is something the rover cannot climb, even if
        # every individual step towards it read comfortably drivable (a
        # gradual staircase). See traversability.CLIMB_LETHAL_M.
        self.declare_parameter('climb_lethal_m', CLIMB_LETHAL_M)
        # Tighter than the climb on purpose - see traversability.DROP_LETHAL_M
        # for why a wrong climb strands a wheel but a wrong drop ends the
        # run on the rover's belly.
        self.declare_parameter('drop_lethal_m', DROP_LETHAL_M)
        # Radius (metres) around the rover's CURRENT pose within which the
        # two limits above apply. Not a sensitivity knob: past this radius
        # the rover-relative test is switched off completely, because a
        # gentle yard slope makes "height above the rover" meaningless a
        # few metres out (see traversability.costmap_seed). 0 disables it.
        self.declare_parameter('relative_radius_m', RELATIVE_RADIUS_M)
        self.declare_parameter('active_goal_topic', ACTIVE_GOAL_TOPIC)
        self.declare_parameter('tuning_topic', TUNING_TOPIC)
        self.declare_parameter('tuning_state_topic', TUNING_STATE_TOPIC)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._startup_clear_radius_m = float(
            self.get_parameter('startup_clear_radius_m').value)
        self._step_lethal_m = float(self.get_parameter('step_lethal_m').value)
        self._wheel_trail_radius_m = float(
            self.get_parameter('wheel_trail_radius_m').value)
        self._floating_gap_m = float(self.get_parameter('floating_gap_m').value)
        self._goal_heal_radius_m = float(
            self.get_parameter('goal_heal_radius_m').value)
        self._rover_heal_radius_m = float(
            self.get_parameter('rover_heal_radius_m').value)
        self._slope_lethal_deg = float(
            self.get_parameter('slope_lethal_deg').value)
        self._climb_lethal_m = float(self.get_parameter('climb_lethal_m').value)
        self._drop_lethal_m = float(self.get_parameter('drop_lethal_m').value)
        self._relative_radius_m = float(
            self.get_parameter('relative_radius_m').value)
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
        # (x, y, z) metres of the MOST RECENT pose, unlike _startup_pose
        # above which is deliberately frozen at the first one. This is what
        # the rover-relative lethal test needs: "can I mount the thing in
        # front of me right now" is a question about where the rover is
        # now, not where it started. None until the first pose arrives.
        self._current_pose = None
        # The reason last logged for a rejected /autonomy/tuning payload
        # (None until one has been). Compared, not counted: a ground
        # station retrying the same bad payload at 2 Hz must not fill the
        # log, but a change in reason is worth a fresh line.
        self._last_tuning_warning = None

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
        # Outbound state is latched (see the module docstring): a ground
        # station that connects after start-up still learns the current
        # six values without waiting on the next retune. Inbound tuning is
        # not - a stale retune sitting in the queue is not something a late
        # subscriber should ever replay onto the node.
        self._tuning_state_publisher = self.create_publisher(
            String, str(self.get_parameter('tuning_state_topic').value),
            latched_qos())
        self.create_subscription(
            String, str(self.get_parameter('tuning_topic').value),
            self._on_tuning, 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._publish_tuning_state()

    def _traversability_subscribers(self) -> int:
        return self.count_subscribers(self._traversability_topic)

    def _on_pose(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        z = float(message.pose.pose.position.z)
        # Every pose feeds the wheel trail (world-lattice cells, deduped);
        # the startup patch below stays fixed at the first pose only. The
        # current pose, unlike the startup one, is overwritten every time -
        # it is the rover's ground for the rover-relative lethal test, which
        # is only meaningful about where the rover is now.
        self._trail.add((int(round(y / RESOLUTION)), int(round(x / RESOLUTION))))
        self._current_pose = (x, y, z)
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
        'rover_heal_radius_m': '_rover_heal_radius_m',
        'startup_clear_radius_m': '_startup_clear_radius_m',
        'slope_lethal_deg': '_slope_lethal_deg',
        'climb_lethal_m': '_climb_lethal_m',
        'drop_lethal_m': '_drop_lethal_m',
        'relative_radius_m': '_relative_radius_m',
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
            # Announced here rather than only on the /autonomy/tuning path,
            # because this callback is where EVERY change arrives - a
            # `ros2 param set` from a terminal on the rover included. The
            # state topic is what the ground station's panel shows as the
            # rover's own value, and a panel showing a number the rover
            # stopped using is worse than a panel showing nothing.
            self._publish_tuning_state()
        return SetParametersResult(successful=True)

    def _warn_about_tuning(self, reason: str, message: str) -> None:
        """Logs `message` unless `reason` is the one last logged here.

        A ground station that cannot reach a rejected value some other way
        will retry it at 2 Hz; that must not fill the log the way one
        warning per distinct reason does not.
        """
        if reason == self._last_tuning_warning:
            return
        self._last_tuning_warning = reason
        self.get_logger().warn(message)

    def _on_tuning(self, message: String) -> None:
        """A live retune sent as JSON, for the ground station's rosbridge
        link, which can reach a topic but never a ROS service.

        Validated the same way a batch from `ros2 param set` is: a bad
        value anywhere in the payload rejects the whole payload, because
        half-applied tuning is worse than none. An unknown key costs the
        rest of the message nothing, so an older ground station sending a
        key this build does not have still gets everything else applied.
        """
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self._warn_about_tuning(
                'bad JSON', f"/autonomy/tuning: not valid JSON ({message.data!r})")
            return
        if not isinstance(payload, dict):
            self._warn_about_tuning(
                'not an object',
                f"/autonomy/tuning: payload must be a JSON object, got {payload!r}")
            return
        if not payload:
            self._warn_about_tuning('empty object', "/autonomy/tuning: empty payload")
            return

        parameters = []
        for name, value in payload.items():
            attribute = self._LIVE_PARAMETERS.get(name)
            if attribute is None:
                continue     # an older or newer ground station's key we don't have
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0.0):
                self.get_logger().warn(
                    f"/autonomy/tuning: {name}={value!r} must be finite and not "
                    "negative; the whole message is ignored")
                return
            parameters.append(Parameter(name, Parameter.Type.DOUBLE, float(value)))

        if not parameters:
            return           # every key present was one this build does not have

        # set_parameters, never a direct attribute write: this is the one
        # validated path that already exists (_on_set_parameters above),
        # and it is what keeps `ros2 param get` telling the truth about
        # what the node is actually using.
        # Atomically, not one at a time: set_parameters() runs the
        # validation callback once PER PARAMETER, so a six-key Apply from
        # the ground station would publish six tuning states, the first
        # five of them half-applied. The atomic form hands the callback the
        # whole batch once, which is also what its all-or-nothing
        # validation was written for. The state announcement rides on that
        # callback, so there is nothing to publish here.
        self.set_parameters_atomically(parameters)

    def _publish_tuning_state(self) -> None:
        """All six live values, latched so a ground station connecting
        after start-up still learns them without waiting on a retune.

        Built from the node's own attributes, not from whatever a message
        last asked for, so what goes out is what the node is using.
        """
        payload = {name: getattr(self, attribute)
                   for name, attribute in self._LIVE_PARAMETERS.items()}
        state = String()
        state.data = json.dumps(payload)
        self._tuning_state_publisher.publish(state)

    def _on_active_goal(self, message: PoseStamped) -> None:
        """The goal the rover is driving to now. Replaced, never
        accumulated: healing follows the current goal, so a waypoint list
        does not leave a trail of cleared discs behind it.

        An empty frame_id is the wire's retraction: the run ended or was
        cancelled, and without honouring it the LAST goal's disc would be
        healed on every tick forever - forced-free ground outliving the
        mission that vouched for it, on a latched topic that replays to
        every restart of this node."""
        if not message.header.frame_id:
            self._active_goal = None
            return
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

        rover_z = None
        rover_cell = None
        if self._current_pose is not None:
            # Per-tick conversion, never cached, same as the startup patch
            # and the goal heal just below: the rolling window's origin
            # moves under it even though the pose itself has not changed.
            x, y, _pose_z = self._current_pose
            rover_cell = (int(round(y / resolution)) - origin_iy,
                         int(round(x / resolution)) - origin_ix)
            # The reference ground is read from THIS map, never from the
            # pose's z: the ZED's z drifts against the grid it built, and a
            # drifted reference turns level ground into a lethal climb ring
            # around the rover - route on screen, goal accepted, wheels
            # never move. See ground_under for the full argument. When the
            # ground under the rover is unmapped there is no reference and
            # the rover-relative test simply sits this tick out - the step,
            # drop and slope layers still guard on their own.
            rover_z = ground_under(
                elevation, rover_cell,
                int(round(FOOTPRINT_RADIUS_M / resolution)))

        layers, cost = seed_from_elevation(
            elevation, resolution,
            step_lethal_m=self._step_lethal_m,
            floating_gap_m=self._floating_gap_m,
            slope_lethal_rad=math.radians(self._slope_lethal_deg),
            rover_z=rover_z,
            rover_cell=rover_cell,
            relative_radius_m=self._relative_radius_m,
            climb_lethal_m=self._climb_lethal_m,
            drop_lethal_m=self._drop_lethal_m)
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
        if rover_cell is not None and self._rover_heal_radius_m > 0.0:
            # Same force-free write as the goal heal, same licence shape:
            # a human vouches for the goal, the rover's own presence
            # vouches for this.
            heal_goal_patch(
                cost, rover_cell,
                int(round(self._rover_heal_radius_m / resolution)))
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
