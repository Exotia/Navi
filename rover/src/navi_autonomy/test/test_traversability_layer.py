"""traversability_layer's plumbing and the end of the chain: a pit in, lethal
cells out. Publishers replaced by recorders; no ROS graph.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_traversability_layer.py -q'
"""
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry

from navi_autonomy.grid_map_io import build_grid_map, layer_from_message
from navi_autonomy.traversability import LETHAL, UNKNOWN, clear_startup_patch
from navi_autonomy.traversability_layer import (
    ACTIVE_GOAL_TOPIC, COSTMAP_SEED_TOPIC, GOAL_HEAL_RADIUS_M, MAP_TOPIC,
    TRAVERSABILITY_TOPIC, TraversabilityLayer)


class Recorder:
    def __init__(self, subscribers=1):
        self.messages = []
        self.subscribers = subscribers

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = TraversabilityLayer()
    node._traversability_publisher = Recorder()
    node._seed_publisher = Recorder()
    node._traversability_subscribers = lambda: 1
    yield node
    node.destroy_node()


def pit_map(depth=0.3, size=6, extent=24, origin_ix=-12, origin_iy=-12):
    # 0.3 m, deeper than the 0.25 m step threshold: a pit these tests
    # need the layer to actually refuse. A 0.2 m pit is drivable now
    # (see test_traversability.py for what that raise cost).
    grid = np.zeros((extent, extent), dtype=np.float32)
    lo = (extent - size) // 2
    grid[lo:lo + size, lo:lo + size] = -depth
    return build_grid_map({'elevation': grid}, origin_ix, origin_iy, 0.05,
                          'map', Time()), lo


def test_the_topics_are_the_spec_names():
    assert MAP_TOPIC == '/autonomy/map'
    assert TRAVERSABILITY_TOPIC == '/autonomy/traversability'
    assert COSTMAP_SEED_TOPIC == '/autonomy/costmap_seed'


def test_a_pit_publishes_lethal_cells_on_its_rim(node):
    message, lo = pit_map()
    node._on_map(message)
    assert len(node._seed_publisher.messages) == 1
    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[lo - 1, lo - 1] == LETHAL
    assert cost[lo - 1, lo + 2] == LETHAL
    assert cost[2, 2] == 0
    assert (cost == LETHAL).sum() == 48


def test_the_seed_carries_the_maps_geometry(node):
    message, _ = pit_map(origin_ix=-12, origin_iy=40)
    node._on_map(message)
    seed = node._seed_publisher.messages[0]
    assert seed.header.frame_id == 'map'
    assert seed.info.resolution == pytest.approx(0.05)
    assert (seed.info.width, seed.info.height) == (24, 24)
    assert seed.info.origin.position.x == pytest.approx(-0.6)
    assert seed.info.origin.position.y == pytest.approx(2.0)


def test_unseen_ground_is_unknown_in_the_seed(node):
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    node._on_map(build_grid_map({'elevation': grid}, 0, 0, 0.05, 'map', Time()))
    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[5, 5] == UNKNOWN
    assert cost[0, 0] == UNKNOWN


def test_the_traversability_grid_map_carries_all_four_layers(node):
    message, _ = pit_map()
    node._on_map(message)
    published = node._traversability_publisher.messages[0]
    assert list(published.layers) == ['slope', 'step', 'roughness', 'valid']
    assert list(published.basic_layers) == ['valid']
    assert published.info.resolution == pytest.approx(0.05)
    step = layer_from_message(published, 'step')
    # The fixture's pit depth: the published layer is the measurement, and
    # it is untouched by where the lethal threshold happens to sit.
    assert np.nanmax(step) == pytest.approx(0.3)


def test_the_expensive_grid_map_is_not_built_when_nobody_is_listening(node):
    node._traversability_subscribers = lambda: 0
    message, _ = pit_map()
    node._on_map(message)
    assert node._traversability_publisher.messages == []
    assert len(node._seed_publisher.messages) == 1     # the seed always goes out


def test_a_map_at_the_wrong_resolution_is_refused(node):
    message, _ = pit_map()
    message.info.resolution = 0.10
    node._on_map(message)
    assert node._seed_publisher.messages == []
    assert node.rejected_maps == 1


def test_a_map_without_an_elevation_layer_is_refused(node):
    message, _ = pit_map()
    message.layers = ['colour']
    node._on_map(message)
    assert node._seed_publisher.messages == []
    assert node.rejected_maps == 1


# -- "the wheels have been here" (one startup patch) ------------------------

def pose_at(x, y):
    message = Odometry()
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    return message


def test_a_startup_pose_clears_a_disc_of_unknown_ground_around_it(node):
    # 50x50 so the far corner sits outside the ~18-cell disc (0.90 m / 0.05 m).
    grid = np.full((50, 50), np.nan, dtype=np.float32)     # nothing seen anywhere
    message = build_grid_map({'elevation': grid}, -25, -25, 0.05, 'map', Time())

    node._on_pose(pose_at(0.0, 0.0))       # rover starts at the map's origin
    node._on_map(message)

    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    # origin_ix = origin_iy = -25, so (x, y) = (0, 0) is cell (25, 25)
    assert cost[25, 25] == 0
    assert cost[49, 49] == UNKNOWN         # far corner, well outside the disc


def test_a_measured_cell_inside_the_startup_patch_is_never_overwritten(node):
    # A locally-mapped, mostly flat patch around the pose, with a small pit
    # in it (measured LETHAL rim, spec section 5's usual fixture) surrounded
    # by genuinely unseen (NaN) ground everywhere else in the 50x50 window.
    # The pit sits 10+ cells from the pose: inside the 18-cell startup
    # patch, where measurements always win - but OUTSIDE the 8-cell wheel
    # trail, which is the one place wheels now outrank the camera.
    grid = np.full((50, 50), np.nan, dtype=np.float32)
    grid[10:35, 10:35] = 0.0
    grid[11:15, 23:27] = -0.2              # a 0.2 m pit, 10 cells above the pose
    message = build_grid_map({'elevation': grid}, -25, -25, 0.05, 'map', Time())

    node._on_pose(pose_at(0.0, 0.0))       # (x, y) = (0, 0) is cell (25, 25)
    node._on_map(message)

    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[11, 24] == LETHAL          # the pit's measured rim survives the patch
    assert cost[25, 8] == 0                # unseen ground inside the disc, outside the
                                            # flat patch, is cleared (col 8 is 17 cells
                                            # from the centre, radius is 18)


def test_the_wheel_trail_frees_the_ground_the_rover_drove_over(node):
    # The stranded-rover night: phantom lethal painted onto the rover's own
    # driven path refused every plan's start pose. Wheels outrank the
    # camera on the trail itself - a measured LETHAL under a visited pose
    # goes free, while the same measurement off-trail stays lethal.
    grid = np.full((50, 50), np.nan, dtype=np.float32)
    grid[10:40, 10:40] = 0.0
    grid[24:27, 12:38] = 0.3               # a phantom "wall" along the driven line
    message = build_grid_map({'elevation': grid}, -25, -25, 0.05, 'map', Time())

    node._on_pose(pose_at(0.0, 0.0))
    node._on_pose(pose_at(0.25, 0.0))      # drove +x along the phantom
    node._on_pose(pose_at(0.50, 0.0))
    node._on_map(message)

    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[25, 25] == 0 and cost[25, 30] == 0     # trail: wheels won
    assert cost[25, 8] == 0                            # startup patch still fills unknown
    # cols 12-16 of the wall sit outside every trail disc (nearest trail
    # centre is col 25, radius 8): the measured phantom stays lethal there.
    assert LETHAL in cost[23:28, 12:16]


def test_a_second_pose_does_not_move_or_add_a_patch(node):
    grid = np.full((40, 40), np.nan, dtype=np.float32)
    message = build_grid_map({'elevation': grid}, -20, -20, 0.05, 'map', Time())

    node._on_pose(pose_at(0.0, 0.0))       # the startup pose -> cell (20, 20)
    node._on_pose(pose_at(5.0, 5.0))       # the rover has since moved - ignored
    node._on_map(message)

    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[20, 20] == 0               # patch still centred on the first pose

    # A single disc's worth of clearing, and nothing more, proves the second
    # pose neither moved the patch nor added one of its own.
    radius_cells = int(round(0.90 / 0.05))
    only_patch = np.full((40, 40), UNKNOWN, dtype=np.int8)
    clear_startup_patch(only_patch, (20, 20), radius_cells)
    assert (cost == 0).sum() == (only_patch == 0).sum()


# -- the goal-heal disc ----------------------------------------------------

def goal_at(x, y):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    return goal


def seed_of(node):
    seed = node._seed_publisher.messages[-1]
    return np.asarray(seed.data, dtype=np.int8).reshape(
        seed.info.height, seed.info.width)


def test_the_goal_topic_and_radius_are_the_operators_numbers():
    assert ACTIVE_GOAL_TOPIC == '/autonomy/active_goal'
    assert GOAL_HEAL_RADIUS_M == pytest.approx(1.4)


def test_a_goal_inside_a_pit_is_healed_to_free_ground(node):
    # The pit's rim is LETHAL (see the first test in this file). A goal
    # placed on that rim is the "target is in the ground" case: without the
    # heal both planners refuse it.
    message, lo = pit_map()
    # cell (lo-1, lo-1) in a map whose corner lattice index is (-12, -12),
    # 0.05 m cells -> metres.
    goal_x = (lo - 1 + -12) * 0.05
    goal_y = (lo - 1 + -12) * 0.05

    node._on_map(message)
    assert seed_of(node)[lo - 1, lo - 1] == LETHAL

    node._on_active_goal(goal_at(goal_x, goal_y))
    node._on_map(message)

    assert seed_of(node)[lo - 1, lo - 1] == 0


def test_the_heal_reaches_a_metre_and_a_bit_and_no_further(node):
    message, lo = pit_map()
    goal_x = (lo - 1 + -12) * 0.05
    goal_y = (lo - 1 + -12) * 0.05
    node._on_active_goal(goal_at(goal_x, goal_y))

    node._on_map(message)
    cost = seed_of(node)

    radius_cells = int(round(GOAL_HEAL_RADIUS_M / 0.05))
    assert radius_cells == 28
    # Everything the 24x24 map holds is inside a 1.4 m disc centred near its
    # corner, so nothing lethal survives - which is the point being made:
    # the disc is large next to this map, and the operator sees it.
    assert (cost == LETHAL).sum() == 0


def test_without_a_goal_nothing_is_healed(node):
    message, lo = pit_map()

    node._on_map(message)

    assert seed_of(node)[lo - 1, lo - 1] == LETHAL


def test_a_new_goal_replaces_the_old_one_rather_than_accumulating(node):
    message, lo = pit_map()
    node._on_active_goal(goal_at(0.0, 0.0))
    node._on_active_goal(goal_at(99.0, 99.0))

    node._on_map(message)

    # The second goal is far off this map, so the first goal's disc must be
    # gone: healing follows the current goal, it does not leave a trail.
    assert seed_of(node)[lo - 1, lo - 1] == LETHAL


def test_a_zero_radius_turns_the_heal_off(node):
    message, lo = pit_map()
    node._goal_heal_radius_m = 0.0
    node._on_active_goal(goal_at((lo - 1 + -12) * 0.05, (lo - 1 + -12) * 0.05))

    node._on_map(message)

    assert seed_of(node)[lo - 1, lo - 1] == LETHAL


# -- retuning in the yard, without losing the map --------------------------

def test_the_step_threshold_can_be_raised_while_the_node_runs(node):
    # A restart costs the ZED's map, its pose and the wheel trail, so the
    # number an operator changes standing next to a rock the rover refuses
    # must not need one.
    message, lo = pit_map()
    node._on_map(message)
    assert seed_of(node)[lo - 1, lo - 1] == LETHAL

    result = node._on_set_parameters(
        [Parameter('step_lethal_m', Parameter.Type.DOUBLE, 0.5)])
    node._on_map(message)

    assert result.successful is True
    assert node._step_lethal_m == pytest.approx(0.5)
    assert seed_of(node)[lo - 1, lo - 1] != LETHAL


def test_a_negative_threshold_is_refused_rather_than_quietly_applied(node):
    # Negative would make every cell lethal and wall off the whole yard; an
    # operator who typed it deserves to be told, not to watch the rover
    # refuse the world.
    before = node._step_lethal_m

    result = node._on_set_parameters(
        [Parameter('step_lethal_m', Parameter.Type.DOUBLE, -1.0)])

    assert result.successful is False
    assert node._step_lethal_m == pytest.approx(before)


def test_a_rejected_parameter_leaves_none_of_its_batch_applied(node):
    before_gap = node._floating_gap_m

    result = node._on_set_parameters([
        Parameter('floating_gap_m', Parameter.Type.DOUBLE, 0.9),
        Parameter('step_lethal_m', Parameter.Type.DOUBLE, -1.0),
    ])

    assert result.successful is False
    assert node._floating_gap_m == pytest.approx(before_gap)


def test_a_topic_name_is_not_treated_as_a_live_parameter(node):
    # Topics are read once when the subscriptions are made, so accepting a
    # new one would report a change that never happened.
    result = node._on_set_parameters(
        [Parameter('map_topic', Parameter.Type.STRING, '/somewhere/else')])

    assert result.successful is True
