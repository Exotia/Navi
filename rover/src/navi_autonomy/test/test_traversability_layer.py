"""traversability_layer's plumbing and the end of the chain: a pit in, lethal
cells out. Publishers replaced by recorders; no ROS graph.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_traversability_layer.py -q'
"""
import json

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from navi_autonomy.grid_map_io import build_grid_map, layer_from_message
from navi_autonomy.traversability import (
    CLIMB_LETHAL_M, DROP_LETHAL_M, LETHAL, RELATIVE_RADIUS_M, UNKNOWN,
    clear_startup_patch)
from navi_autonomy.traversability_layer import (
    ACTIVE_GOAL_TOPIC, COSTMAP_SEED_TOPIC, GOAL_HEAL_RADIUS_M, MAP_TOPIC,
    TRAVERSABILITY_TOPIC, TUNING_STATE_TOPIC, TUNING_TOPIC, TraversabilityLayer)


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
    node._tuning_state_publisher = Recorder()
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
    # 84 with the fitted slope at the 30 degree ceiling: the pit's skirt -
    # the last fit-radius of approach to the lip - is condemned too. See
    # test_traversability.py's pit-rim test for the full argument.
    assert (cost == LETHAL).sum() == 84


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


def test_the_published_slope_layer_is_the_fitted_one_not_the_raw_gradient(node):
    # The operator reads this published layer to see why ground was
    # refused, so it has to be the slope the cost actually used - the
    # fitted plane, not the raw two-cell gradient slope_layer still
    # computes for anyone debugging the fit itself.
    from navi_autonomy.traversability import slope_layer, slope_layer_fitted
    rng = np.random.default_rng(42)
    flat_noisy = rng.normal(0.0, 0.02, (60, 60)).astype(np.float32)
    message = build_grid_map({'elevation': flat_noisy}, -30, -30, 0.05, 'map', Time())

    node._on_map(message)

    published = layer_from_message(node._traversability_publisher.messages[0], 'slope')
    assert np.nanmax(np.degrees(published)) < 15.0
    assert np.nanmax(np.degrees(slope_layer(flat_noisy))) > 45.0   # what it is not
    np.testing.assert_array_equal(published, slope_layer_fitted(flat_noisy))


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

def pose_at(x, y, z=0.0):
    message = Odometry()
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.position.z = float(z)
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

    # The rover heal (operator's order: presence outranks measurement
    # within 1 m) would cover this whole patch and is tested on its own -
    # off here, so what the STARTUP patch does to a measurement is
    # observable at all.
    node._rover_heal_radius_m = 0.0
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


# -- rover-relative lethality: can THIS rover mount the thing in front of it -

def rise_map(height=0.4, rows=60, ramp_cells=40, flat_tail=20, resolution=0.05):
    """A map corner-anchored at (x, y) = (0, 0), flat at 0 out to
    `ramp_cells`, then climbing smoothly to `height` - each single neighbour
    transition stays far under STEP_LETHAL_M no matter how large `height`
    is, so nothing here trips step_layer's own lethal test. Mirrors
    test_traversability.py's _gentle_rise, built as a GridMap message."""
    width = ramp_cells + flat_tail
    grid = np.zeros((rows, width), dtype=np.float32)
    grid[:, :ramp_cells] = np.linspace(0.0, height, ramp_cells, dtype=np.float32)
    grid[:, ramp_cells:] = height
    return build_grid_map({'elevation': grid}, 0, 0, resolution, 'map', Time())


def test_no_pose_yet_means_no_change_from_todays_seed(node):
    message = rise_map()
    row, col = 30, 55            # flat top, comfortably inside a 3 m radius

    node._on_map(message)

    assert seed_of(node)[row, col] != LETHAL      # today's behaviour: unset rover_z


def test_a_pose_arriving_activates_the_rover_relative_test_and_changes_the_seed(node):
    message = rise_map(height=0.4)
    row, col = 30, 55             # 0.4 m up the ramp - past CLIMB_LETHAL_M(0.25)

    node._on_map(message)
    assert seed_of(node)[row, col] != LETHAL      # before any pose

    node._on_pose(pose_at(0.0, 1.5))              # rover standing at (row 30, col 0)
    node._on_map(message)

    assert seed_of(node)[row, col] == LETHAL      # same map, now lethal


def test_a_zero_relative_radius_disables_the_rover_relative_test(node):
    message = rise_map(height=0.4)
    node._on_pose(pose_at(0.0, 1.5))
    node._relative_radius_m = 0.0

    node._on_map(message)

    assert seed_of(node)[30, 55] != LETHAL


def test_the_wheel_trail_still_overrides_a_cell_the_rover_relative_test_calls_lethal(node):
    # The night finding this whole ordering exists for: wheels outrank every
    # camera opinion, and this new test is a camera opinion like any other.
    # The rover drove from (row 30, col 42) to (row 30, col 0); its CURRENT
    # ground (col 0, height 0.0) makes the far side of the ramp (height 0.4)
    # lethal - but col 42 is on the driven trail, so the trail must win there
    # while an equally-lethal, never-driven cell elsewhere stays lethal.
    message = rise_map(height=0.4)

    node._on_pose(pose_at(2.1, 1.5))     # first pose: col 42 joins the trail
    node._on_pose(pose_at(0.0, 1.5))     # current pose: col 0, the rover's now-ground

    node._on_map(message)
    cost = seed_of(node)

    assert cost[30, 42] == 0             # on the trail: wheels win over the new test
    assert cost[30, 55] == LETHAL        # off the trail: the new test's opinion stands


def test_the_three_relative_lethality_limits_are_live_retunable(node):
    assert node._climb_lethal_m == pytest.approx(CLIMB_LETHAL_M)
    assert node._drop_lethal_m == pytest.approx(DROP_LETHAL_M)
    assert node._relative_radius_m == pytest.approx(RELATIVE_RADIUS_M)

    result = node._on_set_parameters([
        Parameter('climb_lethal_m', Parameter.Type.DOUBLE, 0.4),
        Parameter('drop_lethal_m', Parameter.Type.DOUBLE, 0.2),
        Parameter('relative_radius_m', Parameter.Type.DOUBLE, 5.0),
    ])

    assert result.successful is True
    assert node._climb_lethal_m == pytest.approx(0.4)
    assert node._drop_lethal_m == pytest.approx(0.2)
    assert node._relative_radius_m == pytest.approx(5.0)


# -- retuning in the yard, without losing the map --------------------------

def test_the_step_threshold_can_be_raised_while_the_node_runs(node):
    # A restart costs the ZED's map, its pose and the wheel trail, so the
    # number an operator changes standing next to a rock the rover refuses
    # must not need one.
    message, lo = pit_map()
    node._on_map(message)
    assert seed_of(node)[lo - 1, lo - 1] == LETHAL

    result = node._on_set_parameters(
        [Parameter('step_lethal_m', Parameter.Type.DOUBLE, 0.5),
         # The fitted slope at the 30 degree ceiling condemns this rim cell
         # on its own; lifted out of the way so what this test observes is
         # the STEP retune, which is its whole point.
         Parameter('slope_lethal_deg', Parameter.Type.DOUBLE, 89.0)])
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


def test_the_slope_fit_radius_is_live_retunable(node):
    # See traversability.SLOPE_FIT_RADIUS_M: the yard, not the desk, gets
    # to decide how wide a neighbourhood the cost's slope is averaged over,
    # the same as the other five numbers on this wire.
    from navi_autonomy.traversability import SLOPE_FIT_RADIUS_M
    assert node._slope_fit_radius_m == pytest.approx(SLOPE_FIT_RADIUS_M)

    result = node._on_set_parameters(
        [Parameter('slope_fit_radius_m', Parameter.Type.DOUBLE, 0.35)])

    assert result.successful is True
    assert node._slope_fit_radius_m == pytest.approx(0.35)


def test_the_slope_ceiling_can_be_lowered_while_the_node_runs(node):
    # Degrees on the wire, radians inside: an operator reads a slope off the
    # ground in degrees, and a retune that silently wanted radians would let
    # a 35 become a wall at 2 degrees.
    result = node._on_set_parameters(
        [Parameter('slope_lethal_deg', Parameter.Type.DOUBLE, 20.0)])

    assert result.successful is True
    assert node._slope_lethal_deg == pytest.approx(20.0)


# -- the ground station's own retune path: JSON over rosbridge, no service --

def tuning_message(payload: dict) -> String:
    message = String()
    message.data = json.dumps(payload)
    return message


def test_the_tuning_topics_are_the_wire_contracts_names():
    assert TUNING_TOPIC == '/autonomy/tuning'
    assert TUNING_STATE_TOPIC == '/autonomy/tuning_state'


def test_a_valid_payload_changes_the_attribute_and_is_visible_through_get_parameter(node):
    # get_parameter, not just the attribute: this is the proof the value
    # went through set_parameters rather than around it.
    node._on_tuning(tuning_message({'step_lethal_m': 0.6}))

    assert node._step_lethal_m == pytest.approx(0.6)
    assert node.get_parameter('step_lethal_m').value == pytest.approx(0.6)


def test_an_unknown_key_does_not_cost_the_rest_of_the_message(node):
    # An older ground station sending a key this build does not have must
    # not lose everything else in the same message.
    node._on_tuning(tuning_message({
        'not_a_real_parameter': 1.0, 'step_lethal_m': 0.6}))

    assert node._step_lethal_m == pytest.approx(0.6)


def test_a_negative_value_rejects_the_whole_tuning_message(node):
    before_step = node._step_lethal_m
    before_gap = node._floating_gap_m

    node._on_tuning(tuning_message({
        'floating_gap_m': 0.9, 'step_lethal_m': -1.0}))

    assert node._step_lethal_m == pytest.approx(before_step)
    assert node._floating_gap_m == pytest.approx(before_gap)


def test_infinite_and_nan_values_reject_the_whole_tuning_message(node):
    # json.loads accepts both Infinity and NaN as numbers, so both must be
    # caught here the same way a negative value is - neither is finite.
    before_step = node._step_lethal_m
    before_slope = node._slope_lethal_deg

    nan_message = String()
    nan_message.data = '{"slope_lethal_deg": NaN, "step_lethal_m": 0.6}'
    node._on_tuning(nan_message)
    assert node._slope_lethal_deg == pytest.approx(before_slope)
    assert node._step_lethal_m == pytest.approx(before_step)

    inf_message = String()
    inf_message.data = '{"step_lethal_m": Infinity}'
    node._on_tuning(inf_message)
    assert node._step_lethal_m == pytest.approx(before_step)


def test_malformed_json_does_not_raise_out_of_the_callback(node):
    message = String()
    message.data = '{not valid json'

    node._on_tuning(message)      # must not raise


def test_a_json_array_does_not_raise_out_of_the_callback(node):
    message = String()
    message.data = json.dumps([1, 2, 3])

    node._on_tuning(message)      # must not raise


def test_the_state_topic_carries_all_six_keys_at_start_up(node):
    # __init__ already published this once, to the publisher that existed
    # before the fixture swapped a Recorder in for it (same as the seed
    # and traversability publishers above) - calling it again here reaches
    # the identical payload, since nothing has retuned the node since.
    node._publish_tuning_state()

    assert len(node._tuning_state_publisher.messages) == 1
    payload = json.loads(node._tuning_state_publisher.messages[0].data)
    assert set(payload) == set(TraversabilityLayer._LIVE_PARAMETERS)


def test_the_state_topic_is_republished_after_an_accepted_change(node):
    node._on_tuning(tuning_message({'step_lethal_m': 0.6}))

    assert len(node._tuning_state_publisher.messages) == 1
    payload = json.loads(node._tuning_state_publisher.messages[-1].data)
    assert payload['step_lethal_m'] == pytest.approx(0.6)
    # Built from the node's own attributes, not from the message: every
    # other value in the same publish is what the node is actually using.
    assert payload['slope_lethal_deg'] == pytest.approx(node._slope_lethal_deg)


def test_a_retune_from_a_terminal_is_announced_like_one_from_the_ground_station(node):
    # `ros2 param set` on the rover reaches _on_set_parameters and nothing
    # else, so if the state announcement lived only on the tuning topic the
    # ground station's panel would keep showing a number the rover had
    # stopped using - and that panel exists to be believed.
    node._tuning_state_publisher.messages.clear()

    node._on_set_parameters(
        [Parameter('step_lethal_m', Parameter.Type.DOUBLE, 0.31)])

    assert len(node._tuning_state_publisher.messages) == 1
    published = json.loads(node._tuning_state_publisher.messages[-1].data)
    assert published['step_lethal_m'] == pytest.approx(0.31)


def test_a_refused_retune_announces_nothing(node):
    node._tuning_state_publisher.messages.clear()

    result = node._on_set_parameters(
        [Parameter('step_lethal_m', Parameter.Type.DOUBLE, -1.0)])

    assert result.successful is False
    assert node._tuning_state_publisher.messages == []


def test_an_empty_frame_id_retracts_the_heal_disc(node):
    # Without the retraction the LAST goal's disc is healed on every tick
    # forever - forced-free ground outliving the mission that vouched for
    # it, on a latched topic that replays into every restart of this node.
    message, lo = pit_map()
    goal_x = (lo - 1 + -12) * 0.05
    goal_y = (lo - 1 + -12) * 0.05
    node._on_active_goal(goal_at(goal_x, goal_y))
    node._on_map(message)
    assert seed_of(node)[lo - 1, lo - 1] == 0

    cleared = PoseStamped()
    cleared.header.frame_id = ""
    node._on_active_goal(cleared)
    node._on_map(message)

    assert seed_of(node)[lo - 1, lo - 1] == LETHAL


def test_a_drifted_pose_z_no_longer_walls_the_rover_in(node):
    # The operator's "glitches under the ground": the ZED's z drifts
    # against the very grid it built, and with the pose as the reference a
    # rover standing on level mapped ground read every nearby cell as a
    # lethal climb - route on screen, goal accepted, wheels never moving.
    # The reference is the map's own ground under the footprint now, so
    # the pose's z can say anything it likes.
    flat = np.zeros((24, 24), dtype=np.float32)
    message = build_grid_map({'elevation': flat}, -12, -12, 0.05, 'map', Time())
    pose = Odometry()
    pose.pose.pose.position.x = 0.0
    pose.pose.pose.position.y = 0.0
    pose.pose.pose.position.z = -5.0      # five metres under the ground

    node._on_pose(pose)
    node._on_map(message)

    cost = seed_of(node)
    assert (cost == LETHAL).sum() == 0


# -- the rover-centred heal --------------------------------------------------

def test_the_ground_the_rover_stands_on_is_never_lethal(node):
    # The operator's instruction after "start pose is an obstacle" ended a
    # run with the rover parked on good ground: a 1 m disc at the CURRENT
    # position is forced free, measured lethal included, because the rover
    # standing there is the proof.
    message, lo = pit_map()
    pose = Odometry()
    # Park the rover 0.6 m from the rim cell under test: inside the 1 m
    # heal disc but OUTSIDE the 0.40 m wheel trail, so what frees the cell
    # is provably the heal and not the trail.
    pose.pose.pose.position.x = (lo - 1 + -12) * 0.05
    pose.pose.pose.position.y = (lo - 1 + -12 + 12) * 0.05

    node._on_pose(pose)
    node._on_map(message)

    assert seed_of(node)[lo - 1 + 12, lo - 1] != LETHAL
    assert seed_of(node)[lo - 1, lo - 1] == 0


def test_the_rover_heal_touches_cost_and_never_the_height_layers(node):
    # "The rover and current map pos should keep their height": the heal is
    # a statement about drivability, not about the world's shape - the
    # published elevation-derived layers stay exactly what was measured.
    message, lo = pit_map()
    pose = Odometry()
    pose.pose.pose.position.x = (lo - 1 + -12) * 0.05
    pose.pose.pose.position.y = (lo - 1 + -12) * 0.05

    node._on_pose(pose)
    node._on_map(message)

    published = node._traversability_publisher.messages[0]
    step = layer_from_message(published, 'step')
    assert np.nanmax(step) == pytest.approx(0.3)


def test_a_zero_rover_heal_radius_disables_it(node):
    message, lo = pit_map()
    node._rover_heal_radius_m = 0.0
    pose = Odometry()
    # 0.6 m from the rim cell: past the wheel trail's 0.40 m, so with the
    # heal off nothing else frees it.
    pose.pose.pose.position.x = (lo - 1 + -12) * 0.05
    pose.pose.pose.position.y = (lo - 1 + -12 + 12) * 0.05

    node._on_pose(pose)
    node._on_map(message)

    assert seed_of(node)[lo - 1, lo - 1] == LETHAL


def test_the_rover_heal_radius_is_live_retunable(node):
    result = node._on_set_parameters(
        [Parameter('rover_heal_radius_m', Parameter.Type.DOUBLE, 1.5)])

    assert result.successful is True
    assert node._rover_heal_radius_m == pytest.approx(1.5)
