"""Rung 3 of the testing ladder: Nav2 plans on the fixture elevation map.

    ROS_DOMAIN_ID=92 python3 -m pytest \
        rover/src/navi_nav2/test/test_offline_planning.py -q -s

Needs the workspace built and sourced (the launch file, the params and the
fixture node come out of share/).  Brings the whole stack up once for the
module, asserts against it, and kills the process group on the way out.

Domain 92 is a throwaway.  Nothing here publishes /manual_twist or
/rover_twist, and the assertions at the bottom prove neither exists.
"""

import math
import os
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from navi_autonomy.traversability import LETHAL, UNKNOWN
from navi_nav2 import bringup, fixture

DOMAIN = '92'
INSCRIBED_M = 0.75          # robot_radius 0.80 less the 0.05 m cell it stands in
BRINGUP_TIMEOUT_S = 45.0
PLAN_TIMEOUT_S = 30.0

# deploy_rover.sh --test runs `python3 -m pytest test` in every src/*/ on the
# Orin under `set -eo pipefail`, with no ROS_DOMAIN_ID set.  Without this
# guard every test in this file errors in the stack fixture and the whole
# deploy fails - and if the domain happened to be set, the deploy would
# silently bring a full Nav2 stack up on the rover.  SKIP is the right
# outcome there; the Task 6 and Task 7 commands set the domain explicitly
# and still run it.
pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != DOMAIN,
    reason=f"rung 3 needs an explicit ROS_DOMAIN_ID={DOMAIN}; "
           f"run it with the Task 6 command, not from deploy_rover.sh --test")


# --------------------------------------------------------------------------
# the stack, once for the module
# --------------------------------------------------------------------------

@pytest.fixture(scope='module')
def stack():
    assert os.environ.get('ROS_DOMAIN_ID') == DOMAIN, \
        f"run this with ROS_DOMAIN_ID={DOMAIN}; never on domain 0"
    process = subprocess.Popen(
        ['ros2', 'launch', 'navi_nav2', 'nav2_bringup.launch.py',
         'perception:=false', 'bench_fixture:=true'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    try:
        _wait_for_active(BRINGUP_TIMEOUT_S)
        yield process
    finally:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def _lifecycle_state(node_name):
    result = subprocess.run(['ros2', 'lifecycle', 'get', f'/{node_name}'],
                            capture_output=True, text=True)
    return result.stdout.strip()


def _wait_for_active(timeout_s):
    deadline = time.monotonic() + timeout_s
    pending = list(bringup.LIFECYCLE_NODES)
    while pending and time.monotonic() < deadline:
        pending = [n for n in pending if not _lifecycle_state(n).startswith('active')]
        if pending:
            time.sleep(1.0)
    assert not pending, f"not active after {timeout_s} s: {pending}"


@pytest.fixture(scope='module')
def client(stack):
    rclpy.init()
    node = Node('offline_planning_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


# --------------------------------------------------------------------------
# A - bringup
# --------------------------------------------------------------------------

def test_every_lifecycle_node_is_active(stack):
    for name in bringup.LIFECYCLE_NODES:
        assert _lifecycle_state(name).startswith('active'), name


# --------------------------------------------------------------------------
# B - parameter fidelity: what the running nodes actually believe
# --------------------------------------------------------------------------

def running_params(node_name):
    """Nav2 ignores a parameter key it does not know.  A typo in the file
    is therefore invisible everywhere except here, where the node reports
    the plugin default instead of the value we wrote."""
    dumped = subprocess.run(['ros2', 'param', 'dump', '--print', f'/{node_name}'],
                            capture_output=True, text=True, check=True).stdout
    return yaml.safe_load(dumped)[f'/{node_name}']['ros__parameters']


def test_the_planner_runs_theta_star_with_smac_beside_it(stack):
    planner = running_params('planner_server')
    assert planner['planner_plugins'] == ['GridBased', 'SmacBased']
    assert planner['GridBased']['plugin'] == 'nav2_theta_star_planner/ThetaStarPlanner'
    assert planner['SmacBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert planner['GridBased']['allow_unknown'] is False
    assert planner['GridBased']['w_traversal_cost'] == 2.0


def test_the_controller_is_the_shim_and_will_not_reverse(stack):
    controller = running_params('controller_server')
    follow = controller['FollowPath']
    assert follow['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert (follow['primary_controller']
            == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    assert follow['allow_reversing'] is False
    assert follow['desired_linear_vel'] == 0.05
    assert controller['odom_topic'] == '/localization/odom_local'


def test_the_speed_caps_reached_the_smoother(stack):
    smoother = running_params('velocity_smoother')
    assert list(smoother['max_velocity']) == [0.05, 0.0, 0.1]
    assert list(smoother['min_velocity']) == [-0.15, 0.0, -0.1]
    assert smoother['odom_topic'] == '/localization/odom_local'


def test_the_costmaps_read_the_seed_at_five_centimetres(stack):
    for node_name in ('global_costmap/global_costmap', 'local_costmap/local_costmap'):
        costmap = running_params(node_name)
        assert costmap['resolution'] == 0.05
        assert '0.445' in str(costmap['footprint'])
        assert costmap['track_unknown_space'] is True
        assert costmap['static_layer']['map_topic'] == '/autonomy/costmap_seed'
        # trinary_costmap, lethal_cost_threshold and unknown_cost_value are
        # Costmap2DROS's own parameters (the OccupancyGrid->Costmap
        # translation table shared by every layer), not static_layer's -
        # asserting the nested key here would always read a KeyError, since
        # Nav2 never declares a parameter by that nested name at all.
        assert costmap['trinary_costmap'] is False
        assert costmap['lethal_cost_threshold'] == 100


def test_the_cloud_layers_are_off(stack):
    assert running_params(
        'global_costmap/global_costmap')['obstacle_layer']['enabled'] is False
    assert running_params(
        'local_costmap/local_costmap')['voxel_layer']['enabled'] is False
    assert running_params('collision_monitor')['points_filtered']['enabled'] is False


def test_nav2s_velocity_leaves_on_autonomy_twist(stack):
    monitor = running_params('collision_monitor')
    assert monitor['cmd_vel_out_topic'] == '/autonomy_twist'
    assert monitor['base_frame_id'] == 'base_footprint'


def test_the_behaviour_tree_that_is_loaded_is_ours(stack):
    path = running_params('bt_navigator')['default_nav_to_pose_bt_xml']
    assert path.endswith('behavior_trees/navigate_to_pose_no_reverse.xml')
    assert os.path.exists(path)


# --------------------------------------------------------------------------
# C, D, E, F - the plan
# --------------------------------------------------------------------------

def pose_stamped(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def compute_path(node, planner_id):
    action = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    assert action.wait_for_server(timeout_sec=20.0), "no compute_path_to_pose server"
    goal = ComputePathToPose.Goal()
    goal.start = pose_stamped(*fixture.START)
    goal.goal = pose_stamped(*fixture.GOAL)
    goal.planner_id = planner_id
    goal.use_start = True

    send = action.send_goal_async(goal)
    assert spin_until(node, lambda: send.done(), 15.0), "goal was never sent"
    handle = send.result()
    assert handle.accepted, f"{planner_id} refused the goal"
    result = handle.get_result_async()
    assert spin_until(node, lambda: result.done(), PLAN_TIMEOUT_S), \
        f"{planner_id} produced no result in {PLAN_TIMEOUT_S} s"
    outcome = result.result()
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED, \
        f"{planner_id} failed: status {outcome.status}"
    return outcome.result.path


def densify(path, step_m=0.05):
    """Sample the path at the costmap resolution.  Theta* returns any-angle
    segments; checking only the vertices would let a straight run across a
    pit through untouched."""
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    samples = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        span = math.hypot(x1 - x0, y1 - y0)
        count = max(int(span / step_m), 1)
        for i in range(count):
            t = i / count
            samples.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    samples.append(points[-1])
    return samples


def assert_path_is_safe(path, label):
    cost = fixture.seed()
    samples = densify(path)
    assert len(samples) > 50, f"{label}: a path this short is not a 12 m path"
    for x, y in samples:
        ix, iy = fixture.cell_of(x, y)
        assert 0 <= ix < cost.shape[1] and 0 <= iy < cost.shape[0], \
            f"{label}: ({x:.2f}, {y:.2f}) is outside the fixture window"
        value = cost[iy, ix]
        assert value != LETHAL, f"{label}: path crosses a lethal cell at ({x:.2f}, {y:.2f})"
        assert value != UNKNOWN, f"{label}: path crosses unknown ground at ({x:.2f}, {y:.2f})"
        touching = fixture.clearance_cells(cost, x, y, INSCRIBED_M)
        assert touching == 0, (
            f"{label}: {touching} lethal cells within {INSCRIBED_M} m of "
            f"({x:.2f}, {y:.2f}) - the rover's circle would be in the pit")


def test_theta_star_produces_a_path(client):
    path = compute_path(client, 'GridBased')
    assert isinstance(path, Path)
    assert path.header.frame_id == 'map'
    assert len(path.poses) >= 2
    start = path.poses[0].pose.position
    end = path.poses[-1].pose.position
    assert math.hypot(start.x - fixture.START[0], start.y - fixture.START[1]) < 0.5
    assert math.hypot(end.x - fixture.GOAL[0], end.y - fixture.GOAL[1]) < 0.5


def test_theta_stars_path_avoids_every_lethal_cell(client):
    assert_path_is_safe(compute_path(client, 'GridBased'), 'GridBased')


def test_the_path_went_round_the_pit_which_means_the_seed_arrived(client):
    """The assertion that proves the seed carries real cost.

    A stack whose static layer received a *flattened* seed - all free, e.g.
    trinary_costmap flipped to true, or a wrong lethal_cost_threshold, or a
    later layer overwriting the static one - plans the straight line from
    (0,0) to (12,0), which never leaves y = 0, and it passes C and D
    against that empty costmap.  This is the assertion that catches it.

    A stack that received *nothing at all* does not get this far: both
    costmaps set track_unknown_space: true and both planners set
    allow_unknown: false, so an unseeded costmap is entirely
    NO_INFORMATION and Theta*'s isUnsafeToPlan() rejects the start pose -
    the run fails at C.
    """
    path = compute_path(client, 'GridBased')
    excursion = max(abs(p.pose.position.y) for p in path.poses)
    assert excursion >= 1.5, (
        f"the path stayed within {excursion:.2f} m of the straight line: the "
        f"costmap almost certainly never received /autonomy/costmap_seed")


def test_smac_is_a_working_second_opinion(client):
    """Spec section 5: SmacPlanner2D loaded as a second named plugin for
    A/B.  A plugin that is loaded but cannot plan is not an A/B."""
    assert_path_is_safe(compute_path(client, 'SmacBased'), 'SmacBased')


# --------------------------------------------------------------------------
# G, H, I - the action the operator sends, and the single-writer rule
# --------------------------------------------------------------------------

def test_a_navigate_to_pose_goal_is_accepted_planned_and_cancellable(client):
    plans = []
    twists = []
    client.create_subscription(Path, '/plan', plans.append, 10)
    client.create_subscription(Twist, '/autonomy_twist', twists.append, 10)

    action = ActionClient(client, NavigateToPose, 'navigate_to_pose')
    assert action.wait_for_server(timeout_sec=20.0), "no navigate_to_pose server"
    goal = NavigateToPose.Goal()
    goal.pose = pose_stamped(*fixture.GOAL)

    send = action.send_goal_async(goal)
    assert spin_until(client, lambda: send.done(), 15.0)
    handle = send.result()
    assert handle.accepted, "bt_navigator refused the goal"

    assert spin_until(client, lambda: len(plans) > 0, 20.0), \
        "no /plan published for a NavigateToPose goal"
    assert_path_is_safe(plans[-1], '/plan')

    assert spin_until(client, lambda: len(twists) > 0, 20.0), \
        "nothing reached /autonomy_twist: the velocity chain is broken"

    cancel = handle.cancel_goal_async()
    assert spin_until(client, lambda: cancel.done(), 10.0), "cancel was never answered"

    twists.clear()
    spin_until(client, lambda: False, 3.0)
    if twists:
        assert twists[-1].linear.x == 0.0 and twists[-1].angular.z == 0.0, \
            "a cancelled goal must leave a zero twist behind"


def test_the_caps_hold_on_the_wire(client):
    """Whatever the controller asked for, this is what left the stack."""
    twists = []
    client.create_subscription(Twist, '/autonomy_twist', twists.append, 10)

    action = ActionClient(client, NavigateToPose, 'navigate_to_pose')
    assert action.wait_for_server(timeout_sec=20.0)
    goal = NavigateToPose.Goal()
    goal.pose = pose_stamped(*fixture.GOAL)
    send = action.send_goal_async(goal)
    assert spin_until(client, lambda: send.done(), 15.0)
    handle = send.result()
    spin_until(client, lambda: len(twists) >= 5, 20.0)
    handle.cancel_goal_async()
    spin_until(client, lambda: False, 2.0)

    assert twists, "no velocity was produced at all"
    for twist in twists:
        assert twist.linear.y == 0.0, "vy is pinned to zero at the smoother"
        assert -0.15 <= twist.linear.x <= 0.05, f"vx {twist.linear.x} outside the cap"
        assert abs(twist.angular.z) <= 0.1 + 1e-9, f"wz {twist.angular.z} outside the cap"


def test_nothing_in_this_stack_can_reach_the_chassis(stack):
    topics = subprocess.run(['ros2', 'topic', 'list'],
                            capture_output=True, text=True, check=True).stdout.split()
    assert '/rover_twist' not in topics, "only mode_supervisor may publish /rover_twist"
    assert '/manual_twist' not in topics, "nothing may publish /manual_twist, ever"
    assert '/cmd_vel' not in topics, \
        "Nav2's default velocity name is unremapped somewhere; it must be cmd_vel_nav"
    assert '/autonomy_twist' in topics


def test_autonomy_twist_has_exactly_one_publisher(client):
    publishers = client.get_publishers_info_by_topic('/autonomy_twist')
    assert len(publishers) == 1, \
        f"{[p.node_name for p in publishers]} - only the collision monitor may write it"
    assert publishers[0].node_name == 'collision_monitor'
