"""Every number the spec fixes, read back out of params/nav2_rover.yaml.

This is the file-level half of the guard. The graph-level half is in
test_offline_planning.py, which asks the running nodes what they think
their parameters are - because Nav2 ignores a key it does not know, and a
typo here would otherwise configure the stack with the plugin's defaults
and say nothing at all.
"""

import os

import yaml

PARAMS = os.path.join(os.path.dirname(__file__), '..', 'params', 'nav2_rover.yaml')


def params():
    with open(PARAMS) as handle:
        return yaml.safe_load(handle)


def node(name):
    return params()[name]['ros__parameters']


def costmap(which):
    return params()[which][which]['ros__parameters']


# -- the speed caps (spec section 10) ---------------------------------------

def test_the_velocity_smoother_carries_the_manual_cap():
    smoother = node('velocity_smoother')
    assert smoother['max_velocity'] == [0.05, 0.0, 0.2]
    assert smoother['min_velocity'] == [-0.15, 0.0, -0.2]


def test_vy_is_pinned_to_zero_at_the_smoother():
    smoother = node('velocity_smoother')
    assert smoother['max_velocity'][1] == 0.0
    assert smoother['min_velocity'][1] == 0.0
    assert smoother['max_accel'][1] == 0.0
    assert smoother['max_decel'][1] == 0.0


def test_the_angular_acceleration_limit_is_the_one_the_spec_names():
    smoother = node('velocity_smoother')
    assert smoother['max_accel'][2] == 0.5
    assert smoother['max_decel'][2] == -0.5


def test_the_controller_never_asks_for_more_than_the_smoother_passes():
    follow = node('controller_server')['FollowPath']
    smoother = node('velocity_smoother')
    assert follow['desired_linear_vel'] == smoother['max_velocity'][0] == 0.05
    assert follow['rotate_to_heading_angular_vel'] == smoother['max_velocity'][2] == 0.2


# -- no reversing (spec section 5) ------------------------------------------

def test_the_controller_may_not_reverse():
    assert node('controller_server')['FollowPath']['allow_reversing'] is False


def test_reverse_speed_is_capped_at_the_spec_floor():
    assert node('velocity_smoother')['min_velocity'][0] >= -0.15


def test_every_collision_polygon_looks_only_forwards():
    monitor = node('collision_monitor')
    assert monitor['polygons'], "at least one polygon, or nothing is watched"
    for name in monitor['polygons']:
        points = monitor[name]['points']
        xs = points[0::2]
        assert min(xs) >= 0.10, \
            f"{name} has a point behind the x >= 0.10 line: {points}"
        assert max(xs) > 0.0, f"{name} is degenerate: {points}"


# -- the seed contract (SP7) ------------------------------------------------

def test_both_costmaps_read_the_seed_at_the_elevation_resolution():
    for which in ('global_costmap', 'local_costmap'):
        layer = costmap(which)['static_layer']
        assert costmap(which)['resolution'] == 0.05
        assert layer['map_topic'] == '/autonomy/costmap_seed'
        assert layer['map_subscribe_transient_local'] is True
        assert layer['subscribe_to_updates'] is False


def test_the_scaled_cost_band_survives_the_static_layer():
    """trinary_costmap true would collapse SP7's 0..99 band to free/lethal
    and throw away every gradient the traversability layer computed.

    These three are Costmap2DROS's own parameters (the OccupancyGrid ->
    Costmap translation table shared by every layer), not static_layer's -
    confirmed against a live stack in Task 6: nesting them under
    static_layer is an undeclared parameter name Nav2 silently ignores,
    which is exactly the failure mode test_offline_planning.py's
    parameter-fidelity test exists to catch."""
    for which in ('global_costmap', 'local_costmap'):
        top = costmap(which)
        assert top['trinary_costmap'] is False
        assert top['lethal_cost_threshold'] == 100
        assert top['unknown_cost_value'] == -1


def test_unseen_ground_is_not_driveable_ground():
    for which in ('global_costmap', 'local_costmap'):
        assert costmap(which)['track_unknown_space'] is True
    assert node('planner_server')['GridBased']['allow_unknown'] is False
    assert node('planner_server')['SmacBased']['allow_unknown'] is False


def test_the_global_costmap_is_the_48_m_window():
    globals_ = costmap('global_costmap')
    assert globals_['width'] == 48 and globals_['height'] == 48
    assert globals_['rolling_window'] is True


# -- the cloud layers are wired but off (no cloud_filter, no camera) --------

def test_the_cloud_layers_are_configured_and_disabled():
    assert costmap('global_costmap')['obstacle_layer']['enabled'] is False
    assert costmap('local_costmap')['voxel_layer']['enabled'] is False
    assert node('collision_monitor')['points_filtered']['enabled'] is False


def test_the_cloud_layers_point_at_the_topic_cloud_filter_will_publish():
    assert (costmap('global_costmap')['obstacle_layer']['points_filtered']['topic']
            == '/autonomy/points_filtered')
    assert (costmap('local_costmap')['voxel_layer']['points_filtered']['topic']
            == '/autonomy/points_filtered')
    assert node('collision_monitor')['points_filtered']['topic'] == '/autonomy/points_filtered'


# -- frames and odometry (spec section 6, SP6) ------------------------------

def test_nav2_reads_the_odometry_sp6_publishes():
    assert node('controller_server')['odom_topic'] == '/localization/odom_local'
    assert node('bt_navigator')['odom_topic'] == '/localization/odom_local'
    assert node('velocity_smoother')['odom_topic'] == '/localization/odom_local'


def test_every_node_stands_on_base_footprint():
    assert node('bt_navigator')['robot_base_frame'] == 'base_footprint'
    assert node('behavior_server')['robot_base_frame'] == 'base_footprint'
    assert node('collision_monitor')['base_frame_id'] == 'base_footprint'
    for which in ('global_costmap', 'local_costmap'):
        assert costmap(which)['robot_base_frame'] == 'base_footprint'
    assert costmap('global_costmap')['global_frame'] == 'map'
    assert costmap('local_costmap')['global_frame'] == 'odom'


# -- the velocity chain ------------------------------------------------------

def test_nav2_writes_only_to_autonomy_twist():
    monitor = node('collision_monitor')
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == '/autonomy_twist'


# -- the planners (spec section 5) ------------------------------------------

def test_theta_star_plans_and_smac_is_loaded_beside_it_for_ab():
    planner = node('planner_server')
    assert planner['planner_plugins'] == ['GridBased', 'SmacBased']
    assert planner['GridBased']['plugin'] == 'nav2_theta_star_planner/ThetaStarPlanner'
    assert planner['SmacBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'


def test_no_parameter_this_build_does_not_have_is_set():
    """Verified against the installed 1.1.20 binaries on 2026-08-31 by
    dumping the plugin .so strings: none of these keys is declared, and
    Nav2 would ignore them without a word.

    (SmacPlanner2D's use_final_approach_orientation IS declared in 1.1.20 -
    it is simply not set here, which is a choice, not an absence.)"""
    assert 'w_heuristic_cost' not in node('planner_server')['GridBased']
    follow = node('controller_server')['FollowPath']
    assert 'use_fixed_curvature_lookahead' not in follow
    assert 'curvature_lookahead_dist' not in follow
    assert 'odom_topic' not in node('behavior_server')


def test_the_controller_is_the_shim_wrapping_pure_pursuit():
    follow = node('controller_server')['FollowPath']
    assert follow['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert (follow['primary_controller']
            == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')


def test_the_inflation_factors_agree_between_layer_and_controller():
    """RPP scales speed by cost using its own copy of the inflation curve;
    if the two disagree it slows down in the wrong places and says nothing."""
    follow = node('controller_server')['FollowPath']
    for which in ('global_costmap', 'local_costmap'):
        assert (costmap(which)['inflation_layer']['cost_scaling_factor']
                == follow['inflation_cost_scaling_factor'] == 10.0)


def test_the_footprint_is_the_rovers_rectangle_with_a_5cm_shell():
    # Operator decision 2026-08-31: the circle (corner radius 0.65) kept the
    # rover ~20 cm off walls; the true rectangle lets it pass straight at
    # ~5 cm while orientation-aware collision checks still refuse turns
    # that would swing a corner in.
    for which in ('global_costmap', 'local_costmap'):
        assert '0.445' in str(costmap(which)['footprint'])
        assert 'robot_radius' not in costmap(which)


def test_the_arrival_guard_is_looser_than_the_goal_checker_but_still_tight():
    """The GoalReached verification in navigate_to_pose_no_reverse.xml reads
    goal_reached_tol off bt_navigator.  It must sit strictly above the
    controller's xy tolerance (so a genuine arrival never flaps between the
    two frames) and well below any real leg length (so the tf-glitch success
    at the odom origin - the instant point-turn arrival seen live - can
    never pass)."""
    xy = node('controller_server')['general_goal_checker']['xy_goal_tolerance']
    guard = node('bt_navigator')['goal_reached_tol']
    assert xy == 0.25
    assert guard == 0.50
    assert xy < guard <= 1.0
