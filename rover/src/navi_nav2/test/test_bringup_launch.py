"""The launch file's wiring, asserted without launching anything.

generate_launch_description() is not called: it resolves package share
directories and would need the workspace installed.  Everything asserted
here is a module-level constant or a small pure helper, which is why they
exist as such.
"""

import os
import xml.etree.ElementTree as ET

import pytest

from navi_nav2 import bringup

TREES = os.path.join(os.path.dirname(__file__), '..', 'behavior_trees')


# -- the behaviour trees ----------------------------------------------------

@pytest.mark.parametrize('tree', ['navigate_to_pose_no_reverse.xml',
                                  'navigate_through_poses_no_reverse.xml'])
def test_backup_stays_inside_the_spec_cap(tree):
    root = ET.parse(os.path.join(TREES, tree)).getroot()
    backups = list(root.iter('BackUp'))
    for backup in backups:
        assert float(backup.get('backup_dist')) <= 0.6, "spec section 5: BackUp capped 0.6 m"
        assert float(backup.get('backup_speed')) <= 0.15


@pytest.mark.parametrize('tree', ['navigate_to_pose_no_reverse.xml',
                                  'navigate_through_poses_no_reverse.xml'])
def test_nothing_else_in_the_tree_drives_backwards(tree):
    root = ET.parse(os.path.join(TREES, tree)).getroot()
    for node in root.iter('DriveOnHeading'):
        assert float(node.get('dist_to_travel', '0')) >= 0.0
    assert not list(root.iter('AssistedTeleop')), \
        "assisted teleop is a second velocity source; the supervisor owns that job"


def test_follow_path_success_is_verified_against_the_real_goal():
    """Nav2 humble's controller checks the goal in the odom frame against the
    path end transformed map->odom, and ignores a failed transform - the
    comparison then runs against a default pose at the odom origin, which is
    where the rover sits while it point-turns at the start of a run.  The
    tree must therefore re-measure the distance to the true goal in the map
    frame before the navigation may report success."""
    root = ET.parse(os.path.join(TREES, 'navigate_to_pose_no_reverse.xml')).getroot()
    guards = list(root.iter('GoalReached'))
    assert len(guards) == 1, "exactly one arrival verification"
    guard = guards[0]
    assert guard.get('goal') == '{goal}'
    assert guard.get('global_frame') == 'map'
    assert guard.get('robot_base_frame') == 'base_footprint'
    # It must sit AFTER FollowPath inside the same sequence, so it gates the
    # success rather than the start.
    for sequence in root.iter('Sequence'):
        children = list(sequence)
        if any(child.tag == 'GoalReached' for child in children):
            assert children[-1].tag == 'GoalReached'
            assert any(child.iter('FollowPath') is not None and
                       list(child.iter('FollowPath')) for child in children[:-1]), \
                "the guard must follow the FollowPath it verifies"
            break
    else:
        raise AssertionError("GoalReached is not inside a Sequence")


# -- the launch wiring ------------------------------------------------------

def test_the_lifecycle_manager_owns_exactly_the_six_servers():
    assert bringup.LIFECYCLE_NODES == [
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'velocity_smoother', 'collision_monitor']


def test_the_velocity_chain_ends_at_autonomy_twist_and_starts_nowhere_else():
    """controller and behaviours -> cmd_vel_nav -> smoother ->
    cmd_vel_smoothed -> collision monitor -> /autonomy_twist.  The
    intermediate names are remapped away from Nav2's default 'cmd_vel' so
    a stray subscriber cannot pick up an unmonitored velocity."""
    assert bringup.remappings('controller_server') == [
        ('cmd_vel', 'cmd_vel_nav')]
    assert bringup.remappings('behavior_server') == [
        ('cmd_vel', 'cmd_vel_nav'), ('odom', '/localization/odom_local')]
    assert bringup.remappings('velocity_smoother') == [('cmd_vel', 'cmd_vel_nav')]
    # The smoother's output keeps its own name, and the collision monitor
    # takes it by parameter (cmd_vel_in_topic), not by remap.
    assert bringup.remappings('collision_monitor') == []


def test_the_lifecycle_manager_and_the_launch_agree_on_the_node_list():
    """Two places name the six nodes - the parameter file and this module -
    and a manager waiting for a node nobody starts hangs the whole bringup
    with a message about bonds."""
    import os

    import yaml
    params = os.path.join(os.path.dirname(__file__), '..', 'params', 'nav2_rover.yaml')
    with open(params) as handle:
        managed = yaml.safe_load(handle)[
            'lifecycle_manager_navigation']['ros__parameters']['node_names']
    assert managed == bringup.LIFECYCLE_NODES


def test_no_node_is_remapped_onto_a_chassis_topic():
    for node in bringup.LIFECYCLE_NODES:
        for _, destination in bringup.remappings(node):
            assert destination not in ('/rover_twist', '/manual_twist'), \
                f"{node} would write to the chassis; only mode_supervisor may"


def test_the_behaviour_trees_are_rewritten_into_the_parameters():
    rewrites = bringup.bt_rewrites('/share/navi_nav2')
    assert rewrites['default_nav_to_pose_bt_xml'].endswith(
        '/behavior_trees/navigate_to_pose_no_reverse.xml')
    assert rewrites['default_nav_through_poses_bt_xml'].endswith(
        '/behavior_trees/navigate_through_poses_no_reverse.xml')


def test_the_bringup_can_run_without_perception_for_the_offline_test():
    assert 'perception' in bringup.LAUNCH_ARGUMENTS
    assert 'bench_fixture' in bringup.LAUNCH_ARGUMENTS
    assert bringup.LAUNCH_ARGUMENTS['bench_fixture'] == 'false', \
        "the bench fixture fakes the frames the ZED owns; never on by default"


def test_an_obstructed_goal_falls_back_to_smac_with_its_tolerance():
    """Theta* has no goal tolerance: a waypoint inside a lethal splash (a
    phantom night-time step, or a click centimetres into a wall's inflation)
    ends the whole run with "start or goal pose are an obstacle".  The tree
    retries the same goal with SmacBased, whose tolerance plans to the
    nearest valid pose instead - holes stay lethal to drive over."""
    root = ET.parse(os.path.join(TREES, 'navigate_to_pose_no_reverse.xml')).getroot()
    fallbacks = [f for f in root.iter('Fallback')
                 if [c.get('planner_id') for c in f] == ['GridBased', 'SmacBased']]
    assert len(fallbacks) == 1, "Theta* first, Smac as the goal-tolerant retry"


def test_the_recovery_ladder_is_clear_look_around_wait_and_never_reverse():
    """The Spin is the look-around that heals a poisoned map: the camera
    must re-observe night-drift phantoms for the fusion to erase them, and
    a point turn on the wheel-proven trail is the one motion that stands on
    guaranteed ground while doing it. BackUp must never return - reversing
    is blind on this rover."""
    root = ET.parse(os.path.join(TREES, 'navigate_to_pose_no_reverse.xml')).getroot()
    robins = list(root.iter('RoundRobin'))
    assert len(robins) == 1
    children = [c.tag for c in robins[0]]
    assert children == ['Sequence', 'Spin', 'Wait']
    assert not list(root.iter('BackUp'))
