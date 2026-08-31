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
