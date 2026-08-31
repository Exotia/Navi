"""The launch file's wiring as data, so it can be tested without launching.

generate_launch_description() needs package share directories; these
constants and helpers do not, and they are what the wiring actually is.
"""

import os

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]

LAUNCH_ARGUMENTS = {
    'params_file': '',          # filled from the share directory at launch
    'autostart': 'true',
    'perception': 'true',       # include SP7's tile_aggregator + traversability_layer
    'bench_fixture': 'false',   # fake seed, frames and odometry; NEVER with a ZED running
    'log_level': 'info',
}

ODOM_TOPIC = '/localization/odom_local'

# Nav2's internal velocity names, moved off "cmd_vel" so nothing can
# subscribe to an unmonitored velocity by accident.  The only name that
# leaves this stack is /autonomy_twist, and it is set as a parameter on the
# collision monitor (cmd_vel_out_topic), not as a remap.
_REMAPPINGS = {
    'controller_server': [('cmd_vel', 'cmd_vel_nav')],
    'behavior_server': [('cmd_vel', 'cmd_vel_nav'), ('odom', ODOM_TOPIC)],
    'velocity_smoother': [('cmd_vel', 'cmd_vel_nav')],
    'planner_server': [],
    'bt_navigator': [],
    'collision_monitor': [],
}


def remappings(node_name: str) -> list:
    """The (from, to) pairs for one node.  behavior_server's odom remap is
    not cosmetic: that node has no odom_topic parameter - its OdomSmoother
    hard-codes the name - so this is the only way it reads SP6's odometry."""
    return list(_REMAPPINGS[node_name])


def bt_rewrites(share_dir: str) -> dict:
    """The two behaviour-tree paths, as RewrittenYaml param_rewrites."""
    trees = os.path.join(share_dir, 'behavior_trees')
    return {
        'default_nav_to_pose_bt_xml':
            os.path.join(trees, 'navigate_to_pose_no_reverse.xml'),
        'default_nav_through_poses_bt_xml':
            os.path.join(trees, 'navigate_through_poses_no_reverse.xml'),
    }
