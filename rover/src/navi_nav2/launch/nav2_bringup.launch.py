"""Nav2 for the Asterope rover: six lifecycle nodes and one manager.

    ros2 launch navi_nav2 nav2_bringup.launch.py

Arguments:
    params_file:=<path>     override params/nav2_rover.yaml
    autostart:=false        bring the nodes up unconfigured (debugging)
    perception:=false       do not include SP7's tile_aggregator and
                            traversability_layer (the offline test and the
                            bench smoke supply the seed themselves)
    bench_fixture:=true     publish the generated fixture seed, fake
                            map->odom and odom->base_footprint, and fake
                            /localization/odom_local.  NEVER run this with
                            the ZED up: the wrapper owns map->odom and this
                            would give base_footprint a second parent.
    log_level:=debug

The velocity chain:

    controller_server ---.
    behavior_server -----+--> cmd_vel_nav --> velocity_smoother
                                                    |
                                            cmd_vel_smoothed
                                                    |
                                            collision_monitor
                                                    |
                                             /autonomy_twist
                                                    |
                                             mode_supervisor  (SP5)
                                                    |
                                              /rover_twist

Nothing in this launch publishes /rover_twist or /manual_twist.  The
collision monitor is last on purpose: it has the final word on every
velocity, including the behaviours' recovery motions.

MUST DO at the next camera session (none of it is testable without a ZED):
  1. Set global_costmap.obstacle_layer.enabled, local_costmap.voxel_layer.
     enabled and collision_monitor.points_filtered.enabled to true, once
     cloud_filter publishes /autonomy/points_filtered.
  2. Check the seed lines up with real terrain: drive to a known rock, and
     confirm the lethal cells in /global_costmap/costmap sit on it and not
     a metre away.  A misaligned seed looks entirely plausible.
  3. Re-measure CPU with perception live (spec section 11 risk 6) and
     compare against the camera-less numbers recorded below.
  4. Watch the RotationShim hand over to RPP on the real chassis and
     measure the steering slew before any speed stage is raised.
  5. Confirm the collision polygons against the real footprint with the
     rover on blocks before they are trusted in the yard.

Needed for real on 2026-08-31 (Task 7): the package *count* matched
(31 == 31) but one member did not - ros-humble-nav2-msgs was stuck at
1.1.18-1jammy.20250326.040645 on the Orin while every other nav2-*
package there had already moved to 1.1.20 from a 2026-03-26 rebuild
(the laptop's are all 1.1.20 from a 2026-08-04 rebuild). bt_navigator
links against the newer nav2_msgs symbols (ComputeAndTrackRoute) that
1.1.18 does not export, so it failed to configure with a pluginlib
"undefined symbol" error exactly as this section warned it would - the
package-count gate does not catch a stale version of a package that is
still present. The fix, via this runbook: fetched
ros-humble-nav2-msgs_1.1.20-1jammy.20260725.154919_arm64.deb through
the laptop and rsynced it to the Orin, but `dpkg -i` could not run - the
ssh session has no interactive TTY and passwordless sudo is not
configured on the Orin (`sudo -n` fails: "a password is required"), the
same constraint start_navi.sh's ensure_navi_alias already documents.
The .deb is staged at ~/orin-debs on the Orin; someone with a terminal
there needs to run `sudo dpkg -i ~/orin-debs/packages.ros.org/ros2/ubuntu/pool/main/r/ros-humble-nav2-msgs/ros-humble-nav2-msgs_1.1.20-1jammy.20260725.154919_arm64.deb`
before the smoke launch, the CPU/RAM measurement and
test_offline_planning.py can be re-attempted on the Orin. Not done here
because of the missing sudo access, not because the fix is unclear.

# Carrying a package to the Orin (no internet there, arm64, jammy):
#
#   # 1. the Orin computes its own URIs - right architecture, right
#   #    versions, only what it is actually missing:
#   ssh star@a_navi 'apt-get install --print-uris -y --no-install-recommends \
#       ros-humble-<pkg> | grep -oP "(?<=^.)http[^\x27]+" > /tmp/uris.txt; wc -l < /tmp/uris.txt'
#   scp star@a_navi:/tmp/uris.txt /tmp/uris.txt
#   # 2. the laptop, which has internet, does the downloading:
#   wget -x -P ~/orin-debs -i /tmp/uris.txt
#   # 3. carry and install:
#   rsync -a ~/orin-debs/ star@a_navi:~/orin-debs/
#   ssh star@a_navi 'sudo dpkg -i $(find ~/orin-debs -name "*.deb")'
#
# Do NOT apt-get download on the laptop: it is amd64 and the Orin is arm64,
# and the deb would install and then fail to load.

Orin measurements (camera-less, fixture seed, Task 7):
    NOT TAKEN 2026-08-31: the stale nav2-msgs package above (bt_navigator
    pluginlib failure) meant the stack never reached six-node active on
    the Orin during this task - lifecycle_manager aborted bringup with
    bt_navigator stuck at unconfigured. controller_server, planner_server
    and behavior_server did reach inactive (configured) before the
    abort; bt_navigator, velocity_smoother and collision_monitor never
    got past unconfigured. Re-run the smoke launch and this measurement
    once the nav2-msgs deb above is installed with sudo on the Orin
    itself. Laptop-side CPU/RAM was out of scope for Task 6 (offline
    planning only); no numbers exist yet from either machine.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from navi_nav2 import bringup


def generate_launch_description():
    share = get_package_share_directory('navi_nav2')
    default_params = os.path.join(share, 'params', 'nav2_rover.yaml')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    perception = LaunchConfiguration('perception')
    bench_fixture = LaunchConfiguration('bench_fixture')
    log_level = LaunchConfiguration('log_level')

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('bench_fixture', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    # The behaviour-tree paths are the only thing the parameter file cannot
    # spell for itself: they are absolute paths into this package's share.
    configured = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=bringup.bt_rewrites(share),
        convert_types=True)

    def server(package, executable, name):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[configured],
            remappings=bringup.remappings(name),
            arguments=['--ros-args', '--log-level', log_level],
        )

    servers = GroupAction([
        server('nav2_controller', 'controller_server', 'controller_server'),
        server('nav2_planner', 'planner_server', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', 'behavior_server'),
        server('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        server('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
        server('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            # The node list comes from the parameter file like everything
            # else - one place, and test_bringup_launch.py checks it still
            # matches bringup.LIFECYCLE_NODES. autostart is the one thing
            # the command line may override.
            parameters=[configured, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])

    sp7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('navi_autonomy'),
            'launch', 'autonomy_perception.launch.py')),
        condition=IfCondition(perception))

    fixture = Node(
        package='navi_nav2',
        executable='fixture_seed_publisher',
        name='fixture_seed_publisher',
        output='screen',
        parameters=[{'bench_frames': True}],
        condition=IfCondition(bench_fixture),
    )

    return LaunchDescription(arguments + [sp7, fixture, servers])
