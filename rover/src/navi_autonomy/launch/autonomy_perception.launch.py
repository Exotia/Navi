"""tile_aggregator and traversability_layer, the Orin's autonomy perception.

Not included by rover/start_navi.sh: SP9's Nav2 bringup includes this file,
and the two of them start together or not at all - a costmap seed with no
planner is dead weight, and a planner with no seed plans through holes.

Nothing in this file publishes a twist. Both nodes are read-only with respect
to the chassis.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    frame_id = LaunchConfiguration('frame_id')
    window_cells = LaunchConfiguration('window_cells')
    publish_period_s = LaunchConfiguration('publish_period_s')
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        # 48 m at 0.05 m. Spec section 5's documented fallback, if the Orin
        # cannot hold 1 Hz, is 480 (a 24 m window) - measured in SP9/SP10 on
        # the Orin (spec section 5, section 11 risk 6), not guessed here;
        # SP12 re-measures it in the yard.
        DeclareLaunchArgument('window_cells', default_value='960'),
        DeclareLaunchArgument('publish_period_s', default_value='1.0'),
        Node(package='navi_autonomy', executable='tile_aggregator',
             name='tile_aggregator', output='screen',
             parameters=[{'frame_id': frame_id,
                          'window_cells': window_cells,
                          'publish_period_s': publish_period_s}]),
        Node(package='navi_autonomy', executable='traversability_layer',
             name='traversability_layer', output='screen',
             parameters=[{'frame_id': frame_id}]),
    ])
