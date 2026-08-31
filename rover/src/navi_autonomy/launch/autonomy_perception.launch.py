"""tile_aggregator and traversability_layer, the Orin's autonomy perception.

Not included by rover/start_navi.sh: SP9's Nav2 bringup includes this file,
and the two of them start together or not at all - a costmap seed with no
planner is dead weight, and a planner with no seed plans through holes.

Nothing in this file publishes a twist. Both nodes are read-only with respect
to the chassis.

Pure-pipeline cost at the full 960 x 960 window, measured 2026-08-31 on this
laptop (Intel Core i7-9750H, 6c/12t), synthetic elevation with 20% NaN holes
(the brief's timing script, third of three warmed-up runs): seed_from_elevation
(traversability_layer's derive, slope/step/roughness/valid plus the cost seed)
210.4 ms; build_grid_map (tile_aggregator's own /autonomy/map publish, and
traversability_layer's /autonomy/traversability publish while subscribed)
26.7 ms; build_occupancy_grid (the costmap_seed publish) 0.3 ms. That easily
holds the 1 Hz publish_period_s above on this CPU; it is laptop cost, not the
Orin's - the Orin figure is SP9/SP10's to measure (spec section 5, section 11
risk 6), which SP12 re-measures in the yard, with the documented 24 m window
(window_cells=480) as the fallback if 1 Hz is not held there.
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
