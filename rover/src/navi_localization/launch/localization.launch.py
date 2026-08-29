"""Front ZED 2i positional tracking plus the localisation status node.

Includes the wrapper's own zed_camera.launch.py so its container, its
URDF publisher and its parameter loading stay the wrapper's business; we
add one override file and one node.

Startup, measured 2026-08-29 on the Orin, time from launch to
/zed_front/zed_node/odom being advertised and to the first
/localization/status message, each run from a verified-clean Orin
(no component container; note that `pkill -x component_container_isolated`
is a no-op because comm is truncated to 15 characters - the name that
works is `component_conta`):

    this launch file, quiet domain 0    run 1   4.5 s odom / 4.6 s status
                                        run 2   4.4 s odom / 4.6 s status
                                        run 3   4.4 s odom / 4.6 s status
    wrapper defaults, quiet domain 0            4.9 s odom
    wrapper defaults, domain 0 with a
      foreign participant present              88.6 s odom

What costs the time is not libx264 - the ffmpeg publishers this file turns
off initialise in well under a second each, which is why the wrapper's own
defaults also reach odometry in 4.9 s on a quiet domain. It is ROS 2
discovery: when a foreign participant shares the domain, creating a
publisher stalls about 3 s per ten publishers (a bare rclpy node making 60
publishers took 0.1 s on a quiet domain and 17.9 s against one stale
laptop bringup), so the wrapper's ~100 publishers dominate everything else
and the 88.6 s figure above is a measurement of that, not of this package.

Pinning the image topics to raw is therefore worth keeping because nothing
on the rover reads those topics, not because it buys seconds.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Every image topic the wrapper hands to image_transport, relative to the
# node. Each gets its transport list pinned to raw in config/zed_front.yaml,
# which drops 15 ffmpeg publishers nothing on the rover reads. Their libx264
# initialisation is cheap; the 15 extra publisher creations are only
# expensive when a foreign participant shares the domain (module docstring).
#
# This is the authoritative list, taken from the parameters image_transport
# declares rather than from the wrapper's advertised topics - they are not the
# same set. Regenerate it against a running wrapper with:
#   ros2 param list /zed_front/zed_node | grep enable_pub_plugins
# confidence/confidence_map and disparity/disparity_image are advertised but
# never appear there, and point_cloud/cloud_registered does appear but goes
# through point_cloud_transport, not image_transport, so it is left alone.
IMAGE_TOPICS = [
    'rgb/image_rect_color',
    'rgb_gray/image_rect_gray',
    'rgb_raw/image_raw_color',
    'rgb_raw_gray/image_raw_gray',
    'left/image_rect_color',
    'left_gray/image_rect_gray',
    'left_raw/image_raw_color',
    'left_raw_gray/image_raw_gray',
    'right/image_rect_color',
    'right_gray/image_rect_gray',
    'right_raw/image_raw_color',
    'right_raw_gray/image_raw_gray',
    'stereo/image_rect_color',
    'stereo_raw/image_raw_color',
    'depth/depth_registered',
]

# The node name is part of the parameter name and its namespace is not: for
# /zed_front/zed_node/rgb/image_rect_color image_transport declares
# zed_node.rgb.image_rect_color.enable_pub_plugins. The value is the transport
# name, which is the pluginlib class 'image_transport/raw_pub' minus '_pub'.
NODE_NAME = 'zed_node'


def raw_only_plugin_parameters() -> dict:
    """The parameters config/zed_front.yaml carries. Keep the two in step.

    The wrapper's included launch takes no extra node parameters, so these
    cannot be passed from here; this function is how the list is generated
    and checked (see test_localization_launch.py).
    """
    return {f"{NODE_NAME}.{topic.replace('/', '.')}.enable_pub_plugins":
            ['image_transport/raw']
            for topic in IMAGE_TOPICS}


def generate_launch_description():
    share = get_package_share_directory('navi_localization')
    wrapper_share = get_package_share_directory('zed_wrapper')

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrapper_share, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed_front',
            'ros_params_override_path': os.path.join(share, 'config', 'zed_front.yaml'),
            'publish_tf': 'true',
            'publish_map_tf': 'true',
            'publish_urdf': 'true',
        }.items(),
    )

    status = Node(
        package='navi_localization',
        executable='localization_status',
        name='localization_status',
        output='screen',
    )

    # The mapper's subscription is what makes the SDK extract the fused
    # cloud at all, so it belongs in the same launch as the wrapper rather
    # than being something an operator has to remember to start.
    mapper = Node(
        package='navi_localization',
        executable='elevation_mapper',
        name='elevation_mapper',
        output='screen',
    )

    return LaunchDescription([zed, status, mapper])
