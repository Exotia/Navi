"""Front ZED 2i positional tracking plus the localisation status node.

Includes the wrapper's own zed_camera.launch.py so its container, its
URDF publisher and its parameter loading stay the wrapper's business; we
add one override file and one node.

Startup, measured 2026-08-29 on the Orin, time from launch to
/zed_front/zed_node/odom being advertised:

                              domain 0 as found    remote participants excluded
    wrapper defaults               88.6 s                     4.9 s
    this launch file          20.2 / 21.5 / 22.6 / 35.9 s     4.5 / 4.7 s

Read the right-hand column first: pinning the image topics to raw saves
almost nothing on a quiet ROS domain. The left-hand column is what the rover
actually did on the day, and the reason both numbers there are large is not
this package.

Creating a ROS publisher on this rover's domain 0 stalls for 3.06 s roughly
every ten publishers. It is nothing to do with the ZED: a bare rclpy node
creating 60 publishers takes 0.1 s on a quiet domain and 18 s on domain 0.
The cause is four /robot_state_publisher nodes belonging to another machine
on the LAN (the ground station, running the rover simulation) that share
ROS domain 0; with ROS_LOCALHOST_ONLY=1 they are invisible and the stalls
disappear. So the wrapper's start-up here is roughly (publishers created)
x 0.3 s, which is why dropping the 15 ffmpeg publishers takes it from 88.6 s
to about 21 s, and why the remaining 21 s is unstable - it tracks whatever
the ground station is doing, not anything in this file.

The fix belongs to the fleet, not here: give the rover and the ground
station's simulation different ROS_DOMAIN_IDs, or keep the simulation off
the rover's domain. Until then, budget 20-40 s for this launch file rather
than the 4.5 s it needs on its own.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Every image topic the wrapper hands to image_transport, relative to the
# node. Each gets its transport list pinned to raw in config/zed_front.yaml,
# which drops 15 ffmpeg publishers nothing on the rover reads. The libx264
# initialisation those publishers do is cheap; what they cost is 15 more
# publisher creations, and on this rover that is the expensive part (see the
# module docstring).
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

    return LaunchDescription([zed, status])
