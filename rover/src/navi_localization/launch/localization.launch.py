"""Front ZED 2i positional tracking plus the localisation status node.

Includes the wrapper's own zed_camera.launch.py so its container, its
URDF publisher and its parameter loading stay the wrapper's business; we
add one override file and one node.

Measured on the Orin 2026-08-29 (nvpmodel 25 W, camera on the bench,
NEURAL depth at 15 fps): pose 15.0 Hz, depth 15.0 Hz, GPU 29 %, CPU 15 %,
fused cloud 1.0 Hz. Tile rate and bytes/s, rover static (Task 10,
2026-08-29, same bench setup): map_tile 5.0-6.2 Hz aggregate (keepalive
plus dirty tiles from sensor noise on a still rover), 65-95 KB/s once past
the initial burst (714 KB/s for the first few messages, settling within
~10 s); elevation_mapper 6-19 % of one core; GPU 6-95 % (21 s tegrastats
sample, mean ~32 %). Driving measurement not taken: no operator available
for that session.

obstacle_tile, measured 2026-08-29 on the same bench (Task 6, rover
static, 20 s ros2 topic hz/bw sample): 2.7-2.75 Hz, 4.9-5.3 KB/s
(0.13-6.80 KB messages); elevation_mapper 12-33 % of one core (two
5 s-apart top samples).

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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from navi_localization.pose_composition import STATIC_FRAMES

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


def _number(value: float) -> str:
    """A float for a command line. `+ 0.0` folds -0.0 (which inverse()
    produces from a zero component) back to 0.0, so the arguments read like
    the constant they came from."""
    return repr(value + 0.0)


def static_transform_arguments(transform, frame_id: str, child_frame_id: str) -> list:
    """tf2_ros static_transform_publisher's named arguments (Humble). The
    positional form is deprecated and silently reorders roll/pitch/yaw."""
    return [
        '--x', _number(transform.x),
        '--y', _number(transform.y),
        '--z', _number(transform.z),
        '--qx', _number(transform.qx),
        '--qy', _number(transform.qy),
        '--qz', _number(transform.qz),
        '--qw', _number(transform.qw),
        '--frame-id', frame_id,
        '--child-frame-id', child_frame_id,
    ]


def static_frame_arguments() -> list:
    """One argument list per entry in pose_composition.STATIC_FRAMES.

    Separated from static_frame_nodes() so the test can call it: this launch
    file's generate_launch_description() needs the zed_wrapper share
    directory, which does not exist on the laptop.
    """
    return [static_transform_arguments(transform, parent, child)
            for parent, child, transform in STATIC_FRAMES]


def static_frame_nodes() -> list:
    """base_footprint and base_link, hung under the frame the ZED wrapper
    owns.

    Two static_transform_publisher processes rather than a broadcaster node
    of our own: the transforms are fixed, tf2_ros already latches /tf_static
    correctly, and starting them here means they live and die with the
    wrapper they hang from. The numbers are not retyped - they are
    pose_composition's constants, the same ones localization_status uses to
    re-express the pose, so a re-measured mount is still one number in one
    place.

    Deliberately not a URDF-wide state publisher: that would publish the
    URDF root and make zed_front_camera_link a child of base_link, giving
    that link a second parent and splitting the tree the wrapper owns.
    """
    names = ['camera_to_base_footprint_tf', 'base_footprint_to_base_link_tf']
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=arguments,
            output='screen',
        )
        for name, arguments in zip(names, static_frame_arguments())
    ]


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
    #
    # startup_map has no DeclareLaunchArgument of its own: start_navi.sh is
    # the only caller, and ros2 launch sets a launch configuration from a
    # bare `name:=value` on its command line whether or not it was declared
    # - see IncludeLaunchDescription's own launch_arguments above for the
    # same mechanism in use already. The default keeps a plain `ros2 launch
    # navi_localization localization.launch.py` (no start_navi.sh in the
    # way) starting with an empty map rather than failing to resolve.
    mapper = Node(
        package='navi_localization',
        executable='elevation_mapper',
        name='elevation_mapper',
        parameters=[{'startup_map': LaunchConfiguration('startup_map', default='')}],
        output='screen',
    )

    return LaunchDescription([zed, status, mapper] + static_frame_nodes())
