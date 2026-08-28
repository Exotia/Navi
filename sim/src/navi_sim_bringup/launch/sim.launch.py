# sim/src/navi_sim_bringup/launch/sim.launch.py
"""Bring up Gazebo with the rover in the scanned site.

The map mesh path is a launch argument rather than a fixed path: the .obj is
gitignored and 161 MB, so it is not part of this repository and cannot be
assumed to sit anywhere in particular.
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world_with_mesh(context, *args, **kwargs):
    share = get_package_share_directory("navi_sim_bringup")
    mesh = LaunchConfiguration("map_mesh").perform(context)
    if not os.path.exists(mesh):
        raise RuntimeError(
            f"map mesh not found: {mesh}\n"
            "Pass map_mesh:=/path/to/Model3D_mesh2.obj - the mesh is "
            "gitignored, so it is not in the repository.")

    source = os.path.join(share, "worlds", "site.world")
    with open(source) as handle:
        world = handle.read().replace("MAP_MESH_PATH", mesh)

    generated = os.path.join(tempfile.mkdtemp(prefix="navi_sim_world_"), "site.world")
    with open(generated, "w") as handle:
        handle.write(world)

    robot = os.path.join(share, "urdf", "asterope_sim.urdf.xacro")
    description = os.popen(f"xacro {robot}").read()

    return [
        ExecuteProcess(
            cmd=["gazebo", "--verbose", generated,
                 "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so"],
            output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", "asterope",
                        "-z", "0.05"],
             output="screen"),
        Node(package="navi_sim_ik", executable="sim_ik_node", output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_mesh",
            default_value=os.path.expanduser("~/star/Navi/Model3D_mesh2.obj"),
            description="Path to the scanned site mesh (.obj)"),
        OpaqueFunction(function=_world_with_mesh),
    ])
