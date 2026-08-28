# sim/src/navi_sim_bringup/launch/sim.launch.py
"""Bring up Gazebo with the rover in the scanned site.

The map mesh path is a launch argument rather than a fixed path: the .obj is
gitignored and 161 MB, so it is not part of this repository and cannot be
assumed to sit anywhere in particular.
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description(xacro_path):
    """Expands the xacro into a URDF string, or raises.

    This used to be os.popen(f"xacro {robot}").read(), which throws the
    exit status away. Every failure mode - xacro not installed, an XML
    error in the file, an unresolvable $(find navi_sim_bringup) because the
    workspace was not sourced - returned an empty string, which
    robot_state_publisher then published as the robot description and
    spawn_entity.py found nothing to spawn: Gazebo came up with a world and
    no rover, with no error anywhere. Not hypothetical - it happened on
    this branch. The map mesh a few lines below gets an existence check and
    a three-line message; the robot description, which is the whole point
    of the launch, got none.
    """
    command = ["xacro", xacro_path]
    printable = " ".join(command)
    try:
        description = subprocess.check_output(
            command, stderr=subprocess.PIPE, text=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"could not run `{printable}`: {exc}\n"
            "xacro is not on PATH - source /opt/ros/humble/setup.bash "
            "(and sim/install/setup.bash) first.") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"`{printable}` failed with exit status {exc.returncode}.\n"
            f"stderr:\n{exc.stderr}") from exc

    # A command can succeed and still produce nothing useful, and an empty
    # description fails silently downstream rather than here.
    if not description.strip():
        raise RuntimeError(
            f"`{printable}` succeeded but produced an empty robot "
            "description. robot_state_publisher would publish nothing and "
            "spawn_entity.py would spawn nothing, leaving a world with no "
            "rover. Check that sim/install/setup.bash is sourced so "
            "$(find navi_sim_bringup) resolves.")
    return description


def _world_with_mesh(context, *args, **kwargs):
    share = get_package_share_directory("navi_sim_bringup")
    mesh = LaunchConfiguration("map_mesh").perform(context)
    twist_topic = LaunchConfiguration("twist_topic").perform(context)
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
    description = _robot_description(robot)

    return [
        ExecuteProcess(
            # publish_rate is libgazebo_ros_init's /clock rate, and it
            # defaults to 10 Hz. That default is not survivable here:
            # sim_ik_node ticks on the simulation clock with a 0.06 s
            # timestep, a ROS-time timer can only notice time passing when
            # /clock arrives, and 0.06 does not divide 0.1 - so the tick
            # quantised to a measured 13.13 Hz instead of 16.67 and
            # /sim_odom reported 7.204 m where Gazebo had moved the model
            # 8.954 m. 100 Hz makes the granularity 0.01 s, which divides
            # 0.06 exactly. Anything set here must keep dividing
            # kTimestepSeconds; see the comment on that constant.
            #
            # It has to be the long --param. Gazebo's own -p is --play, so
            # `-p publish_rate:=100.0` is swallowed as a log file to replay
            # and gzserver dies with "Invalid logfile [publish_rate:=100.0]".
            # Confirmed both ways.
            #
            # Accepted cost, not an oversight: this simulation publishes
            # /clock onto the rover's shared DDS domain, and this makes that
            # ten times chattier. Harmless today because no node on the
            # rover sets use_sim_time, so nothing out there reads /clock at
            # all - but if one ever does, its clock would be driven by a
            # laptop's Gazebo, which is a trade someone took deliberately
            # rather than a thing to discover in the field.
            cmd=["gazebo", "--verbose", generated,
                 "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so",
                 "--ros-args", "--param", "publish_rate:=100.0"],
            output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}],
             output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", "asterope",
                        "-z", "0.05"],
             output="screen"),
        # use_sim_time matters here beyond timestamp cosmetics:
        # gazebo_ros_joint_pose_trajectory compares each trajectory's
        # header.stamp against the simulation clock to decide when to
        # apply it. Without this, sim_ik_node's now() is the wall clock,
        # so every stamp lands far in the sim's future and the plugin
        # never applies a single position - confirmed by watching
        # `gz model -m asterope -i` sit at a fixed joint angle no matter
        # what /test_manual_twist commanded.
        Node(package="navi_sim_ik", executable="sim_ik_node", output="screen",
             parameters=[{"use_sim_time": True}],
             remappings=[("/manual_twist", twist_topic)]),
        Node(package="navi_sim_video", executable="sim_video_sender", output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_mesh",
            default_value=os.path.expanduser("~/star/Navi/Model3D_mesh2.obj"),
            description="Path to the scanned site mesh (.obj)"),
        DeclareLaunchArgument(
            "twist_topic",
            default_value="/manual_twist",
            description=(
                "Topic the simulation drives from. Defaults to the rover's own "
                "/manual_twist, read off its ROS graph over DDS. Point it at a "
                "scratch topic to exercise the simulation without publishing "
                "onto the topic that drives the physical rover.")),
        OpaqueFunction(function=_world_with_mesh),
    ])
