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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from navi_sim_bringup.world_composition import compose_world, site_scan_required


def _robot_description(xacro_path, planar_move: bool):
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
    # planar_move:= is passed explicitly in both modes rather than relying on
    # the xacro default, so the expansion this function produces is a
    # function of its arguments alone and reading the caller is enough to
    # know which plugins are in the model.
    command = ["xacro", xacro_path, f"planar_move:={'true' if planar_move else 'false'}"]
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
    mode = LaunchConfiguration("mode").perform(context)
    sim_domain = int(LaunchConfiguration("sim_domain").perform(context))
    rover_domain = int(LaunchConfiguration("rover_domain").perform(context))

    if mode not in ("simulation", "semi"):
        raise RuntimeError(
            f"mode must be 'simulation' or 'semi', not {mode!r}.\n"
            "  simulation - the IK-driven, dead-reckoned rover on whatever "
            "domain this process is on. What has always run here.\n"
            "  semi       - the body pose comes from the rover's own "
            "/localization/pose, bridged in from domain "
            f"{rover_domain}. Run it on its own ROS_DOMAIN_ID.")

    # The one place the two modes differ, named once. Everything below reads
    # this rather than re-testing the string, so a third mode cannot be
    # half-added.
    external_pose = mode == "semi"

    if site_scan_required(mode) and not os.path.exists(mesh):
        raise RuntimeError(
            f"map mesh not found: {mesh}\n"
            "Pass map_mesh:=/path/to/Model3D_mesh2.obj - the mesh is "
            "gitignored, so it is not in the repository. In semi mode it is "
            "not needed at all: the ground is the rover's own map.")

    with open(os.path.join(share, "worlds", "site.world")) as handle:
        world_text = handle.read()
    with open(os.path.join(share, "worlds", "site_scan.model")) as handle:
        scan_text = handle.read()
    world = compose_world(world_text, scan_text, mode, mesh)

    generated = os.path.join(tempfile.mkdtemp(prefix="navi_sim_world_"), "site.world")
    with open(generated, "w") as handle:
        handle.write(world)

    robot = os.path.join(share, "urdf", "asterope_sim.urdf.xacro")
    # planar_move and sim_ik_node's set_entity_state would both be writing
    # the model's pose every tick. Two writers of one pose fight.
    description = _robot_description(robot, planar_move=not external_pose)

    actions = [
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
            output="screen",
            # Without Gazebo there is no camera, no rover and nothing to
            # bridge into, yet by default the launch keeps every other
            # process alive: the video sender then sits for hours feeding
            # an encoder no frames, start_sim.sh never returns, and the
            # ground station's "NO FRAMES - nothing arriving" has no
            # matching error anywhere. Gazebo going away ends the launch,
            # so it is one exit code in one terminal instead.
            on_exit=Shutdown(reason="Gazebo exited - the simulation is over")),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}],
             output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", "asterope",
                        "-z", "0.05"],
             output="screen"),
    ]

    # use_sim_time matters here beyond timestamp cosmetics:
    # gazebo_ros_joint_pose_trajectory compares each trajectory's
    # header.stamp against the simulation clock to decide when to
    # apply it. Without this, sim_ik_node's now() is the wall clock,
    # so every stamp lands far in the sim's future and the plugin
    # never applies a single position - confirmed by watching
    # `gz model -m asterope -i` sit at a fixed joint angle no matter
    # what /test_manual_twist commanded.
    ik_parameters = {"use_sim_time": True}
    if external_pose:
        ik_parameters.update({
            "pose_topic": "/localization/pose",
            "status_topic": "/localization/status",
            # The name spawn_entity.py above gives the model. Set together
            # with it or set_entity_state addresses a model that is not there.
            "model_name": "asterope",
            "pose_z_offset": 0.05,
        })
    actions.append(
        Node(package="navi_sim_ik", executable="sim_ik_node", output="screen",
             parameters=[ik_parameters],
             remappings=[("/manual_twist", twist_topic)]))
    actions.append(
        Node(package="navi_sim_video", executable="sim_video_sender", output="screen"))

    if external_pose:
        # The ground the rover has actually seen. Only in semi mode:
        # simulation mode has the organisers' scan and no rover to map
        # anything with.
        actions.append(Node(package="navi_sim_bringup",
                            executable="terrain_writer", output="screen"))

    if external_pose:
        # The twist entry follows twist_topic: with --twist-topic pointed at
        # a scratch topic, the bridge has to carry that one or the wheels in
        # the picture never move.
        bridged = [f"{twist_topic}:geometry_msgs/msg/Twist",
                   "/localization/pose:nav_msgs/msg/Odometry",
                   "/localization/status:std_msgs/msg/String",
                   "/localization/map_tile:grid_map_msgs/msg/GridMap"]
        arguments = ["--rover-domain", str(rover_domain),
                     "--sim-domain", str(sim_domain)]
        for spec in bridged:
            arguments += ["--topic", spec]
        # Arguments rather than parameters: the bridge builds its two
        # contexts before it could own a node to read parameters from, and
        # a node that has to exist before it can be configured is the wrong
        # shape for this.
        actions.append(
            Node(package="navi_sim_bringup", executable="sim_bridge.py",
                 name="sim_bridge", arguments=arguments, output="screen"))

    return actions


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
        DeclareLaunchArgument(
            "mode",
            default_value="simulation",
            description=(
                "'simulation' (default): the IK-driven, dead-reckoned rover, "
                "moved by planar_move - what has always run here. 'semi': the "
                "body pose comes from the rover's own /localization/pose, "
                "bridged in from the rover's domain, planar_move is not "
                "loaded, and the model is placed through "
                "/gazebo/set_entity_state.")),
        DeclareLaunchArgument(
            "sim_domain",
            default_value="42",
            description=(
                "The ROS domain this simulation is expected to be running on, "
                "for sim_bridge's sim-side context. Set ROS_DOMAIN_ID to the "
                "same value for the launch itself - start_sim.sh does both.")),
        DeclareLaunchArgument(
            "rover_domain",
            default_value="0",
            description=(
                "The ROS domain the rover's graph is on, for sim_bridge's "
                "rover-side context. 0 in the field; a throwaway domain when "
                "testing against mock/fake_localization.py.")),
        OpaqueFunction(function=_world_with_mesh),
    ])
