#!/usr/bin/env python3
"""Carries the rover's topics onto the simulation's ROS domain. One way.

Why two domains at all: the simulation publishes /clock at 100 Hz, a /tf
tree, /gazebo/* and its own /robot_description. On one shared domain those
land on the rover's graph, where a node that ever sets use_sim_time would
have its clock driven by a laptop's Gazebo and where two /robot_description
publishers would disagree. The 2026-08-28 design flagged that collision; the
simulation moving to its own domain closes it.

Why not domain_bridge: neither machine has it. Humble's rclpy takes a
domain_id per context, and several contexts live happily in one process.

Why one way, structurally: the sim-side node here has publishers and nothing
else - no subscriptions, no executor. There is no code path that could carry
a message back to the rover, so the guarantee does not depend on anybody
remembering it. The only spinning executor is the rover-side one.

Types are resolved by name at runtime because grid_map_msgs (which
/localization/map_tile needs) arrives with sub-project 3 and is not installed on
this laptop yet. A topic whose type will not import is skipped with a
warning: a bridge that refused to start would take semi-autonomous mode with
it, for a topic nothing is publishing yet anyway.

    ros2 run navi_sim_bringup sim_bridge.py --rover-domain 0 --sim-domain 42
"""

import argparse
import signal
import sys
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import ShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

#: Everything semi-autonomous mode needs out of the rover's graph.
#: /manual_twist so the IK still steers the wheels in the picture; the two
#: localisation topics so the model is placed and gated; map tiles so
#: terrain_writer has something to write. Nothing else, and nothing in the
#: other direction.
DEFAULT_TOPICS = [
    "/manual_twist:geometry_msgs/msg/Twist",
    "/localization/pose:nav_msgs/msg/Odometry",
    "/localization/status:std_msgs/msg/String",
    "/localization/map_tile:grid_map_msgs/msg/GridMap",
]

QUEUE_DEPTH = 10


def parse_topic_spec(spec: str) -> tuple[str, str]:
    """Splits "<topic>:<pkg>/msg/<Type>".

    rpartition, not split: a topic name never contains a colon but this way
    the type - which is the part with the fixed shape - is what anchors the
    split.
    """
    topic, separator, type_name = spec.rpartition(":")
    if not separator or not topic or not type_name:
        raise ValueError(
            f"bad topic spec {spec!r} - expected '<topic>:<pkg>/msg/<Type>', "
            "e.g. '/localization/pose:nav_msgs/msg/Odometry'")
    return topic, type_name


class SimBridge:
    """Two rclpy contexts in one process, and traffic in one direction."""

    def __init__(self, specs, rover_domain_id: int, sim_domain_id: int):
        if rover_domain_id == sim_domain_id:
            raise ValueError(
                f"the rover and simulation domains must differ (both are "
                f"{rover_domain_id}). On one domain there is nothing to bridge "
                "and the simulation's /clock lands on the rover's graph.")

        self.rover_context = Context()
        rclpy.init(context=self.rover_context, domain_id=rover_domain_id)
        self.sim_context = Context()
        rclpy.init(context=self.sim_context, domain_id=sim_domain_id)

        self.rover_node = Node("sim_bridge_rover_side", context=self.rover_context)
        self.sim_node = Node("sim_bridge_sim_side", context=self.sim_context)
        self.bridged: list[str] = []
        self.skipped: list[str] = []

        for spec in specs:
            topic, type_name = parse_topic_spec(spec)
            try:
                message_type = get_message(type_name)
            except Exception as exc:      # noqa: BLE001 - see below
                # Broad on purpose: get_message raises ModuleNotFoundError,
                # AttributeError or ValueError depending on which half of
                # the name is unresolvable, and every one of them means the
                # same thing here - this machine cannot carry this topic.
                self.rover_node.get_logger().warn(
                    f"not bridging {topic}: cannot resolve {type_name} on this "
                    f"machine ({type(exc).__name__}: {exc}). Install the package "
                    "that defines it and restart the simulation.")
                self.skipped.append(topic)
                continue
            publisher = self.sim_node.create_publisher(message_type, topic, QUEUE_DEPTH)
            self.rover_node.create_subscription(
                message_type, topic,
                # Bound as a default argument: a late-bound closure over the
                # loop variable would make every subscription publish on the
                # last topic's publisher.
                lambda message, publisher=publisher: publisher.publish(message),
                QUEUE_DEPTH)
            self.bridged.append(topic)

        # One executor, for the rover side only. The sim-side node is never
        # spun because it has nothing to spin: publishing does not need an
        # executor, and giving it one would be the first step towards it
        # having a subscription.
        self.executor = SingleThreadedExecutor(context=self.rover_context)
        self.executor.add_node(self.rover_node)

    def spin(self, stop: threading.Event = None) -> None:
        """Relays until `stop` is set or the bridge is shut down.

        Not executor.spin(): with two hand-made contexts, rclpy's own SIGINT
        handling never wakes that blocking wait - confirmed by sending the
        process SIGINT and watching it run on until ros2 launch escalated
        to SIGTERM five seconds later, every teardown. A timed spin_once
        returns to Python often enough for a signal handler to be seen.
        """
        try:
            while (stop is None or not stop.is_set()) and self.rover_context.ok():
                self.executor.spin_once(timeout_sec=0.2)
        except ShutdownException:
            # shutdown() was called from another thread, which is how the
            # tests end a spin; not an error.
            pass

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.rover_node.destroy_node()
        self.sim_node.destroy_node()
        rclpy.shutdown(context=self.rover_context)
        rclpy.shutdown(context=self.sim_context)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="One-way ROS domain bridge")
    parser.add_argument("--rover-domain", type=int, default=0,
                        help="the domain the rover's graph is on (default 0)")
    parser.add_argument("--sim-domain", type=int, default=42,
                        help="the domain the simulation runs on (default 42)")
    parser.add_argument("--topic", action="append", metavar="TOPIC:TYPE",
                        help=("a topic to carry, repeatable. Defaults to "
                              + ", ".join(DEFAULT_TOPICS)))
    return parser


def main(argv=None) -> int:
    # parse_known_args, because ros2 launch appends --ros-args to every node
    # it starts and argparse would reject them.
    args, _ = build_arg_parser().parse_known_args(
        sys.argv[1:] if argv is None else argv)

    bridge = SimBridge(args.topic or DEFAULT_TOPICS,
                       args.rover_domain, args.sim_domain)
    print(f"[sim_bridge] domain {args.rover_domain} -> domain {args.sim_domain}, "
          f"one way: {', '.join(bridge.bridged) or '(nothing)'}", flush=True)
    if bridge.skipped:
        print(f"[sim_bridge] not carried: {', '.join(bridge.skipped)}", flush=True)
    # Installed after SimBridge(), whose rclpy.init calls put rclpy's own
    # handlers in place; these replace them. SIGTERM too, so a launch
    # teardown ends with exit code 0 rather than a "process has died".
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    try:
        bridge.spin(stop)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
