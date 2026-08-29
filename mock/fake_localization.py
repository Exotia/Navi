#!/usr/bin/env python3
"""The same square as mock/ros_bridge.py, on a real ROS domain.

The mock rosbridge is a websocket server and puts nothing on a DDS graph, so
it can drive the ground station and nothing else. sim_bridge and sim_ik_node
read DDS, so checking that Gazebo's rover walks the square needs a real
publisher - this one. It imports the same square_walk module the mock does,
which is what makes "the same square" a fact rather than a hope.

Run it on a throwaway domain, never on the rover's:

    bash -c 'source /opt/ros/humble/setup.bash && \\
             ROS_DOMAIN_ID=91 python3 mock/fake_localization.py'

This is a test fixture, not a node anybody ships, which is why it lives in
mock/ beside the mock rosbridge rather than in a ROS package.
"""

import argparse
import importlib.util
import math
import os
import pathlib
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

_SQUARE_WALK = pathlib.Path(__file__).resolve().parent / "square_walk.py"
_spec = importlib.util.spec_from_file_location("square_walk", _SQUARE_WALK)
square_walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(square_walk)

POSE_HZ = 10.0
STATUS_HZ = 2.0


class FakeLocalization(Node):
    def __init__(self, state: str, side: float, speed: float):
        super().__init__("fake_localization")
        self.state = state
        self.side = side
        self.speed = speed
        self.started = time.monotonic()
        self.frozen_at = None
        self.pose_publisher = self.create_publisher(Odometry, "/localization/pose", 10)
        self.status_publisher = self.create_publisher(String, "/localization/status", 10)
        self.create_timer(1.0 / POSE_HZ, self.publish_pose)
        self.create_timer(1.0 / STATUS_HZ, self.publish_status)
        self.get_logger().info(
            f"fake localisation: a {side} m square at {speed} m/s, state {state}, "
            f"on domain {os.environ.get('ROS_DOMAIN_ID', '0')}")

    def refuse_if_the_rover_is_already_publishing(self) -> bool:
        """Two publishers on /localization/pose is the one way this fixture
        can do harm: whoever is downstream would see the fake square and the
        real rover interleaved and have no way to tell. Costs one graph
        query at startup."""
        # Discovery is asynchronous: counted straight after construction
        # the graph only ever contained this node, and the guard never
        # fired. A second is what DDS needs to hear about a peer.
        time.sleep(1.0)
        if self.count_publishers("/localization/pose") > 1:
            self.get_logger().error(
                "something else is already publishing /localization/pose on this "
                "domain - refusing to start. Run this on a throwaway domain "
                "(ROS_DOMAIN_ID=91), never on the rover's.")
            return True
        return False

    def publish_pose(self):
        if self.state == "OFF":
            return
        if self.state == "SEARCHING":
            if self.frozen_at is None:
                self.frozen_at = time.monotonic() - self.started
            elapsed = self.frozen_at
        else:
            elapsed = time.monotonic() - self.started
        x, y, yaw = square_walk.square_pose(elapsed, self.side, self.speed)
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = "base_footprint"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_publisher.publish(message)

    def publish_status(self):
        elapsed = time.monotonic() - self.started
        if self.state == "OK":
            seconds_since_ok, distance = 0.0, self.speed * elapsed
        elif self.state == "SEARCHING":
            seconds_since_ok, distance = elapsed, 0.0
        else:
            seconds_since_ok, distance = None, 0.0
        message = String()
        message.data = square_walk.status_payload(self.state, seconds_since_ok, distance)
        self.status_publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", choices=["OK", "SEARCHING", "OFF"], default="OK")
    parser.add_argument("--side", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=0.5)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = FakeLocalization(args.state, args.side, args.speed)
    if node.refuse_if_the_rover_is_already_publishing():
        node.destroy_node()
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
