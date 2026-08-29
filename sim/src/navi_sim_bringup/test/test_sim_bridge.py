"""Tests for the one-way domain bridge.

The bridge is a script, not a module in an installed Python package
(navi_sim_bringup is ament_cmake), so it is loaded by path.

Domains 91 and 92 are throwaways used by these tests only. Domain 0 is the
rover's and 42 is the simulation's default; neither appears here, and
neither does /manual_twist - the topic that drives the physical rover.
"""

import importlib.util
import pathlib
import threading
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

_PATH = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "sim_bridge.py"
_spec = importlib.util.spec_from_file_location("sim_bridge", _PATH)
sim_bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim_bridge)

ROVER_DOMAIN = 91
SIM_DOMAIN = 92
TOPIC = "/bridge_test"
SPEC = f"{TOPIC}:std_msgs/msg/String"


def test_parse_topic_spec_splits_the_topic_from_the_type():
    assert sim_bridge.parse_topic_spec("/localization/pose:nav_msgs/msg/Odometry") == (
        "/localization/pose", "nav_msgs/msg/Odometry")


def test_parse_topic_spec_refuses_something_that_is_not_a_spec():
    with pytest.raises(ValueError):
        sim_bridge.parse_topic_spec("/localization/pose")
    with pytest.raises(ValueError):
        sim_bridge.parse_topic_spec(":nav_msgs/msg/Odometry")


def test_a_topic_whose_type_is_not_installed_is_skipped_not_fatal():
    # grid_map_msgs arrives with sub-project 3. Until then the bridge has to
    # come up and carry everything else, or semi-autonomous mode cannot be
    # run at all on a laptop that does not have it yet.
    bridge = sim_bridge.SimBridge(
        [SPEC, "/localization/map:definitely_not_a_package/msg/Nothing"],
        ROVER_DOMAIN, SIM_DOMAIN)
    try:
        assert bridge.bridged == [TOPIC]
        assert bridge.skipped == ["/localization/map"]
    finally:
        bridge.shutdown()


def test_the_two_domains_must_differ():
    with pytest.raises(ValueError):
        sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, ROVER_DOMAIN)


def _node_on(domain_id, name):
    """A node on its own context and domain, plus that context, so the
    caller can shut it down. Each test builds its own: a context is not
    reusable after shutdown."""
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    return Node(name, context=context), context


def _spin_until(node, context, predicate, seconds=5.0):
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if predicate():
            break
    executor.remove_node(node)
    return predicate()


def test_a_message_on_the_rover_domain_appears_on_the_sim_domain():
    bridge = sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, SIM_DOMAIN)
    thread = threading.Thread(target=bridge.spin, daemon=True)
    thread.start()

    publisher_node, publisher_context = _node_on(ROVER_DOMAIN, "bridge_test_publisher")
    listener_node, listener_context = _node_on(SIM_DOMAIN, "bridge_test_listener")
    received = []
    listener_node.create_subscription(String, TOPIC, received.append, 10)
    publisher = publisher_node.create_publisher(String, TOPIC, 10)

    try:
        # Discovery is not instant and the bridge's subscription has to find
        # the publisher before anything is carried, so this publishes
        # repeatedly rather than once and waits for the first arrival.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not received:
            publisher.publish(String(data="hello"))
            _spin_until(listener_node, listener_context, lambda: bool(received), seconds=0.5)

        assert received, "nothing crossed from the rover domain to the sim domain"
        assert received[0].data == "hello"
    finally:
        publisher_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown(context=publisher_context)
        rclpy.shutdown(context=listener_context)
        bridge.shutdown()


def test_nothing_crosses_back_from_the_sim_domain():
    # This is the decision the whole two-domain design exists for: /clock,
    # /tf and the sim's /robot_description must never reach the rover's
    # graph. Asserting it with the bridge's own topic is the strongest form
    # of the check - if even the topic it is wired for does not come back,
    # nothing does.
    bridge = sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, SIM_DOMAIN)
    thread = threading.Thread(target=bridge.spin, daemon=True)
    thread.start()

    publisher_node, publisher_context = _node_on(SIM_DOMAIN, "backflow_publisher")
    listener_node, listener_context = _node_on(ROVER_DOMAIN, "backflow_listener")
    received = []
    listener_node.create_subscription(String, TOPIC, received.append, 10)
    publisher = publisher_node.create_publisher(String, TOPIC, 10)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            publisher.publish(String(data="should not cross"))
            _spin_until(listener_node, listener_context, lambda: bool(received), seconds=0.2)

        assert received == [], (
            "a message published on the simulation's domain reached the rover's")
    finally:
        publisher_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown(context=publisher_context)
        rclpy.shutdown(context=listener_context)
        bridge.shutdown()


def test_the_sim_side_node_has_no_subscriptions_at_all():
    # The structural half of the test above: the reverse direction is not
    # forbidden by a rule, it is absent from the object graph.
    bridge = sim_bridge.SimBridge([SPEC], ROVER_DOMAIN, SIM_DOMAIN)
    try:
        # Asked of the nodes themselves, not of the graph:
        # get_subscriptions_info_by_topic reports every subscription anywhere,
        # including the bridge's own rover-side one on the other domain.
        assert len(list(bridge.sim_node.subscriptions)) == 0
        # Every rclpy node owns /rosout and /parameter_events publishers; that
        # is node infrastructure, not a bridged topic, and stays on its own domain.
        assert [p.topic_name for p in bridge.rover_node.publishers
                if p.topic_name not in ("/rosout", "/parameter_events")] == []
    finally:
        bridge.shutdown()


def test_the_built_map_is_among_the_topics_the_bridge_carries_by_default():
    # Sub-project 3: terrain_writer on the simulation's domain reads the
    # elevation map the rover publishes on its own. If this entry went
    # missing the terrain would silently never appear.
    assert "/localization/map:grid_map_msgs/msg/GridMap" in sim_bridge.DEFAULT_TOPICS
