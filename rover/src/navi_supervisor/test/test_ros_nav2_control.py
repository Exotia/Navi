"""RosNav2Control against fake services, on a throwaway domain.

The contract this pins is the one nav2_control.py's docstring names: the
sequence the supervisor asks for must not change now that the stub has
stopped being a stub.  So the test drives the supervisor the way a takeover
does and watches what arrives on the wire.
"""

import os

# 93, not 91: test_mode_supervisor.py already owns 91, and two agents in
# this tree at once would otherwise cross-talk.  setdefault, so the suite
# command below - which exports one domain for the whole pytest process,
# because rclpy reads ROS_DOMAIN_ID once per process - still wins.
os.environ.setdefault("ROS_DOMAIN_ID", "93")   # throwaway; never the rover's

import json
import time

import pytest
import rclpy
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from navi_supervisor.mode_supervisor import ModeSupervisor
from navi_supervisor.nav2_control import Nav2Control
from navi_supervisor.ros_nav2_control import (CANCEL_SERVICE, MANAGE_SERVICE,
                                              RosNav2Control)


class FakeNav2(Node):
    """The two services Nav2 exposes, and nothing else."""

    def __init__(self):
        super().__init__('fake_nav2')
        self.cancels = []
        self.commands = []
        self.create_service(CancelGoal, CANCEL_SERVICE, self._on_cancel)
        self.create_service(ManageLifecycleNodes, MANAGE_SERVICE, self._on_manage)

    def _on_cancel(self, request, response):
        self.cancels.append(request)
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    def _on_manage(self, request, response):
        self.commands.append(request.command)
        response.success = True
        return response


@pytest.fixture
def graph():
    rclpy.init()
    fake = FakeNav2()
    supervisor = ModeSupervisor()
    supervisor.attach_nav2_control(RosNav2Control(supervisor))
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(supervisor)
    yield fake, supervisor, executor
    executor.shutdown()
    fake.destroy_node()
    supervisor.destroy_node()
    rclpy.shutdown()


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


def test_it_is_the_interface_sp5_wrote_and_not_a_new_one():
    assert issubclass(RosNav2Control, Nav2Control)
    assert RosNav2Control.cancel_goal is not Nav2Control.cancel_goal
    assert RosNav2Control.deactivate is not Nav2Control.deactivate


def request_mode(supervisor, mode):
    message = String()
    message.data = json.dumps({"mode": mode})
    supervisor._on_mode_request(message)


def test_a_takeover_cancels_every_goal_and_pauses_the_stack(graph):
    """The supervisor starts in manual, and supervisor_state only queues
    the Nav2 actions when it was autonomous, so the takeover has to be a
    real takeover: into autonomous first, then back out."""
    fake, supervisor, executor = graph
    spin(executor, 2.0)          # service discovery

    request_mode(supervisor, "autonomous")
    spin(executor, 0.5)
    assert not fake.cancels, "entering autonomous must not cancel anything"

    request_mode(supervisor, "manual")
    spin(executor, 2.0)

    assert len(fake.cancels) == 1, "exactly one cancel per takeover"
    assert len(fake.commands) == 1
    assert fake.commands[0] == ManageLifecycleNodes.Request.PAUSE


def test_the_cancel_asks_for_every_goal_not_one(graph):
    """Zero uuid and zero stamp: action_msgs/srv/CancelGoal's 'cancel all
    goals'.  The supervisor never sent the goal, so it has no handle - and
    on a takeover it wants all of them gone regardless."""
    fake, supervisor, executor = graph
    spin(executor, 2.0)
    supervisor._nav2.cancel_goal("test")
    spin(executor, 1.5)

    goal_info = fake.cancels[0].goal_info
    assert list(goal_info.goal_id.uuid) == [0] * 16
    assert goal_info.stamp.sec == 0 and goal_info.stamp.nanosec == 0


def test_it_never_blocks_when_nav2_is_not_there(graph):
    """No Nav2 on the graph is the normal state for most of a session, and
    the one moment this is called is the moment the rover must stop."""
    _, supervisor, _ = graph
    orphan = RosNav2Control(supervisor,
                            cancel_service='/nowhere/_action/cancel_goal',
                            manage_service='/nowhere/manage_nodes')
    started = time.monotonic()
    orphan.cancel_goal("no nav2")
    orphan.deactivate("no nav2")
    assert time.monotonic() - started < 0.5


def test_repeated_calls_do_not_leak_futures(graph):
    """100 sends without spinning the executor: nothing is done(), so the
    only thing holding the list down is the truncation in _reap() - which
    runs *after* every append, so the bound is MAX_PENDING exactly, not
    MAX_PENDING + 1."""
    _, supervisor, executor = graph
    spin(executor, 2.0)
    for _ in range(50):
        supervisor._nav2.cancel_goal("spam")
        supervisor._nav2.deactivate("spam")
    spin(executor, 2.0)
    assert len(supervisor._nav2._pending) <= RosNav2Control.MAX_PENDING
