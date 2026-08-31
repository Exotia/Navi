"""The real Nav2Control: cancel every goal, pause the stack, never wait.

SP5 wrote the decision (supervisor_state.py queues CANCEL_GOAL and
DEACTIVATE_NAV2); this is the half that reaches Nav2.

Two services, not an action client and not a lifecycle client per node:

  * The supervisor did not send the goal - SP11's goal_relay does - so it
    holds no goal handle.  action_msgs/srv/CancelGoal says a zero goal id
    with a zero stamp cancels *all* goals, which is what a takeover wants
    anyway.
  * nav2_msgs/srv/ManageLifecycleNodes with PAUSE deactivates all six nodes
    through the one manager that owns them, in the order it knows to be
    safe.  RESET would tear down the configuration and need a full
    reconfigure; SHUTDOWN would end the processes.  PAUSE is reversible
    with RESUME, which is what starting the next run needs.

Nothing here waits.  The supervisor has already published a zero twist by
the time it calls either method, and a hung Nav2 is exactly the case where
waiting would keep the rover moving.  If the service is not there, that is
a log line, not an exception - Nav2 is absent for most of a manual session.
"""

from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes

from navi_supervisor.nav2_control import Nav2Control

CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'
MANAGE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'


class RosNav2Control(Nav2Control):
    """The interface nav2_control.py declares, wired to a running Nav2."""

    MAX_PENDING = 16

    def __init__(self, node, cancel_service=CANCEL_SERVICE,
                 manage_service=MANAGE_SERVICE):
        self._node = node
        self._logger = node.get_logger()
        self._cancel_client = node.create_client(CancelGoal, cancel_service)
        self._manage_client = node.create_client(ManageLifecycleNodes,
                                                 manage_service)
        self._pending = []

    # -- the interface -----------------------------------------------------

    def cancel_goal(self, reason: str) -> None:
        # Default-constructed goal_info: a zero uuid and a zero stamp, which
        # action_msgs/srv/CancelGoal defines as "cancel all goals".
        self._send(self._cancel_client, CancelGoal.Request(),
                   'cancel every Nav2 goal', reason)

    def deactivate(self, reason: str) -> None:
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        self._send(self._manage_client, request, 'pause the Nav2 stack', reason)

    # -- the one way either of them reaches the graph ----------------------

    def _send(self, client, request, what: str, reason: str) -> None:
        if not client.service_is_ready():
            # Not an error: Nav2 is not running for most of a session, and
            # the supervisor has already stopped the rover by now.
            self._logger.info(
                f"{what} ({reason}): {client.srv_name} is not there; nothing to do")
            return
        try:
            self._pending.append(client.call_async(request))
        except Exception as exc:
            self._logger.error(f"{what} ({reason}) failed to send: {exc!r}")
            return
        # Reap AFTER the append, never before: _reap() truncates only when
        # the list already exceeds MAX_PENDING, so reaping first would leave
        # a steady state of MAX_PENDING + 1 and the bound the docstring
        # promises would be off by one.
        self._reap()
        self._logger.info(f"{what} ({reason}): asked")

    def _reap(self) -> None:
        """Drop finished futures, and never grow without bound: this is
        called at the end of every _send(), and a Nav2 that never answers
        must not turn a takeover into a memory leak.  On return,
        len(self._pending) <= MAX_PENDING - that is the invariant
        test_repeated_calls_do_not_leak_futures asserts."""
        self._pending = [f for f in self._pending if not f.done()]
        if len(self._pending) > self.MAX_PENDING:
            for stale in self._pending[:-self.MAX_PENDING]:
                stale.cancel()
            self._pending = self._pending[-self.MAX_PENDING:]
