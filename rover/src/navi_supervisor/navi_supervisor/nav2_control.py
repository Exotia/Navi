"""What the supervisor is allowed to do to Nav2, and the stub that stands
in until Nav2 exists.

SP9 brings Nav2 up and replaces NullNav2Control with an implementation
that cancels the running NavigateToPose goal and deactivates the
navigation lifecycle. Until then the supervisor still decides when both
must happen, and still records that it asked: the decision is the
safety-critical half, and it is finished and tested now rather than
arriving late with Nav2.

The interface is deliberately two methods that return nothing. The
supervisor must never wait on Nav2 in order to stop the rover - it has
already published a zero twist by the time either of these is called.
"""


class Nav2Control:
    """The interface SP9 implements. Do not change the two names."""

    def cancel_goal(self, reason: str) -> None:
        raise NotImplementedError

    def deactivate(self, reason: str) -> None:
        raise NotImplementedError


class NullNav2Control(Nav2Control):
    """No Nav2 on the graph yet: log the request and record it.

    `calls` is the list of (method, reason) pairs, in order. It is what the
    tests assert on, and it is the contract SP9's real implementation is
    checked against: the sequence the supervisor asks for must not change
    when the stub stops being a stub.
    """

    def __init__(self, logger=None):
        self._logger = logger
        self.calls = []

    def cancel_goal(self, reason: str) -> None:
        self.calls.append(("cancel_goal", reason))
        if self._logger is not None:
            self._logger.info(f"nav2 goal cancel requested ({reason}); no Nav2 yet")

    def deactivate(self, reason: str) -> None:
        self.calls.append(("deactivate", reason))
        if self._logger is not None:
            self._logger.info(f"nav2 deactivate requested ({reason}); no Nav2 yet")
