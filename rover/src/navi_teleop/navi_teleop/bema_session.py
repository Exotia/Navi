"""The BEMA drive session: an rpclib client to the primary's drive server
(:21022) and coordinator (:21031), with the lease dance, the coordinator
heartbeat, and reconnect-with-backoff. No ROS here - the node in
bema_bridge.py owns the timers and calls tick(); this class owns the
protocol, so it can be tested against fake_bema_server with a fake clock.
"""

from navi_teleop.msgpack_rpc import (RpcClient, RpcError, RpcTimeout,
                                     RpcDisconnected)

PING_INTERVAL_S = 0.5
HEARTBEAT_INTERVAL_S = 1.0
_BACKOFF = [1.0, 2.0, 4.0, 5.0]


class BemaSession:
    def __init__(self, host, bema_port, coordinator_port, clock,
                 client_factory=RpcClient):
        self._host = host
        self._bema_port = bema_port
        self._coord_port = coordinator_port
        self._clock = clock
        self._factory = client_factory
        self._bema = None
        self._coord = None
        self._lease = False
        self._command = (0.0, 0.0, 0.0)
        self._last_ping = None
        self._last_hb = None
        self._last_error = None
        self._coord_state = None
        self._movement_sent = False
        self._down_since = None
        self._backoff_index = 0
        self._retry_at = None

    # --- connection lifecycle -------------------------------------------
    def connect(self):
        try:
            self._bema = self._factory(self._host, self._bema_port, timeout_s=0.3)
            self._coord = self._factory(self._host, self._coord_port, timeout_s=0.3)
            self._lease = bool(self._bema.call("__sam__request"))
            # Safety rule 1: reconnect must start with a stop. The link can
            # drop mid-drive with the gamepad still streaming a nonzero
            # twist; by the time the lease is back, the retained _command
            # would still be that stale value, and resuming it unannounced
            # on the fresh connection is exactly the runaway this bridge
            # exists to prevent. So the very first thing sent on the
            # just-opened bema client is a stop, and the retained command is
            # cleared so nothing revives it until set_command runs again. A
            # failure here falls through to the same except below, so it is
            # a _mark_down like any other connect() failure, not a raise.
            self._command = (0.0, 0.0, 0.0)
            self._bema.call("F1", 0.0, 0.0, 0.0)
            self._bema.call("F2")
            self._last_error = None
            self._down_since = None
            self._backoff_index = 0
            self._retry_at = None
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def _mark_down(self, exc):
        self._last_error = str(exc)
        self._lease = False
        self._movement_sent = False
        if self._down_since is None:
            self._down_since = self._clock()
        delay = _BACKOFF[min(self._backoff_index, len(_BACKOFF) - 1)]
        self._retry_at = self._clock() + delay
        self._backoff_index += 1
        for client in (self._bema, self._coord):
            if client is not None:
                client.close()
        self._bema = self._coord = None

    def _mark_refused(self, exc, context):
        # An RpcError is an application-level refusal (rpclib's checkAccess
        # answering error 1) on an otherwise healthy link - typically
        # because the coordinator force-took the BEMA lease to disable
        # movement. Unlike a dead link, this must not close either socket,
        # drop the F10 heartbeat, or back off: the next tick's own
        # re-request is what recovers it, usually within a second.
        self._last_error = f"refused: {exc.error!r} ({context})"
        self._lease = False
        self._movement_sent = False

    # --- per-tick work ---------------------------------------------------
    def tick(self, now):
        if self._bema is None:
            if self._retry_at is not None and now >= self._retry_at:
                self.connect()
            return
        try:
            if not self._lease:
                self._lease = bool(self._bema.call("__sam__request"))
            if self._last_ping is None or now - self._last_ping >= PING_INTERVAL_S:
                if not self._bema.call("__sam__ping"):
                    self._lease = bool(self._bema.call("__sam__request"))
                    # A lost lease usually means the coordinator force-took
                    # it to disable movement; re-enable once it is ours again.
                    self._movement_sent = False
                self._last_ping = now
            if self._last_hb is None or now - self._last_hb >= HEARTBEAT_INTERVAL_S:
                self._coord.call("F10")               # notifyConnected
                state = self._coord.call("F9")        # getState
                # Kept as the int enum it should be; anything else (the
                # primary sent something odd, or a test double stands in
                # for it) is stringified rather than stored raw, since
                # status() below round-trips this through json.dumps and an
                # unserialisable value there would blackout all of
                # /drive_status, not just this one field.
                if isinstance(state, int) and not isinstance(state, bool):
                    self._coord_state = state
                else:
                    self._coord_state = str(state)
                self._last_hb = now
            # BemaServer::drive() is gated on m_movementEnabled, which only
            # the coordinator's movementUpdateLoop would set - and its polite
            # lease request (ServerAccessManager::request) is refused for as
            # long as this session holds the exclusive lease, so its
            # setMovementEnabled(true) can never land. Send it ourselves,
            # once per Manual entry: we hold the lease, so F7 passes
            # checkAccess. Disabling stays the coordinator's job - its
            # forceCapability path still wins whenever the state machine
            # leaves Manual.
            if self._coord_state != 3:                   # 3 = Manual
                self._movement_sent = False
            elif self._lease and not self._movement_sent:
                self._bema.call("F7", True)
                self._movement_sent = True
            vx, vy, w = self._command
            self._bema.call("F1", vx, vy, w)
        except RpcError as exc:
            self._mark_refused(exc, "tick")
        except (RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    # --- commands --------------------------------------------------------
    def set_command(self, vx, vy, w_deg):
        self._command = (float(vx), float(vy), float(w_deg))

    def stop(self):
        self._command = (0.0, 0.0, 0.0)
        # If the F1 zero-send itself marks the link down, _safe() nulls
        # _bema, and the F2 below silently no-ops - stop() degrades to "the
        # command is cleared" instead of actually reaching the primary.
        # Accepted: the recovery is connect()'s own stop-first behaviour
        # (see the safety-rule-1 comment there), which re-sends F1 zero then
        # F2 on the very next successful reconnect.
        self._safe("F1", 0.0, 0.0, 0.0)
        self._safe("F2")

    def init(self):
        self._safe("F0")

    def reset_encoders(self):
        self._safe("F4")

    def reset_odometry(self):
        self._safe("F3")

    def change_drive_mode(self):
        self._safe("F5")

    def change_drive_state(self):
        self._safe("F6")

    def _coord_guarded(self, method, *args, label=None):
        # CoordinatorProxy binds F0-F7 behind checkAccess() on the
        # coordinator's OWN ServerAccessManager - a separate lease from
        # BEMA's. Acquire it just-in-time; it lapses by itself after 4 s of
        # guarded-call silence, and the unguarded F8/F9/F10 traffic neither
        # needs nor refreshes it.
        if self._coord is None:
            return
        try:
            if not self._coord.call("__sam__request"):
                self._last_error = ("coordinator refused the lease for "
                                    f"{label or method}")
                return
            self._coord.call(method, *args)
        except RpcError as exc:
            self._mark_refused(exc, label or method)
        except (RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def start_manual(self):
        self._coord_guarded("F6", label="start_manual")

    def abort(self):
        # F7 on the COORDINATOR is abort - not BEMA's F7, which is
        # setMovementEnabled(bool) on a different server on a different port.
        self._coord_guarded("F7", label="abort")

    def start_navi_task(self, waypoints):
        # CoordinatorProxy binds F0 (startNaViTask) behind checkAccess() on
        # the coordinator's own ServerAccessManager - same just-in-time lease
        # as start_manual() and abort(). waypoints is a list of (x, y, yaw)
        # floats; the coordinator relays them straight back to us as NaVi F3.
        self._coord_guarded("F0", [[float(x), float(y), float(w)]
                                   for x, y, w in waypoints])

    def pause_task(self):
        self._coord_guarded("F4")

    def resume_task(self):
        self._coord_guarded("F5")

    def notify_task_finished(self, tag):
        # CoordinatorProxy binds F8 (notifyTaskFinished) with no
        # checkAccess() - it is @noguard in Coordinator.idl - so this needs
        # no lease and cannot fight anything for one. That is exactly why
        # navi_rpc_server reports progress through this session instead of
        # opening a second client to the primary.
        self._safe_coord("F8", int(tag))

    def _safe(self, method, *args):
        if self._bema is None:
            return
        try:
            self._bema.call(method, *args)
        except RpcError as exc:
            self._mark_refused(exc, method)
        except (RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def _safe_coord(self, method, *args):
        if self._coord is None:
            return
        try:
            self._coord.call(method, *args)
        except RpcError as exc:
            self._mark_refused(exc, method)
        except (RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def close(self):
        if self._bema is not None:
            self.stop()
            if self._bema is not None:
                try:
                    self._bema.call("__sam__release")
                except (RpcError, RpcTimeout, RpcDisconnected, OSError):
                    pass
        for client in (self._bema, self._coord):
            if client is not None:
                client.close()
        self._bema = self._coord = None
        self._lease = False

    def status(self):
        now = self._clock()
        reconnect_in = None
        if self._retry_at is not None and self._bema is None:
            reconnect_in = max(0.0, self._retry_at - now)
        return {
            "connected": self._bema is not None,
            "lease": self._lease,
            "coordinator_state": self._coord_state,
            "last_error": self._last_error,
            "reconnect_in_s": reconnect_in,
        }
