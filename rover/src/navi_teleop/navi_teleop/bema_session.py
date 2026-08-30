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
        self._down_since = None
        self._backoff_index = 0
        self._retry_at = None

    # --- connection lifecycle -------------------------------------------
    def connect(self):
        try:
            self._bema = self._factory(self._host, self._bema_port)
            self._coord = self._factory(self._host, self._coord_port)
            self._lease = bool(self._bema.call("__sam__request"))
            self._last_error = None
            self._down_since = None
            self._backoff_index = 0
            self._retry_at = None
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def _mark_down(self, exc):
        self._last_error = str(exc)
        self._lease = False
        if self._down_since is None:
            self._down_since = self._clock()
        delay = _BACKOFF[min(self._backoff_index, len(_BACKOFF) - 1)]
        self._retry_at = self._clock() + delay
        self._backoff_index += 1
        for client in (self._bema, self._coord):
            if client is not None:
                client.close()
        self._bema = self._coord = None

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
                self._last_ping = now
            if self._last_hb is None or now - self._last_hb >= HEARTBEAT_INTERVAL_S:
                self._coord.call("F10")               # notifyConnected
                self._coord_state = self._coord.call("F9")   # getState
                self._last_hb = now
            vx, vy, w = self._command
            self._bema.call("F1", vx, vy, w)
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    # --- commands --------------------------------------------------------
    def set_command(self, vx, vy, w_deg):
        self._command = (float(vx), float(vy), float(w_deg))

    def stop(self):
        self._command = (0.0, 0.0, 0.0)
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

    def start_manual(self):
        self._safe_coord("F6")

    def _safe(self, method, *args):
        if self._bema is None:
            return
        try:
            self._bema.call(method, *args)
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def _safe_coord(self, method, *args):
        if self._coord is None:
            return
        try:
            self._coord.call(method, *args)
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def close(self):
        if self._bema is not None:
            self.stop()
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
