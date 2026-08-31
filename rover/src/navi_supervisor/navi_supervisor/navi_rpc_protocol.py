"""The rpclib wire, served rather than called, and the lease that guards it.

The primary's coordinator talks to us with rpclib: a request is
[0, msgid, method, [args]], a response [1, msgid, error, result] with error
nil on success, and a notification [2, method, [args]] gets no response at
all. `navi_teleop/msgpack_rpc.py` is the client half of the same wire; this
is the server half, and it is deliberately the same dependency-light shape
(just `msgpack` and `socket`), with no ROS in it, so the whole contract can
be tested against a fake coordinator with no executor running.

The lease is the part that is easy to miss and fatal to omit.
`AutoConnection<navi::NaViEP>::getCapability()` calls `__sam__request` first
and returns nullptr if it is not answered `true` - and nullptr is the branch
that logs "Cannot start navigation task: NaVi not reachable". A server that
binds :21021 and serves F0-F9 but not `__sam__` is, from the coordinator's
side, not there at all.

Two deliberate differences from `starpc::ServerAccessManager`:

  * the lease is released when its connection closes. rpclib leaks it (its
    own TODO says so), which would mean that after a coordinator restart the
    new session's request is refused forever and the rover is permanently
    unreachable.
  * expiry is uniform. rpclib arms its watchdog only in `request`, so a
    forced lease there never times out by itself.
"""

import socket
import threading
from time import monotonic

import msgpack

REQUEST_TYPE = 0
RESPONSE_TYPE = 1
NOTIFY_TYPE = 2

# starpc::ServerAccessManager: rwd.setup(4s), refreshed by __sam__ping and by
# notifyClientActivity() after every successful guarded call.
LEASE_TIMEOUT_S = 4.0

# ServerProxy::checkAccess() answers a non-holder with respond_error(1) - the
# integer, not a string. Unimplemented methods answer with a string instead,
# so the two are told apart in a capture.
ACCESS_DENIED_ERROR = 1

SAM_METHODS = ("__sam__request", "__sam__force", "__sam__ping",
               "__sam__release")

_ACCEPT_TIMEOUT_S = 0.2
_RECV_TIMEOUT_S = 0.2
_RECV_BYTES = 4096
# A waypoint list is a few hundred bytes. Anything approaching this is not a
# coordinator, and unpacking it would be the whole attack.
_MAX_BUFFER_BYTES = 1024 * 1024


class RpcRefusal(Exception):
    """A method that exists but declines to act. `error` goes on the wire."""

    def __init__(self, error):
        super().__init__(str(error))
        self.error = error


class Lease:
    """`starpc::ServerAccessManager`, with the two differences above.

    It holds no clock: every method takes `now` from its caller, which is
    what lets the whole thing be driven by a fake clock in the tests.
    """

    def __init__(self, timeout_s=LEASE_TIMEOUT_S):
        self._timeout_s = float(timeout_s)
        self._lock = threading.RLock()
        self._holder = None
        self._deadline = None

    def _expire(self, now):
        if (self._holder is not None and self._deadline is not None
                and now >= self._deadline):
            self._holder = None
            self._deadline = None

    def request(self, session, now):
        with self._lock:
            self._expire(now)
            if self._holder is not None and self._holder != session:
                return False
            self._holder = session
            self._deadline = now + self._timeout_s
            return True

    def force(self, session, now):
        with self._lock:
            self._holder = session
            self._deadline = now + self._timeout_s

    def ping(self, session, now):
        with self._lock:
            self._expire(now)
            if self._holder != session:
                return False
            self._deadline = now + self._timeout_s
            return True

    def release(self, session, now):
        with self._lock:
            self._expire(now)
            if self._holder != session:
                return False
            self._holder = None
            self._deadline = None
            return True

    def holds(self, session, now):
        with self._lock:
            self._expire(now)
            return self._holder == session

    def touch(self, now):
        """ServerAccessManager::notifyClientActivity()."""
        with self._lock:
            if self._holder is not None:
                self._deadline = now + self._timeout_s

    def drop(self, session):
        with self._lock:
            if self._holder == session:
                self._holder = None
                self._deadline = None

    def holder(self, now):
        with self._lock:
            self._expire(now)
            return self._holder


class RpcServer:
    """One accept thread, one thread per connection, one shared method table.

    `methods` maps a name to a plain callable; `guarded` names the subset
    that needs the lease. The `__sam__` methods are handled here rather than
    in the table, because they are the only ones that need the session id.
    """

    def __init__(self, methods, guarded=(), host="0.0.0.0", port=21021,
                 clock=monotonic, logger=None, backlog=8, lease=None):
        self._methods = dict(methods)
        self._guarded = frozenset(guarded)
        self._clock = clock
        self._logger = logger
        self.lease = lease if lease is not None else Lease()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, int(port)))
        self._sock.listen(backlog)
        self._sock.settimeout(_ACCEPT_TIMEOUT_S)
        self.host, self.port = self._sock.getsockname()
        self._stop = False
        self._sessions = 0
        self._sessions_lock = threading.Lock()
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)

    # --- lifecycle -------------------------------------------------------
    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        try:
            self._sock.close()
        except OSError:
            pass

    def _log(self, level, message):
        if self._logger is None:
            return
        try:
            getattr(self._logger, level)(message)
        except Exception:                              # a logger must not kill us
            pass

    def _accept_loop(self):
        while not self._stop:
            try:
                conn, peer = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._sessions_lock:
                self._sessions += 1
                session = self._sessions
            conn.settimeout(_RECV_TIMEOUT_S)
            threading.Thread(target=self._serve, args=(conn, session, peer),
                             daemon=True).start()

    # --- one connection --------------------------------------------------
    def _serve(self, conn, session, peer):
        unpacker = msgpack.Unpacker(raw=False, max_buffer_size=_MAX_BUFFER_BYTES)
        try:
            while not self._stop:
                try:
                    data = conn.recv(_RECV_BYTES)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                try:
                    unpacker.feed(data)
                except Exception as exc:               # buffer limit exceeded
                    self._log("warn", f"session {session} from {peer}: "
                                      f"oversized stream ({exc!r}); closing")
                    break
                if not self._drain(conn, session, peer, unpacker):
                    break
        finally:
            # Never leak the lease onto a socket nobody holds any more.
            self.lease.drop(session)
            try:
                conn.close()
            except OSError:
                pass

    def _drain(self, conn, session, peer, unpacker):
        """Answer every complete frame in the buffer. False = hang up."""
        while True:
            try:
                message = next(unpacker)
            except StopIteration:
                return True
            except Exception as exc:                   # not msgpack at all
                self._log("warn", f"session {session} from {peer}: "
                                  f"undecodable msgpack ({exc!r}); closing")
                return False
            frame = self._parse(message)
            if frame is None:
                self._log("warn", f"session {session} from {peer}: "
                                  f"not an rpc frame ({message!r}); closing")
                return False
            msgid, method, args, wants_reply = frame
            error, result = self._dispatch(session, method, args)
            if not wants_reply:
                continue
            try:
                conn.sendall(msgpack.packb([RESPONSE_TYPE, msgid, error, result],
                                           use_bin_type=True))
            except OSError:
                return False

    @staticmethod
    def _parse(message):
        """(msgid, method, args, wants_reply), or None if it is not a frame.

        A desynchronised stream cannot be recovered - the next bytes are the
        middle of something - so a frame that is not a request or a
        notification costs that one connection, and nothing else.
        """
        if not isinstance(message, (list, tuple)):
            return None
        if len(message) == 4 and message[0] == REQUEST_TYPE:
            _type, msgid, method, args = message
            wants_reply = True
        elif len(message) == 3 and message[0] == NOTIFY_TYPE:
            _type, method, args = message
            msgid, wants_reply = None, False
        else:
            return None
        if isinstance(method, bytes):
            method = method.decode("utf-8", "replace")
        if not isinstance(method, str):
            return None
        if args is None:
            args = []
        if not isinstance(args, (list, tuple)):
            # Answerable rather than fatal: the frame itself is well formed,
            # so the stream is still in sync.
            return msgid, method, None, wants_reply
        return msgid, method, list(args), wants_reply

    def _dispatch(self, session, method, args):
        now = self._clock()
        if method in SAM_METHODS:
            return self._sam(session, method, now)
        if args is None:
            return f"{method}: arguments must be an array", None
        handler = self._methods.get(method)
        if handler is None:
            return f"unknown method {method!r}", None
        if method in self._guarded and not self.lease.holds(session, now):
            return ACCESS_DENIED_ERROR, None
        try:
            result = handler(*args)
        except RpcRefusal as exc:
            return exc.error, None
        except TypeError as exc:
            # Arity and type mismatches from the call itself. A TypeError
            # raised deeper inside a handler is reported the same way; the
            # message carries which.
            return f"{method}: bad arguments ({exc})", None
        except ValueError as exc:
            return f"{method}: {exc}", None
        except Exception as exc:
            self._log("error", f"{method} failed: {exc!r}")
            return f"{method} failed: {exc!r}", None
        if method in self._guarded:
            self.lease.touch(now)
        return None, result

    def _sam(self, session, method, now):
        if method == "__sam__request":
            return None, self.lease.request(session, now)
        if method == "__sam__force":
            self.lease.force(session, now)
            return None, None
        if method == "__sam__ping":
            return None, self.lease.ping(session, now)
        return None, self.lease.release(session, now)
