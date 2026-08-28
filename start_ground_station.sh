#!/usr/bin/env bash
# Launch the ground station against the rover's rosbridge.
#
#   ./start_ground_station.sh              connect to the Orin (a_navi)
#   ./start_ground_station.sh --mock       start the local mock rosbridge too,
#                                          and connect to that instead - no
#                                          rover, no ROS2 needed
#   ./start_ground_station.sh 192.168.178.99 9090   explicit host/port
#   ./start_ground_station.sh --keep-stale don't clean up a previous run first
#
# The rover counterpart is ~/navi/start_navi.sh on the Orin, which brings up
# rosbridge_server, manual_twist_listener and video_sender.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$REPO_DIR/.venv/bin/python"

ROVER_HOST="${ROVER_HOST:-192.168.178.33}"   # a_navi (Jetson Orin Nano)
ROVER_PORT="${ROVER_PORT:-9090}"

USE_MOCK=0
CLEAN_STALE=1
while true; do
    case "${1:-}" in
        --mock) USE_MOCK=1; shift ;;
        --keep-stale) CLEAN_STALE=0; shift ;;
        *) break ;;
    esac
done
[ $# -ge 1 ] && ROVER_HOST="$1"
[ $# -ge 2 ] && ROVER_PORT="$2"

if [ ! -x "$PYTHON" ]; then
    echo "error: no virtualenv at $REPO_DIR/.venv" >&2
    echo "       create it with: python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Clean up whatever a previous run left behind.
#
# When the app is killed rather than closed, its decode pipeline survives as
# an orphaned gst-launch still bound to the video UDP port. The next run's
# udpsrc then cannot bind, the panel shows a dead receiver, and nothing says
# why. A mock rosbridge from a --mock run holds its TCP port the same way.
# ---------------------------------------------------------------------------

# Every pid from here up to init. pgrep -f matches whole command lines, so a
# wrapper shell whose command line contains this script's path would match
# the patterns below - killing those would kill the shell running this.
own_pids() {
    local pid=$$
    while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
        echo "$pid"
        # || true is load-bearing: under `set -euo pipefail` a failing
        # ps makes the whole pipeline return 1, a bare assignment
        # inherits that status, and set -e kills the subshell - so
        # `mine=$(own_pids)` in kill_stale below fails inside an if
        # body where set -e is not suppressed, and the launcher exits
        # instantly with no output at all (the 2>/dev/null means not
        # even a stderr crumb). Reproduced; one token prevents it.
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ' || true)
    done
}

kill_stale() {
    local description="$1" pattern="$2"
    local mine victims=()
    mine=$(own_pids)
    for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
        grep -qx "$pid" <<< "$mine" && continue
        victims+=("$pid")
    done
    [ ${#victims[@]} -eq 0 ] && return 0
    echo "cleaning up stale $description: ${victims[*]}"
    kill "${victims[@]}" 2>/dev/null || true
    sleep 1
    for pid in "${victims[@]}"; do
        kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null || true
    done
}

if [ "$CLEAN_STALE" -eq 1 ]; then
    # Matched on the elements this project's receive pipeline always has, so
    # an unrelated gst-launch on this laptop is left alone.
    kill_stale "video receive pipelines" "gst-launch-1\.0.*udpsrc.*avdec_h264"
    kill_stale "ground station instances" "python.* -m ground_station\.main"
    kill_stale "mock rosbridge servers" "python.*mock/ros_bridge\.py"
    # No pattern for this script itself: it would match this very process,
    # and a leftover instance whose app has just been killed exits through
    # its own trap anyway.
fi

MOCK_PID=""
cleanup() {
    if [ -n "$MOCK_PID" ] && kill -0 "$MOCK_PID" 2>/dev/null; then
        kill "$MOCK_PID" 2>/dev/null || true
        wait "$MOCK_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [ "$USE_MOCK" -eq 1 ]; then
    ROVER_HOST="localhost"
    echo "starting mock rosbridge on port $ROVER_PORT"
    "$PYTHON" "$REPO_DIR/mock/ros_bridge.py" --port "$ROVER_PORT" &
    MOCK_PID=$!
    # The GUI's initial connect fires as soon as the event loop starts, so the
    # server has to be accepting before we hand off to it.
    sleep 1
elif ! timeout 3 bash -c "</dev/tcp/$ROVER_HOST/$ROVER_PORT" 2>/dev/null; then
    # Not fatal: the app opens unconnected and you can retry from its header.
    echo "warning: nothing listening on $ROVER_HOST:$ROVER_PORT" >&2
    echo "         is ~/navi/start_navi.sh running on the Orin?" >&2
fi

if [ -z "${QT_QPA_PLATFORM:-}" ] && [ -n "${DISPLAY:-}" ] && ! ldconfig -p | grep -q xcb-cursor; then
    # PySide6 >= 6.5 needs this to load its xcb platform plugin; without it Qt
    # aborts at QApplication() before any window appears.
    echo "warning: libxcb-cursor0 is missing - Qt will fail to start on X11" >&2
    echo "         install it with: sudo apt install libxcb-cursor0" >&2
fi

echo "ground station -> $ROVER_HOST:$ROVER_PORT"
# Not exec: that would replace this shell and discard the EXIT trap, leaving
# the mock server running after the app quits.
"$PYTHON" -m ground_station.main --host "$ROVER_HOST" --port "$ROVER_PORT"
