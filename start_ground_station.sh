#!/usr/bin/env bash
# Launch the ground station against the rover's rosbridge.
#
#   ./start_ground_station.sh              connect to the Orin (a_navi)
#   ./start_ground_station.sh --mock       start the local mock rosbridge too,
#                                          and connect to that instead - no
#                                          rover, no ROS2 needed
#   ./start_ground_station.sh 192.168.178.99 9090   explicit host/port
#
# The rover counterpart is ~/navi/start_navi.sh on the Orin, which brings up
# rosbridge_server and the manual_twist_listener node.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$REPO_DIR/.venv/bin/python"

ROVER_HOST="${ROVER_HOST:-192.168.178.33}"   # a_navi (Jetson Orin Nano)
ROVER_PORT="${ROVER_PORT:-9090}"

USE_MOCK=0
if [ "${1:-}" = "--mock" ]; then
    USE_MOCK=1
    shift
fi
[ $# -ge 1 ] && ROVER_HOST="$1"
[ $# -ge 2 ] && ROVER_PORT="$2"

if [ ! -x "$PYTHON" ]; then
    echo "error: no virtualenv at $REPO_DIR/.venv" >&2
    echo "       create it with: python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'" >&2
    exit 1
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
