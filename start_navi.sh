#!/usr/bin/env bash
# Bring up the rover side of the manual-drive link:
#   1. rosbridge_server  - the websocket the ground station connects to
#   2. manual_twist_listener - logs the /manual_twist stream the ground
#      station publishes, so what the rover receives is visible here
#   3. video_sender - waits on /video_request and streams the ZED 2i to
#      whichever address the ground station asks for
#
#   ./start_navi.sh              all three, listener in the foreground
#   ./start_navi.sh --no-bridge  no rosbridge (one is already running)
#   ./start_navi.sh --no-video   no video_sender
#   ./start_navi.sh --port 9091  serve rosbridge on a different port
#
# Ctrl+C stops everything. The laptop counterpart is start_ground_station.sh
# in the Navi repo.

set -eo pipefail
# Deliberately not -u: ROS 2's setup.bash reads unset variables
# (AMENT_TRACE_SETUP_FILES) and would abort the script under -u.

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=9090
START_BRIDGE=1
START_VIDEO=1

while [ $# -gt 0 ]; do
    case "$1" in
        --no-bridge) START_BRIDGE=0; shift ;;
        --no-video) START_VIDEO=0; shift ;;
        --port) PORT="$2"; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

source /opt/ros/humble/setup.bash

if [ ! -f "$WS_DIR/install/local_setup.bash" ]; then
    echo "error: workspace not built - run: cd $WS_DIR && colcon build --symlink-install" >&2
    exit 1
fi
source "$WS_DIR/install/local_setup.bash"

BACKGROUND_PIDS=()
cleanup() {
    for pid in "${BACKGROUND_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

if [ "$START_BRIDGE" -eq 1 ]; then
    if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
        echo "error: rosbridge_server is not installed on this machine." >&2
        echo "       install it with: sudo apt install ros-humble-rosbridge-suite" >&2
        echo "       (or run with --no-bridge to start only the nodes)" >&2
        exit 1
    fi
    echo "starting rosbridge_server on port $PORT"
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$PORT" &
    BRIDGE_PID=$!
    BACKGROUND_PIDS+=("$BRIDGE_PID")
    sleep 2
    if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "error: rosbridge_server exited immediately - is port $PORT already in use?" >&2
        exit 1
    fi
fi

if [ "$START_VIDEO" -eq 1 ]; then
    # Not fatal when the camera is absent: the node idles until a request
    # arrives, and manual drive has to keep working without video.
    if [ ! -e /dev/video0 ]; then
        echo "warning: /dev/video0 does not exist - is the ZED 2i plugged in?" >&2
        echo "         video_sender will start anyway and fail any request" >&2
    fi
    echo "starting video_sender (idle until the ground station asks for video)"
    ros2 run navi_teleop video_sender &
    BACKGROUND_PIDS+=("$!")
fi

echo "starting manual_twist_listener (Ctrl+C to stop everything)"
ros2 run navi_teleop manual_twist_listener
