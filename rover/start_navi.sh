#!/usr/bin/env bash
# Bring up the rover side of the manual-drive link:
#   1. rosbridge_server  - the websocket the ground station connects to
#   2. manual_twist_listener - logs the /manual_twist stream the ground
#      station publishes, so what the rover receives is visible here
#   3. video_sender - waits on /video_request and streams the ZED 2i to
#      whichever address the ground station asks for
#   4. localization.launch.py - the ZED 2i wrapper with positional tracking,
#      plus localization_status publishing /localization/pose and
#      /localization/status
#
#   ./start_navi.sh              all three, listener in the foreground
#   ./start_navi.sh --no-bridge  no rosbridge (one is already running)
#   ./start_navi.sh --no-video   no video_sender
#   ./start_navi.sh --no-localization  no ZED tracking; video from the camera as a UVC device
#   ./start_navi.sh --port 9091  serve rosbridge on a different port
#   ./start_navi.sh --keep-stale don't clean up a previous run first
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
START_LOCALIZATION=1
CLEAN_STALE=1

while [ $# -gt 0 ]; do
    case "$1" in
        --no-bridge) START_BRIDGE=0; shift ;;
        --no-video) START_VIDEO=0; shift ;;
        --no-localization) START_LOCALIZATION=0; shift ;;
        --keep-stale) CLEAN_STALE=0; shift ;;
        --port) PORT="$2"; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# ---------------------------------------------------------------------------
# Clean up whatever a previous run left behind.
#
# A run that is killed rather than stopped (a closed terminal, a dropped ssh
# session, SIGKILL) leaves its nodes orphaned. The one that actually bites is
# rosbridge: it keeps port 9090, the next run's bridge logs "Address already
# in use" and retries forever, and because "ros2 launch" stays alive while it
# retries, the old health check saw a live process and reported success. The
# ground station then connected to the *previous* run's bridge without
# anything saying so. A leftover gst-launch holding /dev/video0 fails the
# next video request the same silent way.
# ---------------------------------------------------------------------------

# Every pid from here up to init. pgrep -f matches on the whole command line,
# so an ssh wrapper like `bash -c '.../start_navi.sh ...'` matches the
# patterns below - killing those would kill the shell running this script.
own_pids() {
    local pid=$$
    while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
        echo "$pid"
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
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

# Empty when nothing holds the port. The `|| true` matters: under
# `set -o pipefail` the grep in this pipeline returns 1 when the port is
# free, which would abort the whole script with no message at all.
port_holder() {
    ss -ltnpH "sport = :$PORT" 2>/dev/null \
        | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2 || true
}

# Whatever is sitting on the rosbridge port. Only killed when it really is a
# rosbridge - anything else on that port is something this script does not
# understand, and guessing would be worse than stopping.
#
# This loops because a rosbridge that failed to bind retries every 5s: kill
# the holder and a retrying one takes the port a moment later. Killing the
# "ros2 launch" wrappers first (above) stops the retries at the source, and
# the loop covers whatever slips through in between.
clear_bridge_port() {
    local holder
    for _ in 1 2 3 4 5; do
        holder=$(port_holder)
        [ -z "$holder" ] && return 0
        if ! ps -o args= -p "$holder" 2>/dev/null | grep -q rosbridge; then
            echo "error: port $PORT is held by pid $holder, which is not a rosbridge:" >&2
            ps -o pid,args= -p "$holder" >&2
            echo "       stop it yourself, or run with --port to use another port" >&2
            exit 1
        fi
        echo "cleaning up stale rosbridge on port $PORT: $holder"
        kill "$holder" 2>/dev/null || true
        for _ in 1 2 3; do
            kill -0 "$holder" 2>/dev/null || break
            sleep 1
        done
        kill -0 "$holder" 2>/dev/null && kill -9 "$holder" 2>/dev/null || true
        sleep 1
    done
    if [ -n "$(port_holder)" ]; then
        echo "error: port $PORT keeps being retaken - check: ss -ltnp | grep $PORT" >&2
        exit 1
    fi
}

if [ "$CLEAN_STALE" -eq 1 ]; then
    kill_stale "navi_teleop nodes" "navi_teleop/(manual_twist_listener|video_sender)"
    kill_stale "ros2 run wrappers" "ros2 run navi_teleop"
    # The pipeline video_sender spawns. Matched on the elements it always
    # contains, so an unrelated gst-launch on this machine is left alone.
    kill_stale "video pipelines" "gst-launch-1\.0.*v4l2src.*udpsink"
    # The wrapper runs in a composable-node container; a killed run leaves
    # it holding the camera, and the next wrapper then fails to open it.
    kill_stale "ZED wrapper containers" "component_container_isolated.*zed"
    kill_stale "localisation launches" "ros2 launch navi_localization"
    kill_stale "localization_status nodes" "navi_localization/localization_status"
    # The stdin-fed encode pipeline video_sender's zed_topic source spawns.
    kill_stale "video pipe pipelines" "gst-launch-1\.0.*fdsrc.*udpsink"
    # Deliberately no pattern for start_navi.sh itself. It would match this
    # very process (and any ssh wrapper whose command line contains the
    # script's path), and there is nothing to gain: a leftover instance
    # whose nodes have just been killed runs its own exit trap and stops.
    if [ "$START_BRIDGE" -eq 1 ]; then
        # Before the port itself: these are the launches this script starts,
        # and a stale one retries the bind every 5s, so it would simply
        # retake the port the moment the current holder is killed.
        kill_stale "rosbridge launches" "ros2 launch rosbridge_server"
        clear_bridge_port
    fi
fi

source /opt/ros/humble/setup.bash

if [ ! -f "$WS_DIR/install/local_setup.bash" ]; then
    echo "error: workspace not built - run: cd $WS_DIR && colcon build --symlink-install" >&2
    exit 1
fi
source "$WS_DIR/install/local_setup.bash"

ZED_WS="${ZED_WS:-$HOME/workspaces/isaac_ros-dev}"
if [ "$START_LOCALIZATION" -eq 1 ]; then
    if [ ! -f "$ZED_WS/install/setup.bash" ]; then
        echo "error: ZED wrapper workspace not found at $ZED_WS" >&2
        echo "       set ZED_WS, or run with --no-localization" >&2
        exit 1
    fi
    source "$ZED_WS/install/setup.bash"
    # After, not before, the wrapper's setup: that setup resets the overlay
    # and this workspace would otherwise fall out of the path.
    source "$WS_DIR/install/local_setup.bash"
fi

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

    # Wait for the port to actually accept a connection, not just for the
    # launch process to still exist. rosbridge retries a failed bind every
    # 5s forever, and "ros2 launch" stays alive throughout - so a liveness
    # check alone reports success while nothing is being served.
    BRIDGE_UP=0
    for _ in $(seq 1 15); do
        if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
            echo "error: rosbridge_server exited during startup" >&2
            exit 1
        fi
        if timeout 1 bash -c "</dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
            BRIDGE_UP=1
            break
        fi
        sleep 1
    done
    if [ "$BRIDGE_UP" -ne 1 ]; then
        echo "error: rosbridge_server never started serving port $PORT" >&2
        echo "       something else is holding it - check: ss -ltnp | grep $PORT" >&2
        exit 1
    fi
    echo "rosbridge_server is serving port $PORT"
fi

if [ "$START_LOCALIZATION" -eq 1 ]; then
    echo "starting localisation (ZED 2i tracking)"
    ros2 launch navi_localization localization.launch.py &
    LOC_PID=$!
    BACKGROUND_PIDS+=("$LOC_PID")

    # Ready means a status message has actually been received, not that
    # the launch process exists: the wrapper spends its first seconds
    # opening the camera and advertising, and a camera it cannot open
    # leaves the launch alive with nothing behind it.
    LOC_UP=0
    for _ in $(seq 1 30); do
        if ! kill -0 "$LOC_PID" 2>/dev/null; then
            echo "error: localisation launch exited during startup" >&2
            exit 1
        fi
        if timeout 3 ros2 topic echo /localization/status --once >/dev/null 2>&1; then
            LOC_UP=1
            break
        fi
    done
    if [ "$LOC_UP" -ne 1 ]; then
        echo "error: /localization/status never arrived within 90 s" >&2
        echo "       is the ZED 2i plugged in? see the wrapper output above" >&2
        exit 1
    fi
    echo "localisation is publishing /localization/status"
fi

if [ "$START_VIDEO" -eq 1 ]; then
    # Not fatal when the camera is absent: the node idles until a request
    # arrives, and manual drive has to keep working without video.
    if [ "$START_LOCALIZATION" -eq 0 ]; then
        if [ ! -e /dev/video0 ]; then
            echo "warning: /dev/video0 does not exist - is the ZED 2i plugged in?" >&2
            echo "         video_sender will start anyway and fail any request" >&2
        fi
    fi
    echo "starting video_sender (idle until the ground station asks for video)"
    ros2 run navi_teleop video_sender --ros-args -p source:=$( [ "$START_LOCALIZATION" -eq 1 ] && echo zed_topic || echo v4l2 ) &
    BACKGROUND_PIDS+=("$!")
fi

echo "starting manual_twist_listener (Ctrl+C to stop everything)"
ros2 run navi_teleop manual_twist_listener
