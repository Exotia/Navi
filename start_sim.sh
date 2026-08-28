#!/usr/bin/env bash
# Launch the Gazebo rover simulation.
#
#   ./start_sim.sh                              build if needed, then launch
#   ./start_sim.sh --keep-stale                 don't clean up a previous run first
#   ./start_sim.sh --twist-topic /sim_test_twist   drive the sim from a scratch
#                                                topic instead of /manual_twist,
#                                                which drives the physical rover
#   ./start_sim.sh --map-mesh /path/to/mesh.obj
#
# Streams its chase camera to the ground station over UDP 5601 - a different
# port than the rover's own 5600, so the two senders can never contend and
# decode each other's late packets as garbage.
#
# The ground station counterpart is ./start_ground_station.sh; switching it
# to Semi-autonomous mode points its video panel at this stream.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/sim"
ROS_SETUP="/opt/ros/humble/setup.bash"

TWIST_TOPIC="${TWIST_TOPIC:-/manual_twist}"
MAP_MESH="${MAP_MESH:-$REPO_DIR/Model3D_mesh2.obj}"

CLEAN_STALE=1
while true; do
    case "${1:-}" in
        --keep-stale) CLEAN_STALE=0; shift ;;
        --twist-topic) TWIST_TOPIC="$2"; shift 2 ;;
        --map-mesh) MAP_MESH="$2"; shift 2 ;;
        *) break ;;
    esac
done

if [ ! -f "$ROS_SETUP" ]; then
    echo "error: no ROS 2 install found at $ROS_SETUP" >&2
    exit 1
fi

if [ ! -f "$MAP_MESH" ]; then
    echo "error: map mesh not found: $MAP_MESH" >&2
    echo "       it is gitignored (161 MB) and not part of the repository - pass" >&2
    echo "       --map-mesh /path/to/Model3D_mesh2.obj if it lives elsewhere." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Clean up whatever a previous run left behind.
#
# Killed rather than shut down cleanly, gzserver/gzclient/sim_ik_node and
# sim_video_sender's gst-launch child survive as orphans - gzserver holds
# the physics step, sim_video_sender's udpsink keeps port 5601 bound, and
# the next run's spawn_entity or udpsink then either collides with a ghost
# model or fights an existing sender over the same port.
# ---------------------------------------------------------------------------

# Every pid from here up to init. pgrep -f matches whole command lines, so a
# wrapper shell whose command line contains this script's path would match
# the patterns below - killing those would kill the shell running this.
own_pids() {
    local pid=$$
    while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
        echo "$pid"
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    done
}

kill_stale() {
    local description="$1" mode="$2" pattern="$3"
    local mine victims=() pgrep_flag
    mine=$(own_pids)
    [ "$mode" = "exact" ] && pgrep_flag="-x" || pgrep_flag="-f"
    for pid in $(pgrep "$pgrep_flag" "$pattern" 2>/dev/null || true); do
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
    # pgrep -x matches against the kernel's comm field, which truncates at
    # 15 characters - gzserver (8), gzclient (8) and sim_ik_node (11) are
    # all short enough to match exactly, checked, not assumed.
    # sim_video_sender is 16, so comm reports "sim_video_sende" and -x
    # against the full name never matches, even when a stale sender is
    # genuinely holding UDP 5601 - confirmed live: a process named
    # sim_video_sender is invisible to `pgrep -x sim_video_sender` but
    # found by `pgrep -f sim_video_sender`, which matches on the full
    # command line rather than the truncated comm field.
    kill_stale "Gazebo servers" exact "gzserver"
    kill_stale "Gazebo clients" exact "gzclient"
    kill_stale "simulation IK nodes" exact "sim_ik_node"
    kill_stale "simulation video senders" pattern "sim_video_sender"
    # Matched on the elements this project's send pipeline always has, so
    # an unrelated gst-launch on this machine is left alone.
    kill_stale "simulation send pipelines" pattern "gst-launch-1\.0.*fdsrc.*udpsink"
    # No pattern for this script itself: it would match this very process,
    # and a leftover instance whose launch has just been killed exits
    # through ros2 launch's own teardown anyway.
fi

echo "building navi_sim_bringup, navi_sim_ik, navi_sim_video"
bash -c "source '$ROS_SETUP' && cd '$SIM_DIR' && \
  colcon build --packages-select navi_sim_bringup navi_sim_ik navi_sim_video"

echo "sim -> Gazebo world with the rover in the scanned site (twist topic: $TWIST_TOPIC)"
echo "     -> chase camera streaming to UDP 5601 once the ground station is in Semi-autonomous mode"
# Not exec until here on purpose is fine: nothing else was started in this
# script that a trap would need to tear down - ros2 launch is the last and
# only long-running process, so replacing this shell with it costs nothing.
exec bash -c "source '$ROS_SETUP' && source '$SIM_DIR/install/setup.bash' && \
  ros2 launch navi_sim_bringup sim.launch.py map_mesh:='$MAP_MESH' twist_topic:='$TWIST_TOPIC'"
