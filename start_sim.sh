#!/usr/bin/env bash
# Launch the Gazebo rover simulation.
#
#   ./start_sim.sh                              build if needed, then launch
#   ./start_sim.sh --mode semi                  place the rover from the rover's
#                                               own /localization/pose, on its
#                                               own ROS domain (default 42)
#   ./start_sim.sh --mode semi --sim-domain 7   ... on domain 7 instead
#   ./start_sim.sh --mode semi --rover-domain 91  read the rover's topics off
#                                               domain 91 instead of 0 (for
#                                               mock/fake_localization.py)
#   ./start_sim.sh --keep-stale                 don't clean up a previous run first
#   ./start_sim.sh --twist-topic /sim_test_twist   drive the sim from a scratch
#                                                topic instead of /manual_twist,
#                                                which drives the physical rover
#   ./start_sim.sh --map-mesh /path/to/mesh.obj
#
# Two modes, and the difference is where the rover in the picture comes from:
#
#   simulation (default) - the pose is integrated from the commanded twist.
#       No localisation, drifts without bound, runs on this process's domain.
#       The ground station's Simulation mode, marked DEAD RECKONING.
#   semi - the pose is the rover's own, read from /localization/pose on the
#       rover's domain and carried across by sim_bridge. The simulation runs
#       on its own ROS domain so its /clock, /tf and /robot_description never
#       land on the rover's graph, and nothing goes back the other way.
#       The ground station's Semi-autonomous mode.
#
# Streams its chase camera to the ground station over UDP 5601 - a different
# port than the rover's own 5600, so the two senders can never contend and
# decode each other's late packets as garbage.
#
# The ground station counterpart is ./start_ground_station.sh.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$REPO_DIR/sim"
ROS_SETUP="/opt/ros/humble/setup.bash"

TWIST_TOPIC="${TWIST_TOPIC:-/manual_twist}"
MAP_MESH="${MAP_MESH:-$REPO_DIR/Model3D_mesh2.obj}"
MODE="${MODE:-simulation}"
SIM_DOMAIN="${SIM_DOMAIN:-42}"
ROVER_DOMAIN="${ROVER_DOMAIN:-0}"

CLEAN_STALE=1
while true; do
    case "${1:-}" in
        --keep-stale) CLEAN_STALE=0; shift ;;
        --twist-topic) TWIST_TOPIC="$2"; shift 2 ;;
        --map-mesh) MAP_MESH="$2"; shift 2 ;;
        --mode) MODE="$2"; shift 2 ;;
        --sim-domain) SIM_DOMAIN="$2"; shift 2 ;;
        --rover-domain) ROVER_DOMAIN="$2"; shift 2 ;;
        *) break ;;
    esac
done

case "$MODE" in
    simulation|semi) ;;
    *) echo "error: --mode must be 'simulation' or 'semi', not '$MODE'" >&2; exit 2 ;;
esac

for pair in "sim domain:$SIM_DOMAIN" "rover domain:$ROVER_DOMAIN"; do
    value="${pair#*:}"
    if ! [[ "$value" =~ ^[0-9]+$ ]] || [ "$value" -gt 232 ]; then
        echo "error: ${pair%%:*} must be a whole number from 0 to 232, not '$value'" >&2
        exit 2
    fi
done

if [ "$MODE" = "semi" ] && [ "$SIM_DOMAIN" = "$ROVER_DOMAIN" ]; then
    # The whole point of semi mode is that the simulation is not on the
    # rover's domain: /clock at 100 Hz, a /tf tree and a second
    # /robot_description would all land there.
    echo "error: --sim-domain and --rover-domain are both $SIM_DOMAIN;" >&2
    echo "       in semi mode the simulation must have a domain of its own." >&2
    exit 2
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "error: no ROS 2 install found at $ROS_SETUP" >&2
    exit 1
fi

# Only simulation mode displays the organisers' scan. In semi mode the
# ground is the rover's own map, spawned at run time by terrain_writer, and
# demanding a 161 MB gitignored .obj for it would be absurd.
if [ "$MODE" = "simulation" ] && [ ! -f "$MAP_MESH" ]; then
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
    # A stale bridge holds two DDS contexts and would keep republishing the
    # rover's topics onto the sim domain alongside the new one's.
    kill_stale "simulation domain bridges" pattern "sim_bridge\.py"
    # Both outlived their launch for hours, found live: a second
    # terrain_writer respawns the same 'terrain' model against the new
    # one, and a second robot_state_publisher publishes a second
    # /robot_description. robot_state_publisher's comm field truncates to
    # robot_state_pub, so that is the exact name to match; terrain_writer
    # is a python3 script, so its comm is "python3" and only the command
    # line identifies it.
    kill_stale "terrain writers" pattern "navi_sim_bringup/terrain_writer"
    kill_stale "robot state publishers" exact "robot_state_pub"
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

echo "sim -> Gazebo world with the rover in the scanned site (mode: $MODE, twist topic: $TWIST_TOPIC)"
if [ "$MODE" = "semi" ]; then
    echo "     -> rover placed from /localization/pose, read off domain $ROVER_DOMAIN"
    echo "     -> simulation on ROS_DOMAIN_ID=$SIM_DOMAIN; nothing goes back the other way"
else
    echo "     -> rover placed by dead reckoning from the commanded twist"
fi
echo "     -> chase camera streaming to UDP 5601 once the ground station shows it"

LAUNCH="ros2 launch navi_sim_bringup sim.launch.py \
  map_mesh:='$MAP_MESH' twist_topic:='$TWIST_TOPIC' mode:='$MODE' \
  sim_domain:='$SIM_DOMAIN' rover_domain:='$ROVER_DOMAIN'"

# The domain is exported for the launch, not for the build: colcon does not
# care, and exporting it earlier would only widen the window in which a
# stray ros2 command in this script talked to the wrong graph. In simulation
# mode the environment is left exactly as it was, so that mode is unchanged.
#
# Not exec until here on purpose is fine: nothing else was started in this
# script that a trap would need to tear down - ros2 launch is the last and
# only long-running process, so replacing this shell with it costs nothing.
if [ "$MODE" = "semi" ]; then
    exec bash -c "source '$ROS_SETUP' && source '$SIM_DIR/install/setup.bash' && \
      export ROS_DOMAIN_ID='$SIM_DOMAIN' && $LAUNCH"
else
    exec bash -c "source '$ROS_SETUP' && source '$SIM_DIR/install/setup.bash' && $LAUNCH"
fi
