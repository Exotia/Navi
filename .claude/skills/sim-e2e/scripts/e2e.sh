#!/usr/bin/env bash
# Laptop end-to-end for semi mode with mocks. Usage: e2e.sh [run_seconds]
set -u
RUN=${1:-120}
ROOT=$(git rev-parse --show-toplevel); cd "$ROOT"
OUT=${TMPDIR:-/tmp}/navi-e2e-$(date +%H%M%S); mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=91
python3 mock/fake_localization.py > "$OUT/fake_loc.log" 2>&1 & P1=$!
PYTHONPATH=$ROOT/rover/src/navi_localization:${PYTHONPATH:-} python3 -m navi_localization.elevation_mapper --ros-args -p map_directory:=/tmp/navi_maps_test > "$OUT/mapper.log" 2>&1 & P2=$!
python3 sim/src/navi_sim_bringup/test/publish_synthetic_cloud.py --size 6 --wall --box > "$OUT/cloud.log" 2>&1 & P3=$!
unset ROS_DOMAIN_ID
./start_sim.sh --mode semi --rover-domain 91 --twist-topic /sim_test_twist > "$OUT/sim.log" 2>&1 & P4=$!
until pgrep -x gzserver >/dev/null; do sleep 2; done
sleep "$RUN"
count() { ROS_DOMAIN_ID=42 timeout 20 ros2 service call /get_model_list gazebo_msgs/srv/GetModelList 2>/dev/null | grep -o "$1_[^,' ]*" | wc -l; }
echo "terrain models: $(count terrain)  obstacle models: $(count obst)"
echo "already-exists errors: $(grep -c 'already exists' "$OUT/sim.log")"
ROS_DOMAIN_ID=42 timeout 30 python3 - "$OUT" <<'PY'
import sys, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as P
rclpy.init(); n=Node('e2e_grab'); out=sys.argv[1]
def cb(m):
    P.frombytes('RGB',(m.width,m.height),bytes(m.data)).save(out+'/frame.png'); print('frame:', out+'/frame.png'); raise SystemExit
n.create_subscription(Image,'/sim_chase_camera/chase/image_raw',cb,1)
try: rclpy.spin(n)
except SystemExit: pass
PY
ROS_DOMAIN_ID=91 ros2 topic pub -1 /localization/map_command std_msgs/String '{data: "{\"action\":\"clear\"}"}' >/dev/null 2>&1
t0=$(date +%s); while [ $(( $(date +%s) - t0 )) -lt 90 ]; do c=$(( $(count terrain) + $(count obst) )); [ "$c" -eq 0 ] && break; sleep 3; done
echo "clear -> $c models after $(( $(date +%s) - t0 )) s"
kill $P4 $P1 $P2 $P3 2>/dev/null; sleep 6; pkill -x gzclient; pkill -x gzserver; sleep 3
pkill -x sim_video_sende; pkill -x sim_ik_node; pkill -x robot_state_pub
for p in $(ps -eo pid,args | grep -E '[t]errain_writer|[s]im_bridge.py' | awk '{print $1}'); do kill $p; done
sleep 2
echo "teardown: gzserver=$(pgrep -x gzserver | wc -l) domain42=$(ROS_DOMAIN_ID=42 timeout 6 ros2 node list 2>/dev/null | wc -l) domain91=$(ROS_DOMAIN_ID=91 timeout 6 ros2 node list 2>/dev/null | wc -l)"
echo "logs: $OUT"
