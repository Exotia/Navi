#!/usr/bin/env bash
# PostToolUse(Edit|Write): run the test file that covers the edited file, with
# the interpreter and PYTHONPATH that part of the repo needs. Fast: one file.
set -u
input=$(cat)
path=$(printf '%s' "$input" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("tool_input",{}).get("file_path",""))' 2>/dev/null)
[ -z "$path" ] && exit 0
case "$path" in *.py) ;; *) exit 0 ;; esac
cd "$(git rev-parse --show-toplevel 2>/dev/null || pwd)" || exit 0
rel=${path#$PWD/}
base=$(basename "$rel" .py)
if [[ "$base" == test_* ]]; then test_file="$rel"; else test_file=$(find "$(dirname "$rel")/../test" "$(dirname "$rel")/../tests" tests -name "test_${base}.py" 2>/dev/null | head -1); fi
[ -z "$test_file" ] || [ ! -f "$test_file" ] && exit 0
case "$rel" in
  rover/*)  cmd="source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PWD/rover/src/navi_teleop:\$PYTHONPATH python3 -m pytest $test_file -q -p no:cacheprovider" ;;
  sim/*)    cmd="source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PWD/rover/src/navi_localization:\$PYTHONPATH python3 -m pytest $test_file -q -p no:cacheprovider" ;;
  ground_station/*|tests/*) cmd="QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest $test_file -q" ;;
  *) exit 0 ;;
esac
out=$(bash -c "$cmd" 2>&1 | tail -3)
echo "[tests] $test_file: $out"
exit 0
