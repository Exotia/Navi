#!/usr/bin/env bash
# start_navi.sh's localisation readiness gate, exercised without a rover.
#
# The gate is bash, so it has no pytest to live in - and it is exactly the
# piece that was wrong: localization_status publishes an initial OFF from its
# constructor, so the old loop, which accepted any /localization/status
# message, reported the rover ready with an unplugged camera behind it.
#
# start_navi.sh is sourced with NAVI_FUNCTIONS_ONLY=1, which stops it before
# anything is launched, and wait_for_localization is then run against a fake
# `ros2` on PATH that prints whatever sequence of states a case needs. The
# budgets are seconds, overridable through the environment, so a case that
# waits out a timeout takes a few seconds rather than two and a half minutes.
#
# Run it directly: bash rover/test/test_start_navi_gate.sh (exit 0 = pass).

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/../start_navi.sh"
FAILURES=0

# A `ros2` that answers `topic echo` with one state per invocation, the last
# one repeating forever. NONE means the call fails, the way a real one does
# when nothing is publishing and `timeout` kills it.
make_fake_ros2() {
    local dir="$1"; shift
    printf '%s\n' "$@" > "$dir/states"
    cat > "$dir/ros2" <<'FAKE'
#!/usr/bin/env bash
here="$(dirname "$0")"
mapfile -t states < "$here/states"
n=$(cat "$here/count" 2>/dev/null || echo 0)
echo $((n + 1)) > "$here/count"
[ "$n" -ge "${#states[@]}" ] && n=$(( ${#states[@]} - 1 ))
state="${states[$n]}"
[ "$state" = "NONE" ] && exit 1
echo "data: '{\"state\": \"$state\", \"seconds_since_ok\": null, \"source\": \"zed_vio\"}'"
echo "---"
FAKE
    chmod +x "$dir/ros2"
}

# Runs the gate in its own shell: start_navi.sh sets `set -e`, which must not
# reach this harness. Watches no launch pid - process supervision is not what
# these cases are about.
run_gate() {
    local dir="$1"
    PATH="$dir:$PATH" \
    NAVI_FUNCTIONS_ONLY=1 \
    LOC_STATUS_SECONDS="${LOC_STATUS_SECONDS:-6}" \
    LOC_TRACKING_SECONDS="${LOC_TRACKING_SECONDS:-3}" \
        bash -c 'source "$0"; wait_for_localization ""' "$SCRIPT" 2>&1
}

expect() {
    local name="$1" want_status="$2" want_text="$3"; shift 3
    local dir output status
    dir=$(mktemp -d)
    make_fake_ros2 "$dir" "$@"
    output=$(run_gate "$dir")
    status=$?
    rm -rf "$dir"

    if [ "$status" -ne "$want_status" ]; then
        echo "FAIL $name: exit $status, expected $want_status"
        echo "$output" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if ! grep -q "$want_text" <<< "$output"; then
        echo "FAIL $name: output does not mention '$want_text'"
        echo "$output" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    echo "ok   $name"
    echo "$output" | sed 's/^/     | /'
}

# A tracking ZED: ready at once, and the state it reported is on screen.
expect "an OK state reports ready and names the state" 0 "tracking (state OK)" OK

# The bug: OFF is what an unplugged camera produces within a second of
# start-up. It must not count as ready.
expect "OFF is waited out, and recovery is reported" 0 "tracking (state OK)" OFF OFF OK

# ... but it must not stop a bring-up either. Loud, and continue.
expect "a state that stays OFF warns loudly and continues" 0 "WARNING" OFF

# Nothing publishing at all is the one case that fails the launcher.
LOC_STATUS_SECONDS=3 expect "no status at all fails the gate" 1 "never arrived" NONE

# ---------------------------------------------------------------------------
# twist_shaper wiring: the launch section is below the NAVI_FUNCTIONS_ONLY
# return above and so cannot be sourced - these check the script text itself.
# ---------------------------------------------------------------------------

check() {
    local name="$1" ok="$2"
    if [ "$ok" -eq 0 ]; then
        echo "ok   $name"
    else
        echo "FAIL $name"
        FAILURES=$((FAILURES + 1))
    fi
}

# bema_bridge must be re-pointed at the shaper's output, not the raw stream -
# this is the whole point of SP10 and must never silently regress.
grep -q 'twist_topic:=/chassis_twist' "$SCRIPT"
check "bema_bridge is passed twist_topic:=/chassis_twist" $?

! grep -q 'twist_topic:=/rover_twist' "$SCRIPT"
check "bema_bridge no longer reads twist_topic:=/rover_twist" $?

# The shaper's launch line must come before the bridge's, so the bridge's
# source has a publisher by the time it subscribes.
shaper_line=$(grep -n 'ros2 run navi_shaper twist_shaper' "$SCRIPT" | head -1 | cut -d: -f1)
bridge_line=$(grep -n 'ros2 run navi_teleop bema_bridge' "$SCRIPT" | head -1 | cut -d: -f1)
[ -n "$shaper_line" ] && [ -n "$bridge_line" ] && awk -v s="$shaper_line" -v b="$bridge_line" 'BEGIN { exit !(s < b) }'
check "twist_shaper launch line precedes bema_bridge's" $?

# --no-shaper must be accepted by the argument loop and set START_SHAPER=0.
grep -q -- '--no-shaper) START_SHAPER=0; shift ;;' "$SCRIPT"
check "--no-shaper is accepted and sets START_SHAPER=0" $?

# The stale-cleanup section must name the shaper's node so a killed run
# doesn't leave a stale twist_shaper answering the next one.
grep -q 'navi_shaper/twist_shaper' "$SCRIPT"
check "stale cleanup names navi_shaper/twist_shaper" $?

if [ "$FAILURES" -ne 0 ]; then
    echo "$FAILURES check(s) failed"
    exit 1
fi
echo "all start_navi.sh readiness-gate checks passed"
