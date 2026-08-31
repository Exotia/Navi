#!/usr/bin/env bash
# start_navi.sh's NaVi alias step, exercised without root and without a LAN.
#
# The coordinator on the primary calls a hard-coded 192.168.178.18:21021, so
# the Orin has to answer on a second address. Adding it needs sudo, which a
# bring-up over ssh may not have - and the one thing this step must never do
# is take the rover down because of that. So: idempotent when the alias is
# already there, loud when it cannot be added, exit 0 either way.
#
# start_navi.sh is sourced with NAVI_FUNCTIONS_ONLY=1, which stops it before
# anything is launched, and ensure_navi_alias is then run against a fake `ip`
# and a fake `sudo` on PATH.
#
# Run it directly: bash rover/test/test_navi_alias.sh (exit 0 = pass).

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/../start_navi.sh"
FAILURES=0

# A fake `ip` that prints the given `ip -o -4 addr show` lines, and records
# every invocation so a case can assert an `addr add` did or did not happen.
make_fakes() {
    local dir="$1" sudo_status="$2"; shift 2
    printf '%s\n' "$@" > "$dir/addrs"
    cat > "$dir/ip" <<'FAKE'
#!/usr/bin/env bash
here="$(dirname "$0")"
echo "ip $*" >> "$here/log"
for arg in "$@"; do
    [ "$arg" = "add" ] && exit 0
done
cat "$here/addrs"
FAKE
    cat > "$dir/sudo" <<FAKE
#!/usr/bin/env bash
here="\$(dirname "\$0")"
echo "sudo \$*" >> "\$here/log"
exit $sudo_status
FAKE
    chmod +x "$dir/ip" "$dir/sudo"
    : > "$dir/log"
}

run_alias() {
    local dir="$1"
    PATH="$dir:$PATH" NAVI_FUNCTIONS_ONLY=1 \
        bash -c 'source "$0"; ensure_navi_alias' "$SCRIPT" 2>&1
}

expect() {
    local name="$1" want_status="$2" want_log_grep="$3" want_absent="$4"
    # want_output_grep is what the operator must SEE. The sudo-refused case
    # exists only to prove the warning is loud, so asserting exit 0 and an
    # attempted `addr add` would assert nothing that case does not share with
    # the case above it.
    local want_output_grep="$5" sudo_status="$6"; shift 6
    local dir output status log
    dir=$(mktemp -d)
    make_fakes "$dir" "$sudo_status" "$@"
    output=$(run_alias "$dir")
    status=$?
    log=$(cat "$dir/log")
    rm -rf "$dir"

    if [ "$status" -ne "$want_status" ]; then
        echo "FAIL $name: exit $status, expected $want_status"
        echo "$output" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$want_log_grep" ] && ! grep -q -- "$want_log_grep" <<< "$log"; then
        echo "FAIL $name: expected '$want_log_grep' in the command log:"
        echo "$log" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$want_absent" ] && grep -q -- "$want_absent" <<< "$log"; then
        echo "FAIL $name: '$want_absent' should not have been run:"
        echo "$log" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$want_output_grep" ] && ! grep -q -- "$want_output_grep" <<< "$output"; then
        echo "FAIL $name: expected '$want_output_grep' in the output:"
        echo "$output" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    echo "ok   $name"
}

LOOPBACK='1: lo    inet 127.0.0.1/8 scope host lo'
ROVER='2: eth0    inet 192.168.178.33/24 brd 192.168.178.255 scope global eth0'
ALIAS='2: eth0    inet 192.168.178.18/24 brd 192.168.178.255 scope global secondary eth0'

expect "already present is a no-op" 0 "" "addr add" "" 0 "$LOOPBACK" "$ROVER" "$ALIAS"
expect "absent and permitted adds it" 0 "addr add 192.168.178.18/24 dev eth0" "" "added the NaVi alias" 0 "$LOOPBACK" "$ROVER"
expect "sudo refused warns and continues" 0 "addr add 192.168.178.18/24 dev eth0" "" "could not add the NaVi alias" 1 "$LOOPBACK" "$ROVER"
expect "no rover LAN warns and continues" 0 "" "addr add" "no interface holds a 192.168.178.x address" 0 "$LOOPBACK"

if [ "$FAILURES" -ne 0 ]; then
    echo "$FAILURES case(s) failed"
    exit 1
fi
echo "all NaVi alias cases pass"
