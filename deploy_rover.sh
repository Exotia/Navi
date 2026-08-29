#!/usr/bin/env bash
# Deploy rover/ to the Orin and build it there.
#
#   ./deploy_rover.sh            rsync, then colcon build on the Orin
#   ./deploy_rover.sh --test     ... and run the Orin-side test suites
#   ./deploy_rover.sh --no-build rsync only
#
# ~/navi on the Orin is a deploy target, not a development tree: edit under
# rover/ here, commit here, deploy with this. The Orin's build/ install/ log/
# are left alone (they are gitignored on both sides).

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_SSH="${ROVER_SSH:-star@a_navi}"
ROVER_DIR="${ROVER_DIR:-navi}"

BUILD=1
TEST=0
while [ $# -gt 0 ]; do
    case "$1" in
        --no-build) BUILD=0; shift ;;
        --test) TEST=1; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# --delete so a file removed here disappears there; the excludes keep the
# Orin's build products and its own .git out of the deletion.
rsync -az --delete \
    --exclude '.git' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
    --exclude '__pycache__/' --exclude '.pytest_cache/' \
    "$REPO_DIR/rover/" "$ROVER_SSH:$ROVER_DIR/"
echo "synced rover/ -> $ROVER_SSH:$ROVER_DIR/"

[ "$BUILD" -eq 1 ] || exit 0

# One remote shell, one script: sourcing ROS is per-shell state.
REMOTE='set -eo pipefail
source /opt/ros/humble/setup.bash
[ -f ~/workspaces/isaac_ros-dev/install/setup.bash ] && source ~/workspaces/isaac_ros-dev/install/setup.bash
cd ~/'"$ROVER_DIR"'
colcon build --symlink-install
'
if [ "$TEST" -eq 1 ]; then
    REMOTE+='source install/local_setup.bash
for pkg in src/*/; do
    [ -d "$pkg/test" ] || continue
    echo "== pytest $pkg"
    (cd "$pkg" && python3 -m pytest test -q)
done
'
fi
ssh "$ROVER_SSH" "$REMOTE"
