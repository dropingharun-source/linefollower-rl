#!/usr/bin/env bash
# Done-check for the ground-truth pose plugin (Phase 3 prep).
# Verifies: /model/linefollower_bot/pose_gt publishes, tracks a set_pose
# teleport, and the wheel-encoder odometry does NOT (that's why we need it).
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
ign gazebo -r -s --headless-rendering "$PKG/worlds/track_oval.sdf" > /tmp/gz_posegt.log 2>&1 &
GZPID=$!
trap 'kill $GZPID 2>/dev/null' EXIT
sleep 8

get_xy() {  # topic -> "x y"
  timeout 10 ign topic -e -t "$1" -n 1 2>/dev/null | python3 -c '
import sys, re
s = sys.stdin.read()
m = re.search(r"position \{([^}]*)", s)
b = m.group(1) if m else ""
def g(k):
    mm = re.search(k + r":\s*(-?[\d.e+-]+)", b)
    return float(mm.group(1)) if mm else 0.0
print(g("x"), g("y"))'
}

GT0=$(get_xy /model/linefollower_bot/pose_gt)
[ -n "$GT0" ] || { echo 'FAIL: pose_gt not publishing'; exit 1; }
echo "pose_gt before teleport: $GT0"

ign service -s /world/track_oval/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
  --req 'name: "linefollower_bot", position: {x: 0.5, y: 0.7, z: 0.05}, orientation: {w: 1.0}' \
  > /dev/null
sleep 1

GT1=$(get_xy /model/linefollower_bot/pose_gt)
OD1=$(get_xy /model/linefollower_bot/odometry)
echo "pose_gt after teleport to (0.5, 0.7): $GT1"
echo "wheel odometry after teleport:        $OD1"

python3 - "$GT1" "$OD1" <<'EOF'
import sys
gx, gy = map(float, sys.argv[1].split())
ox, oy = map(float, sys.argv[2].split())
ok_gt = abs(gx - 0.5) < 0.05 and abs(gy - 0.7) < 0.05
stale_odom = abs(ox - 0.5) > 0.2 or abs(oy - 0.7) > 0.2
print('ground truth tracks teleport:', 'PASS' if ok_gt else 'FAIL')
print('wheel odom stays stale (expected):', 'PASS' if stale_odom else 'FAIL')
sys.exit(0 if (ok_gt and stale_odom) else 1)
EOF
echo 'test_pose_gt: ALL PASS'
