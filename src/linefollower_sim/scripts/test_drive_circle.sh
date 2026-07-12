#!/usr/bin/env bash
# Phase 1 step 1 done-check: spawn the robot in the empty test world (headless),
# command v=0.2 m/s + w=0.5 rad/s (expected circle radius v/w = 0.4 m),
# and print odometry samples so the circular path is visible in the numbers.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"

ign gazebo -s -r -v1 "$PKG/worlds/empty_test.sdf" > /tmp/gz_server.log 2>&1 &
GZPID=$!
trap 'kill $GZPID 2>/dev/null' EXIT
sleep 6

echo "--- topics ---"
ign topic -l | grep -E 'cmd_vel|odometry' || { echo "no topics; server log:"; cat /tmp/gz_server.log; exit 1; }

echo "--- commanding v=0.2 m/s, w=0.5 rad/s ---"
ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.2}, angular: {z: 0.5}'

for t in 2 4 4; do
  sleep "$t"
  echo "--- odometry sample (t+=${t}s) ---"
  ign topic -e -t /model/linefollower_bot/odometry -n 1 | grep -A 3 position | head -9
done
