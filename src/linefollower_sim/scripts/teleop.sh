#!/usr/bin/env bash
# Phase 1 step 5: keyboard teleop on a random track while watching the 64x64 feed.
# Usage: bash scripts/teleop.sh [seed]   (default seed 42)
# Done when: one full lap driven watching ONLY the camera feed. Screen-record it.
# Note: leaves track_random.sdf in place; run generate_track.py with no args
# to restore the oval before any Phase 2 work.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

SEED="${1:-42}"
pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true

python3 "$PKG/scripts/generate_track.py" --seed "$SEED"

ign gazebo -r "$PKG/worlds/track_random.sdf" > /tmp/gz_teleop.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image > /tmp/gz_teleop_bridge.log 2>&1 &
BRPID=$!
sleep 8
ros2 run rqt_image_view rqt_image_view /camera > /tmp/rqt_teleop.log 2>&1 &
RQTPID=$!
trap 'kill $GZPID $BRPID $RQTPID 2>/dev/null' EXIT

echo ""
echo "=== Teleop on random track (seed $SEED) ==="
echo "=== Drive watching ONLY the 64x64 feed window. ==="
echo "=== i=forward  ,=back  j/l=rotate  u/o=curve  k=stop  q/z=faster/slower ==="
echo "=== Ctrl-C here when done (closes sim + viewer too). ==="
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
