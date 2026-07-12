#!/usr/bin/env bash
# Phase 1 step 2 done-check: camera publishes 64x64 frames into ROS 2.
# Checks topic rate, then saves one frame (the black test strip in
# empty_test.sdf must be visible in it). Honors LIBGL_ALWAYS_SOFTWARE
# from the caller's environment so both render paths can be tested.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
source /opt/ros/humble/setup.bash

ign gazebo -s -r -v1 "$PKG/worlds/empty_test.sdf" > /tmp/gz_server.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image > /tmp/gz_bridge.log 2>&1 &
BRPID=$!
trap 'kill $GZPID $BRPID 2>/dev/null' EXIT
sleep 10

echo "--- ros2 topic hz /camera (10s window) ---"
HZ_OUT=$(timeout 10 ros2 topic hz /camera 2>&1 || true)
echo "$HZ_OUT"
echo "$HZ_OUT" | grep -q 'average rate' || { echo "no frames; server log tail:"; tail -20 /tmp/gz_server.log; exit 1; }

echo "--- saving one frame ---"
python3 "$PKG/scripts/save_camera_frame.py" "${1:-/tmp/frame.ppm}"
