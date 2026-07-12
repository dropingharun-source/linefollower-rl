#!/usr/bin/env bash
# One-off: verify the teleop stack (sim + bridge) headless on the current
# track_random.sdf before handing the keyboard to a human.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true

ign gazebo -s -r -v1 "$PKG/worlds/track_random.sdf" > /tmp/gz_smoke.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image > /tmp/bridge_smoke.log 2>&1 &
BRPID=$!
trap 'kill $GZPID $BRPID 2>/dev/null' EXIT
sleep 8

echo "--- camera hz (6s window) ---"
timeout 6 ros2 topic hz /camera 2>&1 | head -2 || true
echo "--- cmd_vel -> odometry ---"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
sleep 3
ign topic -e -t /model/linefollower_bot/odometry -n 1 | grep -A 3 position | head -4
