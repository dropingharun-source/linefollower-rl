#!/usr/bin/env bash
# Verifies the ROS 2 -> Gazebo path: parameter_bridge forwards /cmd_vel
# (geometry_msgs/Twist) into the sim and the robot moves.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
source /opt/ros/humble/setup.bash

ign gazebo -s -r -v1 "$PKG/worlds/empty_test.sdf" > /tmp/gz_server.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist > /tmp/gz_bridge.log 2>&1 &
BRPID=$!
trap 'kill $GZPID $BRPID 2>/dev/null' EXIT
sleep 8

echo "--- publishing /cmd_vel from ROS 2 ---"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"
sleep 4

echo "--- odometry after 4s ---"
ign topic -e -t /model/linefollower_bot/odometry -n 1 | grep -A 3 position | head -9
