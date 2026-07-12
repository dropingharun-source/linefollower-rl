#!/usr/bin/env bash
# Done-check for Phase 3 step 1: the Gym environment.
# Launches the headless training stack (sim + bridge, no GUI), then runs
# test_gym_env.py against it. This same stack is what training will use.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true

ign gazebo -r -s --headless-rendering "$PKG/worlds/track_oval.sdf" \
  > /tmp/gz_gymenv.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /model/linefollower_bot/pose_gt@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  > /tmp/gz_gymenv_bridge.log 2>&1 &
BRPID=$!
trap 'kill $BRPID $GZPID 2>/dev/null' EXIT
sleep 8

cd "$PKG/scripts"
python3 test_gym_env.py
