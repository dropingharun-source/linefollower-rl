#!/usr/bin/env bash
# Done-check for Phase 3 step 3: random track per episode via the track pool.
# Generates the 16-track pool world, launches it headless with the training
# bridge, then runs test_pool_env.py against it.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

python3 "$PKG/scripts/generate_track.py" --pool 16 --pool-seed 0 > /dev/null

ign gazebo -r -s --headless-rendering "$PKG/worlds/track_pool.sdf" \
  > /tmp/gz_poolenv.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /model/linefollower_bot/pose_gt@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  > /tmp/gz_poolenv_bridge.log 2>&1 &
BRPID=$!
trap 'kill $BRPID $GZPID 2>/dev/null' EXIT
sleep 8

cd "$PKG/scripts"
python3 test_pool_env.py
