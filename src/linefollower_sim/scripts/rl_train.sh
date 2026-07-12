#!/usr/bin/env bash
# Phase 3 step 2: PPO training launcher — THE RECORDED FIRST RUN.
# ⚠ FIRST RUN: START OBS BEFORE RUNNING THIS. The untrained policy's drunk
# phase is Claim 1's strongest proof and cannot be re-filmed once it learns.
#
# Usage: bash scripts/rl_train.sh              GUI mode (for filming)
#        bash scripts/rl_train.sh --headless   no GUI (overnight training)
# Extra args go to train_ppo.py, e.g.:
#        bash scripts/rl_train.sh --headless --resume auto
#
# GUI mode windows: Gazebo (god view), /camera (what the policy sees),
# THIS terminal (SB3 logs — ep_rew_mean is the learning curve). Keep all
# three in the OBS frame.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

MODE="gui"
TRAIN_ARGS=()
for a in "$@"; do
  if [ "$a" = "--headless" ]; then MODE="headless"; else TRAIN_ARGS+=("$a"); fi
done

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true

# restore the oval benchmark in case a random track was generated since
python3 "$PKG/scripts/generate_track.py" > /dev/null

if [ "$MODE" = "headless" ]; then
  ign gazebo -r -s --headless-rendering "$PKG/worlds/track_oval.sdf" \
    > /tmp/gz_rltrain.log 2>&1 &
else
  ign gazebo -r "$PKG/worlds/track_oval.sdf" > /tmp/gz_rltrain.log 2>&1 &
fi
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /model/linefollower_bot/pose_gt@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  > /tmp/gz_rltrain_bridge.log 2>&1 &
BRPID=$!
RQTPID=""
if [ "$MODE" = "gui" ]; then
  sleep 8
  ros2 run rqt_image_view rqt_image_view /camera > /tmp/rqt_camera.log 2>&1 &
  RQTPID=$!
  sleep 2
else
  sleep 8
fi
trap 'kill $RQTPID $BRPID $GZPID 2>/dev/null' EXIT

echo ""
echo "=== PPO training on the oval ==="
echo "=== Ctrl-C stops safely (model is saved); resume with:"
echo "===   bash scripts/rl_train.sh --headless --resume auto"
echo ""
python3 "$PKG/scripts/train_ppo.py" "${TRAIN_ARGS[@]}"
