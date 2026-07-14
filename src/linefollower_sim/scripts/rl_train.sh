#!/usr/bin/env bash
# Phase 3 step 2: PPO training launcher — THE RECORDED FIRST RUN.
# ⚠ FIRST RUN: START OBS BEFORE RUNNING THIS. The untrained policy's drunk
# phase is Claim 1's strongest proof and cannot be re-filmed once it learns.
#
# Usage: bash scripts/rl_train.sh              GUI mode (for filming)
#        bash scripts/rl_train.sh --headless   no GUI (overnight training)
#        bash scripts/rl_train.sh --pool 16    random track per episode
#                                              (Phase 3 step 3; add
#                                              --pool-seed S for a new pool)
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
POOL=0
POOL_SEED=0
TRAIN_ARGS=()
EXPECT=""
for a in "$@"; do
  if [ "$EXPECT" = "pool" ]; then POOL="$a"; EXPECT=""
  elif [ "$EXPECT" = "pool_seed" ]; then POOL_SEED="$a"; EXPECT=""
  elif [ "$a" = "--headless" ]; then MODE="headless"
  elif [ "$a" = "--pool" ]; then EXPECT="pool"
  elif [ "$a" = "--pool-seed" ]; then EXPECT="pool_seed"
  else TRAIN_ARGS+=("$a"); fi
done

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

if [ "$POOL" -gt 0 ]; then
  # pool world: N random tracks, one teleport-destination per episode;
  # models/track_line (the oval) is not touched
  python3 "$PKG/scripts/generate_track.py" --pool "$POOL" --pool-seed "$POOL_SEED"
  WORLD="$PKG/worlds/track_pool.sdf"
  TRAIN_ARGS+=(--pool "$POOL" --pool-seed "$POOL_SEED")
else
  # restore the oval benchmark in case a random track was generated since
  python3 "$PKG/scripts/generate_track.py" > /dev/null
  WORLD="$PKG/worlds/track_oval.sdf"
fi

if [ "$MODE" = "headless" ]; then
  ign gazebo -r -s --headless-rendering "$WORLD" \
    > /tmp/gz_rltrain.log 2>&1 &
else
  ign gazebo -r "$WORLD" > /tmp/gz_rltrain.log 2>&1 &
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
if [ "$POOL" -gt 0 ]; then
  echo "=== PPO training on a $POOL-track pool (random track per episode) ==="
  echo "=== Ctrl-C stops safely (model is saved); resume with:"
  echo "===   bash scripts/rl_train.sh --headless --pool $POOL --pool-seed $POOL_SEED --resume auto"
else
  echo "=== PPO training on the oval ==="
  echo "=== Ctrl-C stops safely (model is saved); resume with:"
  echo "===   bash scripts/rl_train.sh --headless --resume auto"
fi
echo ""
python3 "$PKG/scripts/train_ppo.py" "${TRAIN_ARGS[@]}"
