#!/usr/bin/env bash
# Headless eval of the trained policy: same stack as test_gym_env.sh,
# then test_rl_policy.py drives one deterministic episode through the env.
#
# Usage: bash scripts/test_rl_policy.sh [--track N] [model.zip] [--trace]
#   --track N   evaluate on random track seed N instead of the oval —
#               the UNSEEN-track eval (small seeds are never in a pool).
#               Leaves models/track_line holding that random track; any
#               oval script regenerates the oval at its own start.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

TRACK="oval"
PREV=""
for a in "$@"; do
  if [ "$PREV" = "--track" ]; then TRACK="$a"; fi
  PREV="$a"
done

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

if [ "$TRACK" = "oval" ]; then
  python3 "$PKG/scripts/generate_track.py" > /dev/null   # restore the oval
  WORLD="$PKG/worlds/track_oval.sdf"
else
  python3 "$PKG/scripts/generate_track.py" --seed "$TRACK" > /dev/null
  WORLD="$PKG/worlds/track_random.sdf"
fi

ign gazebo -r -s --headless-rendering "$WORLD" \
  > /tmp/gz_rleval.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /model/linefollower_bot/pose_gt@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  > /tmp/gz_rleval_bridge.log 2>&1 &
BRPID=$!
trap 'kill $BRPID $GZPID 2>/dev/null' EXIT
sleep 8

cd "$PKG/scripts"
python3 test_rl_policy.py "$@"
