#!/usr/bin/env bash
# Phase 3: run the TRAINED policy on the oval — the recorded RL laps.
# Same window layout as pid_lap.sh so the footage is comparable:
# Gazebo (god view), /camera (what the policy sees), THIS terminal with the
# lap timer in frame.
#
# Usage: bash scripts/rl_lap.sh [--track N] [model.zip]
#   default model: newest *_last.zip
#   --track N: drive random track seed N instead of the oval — the filmed
#              UNSEEN-track demo (small seeds are never in a training pool)
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

TRACK="oval"
MODEL=""
PREV=""
for a in "$@"; do
  if [ "$PREV" = "--track" ]; then TRACK="$a"
  elif [ "$a" != "--track" ]; then MODEL="$a"; fi
  PREV="$a"
done

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

if [ "$TRACK" = "oval" ]; then
  # restore the oval benchmark in case a random track was generated since
  python3 "$PKG/scripts/generate_track.py" > /dev/null
  WORLD="$PKG/worlds/track_oval.sdf"
else
  python3 "$PKG/scripts/generate_track.py" --seed "$TRACK" > /dev/null
  WORLD="$PKG/worlds/track_random.sdf"
fi

ign gazebo -r "$WORLD" > /tmp/gz_rllap.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /model/linefollower_bot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  > /tmp/gz_rllap_bridge.log 2>&1 &
BRPID=$!
sleep 8
ros2 run rqt_image_view rqt_image_view /camera > /tmp/rqt_camera.log 2>&1 &
RQTPID=$!
python3 "$PKG/scripts/rl_controller.py" $MODEL > /tmp/rl_controller.log 2>&1 &
RLPID=$!
trap 'kill $RLPID $RQTPID $BRPID $GZPID 2>/dev/null' EXIT

echo ""
if [ "$TRACK" = "oval" ]; then
  echo "=== TRAINED POLICY on the oval (PID baseline to beat: 40.4 s/lap) ==="
else
  echo "=== TRAINED POLICY on UNSEEN random track, seed $TRACK ==="
  echo "=== (seed $TRACK was never in any training set) ==="
fi
echo "=== Ctrl-C stops everything ==="
echo ""
python3 "$PKG/scripts/lap_metrics.py"
