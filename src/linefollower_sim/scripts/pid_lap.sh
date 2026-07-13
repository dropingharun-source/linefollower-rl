#!/usr/bin/env bash
# Phase 2: run the PID baseline on the oval — THE RECORDED RUNS.
# ⚠ START OBS BEFORE RUNNING THIS. First attempts cannot be re-filmed honestly.
#
# Usage: bash scripts/pid_lap.sh [kp] [kd] [ki] [speed]
#        defaults:                1.0   0.0  0.0  0.15
# Each tuning iteration = Ctrl-C, rerun with new numbers (gains stay visible
# in the terminal for the recording).
#
# Windows: Gazebo (god view), /camera (raw), /line_debug (what PID sees).
# THIS terminal shows the lap timer + line-loss counter (sim time) — keep it
# in frame.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

KP="${1:-1.0}"; KD="${2:-0.0}"; KI="${3:-0.0}"; SPEED="${4:-0.15}"

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

ign gazebo -r "$PKG/worlds/track_oval.sdf" > /tmp/gz_pidlap.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /model/linefollower_bot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  > /tmp/gz_pidlap_bridge.log 2>&1 &
BRPID=$!
python3 "$PKG/scripts/line_detector.py" > /tmp/line_detector.log 2>&1 &
LDPID=$!
sleep 8
ros2 run rqt_image_view rqt_image_view /camera > /tmp/rqt_camera.log 2>&1 &
RQT1PID=$!
ros2 run rqt_image_view rqt_image_view /line_debug > /tmp/rqt_debug.log 2>&1 &
RQT2PID=$!
python3 "$PKG/scripts/pid_controller.py" --ros-args \
  -p kp:="$KP" -p kd:="$KD" -p ki:="$KI" -p speed:="$SPEED" \
  > /tmp/pid_node.log 2>&1 &
PIDPID=$!
trap 'kill $PIDPID $RQT1PID $RQT2PID $LDPID $BRPID $GZPID 2>/dev/null' EXIT

echo ""
echo "=== PID baseline on the oval ==="
echo "=== kp=$KP  kd=$KD  ki=$KI  speed=$SPEED ==="
echo "=== Ctrl-C stops everything; rerun with: bash scripts/pid_lap.sh KP KD KI SPEED ==="
echo ""
python3 "$PKG/scripts/lap_metrics.py"
