#!/usr/bin/env bash
# Phase 2: watch what the PID will see, live. Launches the oval world, the
# line detector, a raw camera view AND the thresholded /line_debug view,
# plus keyboard teleop so you can drive around and watch the red position
# column follow the line.
# Usage: bash scripts/view_line.sh
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

ign gazebo -r "$PKG/worlds/track_oval.sdf" > /tmp/gz_viewline.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image > /tmp/gz_viewline_bridge.log 2>&1 &
BRPID=$!
python3 "$PKG/scripts/line_detector.py" > /tmp/line_detector.log 2>&1 &
LDPID=$!
sleep 8
ros2 run rqt_image_view rqt_image_view /camera > /tmp/rqt_camera.log 2>&1 &
RQT1PID=$!
ros2 run rqt_image_view rqt_image_view /line_debug > /tmp/rqt_debug.log 2>&1 &
RQT2PID=$!
trap 'kill $GZPID $BRPID $LDPID $RQT1PID $RQT2PID 2>/dev/null' EXIT

echo ""
echo "=== Line view on the oval ==="
echo "=== Windows: Gazebo + raw camera (/camera) + PID view (/line_debug). ==="
echo "=== In /line_debug: green = line inside the decision strip, red column ==="
echo "=== = the position number sent to PID, white = line above the strip.  ==="
echo "=== CLICK THIS TERMINAL FIRST, then drive:                            ==="
echo "=== i=forward  ,=back  j/l=rotate  u/o=curve  k=stop  q/z=faster/slower ==="
echo "=== Ctrl-C here when done (closes everything). ==="
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
