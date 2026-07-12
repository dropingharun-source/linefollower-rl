#!/usr/bin/env bash
# Phase 1 step 3 done-check: the track line is visible in the ROBOT'S camera
# (not just the editor view). Launches track_oval.sdf headless, saves one
# /camera frame, and fails unless the frame contains a meaningful number of
# dark (line) pixels. Honors LIBGL_ALWAYS_SOFTWARE from the caller.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
source /opt/ros/humble/setup.bash

FRAME="${1:-/tmp/track_frame.ppm}"

ign gazebo -s -r -v3 "$PKG/worlds/track_oval.sdf" > /tmp/gz_server.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image > /tmp/gz_bridge.log 2>&1 &
BRPID=$!
trap 'kill $GZPID $BRPID 2>/dev/null' EXIT
sleep 10

echo "--- saving one frame from /camera ---"
python3 "$PKG/scripts/save_camera_frame.py" "$FRAME"

echo "--- checking the line is in the frame ---"
# The robot spawns ON the line facing along it, so the line must run up
# the middle of the view: require dark pixels in the center band of most
# rows. (A plain "any dark pixels" count passes on the robot's own
# shadow — that false pass happened 2026-07-12.)
python3 - "$FRAME" <<'EOF'
import sys
with open(sys.argv[1], 'rb') as f:
    assert f.readline().strip() == b'P6'
    w, h = map(int, f.readline().split())
    f.readline()
    data = f.read()

def dark(r, c):
    i = (r * w + c) * 3
    return (data[i] + data[i + 1] + data[i + 2]) / 3 < 80

rows_with_line = sum(
    1 for r in range(20, 60)
    if any(dark(r, c) for c in range(16, 48)))
print(f'rows 20-59 with dark pixels in center band (cols 16-47): '
      f'{rows_with_line}/40')
if rows_with_line < 25:
    print('FAIL: track line not visible in the camera frame')
    sys.exit(1)
print('OK: line visible')
EOF
