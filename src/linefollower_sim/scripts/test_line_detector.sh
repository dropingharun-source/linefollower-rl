#!/usr/bin/env bash
# Phase 2 step 1 done-check: line_detector.py turns camera frames into a
# line-position number. Verifies on the oval benchmark:
#   (a) robot spawns ON the line -> line visible, position near 0
#   (b) rotate the robot left (CCW) -> the line shifts RIGHT in the image
#       -> position goes clearly positive (also proves the sign convention)
# Saves one /line_debug frame so the thresholded view can be eyeballed.
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

pkill -f '[i]gn gazebo' 2>/dev/null && sleep 2 || true
pkill -f '[p]arameter_bridge' 2>/dev/null && sleep 1 || true

ign gazebo -s -r -v3 "$PKG/worlds/track_oval.sdf" > /tmp/gz_server.log 2>&1 &
GZPID=$!
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist > /tmp/gz_bridge.log 2>&1 &
BRPID=$!
python3 "$PKG/scripts/line_detector.py" > /tmp/line_detector.log 2>&1 &
LDPID=$!
trap 'kill $GZPID $BRPID $LDPID 2>/dev/null' EXIT
sleep 10

python3 - <<'EOF'
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class Checker(Node):
    def __init__(self):
        super().__init__('line_detector_check')
        self.positions, self.visibles = [], []
        self.create_subscription(Float32, '/line_position',
                                 lambda m: self.positions.append(m.data), 10)
        self.create_subscription(Bool, '/line_visible',
                                 lambda m: self.visibles.append(m.data), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def collect(self, seconds):
        self.positions.clear()
        self.visibles.clear()
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.positions), list(self.visibles)

    def spin_in_place(self, wz, seconds):
        msg = Twist()
        msg.angular.z = wz
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.pub.publish(Twist())  # stop


rclpy.init()
node = Checker()
fails = []

print('--- (a) at spawn: on the line, centered ---')
pos, vis = node.collect(3.0)
if len(pos) < 30:
    print(f'FAIL: only {len(pos)} /line_position messages in 3 s')
    sys.exit(1)
vis_ratio = sum(vis) / len(vis)
mean0 = sum(pos) / len(pos)
print(f'{len(pos)} samples, visible {vis_ratio:.0%}, mean position {mean0:+.3f}')
if vis_ratio < 0.9:
    fails.append('line not visible at spawn')
if abs(mean0) > 0.25:
    fails.append(f'position at spawn {mean0:+.3f}, expected near 0')

print('--- (b) rotate left (CCW): line should shift right, position > +0.15 ---')
node.spin_in_place(0.5, 1.2)
pos, vis = node.collect(1.5)
mean1 = sum(pos) / len(pos)
print(f'{len(pos)} samples, visible {sum(vis)}/{len(vis)}, '
      f'mean position {mean1:+.3f}')
if mean1 < 0.15:
    fails.append(f'after CCW rotation position {mean1:+.3f}, expected > +0.15')

rclpy.shutdown()
if fails:
    for f in fails:
        print('FAIL:', f)
    sys.exit(1)
print('OK: line detector reports position correctly')
EOF

echo "--- saving one /line_debug frame (the 'what PID sees' view) ---"
python3 "$PKG/scripts/save_camera_frame.py" /tmp/line_debug.ppm /line_debug
