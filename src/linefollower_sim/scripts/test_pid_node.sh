#!/usr/bin/env bash
# Phase 2 step 2 plumbing check — NO SIMULATION on purpose: the PID must not
# drive before its first attempt is recorded (evidence rule). Feeds fake
# /line_position + /line_visible messages and asserts on /cmd_vel:
#   line right of center  -> turn right (angular.z < 0), cruise speed
#   line left of center   -> turn left  (angular.z > 0)
#   line centered         -> go straight (angular.z ~ 0)
#   line lost             -> slow to lost_speed
#   huge error            -> turn clamped at max_turn
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash

python3 "$PKG/scripts/pid_controller.py" > /tmp/pid_node.log 2>&1 &
PIDPID=$!
trap 'kill $PIDPID 2>/dev/null' EXIT
sleep 3

python3 - <<'EOF'
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class Feeder(Node):
    def __init__(self):
        super().__init__('pid_check')
        self.cmds = []
        self.pub_pos = self.create_publisher(Float32, '/line_position', 10)
        self.pub_vis = self.create_publisher(Bool, '/line_visible', 10)
        self.create_subscription(Twist, '/cmd_vel',
                                 lambda m: self.cmds.append(m), 10)

    def feed(self, position, visible, n=20):
        """Publish n fake detector frames, return the last cmd_vel."""
        self.cmds.clear()
        for _ in range(n):
            self.pub_vis.publish(Bool(data=visible))
            self.pub_pos.publish(Float32(data=position))
            rclpy.spin_once(self, timeout_sec=0.05)
        end = self.get_clock().now().nanoseconds + int(1e9)
        while not self.cmds and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.cmds:
            print('FAIL: no /cmd_vel received from pid_controller')
            sys.exit(1)
        return self.cmds[-1]


rclpy.init()
node = Feeder()
fails = []

# defaults: kp=1.0 ki=0 kd=0 speed=0.15 lost_speed=0.05 max_turn=1.5
cmd = node.feed(+0.5, True)
print(f'line at +0.5 (right) -> linear {cmd.linear.x:.3f}, '
      f'angular {cmd.angular.z:+.3f}')
if not (cmd.angular.z < -0.3):
    fails.append('line right should give a clear right turn (angular < 0)')
if abs(cmd.linear.x - 0.15) > 1e-6:
    fails.append(f'cruise speed {cmd.linear.x}, expected 0.15')

cmd = node.feed(-0.5, True)
print(f'line at -0.5 (left)  -> linear {cmd.linear.x:.3f}, '
      f'angular {cmd.angular.z:+.3f}')
if not (cmd.angular.z > 0.3):
    fails.append('line left should give a clear left turn (angular > 0)')

cmd = node.feed(0.0, True)
print(f'line at 0.0 (center) -> linear {cmd.linear.x:.3f}, '
      f'angular {cmd.angular.z:+.3f}')
if abs(cmd.angular.z) > 0.05:
    fails.append(f'centered line should give ~zero turn, got {cmd.angular.z}')

cmd = node.feed(+0.5, False)
print(f'line LOST at +0.5    -> linear {cmd.linear.x:.3f}, '
      f'angular {cmd.angular.z:+.3f}')
if abs(cmd.linear.x - 0.05) > 1e-6:
    fails.append(f'lost line should slow to 0.05, got {cmd.linear.x}')
if not (cmd.angular.z < 0):
    fails.append('lost line should keep steering toward last known side')

# kp=1.0 on error 5.0 wants turn 5.0 -> must clamp at max_turn 1.5
cmd = node.feed(5.0, True)
print(f'huge error +5.0      -> angular {cmd.angular.z:+.3f} (clamp test)')
if abs(abs(cmd.angular.z) - 1.5) > 1e-6:
    fails.append(f'turn should clamp at 1.5, got {cmd.angular.z}')

rclpy.shutdown()
if fails:
    for f in fails:
        print('FAIL:', f)
    sys.exit(1)
print('OK: pid_controller plumbing correct (sign, speeds, clamp)')
EOF
