#!/usr/bin/env python3
"""Drive the robot with the trained PPO policy — the RL counterpart of
pid_controller.py. Camera frames in, wheel speeds out, no other input
(navigation is camera-only; this node never sees odometry or the track).

Runs the policy on every 3rd frame (10 Hz, same cadence it was trained at)
and holds the last command in between. Deterministic actions — no
exploration noise during evaluation.

Usage (inside the rl_lap.sh stack):
    python3 rl_controller.py [model.zip]     default: newest *_last.zip
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from line_env import WHEEL_MAX, WHEEL_SEP, CONTROL_EVERY

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from stable_baselines3 import PPO

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
CKPT_DIR = os.path.join(REPO, 'training', 'checkpoints')


class RLController(Node):
    def __init__(self, model_path):
        super().__init__('rl_controller')
        self.model = PPO.load(model_path, device='cpu')
        self.get_logger().info('driving with ' + model_path)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera', self.on_frame, 10)
        self.frames = 0

    def on_frame(self, msg):
        self.frames += 1
        if self.frames % CONTROL_EVERY:
            return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(64, 64, 3)
        gray = (0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1]
                + 0.114 * rgb[:, :, 2]).astype(np.uint8).reshape(64, 64, 1)
        action, _ = self.model.predict(gray, deterministic=True)
        vl = (float(action[0]) + 1.0) / 2.0 * WHEEL_MAX
        vr = (float(action[1]) + 1.0) / 2.0 * WHEEL_MAX
        t = Twist()
        t.linear.x = (vl + vr) / 2.0
        t.angular.z = (vr - vl) / WHEEL_SEP
        self.pub.publish(t)


def main():
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        zips = sorted(glob.glob(os.path.join(CKPT_DIR, '*_last.zip')),
                      key=os.path.getmtime)
        if not zips:
            sys.exit('no *_last.zip in ' + CKPT_DIR)
        model_path = zips[-1]

    rclpy.init()
    node = RLController(model_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop the robot on the way out
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
