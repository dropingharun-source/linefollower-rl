#!/usr/bin/env python3
"""Phase 2 step 2: PID controller — the classical baseline the RL policy
must beat. Reads /line_position (-1 left .. +1 right, from line_detector.py)
and drives the robot to keep the line centered.

PID in one line each:
  P: steer harder the further the line is from center
  I: accumulate lingering one-sided error and trim it out
  D: react to the line MOVING away, before it gets far (damps wobble)

Publishes /cmd_vel. When the line is lost (/line_visible False) the robot
slows to lost_speed and keeps steering toward the detector's last known
position until the line is back.

Gains are ROS parameters so tuning is one flag, no code edits:
  python3 pid_controller.py --ros-args -p kp:=1.0 -p kd:=0.2 -p speed:=0.15

dt is fixed at the camera period (1/30 s of SIM time per frame): wall-clock
dt would drift with the real-time factor, which is well below 1.0 on this
laptop under software rendering.
"""
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

DT = 1.0 / 30.0          # camera frame period in sim seconds
INTEGRAL_LIMIT = 0.5     # anti-windup: |integral| never exceeds this


class PidController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('speed', 0.15)       # cruise, m/s
        self.declare_parameter('lost_speed', 0.05)  # crawl while line lost
        self.declare_parameter('max_turn', 1.5)     # rad/s clamp

        self.integral = 0.0
        self.prev_error = None
        self.visible = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/line_visible', self.on_visible, 10)
        self.create_subscription(Float32, '/line_position', self.on_position, 10)

        g = {n: self.get_parameter(n).value
             for n in ('kp', 'ki', 'kd', 'speed', 'lost_speed', 'max_turn')}
        self.get_logger().info(f'gains: {g}')

    def on_visible(self, msg):
        self.visible = msg.data

    def on_position(self, msg):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        error = msg.data  # target is 0 = line centered
        self.integral = max(-INTEGRAL_LIMIT,
                            min(INTEGRAL_LIMIT, self.integral + error * DT))
        derivative = 0.0 if self.prev_error is None \
            else (error - self.prev_error) / DT
        self.prev_error = error

        # positive error = line right of center = turn right = negative z
        turn = -(kp * error + ki * self.integral + kd * derivative)
        max_turn = self.get_parameter('max_turn').value
        turn = max(-max_turn, min(max_turn, turn))

        cmd = Twist()
        cmd.linear.x = self.get_parameter(
            'speed' if self.visible else 'lost_speed').value
        cmd.angular.z = turn
        self.pub.publish(cmd)

    def stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = PidController()
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # zero cmd_vel so the robot doesn't coast on forever
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
