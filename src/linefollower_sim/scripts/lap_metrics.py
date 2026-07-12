#!/usr/bin/env python3
"""Phase 2 step 3 helper: the lap timer + line-loss counter, printed once a
second so it can sit in frame during recordings ("numbers to beat" evidence).

Reads odometry (bridged from /model/linefollower_bot/odometry) and
/line_visible. Counts a "loss" on every visible->lost transition. A lap is
complete when the robot has traveled at least MIN_LAP_DIST and is back
within LAP_RADIUS of where it started.

All times are SIM time (odometry header stamps): the sim runs below real
time on this laptop, so wall-clock lap times would be inflated and unfair
to compare against the RL policy later.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

ODOM_TOPIC = '/model/linefollower_bot/odometry'
MIN_LAP_DIST = 5.0   # m; oval centerline is ~6.35 m, so 5.0 rules out jitter
LAP_RADIUS = 0.35    # m from the start point to count as "back home"


class LapMetrics(Node):
    def __init__(self):
        super().__init__('lap_metrics')
        self.start = None        # (x, y, sim_t) at lap start
        self.prev = None         # (x, y) previous sample
        self.dist = 0.0          # distance this lap
        self.losses = 0          # line losses this lap
        self.total_losses = 0
        self.lap_no = 0
        self.visible = None
        self.last_print = -1.0
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(Bool, '/line_visible', self.on_visible, 10)
        print('lap timer running (sim time) — waiting for odometry...',
              flush=True)

    def on_visible(self, msg):
        if self.visible is True and not msg.data:
            self.losses += 1
            self.total_losses += 1
            print('  ! line lost', flush=True)
        self.visible = msg.data

    def on_odom(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start is None:
            self.start = (p.x, p.y, t)
            self.prev = (p.x, p.y)
            return
        self.dist += ((p.x - self.prev[0]) ** 2
                      + (p.y - self.prev[1]) ** 2) ** 0.5
        self.prev = (p.x, p.y)

        elapsed = t - self.start[2]
        if elapsed - self.last_print >= 1.0:
            self.last_print = elapsed
            print(f'lap {self.lap_no + 1} | t = {elapsed:5.1f} s | '
                  f'dist = {self.dist:4.1f} m | losses = {self.losses}',
                  flush=True)

        home = ((p.x - self.start[0]) ** 2
                + (p.y - self.start[1]) ** 2) ** 0.5
        if self.dist >= MIN_LAP_DIST and home <= LAP_RADIUS:
            self.lap_no += 1
            print(f'=== LAP {self.lap_no} COMPLETE: {elapsed:.1f} s (sim), '
                  f'{self.dist:.1f} m, {self.losses} line losses ===',
                  flush=True)
            self.start = (p.x, p.y, t)
            self.dist = 0.0
            self.losses = 0
            self.last_print = -1.0


def main():
    rclpy.init()
    node = LapMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\ntotal: {node.lap_no} lap(s), '
              f'{node.total_losses} line losses overall', flush=True)
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
