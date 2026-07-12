#!/usr/bin/env python3
"""Phase 2 step 1: turn each /camera frame (sensor_msgs/Image, rgb8, 64x64)
into a single line-position number for the PID controller.

Publishes:
  /line_position (std_msgs/Float32)  where the line is in a band of rows near
      the bottom of the frame (closest to the wheels): -1 = far left,
      0 = center, +1 = far right. When the line is lost, the last known
      value is re-published so a controller always has something to act on.
  /line_visible (std_msgs/Bool)      whether the line is in the band right now.
  /line_debug (sensor_msgs/Image)    thresholded "what PID sees" view:
      line pixels white, band rows highlighted, detected position drawn
      as a red column. View with rqt_image_view /line_debug.

A pixel counts as line when its luminance is below DARK_LUM (same cutoff as
the Phase 1 done-checks). Pure Python on purpose: 64x64 at 30 Hz is tiny.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

DARK_LUM = 80
BAND_TOP = 42          # band = rows 42..63, the road just ahead of the wheels
MIN_LINE_PIXELS = 5    # fewer dark pixels than this = line lost, not noise


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.last_position = 0.0
        self.was_visible = None
        self.pub_pos = self.create_publisher(Float32, '/line_position', 10)
        self.pub_vis = self.create_publisher(Bool, '/line_visible', 10)
        self.pub_dbg = self.create_publisher(Image, '/line_debug', 10)
        self.create_subscription(Image, '/camera', self.callback, 10)

    def callback(self, msg):
        data = bytes(msg.data)
        w, h = msg.width, msg.height

        def dark(r, c):
            i = r * msg.step + c * 3
            return (data[i] + data[i + 1] + data[i + 2]) / 3 < DARK_LUM

        cols = [c for r in range(BAND_TOP, h) for c in range(w) if dark(r, c)]
        visible = len(cols) >= MIN_LINE_PIXELS
        if visible:
            mean_col = sum(cols) / len(cols)
            center = (w - 1) / 2
            self.last_position = (mean_col - center) / center

        self.pub_pos.publish(Float32(data=self.last_position))
        self.pub_vis.publish(Bool(data=visible))
        if visible is not self.was_visible:
            self.was_visible = visible
            self.get_logger().info(
                f'line {"found" if visible else "LOST"} '
                f'(position {self.last_position:+.2f})')

        self.pub_dbg.publish(self.debug_image(msg, dark, visible))

    def debug_image(self, msg, dark, visible):
        w, h = msg.width, msg.height
        out = bytearray(w * h * 3)
        for r in range(h):
            in_band = r >= BAND_TOP
            for c in range(w):
                i = (r * w + c) * 3
                if dark(r, c):
                    out[i:i + 3] = (0, 255, 0) if in_band else (255, 255, 255)
                elif in_band:
                    out[i:i + 3] = (40, 40, 40)
        if visible:
            center = (w - 1) / 2
            pos_col = round(self.last_position * center + center)
            for r in range(BAND_TOP, h):
                i = (r * w + pos_col) * 3
                out[i:i + 3] = (255, 0, 0)
        dbg = Image()
        dbg.header = msg.header
        dbg.height, dbg.width = h, w
        dbg.encoding = 'rgb8'
        dbg.step = w * 3
        dbg.data = bytes(out)
        return dbg


def main():
    rclpy.init()
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
