#!/usr/bin/env python3
"""Save one frame from an Image topic (rgb8) as a PPM file, and print a
coarse ASCII rendering so the frame can be checked in a terminal.
Usage: save_camera_frame.py [out.ppm] [topic]   (defaults: /tmp/frame.ppm /camera)"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameSaver(Node):
    def __init__(self, out_path, topic):
        super().__init__('frame_saver')
        self.out_path = out_path
        self.topic = topic
        self.done = False
        self.create_subscription(Image, topic, self.callback, 10)

    def callback(self, msg):
        if self.done:
            return
        self.done = True
        data = bytes(msg.data)
        with open(self.out_path, 'wb') as f:
            f.write(b'P6\n%d %d\n255\n' % (msg.width, msg.height))
            f.write(data)
        print(f'saved {msg.width}x{msg.height} {msg.encoding} -> {self.out_path}')

        # ASCII preview: one char per pixel, dark pixels rendered as '#'
        for row in range(msg.height):
            line = ''
            for col in range(msg.width):
                i = row * msg.step + col * 3
                lum = (data[i] + data[i + 1] + data[i + 2]) / 3
                line += '#' if lum < 80 else ('+' if lum < 160 else '.')
            print(line)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame.ppm'
    topic = sys.argv[2] if len(sys.argv) > 2 else '/camera'
    rclpy.init()
    node = FrameSaver(out, topic)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.done:
            break
    rclpy.shutdown()
    if not node.done:
        print(f'ERROR: no frame received on {topic}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
