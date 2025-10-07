#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        self.pub = self.create_publisher(LaserScan, '/scan_for_slam', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

        # configure: ignore sector in front of robot (example)
        # angles are in radians relative to scan.angle_min
        self.angle_ignore_min = -0.5  # -45 deg
        self.angle_ignore_max =  0.5  # +45 deg
        self.min_range_ignore = 0.15  # meters (close self-returns)

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities) if msg.intensities else []

        n = len(out.ranges)
        for i in range(n):
            angle = out.angle_min + i * out.angle_increment
            # if angle within ignored sector OR too close -> set to inf (no return)
            if self.angle_ignore_min <= angle <= self.angle_ignore_max or (out.ranges[i] > 0 and out.ranges[i] < self.min_range_ignore):
                out.ranges[i] = float('inf')  # or out.range_max + 1.0

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()