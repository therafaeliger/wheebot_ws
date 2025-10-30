#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import csv
import datetime
import os

class FPSLogger(Node):
    def __init__(self):
        super().__init__('fps_logger')

        self.output_file = f"fps_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.get_logger().info(f"Simpan log FPS ke: {self.output_file}")

        # Buat file CSV dan tulis header
        with open(self.output_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "fps"])

        # Subscribe ke topik FPS
        self.create_subscription(Float32, '/yolo/fps', self.fps_callback, 10)

    def fps_callback(self, msg):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        with open(self.output_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, msg.data])
        self.get_logger().info(f"FPS: {msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
