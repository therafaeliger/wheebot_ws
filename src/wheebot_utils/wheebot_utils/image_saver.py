#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Parameter
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('output_dir', '/home/rafael/wheebot_ws/src/wheebot_utils/results/saved_images')
        self.declare_parameter('format', 'jpg')  # bisa 'png' juga
        self.declare_parameter('save_rate', 1.0)  # detik antar simpan

        self.topic = self.get_parameter('topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.format = self.get_parameter('format').value
        self.save_rate = float(self.get_parameter('save_rate').value)

        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.last_save_time = self.get_clock().now().nanoseconds / 1e9

        # Subscription
        self.create_subscription(Image, self.topic, self.image_callback, 10)
        self.get_logger().info(
            f"📸 ImageSaver aktif — subscribe: {self.topic}, simpan ke: {self.output_dir}/, format: {self.format}"
        )

    def image_callback(self, msg: Image):
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_save_time) < self.save_rate:
            return  # batasi kecepatan simpan

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.output_dir, f"{timestamp}.{self.format}")
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"🖼️ Disimpan: {filename}")
            self.last_save_time = now
        except Exception as e:
            self.get_logger().error(f"Gagal menyimpan gambar: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
