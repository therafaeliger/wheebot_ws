import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2


class ColorDecompressor(Node):
    def __init__(self):
        super().__init__('color_decompressor')

        self.bridge = CvBridge()

        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            Image,
            '/camera/color/image_raw/decompressed',
            qos
        )

        self.get_logger().info("Color Decompressor Ready (RELIABLE).")

    def callback(self, msg):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            ros_img.header = msg.header
            self.pub.publish(ros_img)

        except Exception as e:
            self.get_logger().error(f"COLOR decode error: {e}")


def main():
    rclpy.init()
    node = ColorDecompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
