#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class InpaintingNode(Node):
    def __init__(self):
        super().__init__('inpainting_node')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('inpainting_radius', 7)
        self.inpaint_radius = int(self.get_parameter('inpainting_radius').value)

        # Subscriptions
        self.img_sub = self.create_subscription(
            Image, '/image/dynamic_removed', self.image_cb, 10)

        # Publisher
        self.pub_img = self.create_publisher(Image, '/image/static_only', 10)

        self.declare_parameter('show_debug', True)
        self.show_debug = bool(self.get_parameter('show_debug').value)

        # FPS tracking
        self.last_time = time.time()
        self.fps = 0.0

        self.get_logger().info("InpaintingNode started.")

    def image_cb(self, msg: Image):
        try:
            removed_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Mask: semua pixel hitam dianggap dynamic object
        gray = cv2.cvtColor(removed_img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, 0)

        # Inpainting
        inpainted = cv2.inpaint(removed_img, mask, self.inpaint_radius, cv2.INPAINT_TELEA)

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(inpainted, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)

        # ==== FPS Calculation ====
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.fps = 1.0 / dt
        self.last_time = now

        # Debug visualization
        if self.show_debug:
            stacked = np.hstack((removed_img, inpainted))
            cv2.putText(stacked, f"FPS: {self.fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Inpainting Debug (Left=Masked, Right=Inpainted)", stacked)
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = InpaintingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
