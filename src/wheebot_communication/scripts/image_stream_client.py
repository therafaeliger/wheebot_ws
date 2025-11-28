import socket
import struct
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageStreamClient(Node):
    def __init__(self):
        super().__init__("image_stream_client")

        self.bridge = CvBridge()

        # ROS2 output topics
        self.pub_color = self.create_publisher(Image, "/stream/color", 10)
        self.pub_depth = self.create_publisher(Image, "/stream/depth", 10)

        # TCP client
        HOST = "10.134.229.83"
        PORT = 5555

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.get_logger().info(f"[CLIENT] Terhubung ke {HOST}:{PORT}")

        # Loop timer
        self.timer = self.create_timer(0.0001, self.loop)

    def recvall(self, size):
        buf = b''
        while len(buf) < size:
            chunk = self.sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def loop(self):
        header = self.recvall(8)
        if header is None:
            return
        
        topic, length = struct.unpack("!4sI", header)
        data = self.recvall(length)
        if data is None:
            return

        if topic == b"COLR":
            arr = np.frombuffer(data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.pub_color.publish(msg)

        elif topic == b"DEPT":
            arr = np.frombuffer(data, np.uint8)
            depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
            msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
            self.pub_depth.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
