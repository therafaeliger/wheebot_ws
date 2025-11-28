import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket, struct, cv2, numpy as np


HOST = "0.0.0.0"
PORT = 5555


class ImageStreamServer(Node):
    def __init__(self):
        super().__init__("image_stream_server")

        self.bridge = CvBridge()

        # --- TCP SERVER ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((HOST, PORT))
        self.sock.listen(1)

        self.get_logger().info(f"[SERVER] Menunggu koneksi TCP di port {PORT}...")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"[SERVER] Terhubung dengan {addr}")

        # Subscribe color & depth (RAW atau COMPRESSED bebas)
        self.sub_color = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.color_callback,
            10
        )

        self.sub_depth = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10
        )

    def send_packet(self, topic: bytes, data: bytes):
        header = struct.pack("!4sI", topic, len(data))
        self.conn.sendall(header + data)

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.send_packet(b"COLR", jpg.tobytes())

    def depth_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        ok, png = cv2.imencode(".png", frame)   # 16UC1 → PNG lossless
        if ok:
            self.send_packet(b"DEPT", png.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
