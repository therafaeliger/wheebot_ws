#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class DynamicObjectRemoval(Node):
    def __init__(self):
        super().__init__('dynamic_object_removal')

        self.bridge = CvBridge()

        # === Parameter ===
        self.declare_parameter('inpaint_radius', 3)
        self.inpaint_radius = int(self.get_parameter('inpaint_radius').value)

        # === Subscriptions ===
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.dynamic_sub = self.create_subscription(
            Detection2DArray, '/objects/dynamic', self.dynamic_callback, 10
        )
        self.static_sub = self.create_subscription(
            Detection2DArray, '/objects/static', self.static_callback, 10
        )

        # === Publishers ===
        self.pub_dynamic_removed = self.create_publisher(Image, '/dor/dynamic_removed', 10)
        self.pub_inpainted = self.create_publisher(Image, '/dor/inpainted', 10)
        self.pub_dynamic_only = self.create_publisher(Image, '/dor/dynamic_only', 10)
        self.pub_static_only = self.create_publisher(Image, '/dor/static_only', 10)

        # === Internal storage ===
        self.current_image = None
        self.dynamic_dets = []
        self.static_dets = []

        self.get_logger().info("Dynamic Object Removal node started successfully.")

    # -------------------------------
    # Callback untuk topik kamera
    # -------------------------------
    def image_callback(self, msg: Image):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # if self.dynamic_dets:
        self.process_image(msg.header)

    # -------------------------------
    # Callback untuk deteksi dinamis
    # -------------------------------
    def dynamic_callback(self, msg: Detection2DArray):
        self.dynamic_dets = msg.detections

    # -------------------------------
    # Callback untuk deteksi statis
    # -------------------------------
    def static_callback(self, msg: Detection2DArray):
        self.static_dets = msg.detections

    # -------------------------------
    # Helper aman untuk ambil center bounding box
    # -------------------------------
    def get_bbox_center(self, bbox):
        try:
            return bbox.center.x, bbox.center.y
        except AttributeError:
            try:
                return bbox.center.position.x, bbox.center.position.y
            except AttributeError:
                return 0.0, 0.0

    # -------------------------------
    # Proses utama (masking + inpainting)
    # -------------------------------
    def process_image(self, header):
        if self.current_image is None:
            return

        frame = self.current_image.copy()
        h, w, _ = frame.shape

        mask_dynamic = np.zeros((h, w), dtype=np.uint8)
        mask_static = np.zeros((h, w), dtype=np.uint8)

        # --- Buat bounding box dynamic ---
        if self.dynamic_dets:
            for det in self.dynamic_dets:
                bbox = det.bbox
                cx, cy = self.get_bbox_center(bbox)
                bw, bh = int(bbox.size_x), int(bbox.size_y)
                x, y = int(cx - bw / 2), int(cy - bh / 2)
                cv2.rectangle(mask_dynamic, (x, y), (x + bw, y + bh), 255, -1)

        # --- Buat bounding box static ---
        if self.static_dets:
            for det in self.static_dets:
                bbox = det.bbox
                cx, cy = self.get_bbox_center(bbox)
                bw, bh = int(bbox.size_x), int(bbox.size_y)
                x, y = int(cx - bw / 2), int(cy - bh / 2)
                cv2.rectangle(mask_static, (x, y), (x + bw, y + bh), 255, -1)

        # === Dynamic only ===
        dynamic_only = cv2.bitwise_and(frame, frame, mask=mask_dynamic)

        # === Static only ===
        static_only = cv2.bitwise_and(frame, frame, mask=mask_static)

        # === Dynamic removed ===
        dynamic_removed = frame.copy()
        dynamic_removed[mask_dynamic == 255] = (0, 0, 0)

        # === Inpainted (isi ulang area dynamic) ===
        inpainted = cv2.inpaint(frame, mask_dynamic, self.inpaint_radius, cv2.INPAINT_TELEA)

        # === Publish ===
        self.pub_dynamic_removed.publish(self.bridge.cv2_to_imgmsg(dynamic_removed, "bgr8"))
        self.pub_inpainted.publish(self.bridge.cv2_to_imgmsg(inpainted, "bgr8"))
        self.pub_dynamic_only.publish(self.bridge.cv2_to_imgmsg(dynamic_only, "bgr8"))
        self.pub_static_only.publish(self.bridge.cv2_to_imgmsg(static_only, "bgr8"))

        self.get_logger().debug("Published /dor outputs: dynamic_removed, inpainted, dynamic_only, static_only.")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObjectRemoval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
