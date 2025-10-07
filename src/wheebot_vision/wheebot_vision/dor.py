#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np


# Daftar objek dinamis (COCO dataset)
DYNAMIC_CLASSES = [
    "person", "bicycle", "car", "motorbike", "bus", "truck",
    "dog", "cat", "horse", "sheep", "cow", "elephant", "bear",
    "zebra", "giraffe"
]


class DynamicRemovalNode(Node):
    def __init__(self):
        super().__init__('dynamic_removal_node')
        self.bridge = CvBridge()

        # Subscriptions
        self.img_sub = self.create_subscription(
            # Image, '/camera/image_raw', self.image_cb, 10)
            Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        self.det_sub = self.create_subscription(
            Detection2DArray, '/yolo/classified_detections', self.dets_cb, 10)

        # Publisher: image with dynamic objects masked out
        self.pub_img = self.create_publisher(Image, '/image/dynamic_removed', 10)

        self.latest_img = None
        self.last_dets = []

        self.declare_parameter('show_debug', True)
        self.show_debug = bool(self.get_parameter('show_debug').value)

        self.get_logger().info("DynamicRemovalNode started.")

    def dets_cb(self, msg: Detection2DArray):
        dets = []
        for d in msg.detections:
            if len(d.results) == 0:
                continue

            cls_name = str(d.results[0].hypothesis.class_id)  # bisa angka/string
            x = int(d.bbox.center.position.x)
            y = int(d.bbox.center.position.y)
            w = int(d.bbox.size_x)
            h = int(d.bbox.size_y)

            x1 = max(0, x - w // 2)
            y1 = max(0, y - h // 2)
            x2 = x1 + w
            y2 = y1 + h

            dets.append({'bbox': (x1, y1, x2, y2), 'class': cls_name})

        self.last_dets = dets
        self.get_logger().info(f"Received {len(dets)} detections")

        for d in dets:
            self.get_logger().info(f" Detected: {d['class']} at {d['bbox']}")

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        mask = np.zeros(cv_img.shape[:2], dtype=np.uint8)

        # Filter hanya objek dinamis
        for det in self.last_dets:
            cls_name = det['class'].lower()
            if ":" in cls_name:
                cls_name = cls_name.split(":")[1]  # ambil nama asli setelah prefix

            if cls_name in DYNAMIC_CLASSES:
                x1, y1, x2, y2 = det['bbox']
                cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)


        # Apply mask: area dynamic diganti hitam
        removed_img = cv_img.copy()
        removed_img[mask > 0] = (0, 0, 0)

        # Publish hasilnya
        img_msg = self.bridge.cv2_to_imgmsg(removed_img, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)

        # Debug visualization
        if self.show_debug:
            debug_img = removed_img.copy()
            for det in self.last_dets:
                cls_name = det['class'].lower()
                # buang prefix DYNAMIC:/STATIC:
                if ":" in cls_name:
                    cls_name = cls_name.split(":")[1]

                if cls_name in DYNAMIC_CLASSES:
                    x1, y1, x2, y2 = det['bbox']
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(debug_img, cls_name, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.imshow("Dynamic Removal Debug", debug_img)
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = DynamicRemovalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
