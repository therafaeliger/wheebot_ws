#!/usr/bin/python3

import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class ObjectSegmentation(Node):
    def __init__(self):
        super().__init__('object_segmentation')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolo11s-seg.pt')
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        cv2.namedWindow("YOLO Segmentation", cv2.WINDOW_NORMAL)
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, show=False)

            for result in results:
                masks = result.masks
                boxes = result.boxes

                if masks is not None:
                    for i, mask in enumerate(masks.data.cpu().numpy()):
                        class_id = int(boxes.cls[i].item())
                        conf = float(boxes.conf[i].item())
                        label = f"{self.model.names[class_id]} {conf:.2f}"

                        # Konversi mask ke uint8
                        mask = (mask * 255).astype(np.uint8)
                        mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))

                        # Warna unik berdasarkan class_id
                        np.random.seed(class_id)  # konsisten antar frame
                        color = tuple(np.random.randint(0, 255, 3).tolist())

                        # Buat overlay transparan
                        colored_mask = np.zeros_like(frame, dtype=np.uint8)
                        colored_mask[mask > 128] = color
                        frame = cv2.addWeighted(frame, 1.0, colored_mask, 0.5, 0)

                        # Ambil bounding box untuk posisi label
                        x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                        # Background kotak teks biar jelas
                        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(frame, (x1, y1 - th - 5), (x1 + tw, y1), color, -1)
                        cv2.putText(frame, label, (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("YOLO Segmentation", frame)
            cv2.waitKey(1)

            # Logging tiap 30 frame
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"Processed {self.frame_count} frames")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
