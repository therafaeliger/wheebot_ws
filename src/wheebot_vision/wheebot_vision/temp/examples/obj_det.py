#!/usr/bin/python3

import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.coordinates = []
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, show=False)

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy()

                for i, (x1, y1, x2, y2) in enumerate(boxes.astype(int)):
                    confidence = confidences[i]
                    class_id = int(class_ids[i])
                    label = f"{self.model.names[class_id]} | {confidence:.2f}"

                    # Draw bbox
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Center point
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    self.coordinates.append((cx, cy))

            cv2.imshow("Object Detection", frame)
            cv2.waitKey(1)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"Processed {self.frame_count} frames")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
