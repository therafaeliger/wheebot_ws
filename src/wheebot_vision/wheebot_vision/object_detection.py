#!/usr/bin/env python3

import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # === Parameters ===
        self.declare_parameter('yolo_model', 'yolo11s-seg.pt')
        yolo_model = self.get_parameter('yolo_model').value

        # === Load Model ===
        self.model = YOLO(yolo_model)
        self.bridge = CvBridge()

        # === ROS I/O ===
        self.img_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10
        )
        self.result_pub = self.create_publisher(
            Image, '/yolo/image_result', 10
        )

        self.colors = {
            i: tuple(np.random.randint(0, 255, 3).tolist()) for i in self.model.names
        }

        # FPS tracking
        self.prev_time = time.time()
        self.fps = 0.0

        self.get_logger().info(f"YOLO Segmentation Node started with model: {yolo_model}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
        
        results = self.model(frame, verbose=False)

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        if len(results) == 0:
            self.detection_pub.publish(det_msg)
            return
        
        result = results[0]
        boxes = result.boxes
        masks = result.masks
        annotated_frame = frame.copy()

        if masks is not None:
            for i, mask in enumerate(masks.data.cpu().numpy()):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                label = self.model.names[cls_id]

                # === Mask Processing ===
                mask = (mask * 255).astype(np.uint8)
                mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                color = self.colors.get(cls_id, (0, 255, 0))

                # Overlay mask transparan
                colored_mask = np.zeros_like(frame, dtype=np.uint8)
                colored_mask[mask > 128] = color
                annotated_frame = cv2.addWeighted(annotated_frame, 1.0, colored_mask, 0.4, 0)

                # Bounding box
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, f"{label} {conf:.2f}",
                            (x1, max(15, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # === Create Detection2D message ===
                det = Detection2D()
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = conf
                det.results.append(hyp)

                det_msg.detections.append(det)

        # ==== FPS Calculation ====
        now = time.time()
        dt = now - self.prev_time
        if dt > 0:
            self.fps = 1.0 / dt
        self.prev_time = now
        cv2.putText(annotated_frame, f"FPS: {self.fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # === Publish messages ===
        self.detection_pub.publish(det_msg)

        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        img_msg.header = msg.header
        self.result_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()