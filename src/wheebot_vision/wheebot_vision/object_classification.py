#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ObjectClassifier(Node):
    def __init__(self):
        super().__init__('object_classifier')

        # === Parameter ===
        self.declare_parameter('dynamic_classes', [
            'person', 'car', 'truck', 'motorbike', 'bicycle', 'bus', 'dog', 'cat'
        ])
        self.declare_parameter('static_classes', [
            'chair', 'table', 'sofa', 'monitor', 'tv', 'bed', 'refrigerator'
        ])

        self.dynamic_classes = set(self.get_parameter('dynamic_classes').value)
        self.static_classes = set(self.get_parameter('static_classes').value)

        # === ROS I/O ===
        self.det_sub = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detection_callback, 10
        )

        self.static_pub = self.create_publisher(
            Detection2DArray, '/objects/static', 10
        )
        self.dynamic_pub = self.create_publisher(
            Detection2DArray, '/objects/dynamic', 10
        )

        self.get_logger().info("Object Classifier Node started — separating static and dynamic objects.")

    def detection_callback(self, msg: Detection2DArray):
        static_msg = Detection2DArray()
        static_msg.header = msg.header

        dynamic_msg = Detection2DArray()
        dynamic_msg.header = msg.header

        # Pisahkan berdasarkan class label
        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            if label in self.dynamic_classes:
                dynamic_msg.detections.append(det)
            elif label in self.static_classes:
                static_msg.detections.append(det)
            else:
                # Abaikan kelas yang tidak diketahui
                continue

        # Publish hasil
        self.static_pub.publish(static_msg)
        self.dynamic_pub.publish(dynamic_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
