#!/usr/bin/env python3

import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from ultralytics import YOLO

from concurrent.futures import ThreadPoolExecutor


class ObjectDetectionAndClassification(Node):
    def __init__(self):
        super().__init__('object_detection_and_classification')

        # declare parameters
        self.declare_parameter('yolo_model', 'yolo11n-seg.pt')
        self.declare_parameter('dynamic_classes', ['person','car','truck','motorbike','bicycle','bus','dog','cat'])
        self.declare_parameter('static_classes', ['chair','table','sofa','monitor','tv','bed','refrigerator'])
        self.declare_parameter('mask_dilate', 2)
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('show_debug', True)

        model_path = self.get_parameter('yolo_model').value
        self.dynamic_classes = set(self.get_parameter('dynamic_classes').value)
        self.static_classes = set(self.get_parameter('static_classes').value)
        self.mask_dilate = int(self.get_parameter('mask_dilate').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)

        # declare object detection model
        self.model = YOLO(model_path)
        try:
            self.model.fuse()
        except Exception:
            pass
        self.bridge = CvBridge()

        # QoS Profile
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.cameraInfoCallback, 10
        )
        self.image_worker = ThreadPoolExecutor(max_workers=1)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            # self.imageCallback,
            lambda msg: self.image_worker.submit(self.imageCallback, msg),
            qos_profile=sensor_qos
        )

        # publisher
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10
        )
        self.static_pub = self.create_publisher(
            Image, '/yolo/mask/static_only', 10
        )
        self.dynamic_pub = self.create_publisher(
            Image, '/yolo/mask/dynamic_only', 10
        )
        self.viz_pub = self.create_publisher(
            Image, '/yolo/viz', 10
        )
        
        # cache
        self.camera_frame = 'camera_color_optical_frame'
        
        self.get_logger().info(f"Object Detection and Classification Node started with model: {model_path}")

    def cameraInfoCallback(self, msg: CameraInfo):
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
    
    def imageCallback(self, msg: Image):
        t0 = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # Run YOLO
        results = self.model(frame, verbose=False, conf=self.conf_thres)
        res = results[0]
        boxes = res.boxes
        masks = res.masks
        names = self.model.names

        dynamic_mask = np.zeros((h, w), dtype=np.uint8)
        static_mask = np.zeros((h, w), dtype=np.uint8)
        viz = frame.copy()

        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.header.frame_id = self.camera_frame or msg.header.frame_id

        if masks is not None and boxes is not None and len(masks.data) == len(boxes):
            masks_np = masks.data.cpu().numpy()
            for i in range(len(masks_np)):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                label = names.get(cls_id, str(cls_id))

                mask_u8 = (cv2.resize((masks_np[i] * 255).astype(np.uint8), (w, h)) > 128).astype(np.uint8)*255

                # Dynamic vs static grouping
                if label in self.dynamic_classes:
                    dynamic_mask = cv2.bitwise_or(dynamic_mask, mask_u8)
                    color = (0,0,255)
                elif label in self.static_classes:
                    static_mask = cv2.bitwise_or(static_mask, mask_u8)
                    color = (0,255,0)
                else:
                    color = (255,255,0)

                # Draw bbox + label
                x1,y1,x2,y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                cv2.rectangle(viz, (x1,y1), (x2,y2), color, 2)
                cv2.putText(viz, f"{label} {conf:.2f}", (x1, max(15, y1-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Fill mask overlay for viz
                overlay = np.zeros_like(viz)
                overlay[mask_u8>0] = color
                viz = cv2.addWeighted(viz, 1.0, overlay, 0.3, 0)

                # vision_msgs Detection2D
                det = Detection2D()
                det.bbox.center.position.x = float((x1+x2)/2.0)
                det.bbox.center.position.y = float((y1+y2)/2.0)
                det.bbox.size_x = float(x2-x1)
                det.bbox.size_y = float(y2-y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

            # Clean up masks (morphology)
            if self.mask_dilate > 0:
                k = np.ones((self.mask_dilate, self.mask_dilate), np.uint8)
                dynamic_mask = cv2.dilate(dynamic_mask, k, 1)
                static_mask = cv2.dilate(static_mask, k, 1)

            # Publish
            dyn_msg = self.bridge.cv2_to_imgmsg(dynamic_mask, encoding='mono8')
            dyn_msg.header = msg.header
            dyn_msg.header.frame_id = self.camera_frame or msg.header.frame_id
            self.dynamic_pub.publish(dyn_msg)

            stat_msg = self.bridge.cv2_to_imgmsg(static_mask, encoding='mono8')
            stat_msg.header = msg.header
            stat_msg.header.frame_id = self.camera_frame or msg.header.frame_id
            self.static_pub.publish(stat_msg)

            viz_msg = self.bridge.cv2_to_imgmsg(viz, encoding='bgr8')
            viz_msg.header = msg.header
            viz_msg.header.frame_id = self.camera_frame or msg.header.frame_id
            self.viz_pub.publish(viz_msg)

            det_array.header.stamp = msg.header.stamp
            self.detection_pub.publish(det_array)
        
        else: # kondisi no detection
            dyn_msg = self.bridge.cv2_to_imgmsg(np.zeros((h, w), dtype=np.uint8), encoding='mono8')
            dyn_msg.header = msg.header
            dyn_msg.header.frame_id = self.camera_frame or msg.header.frame_id
            self.dynamic_pub.publish(dyn_msg)

            stat_msg = self.bridge.cv2_to_imgmsg(np.zeros((h, w), dtype=np.uint8), encoding='mono8')
            stat_msg.header = msg.header
            stat_msg.header.frame_id = self.camera_frame or msg.header.frame_id
            self.static_pub.publish(stat_msg)

            viz_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            viz_msg.header = msg.header
            viz_msg.header.frame_id = self.camera_frame or msg.header.frame_id
            self.viz_pub.publish(viz_msg)

            self.detection_pub.publish(det_array)

        if self.show_debug:
            fps = 1.0 / max(1e-3, (time.time()-t0))
            self.get_logger().info(f"FPS: {fps:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionAndClassification()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()