#!/usr/bin/env python3

import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time
import threading
from collections import deque


class DynamicObjectRemovalFullSystem(Node):
    def __init__(self):
        super().__init__('dynamic_object_removal_full_system')

        # === Parameters ===
        self.declare_parameter('yolo_model', 'yolo11s-seg.pt')
        self.declare_parameter('dynamic_classes', ['person', 'car', 'truck', 'motorbike', 'bicycle', 'bus', 'dog', 'cat'])
        self.declare_parameter('static_classes', ['chair', 'table', 'sofa', 'monitor', 'tv', 'bed', 'refrigerator'])
        self.declare_parameter('inpaint_radius', 3)
        self.declare_parameter('mask_dilate', 2)
        self.declare_parameter('show_debug', True)
        self.declare_parameter('max_queue_size', 5)

        # Load model YOLO sekali saja
        yolo_model = self.get_parameter('yolo_model').value
        self.model = YOLO(yolo_model)
        self.bridge = CvBridge()

        # Class sets untuk filtering
        self.dynamic_classes = set(self.get_parameter('dynamic_classes').value)
        self.static_classes = set(self.get_parameter('static_classes').value)
        self.inpaint_radius = self.get_parameter('inpaint_radius').value
        self.mask_dilate = self.get_parameter('mask_dilate').value
        self.show_debug = self.get_parameter('show_debug').value
        self.max_queue_size = self.get_parameter('max_queue_size').value

        # Camera parameters (akan diupdate dari camera_info)
        self.fx, self.fy, self.cx, self.cy = 615.0, 615.0, 320.0, 240.0

        # === QoS Profiles ===
        # Best effort untuk data sensor yang high-frequency
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Reliable untuk data hasil processing
        self.processed_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # === Buffer untuk sinkronisasi ===
        self.rgb_buffer = deque(maxlen=self.max_queue_size)
        self.depth_buffer = deque(maxlen=self.max_queue_size)
        self.lock = threading.Lock()
        
        self.camera_info = None
        self.camera_info_received = False

        # === ROS Subscriptions dengan QoS ===
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.rgb_callback, self.sensor_qos
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, self.sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', 
            self.camera_info_callback, self.processed_qos
        )

        # === ROS Publishers dengan QoS ===
        # Untuk SLAM (RTAB-Map) - Best Effort untuk real-time
        self.static_depth_pub = self.create_publisher(
            Image, '/dor/static_depth', self.sensor_qos
        )
        self.inpainted_pub = self.create_publisher(
            Image, '/dor/inpainted', self.sensor_qos
        )
        
        # Untuk Navigation (Nav2) - Reliable untuk safety
        self.dynamic_cloud_pub = self.create_publisher(
            PointCloud2, '/dor/dynamic_cloud', self.processed_qos
        )
        
        # Untuk Visualisasi - Best Effort
        self.dynamic_removed_pub = self.create_publisher(
            Image, '/dor/dynamic_removed', self.sensor_qos
        )
        self.detection_result_pub = self.create_publisher(
            Image, '/dor/detection_result', self.sensor_qos
        )

        # FPS tracking
        self.prev_time = time.time()
        self.fps = 0.0
        self.frame_count = 0

        # Timer untuk processing
        self.processing_timer = self.create_timer(0.033, self.process_frames)  # ~30Hz

        self.get_logger().info("🚀 Dynamic Object Removal System Started!")
        self.get_logger().info(f"Dynamic classes: {self.dynamic_classes}")
        self.get_logger().info(f"Static classes: {self.static_classes}")
        self.get_logger().info("Using BEST_EFFORT QoS for sensor data")

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from camera info"""
        try:
            k = msg.k
            if len(k) >= 5:
                self.fx = float(k[0])
                self.fy = float(k[4])
                self.cx = float(k[2])
                self.cy = float(k[5])
                self.camera_info = msg
                self.camera_info_received = True
                self.get_logger().info(f"Camera info received: fx={self.fx}, fy={self.fy}")
        except Exception as e:
            self.get_logger().warn(f"Camera info error: {e}")

    def rgb_callback(self, msg: Image):
        """Store RGB image dengan timestamp"""
        with self.lock:
            self.rgb_buffer.append(msg)

    def depth_callback(self, msg: Image):
        """Store depth image dengan timestamp"""
        with self.lock:
            self.depth_buffer.append(msg)

    def find_matching_frames(self):
        """Mencari pasangan RGB dan Depth dengan timestamp terdekat"""
        with self.lock:
            if not self.rgb_buffer or not self.depth_buffer:
                return None, None
            
            # Ambil frame terbaru dari RGB
            rgb_msg = self.rgb_buffer[-1]
            
            # Cari depth frame dengan timestamp terdekat
            best_depth_msg = None
            min_time_diff = float('inf')
            
            for depth_msg in self.depth_buffer:
                time_diff = abs(rgb_msg.header.stamp.sec - depth_msg.header.stamp.sec + 
                               (rgb_msg.header.stamp.nanosec - depth_msg.header.stamp.nanosec) * 1e-9)
                
                if time_diff < min_time_diff and time_diff < 0.1:  # Max 100ms difference
                    min_time_diff = time_diff
                    best_depth_msg = depth_msg
            
            return rgb_msg, best_depth_msg

    def process_frames(self):
        """Main processing ketika kedua RGB dan depth tersedia"""
        rgb_msg, depth_msg = self.find_matching_frames()
        
        if rgb_msg is None or depth_msg is None:
            return

        try:
            # Convert images
            rgb_frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            # Process dengan YOLO
            results = self.model(rgb_frame, verbose=False, imgsz=640)
            
            if len(results) > 0:
                self.process_detections(rgb_frame, depth_frame, results[0], rgb_msg.header)
            
            # Update FPS
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}", throttle_duration_sec=5.0)

    def process_detections(self, rgb_frame, depth_frame, result, header):
        """Process YOLO detections dan buat masks"""
        h, w = rgb_frame.shape[:2]
        
        # Initialize masks
        dynamic_mask = np.zeros((h, w), dtype=np.uint8)
        static_mask = np.zeros((h, w), dtype=np.uint8)
        
        # Convert depth to meters
        if depth_frame.dtype == np.uint16:
            depth_m = depth_frame.astype(np.float32) * 0.001
        else:
            depth_m = depth_frame.astype(np.float32)

        annotated_frame = rgb_frame.copy()
        boxes = result.boxes
        masks = result.masks

        if masks is not None:
            for i, mask in enumerate(masks.data.cpu().numpy()):
                if i >= len(boxes.cls):
                    continue
                    
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                label = self.model.names[cls_id]

                # Process mask
                mask_resized = (mask * 255).astype(np.uint8)
                mask_resized = cv2.resize(mask_resized, (w, h))
                _, binary_mask = cv2.threshold(mask_resized, 128, 255, cv2.THRESH_BINARY)

                # Classify sebagai dynamic atau static
                if label in self.dynamic_classes:
                    dynamic_mask = cv2.bitwise_or(dynamic_mask, binary_mask)
                    color = (0, 0, 255)  # Red untuk dynamic
                elif label in self.static_classes:
                    static_mask = cv2.bitwise_or(static_mask, binary_mask)
                    color = (0, 255, 0)  # Green untuk static
                else:
                    color = (255, 255, 0)  # Yellow untuk unknown

                # Draw pada annotated frame
                colored_mask = np.zeros_like(rgb_frame, dtype=np.uint8)
                colored_mask[binary_mask > 0] = color
                annotated_frame = cv2.addWeighted(annotated_frame, 1.0, colored_mask, 0.3, 0)

                # Draw bounding box
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, f"{label} {conf:.2f}",
                           (x1, max(15, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Apply morphological operations untuk membersihkan masks
        if self.mask_dilate > 0:
            kernel = np.ones((self.mask_dilate, self.mask_dilate), np.uint8)
            dynamic_mask = cv2.dilate(dynamic_mask, kernel, iterations=1)

        # === PROCESSING PIPELINE ===
        
        # 1. Clean depth (remove dynamic objects untuk SLAM)
        depth_clean = depth_m.copy()
        depth_clean[dynamic_mask > 0] = 0.0  # Gunakan 0 bukan NaN

        # 2. Create dynamic pointcloud (untuk Nav2 obstacles)
        dynamic_cloud = self.create_dynamic_pointcloud(depth_m, dynamic_mask, header)

        # 3. RGB processing - dynamic removal dan inpainting
        dynamic_removed = rgb_frame.copy()
        dynamic_removed[dynamic_mask > 0] = 0
        
        inpainted = cv2.inpaint(rgb_frame, dynamic_mask, self.inpaint_radius, cv2.INPAINT_TELEA)

        # === PUBLISH RESULTS ===
        self.publish_results(annotated_frame, depth_clean, dynamic_cloud, 
                           dynamic_removed, inpainted, header)

        # === DEBUG VISUALIZATION ===
        if self.show_debug:
            self.show_debug_views(rgb_frame, dynamic_mask, static_mask, depth_clean, inpainted)

    def create_dynamic_pointcloud(self, depth_m, dynamic_mask, header):
        """Create pointcloud dari dynamic objects saja"""
        rows, cols = depth_m.shape
        u, v = np.meshgrid(np.arange(cols), np.arange(rows))
        valid = (~np.isnan(depth_m)) & (depth_m > 0.1) & (depth_m < 10.0) & (dynamic_mask > 0)

        if not np.any(valid):
            return self.create_empty_pointcloud(header)

        x = (u[valid] - self.cx) * depth_m[valid] / self.fx
        y = (v[valid] - self.cy) * depth_m[valid] / self.fy
        z = depth_m[valid]
        
        pts = np.stack((x, y, z), axis=-1).astype(np.float32)

        cloud = PointCloud2()
        cloud.header.frame_id = header.frame_id
        cloud.header.stamp = header.stamp
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = 12
        cloud.height = 1
        cloud.width = pts.shape[0]
        cloud.row_step = 12 * pts.shape[0]
        cloud.data = pts.tobytes()
        
        return cloud

    def create_empty_pointcloud(self, header):
        """Create empty pointcloud"""
        cloud = PointCloud2()
        cloud.header.frame_id = header.frame_id
        cloud.header.stamp = header.stamp
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = 12
        cloud.height = 1
        cloud.width = 0
        cloud.row_step = 0
        cloud.data = b""
        return cloud

    def publish_results(self, annotated_frame, depth_clean, dynamic_cloud, 
                       dynamic_removed, inpainted, header):
        """Publish semua hasil processing"""
        try:
            # Publish detection result
            det_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            det_msg.header = header
            self.detection_result_pub.publish(det_msg)

            # Publish static depth untuk SLAM - pastikan format benar
            depth_clean_uint16 = (depth_clean * 1000).astype(np.uint16)  # Convert ke mm
            depth_msg = self.bridge.cv2_to_imgmsg(depth_clean_uint16, '16UC1')
            depth_msg.header = header
            self.static_depth_pub.publish(depth_msg)

            # Publish inpainted image untuk SLAM
            inpainted_msg = self.bridge.cv2_to_imgmsg(inpainted, 'bgr8')
            inpainted_msg.header = header
            self.inpainted_pub.publish(inpainted_msg)

            # Publish dynamic cloud untuk Nav2
            self.dynamic_cloud_pub.publish(dynamic_cloud)

            # Publish dynamic removed untuk visualisasi
            dynamic_removed_msg = self.bridge.cv2_to_imgmsg(dynamic_removed, 'bgr8')
            dynamic_removed_msg.header = header
            self.dynamic_removed_pub.publish(dynamic_removed_msg)

            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")

    def update_fps(self):
        """Update dan tampilkan FPS"""
        now = time.time()
        dt = now - self.prev_time
        if dt > 1.0:  # Update FPS setiap 1 detik
            self.fps = self.frame_count / dt
            self.frame_count = 0
            self.prev_time = now
            self.get_logger().info(f"Processing FPS: {self.fps:.1f}", throttle_duration_sec=2.0)

    def show_debug_views(self, rgb_frame, dynamic_mask, static_mask, depth_clean, inpainted):
        """Show debug visualization windows"""
        # Overlay dynamic mask pada RGB
        debug_overlay = rgb_frame.copy()
        debug_overlay[dynamic_mask > 0] = (0, 0, 255)  # Red untuk dynamic
        debug_overlay[static_mask > 0] = (0, 255, 0)   # Green untuk static

        # Normalize depth untuk display
        depth_display = np.nan_to_num(depth_clean)
        if np.max(depth_display) > 0:
            depth_display = (depth_display / np.nanmax(depth_display) * 255).astype(np.uint8)
        else:
            depth_display = np.zeros_like(depth_display, dtype=np.uint8)

        # Display semua debug windows
        cv2.imshow("1. Original RGB", rgb_frame)
        cv2.putText(debug_overlay, f"FPS: {self.fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("2. Detection Overlay", debug_overlay)
        cv2.imshow("3. Dynamic Mask", dynamic_mask)
        cv2.imshow("4. Static Depth", depth_display)
        cv2.imshow("5. Inpainted Result", inpainted)
        
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObjectRemovalFullSystem()
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