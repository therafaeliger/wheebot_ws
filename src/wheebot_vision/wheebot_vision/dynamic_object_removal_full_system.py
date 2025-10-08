#!/usr/bin/env python3

import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time


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

        # Camera parameters (akan diupdate dari camera_info)
        self.fx, self.fy, self.cx, self.cy = 615.0, 615.0, 320.0, 240.0

        # === ROS Subscriptions ===
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )

        # === ROS Publishers ===
        # Untuk SLAM (RTAB-Map)
        self.static_depth_pub = self.create_publisher(Image, '/dor/static_depth', 10)
        self.static_rgb_pub = self.create_publisher(Image, '/dor/static_rgb', 10)
        
        # Untuk Navigation (Nav2)
        self.dynamic_cloud_pub = self.create_publisher(PointCloud2, '/dor/dynamic_cloud', 10)
        
        # Untuk Visualisasi
        self.dynamic_removed_pub = self.create_publisher(Image, '/dor/dynamic_removed', 10)
        self.inpainted_pub = self.create_publisher(Image, '/dor/inpainted', 10)
        self.detection_result_pub = self.create_publisher(Image, '/dor/detection_result', 10)

        # Buffer untuk sinkronisasi
        self.current_rgb = None
        self.current_depth = None
        self.rgb_header = None

        # FPS tracking
        self.prev_time = time.time()
        self.fps = 0.0

        self.get_logger().info("🚀 Dynamic Object Removal System Started!")
        self.get_logger().info(f"Dynamic classes: {self.dynamic_classes}")
        self.get_logger().info(f"Static classes: {self.static_classes}")

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from camera info"""
        try:
            k = msg.k
            if len(k) >= 5:
                self.fx = float(k[0])
                self.fy = float(k[4])
                self.cx = float(k[2])
                self.cy = float(k[5])
        except Exception as e:
            self.get_logger().warn(f"Camera info error: {e}")

    def rgb_callback(self, msg: Image):
        """Store latest RGB image"""
        self.current_rgb = msg
        self.rgb_header = msg.header
        self.process_frames()

    def depth_callback(self, msg: Image):
        """Store latest depth image"""
        self.current_depth = msg
        self.process_frames()

    def process_frames(self):
        """Main processing when both RGB and depth are available"""
        if self.current_rgb is None or self.current_depth is None:
            return

        try:
            # Convert images
            rgb_frame = self.bridge.imgmsg_to_cv2(self.current_rgb, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(self.current_depth, 'passthrough')
            
            # Process with YOLO
            results = self.model(rgb_frame, verbose=False)
            
            if len(results) > 0:
                self.process_detections(rgb_frame, depth_frame, results[0])
            
            # Update FPS
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def process_detections(self, rgb_frame, depth_frame, result):
        """Process YOLO detections and create masks"""
        h, w = rgb_frame.shape[:2]
        
        # Initialize masks
        dynamic_mask = np.zeros((h, w), dtype=np.uint8)
        static_mask = np.zeros((h, w), dtype=np.uint8)
        all_detections_mask = np.zeros((h, w), dtype=np.uint8)
        
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
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                label = self.model.names[cls_id]

                # Process mask
                mask_resized = (mask * 255).astype(np.uint8)
                mask_resized = cv2.resize(mask_resized, (w, h))
                _, binary_mask = cv2.threshold(mask_resized, 128, 255, cv2.THRESH_BINARY)

                # Classify as dynamic or static
                if label in self.dynamic_classes:
                    dynamic_mask = cv2.bitwise_or(dynamic_mask, binary_mask)
                    color = (0, 0, 255)  # Red for dynamic
                elif label in self.static_classes:
                    static_mask = cv2.bitwise_or(static_mask, binary_mask)
                    color = (0, 255, 0)  # Green for static
                else:
                    color = (255, 255, 0)  # Yellow for unknown

                # Add to all detections mask for visualization
                all_detections_mask = cv2.bitwise_or(all_detections_mask, binary_mask)

                # Draw on annotated frame
                colored_mask = np.zeros_like(rgb_frame, dtype=np.uint8)
                colored_mask[binary_mask > 0] = color
                annotated_frame = cv2.addWeighted(annotated_frame, 1.0, colored_mask, 0.3, 0)

                # Draw bounding box
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, f"{label} {conf:.2f}",
                           (x1, max(15, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Apply morphological operations to clean masks
        if self.mask_dilate > 0:
            kernel = np.ones((self.mask_dilate, self.mask_dilate), np.uint8)
            dynamic_mask = cv2.dilate(dynamic_mask, kernel, iterations=1)

        # === PROCESSING PIPELINE ===
        
        # 1. Clean depth (remove dynamic objects for SLAM)
        depth_clean = depth_m.copy()
        depth_clean[dynamic_mask > 0] = np.nan

        # 2. Create dynamic pointcloud (for Nav2 obstacles)
        dynamic_cloud = self.create_dynamic_pointcloud(depth_m, dynamic_mask)

        # 3. RGB processing - dynamic removal and inpainting
        dynamic_removed = rgb_frame.copy()
        dynamic_removed[dynamic_mask > 0] = 0
        
        inpainted = cv2.inpaint(rgb_frame, dynamic_mask, self.inpaint_radius, cv2.INPAINT_TELEA)

        # 4. Static RGB (for visualization)
        static_rgb = rgb_frame.copy()
        static_rgb[dynamic_mask > 0] = 0

        # === PUBLISH RESULTS ===
        self.publish_results(annotated_frame, depth_clean, dynamic_cloud, 
                           dynamic_removed, inpainted, static_rgb)

        # === DEBUG VISUALIZATION ===
        if self.show_debug:
            self.show_debug_views(rgb_frame, dynamic_mask, static_mask, depth_clean, inpainted)

    def create_dynamic_pointcloud(self, depth_m, dynamic_mask):
        """Create pointcloud from dynamic objects only"""
        rows, cols = depth_m.shape
        u, v = np.meshgrid(np.arange(cols), np.arange(rows))
        valid = (~np.isnan(depth_m)) & (depth_m > 0) & (dynamic_mask > 0)

        if not np.any(valid):
            return self.create_empty_pointcloud()

        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        pts = np.stack((x, y, depth_m), axis=-1)[valid].astype(np.float32)

        cloud = PointCloud2()
        cloud.header.frame_id = self.rgb_header.frame_id
        cloud.header.stamp = self.rgb_header.stamp
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

    def create_empty_pointcloud(self):
        """Create empty pointcloud"""
        cloud = PointCloud2()
        cloud.header.frame_id = self.rgb_header.frame_id
        cloud.header.stamp = self.rgb_header.stamp
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
                       dynamic_removed, inpainted, static_rgb):
        """Publish all processed results"""
        try:
            # Publish detection result
            det_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            det_msg.header = self.rgb_header
            self.detection_result_pub.publish(det_msg)

            # Publish static depth for SLAM
            depth_msg = self.bridge.cv2_to_imgmsg(depth_clean, '32FC1')
            depth_msg.header = self.rgb_header
            self.static_depth_pub.publish(depth_msg)

            # Publish static RGB
            static_rgb_msg = self.bridge.cv2_to_imgmsg(static_rgb, 'bgr8')
            static_rgb_msg.header = self.rgb_header
            self.static_rgb_pub.publish(static_rgb_msg)

            # Publish dynamic cloud for Nav2
            self.dynamic_cloud_pub.publish(dynamic_cloud)

            # Publish RGB processing results
            dynamic_removed_msg = self.bridge.cv2_to_imgmsg(dynamic_removed, 'bgr8')
            dynamic_removed_msg.header = self.rgb_header
            self.dynamic_removed_pub.publish(dynamic_removed_msg)

            inpainted_msg = self.bridge.cv2_to_imgmsg(inpainted, 'bgr8')
            inpainted_msg.header = self.rgb_header
            self.inpainted_pub.publish(inpainted_msg)

        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")

    def update_fps(self):
        """Update and display FPS"""
        now = time.time()
        dt = now - self.prev_time
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)  # Smooth FPS
        self.prev_time = now

    def show_debug_views(self, rgb_frame, dynamic_mask, static_mask, depth_clean, inpainted):
        """Show debug visualization windows"""
        # Overlay dynamic mask on RGB
        debug_overlay = rgb_frame.copy()
        debug_overlay[dynamic_mask > 0] = (0, 0, 255)  # Red for dynamic
        debug_overlay[static_mask > 0] = (0, 255, 0)   # Green for static

        # Normalize depth for display
        depth_display = np.nan_to_num(depth_clean)
        if np.max(depth_display) > 0:
            depth_display = (depth_display / np.nanmax(depth_display) * 255).astype(np.uint8)
        else:
            depth_display = np.zeros_like(depth_display, dtype=np.uint8)

        # Display all debug windows
        cv2.imshow("1. Original RGB", rgb_frame)
        cv2.imshow("2. Detection Overlay", debug_overlay)
        cv2.imshow("3. Dynamic Mask", dynamic_mask)
        cv2.imshow("4. Static Depth", depth_display)
        cv2.imshow("5. Inpainted Result", inpainted)
        
        # Add FPS to one window
        cv2.putText(debug_overlay, f"FPS: {self.fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
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