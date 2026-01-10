#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import numpy as np
import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import struct

class PointcloudRepublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_republisher')
        
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('sync_slop', 0.15) # Toleransi waktu (detik)
        self.declare_parameter('sync_queue_size', 50) # Buffer
        self.declare_parameter('frame_id', '') 
        # Decimation: 1 = Full Res, 2 = 1/2 Res (4x faster), 4 = 1/4 Res (16x faster)
        self.declare_parameter('decimation', 2) 

        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_depth_topic', '/camera/depth_image')
        self.declare_parameter('mask_dynamic_topic', '/yolo/mask/dynamic_only')
        self.declare_parameter('mask_static_topic', '/yolo/mask/static_only')
        
        slop = float(self.get_parameter('sync_slop').value)
        queue = int(self.get_parameter('sync_queue_size').value)
        self.decimation = int(self.get_parameter('decimation').value)
        frame_override = self.get_parameter('frame_id').value.strip()

        topic_info = self.get_parameter('camera_info_topic').value
        topic_depth = self.get_parameter('camera_depth_topic').value
        topic_mask_dyn = self.get_parameter('mask_dynamic_topic').value
        topic_mask_stat = self.get_parameter('mask_static_topic').value

        self.bridge = CvBridge()
        self.frame_id = frame_override or 'camera_color_optical_frame'

        # QoS
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, topic_info, self.cameraInfoCallback, 10, callback_group=self.cb_group
        )
        self.depth_sub = Subscriber(self, Image, topic_depth, qos_profile=sensor_qos)
        self.mask_dynamic_sub = Subscriber(self, Image, topic_mask_dyn, qos_profile=sensor_qos)
        self.mask_static_sub = Subscriber(self, Image, topic_mask_stat, qos_profile=sensor_qos)

        # Publishers
        self.pub_pc_static = self.create_publisher(PointCloud2, '/dor/pointcloud/static_only', 10, callback_group=self.cb_group)
        self.pub_pc_dynamic = self.create_publisher(PointCloud2, '/dor/pointcloud/dynamic_only', 10, callback_group=self.cb_group)
        self.pub_pc_removed = self.create_publisher(PointCloud2, '/dor/dynamic_removed/pointcloud', 10, callback_group=self.cb_group)

        # Synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.depth_sub, self.mask_dynamic_sub, self.mask_static_sub], 
            queue_size=queue, slop=slop
        )
        self.ts.registerCallback(self.sync_cb)

        # Cache variables
        self.fx = self.fy = self.cx = self.cy = None
        self._grid_x = None
        self._grid_y = None
        self._last_shape = None

        self.get_logger().info(f"----------------------------------------------------------------")
        self.get_logger().info(f"Pointcloud Republisher Started. Decimation: {self.decimation}")
        self.get_logger().info(f"Subscribing to Camera Info: {topic_info}")
        self.get_logger().info(f"Subscribing to Depth Image: {topic_depth}")
        self.get_logger().info(f"Subscribing to Dynamic Mask: {topic_mask_dyn}")
        self.get_logger().info(f"Subscribing to Static Mask: {topic_mask_stat}")
        self.get_logger().info(f"----------------------------------------------------------------")

    def cameraInfoCallback(self, msg: CameraInfo):
        try:
            if self.fx is None:
                k = msg.k
                self.fx, self.fy, self.cx, self.cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
                if msg.header.frame_id and self.frame_id == 'camera_color_optical_frame':
                    self.frame_id = msg.header.frame_id
                self.get_logger().info("Camera Intrinsics Received.")
        except Exception:
            pass

    def _precompute_grid(self, h, w):
        if self.fx is None: return

        u_indices = np.arange(0, w * self.decimation, self.decimation)
        v_indices = np.arange(0, h * self.decimation, self.decimation)
        
        u_indices = u_indices[:w]
        v_indices = v_indices[:h]

        uu, vv = np.meshgrid(u_indices, v_indices)
        
        self._grid_x = (uu - self.cx) / self.fx
        self._grid_y = (vv - self.cy) / self.fy
        self._last_shape = (h, w)

    def _fast_create_cloud(self, points_nx3, header):
        cloud = PointCloud2()
        cloud.header = header
        
        cloud.header.frame_id = self.frame_id

        cloud.height = 1
        cloud.width = points_nx3.shape[0]
        cloud.is_dense = False
        cloud.is_bigendian = False
        
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        
        cloud.data = points_nx3.astype(np.float32).tobytes()
        
        return cloud

    def sync_cb(self, depth_msg, dyn_mask_msg, stat_mask_msg):
        if self.fx is None:
            return

        try:
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            d = self.decimation
            depth_sub = depth_raw[::d, ::d]
            
            h, w = depth_sub.shape
            
            if self._last_shape != (h, w):
                self._precompute_grid(h, w)

            z_m = depth_sub.astype(np.float32) * 0.001
            
            valid_z = (z_m > 0.1) & (z_m < 10.0) # Range filter sederhana (optional)

            dyn_mask_full = self.bridge.imgmsg_to_cv2(dyn_mask_msg, 'mono8')
            stat_mask_full = self.bridge.imgmsg_to_cv2(stat_mask_msg, 'mono8')

            dyn_mask_sub = cv2.resize(dyn_mask_full, (w, h), interpolation=cv2.INTER_NEAREST)
            stat_mask_sub = cv2.resize(stat_mask_full, (w, h), interpolation=cv2.INTER_NEAREST)

            is_dynamic = (dyn_mask_sub > 0)
            is_static_obj = (stat_mask_sub > 0)
            
            mask_dyn = valid_z & is_dynamic
            
            mask_stat = valid_z & is_static_obj
            
            mask_removed = valid_z & (~is_dynamic)

            
            def generate_xyz(mask):
                if not np.any(mask):
                    return np.zeros((0, 3), dtype=np.float32)
                
                z_vals = z_m[mask]
                
                x_vals = self._grid_x[mask] * z_vals
                
                y_vals = self._grid_y[mask] * z_vals
                
                return np.stack((x_vals, y_vals, z_vals), axis=-1)

            pts_dyn = generate_xyz(mask_dyn)
            pts_stat = generate_xyz(mask_stat)
            pts_removed = generate_xyz(mask_removed)

            header = depth_msg.header
            header.frame_id = self.frame_id

            self.pub_pc_dynamic.publish(self._fast_create_cloud(pts_dyn, header))
            self.pub_pc_static.publish(self._fast_create_cloud(pts_stat, header))
            self.pub_pc_removed.publish(self._fast_create_cloud(pts_removed, header))

        except Exception as e:
            self.get_logger().error(f"Error in PC Republisher: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudRepublisher()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()