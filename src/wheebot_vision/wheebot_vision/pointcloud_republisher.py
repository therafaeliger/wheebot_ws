#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import numpy as np
import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs_py import point_cloud2 as pc2


class PointcloudRepublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_republisher')
        
        # declare parameters
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('frame_id', '') # optional override

        slop = float(self.get_parameter('sync_slop').value)
        queue = int(self.get_parameter('sync_queue_size').value)
        frame_override = self.get_parameter('frame_id').value.strip()

        self.bridge = CvBridge()

        # QoS Profile
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.cameraInfoCallback, 10
        )
        self.depth_sub = Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw', qos_profile=sensor_qos
        )
        self.mask_dynamic_sub = Subscriber(
            self, Image, '/yolo/mask/dynamic_only', qos_profile=sensor_qos
        )
        self.mask_static_sub = Subscriber(
            self, Image, '/yolo/mask/static_only', qos_profile=sensor_qos
        )

        # publisher
        self.pub_pc_static = self.create_publisher(
            PointCloud2,  '/dor/pointcloud/static_only', 10
        )
        self.pub_pc_dynamic = self.create_publisher(
            PointCloud2, '/dor/pointcloud/dynamic_only', 10
        )
        self.pub_pc_removed = self.create_publisher(
            PointCloud2, '/dor/dynamic_removed/pointcloud', 10
        )

        # time synchronizer
        self.ts = ApproximateTimeSynchronizer([self.depth_sub, self.mask_dynamic_sub, self.mask_static_sub], queue_size=queue, slop=slop)
        self.ts.registerCallback(self.sync_cb)

        # cache
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_id = frame_override or 'camera_color_optical_frame'
        self.u = None
        self.v = None
        self._last_shape = None

        self.get_logger().info(f"Pointcloud Republisher Node started")


    def cameraInfoCallback(self, msg: CameraInfo):
        try:
            k = msg.k
            self.fx, self.fy, self.cx, self.cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f"Failed to parse camera intrinsics: {e}")

    @staticmethod
    def _depth_to_meters(depth_raw: np.ndarray) -> np.ndarray:
        if depth_raw.dtype == np.uint16:
            return depth_raw.astype(np.float32) * 0.001
        return depth_raw.astype(np.float32)
    
    def _ensure_grid(self, shape):
        if self._last_shape != shape:
            rows, cols = shape
            self._u, self._v = np.meshgrid(np.arange(cols), np.arange(rows))
            self._last_shape = shape

    def _make_cloud(self, depth_m: np.ndarray, valid_mask: np.ndarray, header) -> PointCloud2:
        if None in (self.fx, self.fy, self.cx, self.cy):
            return self._empty_cloud(header)

        if not np.any(valid_mask):
            return self._empty_cloud(header)

        self._ensure_grid(depth_m.shape)
        x = (self._u - self.cx) * depth_m / self.fx
        y = (self._v - self.cy) * depth_m / self.fy
        pts = np.stack((x, y, depth_m), axis=-1)[valid_mask].astype(np.float32)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = pc2.create_cloud(header, fields, pts)
        cloud.is_dense = True
        cloud.header.frame_id = self.frame_id or header.frame_id
        return cloud

    
    def _empty_cloud(self, header):
        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = self.frame_id or header.frame_id
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = 12
        cloud.row_step = 0
        cloud.data = b''
        return cloud


    def sync_cb(self, depth_msg: Image, dyn_mask_msg: Image, stat_mask_msg: Image):
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        dyn_mask = self.bridge.imgmsg_to_cv2(dyn_mask_msg, 'mono8')
        stat_mask = self.bridge.imgmsg_to_cv2(stat_mask_msg, 'mono8')

        # Match sizes
        if dyn_mask.shape[:2] != depth_raw.shape[:2]:
            dyn_mask = cv2.resize(dyn_mask, (depth_raw.shape[1], depth_raw.shape[0]), interpolation=cv2.INTER_NEAREST)
        if stat_mask.shape[:2] != depth_raw.shape[:2]:
            stat_mask = cv2.resize(stat_mask, (depth_raw.shape[1], depth_raw.shape[0]), interpolation=cv2.INTER_NEAREST)

        depth_m = self._depth_to_meters(depth_raw)
        self._ensure_grid(depth_m.shape)

        # valid depths
        finite = np.isfinite(depth_m) & (depth_m > 0)

        # Dynamic / Static / Removed regions
        dynamic_mask = (dyn_mask > 0) & finite
        static_mask = (stat_mask > 0) & finite
        removed_mask = (~(dyn_mask > 0)) & finite  # inverse of dynamic

        # Build and publish
        header = depth_msg.header

        pc_dyn = self._make_cloud(depth_m, dynamic_mask, header)
        pc_stat = self._make_cloud(depth_m, static_mask, header)
        pc_removed = self._make_cloud(depth_m, removed_mask, header)

        self.pub_pc_dynamic.publish(pc_dyn)
        self.pub_pc_static.publish(pc_stat)
        self.pub_pc_removed.publish(pc_removed)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()