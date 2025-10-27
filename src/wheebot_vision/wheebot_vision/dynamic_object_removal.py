#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber


class DynamicObjectRemoval(Node):
    def __init__(self):
        super().__init__('dynamic_object_removal')

        # declare parameters
        self.declare_parameter('inpaint_radius', 3)
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('missing_depth_value', 'nan') # 'nan' | 'zero'

        self.inpaint_radius = self.get_parameter('inpaint_radius').value
        slop = float(self.get_parameter('sync_slop').value)
        queue = int(self.get_parameter('sync_queue_size').value)
        self.missing_depth_mode = str(self.get_parameter('missing_depth_value').value).lower()

        self.bridge = CvBridge()

        # QoS Profile
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        # reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        # subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.cameraInfoCallback, 10
        )
        self.image_sub = Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=sensor_qos
        )
        self.depth_sub = Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw', qos_profile=sensor_qos
        )
        self.mask_sub = Subscriber(
            self, Image, '/yolo/mask/dynamic_only', qos_profile=sensor_qos
        )

        # publisher
        self.dynamic_removed_image_pub = self.create_publisher(
            Image, '/dor/dynamic_removed/image', 10
        )
        self.dynamic_removed_depth_pub = self.create_publisher(
            Image, '/dor/dynamic_removed/depth', 10
        )
        self.inpainted_image_pub = self.create_publisher(
            Image, '/dor/inpainted/image', 10
        )
        self.inpainted_depth_pub = self.create_publisher(
            Image, '/dor/inpainted/depth', 10
        )
        
        # time synchronizer
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.mask_sub], queue_size=queue, slop=slop)
        self.ts.registerCallback(self.sync_cb)

        # cache
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_id = 'camera_color_optical_frame'
        self.u = None
        self.v = None

        self.get_logger().info(f"Dynamic Object Removal Node started")


    def cameraInfoCallback(self, msg: CameraInfo):
        try:
            k = msg.k
            self.fx, self.fy, self.cx, self.cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f"Failed to parse camera intrinsics: {e}")

    @staticmethod
    def depth_to_meters(depth_raw: np.ndarray) -> np.ndarray:
        if depth_raw.dtype == np.uint16:
            return depth_raw.astype(np.float32) * 0.001
        return depth_raw.astype(np.float32)

    def sync_cb(self, rgb_msg: Image, depth_msg: Image, mask_msg: Image):
        # Konversi image dulu
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        dyn_mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')

        if dyn_mask.shape[:2] != depth_raw.shape[:2]:
            dyn_mask = cv2.resize(dyn_mask, (depth_raw.shape[1], depth_raw.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Depth to meters (32FC1)
        depth_m = self.depth_to_meters(depth_raw)

        # dynamic_removed
        # RGB:
        inv_mask = (dyn_mask == 0).astype(np.uint8) * 255
        dyn_removed_rgb = cv2.bitwise_and(rgb, rgb, mask=inv_mask)
        # Depth:
        dyn_removed_depth = depth_m.copy()
        if self.missing_depth_mode == 'zero':
            dyn_removed_depth[dyn_mask > 0] = 0.0
        else: # default: NaN
            dyn_removed_depth[dyn_mask > 0] = np.nan

        # inpainted
        # RGB:
        inpaint_mask = (dyn_mask > 0).astype(np.uint8) * 255
        inpaint_rgb = cv2.inpaint(dyn_removed_rgb, inpaint_mask, self.inpaint_radius, cv2.INPAINT_TELEA)
        # Depth:
        depth_for_inpaint = dyn_removed_depth.copy()
        nan_mask = np.isnan(depth_for_inpaint)
        if np.any(nan_mask):
            depth_for_inpaint[nan_mask] = 0.0
        inpaint_depth = cv2.inpaint(depth_for_inpaint, inpaint_mask, self.inpaint_radius, cv2.INPAINT_TELEA)

        # publish
        # dynamic_removed/rgb
        msg_dyn_rgb = self.bridge.cv2_to_imgmsg(dyn_removed_rgb, encoding='bgr8')
        msg_dyn_rgb.header = rgb_msg.header
        msg_dyn_rgb.header.frame_id = self.frame_id or rgb_msg.header.frame_id
        self.dynamic_removed_image_pub.publish(msg_dyn_rgb)

        # dynamic_removed/depth (32FC1)
        msg_dyn_depth = self.bridge.cv2_to_imgmsg(dyn_removed_depth, encoding='32FC1')
        msg_dyn_depth.header = depth_msg.header
        msg_dyn_depth.header.frame_id = self.frame_id or depth_msg.header.frame_id
        self.dynamic_removed_depth_pub.publish(msg_dyn_depth)

        # inpainted/rgb
        msg_inp_rgb = self.bridge.cv2_to_imgmsg(inpaint_rgb, encoding='bgr8')
        msg_inp_rgb.header = rgb_msg.header
        msg_inp_rgb.header.frame_id = self.frame_id or rgb_msg.header.frame_id
        self.inpainted_image_pub.publish(msg_inp_rgb)

        # inpainted/depth (32FC1)
        msg_inp_depth = self.bridge.cv2_to_imgmsg(inpaint_depth.astype(np.float32), encoding='32FC1')
        msg_inp_depth.header = depth_msg.header
        msg_inp_depth.header.frame_id = self.frame_id or depth_msg.header.frame_id
        self.inpainted_depth_pub.publish(msg_inp_depth)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObjectRemoval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()