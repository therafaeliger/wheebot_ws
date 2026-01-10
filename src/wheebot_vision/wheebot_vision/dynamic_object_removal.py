#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber

class DynamicObjectRemoval(Node):
    def __init__(self):
        super().__init__('dynamic_object_removal')

        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('inpaint_radius', 1)
        self.declare_parameter('sync_slop', 0.15)
        self.declare_parameter('sync_queue_size', 50)
        self.declare_parameter('enable_inpainting', True) 
        
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_image_topic', '/camera/image')
        self.declare_parameter('camera_depth_topic', '/camera/depth_image')
        self.declare_parameter('mask_topic', '/yolo/mask/dynamic_only')

        self.inpaint_radius = self.get_parameter('inpaint_radius').value
        slop = float(self.get_parameter('sync_slop').value)
        queue = int(self.get_parameter('sync_queue_size').value)
        self.enable_inpainting = self.get_parameter('enable_inpainting').value

        topic_info = self.get_parameter('camera_info_topic').value
        topic_img = self.get_parameter('camera_image_topic').value
        topic_depth = self.get_parameter('camera_depth_topic').value
        topic_mask = self.get_parameter('mask_topic').value

        self.bridge = CvBridge()

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        
        # --- Subscribers ---
        self.camera_info_sub = self.create_subscription(
            CameraInfo, topic_info, self.cameraInfoCallback, 10, callback_group=self.cb_group
        )
        self.image_sub = Subscriber(self, Image, topic_img, qos_profile=sensor_qos)
        self.depth_sub = Subscriber(self, Image, topic_depth, qos_profile=sensor_qos)
        self.mask_sub = Subscriber(self, Image, topic_mask, qos_profile=sensor_qos)

        # --- Publishers ---
        self.dynamic_removed_image_pub = self.create_publisher(Image, '/dor/dynamic_removed/image', 10, callback_group=self.cb_group)
        self.dynamic_removed_depth_pub = self.create_publisher(Image, '/dor/dynamic_removed/depth', 10, callback_group=self.cb_group)
        
        if self.enable_inpainting:
            self.inpainted_image_pub = self.create_publisher(Image, '/dor/inpainted/image', 10, callback_group=self.cb_group)
            self.inpainted_depth_pub = self.create_publisher(Image, '/dor/inpainted/depth', 10, callback_group=self.cb_group)
        
        # --- Time Synchronizer ---
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.mask_sub], 
            queue_size=queue, 
            slop=slop
        )
        self.ts.registerCallback(self.sync_cb)

        self.frame_id = 'camera_depth_optical_frame'

        mode_str = "INPAINTING" if self.enable_inpainting else "MASKING ONLY"
        self.get_logger().info(f"----------------------------------------------------------------")
        self.get_logger().info(f"DOR Node Started. Mode: {mode_str}")
        self.get_logger().info(f"Subscribing to Camera Info: {topic_info}")
        self.get_logger().info(f"Subscribing to Image: {topic_img}")
        self.get_logger().info(f"Subscribing to Depth Image: {topic_depth}")
        self.get_logger().info(f"Subscribing to Mask: {topic_mask}")
        self.get_logger().info(f"----------------------------------------------------------------")

    def cameraInfoCallback(self, msg: CameraInfo):
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def sync_cb(self, rgb_msg: Image, depth_msg: Image, mask_msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough') 
            dyn_mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')

            if dyn_mask.shape[:2] != depth_raw.shape[:2]:
                dyn_mask = cv2.resize(dyn_mask, (depth_raw.shape[1], depth_raw.shape[0]), interpolation=cv2.INTER_NEAREST)

            bool_mask = dyn_mask > 0 

            rgb_removed = rgb.copy()
            rgb_removed[bool_mask] = 0

            depth_removed = depth_raw.copy()
            depth_removed[bool_mask] = 0

            msg_dyn_rgb = self.bridge.cv2_to_imgmsg(rgb_removed, encoding='bgr8')
            msg_dyn_rgb.header = rgb_msg.header
            self.dynamic_removed_image_pub.publish(msg_dyn_rgb)

            msg_dyn_depth = self.bridge.cv2_to_imgmsg(depth_removed, encoding='16UC1')
            msg_dyn_depth.header = depth_msg.header
            self.dynamic_removed_depth_pub.publish(msg_dyn_depth)

            if self.enable_inpainting:
                rgb_inpainted = cv2.inpaint(rgb_removed, dyn_mask, self.inpaint_radius, cv2.INPAINT_TELEA)
                
                depth_inpainted = cv2.inpaint(depth_removed, dyn_mask, self.inpaint_radius, cv2.INPAINT_TELEA)

                msg_inp_rgb = self.bridge.cv2_to_imgmsg(rgb_inpainted, encoding='bgr8')
                msg_inp_rgb.header = rgb_msg.header
                self.inpainted_image_pub.publish(msg_inp_rgb)

                msg_inp_depth = self.bridge.cv2_to_imgmsg(depth_inpainted, encoding='16UC1')
                msg_inp_depth.header = depth_msg.header
                self.inpainted_depth_pub.publish(msg_inp_depth)

        except Exception as e:
            self.get_logger().error(f"Error in DOR sync_cb: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObjectRemoval()
    
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