#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import struct

# ===============================
# Dynamic classes (COCO)
# ===============================
DYNAMIC_CLASSES = [
    "person", "bicycle", "car", "motorbike", "bus", "truck",
    "dog", "cat", "horse", "sheep", "cow", "elephant", "bear",
    "zebra", "giraffe"
]


# ===============================
# PointCloud helpers (flat cloud)
# ===============================
def make_pointcloud_fields():
    return [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]


def create_flat_pointcloud_from_mask(depth_m, mask, fx, fy, cx, cy, frame_id, stamp):
    """
    Create flat PointCloud2 that contains only the dynamic points (non-organized).
    depth_m: float32 depth in meters (rows,cols)
    mask: uint8 mask same size
    """
    rows, cols = depth_m.shape
    u = np.arange(cols)
    v = np.arange(rows)
    uu, vv = np.meshgrid(u, v)

    z = depth_m
    valid = (~np.isnan(z)) & (z > 0.0) & (mask > 0)

    if not np.any(valid):
        return None

    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy
    pts = np.stack((x, y, z), axis=2)
    pts_valid = pts[valid]
    pts_flat = pts_valid.astype(np.float32)

    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = pts_flat.shape[0]
    cloud.is_bigendian = False
    cloud.is_dense = False
    cloud.fields = make_pointcloud_fields()
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = pts_flat.tobytes()
    return cloud


def create_empty_pointcloud(frame_id, stamp):
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = 0
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.fields = make_pointcloud_fields()
    cloud.point_step = 12
    cloud.row_step = 0
    cloud.data = b""
    return cloud


# ===============================
# Node utama
# ===============================
class DepthDynamicRemovalNode(Node):
    def __init__(self):
        super().__init__('depth_dynamic_removal_node_final')
        self.bridge = CvBridge()

        # parameters
        self.declare_parameter('show_debug', False)
        self.declare_parameter('fx', 615.0)
        self.declare_parameter('fy', 615.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('mask_dilate', 3)
        self.declare_parameter('inpaint_rgb', True)
        self.declare_parameter('inpaint_radius', 3)
        self.declare_parameter('sync_slop_sec', 0.06)  # allowed time difference between messages

        self.show_debug = bool(self.get_parameter('show_debug').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.mask_dilate = int(self.get_parameter('mask_dilate').value)
        self.inpaint_rgb = bool(self.get_parameter('inpaint_rgb').value)
        self.inpaint_radius = int(self.get_parameter('inpaint_radius').value)
        self.sync_slop = float(self.get_parameter('sync_slop_sec').value)

        # Publishers
        # Cleaned RGB for SLAM
        self.pub_rgb_clean = self.create_publisher(Image, '/camera/color/image_clean', 5)
        # Cleaned depth for SLAM (32FC1 meters)
        self.pub_depth_clean = self.create_publisher(Image, '/camera/depth/dynamic_removed', 5)
        # Flat dynamic obstacles for Nav2
        self.pub_dyn_pc = self.create_publisher(PointCloud2, '/dynamic_obstacles', 5)

        # CameraInfo subscription to get intrinsics
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        # Subscriptions (manual sync)
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.det_topic = '/yolo/classified_detections'

        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Detection2DArray, self.det_topic, self.det_cb, 10)

        # buffers
        self._last_rgb = None
        self._last_depth = None
        self._last_det = None

        self.get_logger().info("DepthDynamicRemovalNode (final) started.")
        self.last_camera_info_received = False

    def camera_info_cb(self, msg: CameraInfo):
        try:
            k = msg.k
            if len(k) >= 5:
                self.fx = float(k[0])
                self.fy = float(k[4])
                self.cx = float(k[2])
                self.cy = float(k[5])
                self.last_camera_info_received = True
        except Exception as e:
            self.get_logger().warn(f"camera_info_cb error: {e}")

    # ---- callbacks store latest msg and try to sync ----
    def rgb_cb(self, msg: Image):
        self._last_rgb = msg
        self.try_sync()

    def depth_cb(self, msg: Image):
        self._last_depth = msg
        self.try_sync()

    def det_cb(self, msg: Detection2DArray):
        self._last_det = msg
        self.try_sync()

    def stamps_close(self, a, b):
        # both are builtin_interfaces.msg.Time
        ta = a.sec + a.nanosec * 1e-9
        tb = b.sec + b.nanosec * 1e-9
        return abs(ta - tb) <= self.sync_slop

    def try_sync(self):
        if self._last_rgb is None or self._last_depth is None or self._last_det is None:
            return

        # check timestamps approximately equal
        if not (self.stamps_close(self._last_rgb.header.stamp, self._last_depth.header.stamp) and
                self.stamps_close(self._last_rgb.header.stamp, self._last_det.header.stamp)):
            # if not close, don't process yet; keep latest and wait
            return

        # All good — make local copies and clear buffer
        rgb_msg = self._last_rgb
        depth_msg = self._last_depth
        det_msg = self._last_det
        self._last_rgb = None
        self._last_depth = None
        self._last_det = None

        # process synchronized set
        self.process_frame(rgb_msg, depth_msg, det_msg)

    def process_frame(self, rgb_msg: Image, depth_msg: Image, det_msg: Detection2DArray):
        stamp = depth_msg.header.stamp
        frame_id = depth_msg.header.frame_id if depth_msg.header.frame_id else rgb_msg.header.frame_id

        # decode input images safely
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            cv_depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # convert depth to meters float32
        if cv_depth_raw.dtype == np.uint16:
            depth_m = cv_depth_raw.astype(np.float32) * 0.001
        else:
            depth_m = cv_depth_raw.astype(np.float32)

        # build detection mask (bbox-based)
        mask = np.zeros(cv_rgb.shape[:2], dtype=np.uint8)
        for d in det_msg.detections:
            if len(d.results) == 0:
                continue
            cls_name = str(d.results[0].hypothesis.class_id).split(":")[-1].lower()
            if cls_name not in DYNAMIC_CLASSES:
                continue
            x = int(d.bbox.center.position.x)
            y = int(d.bbox.center.position.y)
            w = int(d.bbox.size_x)
            h = int(d.bbox.size_y)
            x1, y1 = max(0, x - w // 2), max(0, y - h // 2)
            x2, y2 = min(mask.shape[1], x1 + w), min(mask.shape[0], y1 + h)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)

        # expand mask a bit
        if self.mask_dilate > 0:
            k = np.ones((self.mask_dilate, self.mask_dilate), dtype=np.uint8)
            mask = cv2.dilate(mask, k, iterations=1)

        # === Prepare cleaned RGB ===
        rgb_clean = cv_rgb.copy()
        if self.inpaint_rgb:
            # inpaint needs 8-bit single channel mask where >0 = inpaint
            inpaint_mask = (mask > 0).astype(np.uint8) * 255
            # convert BGR to BGR (cv2 inpaint expects 3-channel BGR input)
            try:
                rgb_clean = cv2.inpaint(rgb_clean, inpaint_mask, self.inpaint_radius, cv2.INPAINT_TELEA)
            except Exception as e:
                self.get_logger().warn(f"inpaint failed: {e}; falling back to black-fill")
                rgb_clean[inpaint_mask > 0] = (0, 0, 0)
        else:
            # simply zero out masked region
            rgb_clean[mask > 0] = (0, 0, 0)

        # Publish cleaned rgb image
        try:
            rgb_out_msg = self.bridge.cv2_to_imgmsg(rgb_clean, encoding='bgr8')
            rgb_out_msg.header = rgb_msg.header
            self.pub_rgb_clean.publish(rgb_out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish cleaned RGB: {e}")

        # === Prepare cleaned depth (set masked pixels to NaN) ===
        depth_clean = depth_m.copy()
        depth_clean[mask > 0] = np.nan

        # Publish cleaned depth as 32FC1 (meters)
        try:
            depth_out_msg = self.bridge.cv2_to_imgmsg(depth_clean, encoding='32FC1')
            depth_out_msg.header = depth_msg.header
            self.pub_depth_clean.publish(depth_out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish cleaned depth: {e}")

        # === Publish flat dynamic obstacle cloud for Nav2 ===
        try:
            flat_pc = create_flat_pointcloud_from_mask(depth_m, mask, self.fx, self.fy, self.cx, self.cy, frame_id, stamp)
            if flat_pc is None:
                self.pub_dyn_pc.publish(create_empty_pointcloud(frame_id, stamp))
            else:
                self.pub_dyn_pc.publish(flat_pc)
        except Exception as e:
            self.get_logger().error(f"Error creating flat dynamic cloud: {e}")

        # Debug visualize
        if self.show_debug:
            dbg = cv_rgb.copy()
            dbg[mask > 0] = (0, 0, 255)
            cv2.imshow("Dynamic Removal Debug (mask overlay)", dbg)
            cv2.imshow("RGB Clean", rgb_clean)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthDynamicRemovalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
