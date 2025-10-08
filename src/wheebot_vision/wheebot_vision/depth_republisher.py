#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np


# ===============================
# Helper untuk PointCloud
# ===============================
def make_pointcloud_fields():
    return [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]


def create_flat_pointcloud_from_mask(depth_m, mask, fx, fy, cx, cy, frame_id, stamp):
    """Buat PointCloud2 datar berdasarkan area mask (misal area dinamis)."""
    rows, cols = depth_m.shape
    u, v = np.meshgrid(np.arange(cols), np.arange(rows))
    valid = (~np.isnan(depth_m)) & (depth_m > 0) & (mask > 0)

    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.fields = make_pointcloud_fields()
    cloud.is_bigendian = False
    cloud.is_dense = False
    cloud.point_step = 12

    if not np.any(valid):
        # Cloud kosong
        cloud.height = 1
        cloud.width = 0
        cloud.row_step = 0
        cloud.data = b""
        return cloud

    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    pts = np.stack((x, y, depth_m), axis=-1)[valid].astype(np.float32)

    cloud.height = 1
    cloud.width = pts.shape[0]
    cloud.row_step = 12 * pts.shape[0]
    cloud.data = pts.tobytes()
    return cloud


# ===============================
# Node utama
# ===============================
class DepthDynamicRemoval(Node):
    def __init__(self):
        super().__init__('depth_dynamic_removal')

        self.bridge = CvBridge()

        # === Parameter ===
        self.declare_parameter('mask_dilate', 3)
        self.declare_parameter('fx', 615.0)
        self.declare_parameter('fy', 615.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('show_debug', False)

        self.mask_dilate = int(self.get_parameter('mask_dilate').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)

        # === ROS Topic I/O ===
        self.sub_rgb = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.rgb_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10
        )
        self.sub_det_dynamic = self.create_subscription(
            Detection2DArray, '/objects/dynamic', self.det_callback, 10
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )

        # === Publishers ===
        self.pub_depth_clean = self.create_publisher(Image, '/dor/static_depth', 10)
        self.pub_dynamic_cloud = self.create_publisher(PointCloud2, '/dor/dynamic_cloud', 10)

        # === Buffers ===
        self.last_rgb = None
        self.last_depth = None
        self.last_det = None
        self.last_camera_info_received = False

        self.get_logger().info("Depth Dynamic Removal Node aktif — publish depth bersih & pointcloud dinamis.")

    # === Callback Camera Info ===
    def camera_info_callback(self, msg: CameraInfo):
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

    # === Callback ===
    def rgb_callback(self, msg):
        self.last_rgb = msg
        self.try_process()

    def depth_callback(self, msg):
        self.last_depth = msg
        self.try_process()

    def det_callback(self, msg):
        self.last_det = msg
        self.try_process()

    # === Sinkronisasi sederhana ===
    def try_process(self):
        if self.last_rgb is None or self.last_depth is None or self.last_det is None:
            return

        # Pastikan timestamp cukup dekat
        t_rgb = self.last_rgb.header.stamp.sec + self.last_rgb.header.stamp.nanosec * 1e-9
        t_depth = self.last_depth.header.stamp.sec + self.last_depth.header.stamp.nanosec * 1e-9
        if abs(t_rgb - t_depth) > 0.1:
            return

        rgb_msg = self.last_rgb
        depth_msg = self.last_depth
        det_msg = self.last_det

        # Reset buffer (biar real-time, tidak tertahan frame lama)
        self.last_rgb = None
        self.last_depth = None
        self.last_det = None

        self.process_frame(rgb_msg, depth_msg, det_msg)

    # === Proses utama ===
    def process_frame(self, rgb_msg, depth_msg, det_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # Konversi depth ke meter
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)

        mask = np.zeros(rgb.shape[:2], dtype=np.uint8)

        # Mask dari bounding box dinamis
        for det in det_msg.detections:
            if not det.results:
                continue
            x = int(det.bbox.center.position.x)
            y = int(det.bbox.center.position.y)
            w = int(det.bbox.size_x)
            h = int(det.bbox.size_y)
            x1, y1 = max(0, x - w // 2), max(0, y - h // 2)
            x2, y2 = min(mask.shape[1], x1 + w), min(mask.shape[0], y1 + h)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)

        if self.mask_dilate > 0:
            kernel = np.ones((self.mask_dilate, self.mask_dilate), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=1)

        # === Bersihkan depth (hapus area dinamis) ===
        depth_clean = depth_m.copy()
        depth_clean[mask > 0] = np.nan

        # Publish depth bersih
        try:
            depth_msg_out = self.bridge.cv2_to_imgmsg(depth_clean, encoding='32FC1')
            depth_msg_out.header = depth_msg.header
            self.pub_depth_clean.publish(depth_msg_out)
        except Exception as e:
            self.get_logger().error(f"Gagal publish depth bersih: {e}")

        # === Publish pointcloud dinamis ===
        try:
            cloud_msg = create_flat_pointcloud_from_mask(
                depth_m, mask, self.fx, self.fy, self.cx, self.cy,
                depth_msg.header.frame_id, depth_msg.header.stamp
            )
            self.pub_dynamic_cloud.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Gagal buat PointCloud: {e}")

        # === Debug tampilan opsional ===
        if self.show_debug:
            dbg = rgb.copy()
            dbg[mask > 0] = (0, 0, 255)
            cv2.imshow("Dynamic Mask Overlay", dbg)
            cv2.imshow("Depth Clean Preview", (np.nan_to_num(depth_clean) / np.nanmax(depth_clean)))
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthDynamicRemoval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
