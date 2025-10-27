#!/usr/bin/env python3
import os
import io
import json
import math
import time
import glob
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, PointCloud2
from nav2_msgs.action import NavigateToPose
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

try:
    # cv bridge-like minimal encoder using OpenCV if available; else PIL fallback
    import cv2
    _HAS_CV2 = True
except Exception:
    from PIL import Image as PILImage
    _HAS_CV2 = False

# --------- helpers ---------

def now():
    return time.monotonic()

@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

# --------- Core ROS Node ---------
class WheeBotCore(Node):
    def __init__(self):
        # super().__init__('wheebot_webui_core')
        super().__init__(f'wheebot_webui_core_{int(time.time()*1000)%10000}')

        # Params
        self.declare_parameter('image_topics', ['/camera/color/image_raw', '/camera/depth/image_raw'])
        self.declare_parameter('pointcloud_topics', ['/pointcloud/full', '/pointcloud/static_only', '/pointcloud/dynamic_only'])
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('pose_topic', '/odom')  # used only for presence; TF would be better, kept simple

        self.image_topics = set(self.get_parameter('image_topics').value)
        self.pointcloud_topics = set(self.get_parameter('pointcloud_topics').value)
        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        # State stores
        self._latest_images: Dict[str, Tuple[float, Image]] = {}
        self._latest_map: Optional[Tuple[float, OccupancyGrid]] = None
        self._latest_path: Optional[Tuple[float, Path]] = None
        self._pose = RobotPose()
        self._robot_alive_heartbeat = 0.0

        qos_best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # Publishers
        self.pub_key_vel = self.create_publisher(Twist, 'key_vel', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/wheebot_controller/cmd_vel', 10)

        # Subscribers (lazy per topic when requested)
        self.create_subscription(OccupancyGrid, self.map_topic, self._cb_map, qos_best_effort)
        self.create_subscription(Path, self.path_topic, self._cb_path, qos_best_effort)
        self.create_subscription(Odometry, '/odom', self._cb_any_alive, qos_best_effort)

        # Action: Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Heartbeat by observing cmd_vel traffic
        self.create_subscription(Twist, '/wheebot_controller/cmd_vel', self._cb_heartbeat, qos_best_effort)

        # Timer to mark alive if any data flows
        self.create_timer(1.0, self._tick_alive)

        # Dynamic subscriptions for images & pointclouds will be made on demand
        self._img_subs: Dict[str, any] = {}
        self._pc_subs: Dict[str, any] = {}

    def _mark_alive(self):
        self._robot_alive_heartbeat = now()
    
    def _graph_alive(self) -> bool:
        """Cek apakah ada publisher aktif di topik penting"""
        check_topics = [
            '/wheebot_controller/cmd_vel',
            '/cmd_vel',
            '/odom',
            '/camera/color/image_raw',
            '/map'
        ]
        try:
            for t in check_topics:
                if len(self.get_publishers_info_by_topic(t)) > 0:
                    return True
        except Exception:
            pass
        return False

    def _cb_any_alive(self, *_):
        self._mark_alive()


    def _tick_alive(self):
    # just keep ROS timers spinning, nothing to do
        pass

    # --- Callbacks ---
    def _cb_map(self, msg: OccupancyGrid):
        self._latest_map = (now(), msg)
        self._mark_alive()

    def _cb_path(self, msg: Path):
        self._latest_path = (now(), msg)
        self._mark_alive()

    def _cb_heartbeat(self, msg: Twist):
        self._robot_alive_heartbeat = now()

    def _ensure_image_sub(self, topic: str):
        if topic in self._img_subs:
            return
        if topic not in self.image_topics:
            self.image_topics.add(topic)
        def cb(msg):
            self._latest_images[topic] = (now(), msg)
        sub = self.create_subscription(Image, topic, cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._img_subs[topic] = sub
        self._mark_alive()

    def _ensure_pc_sub(self, topic: str):
        if topic in self._pc_subs:
            return
        if topic not in self.pointcloud_topics:
            self.pointcloud_topics.add(topic)
        def cb(msg: PointCloud2):
            # store last receive time only; we will render simple top-down density on request by sampling
            self._pc_last = (topic, now(), msg)
        sub = self.create_subscription(PointCloud2, topic, cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._pc_subs[topic] = sub
        self._mark_alive()

    # --- Public API used by HTTP server ---
    def get_status(self) -> Dict:
        alive = (now() - self._robot_alive_heartbeat) < 2.5 or self._graph_alive()
        return {
            'robot_on': bool(alive),
            'map_topic': self.map_topic,
            'path_topic': self.path_topic,
            'image_topics': sorted(list(self.image_topics)),
            'pointcloud_topics': sorted(list(self.pointcloud_topics)),
        }

    def list_ports(self) -> Dict:
        """Very simple port scan; you can improve with pyserial later if desired."""
        usb = sorted(glob.glob('/dev/ttyUSB*'))
        acm = sorted(glob.glob('/dev/ttyACM*'))
        realsense = sorted(glob.glob('/dev/video*'))
        return {'ttyUSB': usb, 'ttyACM': acm, 'video': realsense}

    def publish_wasd(self, key: str, linear: float = 0.3, angular: float = 0.9):
        key = key.upper()
        tw = Twist()
        if key == 'W':
            tw.linear.x = linear
        elif key == 'S':
            tw.linear.x = -linear
        elif key == 'A':
            tw.angular.z = angular
        elif key == 'D':
            tw.angular.z = -angular
        elif key == 'X':
            pass  # stop
        else:
            return {'ok': False, 'error': 'invalid key'}
        self.pub_key_vel.publish(tw)
        self._mark_alive()
        return {'ok': True}

    def emergency_stop(self):
        tw = Twist()
        self.pub_cmd_vel.publish(tw)
        self._mark_alive()
        return {'ok': True}

    def nav2_send_goal(self, x: float, y: float, yaw: float) -> Dict:
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return {'ok': False, 'error': 'Nav2 action server not available'}
        from geometry_msgs.msg import PoseStamped, Quaternion
        from builtin_interfaces.msg import Time as RosTime
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        # yaw -> quaternion
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps
        send_future = self.nav_client.send_goal_async(goal)
        # Fire-and-forget for simplicity; production should track result
        return {'ok': True, 'sent': True}

    # ---- Snapshots ----
    def snapshot_image_png(self, topic: str) -> Tuple[bytes, str]:
        self._ensure_image_sub(topic)
        # wait a bit for first frame
        t0 = now()
        while (topic not in self._latest_images) and (now() - t0 < 1.0):
            time.sleep(0.05)
        if topic not in self._latest_images:
            raise RuntimeError('no image received')
        _, msg = self._latest_images[topic]
        # encode to PNG
        if _HAS_CV2:
            import numpy as np
            h = msg.height
            w = msg.width
            ch = 1 if msg.encoding in ('mono8','8UC1') else 3
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if ch == 3:
                # assume bgr8 or rgb8; if bgr8 flip channels
                arr = arr.reshape((h, w, 3))
            else:
                arr = arr.reshape((h, w))
            ok, buf = cv2.imencode('.png', arr)
            if not ok:
                raise RuntimeError('cv2 encode failed')
            return (buf.tobytes(), 'image/png')
        else:
            # fallback using PIL expects RGB; do simple pass-through
            pil = PILImage.frombytes('L', (msg.width, msg.height), bytes(msg.data))
            out = io.BytesIO()
            pil.save(out, format='PNG')
            return (out.getvalue(), 'image/png')

    def snapshot_map_png(self, overlay_path=True, overlay_pose=True) -> Tuple[bytes, str]:
        # wait map
        t0 = now()
        while (self._latest_map is None) and (now() - t0 < 1.0):
            time.sleep(0.05)
        if self._latest_map is None:
            raise RuntimeError('no map data')
        _, m = self._latest_map
        import numpy as np
        if not _HAS_CV2:
            raise RuntimeError('OpenCV required for map render')
        w = m.info.width
        h = m.info.height
        data = np.array(m.data, dtype=np.int16).reshape((h, w))
        # convert occupancy [-1,0..100] → grayscale
        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 205
        occ = data >= 50
        free = data == 0
        img[occ] = 0
        img[free] = 255
        # BGR for drawing
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # overlay path
        if overlay_path and self._latest_path is not None:
            _, path = self._latest_path
            res = m.info.resolution
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            for pose in path.poses:
                px = int((pose.pose.position.x - ox)/res)
                py = int((pose.pose.position.y - oy)/res)
                if 0 <= px < w and 0 <= py < h:
                    cv2.circle(img, (px, h-1-py), 1, (0,0,255), -1)
        # overlay pose as triangle
        if overlay_pose:
            # very simple: draw at center for now (upgrade with TF later or odom)
            cx, cy = w//2, h//2
            cv2.circle(img, (cx, cy), 4, (0,255,255), -1)
        ok, buf = cv2.imencode('.png', img)
        if not ok:
            raise RuntimeError('map encode failed')
        return (buf.tobytes(), 'image/png')

    def snapshot_pointcloud_topdown_png(self, topic: str, size: int = 512, scale: float = 10.0) -> Tuple[bytes, str]:
        self._ensure_pc_sub(topic)
        t0 = now()
        pc_msg = None
        while (pc_msg is None) and (now() - t0 < 1.0):
            time.sleep(0.05)
            if hasattr(self, '_pc_last') and self._pc_last[0] == topic:
                pc_msg = self._pc_last[2]
        if pc_msg is None:
            raise RuntimeError('no pointcloud received')
        import numpy as np
        if not _HAS_CV2:
            raise RuntimeError('OpenCV required for pc render')
        # Very crude reader: assume XYZ float32 contiguous fields
        step = pc_msg.point_step
        buf = memoryview(pc_msg.data)
        n = len(buf)//step
        img = np.zeros((size, size), dtype=np.uint8)
        half = size//2
        for i in range(0, n, max(1, n//50000)):
            base = i*step
            x = float.fromhex(hex(int.from_bytes(buf[base+0:base+4], 'little')))
            y = float.fromhex(hex(int.from_bytes(buf[base+4:base+8], 'little')))
            # quick & dirty: use struct unpack instead for correctness
        import struct
        img = np.zeros((size, size), dtype=np.uint8)
        half = size//2
        for i in range(0, n, max(1, n//60000)):
            base = i*step
            x = struct.unpack('<f', buf[base+0:base+4])[0]
            y = struct.unpack('<f', buf[base+4:base+8])[0]
            u = int(half + x/scale*half)
            v = int(half - y/scale*half)
            if 0 <= u < size and 0 <= v < size:
                img[v, u] = 255
        import cv2
        ok, png = cv2.imencode('.png', img)
        if not ok:
            raise RuntimeError('pc encode failed')
        return (png.tobytes(), 'image/png')

# --------- Spin helper in background thread ---------
class CoreThread:
    def __init__(self):
        self.node = WheeBotCore()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)
        except Exception as e:
            self.node.get_logger().info(f"Spin thread stopped: {e}")

    # Thin wrappers used by HTTP server
    def api(self):
        return self.node


core_singleton = None

def get_core():
    global core_singleton
    if core_singleton is not None:
        return core_singleton.api()
    if not rclpy.ok():
        rclpy.init(args=None)
    core_singleton = CoreThread()
    return core_singleton.api()