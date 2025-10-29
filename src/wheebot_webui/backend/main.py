#!/usr/bin/env python3
import sys
sys.path.insert(0, "/home/rafael/env/yolo_env/lib/python3.12/site-packages")

# import library
import sys, os, glob, json, asyncio, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import cv2
from cv_bridge import CvBridge

# ROS msgs
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from nav2_msgs.action import NavigateToPose

# Converters
try:
    BRIDGE = CvBridge()
except Exception:
    BRIDGE = None
    cv2 = None

# PointCloud2 helper
try:
    from sensor_msgs_py import point_cloud2
except Exception:
    point_cloud2 = None

# initialize API and ROS2
app = FastAPI()
rclpy.init()

# interface ROS2
class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__('wheebot_webui_node')

        # publisher
        self.key_pub = self.create_publisher(Twist, '/key_vel', 10)
        self.estop_pub = self.create_publisher(TwistStamped, '/wheebot_controller/cmd_vel', 10)

        # subscription
        self.subscription = self.create_subscription(TwistStamped, '/wheebot_controller/cmd_vel', self.cmd_vel_callback, 10)

        self.map_sub = None
        self.image_sub = None
        self.scan_sub = None
        self.cloud_sub = None

        # Fixed: odom for robot pose overlay
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # --- State ---
        self.cmd_vel_active = False
        self.last_msg_time = self.get_clock().now()
        self.estop_active = False

        self.map_meta = None      # (width, height, res, origin[x,y])
        self.map_data = None      # list[int 0..100/-1]
        self.robot_pose = None    # (x, y, yaw)

        self.last_image_b64 = None
        self.last_scan = None     # downsampled XY list
        self.last_cloud = None    # downsampled XY list

        # Nav2 Action
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def cmd_vel_callback(self, msg):
        self.cmd_vel_active = True
        self.last_msg_time = self.get_clock().now()

    def odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion
        yaw = self.quat_to_yaw(q)
        self.robot_pose = (float(px), float(py), float(yaw))
    
    def map_cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        self.map_meta = (w, h, res, (ox, oy))
        self.map_data = list(msg.data)

    def image_cb(self, msg: Image):
        if not BRIDGE or not cv2:
            return
        try:
            cv_img = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", cv_img)
            if ok:
                import base64
                self.last_image_b64 = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")
        except Exception as e:
            self.get_logger().warn(f"image_cb error: {e}")

    def scan_cb(self, msg: LaserScan):
        # Convert to XY (downsample to keep UI light)
        points = []
        angle = msg.angle_min
        step = max(1, int(len(msg.ranges) / 360))  # about 360 samples
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if 0.01 < r < msg.range_max:
                x = r * float(self.cos(angle))
                y = r * float(self.sin(angle))
                points.append((x, y))
            angle += msg.angle_increment * step
        self.last_scan = points[:1500]  # cap

    def cloud_cb(self, msg: PointCloud2):
        if not point_cloud2:
            return
        pts = []
        # sample up to ~3000 points
        try:
            count = 0
            for p in point_cloud2.read_points(msg, field_names=("x","y"), skip_nans=True):
                pts.append((float(p[0]), float(p[1])))
                count += 1
                if count >= 3000:
                    break
            self.last_cloud = pts
        except Exception as e:
            self.get_logger().warn(f"cloud_cb error: {e}")

    # --- Helpers ---
    @staticmethod
    def quat_to_yaw(q: Quaternion):
        import math
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def cos(x):
        import math
        return math.cos(x)

    @staticmethod
    def sin(x):
        import math
        return math.sin(x)

    def publish_velocity(self, linear_x, angular_z):
        if not self.estop_active:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.key_pub.publish(twist)
    
    def publish_estop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.estop_pub.publish(msg)
    
    # Dynamic subscribers
    def set_map_topic(self, topic):
        if self.map_sub:
            self.destroy_subscription(self.map_sub)
        self.map_sub = self.create_subscription(OccupancyGrid, topic, self.map_cb, 10)
        self.get_logger().info(f"Subscribed MAP: {topic}")

    def set_image_topic(self, topic):
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
        self.image_sub = self.create_subscription(Image, topic, self.image_cb, 10)
        self.get_logger().info(f"Subscribed IMAGE: {topic}")

    def set_scan_topic(self, topic):
        if self.scan_sub:
            self.destroy_subscription(self.scan_sub)
        self.scan_sub = self.create_subscription(LaserScan, topic, self.scan_cb, 10)
        self.get_logger().info(f"Subscribed SCAN: {topic}")

    def set_cloud_topic(self, topic):
        if self.cloud_sub:
            self.destroy_subscription(self.cloud_sub)
        self.cloud_sub = self.create_subscription(PointCloud2, topic, self.cloud_cb, 10)
        self.get_logger().info(f"Subscribed POINTCLOUD: {topic}")

    # Nav2 goal
    async def send_nav_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("navigate_to_pose action server not available")
            return False, "Nav2 action server not available"

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        import math
        qz = self.sin(yaw/2.0)
        qw = self.cos(yaw/2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        r = await asyncio.wrap_future(send_future)
        if not r.accepted:
            return False, "Goal rejected"
        result_future = await asyncio.wrap_future(r.get_result_async())
        status = result_future.status
        return True, f"Result status: {status}"

ros_node = WebInterfaceNode()

# run executor in background
def ros_spin():
    rclpy.spin(ros_node)
threading.Thread(target=ros_spin, daemon=True).start()

# e-stop loop
async def estop_loop():
    while True:
        if ros_node.estop_active:
            ros_node.publish_estop()
        await asyncio.sleep(0.2)

def port_status():
    return {
        "lidar": os.path.exists("/dev/ttyUSB1"),
        "arduino": os.path.exists("/dev/ttyUSB0"),
        "realsense_videos": sorted(glob.glob("/dev/video*"))[:6]
    }

# interface websocket
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()

    # --- Task 1: Receiver ---
    # Bertugas menerima pesan dari client secepat mungkin
    async def receiver(ws: WebSocket):
        while True:
            try:
                data = await ws.receive_text()
                cmd = json.loads(data)

                t = cmd.get("type")
                if t == "vel":
                    ros_node.publish_velocity(cmd["linear_x"], cmd["angular_z"])

                elif t == "estop":
                    ros_node.estop_active = bool(cmd["active"])
                    if ros_node.estop_active:
                        ros_node.get_logger().info("⚠️ E-STOP ACTIVATED")
                        ros_node.publish_estop()  # force immediately
                    else:
                        ros_node.get_logger().info("✅ E-STOP RELEASED")

                elif t == "set_topics":
                    # { map, image, scan, cloud }
                    if "map" in cmd: ros_node.set_map_topic(cmd["map"])
                    if "image" in cmd: ros_node.set_image_topic(cmd["image"])
                    if "scan" in cmd: ros_node.set_scan_topic(cmd["scan"])
                    if "cloud" in cmd: ros_node.set_cloud_topic(cmd["cloud"])

                elif t == "nav_to_pose":
                    # { x, y, yaw }
                    ok, msg = await ros_node.send_nav_goal(cmd["x"], cmd["y"], cmd.get("yaw", 0.0))
                    await ws.send_json({"type":"nav_feedback", "ok": ok, "msg": msg})


            
            except Exception as e:
                # print(f"Receiver error: {e}") # Debug
                break # Keluar dari loop jika koneksi error

    # --- Task 2: Sender ---
    # Bertugas mengirim status ke client secara periodik
    async def sender(ws: WebSocket):
        while True:
            try:
                # Robot ON/OFF status check
                now = ros_node.get_clock().now()
                dt = (now - ros_node.last_msg_time).nanoseconds / 1e9
                rbt_status = "ON" if dt < 2.0 and ros_node.cmd_vel_active else "OFF"

                payload = {
                    "type": "tick",
                    "rbt_status": rbt_status,
                    "estop": ros_node.estop_active,
                    "ports": port_status(),
                    "image": ros_node.last_image_b64,  # may be None
                    "map": {
                        "meta": ros_node.map_meta,
                        "data": ros_node.map_data[:(ros_node.map_meta[0]*ros_node.map_meta[1])] if ros_node.map_meta and ros_node.map_data else None
                    },
                    "odom": ros_node.robot_pose,
                    "scan_xy": ros_node.last_scan,
                    "cloud_xy": ros_node.last_cloud
                }
                await ws.send_json(payload)
                await asyncio.sleep(0.2)
            
            except Exception as e:
                # print(f"Sender error: {e}") # Debug
                break # Keluar dari loop jika koneksi error

    # --- Menjalankan kedua task secara bersamaan ---
    receiver_task = asyncio.create_task(receiver(ws))
    sender_task = asyncio.create_task(sender(ws))

    try:
        # Tunggu salah satu task selesai (artinya koneksi terputus)
        done, pending = await asyncio.wait(
            {receiver_task, sender_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
        
        # Batalkan task yang masih berjalan
        for task in pending:
            task.cancel()
        
        # Tunggu pembatalan selesai
        await asyncio.gather(*pending, return_exceptions=True)

    except Exception as e:
        print(f"WebSocket connection closed: {e}")
    finally:
        # Pastikan semua task dibatalkan saat keluar
        if not receiver_task.done():
            receiver_task.cancel()
        if not sender_task.done():
            sender_task.cancel()
        print("WebSocket connection cleaned up.")

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(estop_loop())

# cors
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# main
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
