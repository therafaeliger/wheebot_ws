import sys
import threading
import math
import time
import datetime
import csv
import os
import psutil
import subprocess
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32  # Untuk FPS

import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import filedialog
from PIL import Image as PILImage, ImageTk
import numpy as np
import cv2

# Matplotlib integration
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# ==========================================
# BAGIAN 1: STREAM VIEWER (CAMERA & PCL)
# ==========================================
class StreamViewer:
    def __init__(self, parent_ui, ros_node, title, default_topic, msg_type, qos_profile):
        self.node = ros_node
        self.current_topic = default_topic
        self.msg_type = msg_type
        self.qos_profile = qos_profile
        self.subscription = None
        self.active = False
        self.projection_mode = "XY"
        self.target_height = 250 

        self.frame = tk.Frame(parent_ui, bg="#34495e", bd=2, relief="groove")
        
        # Header
        header_row = tk.Frame(self.frame, bg="#2c3e50")
        header_row.pack(fill=tk.X, padx=2, pady=2)
        tk.Label(header_row, text=title, bg="#2c3e50", fg="#ecf0f1", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=5)
        self.status_lbl = tk.Label(header_row, text="● Offline", bg="#2c3e50", fg="#95a5a6", font=("Arial", 8))
        self.status_lbl.pack(side=tk.RIGHT, padx=5)

        # Input Row
        input_row = tk.Frame(self.frame, bg="#34495e")
        input_row.pack(fill=tk.X, padx=5, pady=2)
        tk.Button(input_row, text="↻", command=self.update_topic, bg="#2980b9", fg="white", relief="flat", font=("Arial", 10, "bold"), width=3).pack(side=tk.RIGHT, padx=(5, 0))
        self.entry = tk.Entry(input_row, bg="#2c3e50", fg="white", insertbackground="white", relief="flat", font=("Consolas", 10))
        self.entry.insert(0, default_topic)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=3)
        self.entry.bind('<Return>', lambda event: self.update_topic())

        # PCL Controls
        if self.msg_type == PointCloud2:
            pcl_row = tk.Frame(self.frame, bg="#34495e")
            pcl_row.pack(fill=tk.X, padx=5, pady=2)
            def mk_btn(text, mode):
                tk.Button(pcl_row, text=text, command=lambda: self.set_proj(mode), bg="#7f8c8d", fg="white", width=5, font=("Arial", 8)).pack(side=tk.LEFT, padx=1)
            mk_btn("XY", "XY"); mk_btn("XZ", "XZ"); mk_btn("YZ", "YZ")
            self.lbl_mode = tk.Label(pcl_row, text="Mode: XY", bg="#34495e", fg="#f1c40f", font=("Arial", 8, "bold"))
            self.lbl_mode.pack(side=tk.RIGHT)

        # Container
        container = tk.Frame(self.frame, bg="black")
        container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.img_label = tk.Label(container, bg="black")
        self.img_label.pack(expand=True)

    def set_proj(self, mode):
        self.projection_mode = mode
        if hasattr(self, 'lbl_mode'): self.lbl_mode.config(text=f"Mode: {mode}")

    def enable(self):
        if not self.active:
            self.subscribe(); self.active = True
    
    def disable(self):
        if self.active:
            self.unsubscribe(); self.active = False

    def subscribe(self):
        self.unsubscribe()
        try:
            self.subscription = self.node.create_subscription(self.msg_type, self.current_topic, self.data_callback, self.qos_profile)
            self.status_lbl.config(text="● Live", fg="#2ecc71")
        except: self.status_lbl.config(text="● Error", fg="#e74c3c")

    def unsubscribe(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
            self.status_lbl.config(text="● Paused", fg="#f39c12")

    def update_topic(self):
        self.current_topic = self.entry.get()
        if self.active: self.subscribe()

    def set_resolution_mode(self, is_large):
        self.target_height = 550 if is_large else 250

    def data_callback(self, msg):
        try:
            if isinstance(msg, PointCloud2): cv_image = self.process_pointcloud(msg)
            else: cv_image = self.process_image(msg)

            h, w = cv_image.shape[:2]
            scale = self.target_height / h if h > 0 else 1
            target_w = int(w * scale)
            
            if target_w > 0:
                interp = cv2.INTER_LINEAR if self.target_height > 400 else cv2.INTER_NEAREST
                resized = cv2.resize(cv_image, (target_w, self.target_height), interpolation=interp)
                cv_img_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                imgtk = ImageTk.PhotoImage(image=PILImage.fromarray(cv_img_rgb))
                self.img_label.imgtk = imgtk
                self.img_label.configure(image=imgtk)
        except: pass

    def process_image(self, msg):
        if '16UC' in msg.encoding or 'mono16' in msg.encoding: dtype=np.uint16; n=1
        else: dtype=np.uint8; n=-1 
        np_arr = np.frombuffer(msg.data, dtype=dtype)
        if n == 1:
            img = np_arr.reshape((msg.height, msg.width))
            img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
            img = np.uint8(img)
            img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        else:
            img = np_arr.reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8': img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def process_pointcloud(self, msg):
        SIZE = 600 if self.target_height > 400 else 300 
        canvas = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        x_off, y_off, z_off = -1,-1,-1
        for f in msg.fields:
            if f.name=='x': x_off=f.offset
            if f.name=='y': y_off=f.offset
            if f.name=='z': z_off=f.offset
        if x_off==-1: return canvas
        x = raw[:, x_off:x_off+4].view(np.float32).flatten()
        y = raw[:, y_off:y_off+4].view(np.float32).flatten()
        z = raw[:, z_off:z_off+4].view(np.float32).flatten()
        mask = ~np.isnan(x) & ~np.isnan(y) & ~np.isnan(z)
        x=x[mask]; y=y[mask]; z=z[mask]
        if len(x)==0: return canvas
        ppm = SIZE / 10.0; cx, cy = SIZE//2, SIZE//2
        u, v = None, None
        if self.projection_mode == "XY":
            u = cx - (y * ppm); v = cy - (x * ppm)
            cv2.arrowedLine(canvas, (cx, cy+10), (cx, cy-10), (0,0,255), 2)
        elif self.projection_mode == "XZ":
            u = 50 + (x * ppm); v = (SIZE-50) - (z * ppm)
            cv2.line(canvas, (0, SIZE-50), (SIZE, SIZE-50), (100,100,100), 1)
        elif self.projection_mode == "YZ": 
             f = SIZE; x_c=-y; y_c=-z; z_c=x
             u = (x_c * f / (z_c+0.001)) + cx
             v = (y_c * f / (z_c+0.001)) + cy
        if u is not None:
            u=u.astype(np.int32); v=v.astype(np.int32)
            valid=(u>=0)&(u<SIZE)&(v>=0)&(v<SIZE)
            canvas[v[valid], u[valid]] = (0,255,0)
            canvas = cv2.dilate(canvas, np.ones((2,2),np.uint8))
        return canvas

# ==========================================
# BAGIAN 2: MAP VIEWER (DENGAN ODOMETRY)
# ==========================================
class MapViewer:
    def __init__(self, parent_ui, ros_node):
        self.node = ros_node
        self.frame = tk.Frame(parent_ui, bg="#2c3e50")
        self.frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.current_topic = "/map"
        self.odom_topic = "/odom" 
        self.subscription_map = None
        self.subscription_odom = None
        
        self.map_data = None    
        self.map_info = None    
        self.robot_pose = None  

        self.map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.odom_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Header Bar
        header = tk.Frame(self.frame, bg="#1a252f", height=50)
        header.pack(side=tk.TOP, fill=tk.X)
        tk.Label(header, text="NAVIGATION MAP", bg="#1a252f", fg="white", font=("Arial", 12, "bold")).pack(side=tk.LEFT, padx=10)

        # Input Topic Area
        input_frame = tk.Frame(header, bg="#1a252f")
        input_frame.pack(side=tk.LEFT, padx=20)
        
        tk.Label(input_frame, text="Map:", bg="#1a252f", fg="gray", font=("Arial", 8)).pack(side=tk.LEFT)
        self.entry = tk.Entry(input_frame, bg="#34495e", fg="white", insertbackground="white", relief="flat", font=("Consolas", 10), width=20)
        self.entry.insert(0, self.current_topic)
        self.entry.pack(side=tk.LEFT, padx=5, ipady=3)
        self.entry.bind('<Return>', lambda event: self.update_topic())

        tk.Label(input_frame, text="Odom:", bg="#1a252f", fg="gray", font=("Arial", 8)).pack(side=tk.LEFT, padx=(10,0))
        self.entry_odom = tk.Entry(input_frame, bg="#34495e", fg="white", insertbackground="white", relief="flat", font=("Consolas", 10), width=15)
        self.entry_odom.insert(0, self.odom_topic)
        self.entry_odom.pack(side=tk.LEFT, padx=5, ipady=3)
        self.entry_odom.bind('<Return>', lambda event: self.update_topic())

        btn = tk.Button(input_frame, text="↻", command=self.update_topic, bg="#2980b9", fg="white", font=("Arial", 10, "bold"), width=3)
        btn.pack(side=tk.LEFT)

        self.status = tk.Label(header, text="Offline", bg="#1a252f", fg="orange", font=("Arial", 10))
        self.status.pack(side=tk.RIGHT, padx=10)

        # Container Map
        self.map_container = tk.Frame(self.frame, bg="black")
        self.map_container.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        self.img_label = tk.Label(self.map_container, bg="black")
        self.img_label.pack(expand=True, fill=tk.BOTH)

        self.subscribe()

    def update_topic(self):
        self.current_topic = self.entry.get()
        self.odom_topic = self.entry_odom.get()
        self.subscribe()

    def subscribe(self):
        if self.subscription_map: self.node.destroy_subscription(self.subscription_map)
        if self.subscription_odom: self.node.destroy_subscription(self.subscription_odom)
        
        try:
            self.subscription_map = self.node.create_subscription(OccupancyGrid, self.current_topic, self.map_callback, self.map_qos)
            self.subscription_odom = self.node.create_subscription(Odometry, self.odom_topic, self.odom_callback, self.odom_qos)
            self.status.config(text=f"Live: {self.current_topic} & {self.odom_topic}", fg="#2ecc71")
        except Exception as e:
            self.status.config(text=f"Error: {e}", fg="red")

    def map_callback(self, msg):
        try:
            self.map_info = msg.info
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            img = np.full((height, width), 127, dtype=np.uint8) 
            img[data == 0] = 255   
            img[data == 100] = 0   
            self.map_data = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            self.draw_map_with_robot()
        except: self.status.config(text=f"Map Error", fg="red")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = (x, y, yaw)
        self.draw_map_with_robot()

    def draw_map_with_robot(self):
        if self.map_data is None: return
        display_img = self.map_data.copy()
        if self.map_info is not None and self.robot_pose is not None:
            rx, ry, ryaw = self.robot_pose
            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            res = self.map_info.resolution
            px = int((rx - origin_x) / res)
            py = int((ry - origin_y) / res)
            h, w, _ = display_img.shape
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(display_img, (px, py), 3, (0, 0, 255), -1) 
                arrow_len = 10
                end_x = int(px + arrow_len * math.cos(ryaw))
                end_y = int(py + arrow_len * math.sin(ryaw))
                cv2.arrowedLine(display_img, (px, py), (end_x, end_y), (0, 0, 255), 2, tipLength=0.3)
        display_img = cv2.flip(display_img, 0)
        screen_w = self.map_container.winfo_width()
        screen_h = self.map_container.winfo_height()
        if screen_w < 100: screen_w = 800
        if screen_h < 100: screen_h = 600
        h, w, _ = display_img.shape
        scale = min(screen_w / w, screen_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = cv2.resize(display_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        imgtk = ImageTk.PhotoImage(image=PILImage.fromarray(resized))
        self.img_label.imgtk = imgtk
        self.img_label.configure(image=imgtk)
        if self.robot_pose:
            self.status.config(text=f"Robot: X={self.robot_pose[0]:.2f}, Y={self.robot_pose[1]:.2f}", fg="#2ecc71")

# ==========================================
# BAGIAN 3: DASHBOARD TAB (Adaptive Grid)
# ==========================================
class DashboardTab:
    def __init__(self, parent_notebook, ros_node):
        self.frame = tk.Frame(parent_notebook, bg="#2c3e50")
        self.node = ros_node
        
        ctrl = tk.Frame(self.frame, bg="#1a252f", height=50)
        ctrl.pack(side=tk.TOP, fill=tk.X)
        tk.Label(ctrl, text="MONITORING", font=("Impact", 14), bg="#1a252f", fg="#3498db").pack(side=tk.LEFT, padx=10)
        chk_panel = tk.Frame(ctrl, bg="#1a252f")
        chk_panel.pack(side=tk.RIGHT, padx=10)

        self.content = tk.Frame(self.frame, bg="#2c3e50")
        self.content.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        qp = ros_node.qos_profile
        self.v = []
        self.v.append(StreamViewer(self.content, self.node, "RGB 1", "/camera/color/image_raw", Image, qp))
        self.v.append(StreamViewer(self.content, self.node, "Depth 1", "/camera/depth/image_rect_raw", Image, qp))
        self.v.append(StreamViewer(self.content, self.node, "PCL 1", "/dor/pointcloud/dynamic_only", PointCloud2, qp))
        self.v.append(StreamViewer(self.content, self.node, "RGB 2", "/camera/color/image_raw_2", Image, qp))
        self.v.append(StreamViewer(self.content, self.node, "Depth 2", "/camera/depth/image_rect_raw_2", Image, qp))
        self.v.append(StreamViewer(self.content, self.node, "PCL 2", "/dor/pointcloud/dynamic_only_2", PointCloud2, qp))

        self.vars = [tk.BooleanVar(value=True) for _ in range(6)]
        
        names = ["RGB 1", "Depth 1", "PCL 1", "RGB 2", "Depth 2", "PCL 2"]
        for i in range(6):
            r = 0 if i < 3 else 1
            c = i % 3
            ttk.Checkbutton(chk_panel, text=names[i], variable=self.vars[i], command=self.update_layout).grid(row=r, column=c, padx=5)

        self.update_layout()

    def update_layout(self):
        for i in range(3): self.content.grid_columnconfigure(i, weight=0)
        for i in range(2): self.content.grid_rowconfigure(i, weight=0)

        active_r1 = any([self.vars[0].get(), self.vars[1].get(), self.vars[2].get()])
        active_r2 = any([self.vars[3].get(), self.vars[4].get(), self.vars[5].get()])
        active_c1 = any([self.vars[0].get(), self.vars[3].get()])
        active_c2 = any([self.vars[1].get(), self.vars[4].get()])
        active_c3 = any([self.vars[2].get(), self.vars[5].get()])

        if active_r1: self.content.grid_rowconfigure(0, weight=1)
        if active_r2: self.content.grid_rowconfigure(1, weight=1)
        if active_c1: self.content.grid_columnconfigure(0, weight=1)
        if active_c2: self.content.grid_columnconfigure(1, weight=1)
        if active_c3: self.content.grid_columnconfigure(2, weight=1)

        total_active = sum([v.get() for v in self.vars])
        is_large = (total_active <= 2)

        for i, viewer in enumerate(self.v):
            if self.vars[i].get():
                viewer.enable()
                viewer.set_resolution_mode(is_large)
                viewer.frame.grid(row=0 if i<3 else 1, column=i%3, sticky="nsew", padx=3, pady=3)
            else:
                viewer.disable()
                viewer.frame.grid_forget()

# ==========================================
# BAGIAN 4: PERFORMANCE MONITORING TAB
# ==========================================
class PerformanceTab:
    def __init__(self, parent_notebook, ros_node):
        self.node = ros_node
        self.frame = tk.Frame(parent_notebook, bg="#2c3e50")
        
        self.is_logging = False
        self.review_mode = False # Mode untuk melihat data lama
        self.data_buffer_len = 60 
        
        # Data buffers
        self.time_history = collections.deque(maxlen=self.data_buffer_len)
        self.fps_history = collections.deque(maxlen=self.data_buffer_len)
        self.cpu_history = collections.deque(maxlen=self.data_buffer_len)
        self.ram_history = collections.deque(maxlen=self.data_buffer_len)
        self.gpu_history = collections.deque(maxlen=self.data_buffer_len)
        self.vram_history = collections.deque(maxlen=self.data_buffer_len)

        self.curr_fps = 0.0
        
        # CSV handlers
        self.csv_file_fps = None
        self.csv_file_sys = None
        self.csv_writer_fps = None
        self.csv_writer_sys = None
        self.start_time = None

        # --- UI Layout ---
        ctrl_frame = tk.Frame(self.frame, bg="#1a252f", height=50)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X)
        
        tk.Label(ctrl_frame, text="PERFORMANCE", font=("Impact", 14), bg="#1a252f", fg="#e74c3c").pack(side=tk.LEFT, padx=10)
        
        # Tombol Start/Stop Log
        self.btn_start = tk.Button(ctrl_frame, text="▶ START LOG", command=self.start_logging, bg="#27ae60", fg="white", font=("Arial", 9, "bold"))
        self.btn_start.pack(side=tk.LEFT, padx=5)
        
        self.btn_stop = tk.Button(ctrl_frame, text="■ STOP", command=self.stop_logging, bg="#c0392b", fg="white", font=("Arial", 9, "bold"), state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=5)

        # Tombol Load CSV (Fitur Baru)
        self.btn_load = tk.Button(ctrl_frame, text="📂 LOAD CSV", command=self.load_csv_data, bg="#8e44ad", fg="white", font=("Arial", 9, "bold"))
        self.btn_load.pack(side=tk.LEFT, padx=5)

        self.btn_reset = tk.Button(ctrl_frame, text="⟳ RESET", command=self.reset_data, bg="#f39c12", fg="white", font=("Arial", 9, "bold"))
        self.btn_reset.pack(side=tk.LEFT, padx=5)

        self.status_lbl = tk.Label(ctrl_frame, text="Ready", bg="#1a252f", fg="#95a5a6", font=("Consolas", 10))
        self.status_lbl.pack(side=tk.RIGHT, padx=10)

        # Config Frame
        cfg_frame = tk.Frame(ctrl_frame, bg="#1a252f")
        cfg_frame.pack(side=tk.LEFT, padx=20)
        tk.Label(cfg_frame, text="FPS Topic:", bg="#1a252f", fg="gray").pack(side=tk.LEFT)
        self.fps_entry = tk.Entry(cfg_frame, bg="#34495e", fg="white", width=20)
        self.fps_entry.insert(0, "/yolo/fps")
        self.fps_entry.pack(side=tk.LEFT, padx=5)
        tk.Button(cfg_frame, text="Set", command=self.update_sub, bg="#2980b9", fg="white", height=1).pack(side=tk.LEFT)

        # Plotting Area
        content_frame = tk.Frame(self.frame, bg="#2c3e50")
        content_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.fig.patch.set_facecolor('#2c3e50')
        
        gs = self.fig.add_gridspec(2, 1)
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.ax2 = self.fig.add_subplot(gs[1, 0])

        self.setup_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=content_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.sub_fps = None
        self.update_sub()

        self.monitoring_thread = None
        self.update_graph_loop()

    def setup_axes(self):
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_title("Inference Speed (FPS)", fontsize=10, color="white")
        self.ax1.set_ylabel("FPS")
        self.ax1.grid(True, linestyle='--', alpha=0.3)
        
        self.ax2.set_title("System Usage (%)", fontsize=10, color="white")
        self.ax2.set_ylabel("Percent")
        self.ax2.set_xlabel("Time (Seconds)")
        self.ax2.set_ylim(0, 100)
        self.ax2.grid(True, linestyle='--', alpha=0.3)

    def update_sub(self):
        if self.sub_fps: self.node.destroy_subscription(self.sub_fps)
        topic = self.fps_entry.get()
        try:
            self.sub_fps = self.node.create_subscription(Float32, topic, self.fps_cb, 10)
        except: pass

    def fps_cb(self, msg):
        self.curr_fps = msg.data

    def get_gpu_usage(self):
        try:
            result = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
                 "--format=csv,noheader,nounits"], encoding="utf-8")
            gpu_util, mem_used, mem_total = map(float, result.strip().split(','))
            return gpu_util, mem_used / mem_total * 100
        except: return 0.0, 0.0

    def background_monitor(self):
        while self.is_logging:
            now = time.time() - self.start_time
            cpu = psutil.cpu_percent(interval=None)
            ram = psutil.virtual_memory().percent
            gpu, vram = self.get_gpu_usage()
            
            # Update buffers (hanya jika TIDAK dalam review mode)
            if not self.review_mode:
                self.time_history.append(now)
                self.fps_history.append(self.curr_fps)
                self.cpu_history.append(cpu)
                self.ram_history.append(ram)
                self.gpu_history.append(gpu)
                self.vram_history.append(vram)

            # CSV Logging
            ts_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            if self.csv_file_fps:
                self.csv_writer_fps.writerow([ts_str, f"{self.curr_fps:.2f}"])
                self.csv_file_fps.flush()
            if self.csv_file_sys:
                self.csv_writer_sys.writerow([ts_str, cpu, ram, gpu, vram])
                self.csv_file_sys.flush()
            time.sleep(1)

    # --- FITUR BARU: LOAD CSV ---
    def load_csv_data(self):
        # Stop live logging jika sedang berjalan
        if self.is_logging: self.stop_logging()
        
        # Minta user memilih file (Bisa pilih 2 file sekaligus: FPS dan System)
        filenames = filedialog.askopenfilenames(
            title="Select Logs (Select FPS and/or System CSV)",
            filetypes=[("CSV Files", "*.csv")]
        )
        
        if not filenames: return

        # Reset data
        self.review_mode = True
        self.status_lbl.config(text="Mode: REVIEW (Static)", fg="#f1c40f")
        
        # Tempat penyimpanan sementara
        t_fps, d_fps = [], []
        t_sys, d_cpu, d_ram, d_gpu, d_vram = [], [], [], [], []

        try:
            for fname in filenames:
                with open(fname, 'r') as f:
                    reader = csv.reader(f)
                    header = next(reader) # Skip header
                    
                    rows = list(reader)
                    if not rows: continue
                    
                    # Deteksi tipe file berdasarkan header
                    is_sys = "cpu_percent" in header
                    is_fps = "fps" in header

                    # Ambil waktu mulai dari baris pertama untuk normalisasi waktu (t=0)
                    start_dt = datetime.datetime.strptime(rows[0][0], "%Y-%m-%d %H:%M:%S.%f")

                    for row in rows:
                        dt = datetime.datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S.%f")
                        rel_time = (dt - start_dt).total_seconds()

                        if is_fps:
                            t_fps.append(rel_time)
                            d_fps.append(float(row[1]))
                        elif is_sys:
                            t_sys.append(rel_time)
                            d_cpu.append(float(row[1]))
                            d_ram.append(float(row[2]))
                            d_gpu.append(float(row[3]))
                            d_vram.append(float(row[4]))

            # Plotting Manual untuk Mode Review
            self.ax1.clear(); self.ax2.clear()
            self.setup_axes()

            if t_fps:
                self.ax1.plot(t_fps, d_fps, color='#00d2d3', linewidth=2, label="FPS (Log)")
                self.ax1.fill_between(t_fps, d_fps, color='#00d2d3', alpha=0.1)
                self.ax1.legend(loc="upper right")
            
            if t_sys:
                self.ax2.plot(t_sys, d_cpu, label="CPU", color='#ff6b6b')
                self.ax2.plot(t_sys, d_ram, label="RAM", color='#54a0ff')
                self.ax2.plot(t_sys, d_gpu, label="GPU", color='#1dd1a1')
                self.ax2.plot(t_sys, d_vram, label="VRAM", color='#feca57')
                self.ax2.legend(loc="upper right")

            self.canvas.draw()
            messagebox.showinfo("Success", f"Loaded {len(filenames)} file(s) successfully.")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to parse CSV: {str(e)}")
            self.reset_data() # Kembali ke mode live jika gagal

    def start_logging(self):
        if self.is_logging: return
        self.review_mode = False # Pastikan bukan mode review
        self.reset_data(clear_axes=False) # Bersihkan buffer tapi jangan reset axes dulu

        # Setup CSV
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        base_path = "/home/rfid/wheebot_ws/src/wheebot_dashboard/dashboard_monitoring/results/"
        if not os.path.exists(base_path): base_path = "./" 

        fps_path = f"{base_path}fps_log_{timestamp}.csv"
        sys_path = f"{base_path}system_usage_{timestamp}.csv"

        try:
            self.csv_file_fps = open(fps_path, 'w', newline='')
            self.csv_writer_fps = csv.writer(self.csv_file_fps)
            self.csv_writer_fps.writerow(["timestamp", "fps"])

            self.csv_file_sys = open(sys_path, 'w', newline='')
            self.csv_writer_sys = csv.writer(self.csv_file_sys)
            self.csv_writer_sys.writerow(["timestamp", "cpu_percent", "ram_percent", "gpu_percent", "vram_percent"])
            
            self.status_lbl.config(text=f"Rec: ...{timestamp}.csv", fg="#2ecc71")
        except Exception as e:
            self.status_lbl.config(text=f"CSV Error: {e}", fg="red")

        self.is_logging = True
        self.start_time = time.time()
        self.btn_start.config(state=tk.DISABLED, bg="#7f8c8d")
        self.btn_stop.config(state=tk.NORMAL, bg="#c0392b")
        self.btn_load.config(state=tk.DISABLED) # Disable load saat logging
        
        if not self.monitoring_thread or not self.monitoring_thread.is_alive():
            self.monitoring_thread = threading.Thread(target=self.background_monitor, daemon=True)
            self.monitoring_thread.start()

    def stop_logging(self):
        self.is_logging = False
        if self.csv_file_fps: self.csv_file_fps.close()
        if self.csv_file_sys: self.csv_file_sys.close()
        
        self.btn_start.config(state=tk.NORMAL, bg="#27ae60")
        self.btn_stop.config(state=tk.DISABLED, bg="#7f8c8d")
        self.btn_load.config(state=tk.NORMAL) # Enable load kembali
        self.status_lbl.config(text="Logging Stopped", fg="#f39c12")

    def reset_data(self, clear_axes=True):
        self.review_mode = False # Kembali ke mode live
        self.time_history.clear()
        self.fps_history.clear()
        self.cpu_history.clear()
        self.ram_history.clear()
        self.gpu_history.clear()
        self.vram_history.clear()
        
        if clear_axes:
            self.setup_axes()
            self.canvas.draw()
            self.status_lbl.config(text="Live Monitor Ready", fg="white")
        
        self.btn_load.config(state=tk.NORMAL)

    def update_graph_loop(self):
        # HANYA update grafik jika TIDAK dalam Review Mode dan ada data baru
        if not self.review_mode and self.is_logging and len(self.time_history) > 1:
            self.ax1.clear()
            self.ax2.clear()
            
            t = list(self.time_history)
            
            # Plot FPS
            self.ax1.plot(t, list(self.fps_history), color='#00d2d3', linewidth=2, label="FPS")
            self.ax1.fill_between(t, list(self.fps_history), color='#00d2d3', alpha=0.1)
            self.ax1.set_title(f"Inference Speed: {self.curr_fps:.1f} FPS", fontsize=10, color="white")
            self.ax1.legend(loc="upper right", fontsize=8)
            self.ax1.grid(True, linestyle='--', alpha=0.3)
            
            # Plot System
            self.ax2.plot(t, list(self.cpu_history), label="CPU", color='#ff6b6b')
            self.ax2.plot(t, list(self.ram_history), label="RAM", color='#54a0ff')
            self.ax2.plot(t, list(self.gpu_history), label="GPU", color='#1dd1a1')
            self.ax2.plot(t, list(self.vram_history), label="VRAM", color='#feca57')
            
            self.ax2.set_title("System Usage", fontsize=10, color="white")
            self.ax2.set_ylim(0, 105)
            self.ax2.legend(loc="upper right", fontsize=8, ncol=4)
            self.ax2.grid(True, linestyle='--', alpha=0.3)
            
            self.canvas.draw()
        
        self.frame.after(1000, self.update_graph_loop)

# ==========================================
# MAIN APP
# ==========================================
class RobotDashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

def run_ros_node(node): rclpy.spin(node)

def main():
    root = tk.Tk()
    root.title("Wheebot Control Center")
    root.geometry("1300x850")
    root.configure(bg="#2c3e50")
    
    style = ttk.Style()
    style.theme_use('clam')
    style.configure("TCheckbutton", background="#1a252f", foreground="white")
    style.configure("TNotebook", background="#2c3e50", borderwidth=0)
    style.configure("TNotebook.Tab", background="#34495e", foreground="white", padding=[10, 5], font=("Arial", 10))
    style.map("TNotebook.Tab", background=[("selected", "#3498db")], foreground=[("selected", "white")])

    app_header = tk.Frame(root, bg="#1a252f", height=50)
    app_header.pack(side=tk.TOP, fill=tk.X)
    tk.Label(app_header, text="WHEEBOT UI", font=("Impact", 20), bg="#1a252f", fg="#f1c40f").pack(side=tk.LEFT, padx=20, pady=10)

    rclpy.init()
    node = RobotDashboardNode()

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    # Tab 1: Streams
    tab1 = DashboardTab(notebook, node)
    notebook.add(tab1.frame, text="  Stream Monitor  ")

    # Tab 2: Map
    tab2_frame = tk.Frame(notebook, bg="#2c3e50")
    notebook.add(tab2_frame, text="  Navigation Map  ")
    map_viewer = MapViewer(tab2_frame, node)

    # Tab 3: Performance (BARU)
    tab3 = PerformanceTab(notebook, node)
    notebook.add(tab3.frame, text="  Performance  ")

    threading.Thread(target=run_ros_node, args=(node,), daemon=True).start()

    try: root.mainloop()
    except: pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__': main()
