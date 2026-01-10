#!/usr/bin/env python3

import sys, os
home = os.path.expanduser("~")
sys.path.insert(0, f"{home}/env/yolo_env/lib/python3.12/site-packages")

import threading
import time
import traceback
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from std_msgs.msg import Float32


try:
    from ultralytics import YOLO
except ImportError as e:
    print(f"Gagal import YOLO. Pastikan path '{home}/env/yolo_env/...' benar. Error: {e}")
    sys.exit(1)

class ObjectDetectionAndClassification(Node):
    def __init__(self):
        super().__init__('object_detection_and_classification')

        self.declare_parameter('yolo_model', 'yolov8n-seg.pt')
        self.declare_parameter('dynamic_classes', ['person','car','truck','motorbike','bicycle','bus','dog','cat'])
        self.declare_parameter('static_classes', ['chair','table','sofa','monitor','tv','bed','refrigerator'])
        self.declare_parameter('mask_dilate', 5)
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('show_debug', True)
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_image_topic', '/camera/image')
        self.declare_parameter('inference_size', 416) # Optimasi: Inference size dikecilkan (416 atau 320) untuk boost FPS di edge device

        model_path = self.get_parameter('yolo_model').value
        self.dynamic_classes = set(self.get_parameter('dynamic_classes').value)
        self.static_classes = set(self.get_parameter('static_classes').value)
        self.mask_dilate = int(self.get_parameter('mask_dilate').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_image_topic = self.get_parameter('camera_image_topic').value
        self.inference_size = int(self.get_parameter('inference_size').value)


        self.get_logger().info(f"Loading YOLO Model: {model_path} with imgsz={self.inference_size}...")
        try:
            self.model = YOLO(model_path)
            self.model.predict(source=np.zeros((self.inference_size, self.inference_size, 3), dtype=np.uint8), verbose=False)
        except Exception as e:
            self.get_logger().error(f"Gagal memuat model YOLO: {e}")
            raise e
            
        self.bridge = CvBridge()
        self.camera_frame = 'camera_color_optical_frame'

        # --- SUBSCRIBRS ---
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.cameraInfoCallback, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.camera_image_topic,
            self.image_listener_callback,
            qos_profile=sensor_qos
        )

        # --- PUBLISHERS ---
        self.detection_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.static_pub = self.create_publisher(Image, '/yolo/mask/static_only', 10)
        self.dynamic_pub = self.create_publisher(Image, '/yolo/mask/dynamic_only', 10)
        self.viz_pub = self.create_publisher(Image, '/yolo/viz', 10)
        self.fps_pub = self.create_publisher(Float32, '/yolo/fps', 10)

        # --- THREADING VARIABLES (DROP FRAME LOGIC) ---
        self.last_msg = None
        self.process_lock = threading.Lock()
        
        self.processing_thread = threading.Thread(target=self.processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info(f"----------------------------------------------------------------")
        self.get_logger().info(f"Object Detection and Classification Node started, model: {model_path}")
        self.get_logger().info(f"Subscribing to Camera Info: {self.camera_info_topic}")
        self.get_logger().info(f"Subscribing to Image: {self.camera_image_topic}")
        self.get_logger().info(f"----------------------------------------------------------------")

    def cameraInfoCallback(self, msg: CameraInfo):
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    def image_listener_callback(self, msg: Image):
        with self.process_lock:
            self.last_msg = msg

    def processing_loop(self):
        while rclpy.ok():
            try:
                msg_to_process = None
                with self.process_lock:
                    if self.last_msg is not None:
                        msg_to_process = self.last_msg
                        self.last_msg = None 
                
                if msg_to_process is not None:
                    self.process_image(msg_to_process)
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f"Error in processing loop: {e}")
                # Jangan biarkan thread mati, lanjutkan loop
                time.sleep(0.1)

    def process_image(self, msg: Image):
        t0 = time.time()
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        h, w = frame.shape[:2]

        try:
            results = self.model(frame, verbose=False, conf=self.conf_thres, imgsz=self.inference_size, retina_masks=True)
        except Exception as e:
            self.get_logger().error(f"YOLO Inference Error: {e}")
            return

        res = results[0]
        boxes = res.boxes
        masks = res.masks
        names = self.model.names

        dynamic_mask = np.zeros((h, w), dtype=np.uint8)
        static_mask = np.zeros((h, w), dtype=np.uint8)
        
        viz = frame.copy()
        overlay_layer = np.zeros_like(viz) 
        has_overlay = False

        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.header.frame_id = self.camera_frame or msg.header.frame_id

        if masks is not None and boxes is not None and len(masks.data) > 0:
            masks_np = masks.data.cpu().numpy()
            xyxy_np = boxes.xyxy.cpu().numpy().astype(int)
            cls_np = boxes.cls.cpu().numpy().astype(int)
            conf_np = boxes.conf.cpu().numpy()

            for i in range(len(masks_np)):
                cls_id = cls_np[i]
                conf = float(conf_np[i])
                label = names.get(cls_id, str(cls_id))
                
                # Resize mask
                mask_raw = masks_np[i]
                if mask_raw.shape[:2] != (h, w):
                     mask_u8 = (cv2.resize(mask_raw, (w, h), interpolation=cv2.INTER_NEAREST) > 0.5).astype(np.uint8) * 255
                else:
                     mask_u8 = (mask_raw > 0.5).astype(np.uint8) * 255

                color = (255, 255, 0)
                is_dynamic = label in self.dynamic_classes
                is_static = label in self.static_classes

                if is_dynamic:
                    cv2.bitwise_or(dynamic_mask, mask_u8, dst=dynamic_mask)
                    color = (0, 0, 255) 
                elif is_static:
                    cv2.bitwise_or(static_mask, mask_u8, dst=static_mask)
                    color = (0, 255, 0)

                # Viz
                x1, y1, x2, y2 = xyxy_np[i]
                cv2.rectangle(viz, (x1, y1), (x2, y2), color, 2)
                label_text = f"{label} {conf:.2f}"
                cv2.putText(viz, label_text, (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                if mask_u8.max() > 0:
                    bool_mask = mask_u8 > 0
                    overlay_layer[bool_mask] = color
                    has_overlay = True

                # Detection MSG
                det = Detection2D()
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

            # --- POST-PROCESSING ---
            if self.mask_dilate > 0:
                k = np.ones((self.mask_dilate, self.mask_dilate), np.uint8)
                dynamic_mask = cv2.dilate(dynamic_mask, k, iterations=1)
                static_mask = cv2.dilate(static_mask, k, iterations=1)

            if has_overlay:
                cv2.addWeighted(viz, 1.0, overlay_layer, 0.3, 0, dst=viz)

        # --- PUBLISH ---
        dyn_msg = self.bridge.cv2_to_imgmsg(dynamic_mask, encoding='mono8')
        dyn_msg.header = msg.header
        self.dynamic_pub.publish(dyn_msg)

        stat_msg = self.bridge.cv2_to_imgmsg(static_mask, encoding='mono8')
        stat_msg.header = msg.header
        self.static_pub.publish(stat_msg)

        self.detection_pub.publish(det_array)

        viz_msg = self.bridge.cv2_to_imgmsg(viz, encoding='bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)

        # --- FPS ---
        if self.show_debug:
            fps = 1.0 / max(1e-3, (time.time() - t0))
            self.get_logger().info(f"FPS: {fps:.1f} | Size: {self.inference_size}")
            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_pub.publish(fps_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObjectDetectionAndClassification()
        rclpy.spin(node)
    except Exception as e:
        print("--------------------------------------------------")
        print("NODE DIED WITH EXCEPTION:")
        traceback.print_exc()
        print("--------------------------------------------------")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()