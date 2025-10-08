#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from scipy.optimize import linear_sum_assignment
import time
import math

# -------------------
# Utility helpers
# -------------------
def bbox_to_xywh(bbox):
    x1, y1, x2, y2 = bbox
    w = max(0.0, x2 - x1)
    h = max(0.0, y2 - y1)
    cx = x1 + w / 2.0
    cy = y1 + h / 2.0
    return (cx, cy, w, h)

def iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interW = max(0.0, xB - xA)
    interH = max(0.0, yB - yA)
    if interW <= 0 or interH <= 0:
        return 0.0
    interArea = interW * interH
    boxAArea = max(1e-6, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
    boxBArea = max(1e-6, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))
    return interArea / (boxAArea + boxBArea - interArea)

def clamp_bbox(bbox, w, h):
    x1, y1, x2, y2 = bbox
    x1 = int(max(0, min(x1, w - 1)))
    x2 = int(max(0, min(x2, w - 1)))
    y1 = int(max(0, min(y1, h - 1)))
    y2 = int(max(0, min(y2, h - 1)))
    return (x1, y1, x2, y2)

def parse_detection2d(d):
    try:
        cx = float(d.bbox.center.position.x)
        cy = float(d.bbox.center.position.y)
    except Exception:
        try:
            cx = float(d.bbox.center.x)
            cy = float(d.bbox.center.y)
        except Exception:
            return None

    try:
        w = float(d.bbox.size_x)
        h = float(d.bbox.size_y)
    except Exception:
        return None

    x1 = cx - w / 2.0
    y1 = cy - h / 2.0
    x2 = cx + w / 2.0
    y2 = cy + h / 2.0

    class_field = None
    score = 0.0
    if hasattr(d, 'results') and len(d.results) > 0:
        r = d.results[0]
        try:
            class_field = str(r.hypothesis.class_id)
        except Exception:
            try:
                class_field = str(getattr(r, 'id', ''))
            except Exception:
                class_field = ''
        try:
            score = float(r.hypothesis.score)
        except Exception:
            try:
                score = float(getattr(r, 'score', 0.0))
            except Exception:
                score = 0.0

    return (x1, y1, x2, y2, class_field, score)

# -------------------
# Node
# -------------------
class DynamicStaticClassifier(Node):
    def __init__(self):
        super().__init__('dynamic_static_classifier')
        self.bridge = CvBridge()

        # Parameters (tuneable)
        self.declare_parameter('vel_px_threshold', 2.0)
        self.declare_parameter('ma_window', 5)
        self.declare_parameter('iou_match_thresh', 0.3)
        self.declare_parameter('confirm_dynamic_frames', 3)
        self.declare_parameter('confirm_static_frames', 6)
        self.declare_parameter('min_detection_score', 0.25)
        self.declare_parameter('visualize', True)
        self.declare_parameter('publish_viz', True)
        self.declare_parameter('max_track_age', 5)
        self.declare_parameter('dynamic_class_names', ['person','bicycle','car','motorcycle','dog','cat'])
        self.declare_parameter('static_class_names', ['chair','table','sofa','bed','bench'])

        self.vel_px_threshold = float(self.get_parameter('vel_px_threshold').value)
        self.ma_window = int(self.get_parameter('ma_window').value)
        self.iou_match_thresh = float(self.get_parameter('iou_match_thresh').value)
        self.confirm_dyn = int(self.get_parameter('confirm_dynamic_frames').value)
        self.confirm_stat = int(self.get_parameter('confirm_static_frames').value)
        self.min_score = float(self.get_parameter('min_detection_score').value)
        self.visualize = bool(self.get_parameter('visualize').value)
        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.max_track_age = int(self.get_parameter('max_track_age').value)

        dyn_list = self.get_parameter('dynamic_class_names').value
        stat_list = self.get_parameter('static_class_names').value
        # normalize to strings
        self.dynamic_class_names = set([str(x) for x in dyn_list])
        self.static_class_names = set([str(x) for x in stat_list])

        # tracker state
        self.next_id = 1
        self.tracks = {}   # tid -> dict {bbox, class, score, centroid_hist(deque), dyn_count, stat_count, last_label, age}
        self.max_history = self.ma_window

        # ROS I/O
        # self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        self.det_sub = self.create_subscription(Detection2DArray, '/yolo/detections', self.detections_cb, 10)
        self.pub_image = self.create_publisher(Image, '/segmented_image', 10)
        self.pub_classified = self.create_publisher(Detection2DArray, '/yolo/classified_detections', 10)

        self.last_detections = []
        self.latest_image = None

        # colors
        np.random.seed(1234)
        self.status_colors = {'DYNAMIC': (0, 0, 255), 'STATIC': (255, 0, 0)}  # BGR

        self.get_logger().info('DynamicStaticClassifier started')
        self.get_logger().info(f"Dynamic classes: {sorted(list(self.dynamic_class_names))}")

    # -------------------
    # subscribers
    # -------------------
    def detections_cb(self, msg: Detection2DArray):
        dets = []
        for d in msg.detections:
            parsed = parse_detection2d(d)
            if parsed is None:
                continue
            x1, y1, x2, y2, class_field, score = parsed
            if score < self.min_score:
                continue
            cls_name = class_field if class_field is not None else ''
            dets.append({'bbox': (x1, y1, x2, y2), 'class': cls_name, 'score': float(score)})
        self.last_detections = dets

    def image_cb(self, img_msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        self.latest_image = cv_img

        # If there are no detections this frame, age existing tracks (so they can expire)
        if len(self.last_detections) == 0:
            if len(self.tracks) > 0:
                self._age_tracks()
            if self.publish_viz:
                out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                out_msg.header = img_msg.header
                self.pub_image.publish(out_msg)
            return

        self.process_frame(cv_img, img_msg.header)

    # -------------------
    # helper: age & prune tracks
    # -------------------
    def _age_tracks(self):
        to_delete = []
        for tid, tr in list(self.tracks.items()):
            tr['age'] += 1
            if tr['age'] > self.max_track_age:
                to_delete.append(tid)
        for tid in to_delete:
            self.get_logger().info(f"Removing stale track ID{tid} (age>{self.max_track_age})")
            del self.tracks[tid]

    # -------------------
    # main processing
    # -------------------
    def process_frame(self, frame, header):
        detections = self.last_detections
        if len(detections) == 0:
            return

        H, W = frame.shape[:2]
        bboxes = np.array([d['bbox'] for d in detections], dtype=np.float32)  # Nx4
        classes = [d['class'] for d in detections]
        scores = np.array([d['score'] for d in detections], dtype=np.float32)

        # clamp bboxes to image
        bboxes[:, 0] = np.clip(bboxes[:, 0], 0, W - 1)
        bboxes[:, 1] = np.clip(bboxes[:, 1], 0, H - 1)
        bboxes[:, 2] = np.clip(bboxes[:, 2], 0, W - 1)
        bboxes[:, 3] = np.clip(bboxes[:, 3], 0, H - 1)

        # --- matching: cost matrix (1 - IoU) between existing tracks and detections ---
        track_ids = list(self.tracks.keys())
        if len(track_ids) == 0:
            # initialize tracks
            for i in range(len(bboxes)):
                self._create_track(tuple(bboxes[i]), classes[i], float(scores[i]))
        else:
            T = len(track_ids)
            D = len(bboxes)
            cost = np.ones((T, D), dtype=np.float32)
            for ti, tid in enumerate(track_ids):
                tb = np.array(self.tracks[tid]['bbox'], dtype=np.float32)
                xa1 = np.maximum(tb[0], bboxes[:, 0])
                ya1 = np.maximum(tb[1], bboxes[:, 1])
                xa2 = np.minimum(tb[2], bboxes[:, 2])
                ya2 = np.minimum(tb[3], bboxes[:, 3])

                iw = np.maximum(0.0, xa2 - xa1)
                ih = np.maximum(0.0, ya2 - ya1)
                inter = iw * ih
                area_tb = max(1e-6, (tb[2] - tb[0]) * (tb[3] - tb[1]))
                area_db = np.maximum(1e-6, (bboxes[:, 2] - bboxes[:, 0]) * (bboxes[:, 3] - bboxes[:, 1]))
                union = area_tb + area_db - inter
                ious = inter / union
                cost[ti, :] = 1.0 - ious

            row_idx, col_idx = linear_sum_assignment(cost)
            assigned_tracks = set()
            assigned_dets = set()
            for r, c in zip(row_idx, col_idx):
                if cost[r, c] < (1.0 - self.iou_match_thresh):
                    tid = track_ids[r]
                    assigned_tracks.add(tid)
                    assigned_dets.add(c)
                    self._update_track(tid, tuple(bboxes[c]), classes[c], float(scores[c]))

            # create new tracks for unassigned detections
            for j in range(len(bboxes)):
                if j not in assigned_dets:
                    self._create_track(tuple(bboxes[j]), classes[j], float(scores[j]))

            # age out old tracks
            for tid in track_ids:
                if tid not in assigned_tracks:
                    self.tracks[tid]['age'] += 1
                    if self.tracks[tid]['age'] > self.max_track_age:
                        self.get_logger().info(f"Removing stale track ID{tid} (age>{self.max_track_age})")
                        del self.tracks[tid]

        # --- determine status per track and draw visualization ---
        overlay = frame.copy()
        mask = np.zeros_like(frame, dtype=np.uint8)
        classified_msg = Detection2DArray()
        classified_msg.header = header

        frame_print_lines = []

        for tid, tr in list(self.tracks.items()):
            bbox = tr['bbox']
            cx, cy, w, h = bbox_to_xywh(bbox)
            hist = tr['centroid_hist']
            vel = 0.0
            if len(hist) >= 2:
                diffs = [math.hypot(hist[i][0] - hist[i - 1][0], hist[i][1] - hist[i - 1][1]) for i in range(1, len(hist))]
                vel = float(sum(diffs) / len(diffs)) if diffs else 0.0

            cls_name = tr.get('class', '')
            class_based_dyn = (cls_name in self.dynamic_class_names)
            motion_dyn = vel >= self.vel_px_threshold

            # hysteresis counters
            if motion_dyn or class_based_dyn:
                tr['dyn_count'] += 1
                tr['stat_count'] = 0
            else:
                tr['stat_count'] += 1
                tr['dyn_count'] = 0

            status = 'STATIC'
            if class_based_dyn:
                status = 'DYNAMIC'
            else:
                if tr['dyn_count'] >= self.confirm_dyn:
                    status = 'DYNAMIC'
                elif tr['stat_count'] >= self.confirm_stat:
                    status = 'STATIC'
                else:
                    status = tr.get('last_label', 'STATIC')

            tr['last_label'] = status

            # colors and draw
            color = self.status_colors['DYNAMIC'] if status == 'DYNAMIC' else self.status_colors['STATIC']
            x1, y1, x2, y2 = clamp_bbox(bbox, W, H)
            mask[y1:y2, x1:x2] = color
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            label = f"ID{tid} {cls_name} {status} {vel:.1f}px"
            cv2.putText(overlay, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            frame_print_lines.append(f"ID{tid} | class:{cls_name} | {status} | vel:{vel:.2f}px")

            # build Detection2D to publish downstream
            d2 = Detection2D()
            try:
                d2.bbox.center.position.x = float((x1 + x2) / 2.0)
                d2.bbox.center.position.y = float((y1 + y2) / 2.0)
            except Exception:
                try:
                    d2.bbox.center.x = float((x1 + x2) / 2.0)
                    d2.bbox.center.y = float((y1 + y2) / 2.0)
                except Exception:
                    pass
            d2.bbox.size_x = float(x2 - x1)
            d2.bbox.size_y = float(y2 - y1)

            res = ObjectHypothesisWithPose()
            d2.results.append(res)
            try:
                # IMPORTANT: include status as prefix so downstream nodes can detect DYNAMIC easily
                res.hypothesis.class_id = f"{status}:{cls_name}"
                res.hypothesis.score = float(tr.get('score', 0.0))
            except Exception:
                try:
                    res.hypothesis.class_id = f"{status}:unknown"
                    res.hypothesis.score = float(tr.get('score', 0.0))
                except Exception:
                    pass

            # keep ID in theta field for debugging/traceability
            try:
                d2.bbox.center.theta = float(tid)
            except Exception:
                pass

            classified_msg.detections.append(d2)

        # alpha blend mask for visualization
        out_img = cv2.addWeighted(overlay, 1.0, mask, 0.5, 0)

        # publish image and classified detections
        out_msg = self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8')
        out_msg.header = header
        if self.publish_viz:
            self.pub_image.publish(out_msg)
        self.pub_classified.publish(classified_msg)

        # visualization window
        if self.visualize:
            cv2.imshow("Dynamic/Static Segmentation", out_img)
            cv2.waitKey(1)

        # concise print
        if len(frame_print_lines) == 0:
            self.get_logger().info("No tracks this frame.")
        else:
            print(f"\n[Frame {time.strftime('%H:%M:%S')}] Tracks: {len(frame_print_lines)}")
            for ln in frame_print_lines:
                print("  " + ln)

    # -------------------
    # track management
    # -------------------
    def _create_track(self, bbox, class_name, score):
        tid = self.next_id
        self.next_id += 1
        cx, cy, w, h = bbox_to_xywh(bbox)
        tr = {
            'bbox': bbox,
            'class': class_name if class_name is not None else '',
            'score': float(score),
            'centroid_hist': deque(maxlen=self.max_history),
            'dyn_count': 0,
            'stat_count': 0,
            'last_label': 'STATIC',
            'age': 0
        }
        tr['centroid_hist'].append((cx, cy))
        self.tracks[tid] = tr
        return tid

    def _update_track(self, tid, bbox, class_name, score):
        tr = self.tracks.get(tid, None)
        if tr is None:
            return
        tr['bbox'] = bbox
        tr['class'] = class_name if class_name is not None else tr.get('class', '')
        tr['score'] = float(score)
        cx, cy, w, h = bbox_to_xywh(bbox)
        tr['centroid_hist'].append((cx, cy))
        tr['age'] = 0  # reset age only when we actually updated/assigned the track

# -------------------
# run
# -------------------
def main(args=None):
    rclpy.init(args=args)
    node = DynamicStaticClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
