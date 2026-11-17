"""Perception node — fused lane detection + YOLO obstacle fusion.

Unified rewrite (2026-05-18) that combines:
  * HLS color masking (white + yellow lane markings) — approach 2
  * Canny edge detection on the L channel — new, for low-light robustness
  * Sliding-window polynomial fit                  — approach 2
  * Per-fit sanity checks                          — new (a, b, lane width)
  * EMA smoothing of polynomial coefficients       — approach 2
  * Confidence scoring + multi-frame debounce      — new (anti-chatter)
  * YOLO ego-lane filter for obstacle detection    — approach 2

External ROS contract preserved byte-identical:
    /lane_error            std_msgs/Float32
    /lane_found            std_msgs/Bool       (now DEBOUNCED)
    /obstacle_ahead        std_msgs/Bool       (now DEBOUNCED)
    /yolo_blindspot_right  std_msgs/Bool       (now DEBOUNCED)
    /perception_debug      sensor_msgs/Image
    /lane_error_norm       std_msgs/Float32
    /lane_curvature        std_msgs/Float32
    /lane_poly             std_msgs/Float32MultiArray

NEW topic (additive, optional):
    /lane_confidence       std_msgs/Float32    in [0.0, 1.0]
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from .common import (
    CAM_W,
    LANE_BACKEND_DEFAULT,
    ROI_TOP_FRAC,
    HLS_WHITE_L_MIN, HLS_WHITE_S_MAX,
    HLS_YELLOW_H_MIN, HLS_YELLOW_H_MAX,
    HLS_YELLOW_S_MIN, HLS_YELLOW_L_MIN,
    CANNY_LOW, CANNY_HIGH, CANNY_BLUR_KERNEL,
    NWINDOWS, WINDOW_MARGIN_PX, MIN_PIX_PER_WINDOW,
    LANE_POLY_ALPHA, HALF_LANE_FRAC,
    POLY_A_ABS_MAX, POLY_B_ABS_MAX,
    LANE_WIDTH_PX_MIN, LANE_WIDTH_PX_MAX,
    LANE_CONF_HI, LANE_CONF_LO,
    LANE_FOUND_TRUE_FRAMES, LANE_FOUND_FALSE_FRAMES,
    OBSTACLE_TRUE_FRAMES, OBSTACLE_FALSE_FRAMES,
    BLINDSPOT_TRUE_FRAMES, BLINDSPOT_FALSE_FRAMES,
    YOLO_MODEL_NAME, YOLO_CONF, YOLO_IMGSZ, YOLO_FRAME_INTERVAL,
    YOLO_VEHICLE_CLASSES, YOLO_PERSON_CLASS,
    YOLO_FRONT_CLOSE_HEIGHT_FRAC, YOLO_FRONT_CLOSE_BOTTOM_FRAC,
    YOLO_FRONT_EGO_LANE_LEFT_FRAC, YOLO_FRONT_EGO_LANE_RIGHT_FRAC,
    YOLO_REAR_BLINDSPOT_LEFT_FRAC, YOLO_REAR_BLINDSPOT_RIGHT_FRAC,
    YOLO_REAR_MIN_HEIGHT_FRAC,
)


# ===========================================================================
#  Multi-frame debounce primitive — used by /lane_found, /obstacle_ahead,
#  /yolo_blindspot_right.  A signal must be True for `true_frames`
#  consecutive ticks to flip the latched output True, and False for
#  `false_frames` consecutive ticks to flip it back to False.  This is the
#  main anti-chatter mechanism for the FSM.
# ===========================================================================
class Debouncer:

    def __init__(self, true_frames: int, false_frames: int,
                 initial: bool = False):
        self.true_frames = int(true_frames)
        self.false_frames = int(false_frames)
        self._state = bool(initial)
        self._true_count = 0
        self._false_count = 0

    def update(self, raw: bool) -> bool:
        if bool(raw):
            self._true_count += 1
            self._false_count = 0
            if not self._state and self._true_count >= self.true_frames:
                self._state = True
        else:
            self._false_count += 1
            self._true_count = 0
            if self._state and self._false_count >= self.false_frames:
                self._state = False
        return self._state

    @property
    def state(self) -> bool:
        return self._state


# ===========================================================================
#  ClassicalLaneDetector — fused HLS + Canny + polyfit with confidence.
#  Returns: { 'found': bool, 'confidence': float, 'error_px': float,
#             'curvature': float, 'poly': [a,b,c] | None,
#             'debug': np.ndarray | None }
# ===========================================================================
class ClassicalLaneDetector:

    def __init__(self, publish_debug: bool):
        self.publish_debug = publish_debug
        self._left_poly  = None
        self._right_poly = None
        self._prev_err   = 0.0
        # last-known-good fallback for transient losses
        self._last_good_err = 0.0
        self._last_good_age = 0

    # ------------------------------------------------------------------
    @staticmethod
    def _color_mask(bgr):
        """HLS-based white + yellow mask."""
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        h_ch, l_ch, s_ch = cv2.split(hls)
        white = (l_ch >= HLS_WHITE_L_MIN) & (s_ch <= HLS_WHITE_S_MAX)
        yellow = ((h_ch >= HLS_YELLOW_H_MIN) & (h_ch <= HLS_YELLOW_H_MAX)
                  & (s_ch >= HLS_YELLOW_S_MIN) & (l_ch >= HLS_YELLOW_L_MIN))
        return ((white | yellow).astype(np.uint8) * 255, l_ch)

    @staticmethod
    def _edge_mask(l_channel):
        """Independent edge-based mask via Canny on lightness."""
        blurred = cv2.GaussianBlur(l_channel,
                                    (CANNY_BLUR_KERNEL, CANNY_BLUR_KERNEL), 0)
        return cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

    @staticmethod
    def _combine_and_close(color_mask, edge_mask):
        combined = cv2.bitwise_or(color_mask, edge_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))
        return cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

    # ------------------------------------------------------------------
    @staticmethod
    def _fit_one_side(mask_roi, start_x):
        h_roi, w_roi = mask_roi.shape
        win_h = max(1, h_roi // NWINDOWS)
        current_x = int(start_x)
        nonzero = mask_roi.nonzero()
        nz_y, nz_x = nonzero[0], nonzero[1]
        lane_idx = []
        good_window_count = 0
        for win in range(NWINDOWS):
            y_lo = h_roi - (win + 1) * win_h
            y_hi = h_roi - win * win_h
            x_lo = max(0, current_x - WINDOW_MARGIN_PX)
            x_hi = min(w_roi, current_x + WINDOW_MARGIN_PX)
            good = ((nz_y >= y_lo) & (nz_y < y_hi) &
                    (nz_x >= x_lo) & (nz_x < x_hi)).nonzero()[0]
            if len(good) > MIN_PIX_PER_WINDOW:
                lane_idx.append(good)
                current_x = int(np.mean(nz_x[good]))
                good_window_count += 1
        if not lane_idx:
            return None, 0
        lane_idx = np.concatenate(lane_idx)
        ys, xs = nz_y[lane_idx], nz_x[lane_idx]
        if len(ys) < 3 * MIN_PIX_PER_WINDOW:
            return None, good_window_count
        try:
            return np.polyfit(ys, xs, 2), good_window_count
        except (np.linalg.LinAlgError, ValueError):
            return None, good_window_count

    # ------------------------------------------------------------------
    @staticmethod
    def _poly_sane(poly):
        """Cheap sanity filter on the fitted polynomial."""
        if poly is None:
            return False
        a, b, _c = poly
        return abs(a) <= POLY_A_ABS_MAX and abs(b) <= POLY_B_ABS_MAX

    @staticmethod
    def _ema_poly(prev, new):
        if prev is None:
            return np.asarray(new, dtype=np.float64)
        return (LANE_POLY_ALPHA * np.asarray(new)
                + (1 - LANE_POLY_ALPHA) * prev)

    @staticmethod
    def _eval_poly(poly, y):
        return poly[0] * y * y + poly[1] * y + poly[2]

    @staticmethod
    def _curvature(poly, y):
        a, b, _ = poly
        return abs(2 * a) / (1 + (2 * a * y + b) ** 2) ** 1.5

    # ------------------------------------------------------------------
    def detect(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        img_center = w // 2

        color_mask, l_chan = self._color_mask(frame_bgr)
        edge_mask = self._edge_mask(l_chan)
        mask = self._combine_and_close(color_mask, edge_mask)

        roi_top = int(h * ROI_TOP_FRAC)
        mask_roi = mask[roi_top:, :]
        h_roi, w_roi = mask_roi.shape

        # seed from histogram of the bottom 25% of the ROI
        seed = mask_roi[(3 * h_roi) // 4:, :]
        col_sum = np.sum(seed, axis=0)
        mid = w // 2
        left_start = (int(np.argmax(col_sum[:mid]))
                      if col_sum[:mid].any() else mid // 2)
        right_start = (int(np.argmax(col_sum[mid:])) + mid
                       if col_sum[mid:].any() else (mid + w) // 2)

        left_raw, left_wins  = self._fit_one_side(mask_roi, left_start)
        right_raw, right_wins = self._fit_one_side(mask_roi, right_start)

        # Sanity-filter each side's raw fit before EMA so a wild outlier
        # doesn't poison the smoothed coefficients.
        if self._poly_sane(left_raw):
            self._left_poly = self._ema_poly(self._left_poly, left_raw)
        if self._poly_sane(right_raw):
            self._right_poly = self._ema_poly(self._right_poly, right_raw)

        have_left = self._left_poly is not None and self._poly_sane(self._left_poly)
        have_right = self._right_poly is not None and self._poly_sane(self._right_poly)
        y_eval = float(h_roi - 1)

        confidence = 0.0
        lane_width_ok = True
        if have_left and have_right:
            x_l = self._eval_poly(self._left_poly, y_eval)
            x_r = self._eval_poly(self._right_poly, y_eval)
            lane_width = abs(x_r - x_l)
            lane_width_ok = (LANE_WIDTH_PX_MIN <= lane_width <= LANE_WIDTH_PX_MAX)
            lane_center = 0.5 * (x_l + x_r)
            avg_poly = 0.5 * (self._left_poly + self._right_poly)
            curv = self._curvature(avg_poly, y_eval)
            poly_out = avg_poly
            # Confidence: both sides found AND width plausible.
            confidence = (
                0.30 +                                       # base for both sides
                0.15 * (left_wins / NWINDOWS) +
                0.15 * (right_wins / NWINDOWS) +
                (0.40 if lane_width_ok else 0.0)
            )
        elif have_left:
            x_l = self._eval_poly(self._left_poly, y_eval)
            lane_center = x_l + HALF_LANE_FRAC * w
            curv = self._curvature(self._left_poly, y_eval)
            poly_out = self._left_poly
            confidence = 0.10 + 0.25 * (left_wins / NWINDOWS)
        elif have_right:
            x_r = self._eval_poly(self._right_poly, y_eval)
            lane_center = x_r - HALF_LANE_FRAC * w
            curv = self._curvature(self._right_poly, y_eval)
            poly_out = self._right_poly
            confidence = 0.10 + 0.25 * (right_wins / NWINDOWS)
        else:
            # Both sides lost.  Hold the last good error for a few frames
            # so the controller doesn't yank the wheel back to centre on
            # a single-frame dropout.
            self._prev_err *= 0.85
            self._last_good_age += 1
            if self._last_good_age < 5:
                return {
                    'found': False,  # the boolean is what's debounced upstream
                    'confidence': 0.0,
                    'error_px': float(self._last_good_err * 0.7),
                    'curvature': 0.0,
                    'poly': None,
                    'debug': None,
                }
            return {
                'found': False, 'confidence': 0.0,
                'error_px': float(self._prev_err),
                'curvature': 0.0, 'poly': None, 'debug': None,
            }

        confidence = float(max(0.0, min(1.0, confidence)))

        raw_err = float(lane_center - img_center)
        smooth = 0.7 * raw_err + 0.3 * self._prev_err
        self._prev_err = smooth
        self._last_good_err = smooth
        self._last_good_age = 0

        dbg = self._render_debug(frame_bgr, mask, roi_top, have_left, have_right,
                                  lane_center, smooth, curv, confidence) \
              if self.publish_debug else None

        return {
            'found': True,
            'confidence': confidence,
            'error_px': float(smooth),
            'curvature': float(curv),
            'poly': [float(poly_out[0]), float(poly_out[1]),
                     float(poly_out[2])],
            'debug': dbg,
        }

    # ------------------------------------------------------------------
    def _render_debug(self, frame_bgr, mask, roi_top,
                       have_left, have_right,
                       lane_center, smooth, curv, conf):
        h, w = frame_bgr.shape[:2]
        dbg = frame_bgr.copy()
        cv2.line(dbg, (0, roi_top), (w - 1, roi_top), (255, 255, 0), 1)
        ys = np.linspace(roi_top, h - 1, 30)
        ys_local = ys - roi_top
        if have_left:
            xs = self._eval_poly(self._left_poly, ys_local)
            for i in range(1, len(ys)):
                cv2.line(dbg, (int(xs[i-1]), int(ys[i-1])),
                          (int(xs[i]), int(ys[i])), (255, 0, 0), 2)
        if have_right:
            xs = self._eval_poly(self._right_poly, ys_local)
            for i in range(1, len(ys)):
                cv2.line(dbg, (int(xs[i-1]), int(ys[i-1])),
                          (int(xs[i]), int(ys[i])), (0, 0, 255), 2)
        lc_y = h - 30
        cv2.line(dbg, (int(lane_center), lc_y - 25),
                  (int(lane_center), lc_y + 5), (0, 255, 0), 3)
        cv2.line(dbg, (w // 2, lc_y - 25),
                  (w // 2, lc_y + 5), (255, 255, 255), 2)
        # confidence bar
        bar_w = int(180 * conf)
        cv2.rectangle(dbg, (10, h - 18), (10 + bar_w, h - 8),
                       (0, int(200 * conf), int(200 * (1 - conf))), -1)
        cv2.rectangle(dbg, (10, h - 18), (10 + 180, h - 8),
                       (180, 180, 180), 1)
        cv2.putText(dbg,
                    f"err={smooth:+.1f}px conf={conf:.2f} curv={curv:.4f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return dbg


class SegNetLaneDetector:
    """Future-hook stub. Falls back to classical on first call and warns once."""

    def __init__(self, publish_debug, logger):
        self._fallback = ClassicalLaneDetector(publish_debug)
        self._warned = False
        self._logger = logger

    def detect(self, frame_bgr):
        if not self._warned:
            self._logger.warn(
                "lane_backend='segnet' is not wired up yet — "
                "falling back to classical.")
            self._warned = True
        return self._fallback.detect(frame_bgr)


# ===========================================================================
#  YOLO wrapper
# ===========================================================================
class YoloDetector:

    def __init__(self, logger, model_name=YOLO_MODEL_NAME):
        self.logger = logger
        self.available = False
        self.model = None
        self._frame_idx = 0
        self._last_front = []
        self._last_rear = []
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as e:
            logger.warn(f"ultralytics not installed ({e}); "
                        f"running without object detection")
            return
        try:
            self.model = YOLO(model_name)
            try:
                _ = self.model(np.zeros((YOLO_IMGSZ, YOLO_IMGSZ, 3),
                                        dtype=np.uint8),
                                imgsz=YOLO_IMGSZ, conf=YOLO_CONF, verbose=False)
            except Exception:
                pass
            self.available = True
            logger.info(f"YOLO loaded ({model_name}, imgsz={YOLO_IMGSZ}, "
                        f"every {YOLO_FRAME_INTERVAL} frames)")
        except Exception as e:
            logger.warn(f"Could not load {model_name}: {e}")
            self.model = None

    def _run(self, bgr):
        if not self.available or bgr is None:
            return []
        try:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            res = self.model(rgb, imgsz=YOLO_IMGSZ, conf=YOLO_CONF,
                              verbose=False)
        except Exception as e:
            self.logger.warn(f"YOLO inference error: {e}")
            return []
        if not res:
            return []
        r0 = res[0]
        try:
            xyxy = r0.boxes.xyxy.cpu().numpy()
            conf = r0.boxes.conf.cpu().numpy()
            cls = r0.boxes.cls.cpu().numpy().astype(int)
        except Exception:
            return []
        out = []
        for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
            if k not in YOLO_VEHICLE_CLASSES and k != YOLO_PERSON_CLASS:
                continue
            out.append({'cls': int(k), 'conf': float(c),
                        'bbox': (float(x1), float(y1),
                                  float(x2), float(y2))})
        return out

    def step(self, front_bgr, rear_bgr):
        self._frame_idx += 1
        if not self.available:
            return ([], [])
        if (self._frame_idx % YOLO_FRAME_INTERVAL) != 0:
            return (self._last_front, self._last_rear)
        self._last_front = self._run(front_bgr) if front_bgr is not None else []
        self._last_rear = self._run(rear_bgr) if rear_bgr is not None else []
        return (self._last_front, self._last_rear)


# ===========================================================================
#  Perception node
# ===========================================================================
class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('publish_debug', True)
        self.declare_parameter('lane_backend', LANE_BACKEND_DEFAULT)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.lane_backend = str(self.get_parameter('lane_backend').value)

        self.bridge = CvBridge()
        self.yolo = YoloDetector(self.get_logger())
        if self.lane_backend == 'segnet':
            self.lane = SegNetLaneDetector(self.publish_debug, self.get_logger())
        else:
            if self.lane_backend != 'classical':
                self.get_logger().warn(
                    f"unknown lane_backend '{self.lane_backend}' — using 'classical'")
            self.lane = ClassicalLaneDetector(self.publish_debug)
        self.get_logger().info(
            f"lane backend: {type(self.lane).__name__} "
            f"(param='{self.lane_backend}')")

        # Debouncers (the FSM-stabilising layer).
        self._lane_found_db = Debouncer(LANE_FOUND_TRUE_FRAMES,
                                         LANE_FOUND_FALSE_FRAMES)
        self._obstacle_db = Debouncer(OBSTACLE_TRUE_FRAMES,
                                       OBSTACLE_FALSE_FRAMES)
        self._blindspot_db = Debouncer(BLINDSPOT_TRUE_FRAMES,
                                        BLINDSPOT_FALSE_FRAMES)

        self._front_bgr = None
        self._rear_bgr = None
        self._rear_lock = threading.Lock()

        self.lane_err_pub      = self.create_publisher(Float32,           '/lane_error',           10)
        self.lane_err_norm_pub = self.create_publisher(Float32,           '/lane_error_norm',      10)
        self.lane_found_pub    = self.create_publisher(Bool,              '/lane_found',           10)
        self.lane_curv_pub     = self.create_publisher(Float32,           '/lane_curvature',       10)
        self.lane_poly_pub     = self.create_publisher(Float32MultiArray, '/lane_poly',            10)
        self.lane_conf_pub     = self.create_publisher(Float32,           '/lane_confidence',      10)
        self.obst_pub          = self.create_publisher(Bool,              '/obstacle_ahead',       10)
        self.blind_pub         = self.create_publisher(Bool,              '/yolo_blindspot_right', 10)
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/perception_debug', 1)
        else:
            self.debug_pub = None

        self.create_subscription(Image, '/front_camera', self._front_cb,
                                  qos_profile_sensor_data)
        self.create_subscription(Image, '/rear_camera', self._rear_cb,
                                  qos_profile_sensor_data)

        self.get_logger().info("perception_node ready")

    # ------------------------------------------------------------------
    def _front_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge(front): {e}")
            return
        self._front_bgr = frame
        self._process_front(frame, msg)

    def _rear_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge(rear): {e}")
            return
        with self._rear_lock:
            self._rear_bgr = frame

    # ------------------------------------------------------------------
    def _process_front(self, frame, msg):
        res = self.lane.detect(frame)
        err = res['error_px']
        conf = res['confidence']
        raw_found = res['found']
        curv = res['curvature']
        poly = res['poly']
        dbg = res['debug']

        # debounced lane_found gate
        raw_found_with_conf = bool(raw_found) and conf >= LANE_CONF_HI
        if conf < LANE_CONF_LO:
            raw_found_with_conf = False
        lane_found = self._lane_found_db.update(raw_found_with_conf)

        self.lane_err_pub.publish(Float32(data=float(err)))
        half_w = max(1.0, frame.shape[1] * 0.5)
        err_norm = max(-1.0, min(1.0, err / half_w))
        self.lane_err_norm_pub.publish(Float32(data=float(err_norm)))
        self.lane_found_pub.publish(Bool(data=bool(lane_found)))
        self.lane_curv_pub.publish(Float32(data=float(curv)))
        self.lane_conf_pub.publish(Float32(data=float(conf)))
        if poly is not None:
            arr = Float32MultiArray()
            arr.data = [float(poly[0]), float(poly[1]), float(poly[2])]
            self.lane_poly_pub.publish(arr)

        with self._rear_lock:
            rear = self._rear_bgr
        front_dets, rear_dets = self.yolo.step(frame, rear)
        raw_front_close = self._yolo_front_vehicle_close(
            front_dets, frame.shape[1], frame.shape[0])
        front_close = self._obstacle_db.update(raw_front_close)
        self.obst_pub.publish(Bool(data=bool(front_close)))

        if rear is not None:
            raw_bs = self._yolo_right_blindspot(
                rear_dets, rear.shape[1], rear.shape[0])
            bs = self._blindspot_db.update(raw_bs)
            self.blind_pub.publish(Bool(data=bool(bs)))

        if self.debug_pub is not None and dbg is not None:
            self._draw_yolo_boxes(dbg, front_dets, front=True,
                                   close_flag=front_close)
            try:
                dmsg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                dmsg.header = msg.header
                self.debug_pub.publish(dmsg)
            except Exception:
                pass

    # ------------------------------------------------------------------
    def _yolo_front_vehicle_close(self, dets, frame_w, frame_h):
        if not dets or frame_h <= 0 or frame_w <= 0:
            return False
        min_h = YOLO_FRONT_CLOSE_HEIGHT_FRAC * frame_h
        min_by = YOLO_FRONT_CLOSE_BOTTOM_FRAC * frame_h
        lane_lo = YOLO_FRONT_EGO_LANE_LEFT_FRAC * frame_w
        lane_hi = YOLO_FRONT_EGO_LANE_RIGHT_FRAC * frame_w
        for d in dets:
            if d['cls'] not in YOLO_VEHICLE_CLASSES and d['cls'] != YOLO_PERSON_CLASS:
                continue
            x1, y1, x2, y2 = d['bbox']
            box_h = y2 - y1
            if box_h < min_h or y2 < min_by:
                continue
            cx = 0.5 * (x1 + x2)
            in_band = (lane_lo <= cx <= lane_hi) or (
                x1 <= lane_hi and x2 >= lane_lo and box_h >= 1.5 * min_h)
            if in_band:
                return True
        return False

    def _yolo_right_blindspot(self, dets, frame_w, frame_h):
        if not dets or frame_w <= 0 or frame_h <= 0:
            return False
        x_lo = YOLO_REAR_BLINDSPOT_LEFT_FRAC * frame_w
        x_hi = YOLO_REAR_BLINDSPOT_RIGHT_FRAC * frame_w
        min_h = YOLO_REAR_MIN_HEIGHT_FRAC * frame_h
        for d in dets:
            if d['cls'] not in YOLO_VEHICLE_CLASSES:
                continue
            x1, y1, x2, y2 = d['bbox']
            cx = 0.5 * (x1 + x2)
            if (y2 - y1) < min_h:
                continue
            if x_lo <= cx <= x_hi:
                return True
        return False

    def _draw_yolo_boxes(self, img, dets, front, close_flag):
        if img is None or not dets:
            return
        h, w = img.shape[:2]
        min_h_close = YOLO_FRONT_CLOSE_HEIGHT_FRAC * h
        min_by_close = YOLO_FRONT_CLOSE_BOTTOM_FRAC * h
        lane_lo = YOLO_FRONT_EGO_LANE_LEFT_FRAC * w
        lane_hi = YOLO_FRONT_EGO_LANE_RIGHT_FRAC * w
        if front:
            cv2.line(img, (int(lane_lo), 0), (int(lane_lo), h - 1),
                      (80, 80, 80), 1)
            cv2.line(img, (int(lane_hi), 0), (int(lane_hi), h - 1),
                      (80, 80, 80), 1)
        for d in dets:
            x1, y1, x2, y2 = d['bbox']
            x1i, y1i = int(max(0, x1)), int(max(0, y1))
            x2i, y2i = int(min(w - 1, x2)), int(min(h - 1, y2))
            is_person = d['cls'] == YOLO_PERSON_CLASS
            box_h = y2i - y1i
            is_close = (box_h >= min_h_close and y2i >= min_by_close)
            cx = 0.5 * (x1i + x2i)
            in_lane = (lane_lo <= cx <= lane_hi) or (
                x1i <= lane_hi and x2i >= lane_lo and box_h >= 1.5 * min_h_close)
            if front and not is_person:
                if is_close and in_lane and close_flag:
                    color, thick = (0, 0, 255), 2
                elif is_close and not in_lane:
                    color, thick = (0, 128, 255), 1
                else:
                    color, thick = (0, 255, 0), 1
            else:
                color = (255, 0, 255) if is_person else (0, 255, 0)
                thick = 1
            cv2.rectangle(img, (x1i, y1i), (x2i, y2i), color, thick)
            cv2.putText(img, f"{d['cls']}:{d['conf']:.2f}",
                        (x1i, max(10, y1i - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
