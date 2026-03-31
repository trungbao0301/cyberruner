"""
estimator_node  (click-based, no ArUco)
----------------------------------------
Click 8 points on the RECTIFIED image:
  First  4 clicks → stable corners (TL, TR, BR, BL) fixed to the TABLE
  Next   4 clicks → moving corners (TL, TR, BR, BL) on the MAZE BOARD

Keys in the RECTIFIED window:
  c  → clear all clicks and start over
  s  → save config to file (no need to click next time)

Publishes
  /camera/topdown      Image   (warped overhead view, stable corners only)
  /estimator/state     Float32MultiArray (15 floats)
    [0-1]  origin_x, origin_y  (topdown px — centre of moving corners)
    [2]    scale  px/mm  (requires board_width_mm param, else 1.0)
    [3-4]  tilt_x_deg, tilt_y_deg  (from moving quad skew vs flat reference)
    [5]    board_valid  (1.0 once all 8 clicked)
    [6-14] homography H row-major (3x3, stable corners → topdown)

Services
  /estimator/save_config   Trigger
  /estimator/load_config   Trigger

Parameters
  config_path       str    ~/cyberrunner_config.json
  board_width_mm    float  400.0   (physical TL→TR distance, for scale)
  board_height_mm   float  400.0
  topdown_size      int    1000
  show_window       bool   true
"""
import os
import json
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")

        self.declare_parameter("config_path",
                               os.path.expanduser("~/cyberrunner_config.json"))
        self.declare_parameter("board_width_mm",  320.0)
        self.declare_parameter("board_height_mm", 295.0)
        self.declare_parameter("topdown_size",    1000)
        self.declare_parameter("show_window",     True)
        self.declare_parameter("blue_lo",         "90,80,50")
        self.declare_parameter("blue_hi",         "130,255,255")
        self.declare_parameter("min_dot_radius",  4)
        self.declare_parameter("max_dot_radius",  30)

        self.config_path  = self.get_parameter("config_path").value
        self.bw           = self.get_parameter("board_width_mm").value
        self.bh           = self.get_parameter("board_height_mm").value
        self.TOP          = self.get_parameter("topdown_size").value
        self.show_window  = self.get_parameter("show_window").value
        self._reload_blue_params()

        self.bridge       = CvBridge()
        self.last_rect    = None   # latest rectified frame

        # Click state
        # stable[0-3] = TL,TR,BR,BL on fixed frame
        # moving[0-3] = TL,TR,BR,BL on maze board
        self.stable_pts   = []    # list of [x,y] in rectified px
        self.moving_pts   = []
        self.H            = None  # rectified → topdown homography
        self.state        = np.zeros(15, dtype=np.float32)

        # Flat (level) moving quad saved when board is level → used for tilt
        self.moving_flat  = None  # np.array shape (4,2)

        # Cached flat-reference dimensions in topdown space (recomputed only when
        # H or moving_flat changes, not every frame)
        self._flat_ref_wh = None  # (ref_w, ref_h) in topdown px

        # Morphological kernel for blue-dot detection
        self._k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Window name
        self.WIN = "ESTIMATOR  (click 4 stable → 4 moving | c=clear | s=save)"

        # ROS I/O
        self.sub = self.create_subscription(
            Image, "/camera/rectified", self._on_image, 2)
        self.pub_td             = self.create_publisher(Image, "/camera/topdown", 2)
        self.pub_state          = self.create_publisher(
            Float32MultiArray, "/estimator/state", 2)
        self.pub_board_xfm      = self.create_publisher(
            Float32MultiArray, "/estimator/board_transform", 2)
        self.create_service(Trigger, "/estimator/save_config", self._svc_save)
        self.create_service(Trigger, "/estimator/load_config", self._svc_load)

        # Timer drives the GUI
        self.timer = self.create_timer(0.033, self._tick)

        # Auto-load saved config
        if os.path.exists(self.config_path):
            self._load_config(self.config_path)
            self.get_logger().info(
                "Loaded config from " + self.config_path)
        else:
            self.get_logger().info(
                "No config found. Click 4 stable corners then 4 moving corners.")

        if self.show_window:
            cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.WIN, self._on_mouse)

    # ── Mouse callback ────────────────────────────────────────────────────────
    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if len(self.stable_pts) < 4:
            self.stable_pts.append([x, y])
            idx = len(self.stable_pts)
            labels = ["TL", "TR", "BR", "BL"]
            self.get_logger().info(
                "Stable " + labels[idx-1] + " = (" + str(x) + "," + str(y) + ")"
                + "  (" + str(idx) + "/4)")
            if len(self.stable_pts) == 4:
                self._compute_H()
                self.get_logger().info(
                    "Stable corners done! Now click 4 MOVING corners (TL,TR,BR,BL).")

        elif len(self.moving_pts) < 4:
            self.moving_pts.append([x, y])
            idx = len(self.moving_pts)
            labels = ["TL", "TR", "BR", "BL"]
            self.get_logger().info(
                "Moving " + labels[idx-1] + " = (" + str(x) + "," + str(y) + ")"
                + "  (" + str(idx) + "/4)")
            if len(self.moving_pts) == 4:
                # Save the flat reference (board level) on first completion
                if self.moving_flat is None:
                    self.moving_flat = np.array(self.moving_pts, dtype=np.float32)
                    self._update_flat_ref()
                self.get_logger().info(
                    "All 8 points set! Estimator active.")
        else:
            self.get_logger().info(
                "All 8 points already set. Press 'c' to clear and redo.")

    # ── GUI tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        if not self.show_window or self.last_rect is None:
            return

        disp = self.last_rect.copy()
        self._draw_overlay(disp)
        cv2.imshow(self.WIN, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            self.stable_pts.clear()
            self.moving_pts.clear()
            self.H            = None
            self.moving_flat  = None
            self._flat_ref_wh = None
            self.state        = np.zeros(15, dtype=np.float32)
            self.get_logger().info("Cleared. Click 4 stable corners first.")

        elif key == ord('s'):
            self._save_config(self.config_path)
            self.get_logger().info("Config saved to " + self.config_path)

    def _draw_overlay(self, img):
        stable_colors = [(0,255,0),(0,200,0),(0,150,0),(0,100,0)]
        moving_colors = [(0,100,255),(0,80,200),(0,60,150),(0,40,100)]
        labels = ["TL","TR","BR","BL"]

        for i, pt in enumerate(self.stable_pts):
            cv2.circle(img, (int(pt[0]), int(pt[1])), 8, stable_colors[i], -1)
            cv2.putText(img, "S-" + labels[i],
                        (int(pt[0])+10, int(pt[1])-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, stable_colors[i], 2)

        for i, pt in enumerate(self.moving_pts):
            cv2.circle(img, (int(pt[0]), int(pt[1])), 8, moving_colors[i], -1)
            cv2.putText(img, "M-" + labels[i],
                        (int(pt[0])+10, int(pt[1])-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, moving_colors[i], 2)

        # Draw quads
        if len(self.stable_pts) == 4:
            pts = np.array(self.stable_pts, dtype=np.int32)
            cv2.polylines(img, [pts], True, (0, 255, 0), 2)

        if len(self.moving_pts) == 4:
            pts = np.array(self.moving_pts, dtype=np.int32)
            cv2.polylines(img, [pts], True, (0, 100, 255), 2)

        # HUD
        n_stable = len(self.stable_pts)
        n_moving = len(self.moving_pts)
        if n_stable < 4:
            msg = "Click STABLE corners: " + str(n_stable) + "/4  (TL->TR->BR->BL)"
            color = (0, 255, 0)
        elif n_moving < 4:
            msg = "Click MOVING corners: " + str(n_moving) + "/4  (TL->TR->BR->BL)"
            color = (0, 100, 255)
        else:
            tx = self.state[3]; ty = self.state[4]
            msg = ("ACTIVE  tiltX=" + str(round(float(tx), 1)) +
                   "  tiltY=" + str(round(float(ty), 1)) +
                   "  c=clear  s=save")
            color = (0, 255, 255)

        cv2.putText(img, msg, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # ── Homography (stable corners → topdown) ────────────────────────────────
    def _compute_H(self):
        if len(self.stable_pts) != 4:
            return
        T  = float(self.TOP)
        bw = self.bw; bh = self.bh
        scale = T / max(bw, bh)
        ox = (T - bw * scale) / 2.0
        oy = (T - bh * scale) / 2.0
        mx = bw * scale + ox
        my = bh * scale + oy
        src = np.array(self.stable_pts, dtype=np.float32)
        dst = np.array([[ox,oy],[mx,oy],[mx,my],[ox,my]], dtype=np.float32)
        self.H, _ = cv2.findHomography(src, dst)
        self.state[6:15] = self.H.flatten().astype(np.float32)
        self.state[2]    = float(scale)   # px/mm — matches scale used in homography
        self._update_flat_ref()

    # ── Flat reference cache ──────────────────────────────────────────────────
    def _update_flat_ref(self):
        """Pre-compute flat-reference dimensions in topdown space.
        Called only when H or moving_flat changes — not every frame."""
        if self.H is None or self.moving_flat is None:
            self._flat_ref_wh = None
            return
        flat_td = cv2.perspectiveTransform(
            self.moving_flat.reshape(-1, 1, 2), self.H).reshape(-1, 2)
        ref_w = max(float(np.linalg.norm(flat_td[1] - flat_td[0])), 1.0)
        ref_h = max(float(np.linalg.norm(flat_td[3] - flat_td[0])), 1.0)
        self._flat_ref_wh = (ref_w, ref_h)

    # ── Tilt estimation from moving quad skew ────────────────────────────────
    def _estimate_tilt(self):
        """
        Compares current moving quad to the saved flat reference.
        Projects moving points into topdown space, computes centroid (origin)
        and estimates tilt from the perspective distortion of the quad.

        Returns tilt_x_deg, tilt_y_deg, origin_x, origin_y
        """
        if self.H is None or len(self.moving_pts) != 4:
            return 0.0, 0.0, self.TOP / 2.0, self.TOP / 2.0

        cur = np.array(self.moving_pts, dtype=np.float32).reshape(-1, 1, 2)

        # Project moving pts into topdown space
        td_pts = cv2.perspectiveTransform(cur, self.H).reshape(-1, 2)

        # Origin = centroid of moving corners in topdown
        origin = td_pts.mean(axis=0)

        # Estimate tilt from width/height asymmetry of projected quad
        # Top edge vs bottom edge width → X tilt
        top_w    = float(np.linalg.norm(td_pts[1] - td_pts[0]))
        bot_w    = float(np.linalg.norm(td_pts[2] - td_pts[3]))
        left_h   = float(np.linalg.norm(td_pts[3] - td_pts[0]))
        right_h  = float(np.linalg.norm(td_pts[2] - td_pts[1]))

        # Normalise against flat reference — use cached value (recomputed only on click)
        if self._flat_ref_wh is not None:
            ref_w, ref_h = self._flat_ref_wh
        else:
            ref_w = max((top_w + bot_w) / 2.0, 1.0)
            ref_h = max((left_h + right_h) / 2.0, 1.0)

        # Ratio → angle (small angle approx, scale factor ~30 for degrees)
        tilt_x = math.degrees(math.atan2(bot_w - top_w,   ref_w * 2.0))
        tilt_y = math.degrees(math.atan2(right_h - left_h, ref_h * 2.0))

        return tilt_x, tilt_y, float(origin[0]), float(origin[1])

    # ── Blue-dot detection ────────────────────────────────────────────────────
    def _reload_blue_params(self):
        lo = list(map(int, self.get_parameter("blue_lo").value.split(",")))
        hi = list(map(int, self.get_parameter("blue_hi").value.split(",")))
        self._blue_lo     = np.array(lo, dtype=np.uint8)
        self._blue_hi     = np.array(hi, dtype=np.uint8)
        self._min_dot_r   = float(self.get_parameter("min_dot_radius").value)
        self._max_dot_r   = float(self.get_parameter("max_dot_radius").value)

    def _detect_blue_dots(self, rect):
        """Detect 4 blue corner markers in rectified image.
        Only searches inside the stable quad to avoid picking up outer frame corners.
        Returns [[x,y]×4] in TL,TR,BR,BL order, or None."""
        hsv  = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._blue_lo, self._blue_hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._k5, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k5, iterations=2)

        # Restrict search to inside the stable quad so outer frame corners are excluded
        if len(self.stable_pts) == 4:
            roi = np.zeros(mask.shape, dtype=np.uint8)
            pts = np.array(self.stable_pts, dtype=np.int32)
            cv2.fillPoly(roi, [pts], 255)
            mask = cv2.bitwise_and(mask, roi)

        # Debug window — shows masked region so HSV range can be tuned
        if self.show_window:
            dbg = rect.copy()
            dbg[mask > 0] = (0, 255, 255)
            cv2.imshow("BLUE DOT MASK", dbg)

        cnts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_min = math.pi * self._min_dot_r ** 2 * 0.3
        area_max = math.pi * self._max_dot_r ** 2 * 2.0
        dots = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < area_min or area > area_max:
                continue
            (x, y), _ = cv2.minEnclosingCircle(c)
            dots.append((float(x), float(y), area))

        if len(dots) < 4:
            return None

        # Keep 4 largest blobs
        dots.sort(key=lambda d: d[2], reverse=True)
        pts = [(d[0], d[1]) for d in dots[:4]]

        # Sort into TL, TR, BR, BL
        pts.sort(key=lambda p: p[1])      # top-2 / bottom-2
        top = sorted(pts[:2], key=lambda p: p[0])
        bot = sorted(pts[2:], key=lambda p: p[0])
        return [[top[0][0], top[0][1]],   # TL
                [top[1][0], top[1][1]],   # TR
                [bot[1][0], bot[1][1]],   # BR
                [bot[0][0], bot[0][1]]]   # BL

    # ── Image callback ────────────────────────────────────────────────────────
    def _on_image(self, msg):
        rect = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.last_rect = rect

        # Topdown warp
        if self.H is not None:
            td  = cv2.warpPerspective(rect, self.H, (self.TOP, self.TOP))
            tdm = self.bridge.cv2_to_imgmsg(td, encoding="bgr8")
            tdm.header = msg.header
            self.pub_td.publish(tdm)

        # Blue-dot detection: update moving_pts from live image
        if len(self.moving_pts) == 4:
            detected = self._detect_blue_dots(rect)
            if detected is not None:
                self.moving_pts = detected
            else:
                self.get_logger().warn(
                    "Blue dots not found — keeping last position",
                    throttle_duration_sec=2.0)

        # Tilt + origin
        if len(self.moving_pts) == 4:
            tx, ty, ox, oy = self._estimate_tilt()
            self.state[0] = ox;  self.state[1] = oy
            self.state[3] = tx;  self.state[4] = ty
            self.state[5] = 1.0

            # Board transform: flat reference → current (both in topdown space)
            # Lets the controller keep waypoints aligned with the moving maze
            if self.H is not None and self.moving_flat is not None:
                flat_td = cv2.perspectiveTransform(
                    self.moving_flat.reshape(-1, 1, 2), self.H).reshape(-1, 2)
                cur_td  = cv2.perspectiveTransform(
                    np.array(self.moving_pts, dtype=np.float32).reshape(-1, 1, 2),
                    self.H).reshape(-1, 2)
                M, _ = cv2.findHomography(flat_td, cur_td)
                if M is not None:
                    xfm = Float32MultiArray()
                    xfm.data = M.flatten().astype(float).tolist()
                    self.pub_board_xfm.publish(xfm)
        else:
            self.state[5] = 0.0

        out = Float32MultiArray()
        out.data = self.state.tolist()
        self.pub_state.publish(out)

    # ── Persistence ───────────────────────────────────────────────────────────
    def _save_config(self, path):
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({
                "stable_pts":      self.stable_pts,
                "moving_pts":      self.moving_pts,
                "moving_flat":     self.moving_flat.tolist()
                                   if self.moving_flat is not None else None,
                "board_width_mm":  self.bw,
                "board_height_mm": self.bh,
                "topdown_size":    self.TOP,
            }, f, indent=2)
        os.replace(tmp, path)

    def _load_config(self, path):
        with open(path) as f:
            cfg = json.load(f)
        self.stable_pts  = cfg.get("stable_pts",  [])
        self.moving_pts  = cfg.get("moving_pts",  [])
        mf               = cfg.get("moving_flat", None)
        self.moving_flat = np.array(mf, dtype=np.float32) if mf else None
        self.bw          = cfg.get("board_width_mm",  self.bw)
        self.bh          = cfg.get("board_height_mm", self.bh)
        self.TOP         = cfg.get("topdown_size",    self.TOP)
        if len(self.stable_pts) == 4:
            self._compute_H()
        if len(self.moving_pts) == 4:
            self.state[5] = 1.0

    # ── Services ──────────────────────────────────────────────────────────────
    def _svc_save(self, req, res):
        try:
            self._save_config(self.config_path)
            res.success = True
            res.message = "Saved to " + self.config_path
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def _svc_load(self, req, res):
        try:
            self._load_config(self.config_path)
            res.success = True
            res.message = "Loaded from " + self.config_path
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()