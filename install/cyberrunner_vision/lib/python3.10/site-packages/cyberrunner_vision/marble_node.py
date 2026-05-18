"""
marble_node  (improved stability for dark/deep green marble)
------------------------------------------------------------
- Combines HSV mask + Hough circles with weighted confidence
- Temporal smoothing: exponential moving average on position
- Lost-marble recovery: keeps last known position for N frames
- Publishes geometry_msgs/Point  x,y=topdown px  z=radius (-1=lost)

Parameters
  hsv_lo        str   "40,80,20"
  hsv_hi        str   "85,255,255"
  min_radius    int   15    (marble ⌀12.77mm → ~23 px radius at 3.571 px/mm scale)
  max_radius    int   29
  hough_param2  int   18
  smooth_alpha  float 0.4    (0=very smooth/laggy  1=raw/jumpy)
  lost_frames   int   8      (frames to keep last pos before publishing -1)
  show_debug    bool  true
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class MarbleNode(Node):
    def __init__(self):
        super().__init__("marble_node")
        self.declare_parameter("hsv_lo",       "40,80,20")
        self.declare_parameter("hsv_hi",       "85,255,255")
        self.declare_parameter("min_radius",   14)   # marble ⌀12.77 mm → ~20 px
        self.declare_parameter("max_radius",   26)
        self.declare_parameter("hough_param2", 18)
        self.declare_parameter("smooth_alpha", 0.4)
        self.declare_parameter("lost_frames",  8)
        self.declare_parameter("show_debug",   False)

        self._refresh_params()
        self.add_on_set_parameters_callback(self._on_params_changed)
        self.bridge = CvBridge()

        # Morphological kernels — built once, reused every frame
        self._k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Tracking state
        self.smooth_x   = None
        self.smooth_y   = None
        self.smooth_r   = None
        self.lost_count = 0

        self.sub_td = self.create_subscription(
            Image, "/camera/topdown", self._on_image, 2)
        self.pub = self.create_publisher(Point, "/marble/position", 2)
        self.get_logger().info("MarbleNode started  (deep green mode)")

    def _refresh_params(self):
        lo = list(map(int, self.get_parameter("hsv_lo").value.split(",")))
        hi = list(map(int, self.get_parameter("hsv_hi").value.split(",")))
        self.hsv_lo     = np.array(lo, dtype=np.uint8)
        self.hsv_hi     = np.array(hi, dtype=np.uint8)
        self.min_r      = self.get_parameter("min_radius").value
        self.max_r      = self.get_parameter("max_radius").value
        self.hough_p2   = self.get_parameter("hough_param2").value
        self.alpha      = float(self.get_parameter("smooth_alpha").value)
        self.lost_max   = self.get_parameter("lost_frames").value
        self.show_debug = self.get_parameter("show_debug").value
        self._area_min  = np.pi * self.min_r ** 2 * 0.4
        self._area_max  = np.pi * self.max_r ** 2 * 1.8

    def _on_params_changed(self, params):
        self._refresh_params()
        return SetParametersResult(successful=True)

    def _on_image(self, msg):
        td  = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        raw, mask = self._detect(td)

        if raw is not None:
            cx, cy, r = raw
            self.lost_count = 0
            if self.smooth_x is None:
                self.smooth_x = float(cx)
                self.smooth_y = float(cy)
                self.smooth_r = float(r)
            else:
                a = self.alpha
                self.smooth_x = a * cx + (1 - a) * self.smooth_x
                self.smooth_y = a * cy + (1 - a) * self.smooth_y
                self.smooth_r = a * r  + (1 - a) * self.smooth_r
        else:
            self.lost_count += 1

        pt = Point()
        if self.smooth_x is not None and self.lost_count <= self.lost_max:
            pt.x = self.smooth_x
            pt.y = self.smooth_y
            pt.z = self.smooth_r
        else:
            pt.z = -1.0

        self.pub.publish(pt)

        if self.show_debug:
            self._show_debug(td, mask, raw, pt)

    def _detect(self, bgr):
        """
        Returns ((cx, cy, radius), mask) or (None, mask).
        Stage 1: HSV mask → largest valid blob
        Stage 2: Hough circles on green-channel enhanced image
        """
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lo, self.hsv_hi)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._k3, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k5, iterations=3)
        mask = cv2.dilate(mask, self._k3, iterations=1)

        # ── Stage 1: blob in HSV mask ────────────────────────────────────────
        cnts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self._area_min:
                continue
            if area > self._area_max:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.45:
                continue
            if self.min_r <= r <= self.max_r:
                if best is None or area > best[3]:
                    best = (int(x), int(y), float(r), area)
        if best:
            return (best[0], best[1], best[2]), mask

        # ── Stage 2: Hough on green-channel ──────────────────────────────────
        b, g, r_ch = cv2.split(bgr)
        green_enh = cv2.subtract(g, cv2.addWeighted(b, 0.5, r_ch, 0.5, 0))
        green_enh = cv2.GaussianBlur(green_enh, (5, 5), 0)

        circles = cv2.HoughCircles(
            green_enh, cv2.HOUGH_GRADIENT,
            dp=1.2, minDist=20,
            param1=60, param2=self.hough_p2,
            minRadius=self.min_r, maxRadius=self.max_r)
        if circles is not None:
            c = np.round(circles[0, 0]).astype(int)
            cx, cy, cr = c[0], c[1], c[2]
            h, w = mask.shape[:2]
            roi_mask = mask[max(0, cy - cr):min(h, cy + cr),
                           max(0, cx - cr):min(w, cx + cr)]
            if roi_mask.size > 0 and roi_mask.mean() > 10:
                return (cx, cy, float(cr)), mask

        return None, mask

    def _show_debug(self, bgr, mask, raw, pt):
        dbg = bgr.copy()

        tint = np.zeros_like(bgr)
        tint[:, :, 1] = mask
        dbg  = cv2.addWeighted(dbg, 0.75, tint, 0.25, 0)

        if raw:
            cv2.circle(dbg, (int(raw[0]), int(raw[1])), int(raw[2]),
                       (255, 255, 255), 1)

        if pt.z > 0:
            cx, cy, r = int(pt.x), int(pt.y), int(pt.z)
            cv2.circle(dbg, (cx, cy), r,  (0, 255, 0), 2)
            cv2.circle(dbg, (cx, cy), 3,  (0, 255, 0), -1)
            cv2.putText(dbg, "(" + str(cx) + "," + str(cy) + ")",
                        (cx + r + 4, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            status = "TRACKING  lost=" + str(self.lost_count)
            color  = (0, 255, 0)
        else:
            status = "LOST  (" + str(self.lost_count) + " frames)"
            color  = (0, 0, 255)

        cv2.putText(dbg, status, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        cv2.putText(dbg,
                    "hsv_lo=" + str(self.hsv_lo.tolist()) +
                    "  hsv_hi=" + str(self.hsv_hi.tolist()),
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

        cv2.imshow("MARBLE_DETECT", dbg)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarbleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
