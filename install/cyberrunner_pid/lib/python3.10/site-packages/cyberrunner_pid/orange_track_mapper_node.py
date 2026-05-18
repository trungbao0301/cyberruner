#!/usr/bin/env python3
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from cyberrunner_interfaces.msg import StateEstimateSub  # adjust if your msg name differs


# ---------------- helpers ----------------
def skeletonize(binary_mask: np.ndarray) -> np.ndarray:
    img = (binary_mask > 0).astype(np.uint8) * 255
    skel = np.zeros_like(img)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        eroded = cv2.erode(img, element)
        opened = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, opened)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded
        if cv2.countNonZero(img) == 0:
            break
    return skel

def largest_component(mask: np.ndarray) -> np.ndarray:
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return mask
    idx = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    out = np.zeros_like(mask)
    out[labels == idx] = 255
    return out

def order_skeleton(skel: np.ndarray):
    ys, xs = np.where(skel > 0)
    pts = list(zip(xs.tolist(), ys.tolist()))
    if not pts:
        return []
    S = set(pts)

    def nbs(p):
        x, y = p
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                q = (x + dx, y + dy)
                if q in S:
                    out.append(q)
        return out

    ends = [p for p in pts if len(nbs(p)) == 1]
    start = ends[0] if ends else pts[0]

    path = [start]
    visited = {start}
    prev = None
    cur = start

    while True:
        cand = [q for q in nbs(cur) if q not in visited]
        if not cand:
            break
        if prev is None:
            nxt = cand[0]
        else:
            vx, vy = cur[0] - prev[0], cur[1] - prev[1]
            nxt = max(cand, key=lambda q: vx*(q[0]-cur[0]) + vy*(q[1]-cur[1]))
        prev, cur = cur, nxt
        visited.add(cur)
        path.append(cur)
    return path

def detect_hole_centers(warp_bgr, v_max=60, min_area=250, max_area=50000, min_circ=0.55):
    hsv = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    hole = (v <= v_max).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    hole = cv2.morphologyEx(hole, cv2.MORPH_OPEN, k, iterations=1)
    hole = cv2.morphologyEx(hole, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(hole, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        per = cv2.arcLength(c, True)
        if per < 1e-6:
            continue
        circ = 4.0 * np.pi * area / (per * per)
        if circ < min_circ:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        centers.append((float(x), float(y)))
    return np.array(centers, dtype=np.float32)  # shape (M,2)

def detect_walls_black_thick(warp_bgr, v_max=70, s_max=120, dilate_px=3):
    hsv = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2HSV)
    S = hsv[:, :, 1]
    V = hsv[:, :, 2]
    wall = ((V <= v_max) & (S <= s_max)).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, k, iterations=2)
    wall = cv2.morphologyEx(wall, cv2.MORPH_OPEN,  k, iterations=1)

    if dilate_px > 0:
        kd = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*dilate_px+1, 2*dilate_px+1))
        wall = cv2.dilate(wall, kd, iterations=1)
    return wall

def wall_distance_m(wall_mask, board_w_m, board_h_m):
    free = (wall_mask == 0).astype(np.uint8) * 255
    dist_px = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    H, W = wall_mask.shape[:2]
    m_per_px = 0.5 * (board_w_m / max(1.0, W - 1) + board_h_m / max(1.0, H - 1))
    return dist_px.astype(np.float32) * float(m_per_px)

def px_to_m(x_px, y_px, W, H, board_w_m, board_h_m):
    x_m = (x_px / max(1.0, (W - 1))) * board_w_m
    y_m = (y_px / max(1.0, (H - 1))) * board_h_m
    return x_m, y_m

def m_to_px(x_m, y_m, W, H, board_w_m, board_h_m):
    u = int(round((x_m / board_w_m) * (W - 1)))
    v = int(round((y_m / board_h_m) * (H - 1)))
    u = max(0, min(u, W - 1))
    v = max(0, min(v, H - 1))
    return u, v


# ---------------- node ----------------
class OrangeLiveTarget(Node):
    def __init__(self):
        super().__init__("orange_live_target_node")

        # ---- parameters ----
        self.declare_parameter("device", "/dev/video2")
        self.declare_parameter("cap_w", 1920)
        self.declare_parameter("cap_h", 1200)
        self.declare_parameter("fps", 30)

        self.declare_parameter("warp_w_px", 900)
        self.declare_parameter("warp_h_px", 600)

        # IMPORTANT: set to your real board size
        self.declare_parameter("board_w_m", 0.30)
        self.declare_parameter("board_h_m", 0.20)

        # lookahead on path (meters)
        self.declare_parameter("lookahead_m", 0.03)

        # hole: diameter 1.5cm => radius 0.0075m (fixed)
        self.declare_parameter("hole_radius_m", 0.0075)
        self.declare_parameter("ball_radius_m", 0.0070)
        self.declare_parameter("hole_margin_m", 0.0100)

        # wall safety
        self.declare_parameter("wall_margin_m", 0.012)

        # smoothing for orange mask flicker
        self.declare_parameter("ema_alpha", 0.90)

        # topics
        self.declare_parameter("estimate_topic", "cyberrunner_state_estimation/estimate_subimg")
        self.declare_parameter("target_topic", "/ball/target")

        # ---- get params ----
        self.dev = self.get_parameter("device").value
        self.cap_w = int(self.get_parameter("cap_w").value)
        self.cap_h = int(self.get_parameter("cap_h").value)
        self.fps = int(self.get_parameter("fps").value)

        self.WW = int(self.get_parameter("warp_w_px").value)
        self.HH = int(self.get_parameter("warp_h_px").value)

        self.board_w_m = float(self.get_parameter("board_w_m").value)
        self.board_h_m = float(self.get_parameter("board_h_m").value)

        self.lookahead_m = float(self.get_parameter("lookahead_m").value)

        hole_r = float(self.get_parameter("hole_radius_m").value)
        ball_r = float(self.get_parameter("ball_radius_m").value)
        hole_margin = float(self.get_parameter("hole_margin_m").value)
        self.unsafe_hole_r = hole_r + ball_r + hole_margin

        self.wall_margin_m = float(self.get_parameter("wall_margin_m").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)

        # ---- camera ----
        self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self.dev}")

        time.sleep(0.4)
        for _ in range(10):
            self.cap.grab()

        # ---- warp selection ----
        self.corner_mode = False
        self.corners = []
        self.H = None
        self.have_warp = False

        # ---- track mask smoothing ----
        self.mask_ema = None

        # ---- ball position (meters) ----
        self.ball_xy = None

        # ---- publisher/subscriber ----
        self.pub = self.create_publisher(Point, self.get_parameter("target_topic").value, 10)
        self.sub = self.create_subscription(StateEstimateSub, self.get_parameter("estimate_topic").value, self.on_est, 10)

        # ---- UI windows ----
        cv2.namedWindow("frame",   cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("overlay", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("tuner",   cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 960, 600)
        cv2.resizeWindow("overlay", 960, 600)
        cv2.resizeWindow("tuner", 720, 360)

        def nothing(_): pass

        # Orange HSV
        cv2.createTrackbar("H min", "tuner", 5,   179, nothing)
        cv2.createTrackbar("H max", "tuner", 30,  179, nothing)
        cv2.createTrackbar("S min", "tuner", 80,  255, nothing)
        cv2.createTrackbar("S max", "tuner", 255, 255, nothing)
        cv2.createTrackbar("V min", "tuner", 80,  255, nothing)
        cv2.createTrackbar("V max", "tuner", 255, 255, nothing)

        # Holes
        cv2.createTrackbar("Hole V max", "tuner", 60, 255, nothing)
        cv2.createTrackbar("Hole min area", "tuner", 250, 20000, nothing)
        cv2.createTrackbar("Hole min circ x100", "tuner", 55, 100, nothing)

        # Walls (black thick 5mm)
        cv2.createTrackbar("Wall V max", "tuner", 70, 255, nothing)
        cv2.createTrackbar("Wall S max", "tuner", 120, 255, nothing)
        cv2.createTrackbar("Wall dilate px", "tuner", 3, 15, nothing)

        cv2.setMouseCallback("frame", self.on_mouse)

        self.get_logger().info(
            "LIVE mode (no saving). Keys: c=corners TL,TR,BR,BL | r=reset | q=quit"
        )
        self.get_logger().info(
            f"Unsafe hole radius = {self.unsafe_hole_r:.4f} m | wall margin = {self.wall_margin_m:.4f} m"
        )

        self.timer = self.create_timer(0.03, self.loop)

    def on_est(self, msg: StateEstimateSub):
        # ball position in meters (must be in same board frame as mapper assumes)
        self.ball_xy = np.array([float(msg.x), float(msg.y)], dtype=np.float32)

    def on_mouse(self, event, x, y, flags, param=None):
        if event == cv2.EVENT_LBUTTONDOWN and self.corner_mode:
            self.corners.append((x, y))
            self.get_logger().info(f"Corner {len(self.corners)}/4: {(x,y)}")
            if len(self.corners) == 4:
                self.compute_warp()

    def compute_warp(self):
        src = np.array(self.corners, dtype=np.float32)
        dst = np.array([[0,0],[self.WW-1,0],[self.WW-1,self.HH-1],[0,self.HH-1]], dtype=np.float32)
        self.H = cv2.getPerspectiveTransform(src, dst)
        self.have_warp = True
        self.corner_mode = False
        self.mask_ema = None
        self.get_logger().info("Warp ready.")

    def safe_from_holes(self, pt_xy_m, holes_xy_m):
        if holes_xy_m.shape[0] == 0:
            return True
        d = np.linalg.norm(holes_xy_m - pt_xy_m[None, :], axis=1)
        return bool(np.all(d > self.unsafe_hole_r))

    def safe_from_walls(self, pt_xy_m, wall_dist_m_map):
        if wall_dist_m_map is None:
            return True
        u, v = m_to_px(float(pt_xy_m[0]), float(pt_xy_m[1]),
                       self.WW, self.HH, self.board_w_m, self.board_h_m)
        return float(wall_dist_m_map[v, u]) >= self.wall_margin_m

    def loop(self):
        ok, frame = self.cap.read()
        if not ok or frame is None or frame.size == 0:
            return

        show = frame.copy()
        for i, p in enumerate(self.corners):
            cv2.circle(show, p, 6, (0,255,0), -1)
            cv2.putText(show, str(i+1), (p[0]+8,p[1]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        if self.have_warp:
            view = cv2.warpPerspective(frame, self.H, (self.WW, self.HH))
        else:
            view = frame

        overlay = view.copy()

        # Need warp AND ball to publish targets
        if not self.have_warp or self.ball_xy is None:
            cv2.imshow("frame", show)
            cv2.imshow("overlay", overlay)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                rclpy.shutdown()
            elif k == ord("c"):
                self.corners = []
                self.corner_mode = True
                self.have_warp = False
                self.H = None
                self.mask_ema = None
                self.get_logger().info("Click corners TL,TR,BR,BL")
            elif k == ord("r"):
                self.corners = []
                self.corner_mode = False
                self.have_warp = False
                self.H = None
                self.mask_ema = None
                self.get_logger().info("Reset")
            return

        # ---------- sliders ----------
        hmin = cv2.getTrackbarPos("H min", "tuner")
        hmax = cv2.getTrackbarPos("H max", "tuner")
        smin = cv2.getTrackbarPos("S min", "tuner")
        smax = cv2.getTrackbarPos("S max", "tuner")
        vmin = cv2.getTrackbarPos("V min", "tuner")
        vmax = cv2.getTrackbarPos("V max", "tuner")

        hole_vmax = cv2.getTrackbarPos("Hole V max", "tuner")
        hole_min_area = cv2.getTrackbarPos("Hole min area", "tuner")
        hole_min_circ = cv2.getTrackbarPos("Hole min circ x100", "tuner") / 100.0

        wall_vmax = cv2.getTrackbarPos("Wall V max", "tuner")
        wall_smax = cv2.getTrackbarPos("Wall S max", "tuner")
        wall_dilate = cv2.getTrackbarPos("Wall dilate px", "tuner")

        # ---------- track mask ----------
        hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (hmin,smin,vmin), (hmax,smax,vmax))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)),
                                iterations=1)
        track = largest_component(mask)

        if self.mask_ema is None or self.mask_ema.shape != track.shape:
            self.mask_ema = track.astype(np.float32)
        else:
            a = self.ema_alpha
            self.mask_ema = a*self.mask_ema + (1-a)*track.astype(np.float32)
        track_smooth = (self.mask_ema > 127).astype(np.uint8) * 255

        skel = skeletonize(track_smooth)
        path_px = order_skeleton(skel)

        # ---------- holes + walls ----------
        holes_px = detect_hole_centers(view, v_max=hole_vmax, min_area=hole_min_area, min_circ=hole_min_circ)
        if holes_px.shape[0] > 0:
            holes_xy_m = np.array([px_to_m(x,y,self.WW,self.HH,self.board_w_m,self.board_h_m) for x,y in holes_px],
                                  dtype=np.float32)
        else:
            holes_xy_m = np.zeros((0,2), dtype=np.float32)

        wall_mask = detect_walls_black_thick(view, v_max=wall_vmax, s_max=wall_smax, dilate_px=wall_dilate)
        wall_dist_map = wall_distance_m(wall_mask, self.board_w_m, self.board_h_m)

        # ---------- choose target on path ----------
        target_xy = None
        if len(path_px) >= 50:
            # convert path to meters
            path_xy_m = np.array([px_to_m(x,y,self.WW,self.HH,self.board_w_m,self.board_h_m) for (x,y) in path_px],
                                 dtype=np.float32)

            # nearest path point to ball
            d2 = np.sum((path_xy_m - self.ball_xy[None, :])**2, axis=1)
            i = int(np.argmin(d2))

            # walk forward until travel >= lookahead and safe
            dist = 0.0
            j = i
            max_steps = min(len(path_xy_m)-1, i + 1500)
            while j < max_steps:
                if j > i:
                    dist += float(np.linalg.norm(path_xy_m[j] - path_xy_m[j-1]))
                if dist >= self.lookahead_m:
                    pt = path_xy_m[j]
                    if self.safe_from_holes(pt, holes_xy_m) and self.safe_from_walls(pt, wall_dist_map):
                        target_xy = pt
                        break
                j += 1

            # fallback: nearest safe point around i
            if target_xy is None:
                for jj in range(i, min(i+800, len(path_xy_m))):
                    pt = path_xy_m[jj]
                    if self.safe_from_holes(pt, holes_xy_m) and self.safe_from_walls(pt, wall_dist_map):
                        target_xy = pt
                        break

        # ---------- publish ----------
        if target_xy is not None:
            msg = Point()
            msg.x = float(target_xy[0])
            msg.y = float(target_xy[1])
            msg.z = 0.0
            self.pub.publish(msg)

            # draw target + ball
            bu, bv = m_to_px(float(self.ball_xy[0]), float(self.ball_xy[1]),
                             self.WW, self.HH, self.board_w_m, self.board_h_m)
            tu, tv = m_to_px(float(target_xy[0]), float(target_xy[1]),
                             self.WW, self.HH, self.board_w_m, self.board_h_m)
            cv2.circle(overlay, (bu,bv), 6, (255,255,255), -1)  # ball
            cv2.circle(overlay, (tu,tv), 6, (0,255,255), -1)    # target

        # draw skeleton, walls, holes
        overlay[skel > 0] = (0,0,255)
        overlay[wall_mask > 0] = (255,0,0)
        for (x,y) in holes_px.astype(np.int32):
            cv2.circle(overlay, (int(x), int(y)), 8, (0,255,0), 2)

        cv2.imshow("frame", show)
        cv2.imshow("overlay", overlay)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            rclpy.shutdown()
        elif k == ord("c"):
            self.corners = []
            self.corner_mode = True
            self.have_warp = False
            self.H = None
            self.mask_ema = None
            self.get_logger().info("Click corners TL,TR,BR,BL")
        elif k == ord("r"):
            self.corners = []
            self.corner_mode = False
            self.have_warp = False
            self.H = None
            self.mask_ema = None
            self.get_logger().info("Reset")

    def destroy_node(self):
        try:
            self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = OrangeLiveTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
