#!/usr/bin/env python3
import os
import numpy as np
import cv2
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory
from cyberrunner_interfaces.msg import StateEstimate


# ===================== Markers / Warp =====================
def load_markers_csv_from_share(pkg_name="cyberrunner_state_estimation", fname="markers.csv"):
    shared = get_package_share_directory(pkg_name)
    path = os.path.join(shared, fname)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing markers file: {path}")
    m = np.loadtxt(path, delimiter=",").astype(np.float32)
    if m.shape[0] < 8:
        raise RuntimeError(f"markers.csv must have 8 points (outer+inner), got {m.shape}")
    return m, path


def compute_warp_from_markers_4567(markers_xy):
    """
    Use INNER markers: indices 4..7
    Expected order from your selector:
      4 lower-left, 5 lower-right, 6 upper-right, 7 upper-left
    """
    ll = markers_xy[4]
    lr = markers_xy[5]
    ur = markers_xy[6]
    ul = markers_xy[7]

    w1 = np.linalg.norm(lr - ll)
    w2 = np.linalg.norm(ur - ul)
    h1 = np.linalg.norm(ul - ll)
    h2 = np.linalg.norm(ur - lr)

    W = int(round(max(w1, w2)))
    H = int(round(max(h1, h2)))
    W = max(400, W)
    H = max(400, H)

    src = np.array([ul, ur, lr, ll], dtype=np.float32)
    dst = np.array([[0, 0], [W - 1, 0], [W - 1, H - 1], [0, H - 1]], dtype=np.float32)
    Hm = cv2.getPerspectiveTransform(src, dst)
    return Hm, W, H


def m_centered_to_px(x_m, y_m, W, H, board_w_m, board_h_m):
    u = int(round((x_m / board_w_m) * W + 0.5 * W))
    v = int(round(0.5 * H - (y_m / board_h_m) * H))
    return max(0, min(u, W - 1)), max(0, min(v, H - 1))


def px_to_m_centered(u, v, W, H, board_w_m, board_h_m):
    x_m = ((u / max(1.0, W - 1)) - 0.5) * board_w_m
    y_m = (0.5 - (v / max(1.0, H - 1))) * board_h_m
    return float(x_m), float(y_m)


# ===================== Vision helpers =====================
def skeletonize(binary_mask: np.ndarray) -> np.ndarray:
    """
    Morphological skeletonization.
    Input: uint8 0/255
    Output: uint8 0/255
    """
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


def detect_holes_mask_and_centers(warp_bgr, v_max=60, min_area=250, max_area=50000, min_circ=0.55):
    """
    Detect dark circular holes using V channel threshold.
    Returns: (centers Nx2 float32), (hole_mask uint8 0/255 filled)
    """
    hsv = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2HSV)
    V = hsv[:, :, 2]
    hole = (V <= v_max).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    hole = cv2.morphologyEx(hole, cv2.MORPH_OPEN, k, iterations=1)
    hole = cv2.morphologyEx(hole, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(hole, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    hole_mask = np.zeros_like(hole)

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

        # Fill the hole as obstacle
        cv2.drawContours(hole_mask, [c], -1, 255, -1)

    return np.array(centers, dtype=np.float32), hole_mask


def detect_walls_black_thick(warp_bgr, v_max=65, s_max=140, dilate_px=0):
    """
    Your existing wall detector (dark + low saturation).
    Returns wall mask 0/255.
    """
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


def nearest_skel_point(skel_255: np.ndarray, uv):
    """
    Snap a point to nearest skeleton pixel.
    """
    if uv is None:
        return None
    u0, v0 = int(uv[0]), int(uv[1])
    h, w = skel_255.shape

    # Fast local search
    for rad in [10, 20, 35, 60, 90]:
        u1 = max(0, u0 - rad); u2 = min(w - 1, u0 + rad)
        v1 = max(0, v0 - rad); v2 = min(h - 1, v0 + rad)
        roi = skel_255[v1:v2+1, u1:u2+1]
        ys, xs = np.where(roi > 0)
        if xs.size > 0:
            xs = xs + u1
            ys = ys + v1
            d2 = (xs - u0)**2 + (ys - v0)**2
            i = int(np.argmin(d2))
            return (int(xs[i]), int(ys[i]))

    # Global fallback (rare)
    ys, xs = np.where(skel_255 > 0)
    if xs.size == 0:
        return None
    d2 = (xs - u0)**2 + (ys - v0)**2
    i = int(np.argmin(d2))
    return (int(xs[i]), int(ys[i]))


def bfs_path_on_skeleton(skel_255: np.ndarray, start_uv, goal_uv):
    """
    BFS on 8-connected skeleton pixels.
    Returns list of (u,v) from start->goal or None.
    """
    if start_uv is None or goal_uv is None:
        return None
    if start_uv == goal_uv:
        return [start_uv]

    h, w = skel_255.shape
    skel = (skel_255 > 0)

    sx, sy = start_uv
    gx, gy = goal_uv

    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None
    if not skel[sy, sx] or not skel[gy, gx]:
        return None

    q = deque()
    q.append((sx, sy))
    prev = { (sx, sy): None }

    nbrs = [(-1,-1),(0,-1),(1,-1),
            (-1, 0),      (1, 0),
            (-1, 1),(0, 1),(1, 1)]

    while q:
        x, y = q.popleft()
        if (x, y) == (gx, gy):
            break
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and skel[ny, nx]:
                if (nx, ny) not in prev:
                    prev[(nx, ny)] = (x, y)
                    q.append((nx, ny))

    if (gx, gy) not in prev:
        return None

    # reconstruct
    path = []
    cur = (gx, gy)
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


def downsample_path(path, step=3):
    if path is None or len(path) < 2:
        return path
    out = [path[0]]
    for i in range(step, len(path), step):
        out.append(path[i])
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


# ===================== Node =====================
class WallHolePathTarget(Node):
    def __init__(self):
        super().__init__("wall_hole_path_target_node")

        self.declare_parameter("camera_topic", "/cyberrunner_camera/image")
        self.declare_parameter("estimate_topic", "/cyberrunner_state_estimation/state_estimate")
        self.declare_parameter("target_topic", "/ball/target")

        self.declare_parameter("board_w_m", 0.30)
        self.declare_parameter("board_h_m", 0.20)

        self.declare_parameter("lookahead_m", 0.03)
        self.declare_parameter("arrow_len_m", 0.01)      # 1cm

        self.declare_parameter("ema_alpha", 0.85)

        self.board_w_m = float(self.get_parameter("board_w_m").value)
        self.board_h_m = float(self.get_parameter("board_h_m").value)
        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.arrow_len_m = float(self.get_parameter("arrow_len_m").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)

        # obstacle mask EMA (stabilize)
        self.obs_ema = None

        markers, marker_path = load_markers_csv_from_share()
        self.Hwarp, self.WW, self.HH = compute_warp_from_markers_4567(markers)
        self.get_logger().info(f"Using markers file: {marker_path}")
        self.get_logger().info(f"Warp using INNER markers 4-5-6-7. Size: {self.WW} x {self.HH}")

        self.br = CvBridge()
        self.last_frame = None
        self.ball_xy_m = None

        self.goal_uv = None  # clicked goal in warp px (snapped each frame)

        self.create_subscription(Image, self.get_parameter("camera_topic").value, self.on_image, 10)
        self.create_subscription(StateEstimate, self.get_parameter("estimate_topic").value, self.on_est, 10)
        self.pub_target = self.create_publisher(Point, self.get_parameter("target_topic").value, 10)

        # windows
        cv2.namedWindow("orange_live_overlay", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("obstacle_mask", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("skeleton", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("tuner", cv2.WINDOW_NORMAL)

        cv2.resizeWindow("orange_live_overlay", 1200, 800)
        cv2.resizeWindow("obstacle_mask", 900, 700)
        cv2.resizeWindow("skeleton", 900, 700)
        cv2.resizeWindow("tuner", 720, 420)

        cv2.setMouseCallback("orange_live_overlay", self.on_mouse)

        def nothing(_): pass

        # ---- Hole tuning
        cv2.createTrackbar("Hole V max", "tuner", 60, 255, nothing)
        cv2.createTrackbar("Hole min area", "tuner", 250, 20000, nothing)
        cv2.createTrackbar("Hole min circ x100", "tuner", 55, 100, nothing)

        # ---- Wall tuning
        cv2.createTrackbar("Wall V max", "tuner", 65, 255, nothing)
        cv2.createTrackbar("Wall S max", "tuner", 140, 255, nothing)
        cv2.createTrackbar("Wall dilate px", "tuner", 2, 12, nothing)

        # ---- Centerline tuning (distance transform threshold)
        cv2.createTrackbar("Center dist thresh", "tuner", 22, 80, nothing)  # 18-35 typical
        cv2.createTrackbar("Obs close k", "tuner", 7, 31, nothing)
        cv2.createTrackbar("Obs close iters", "tuner", 2, 10, nothing)

        self.timer = self.create_timer(0.03, self.loop)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            self.goal_uv = (int(x), int(y))
            self.get_logger().info(f"Clicked goal (warp px): {self.goal_uv}")

    def on_image(self, msg: Image):
        self.last_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_est(self, msg: StateEstimate):
        x = float(msg.x_b); y = float(msg.y_b)
        if np.isfinite(x) and np.isfinite(y):
            self.ball_xy_m = np.array([x, y], dtype=np.float32)
        else:
            self.ball_xy_m = None

    def put_status(self, img, lines, color=(0, 0, 255)):
        y = 30
        for s in lines:
            cv2.putText(img, s, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
            y += 28

    def loop(self):
        if self.last_frame is None:
            img = np.zeros((600, 900, 3), dtype=np.uint8)
            self.put_status(img, ["WAITING FOR CAMERA FRAME..."], (0, 0, 255))
            cv2.imshow("orange_live_overlay", img)
            cv2.waitKey(1)
            return

        warp = cv2.warpPerspective(self.last_frame, self.Hwarp, (self.WW, self.HH))
        overlay = warp.copy()
        status = []

        # sliders
        hole_vmax = cv2.getTrackbarPos("Hole V max", "tuner")
        hole_min_area = cv2.getTrackbarPos("Hole min area", "tuner")
        hole_min_circ = cv2.getTrackbarPos("Hole min circ x100", "tuner") / 100.0

        wall_vmax = cv2.getTrackbarPos("Wall V max", "tuner")
        wall_smax = cv2.getTrackbarPos("Wall S max", "tuner")
        wall_dilate = cv2.getTrackbarPos("Wall dilate px", "tuner")

        center_thresh = cv2.getTrackbarPos("Center dist thresh", "tuner")

        obs_close_k = cv2.getTrackbarPos("Obs close k", "tuner")
        obs_close_k = max(3, obs_close_k | 1)
        obs_close_it = cv2.getTrackbarPos("Obs close iters", "tuner")

        # ---- Detect holes + walls (obstacles)
        holes_px, hole_mask = detect_holes_mask_and_centers(
            warp, v_max=hole_vmax, min_area=hole_min_area, min_circ=hole_min_circ
        )
        wall_mask = detect_walls_black_thick(
            warp, v_max=wall_vmax, s_max=wall_smax, dilate_px=wall_dilate
        )

        obs = cv2.bitwise_or(wall_mask, hole_mask)

        # close obstacles to avoid leaks
        kclose = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (obs_close_k, obs_close_k))
        obs = cv2.morphologyEx(obs, cv2.MORPH_CLOSE, kclose, iterations=max(1, obs_close_it))

        # EMA stabilize obstacle mask (helps flicker)
        if self.obs_ema is None or self.obs_ema.shape != obs.shape:
            self.obs_ema = obs.astype(np.float32)
        else:
            a = self.ema_alpha
            self.obs_ema = a * self.obs_ema + (1.0 - a) * obs.astype(np.float32)
        obs_smooth = (self.obs_ema > 127).astype(np.uint8) * 255

        cv2.imshow("obstacle_mask", obs_smooth)

        # overlay obstacle visualization
        overlay[wall_mask > 0] = (255, 0, 0)  # blue walls
        for (x, y) in holes_px.astype(np.int32):
            cv2.circle(overlay, (int(x), int(y)), 8, (0, 255, 0), 2)  # green holes

        status.append(f"holes={holes_px.shape[0]}  wall_dilate={wall_dilate}  center_thr={center_thresh}")

        # ---- Free space -> distance transform -> center-ish -> skeleton
        free = cv2.bitwise_not(obs_smooth)
        free = cv2.morphologyEx(
            free, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1
        )

        free_u8 = (free > 0).astype(np.uint8)
        dist = cv2.distanceTransform(free_u8, cv2.DIST_L2, 5)
        dist_norm = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        centerish = (dist_norm > max(5, center_thresh)).astype(np.uint8) * 255
        centerish = cv2.morphologyEx(centerish, cv2.MORPH_OPEN,
                                     cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=1)

        skel = skeletonize(centerish)
        cv2.imshow("skeleton", skel)

        skel_count = int(cv2.countNonZero(skel))
        status.append(f"skel_px={skel_count}")

        if skel_count < 200:
            status.append("SKELETON TOO SMALL -> reduce Center dist thresh OR wall_dilate")
            if self.goal_uv is not None:
                cv2.circle(overlay, self.goal_uv, 10, (0, 0, 255), 2)
            self.put_status(overlay, status, (0, 0, 255))
            cv2.imshow("orange_live_overlay", overlay)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                rclpy.shutdown()
            return

        # ---- Ball (from state estimate)
        if self.ball_xy_m is None:
            status.append("NO BALL STATE (waiting on estimate topic)")
            if self.goal_uv is not None:
                cv2.circle(overlay, self.goal_uv, 10, (0, 0, 255), 2)
            self.put_status(overlay, status, (0, 0, 255))
            cv2.imshow("orange_live_overlay", overlay)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                rclpy.shutdown()
            return

        bu, bv = m_centered_to_px(float(self.ball_xy_m[0]), float(self.ball_xy_m[1]),
                                  self.WW, self.HH, self.board_w_m, self.board_h_m)

        cv2.circle(overlay, (bu, bv), 7, (255, 255, 255), -1)
        cv2.circle(overlay, (bu, bv), 7, (0, 0, 0), 1)

        # ---- Snap ball + goal to skeleton
        ball_s = nearest_skel_point(skel, (bu, bv))

        goal_s = None
        if self.goal_uv is not None:
            goal_s = nearest_skel_point(skel, self.goal_uv)

        if ball_s is None:
            status.append("ball cannot snap to skeleton -> reduce Center dist thresh")
            if goal_s is not None:
                cv2.circle(overlay, goal_s, 10, (0, 0, 255), 2)
            self.put_status(overlay, status, (0, 0, 255))
            cv2.imshow("orange_live_overlay", overlay)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                rclpy.shutdown()
            return

        cv2.circle(overlay, ball_s, 4, (0, 255, 255), -1)  # yellow snap point

        if goal_s is not None:
            self.goal_uv = goal_s  # keep snapped
            cv2.circle(overlay, goal_s, 10, (0, 0, 255), 2)

        # ---- Path (only if goal exists)
        path = None
        if goal_s is not None:
            path = bfs_path_on_skeleton(skel, ball_s, goal_s)
            if path is None or len(path) < 2:
                status.append("NO PATH -> walls mask leaking or skeleton disconnected")
                self.put_status(overlay, status, (0, 0, 255))
                cv2.imshow("orange_live_overlay", overlay)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    rclpy.shutdown()
                return

        # ---- If no goal, just show local forward direction (do not publish)
        if path is None:
            status.append("click target to compute path")
            self.put_status(overlay, status, (0, 0, 255))
            cv2.imshow("orange_live_overlay", overlay)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                rclpy.shutdown()
            return

        # downsample for drawing speed
        path = downsample_path(path, step=2)

        # draw only forward path from ball
        for (u, v) in path:
            overlay[v, u] = (0, 0, 255)  # red

        status.append(f"path_len={len(path)}")

        # ---- Arrow (1 cm)
        m_per_px = 0.5 * (self.board_w_m / max(1.0, self.WW - 1) + self.board_h_m / max(1.0, self.HH - 1))
        m_per_px = max(1e-6, m_per_px)
        arrow_len_px = int(max(5, round(self.arrow_len_m / m_per_px)))

        # direction: toward a point ahead on path
        j = min(15, len(path) - 1)
        p0 = np.array([bu, bv], dtype=np.float32)
        p1 = np.array(path[j], dtype=np.float32)
        d = p1 - p0
        dn = float(np.linalg.norm(d))
        if dn > 1e-6:
            dir_uv = d / dn
        else:
            dir_uv = np.array([1.0, 0.0], dtype=np.float32)

        end_u = int(round(bu + dir_uv[0] * arrow_len_px))
        end_v = int(round(bv + dir_uv[1] * arrow_len_px))
        end_u = max(0, min(end_u, self.WW - 1))
        end_v = max(0, min(end_v, self.HH - 1))
        cv2.arrowedLine(overlay, (bu, bv), (end_u, end_v), (0, 0, 255), 2, tipLength=0.35)

        # ---- Publish lookahead target point
        lookahead_px = int(max(10, round(self.lookahead_m / m_per_px)))
        jj = min(max(1, lookahead_px), len(path) - 1)
        tgt_u, tgt_v = int(path[jj][0]), int(path[jj][1])
        cv2.circle(overlay, (tgt_u, tgt_v), 6, (0, 0, 255), -1)

        tx, ty = px_to_m_centered(tgt_u, tgt_v, self.WW, self.HH, self.board_w_m, self.board_h_m)
        out = Point()
        out.x, out.y, out.z = float(tx), float(ty), 0.0
        self.pub_target.publish(out)

        status.append(f"ball_px=({bu},{bv}) lookahead_px={lookahead_px}")
        self.put_status(overlay, status, (0, 0, 255))

        cv2.imshow("orange_live_overlay", overlay)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallHolePathTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
