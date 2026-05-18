#!/usr/bin/env python3
"""
maze_solve_pid_hiwonder_click_goal.py

FULL ONE-FILE SOLUTION:
- Subscribes to camera Image
- Warps image using your markers.csv outer markers indices 4..7
- Builds obstacle/free map from ORANGE line + (optional) holes
- A* plans from current ball position -> GOAL (set by mouse click on warped view)
- Converts path -> spaced waypoints
- PD/PID controller -> stepped servo positions (step=25)
- Auto return-to-center (500) when close/slow
- Publishes to your existing Hiwonder servo node topic: /hiwonder/cmd
  Int32MultiArray: [pos1, pos2, time_ms]

How to use:
1) Start your estimator node (publishes StateEstimate)
2) Start your hiwonder servo node (subscribes /hiwonder/cmd)
3) Run this node
4) Click on the finish cell (number 60) in the "warp_map" window to set the goal.
"""

import os, math, heapq
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray

from ament_index_python.packages import get_package_share_directory
from cyberrunner_interfaces.msg import StateEstimate


# ----------------------- helpers -----------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def quantize_around_center(pos, center, step_size):
    off = pos - center
    off_q = int(round(off / step_size)) * step_size
    return int(center + off_q)

def load_markers_csv_from_share(pkg_name="cyberrunner_state_estimation", fname="markers.csv"):
    shared = get_package_share_directory(pkg_name)
    path = os.path.join(shared, fname)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing markers file: {path}")
    m = np.loadtxt(path, delimiter=",").astype(np.float32)
    if m.shape[0] < 8:
        raise RuntimeError(f"markers.csv must have 8 points (outer+inner), got {m.shape}")
    return m, path

def compute_warp_from_outer_markers_auto(markers_xy):
    # indices 4..7: ll, lr, ur, ul (as in your other code)
    ll = markers_xy[4]; lr = markers_xy[5]; ur = markers_xy[6]; ul = markers_xy[7]
    w1 = np.linalg.norm(lr - ll)
    w2 = np.linalg.norm(ur - ul)
    h1 = np.linalg.norm(ul - ll)
    h2 = np.linalg.norm(ur - lr)
    WW = int(round(max(w1, w2)))
    HH = int(round(max(h1, h2)))
    WW = max(200, WW); HH = max(200, HH)
    src = np.array([ul, ur, lr, ll], dtype=np.float32)
    dst = np.array([[0, 0], [WW - 1, 0], [WW - 1, HH - 1], [0, HH - 1]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    return H, WW, HH

def m_to_px_on_warp(x_m, y_m, WW, HH, board_w_m, board_h_m):
    u = int(round((x_m / board_w_m) * WW + 0.5 * WW))
    v = int(round(0.5 * HH - (y_m / board_h_m) * HH))
    u = max(0, min(u, WW - 1))
    v = max(0, min(v, HH - 1))
    return u, v

def px_to_m_on_warp(u, v, WW, HH, board_w_m, board_h_m):
    x_m = ((u - 0.5 * WW) / WW) * board_w_m
    y_m = ((0.5 * HH - v) / HH) * board_h_m
    return float(x_m), float(y_m)

def nearest_free(free_u8, start_uv, max_r=80):
    """If point is not free, find nearest free pixel in expanding square rings."""
    H, W = free_u8.shape
    sx, sy = start_uv
    if 0 <= sx < W and 0 <= sy < H and free_u8[sy, sx] != 0:
        return (sx, sy)
    for r in range(1, max_r + 1):
        x0, x1 = max(0, sx - r), min(W - 1, sx + r)
        y0, y1 = max(0, sy - r), min(H - 1, sy + r)
        for x in range(x0, x1 + 1):
            if free_u8[y0, x] != 0: return (x, y0)
            if free_u8[y1, x] != 0: return (x, y1)
        for y in range(y0, y1 + 1):
            if free_u8[y, x0] != 0: return (x0, y)
            if free_u8[y, x1] != 0: return (x1, y)
    return None

def astar_on_free(free_bool, start, goal):
    """
    free_bool: HxW bool (True=free)
    start/goal: (u,v)
    returns list[(u,v)] or []
    """
    H, W = free_bool.shape
    sx, sy = start
    gx, gy = goal

    def inb(x, y): return 0 <= x < W and 0 <= y < H
    def h(x, y):   return abs(x - gx) + abs(y - gy)

    neigh = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

    openpq = []
    heapq.heappush(openpq, (h(sx, sy), 0.0, (sx, sy)))
    came = {}
    gscore = {(sx, sy): 0.0}

    while openpq:
        f, g, cur = heapq.heappop(openpq)
        if cur == (gx, gy):
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path

        cx, cy = cur
        for dx, dy in neigh:
            nx, ny = cx + dx, cy + dy
            if not inb(nx, ny): 
                continue
            if not free_bool[ny, nx]:
                continue
            step = math.hypot(dx, dy)
            ng = g + step
            if (nx, ny) not in gscore or ng < gscore[(nx, ny)]:
                gscore[(nx, ny)] = ng
                came[(nx, ny)] = (cx, cy)
                heapq.heappush(openpq, (ng + h(nx, ny), ng, (nx, ny)))

    return []

def sparsify_path(path_uv, spacing_px=35):
    if not path_uv:
        return []
    out = [path_uv[0]]
    lx, ly = path_uv[0]
    spacing2 = float(spacing_px * spacing_px)
    for (x, y) in path_uv[1:]:
        if (x - lx) * (x - lx) + (y - ly) * (y - ly) >= spacing2:
            out.append((x, y))
            lx, ly = x, y
    if out[-1] != path_uv[-1]:
        out.append(path_uv[-1])
    return out


# ----------------------- node -----------------------
class MazeSolvePidHiwonder(Node):
    def __init__(self):
        super().__init__("maze_solve_pid_hiwonder_click_goal")
        self.br = CvBridge()

        # ===== topics =====
        self.declare_parameter("image_topic", "/cyberrunner_camera/image")
        self.declare_parameter("state_topic", "/cyberrunner_state_estimation/state_estimate")
        self.declare_parameter("servo_cmd_topic", "/hiwonder/cmd")

        # ===== board meters (match your estimator calibration) =====
        self.declare_parameter("board_w_m", 0.30)
        self.declare_parameter("board_h_m", 0.20)

        # ===== planning update rate =====
        self.declare_parameter("plan_every_s", 0.5)

        # ===== warp size override (0 = auto from markers) =====
        self.declare_parameter("warp_w_px", 0)
        self.declare_parameter("warp_h_px", 0)

        # ===== goal pixel default (will be replaced by click) =====
        self.declare_parameter("goal_u", 50)
        self.declare_parameter("goal_v", 50)

        # ===== map construction =====
        self.declare_parameter("ball_radius_px", 10)
        self.declare_parameter("inflate_extra_px", 3)

        # ORANGE detection thresholds (HSV) — tune for your lighting
        self.declare_parameter("orange_h_min", 5)
        self.declare_parameter("orange_h_max", 30)
        self.declare_parameter("orange_s_min", 60)
        self.declare_parameter("orange_v_min", 60)

        # connect gaps in wall mask
        self.declare_parameter("wall_close_k", 5)
        self.declare_parameter("wall_close_iter", 1)

        # holes (optional)
        self.declare_parameter("detect_holes", True)
        self.declare_parameter("holes_min_r_px", 10)
        self.declare_parameter("holes_max_r_px", 25)

        # path -> waypoints
        self.declare_parameter("waypoint_spacing_px", 35)
        self.declare_parameter("reach_r_m", 0.015)

        # controller gains
        self.declare_parameter("Kp", 1200.0)
        self.declare_parameter("Kd", 350.0)
        self.declare_parameter("Ki", 0.0)

        # stepped output
        self.declare_parameter("step_size", 25)
        self.declare_parameter("step_max", 25)

        # servo limits
        self.declare_parameter("s1_center", 500)
        self.declare_parameter("s2_center", 500)
        self.declare_parameter("s1_min", 350)
        self.declare_parameter("s1_max", 650)
        self.declare_parameter("s2_min", 350)
        self.declare_parameter("s2_max", 650)
        self.declare_parameter("u_max", 120.0)
        self.declare_parameter("move_time_ms", 40)

        # return-to-center stability
        self.declare_parameter("return_err_m", 0.010)
        self.declare_parameter("return_vel_mps", 0.030)

        # axis mapping / sign (fix direction without changing code)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("sign_s1", +1.0)
        self.declare_parameter("sign_s2", +1.0)

        # debug
        self.declare_parameter("show_debug", True)

        # ----- warp from markers -----
        markers, marker_path = load_markers_csv_from_share()
        self.Hwarp, autoW, autoH = compute_warp_from_outer_markers_auto(markers)

        Wp = int(self.get_parameter("warp_w_px").value)
        Hp = int(self.get_parameter("warp_h_px").value)
        self.WW = autoW if Wp <= 0 else Wp
        self.HH = autoH if Hp <= 0 else Hp

        self.board_w_m = float(self.get_parameter("board_w_m").value)
        self.board_h_m = float(self.get_parameter("board_h_m").value)

        # goal (click will overwrite)
        self.goal_uv = (int(self.get_parameter("goal_u").value),
                        int(self.get_parameter("goal_v").value))

        self.show_debug = bool(self.get_parameter("show_debug").value)
        if self.show_debug:
            cv2.namedWindow("warp_map", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow("warp_map", 960, 600)
            cv2.namedWindow("free_grid", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow("free_grid", 480, 300)

            # mouse callback to set goal (click on "60")
            def on_mouse(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    self.goal_uv = (int(x), int(y))
                    # also update ROS params so it's visible/savable
                    self.set_parameters([
                        Parameter("goal_u", Parameter.Type.INTEGER, int(x)),
                        Parameter("goal_v", Parameter.Type.INTEGER, int(y)),
                    ])
                    self.get_logger().info(f"GOAL set by click: warp px ({x}, {y})")

            cv2.setMouseCallback("warp_map", on_mouse)

        self.get_logger().info(f"Using markers: {marker_path}")
        self.get_logger().info(f"Warp size: {self.WW} x {self.HH}")
        self.get_logger().info("Click on the finish cell (60) in 'warp_map' to set the goal.")

        # ----- ROS I/O -----
        self.sub_img = self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, 10)
        self.sub_state = self.create_subscription(StateEstimate, self.get_parameter("state_topic").value, self.on_state, 10)
        self.pub_cmd = self.create_publisher(Int32MultiArray, self.get_parameter("servo_cmd_topic").value, 10)

        # ----- caches -----
        self.last_frame = None
        self.last_ball = None         # (x,y,vx,vy) meters
        self.last_ball_px = None      # (u,v)

        self.last_plan_t = 0.0
        self.waypoints_m = []
        self.wp_i = 0

        self.last_t = None
        self.Ix = 0.0
        self.Iy = 0.0
        self.Imax = 0.50

        self.s1_last = int(self.get_parameter("s1_center").value)
        self.s2_last = int(self.get_parameter("s2_center").value)

    # ---------- callbacks ----------
    def on_image(self, msg: Image):
        self.last_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_state(self, msg: StateEstimate):
        x = float(msg.x_b);  y = float(msg.y_b)
        vx = float(msg.x_b_dot); vy = float(msg.y_b_dot)
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(vx) and np.isfinite(vy)):
            return
        self.last_ball = (x, y, vx, vy)

        u, v = m_to_px_on_warp(x, y, self.WW, self.HH, self.board_w_m, self.board_h_m)
        self.last_ball_px = (u, v)

        self.tick()

    # ---------- map ----------
    def build_obstacle_map(self, warp_bgr):
        hsv = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2HSV)

        hmin = int(self.get_parameter("orange_h_min").value)
        hmax = int(self.get_parameter("orange_h_max").value)
        smin = int(self.get_parameter("orange_s_min").value)
        vmin = int(self.get_parameter("orange_v_min").value)

        wall = cv2.inRange(hsv, (hmin, smin, vmin), (hmax, 255, 255))

        # close gaps
        ksz = int(self.get_parameter("wall_close_k").value)
        ksz = max(3, ksz | 1)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (ksz, ksz))
        it = int(self.get_parameter("wall_close_iter").value)
        wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, k, iterations=max(1, it))

        # thicken a bit
        wall = cv2.dilate(wall, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1)

        holes_list = []
        hole_mask = np.zeros_like(wall)

        if bool(self.get_parameter("detect_holes").value):
            gray = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2GRAY)
            gray_blur = cv2.GaussianBlur(gray, (9, 9), 1.5)

            minr = int(self.get_parameter("holes_min_r_px").value)
            maxr = int(self.get_parameter("holes_max_r_px").value)

            circles = cv2.HoughCircles(
                gray_blur, cv2.HOUGH_GRADIENT,
                dp=1.2, minDist=30,
                param1=120, param2=18,
                minRadius=minr, maxRadius=maxr
            )
            if circles is not None:
                circles = np.round(circles[0]).astype(int)
                for (cx, cy, r) in circles:
                    if 0 <= cx < self.WW and 0 <= cy < self.HH:
                        holes_list.append((cx, cy, r))
                        cv2.circle(hole_mask, (cx, cy), r + 2, 255, -1)

        obstacles = cv2.bitwise_or(wall, hole_mask)

        # inflate by ball radius
        ball_r = int(self.get_parameter("ball_radius_px").value)
        extra = int(self.get_parameter("inflate_extra_px").value)
        infl = max(1, ball_r + extra)
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * infl + 1, 2 * infl + 1))
        obstacles = cv2.dilate(obstacles, ker, iterations=1)

        free = cv2.bitwise_not(obstacles)  # 255 free
        # block borders slightly
        free[0:2, :] = 0; free[-2:, :] = 0; free[:, 0:2] = 0; free[:, -2:] = 0

        return obstacles, free, holes_list

    # ---------- planning ----------
    def plan_if_needed(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        plan_every = float(self.get_parameter("plan_every_s").value)
        if (now - self.last_plan_t) < plan_every:
            return

        if self.last_frame is None or self.last_ball_px is None:
            return

        warp = cv2.warpPerspective(self.last_frame, self.Hwarp, (self.WW, self.HH))
        obstacles, free, holes = self.build_obstacle_map(warp)
        free_bool = (free > 0)

        start = nearest_free(free, self.last_ball_px, max_r=80)
        goal = nearest_free(free, self.goal_uv, max_r=80)

        wp_px = []
        no_path = False

        if start is None or goal is None:
            self.waypoints_m = []
            self.wp_i = 0
            no_path = True
        else:
            path = astar_on_free(free_bool, start, goal)
            if not path:
                self.waypoints_m = []
                self.wp_i = 0
                no_path = True
            else:
                spacing = int(self.get_parameter("waypoint_spacing_px").value)
                wp_px = sparsify_path(path, spacing_px=max(10, spacing))
                self.waypoints_m = [
                    px_to_m_on_warp(u, v, self.WW, self.HH, self.board_w_m, self.board_h_m)
                    for (u, v) in wp_px
                ]
                self.wp_i = 0

        self.last_plan_t = now

        if self.show_debug:
            self.show_debug_view(warp, obstacles, free, holes, wp_px, start, goal, no_path=no_path)

    def current_target(self):
        if not self.waypoints_m or self.wp_i >= len(self.waypoints_m):
            return None
        return self.waypoints_m[self.wp_i]

    def advance_waypoint(self, ex, ey):
        reach_r = float(self.get_parameter("reach_r_m").value)
        if self.waypoints_m and self.wp_i < len(self.waypoints_m):
            if math.hypot(ex, ey) < reach_r:
                self.wp_i += 1

    # ---------- control ----------
    def publish_servo(self, s1, s2):
        t_ms = int(self.get_parameter("move_time_ms").value)
        out = Int32MultiArray()
        out.data = [int(s1), int(s2), int(t_ms)]
        self.pub_cmd.publish(out)

    def tick(self):
        # planning
        self.plan_if_needed()

        if self.last_ball is None:
            return

        # dt
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.2:
            return

        x, y, vx, vy = self.last_ball

        s1c = int(self.get_parameter("s1_center").value)
        s2c = int(self.get_parameter("s2_center").value)

        tgt = self.current_target()
        if tgt is None:
            # no path -> home
            self.Ix = 0.0; self.Iy = 0.0
            self.s1_last = s1c; self.s2_last = s2c
            self.publish_servo(s1c, s2c)
            return

        gx, gy = tgt
        ex = gx - x
        ey = gy - y

        # advance + recompute
        self.advance_waypoint(ex, ey)
        tgt = self.current_target()
        if tgt is None:
            self.publish_servo(s1c, s2c)
            return
        gx, gy = tgt
        ex = gx - x
        ey = gy - y

        err = math.hypot(ex, ey)
        speed = math.hypot(vx, vy)

        return_err = float(self.get_parameter("return_err_m").value)
        return_vel = float(self.get_parameter("return_vel_mps").value)

        if err < return_err or speed < return_vel:
            s1_cmd = s1c
            s2_cmd = s2c
            self.Ix = 0.0; self.Iy = 0.0
        else:
            Kp = float(self.get_parameter("Kp").value)
            Kd = float(self.get_parameter("Kd").value)
            Ki = float(self.get_parameter("Ki").value)

            if Ki != 0.0:
                self.Ix = clamp(self.Ix + ex * dt, -self.Imax, self.Imax)
                self.Iy = clamp(self.Iy + ey * dt, -self.Imax, self.Imax)

            swap_xy = bool(self.get_parameter("swap_xy").value)
            sign1 = float(self.get_parameter("sign_s1").value)
            sign2 = float(self.get_parameter("sign_s2").value)

            if not swap_xy:
                u1 = sign1 * (Kp * ex - Kd * vx + Ki * self.Ix)
                u2 = sign2 * (Kp * ey - Kd * vy + Ki * self.Iy)
            else:
                u1 = sign1 * (Kp * ey - Kd * vy + Ki * self.Iy)
                u2 = sign2 * (Kp * ex - Kd * vx + Ki * self.Ix)

            u_max = float(self.get_parameter("u_max").value)
            u1 = clamp(u1, -u_max, u_max)
            u2 = clamp(u2, -u_max, u_max)

            s1_cmd = int(round(s1c + u1))
            s2_cmd = int(round(s2c + u2))

            # stepped output
            step_size = int(self.get_parameter("step_size").value)
            s1_cmd = quantize_around_center(s1_cmd, s1c, step_size)
            s2_cmd = quantize_around_center(s2_cmd, s2c, step_size)

            # clamp servo hard limits
            s1_cmd = clamp(s1_cmd, int(self.get_parameter("s1_min").value), int(self.get_parameter("s1_max").value))
            s2_cmd = clamp(s2_cmd, int(self.get_parameter("s2_min").value), int(self.get_parameter("s2_max").value))

        # rate limit
        step_max = int(self.get_parameter("step_max").value)
        s1_cmd = clamp(s1_cmd, self.s1_last - step_max, self.s1_last + step_max)
        s2_cmd = clamp(s2_cmd, self.s2_last - step_max, self.s2_last + step_max)
        self.s1_last = s1_cmd
        self.s2_last = s2_cmd

        self.publish_servo(s1_cmd, s2_cmd)

    # ---------- debug ----------
    def show_debug_view(self, warp, obstacles, free, holes_list, wp_px, start_uv, goal_uv, no_path=False):
        vis = warp.copy()

        # obstacles overlay
        obs = (obstacles > 0).astype(np.uint8) * 255
        tint = np.zeros_like(vis)
        tint[:, :, 2] = obs
        vis = cv2.addWeighted(vis, 1.0, tint, 0.35, 0.0)

        # holes
        for (cx, cy, r) in holes_list:
            cv2.circle(vis, (cx, cy), r, (0, 255, 0), 2)

        # start / goal / click hint
        if start_uv is not None:
            cv2.circle(vis, start_uv, 6, (0, 255, 255), -1)
        if goal_uv is not None:
            cv2.circle(vis, goal_uv, 8, (0, 0, 255), 2)

        # waypoints
        for (u, v) in wp_px:
            cv2.circle(vis, (u, v), 4, (255, 255, 0), -1)

        msg = "CLICK GOAL (cell 60)" if goal_uv is None else ("NO PATH" if no_path else f"WPs={len(wp_px)}")
        cv2.putText(vis, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255) if no_path else (0, 255, 255), 2)

        cv2.imshow("warp_map", vis)
        cv2.imshow("free_grid", free)
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolvePidHiwonder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
