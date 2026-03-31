"""
path_node  (click-to-add-point mode)
-------------------------------------
Click individual points in the topdown window.
Points connect automatically with lines.
Each point shows its (x,y) coordinate.

Passed waypoints are hidden during active run.
When marble disappears, all waypoints are shown again.

Keys in GUI window:
  left click   → add point
  right click  → remove nearest point
  z            → undo last point
  x            → clear all points
  s            → save and close
  ESC          → close without saving

Parameters
  path_file   str   default ~/cyberrunner_path.json
"""
import os
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np


class PathNode(Node):
    def __init__(self):
        super().__init__("path_node")
        self.declare_parameter("path_file",
                               os.path.expanduser("~/cyberrunner_path.json"))
        self.path_file = self.get_parameter("path_file").value

        self.bridge         = CvBridge()
        self.last_td        = None
        self.waypoints      = []
        self.gui_open       = False
        self.hover_pt       = None
        self.current_wp_idx = -1   # -1 = not running / reset, show all points
        self._path_dirty    = False  # set True whenever waypoints change
        self.WIN            = "PATH  (click=add  rclick=remove  z=undo  x=clear  s=save  ESC=close)"

        # ROS I/O
        self.board_M     = None   # flat→current topdown  (from estimator)
        self.board_M_inv = None   # current topdown→flat

        self.sub_td     = self.create_subscription(
            Image, "/camera/topdown", self._on_topdown, 2)
        self.sub_wp_idx = self.create_subscription(
            Int32MultiArray, "/controller/wp_idx",
            lambda m: setattr(self, "current_wp_idx", m.data[0] if m.data else -1), 2)
        self.sub_board_xfm = self.create_subscription(
            Float32MultiArray, "/estimator/board_transform",
            self._on_board_transform, 2)
        self.pub_wp     = self.create_publisher(
            Float32MultiArray, "/path/waypoints", 2)
        self.pub_active = self.create_publisher(Bool, "/path/active", 2)
        self.create_service(Trigger, "/path/save",  self._svc_save)
        self.create_service(Trigger, "/path/load",  self._svc_load)
        self.create_service(Trigger, "/path/draw",  self._svc_draw)
        self.create_service(Trigger, "/path/clear", self._svc_clear)

        self.timer = self.create_timer(0.033, self._tick)

        # Auto-load
        if os.path.exists(self.path_file):
            self._load_path(self.path_file)
            self.get_logger().info(
                "Auto-loaded path (" + str(len(self.waypoints)) + " pts)")

    # ── Board transform ───────────────────────────────────────────────────────
    def _on_board_transform(self, msg):
        M = np.array(msg.data, dtype=np.float64).reshape(3, 3)
        try:
            if np.linalg.cond(M) > 1e10:
                return
            self.board_M     = M
            self.board_M_inv = np.linalg.inv(M)
        except np.linalg.LinAlgError:
            pass

    def _to_flat(self, x, y):
        """Current topdown → flat (board-relative) space."""
        if self.board_M_inv is None:
            return float(x), float(y)
        ph = self.board_M_inv @ np.array([x, y, 1.0])
        return float(ph[0] / ph[2]), float(ph[1] / ph[2])

    def _to_display(self, x, y):
        """Flat (board-relative) → current topdown space for display."""
        if self.board_M is None:
            return float(x), float(y)
        ph = self.board_M @ np.array([x, y, 1.0])
        return float(ph[0] / ph[2]), float(ph[1] / ph[2])

    # ── Subscribers ───────────────────────────────────────────────────────────
    def _on_topdown(self, msg):
        self.last_td = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # ── Mouse callback ────────────────────────────────────────────────────────
    def _on_mouse(self, event, x, y, flags, param):

        if event == cv2.EVENT_MOUSEMOVE:
            self.hover_pt = (x, y)

        elif event == cv2.EVENT_LBUTTONDOWN:
            fx, fy = self._to_flat(x, y)
            self.waypoints.append([fx, fy])
            self.get_logger().info(
                "Added point " + str(len(self.waypoints)) +
                "  flat=(" + str(round(fx, 1)) + ", " + str(round(fy, 1)) + ")")
            self._path_dirty = True
            self._publish_waypoints()

        elif event == cv2.EVENT_RBUTTONDOWN:
            if not self.waypoints:
                return
            # Hit-test in display (topdown) space so the click feels natural
            disp = [self._to_display(*p) for p in self.waypoints]
            dists   = [math.hypot(x - d[0], y - d[1]) for d in disp]
            nearest = int(np.argmin(dists))
            if dists[nearest] < 30:
                removed = self.waypoints.pop(nearest)
                self.get_logger().info(
                    "Removed point " + str(nearest + 1) +
                    "  (" + str(round(removed[0], 1)) + ", " + str(round(removed[1], 1)) + ")")
                self._path_dirty = True
                self._publish_waypoints()

    # ── GUI tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        if not self.gui_open:
            return
        if self.last_td is None:
            return

        disp = self.last_td.copy()
        self._draw_overlay(disp)
        cv2.imshow(self.WIN, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('z'):
            if self.waypoints:
                removed = self.waypoints.pop()
                self.get_logger().info(
                    "Undo — removed (" +
                    str(removed[0]) + ", " + str(removed[1]) + ")")
                self._path_dirty = True
                self._publish_waypoints()

        elif key == ord('x'):
            self.waypoints.clear()
            self.current_wp_idx = -1
            self.get_logger().info("Path cleared")
            self._path_dirty = True
            self._publish_waypoints()

        elif key == ord('s'):
            self._save_path(self.path_file)
            self._close_gui()
            self.get_logger().info(
                "Saved " + str(len(self.waypoints)) + " points")

        elif key == 27:  # ESC
            self._close_gui()

    def _open_gui(self):
        cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.WIN, self._on_mouse)
        self.gui_open = True
        self.get_logger().info("Path GUI opened — click points to build path")

    def _close_gui(self):
        cv2.destroyWindow(self.WIN)
        self.gui_open = False
        self.hover_pt = None
        self._publish_waypoints()

    # ── Draw overlay ──────────────────────────────────────────────────────────
    def _draw_overlay(self, img):
        n       = len(self.waypoints)
        running = self.current_wp_idx >= 0   # True = active run, hide passed pts
        idx     = self.current_wp_idx if running else 0

        # Convert all waypoints to current topdown (display) coords
        disp = [self._to_display(*pt) for pt in self.waypoints]

        # Draw connecting lines
        if n >= 2:
            for i in range(n - 1):
                # Skip passed segments only during active run
                if running and i < idx - 1:
                    continue
                p1 = (int(disp[i][0]),   int(disp[i][1]))
                p2 = (int(disp[i+1][0]), int(disp[i+1][1]))
                cv2.line(img, p1, p2, (0, 220, 255), 2)

        # Draw preview line from last point to mouse
        if n >= 1 and self.hover_pt is not None:
            last = (int(disp[-1][0]), int(disp[-1][1]))
            cv2.line(img, last, self.hover_pt, (100, 100, 255), 1)

        # Draw each waypoint
        for i, (pt, dp) in enumerate(zip(self.waypoints, disp)):
            # Skip already-passed points only during active run
            if running and i < idx:
                continue

            px, py = int(dp[0]), int(dp[1])

            # Colour logic
            if running and i == idx:
                color = (0, 255, 255)       # cyan = current target
                label = ">" + str(i + 1)
            elif i == 0:
                color = (0, 255, 0)         # green = start
                label = "START"
            elif i == n - 1:
                color = (0, 0, 255)         # red = end
                label = "END"
            else:
                color = (0, 220, 255)       # yellow = intermediate
                label = str(i + 1)

            cv2.circle(img, (px, py), 6, color, -1)
            cv2.circle(img, (px, py), 8, (255, 255, 255), 1)

            coord_text = "(" + str(round(pt[0])) + "," + str(round(pt[1])) + ")"
            cv2.putText(img, label,
                        (px + 10, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(img, coord_text,
                        (px + 10, py + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # HUD
        status = ("RUNNING  wp=" + str(idx) + "/" + str(n - 1)
                  if running else "IDLE")
        cv2.putText(img,
                    "Points: " + str(n) + "  " + status +
                    "  | left=add  right=remove  z=undo  x=clear  s=save",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if self.hover_pt is not None:
            cv2.putText(img,
                        "cursor: (" + str(self.hover_pt[0]) +
                        ", " + str(self.hover_pt[1]) + ")",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (150, 150, 150), 1)

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish_waypoints(self):
        if not self._path_dirty:
            return
        self._path_dirty = False
        msg = Float32MultiArray()
        msg.data = [float(v) for pt in self.waypoints for v in pt]
        self.pub_wp.publish(msg)
        b = Bool()
        b.data = len(self.waypoints) > 1
        self.pub_active.publish(b)

    # ── Persistence ───────────────────────────────────────────────────────────
    def _save_path(self, path):
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"waypoints": self.waypoints}, f, indent=2)
        os.replace(tmp, path)

    def _load_path(self, path):
        with open(path) as f:
            data = json.load(f)
        self.waypoints   = data.get("waypoints", [])
        self._path_dirty = True
        self._publish_waypoints()

    # ── Services ──────────────────────────────────────────────────────────────
    def _svc_save(self, req, res):
        try:
            self._save_path(self.path_file)
            res.success = True
            res.message = "Saved " + str(len(self.waypoints)) + " points"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def _svc_load(self, req, res):
        try:
            self._load_path(self.path_file)
            res.success = True
            res.message = "Loaded " + str(len(self.waypoints)) + " points"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def _svc_draw(self, req, res):
        self._open_gui()
        res.success = True
        res.message = "GUI opened"
        return res

    def _svc_clear(self, req, res):
        self.waypoints.clear()
        self.current_wp_idx = -1
        self._path_dirty = True
        self._publish_waypoints()
        res.success = True
        res.message = "Path cleared"
        return res

    def destroy_node(self):
        if self.gui_open:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()