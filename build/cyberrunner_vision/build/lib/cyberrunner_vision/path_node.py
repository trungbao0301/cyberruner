"""
path_node
---------
Interactive path drawing + persistence.

Call /path/draw to open GUI.
  d   toggle draw mode (click-drag)
  x   clear path
  s   save & close
  ESC close without saving

Parameters
  path_file   str   default ~/cyberrunner_path.json
"""
import os
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2


class PathNode(Node):
    def __init__(self):
        super().__init__("path_node")
        self.declare_parameter("path_file",
                               os.path.expanduser("~/cyberrunner_path.json"))
        self.path_file = self.get_parameter("path_file").value

        self.bridge     = CvBridge()
        self.last_td    = None
        self.state      = None
        self.waypoints  = []
        self.gui_open   = False
        self.draw_mode  = False
        self.mouse_down = False
        self.raw_pts    = []
        self.WIN        = "PATH_DRAW  (d=draw  s=save  x=clear  ESC=close)"

        self.sub_td    = self.create_subscription(
            Image, "/camera/topdown", self._on_topdown, 2)
        self.sub_state = self.create_subscription(
            Float32MultiArray, "/estimator/state",
            lambda m: setattr(self, "state", list(m.data)), 10)
        self.pub_wp    = self.create_publisher(
            Float32MultiArray, "/path/waypoints", 2)
        self.pub_active = self.create_publisher(Bool, "/path/active", 2)
        self.create_service(Trigger, "/path/save",  self._svc_save)
        self.create_service(Trigger, "/path/load",  self._svc_load)
        self.create_service(Trigger, "/path/draw",  self._svc_draw)
        self.create_service(Trigger, "/path/clear", self._svc_clear)

        self.timer = self.create_timer(0.033, self._tick)

        if os.path.exists(self.path_file):
            self._load_path(self.path_file)
            self.get_logger().info(
                f"Auto-loaded path ({len(self.waypoints)} pts)")

    def _on_topdown(self, msg):
        self.last_td = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _tick(self):
        if not self.gui_open:
            self._publish_waypoints()
            return
        if self.last_td is None:
            return
        disp = self.last_td.copy()
        self._draw_overlay(disp)
        cv2.imshow(self.WIN, disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("d"):
            self.draw_mode  = not self.draw_mode
            self.mouse_down = False
        elif key == ord("x"):
            self.waypoints.clear()
            self.raw_pts.clear()
        elif key == ord("s"):
            self._save_path(self.path_file)
            self._close_gui()
        elif key == 27:
            self._close_gui()

    def _open_gui(self):
        cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.WIN, self._on_mouse)
        self.gui_open = True

    def _close_gui(self):
        cv2.destroyWindow(self.WIN)
        self.gui_open = False
        self._publish_waypoints()

    def _on_mouse(self, event, x, y, flags, param):
        if not self.draw_mode:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_down = True
            self.raw_pts.clear()
            self.raw_pts.append([x, y])
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse_down:
            if self.raw_pts:
                lx, ly = self.raw_pts[-1]
                if math.hypot(x - lx, y - ly) >= 8:
                    self.raw_pts.append([x, y])
            else:
                self.raw_pts.append([x, y])
        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_down = False
            if self.raw_pts:
                self.waypoints = self._downsample(self.raw_pts, min_dist=12)

    def _draw_overlay(self, img):
        pts = self.raw_pts if (self.draw_mode and self.mouse_down)               else self.waypoints
        if len(pts) >= 2:
            for i in range(len(pts) - 1):
                cv2.line(img,
                         (int(pts[i][0]),     int(pts[i][1])),
                         (int(pts[i+1][0]),   int(pts[i+1][1])),
                         (0, 220, 255), 3)
        if self.waypoints:
            cv2.circle(img,
                       (int(self.waypoints[0][0]),  int(self.waypoints[0][1])),
                       10, (0, 255, 0), 2)
            cv2.circle(img,
                       (int(self.waypoints[-1][0]), int(self.waypoints[-1][1])),
                       10, (0, 0, 255), 2)
        mode = "DRAW" if self.draw_mode else "VIEW"
        cv2.putText(img, f"Mode:{mode}  pts:{len(self.waypoints)}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    @staticmethod
    def _downsample(pts, min_dist=12):
        if not pts:
            return []
        out = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) >= min_dist:
                out.append(p)
        return out

    def _publish_waypoints(self):
        if not self.waypoints:
            return
        msg = Float32MultiArray()
        msg.data = [float(v) for pt in self.waypoints for v in pt]
        self.pub_wp.publish(msg)
        b = Bool(); b.data = True
        self.pub_active.publish(b)

    def _save_path(self, path):
        with open(path, "w") as f:
            json.dump({"waypoints": self.waypoints}, f, indent=2)

    def _load_path(self, path):
        with open(path) as f:
            data = json.load(f)
        self.waypoints = data.get("waypoints", [])
        self._publish_waypoints()

    def _svc_save(self, req, res):
        try:
            self._save_path(self.path_file)
            res.success = True
            res.message = f"Saved {len(self.waypoints)} pts"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def _svc_load(self, req, res):
        try:
            self._load_path(self.path_file)
            res.success = True
            res.message = f"Loaded {len(self.waypoints)} pts"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def _svc_draw(self, req, res):
        self._open_gui()
        res.success = True; res.message = "GUI opened"
        return res

    def _svc_clear(self, req, res):
        self.waypoints.clear()
        res.success = True; res.message = "Path cleared"
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
