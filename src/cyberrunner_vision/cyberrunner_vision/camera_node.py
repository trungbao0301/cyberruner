"""
camera_node  (Linux-compatible)
--------------------------------
Parameters
  camera_index  int     default 0
  device_path   str     default ""   e.g. "/dev/video0"  (overrides camera_index)
  fx            float   default 850.0
  fy            float   default 750.0
  width         int     default 1920
  height        int     default 1200
  fps           int     default 30
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from .ocam_params import OCAM, OUT_W, OUT_H


# ── OCamCalib rectification ───────────────────────────────────────────────────
def _world2cam(dirs, ocam):
    X, Y, Z = dirs[..., 0], dirs[..., 1], dirs[..., 2]
    rho = np.sqrt(X * X + Y * Y) + 1e-12
    t   = np.arctan2(-Z, rho)
    r   = np.zeros_like(t)
    tt  = np.ones_like(t)
    for a in ocam["invpol"]:
        r += a * tt
        tt *= t
    x = X / rho;  y = Y / rho
    u = ocam["cx"] + (x * r) * ocam["c"] + (y * r) * ocam["d"]
    v = ocam["cy"] + (x * r) * ocam["e"] + (y * r)
    return u.astype(np.float32), v.astype(np.float32)


def build_rectify_map(ocam, out_w, out_h, fx, fy):
    uu, vv = np.meshgrid(np.arange(out_w, dtype=np.float64),
                         np.arange(out_h, dtype=np.float64))
    x = (uu - out_w / 2) / fx
    y = (vv - out_h / 2) / fy
    z = np.ones_like(x)
    n = np.sqrt(x * x + y * y + z * z)
    dirs = np.stack([x / n, y / n, z / n], axis=-1)
    return _world2cam(dirs, ocam)


# ── Open camera (Linux-safe) ──────────────────────────────────────────────────
def open_camera(index: int, device_path: str, width: int, height: int, fps: int):
    """
    Try multiple backends in order until one works.
    On Linux: V4L2 first, then ANY.
    device_path overrides index when non-empty.
    """
    source = device_path if device_path else index

    # Build explicit GStreamer pipeline (most reliable for this camera)
    gst = (
        f"v4l2src device={source if isinstance(source, str) else '/dev/video'+str(source)} "
        f"! image/jpeg,width={width},height={height},framerate={fps}/1 "
        f"! jpegdec ! videoconvert ! appsink"
    )
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        for _ in range(3):
            cap.grab()
        ok, _ = cap.read()
        if ok:
            return cap, "gstreamer-pipeline"
        cap.release()

    # Fallback: V4L2 with MJPG
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if width > 0:  cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        if height > 0: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:    cap.set(cv2.CAP_PROP_FPS,          fps)
        for _ in range(3):
            cap.grab()
        ok, _ = cap.read()
        if ok:
            return cap, "v4l2"
        cap.release()

    return None, None


# ── Node ─────────────────────────────────────────────────────────────────────
class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("camera_index",  0)
        self.declare_parameter("device_path",   "")     # e.g. "/dev/video0"
        self.declare_parameter("fx",      850.0)
        self.declare_parameter("fy",      750.0)
        self.declare_parameter("width",   OCAM["width"])
        self.declare_parameter("height",  OCAM["height"])
        self.declare_parameter("fps",     30)
        self.declare_parameter("show_preview", True)    # set False on headless machines

        idx    = self.get_parameter("camera_index").value
        dpath  = self.get_parameter("device_path").value
        fx     = self.get_parameter("fx").value
        fy     = self.get_parameter("fy").value
        width  = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fps    = self.get_parameter("fps").value

        self.map_x, self.map_y = build_rectify_map(OCAM, OUT_W, OUT_H, fx, fy)
        self.bridge = CvBridge()

        self.cap, backend = open_camera(idx, dpath, width, height, fps)

        if self.cap is None:
            self.get_logger().error(
                f"Cannot open camera (index={idx}, device_path='{dpath}'). "
                f"Run:  ls /dev/video*   to find the correct device.")
            raise RuntimeError("Camera open failed")

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        src_str = dpath if dpath else str(idx)
        self.get_logger().info(
            "Camera opened  source=" + src_str +
            "  backend=" + str(backend) +
            "  resolution=" + str(actual_w) + "x" + str(actual_h) +
            "  fx=" + str(fx) + "  fy=" + str(fy))

        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.pub   = self.create_publisher(Image, "/camera/rectified", 2)
        self.timer = self.create_timer(1.0 / fps, self._tick)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Camera read failed", throttle_duration_sec=2.0)
            return
        rect = cv2.remap(frame, self.map_x, self.map_y,
                         cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)
        msg = self.bridge.cv2_to_imgmsg(rect, encoding="bgr8")
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.pub.publish(msg)

        if self.show_preview:
            cv2.imshow("RECTIFIED", rect)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()