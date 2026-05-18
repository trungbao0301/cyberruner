#!/usr/bin/env python3
import os
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

from cyberrunner_state_estimation.core.estimation_pipeline import EstimationPipeline
from cyberrunner_interfaces.msg import StateEstimate


def load_markers_csv_from_share(pkg_name="cyberrunner_state_estimation", fname="markers.csv"):
    shared = get_package_share_directory(pkg_name)
    path = os.path.join(shared, fname)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing markers file: {path}")
    m = np.loadtxt(path, delimiter=",").astype(np.float32)
    if m.shape[0] < 4:
        raise RuntimeError(f"markers.csv must have at least 4 points, got {m.shape}")
    return m, path


def compute_warp_from_outer_markers_auto(markers_xy):
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
    # Assumption: estimator x,y are meters with origin at board center.
    # +x to right, +y up
    u = int(round((x_m / board_w_m) * WW + 0.5 * WW))
    v = int(round(0.5 * HH - (y_m / board_h_m) * HH))
    u = max(0, min(u, WW - 1))
    v = max(0, min(v, HH - 1))
    return u, v


class CyberrunnerStateEstimationNode(Node):
    def __init__(self):
        super().__init__("cyberrunner_state_estimation_node")

        
        self.declare_parameter("image_topic", "/cyberrunner_camera/image")
        self.declare_parameter("out_topic", "/cyberrunner_state_estimation/state_estimate")

        # board meters (for debug mapping only; should match estimator calibration)
        self.declare_parameter("board_w_m", 0.30)
        self.declare_parameter("board_h_m", 0.20)

        # warp debug view size
        self.declare_parameter("warp_w_px", 900)
        self.declare_parameter("warp_h_px", 600)

        # terminal print rate
        self.declare_parameter("print_every_n", 10)

        # debug window
        self.declare_parameter("show_debug", True)

        self.board_w_m = float(self.get_parameter("board_w_m").value)
        self.board_h_m = float(self.get_parameter("board_h_m").value)
        self.WW = int(self.get_parameter("warp_w_px").value)
        self.HH = int(self.get_parameter("warp_h_px").value)
        self.print_every_n = int(self.get_parameter("print_every_n").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)

        # Load markers.csv (your old save location)
        markers, marker_path = load_markers_csv_from_share()
        self.Hwarp, self.WW, self.HH = compute_warp_from_outer_markers_auto(markers)
        self.get_logger().info(f"Auto warp size: {self.WW} x {self.HH}")

        self.get_logger().info(f"Using markers file: {marker_path}")

        self.sub = self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.listener_callback, 10
        )
        self.pub = self.create_publisher(
            StateEstimate, self.get_parameter("out_topic").value, 10
        )

        self.br = CvBridge()

        self.estimation_pipeline = EstimationPipeline(
            fps=55.0,
            estimator="FiniteDiff",
            FiniteDiff_mean_steps=4,
            print_measurements=False,
            show_image=False,
            do_anim_3d=False,
            viewpoint="top",
            show_subimages_detector=False,
        )

        self._count = 0

        if self.show_debug:
            cv2.namedWindow("estimator_debug_warp", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow("estimator_debug_warp", 960, 600)

        self.get_logger().info("Estimator running (NaN-safe, warped debug aligned to markers).")

    def listener_callback(self, data: Image):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")

        out = self.estimation_pipeline.estimate(frame)

        # Supports: (x_hat, P, angles) OR (x_hat, P, angles, ...)
        if not isinstance(out, (tuple, list)) or len(out) < 3:
            return
        x_hat, P, angles = out[0], out[1], out[2]

        if x_hat is None or len(x_hat) < 4:
            return

        x_b = float(x_hat[0])
        y_b = float(x_hat[1])
        vx  = float(x_hat[2])
        vy  = float(x_hat[3])

        # If ball not detected, pipeline may output NaNs. Do NOT crash.
        if not (np.isfinite(x_b) and np.isfinite(y_b) and np.isfinite(vx) and np.isfinite(vy)):
            # still show warped image if you want, but don’t publish bad state
            if self.show_debug:
                warp = cv2.warpPerspective(frame, self.Hwarp, (self.WW, self.HH))
                cv2.putText(warp, "BALL NOT DETECTED", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow("estimator_debug_warp", warp)
                cv2.waitKey(1)
            return

        msg = StateEstimate()
        msg.x_b = x_b
        msg.y_b = y_b
        msg.x_b_dot = vx
        msg.y_b_dot = vy
        msg.alpha = float(-angles[1])
        msg.beta = float(angles[0])

        self.pub.publish(msg)

        # Debug draw on WARP (correct alignment)
        if self.show_debug or (self.print_every_n > 0):
            u, v = m_to_px_on_warp(x_b, y_b, self.WW, self.HH, self.board_w_m, self.board_h_m)

        self._count += 1
        if self.print_every_n > 0 and (self._count % self.print_every_n == 0):
            self.get_logger().info(
                f"x={x_b:+.3f} m  y={y_b:+.3f} m | "
                f"vx={vx:+.3f} vy={vy:+.3f} m/s | warp_px=({u},{v})"
            )

        if self.show_debug:
            warp = cv2.warpPerspective(frame, self.Hwarp, (self.WW, self.HH))
            cv2.circle(warp, (u, v), 8, (0, 255, 255), -1)
            cv2.putText(
                warp, f"x={x_b:+.3f} y={y_b:+.3f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2
            )
            cv2.imshow("estimator_debug_warp", warp)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CyberrunnerStateEstimationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
