#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from cyberrunner_state_estimation.core.estimation_pipeline import EstimationPipeline
from cyberrunner_interfaces.msg import StateEstimateSub


class ImageSubscriber(Node):
    def __init__(self, skip=1):
        super().__init__("cyberrunner_state_estimation_subimg")

        # -------------------------
        # ROS I/O
        # -------------------------
        self.sub_img = self.create_subscription(
            Image,
            "cyberrunner_camera/image",
            self.on_image,
            10,
        )

        # Route follower publishes (row, col) in Point(x=row, y=col)
        self.target_pixel = None
        self.sub_target_pixel = self.create_subscription(
            Point,
            "/target_pixel",
            self.on_target_pixel,
            10,
        )

        # State estimate output
        self.pub_est = self.create_publisher(
            StateEstimateSub,
            "cyberrunner_state_estimation/estimate_subimg",
            10,
        )

        # Converted target for PID (maze coordinates)
        self.pub_ball_target = self.create_publisher(Point, "/ball/target", 10)

        # TF
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish /ball/target at fixed rate so ros2 topic echo always sees messages
        self.target_timer = self.create_timer(0.05, self.publish_ball_target_from_pixel)  # 20 Hz

        # -------------------------
        # Vision / Estimation
        # -------------------------
        self.bridge = CvBridge()
        self.estimation_pipeline = EstimationPipeline(
            fps=55.0,
            estimator="KF",
            print_measurements=True,
            show_image=False,
            do_anim_3d=False,
            viewpoint="top",
            show_subimages_detector=False,
        )

        self.skip = int(skip)
        self.count = 0

        # last computed ball position (for debugging)
        self.last_ball = None

        # warn throttle flags
        self._warned_no_pose = False
        self._warned_convert_fail = False

        self.get_logger().info("cyberrunner_state_estimation_subimg READY")

    # -------------------------
    # Route click target (pixel)
    # -------------------------
    def on_target_pixel(self, msg: Point):
        # route publishes (row, col) as (x, y)
        self.target_pixel = np.array([msg.x, msg.y], dtype=float)
        self.get_logger().info(f"Got /target_pixel row={msg.x:.1f} col={msg.y:.1f}")

    # -------------------------
    # Main image callback
    # -------------------------
    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Skip green-only frames (kept from your earlier file behavior)
        b, g, r = np.mean(np.mean(frame, axis=0), axis=0)
        if g > 100 and b < 40 and r < 40:
            return

        # Run estimator pipeline
        x_hat, P, angles, subimg, xb, yb = self.estimation_pipeline.estimate(
            frame, return_ball_subimg=True
        )
        self.last_ball = (float(xb), float(yb))

        # Publish StateEstimateSub (same pattern as before)
        if self.count % self.skip == 0:
            out = StateEstimateSub()
            out.state.x_b = float(xb)
            out.state.y_b = float(yb)
            out.state.x_b_dot = float(x_hat[2])
            out.state.y_b_dot = float(x_hat[3])
            out.state.alpha = float(-angles[1])
            out.state.beta = float(angles[0])
            out.subimg = self.bridge.cv2_to_imgmsg(subimg)
            self.pub_est.publish(out)

        # Broadcast TFs (safe, won’t crash)
        self.broadcast_tfs()

        self.count += 1

    # -------------------------
    # Publish /ball/target from /target_pixel
    # -------------------------
    def publish_ball_target_from_pixel(self):
        if self.target_pixel is None:
            return

        meas = getattr(self.estimation_pipeline, "measurements", None)
        if meas is None:
            return

        plate_pose = getattr(meas, "plate_pose", None)
        if plate_pose is None or getattr(plate_pose, "T__C_M", None) is None:
            if not self._warned_no_pose:
                self.get_logger().warn("plate_pose / T__C_M not ready yet -> cannot publish /ball/target")
                self._warned_no_pose = True
            return

        try:
            # route provides (row, col); camera math expects (x, y) = (col, row)
            row = float(self.target_pixel[0])
            col = float(self.target_pixel[1])

            px_xy = np.array([[col, row]], dtype=float)  # (x, y)

            if hasattr(plate_pose, "undistort_points"):
                und_xy = plate_pose.undistort_points(px_xy)[0]  # (x, y)
            else:
                und_xy = px_xy[0]

            # backproject to maze/world (depends on your pipeline function)
            target_M = meas.ball_pos_backproject(und_xy, plate_pose.K, plate_pose.T__C_M)
            if target_M is None:
                return

            target_M = np.array(target_M, dtype=float).reshape(-1)
            if target_M.size < 3:
                return
            if np.any(np.isnan(target_M[:3])):
                return

            out = Point()
            out.x = float(target_M[0])
            out.y = float(target_M[1])
            out.z = float(target_M[2])
            self.pub_ball_target.publish(out)

        except Exception as e:
            if not self._warned_convert_fail:
                self.get_logger().warn(f"convert /target_pixel -> /ball/target failed: {e}")
                self._warned_convert_fail = True

    # -------------------------
    # TF broadcast (safe)
    # -------------------------
    def broadcast_tfs(self):
        meas = getattr(self.estimation_pipeline, "measurements", None)
        if meas is None:
            return
        plate_pose = getattr(meas, "plate_pose", None)
        if plate_pose is None:
            return

        try:
            # Static camera->world once
            if self.count == 0 and getattr(plate_pose, "T__W_C", None) is not None:
                t = self.make_tf(plate_pose.T__W_C, "camera", "world")
                self.tf_static_broadcaster.sendTransform(t)

            # Maze->world and Ball->maze
            if getattr(plate_pose, "T__W_M", None) is not None:
                t_maze = self.make_tf(plate_pose.T__W_M, "maze", "world")

                T__B_M = np.eye(4)
                T__B_M[:3, -1] = meas.get_ball_position_in_maze()
                t_ball = self.make_tf(T__B_M, "maze", "ball")

                self.tf_broadcaster.sendTransform([t_maze, t_ball])
        except Exception:
            pass

    def make_tf(self, se3, parent, child):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = float(se3[0, 3])
        t.transform.translation.y = float(se3[1, 3])
        t.transform.translation.z = float(se3[2, 3])

        q = Rotation.from_matrix(se3[:3, :3]).as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        return t


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber(skip=1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
