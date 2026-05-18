#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from cyberrunner_interfaces.msg import StateEstimateSub  # adjust if needed


class PathFollowerAvoidHoles(Node):
    def __init__(self):
        super().__init__("path_follower_node")

        self.declare_parameter("path_file",  "/home/trungbao/cyberrunner_maps/path_xy.npy")
        self.declare_parameter("holes_file", "/home/trungbao/cyberrunner_maps/holes_xy.npy")

        self.declare_parameter("lookahead_m", 0.03)

        # Real hole diameter = 1.5 cm => radius = 0.0075 m (fixed)
        self.declare_parameter("hole_radius_m", 0.0075)

        # Set these based on your marble
        self.declare_parameter("ball_radius_m", 0.0070)     # ~14mm marble diameter
        self.declare_parameter("safety_margin_m", 0.0100)   # extra safety for dynamics

        self.declare_parameter("estimate_topic", "cyberrunner_state_estimation/estimate_subimg")
        self.declare_parameter("target_topic", "/ball/target")

        self.path = np.load(self.get_parameter("path_file").value).astype(np.float32)
        self.holes_xy = np.load(self.get_parameter("holes_file").value).astype(np.float32)

        self.lookahead = float(self.get_parameter("lookahead_m").value)

        self.hole_r = float(self.get_parameter("hole_radius_m").value)
        self.ball_r = float(self.get_parameter("ball_radius_m").value)
        self.margin = float(self.get_parameter("safety_margin_m").value)

        self.unsafe_r = self.hole_r + self.ball_r + self.margin

        diffs = np.diff(self.path, axis=0)
        ds = np.linalg.norm(diffs, axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(ds)])

        self.pub = self.create_publisher(Point, self.get_parameter("target_topic").value, 10)
        self.sub = self.create_subscription(
            StateEstimateSub,
            self.get_parameter("estimate_topic").value,
            self.cb,
            10
        )

        self.get_logger().info(f"Loaded path N={len(self.path)}")
        self.get_logger().info(f"Loaded holes M={len(self.holes_xy)}")
        self.get_logger().info(f"Unsafe radius around hole center: {self.unsafe_r:.4f} m")

    def safe_point(self, pt):
        if self.holes_xy.shape[0] == 0:
            return True
        d = np.linalg.norm(self.holes_xy - pt[None, :], axis=1)
        return np.all(d > self.unsafe_r)

    def cb(self, msg: StateEstimateSub):
        xb = float(msg.x)
        yb = float(msg.y)
        p = np.array([xb, yb], dtype=np.float32)

        # nearest point on path
        d2 = np.sum((self.path - p) ** 2, axis=1)
        i = int(np.argmin(d2))

        # initial target index from arc-length lookahead
        s_target = min(self.s[i] + self.lookahead, self.s[-1])
        j = int(np.searchsorted(self.s, s_target))
        j = max(0, min(j, len(self.path) - 1))

        # scan forward until safe
        for _ in range(600):
            pt = self.path[j]
            if self.safe_point(pt):
                break
            if j >= len(self.path) - 1:
                break
            j += 1

        xt, yt = self.path[j]
        out = Point()
        out.x = float(xt)
        out.y = float(yt)
        out.z = 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PathFollowerAvoidHoles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
