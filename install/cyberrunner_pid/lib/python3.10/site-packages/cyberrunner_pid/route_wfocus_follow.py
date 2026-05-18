#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class RouteWFocusFollow(Node):
    """
    Click start then end on the image.
    This node publishes the CURRENT waypoint pixel as:
        /target_pixel  geometry_msgs/Point
            x = row (pixel y)
            y = col (pixel x)
    Estimator should convert /target_pixel -> /ball/target.
    PID should track /ball/target -> /hiwonder/cmd.
    """

    def __init__(self):
        super().__init__("route_wfocus_follow")

        # Params
        self.declare_parameter("image_topic", "cyberrunner_camera/image")
        self.declare_parameter("target_pixel_topic", "/target_pixel")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("step_px", 3)

        self.image_topic = self.get_parameter("image_topic").value
        self.target_pixel_topic = self.get_parameter("target_pixel_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.step_px = int(self.get_parameter("step_px").value)

        # ROS I/O
        self.bridge = CvBridge()
        self.last_img = None

        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_img, 10)
        self.pub_target = self.create_publisher(Point, self.target_pixel_topic, 10)

        # route state
        self.clicks = []          # list of (col, row)
        self.waypoints = []       # list of (row, col)
        self.wp_i = 0
        self.need_build = False

        # OpenCV window
        self.window = "Route_wFocus Follow (click start, click end, q quit, r reset)"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.on_mouse)

        # Timer
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_hz), self.tick)

        self.get_logger().info(f"Image: {self.image_topic}")
        self.get_logger().info(f"Publishing /target_pixel: {self.target_pixel_topic}")
        self.get_logger().info("Click start then end to build a route.")

    def on_img(self, msg: Image):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.last_img = None

    def on_mouse(self, event, x_col, y_row, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.last_img is None:
            self.get_logger().warn("No image received yet.")
            return

        # If already had 2 clicks, start over
        if len(self.clicks) >= 2:
            self.clicks = []
            self.waypoints = []
            self.wp_i = 0

        self.clicks.append((int(x_col), int(y_row)))
        self.get_logger().info(f"click: col={x_col} row={y_row}")

        if len(self.clicks) == 2:
            self.need_build = True

    def build_route_simple(self):
        """Simple straight-line route from start to end (robust, no contour needed)."""
        (c0, r0) = self.clicks[0]
        (c1, r1) = self.clicks[1]

        # number of samples based on pixel distance and step_px
        dist = max(1.0, float(np.hypot(c1 - c0, r1 - r0)))
        n = int(dist / max(1, self.step_px)) + 1

        cols = np.linspace(c0, c1, n)
        rows = np.linspace(r0, r1, n)

        self.waypoints = [(int(round(r)), int(round(c))) for r, c in zip(rows, cols)]
        self.wp_i = 0
        self.get_logger().info(f"Built {len(self.waypoints)} waypoints (straight line).")

    def publish_current_waypoint(self):
        if not self.waypoints:
            return
        row, col = self.waypoints[self.wp_i]

        msg = Point()
        # IMPORTANT: (row,col) -> (x,y)
        msg.x = float(row)
        msg.y = float(col)
        msg.z = 0.0
        self.pub_target.publish(msg)

    def draw(self):
        if self.last_img is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera image...",
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow(self.window, blank)
            return

        vis = self.last_img.copy()

        # draw clicks
        for (c, r) in self.clicks:
            cv2.circle(vis, (c, r), 6, (0, 255, 255), -1)

        # draw current waypoint
        if self.waypoints:
            r, c = self.waypoints[self.wp_i]
            cv2.circle(vis, (c, r), 8, (0, 0, 255), -1)

        cv2.imshow(self.window, vis)

    def tick(self):
        # build route after second click
        if self.need_build:
            self.need_build = False
            self.build_route_simple()

        # publish the current waypoint continuously (so estimator always sees it)
        self.publish_current_waypoint()

        # show GUI
        self.draw()
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt
        if key == ord("r"):
            self.clicks = []
            self.waypoints = []
            self.wp_i = 0
            self.get_logger().info("Route reset. Click start/end again.")


def main(args=None):
    rclpy.init(args=args)
    node = RouteWFocusFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
