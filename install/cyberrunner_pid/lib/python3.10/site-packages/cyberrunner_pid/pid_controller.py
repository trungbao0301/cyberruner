#!/usr/bin/env python3
# ============================================================
# FILE 2/2: ball_pid_step_controller.py
# - Subscribes:
#     /cyberrunner_state_estimation/state_estimate  (StateEstimate)
#     /ball/target                                 (geometry_msgs/Point)  from FILE 1
# - Publishes:
#     /hiwonder/cmd                                (Int32MultiArray) -> your HID servo node
# - Features:
#   * PID/PD in meters -> servo counts
#   * step output (25 counts per step)
#   * return-to-center (500) when close/slow/no target
# ============================================================

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point
from cyberrunner_interfaces.msg import StateEstimate


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def quantize_around_center(pos, center, step_size):
    off = pos - center
    off_q = int(round(off / step_size)) * step_size
    return int(center + off_q)


class BallPidStepController(Node):
    def __init__(self):
        super().__init__("ball_pid_step_controller")

        # topics
        self.declare_parameter("state_topic", "/cyberrunner_state_estimation/state_estimate")
        self.declare_parameter("target_topic", "/ball/target")
        self.declare_parameter("servo_cmd_topic", "/hiwonder/cmd")

        # gains (counts per unit)
        self.declare_parameter("Kp", 1200.0)  # counts / m
        self.declare_parameter("Kd", 350.0)   # counts / (m/s)
        self.declare_parameter("Ki", 0.0)     # counts / (m*s)

        # servo centers + limits
        self.declare_parameter("s1_center", 500)
        self.declare_parameter("s2_center", 500)
        self.declare_parameter("s1_min", 350)
        self.declare_parameter("s1_max", 650)
        self.declare_parameter("s2_min", 350)
        self.declare_parameter("s2_max", 650)

        # max output offset (counts)
        self.declare_parameter("u_max", 120.0)

        # step output
        self.declare_parameter("step_size", 25)
        self.declare_parameter("step_max", 25)  # max change per callback

        # return-to-center thresholds
        self.declare_parameter("return_err_m", 0.010)
        self.declare_parameter("return_vel_mps", 0.030)

        # servo move time
        self.declare_parameter("move_time_ms", 40)

        # axis mapping
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("sign_s1", +1.0)
        self.declare_parameter("sign_s2", +1.0)

        # state
        self.target_xy = None  # (x,y) meters
        self.last_t = None
        self.Ix = 0.0
        self.Iy = 0.0
        self.Imax = 0.50

        self.s1_last = int(self.get_parameter("s1_center").value)
        self.s2_last = int(self.get_parameter("s2_center").value)

        # ros io
        self.sub_state = self.create_subscription(
            StateEstimate, self.get_parameter("state_topic").value, self.on_state, 20
        )
        self.sub_target = self.create_subscription(
            Point, self.get_parameter("target_topic").value, self.on_target, 20
        )
        self.pub = self.create_publisher(
            Int32MultiArray, self.get_parameter("servo_cmd_topic").value, 20
        )

        self.get_logger().info("BallPidStepController running.")

    def on_target(self, msg: Point):
        self.target_xy = (float(msg.x), float(msg.y))

    def publish_servo(self, s1, s2):
        out = Int32MultiArray()
        out.data = [int(s1), int(s2), int(self.get_parameter("move_time_ms").value)]
        self.pub.publish(out)

    def on_state(self, msg: StateEstimate):
        x = float(msg.x_b)
        y = float(msg.y_b)
        vx = float(msg.x_b_dot)
        vy = float(msg.y_b_dot)

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(vx) and math.isfinite(vy)):
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

        s1c = int(self.get_parameter("s1_center").value)
        s2c = int(self.get_parameter("s2_center").value)

        # if no target -> go home
        if self.target_xy is None:
            self.Ix = 0.0; self.Iy = 0.0
            self.s1_last = s1c; self.s2_last = s2c
            self.publish_servo(s1c, s2c)
            return

        gx, gy = self.target_xy
        ex = gx - x
        ey = gy - y
        err = math.hypot(ex, ey)
        speed = math.hypot(vx, vy)

        return_err = float(self.get_parameter("return_err_m").value)
        return_vel = float(self.get_parameter("return_vel_mps").value)

        # return-to-center (stability)
        if err < return_err or speed < return_vel:
            s1_cmd = s1c
            s2_cmd = s2c
            self.Ix = 0.0
            self.Iy = 0.0
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

            # quantize steps
            step_size = int(self.get_parameter("step_size").value)
            s1_cmd = quantize_around_center(s1_cmd, s1c, step_size)
            s2_cmd = quantize_around_center(s2_cmd, s2c, step_size)

            # clamp
            s1_cmd = clamp(s1_cmd, int(self.get_parameter("s1_min").value), int(self.get_parameter("s1_max").value))
            s2_cmd = clamp(s2_cmd, int(self.get_parameter("s2_min").value), int(self.get_parameter("s2_max").value))

        # rate limit
        step_max = int(self.get_parameter("step_max").value)
        s1_cmd = clamp(s1_cmd, self.s1_last - step_max, self.s1_last + step_max)
        s2_cmd = clamp(s2_cmd, self.s2_last - step_max, self.s2_last + step_max)
        self.s1_last = s1_cmd
        self.s2_last = s2_cmd

        self.publish_servo(s1_cmd, s2_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallPidStepController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
