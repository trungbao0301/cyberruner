"""
controller_node  (segment-following path + constant speed-limit braking)
------------------------------------------------------------------------
Servo mapping (Hiwonder 0-1000, centre=500):
  servo 1 → X-axis tilt  (right = >500, left = <500)
  servo 2 → Y-axis tilt  (down  = >500, up   = <500)

Control idea:
- Follow path segments instead of directly chasing only the next waypoint
- Target = closest point on current segment + small lookahead forward
- Braking is separate for X and Y:
    if marble speed on an axis exceeds a threshold,
    apply a fixed opposite brake tilt on that axis

PID error is in PIXELS (topdown space, 0-1000px).
Output is directly in servo UNITS (not degrees).

Parameters:
  kp_x, kp_y          float
  ki_x, ki_y          float
  kd_x, kd_y          float
  max_output          int
  servo_center        int
  arrival_px          float   waypoint-switch radius
  lookahead_px        float   forward target distance along segment
  cmd_time_ms         int
  ff_gain             float
  invert_x            bool
  invert_y            bool

  max_speed_px_x      float   speed threshold for X braking
  max_speed_px_y      float   speed threshold for Y braking
  brake_output_x      float   fixed brake output on X
  brake_output_y      float   fixed brake output on Y

Services:
  /controller/start      Trigger
  /controller/stop       Trigger
  /controller/calibrate  Trigger
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


# ── PID ───────────────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, lo, hi):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lo = lo
        self.hi = hi
        self._i = 0.0
        self._prev = 0.0

    def reset(self):
        self._i = 0.0
        self._prev = 0.0

    def set_gains_and_limits(self, kp, ki, kd, lo, hi):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lo = lo
        self.hi = hi

        i_max = self.hi / max(abs(self.ki), 1e-9)
        self._i = float(np.clip(self._i, -i_max, i_max))

    def update(self, err, dt):
        dt = max(dt, 1e-3)

        i_max = self.hi / max(abs(self.ki), 1e-9)
        self._i = float(np.clip(self._i + err * dt, -i_max, i_max))

        d = (err - self._prev) / dt
        self._prev = err

        out = self.kp * err + self.ki * self._i + self.kd * d
        return float(np.clip(out, self.lo, self.hi))


# ── Segment-following path follower ──────────────────────────────────────────
class PathFollower:
    def __init__(self, waypoints_flat, arrival_px, lookahead_px=25.0):
        self.path = np.array(waypoints_flat, dtype=np.float32).reshape(-1, 2)
        self.arrival = float(arrival_px)
        self.lookahead = float(lookahead_px)
        self.idx = 0              # current segment start index
        self.done = False

    def reset(self):
        self.idx = 0
        self.done = False

    @staticmethod
    def _project_to_segment(p, a, b):
        """
        Project point p onto segment a->b.
        Returns:
          proj      : closest point on segment
          t_clamped : scalar in [0,1]
          seg_len   : length of segment
        """
        ab = b - a
        ap = p - a
        ab2 = float(np.dot(ab, ab))

        if ab2 < 1e-9:
            return a.copy(), 0.0, 0.0

        t = float(np.dot(ap, ab) / ab2)
        t_clamped = max(0.0, min(1.0, t))
        proj = a + t_clamped * ab
        seg_len = float(np.linalg.norm(ab))
        return proj, t_clamped, seg_len

    def update(self, marble_xy):
        p = np.array(marble_xy, dtype=np.float32)

        if len(self.path) == 0:
            self.done = True
            return np.array([0.0, 0.0], dtype=np.float32), -1, True

        if len(self.path) == 1:
            target = self.path[0]
            if np.linalg.norm(p - target) < self.arrival:
                self.done = True
            return target, 0, self.done

        # Keep idx valid
        self.idx = int(np.clip(self.idx, 0, len(self.path) - 2))

        # Advance to next segment if marble is close to next waypoint
        while self.idx < len(self.path) - 2:
            next_wp = self.path[self.idx + 1]
            if np.linalg.norm(p - next_wp) < self.arrival:
                self.idx += 1
            else:
                break

        a = self.path[self.idx]
        b = self.path[self.idx + 1]

        proj, t, seg_len = self._project_to_segment(p, a, b)

        # Look a bit forward along the segment
        if seg_len > 1e-9:
            t_look = min(1.0, t + self.lookahead / seg_len)
            target = a + t_look * (b - a)
        else:
            target = b.copy()

        final_wp = self.path[-1]
        if np.linalg.norm(p - final_wp) < self.arrival:
            self.done = True
            target = final_wp.copy()

        return target, self.idx, self.done


# ── Controller Node ───────────────────────────────────────────────────────────
class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Parameters
        self.declare_parameter("kp_x", 0.3)
        self.declare_parameter("kp_y", 0.3)
        self.declare_parameter("ki_x", 0.001)
        self.declare_parameter("ki_y", 0.001)
        self.declare_parameter("kd_x", 0.08)
        self.declare_parameter("kd_y", 0.08)
        self.declare_parameter("max_output", 80)
        self.declare_parameter("servo_center", 500)
        self.declare_parameter("arrival_px", 35.0)
        self.declare_parameter("lookahead_px", 25.0)
        self.declare_parameter("cmd_time_ms", 20)
        self.declare_parameter("ff_gain", 0.0)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)

        self.declare_parameter("max_speed_px_x", 100.0)
        self.declare_parameter("max_speed_px_y", 100.0)
        self.declare_parameter("brake_output_x", 20.0)
        self.declare_parameter("brake_output_y", 20.0)

        # State
        self.marble_pos = None
        self.waypoints = None
        self.est_state = None
        self.follower = None
        self.active = False

        self.marble_lost_frames = 0
        self.marble_lost_threshold = 30

        self.t_last = time.time()
        self.prev_marble_pos = None

        # PID objects created once
        self.pid_x = PID(0.0, 0.0, 0.0, -80.0, 80.0)
        self.pid_y = PID(0.0, 0.0, 0.0, -80.0, 80.0)

        self._load_params()

        # ROS
        self.sub_marble = self.create_subscription(
            Point, "/marble/position", self._on_marble, 2
        )
        self.sub_wp = self.create_subscription(
            Float32MultiArray, "/path/waypoints", self._on_waypoints, 2
        )
        self.sub_state = self.create_subscription(
            Float32MultiArray,
            "/estimator/state",
            lambda m: setattr(self, "est_state", list(m.data)),
            10,
        )

        self.pub_cmd = self.create_publisher(Int32MultiArray, "/hiwonder/cmd", 1)
        self.pub_wp_idx = self.create_publisher(Int32MultiArray, "/controller/wp_idx", 1)

        self.create_service(Trigger, "/controller/start", self._svc_start)
        self.create_service(Trigger, "/controller/stop", self._svc_stop)
        self.create_service(Trigger, "/controller/calibrate", self._svc_calibrate)

        self.timer = self.create_timer(0.033, self._tick)

        self.get_logger().info(
            "\n=== ControllerNode ready ===\n"
            "  /controller/start      → start following path\n"
            "  /controller/stop       → stop + level board\n"
            "  /controller/calibrate  → send centre, check marble pos\n"
        )

    def _load_params(self):
        mx = float(self.get_parameter("max_output").value)

        self.servo_center = int(self.get_parameter("servo_center").value)
        self.arrival_px = float(self.get_parameter("arrival_px").value)
        self.lookahead_px = float(self.get_parameter("lookahead_px").value)
        self.cmd_time_ms = int(self.get_parameter("cmd_time_ms").value)
        self.ff_gain = float(self.get_parameter("ff_gain").value)
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.max_speed_px_x = float(self.get_parameter("max_speed_px_x").value)
        self.max_speed_px_y = float(self.get_parameter("max_speed_px_y").value)
        self.brake_output_x = float(self.get_parameter("brake_output_x").value)
        self.brake_output_y = float(self.get_parameter("brake_output_y").value)

        self.pid_x.set_gains_and_limits(
            float(self.get_parameter("kp_x").value),
            float(self.get_parameter("ki_x").value),
            float(self.get_parameter("kd_x").value),
            -mx,
            mx,
        )
        self.pid_y.set_gains_and_limits(
            float(self.get_parameter("kp_y").value),
            float(self.get_parameter("ki_y").value),
            float(self.get_parameter("kd_y").value),
            -mx,
            mx,
        )

        if self.follower is not None:
            self.follower.arrival = self.arrival_px
            self.follower.lookahead = self.lookahead_px

    def _publish_wp_idx(self, idx):
        msg = Int32MultiArray()
        msg.data = [int(idx)]
        self.pub_wp_idx.publish(msg)

    def _reset_motion_history(self):
        self.prev_marble_pos = None

    def _reset_all(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self._reset_motion_history()

        if self.follower is not None:
            self.follower.reset()

        self._publish_wp_idx(-1)
        self.get_logger().info("Marble lost — reset controller state")

    def _on_marble(self, msg):
        was_none = self.marble_pos is None
        new_pos = None if msg.z < 0 else (float(msg.x), float(msg.y))

        if new_pos is None:
            self.marble_lost_frames += 1
            if self.marble_lost_frames >= self.marble_lost_threshold and not was_none:
                self.marble_pos = None
                self.active = False
                self._stop_and_level()
                self._reset_all()
        else:
            if was_none and self.marble_lost_frames >= self.marble_lost_threshold:
                self.get_logger().info("Marble reacquired")
            self.marble_lost_frames = 0
            self.marble_pos = new_pos

    def _on_waypoints(self, msg):
        new_waypoints = list(msg.data)

        if not new_waypoints or len(new_waypoints) < 4:
            return

        if self.waypoints == new_waypoints:
            return

        if self.active:
            self.get_logger().warn("Received new path while active; ignoring update")
            return

        self.waypoints = new_waypoints
        self.follower = PathFollower(
            self.waypoints,
            self.arrival_px,
            self.lookahead_px
        )
        self.get_logger().info(
            f"Loaded new path with {len(self.follower.path)} waypoints"
        )

    def _send_servo(self, out_x, out_y, time_ms=None):
        if time_ms is None:
            time_ms = self.cmd_time_ms

        sx = -1 if self.invert_x else 1
        sy = -1 if self.invert_y else 1

        pos1 = int(np.clip(self.servo_center + sx * out_x, 0, 1000))
        pos2 = int(np.clip(self.servo_center + sy * out_y, 0, 1000))

        cmd = Int32MultiArray()
        cmd.data = [pos1, pos2, int(time_ms)]
        self.pub_cmd.publish(cmd)
        return pos1, pos2

    def _tick(self):
        self._load_params()

        now = time.time()
        dt = max(now - self.t_last, 1e-3)
        self.t_last = now

        if not self.active or self.follower is None:
            self._publish_wp_idx(-1)
            return

        if self.marble_pos is None:
            self.get_logger().warn(
                "Waiting for marble...",
                throttle_duration_sec=1.0
            )
            return

        target, seg_idx, done = self.follower.update(self.marble_pos)
        self._publish_wp_idx(seg_idx)

        if done:
            self.get_logger().info("Goal reached! Stopping controller.")
            self._stop_and_level()
            self._publish_wp_idx(-1)
            self.active = False
            return

        err_x = float(target[0] - self.marble_pos[0])
        err_y = float(target[1] - self.marble_pos[1])

        out_x = self.pid_x.update(err_x, dt)
        out_y = self.pid_y.update(err_y, dt)

        # Constant speed-limit braking
        brake_x = 0.0
        brake_y = 0.0
        vel_x = 0.0
        vel_y = 0.0

        if self.prev_marble_pos is not None:
            vel_x = (self.marble_pos[0] - self.prev_marble_pos[0]) / dt
            vel_y = (self.marble_pos[1] - self.prev_marble_pos[1]) / dt

            # Oppose marble velocity:
            # vel > 0  -> brake output negative
            # vel < 0  -> brake output positive
            if abs(vel_x) > self.max_speed_px_x:
                brake_x = -math.copysign(self.brake_output_x, vel_x)
                self.get_logger().warn(
                    f"HIGH SPEED X: {round(vel_x,1)} px/s  braking: {round(brake_x,1)}",
                    throttle_duration_sec=0.2,
                )

            if abs(vel_y) > self.max_speed_px_y:
                brake_y = math.copysign(self.brake_output_y, vel_y)
                self.get_logger().warn(
                    f"HIGH SPEED Y: {round(vel_y,1)} px/s  braking: {round(brake_y,1)}",
                    throttle_duration_sec=0.2,
                )

        self.prev_marble_pos = self.marble_pos

        mx = float(self.get_parameter("max_output").value)
        out_x = float(np.clip(out_x + brake_x, -mx, mx))
        out_y = float(np.clip(out_y + brake_y, -mx, mx))

        # Feed-forward
        if self.est_state and len(self.est_state) > 5 and self.ff_gain > 0 and self.est_state[5] > 0.5:
            out_x += self.ff_gain * self.est_state[3] * 10.0
            out_y += self.ff_gain * self.est_state[4] * 10.0

        out_x = float(np.clip(out_x, -mx, mx))
        out_y = float(np.clip(out_y, -mx, mx))

        self._send_servo(out_x, out_y)

        self.get_logger().info(
            "seg=" + str(seg_idx) + "/" + str(len(self.follower.path) - 2)
            + "  target=(" + str(round(float(target[0]))) + "," + str(round(float(target[1]))) + ")"
            + "  err=(" + str(round(err_x, 1)) + "," + str(round(err_y, 1)) + ")"
            + "  vel=(" + str(round(vel_x, 1)) + "," + str(round(vel_y, 1)) + ")"
            + "  brake=(" + str(round(brake_x, 1)) + "," + str(round(brake_y, 1)) + ")"
            + "  out=(" + str(round(out_x, 1)) + "," + str(round(out_y, 1)) + ")",
            throttle_duration_sec=0.2,
        )

    def _stop_and_level(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self._reset_motion_history()
        self._send_servo(0, 0, time_ms=60)

    def _svc_start(self, req, res):
        if self.follower is None and self.waypoints:
            self.follower = PathFollower(
                self.waypoints,
                self.arrival_px,
                self.lookahead_px
            )

        if self.follower is None:
            res.success = False
            res.message = "No path loaded — call /path/draw first"
            return res

        if self.marble_pos is None:
            res.success = False
            res.message = "Marble not detected — check /marble/position"
            return res

        self.pid_x.reset()
        self.pid_y.reset()
        self._reset_motion_history()
        self.follower.reset()
        self.active = True
        self.t_last = time.time()

        res.success = True
        res.message = "Controller started"
        self.get_logger().info("Controller STARTED")
        return res

    def _svc_stop(self, req, res):
        self.active = False
        self._stop_and_level()
        self._publish_wp_idx(-1)

        res.success = True
        res.message = "Stopped, board levelled"
        self.get_logger().info("Controller STOPPED")
        return res

    def _svc_calibrate(self, req, res):
        self.active = False
        self._stop_and_level()

        if self.marble_pos is not None:
            res.message = (
                "Board levelled. Marble at ("
                + str(round(self.marble_pos[0])) + ","
                + str(round(self.marble_pos[1]))
                + ") in topdown space (0-1000)"
            )
        else:
            res.message = "Board levelled. Marble NOT detected."

        res.success = True
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()