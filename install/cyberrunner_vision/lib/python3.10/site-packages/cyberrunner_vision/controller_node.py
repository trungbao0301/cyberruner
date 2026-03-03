"""
controller_node  (fixed PID + calibration mode)
------------------------------------------------
Servo mapping (Hiwonder 0-1000, centre=500):
  servo 1 → X-axis tilt  (right = >500, left = <500)
  servo 2 → Y-axis tilt  (down  = >500, up   = <500)

PID error is in PIXELS (topdown space, 0-1000px).
Output is directly in servo UNITS (not degrees).
This avoids the deg_per_unit confusion entirely.

New parameters:
  kp_x, kp_y    float   independent gains per axis
  ki_x, ki_y    float
  kd_x, kd_y    float
  max_output     int     max servo units from centre (default 150)
  servo_center   int     500
  lookahead_px   float   60
  arrival_px     float   45
  cmd_time_ms    int     80
  ff_gain        float   0.0   (feed-forward from tilt estimator)
  invert_x       bool    false  (flip if board tilts wrong direction)
  invert_y       bool    false

New service:
  /controller/calibrate   Trigger
    → sends centre position and prints current marble pos for manual check

Tuning guide printed at startup.
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


# ── PID (error in pixels, output in servo units) ──────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, lo, hi):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.lo = lo; self.hi = hi
        self._i = 0.0; self._prev = 0.0

    def reset(self):
        self._i = 0.0; self._prev = 0.0

    def update(self, err, dt):
        dt = max(dt, 1e-4)
        # Anti-windup: clamp integral
        i_max = self.hi / max(abs(self.ki), 1e-9)
        self._i    = float(np.clip(self._i + err * dt, -i_max, i_max))
        d          = (err - self._prev) / dt
        self._prev = err
        return float(np.clip(
            self.kp * err + self.ki * self._i + self.kd * d,
            self.lo, self.hi))


# ── Lookahead path follower ───────────────────────────────────────────────────
class PathFollower:
    def __init__(self, waypoints_flat, lookahead_px, arrival_px):
        self.path      = np.array(waypoints_flat,
                                  dtype=np.float32).reshape(-1, 2)
        self.idx       = 0
        self.lookahead = lookahead_px
        self.arrival   = arrival_px
        self.done      = False

    def update(self, marble_xy):
        mx, my = marble_xy

        # Advance while marble close enough to current waypoint
        while self.idx < len(self.path) - 1:
            tx, ty = self.path[self.idx]
            if math.hypot(mx - tx, my - ty) < self.arrival:
                self.idx += 1
            else:
                break

        if self.idx >= len(self.path) - 1:
            self.done = True

        # Scan forward for lookahead target
        look_idx = self.idx
        acc = 0.0
        while look_idx < len(self.path) - 1:
            acc += float(np.linalg.norm(
                self.path[look_idx + 1] - self.path[look_idx]))
            look_idx += 1
            if acc >= self.lookahead:
                break

        return self.path[look_idx], self.idx, self.done


# ── Node ──────────────────────────────────────────────────────────────────────
class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Per-axis PID gains (pixel error → servo units)
        # Start small — a 100px error with kp=0.5 gives 50 servo units
        self.declare_parameter("kp_x",         0.5)
        self.declare_parameter("kp_y",         0.5)
        self.declare_parameter("ki_x",         0.001)
        self.declare_parameter("ki_y",         0.001)
        self.declare_parameter("kd_x",         0.08)
        self.declare_parameter("kd_y",         0.08)
        self.declare_parameter("max_output",   150)    # servo units from centre
        self.declare_parameter("servo_center", 500)
        self.declare_parameter("lookahead_px", 60.0)
        self.declare_parameter("arrival_px",   45.0)
        self.declare_parameter("cmd_time_ms",  20)
        self.declare_parameter("ff_gain",      0.0)
        self.declare_parameter("invert_x",     False)
        self.declare_parameter("invert_y",     True)

        self._load_params()

        self.marble_pos = None
        self.waypoints  = None
        self.est_state  = None
        self.follower   = None
        self.active     = False
        self.t_last     = time.time()

        self.sub_marble = self.create_subscription(
            Point, "/marble/position", self._on_marble, 2)
        self.sub_wp     = self.create_subscription(
            Float32MultiArray, "/path/waypoints", self._on_waypoints, 2)
        self.sub_state  = self.create_subscription(
            Float32MultiArray, "/estimator/state",
            lambda m: setattr(self, "est_state", list(m.data)), 10)
        self.pub_cmd    = self.create_publisher(
            Int32MultiArray, "/hiwonder/cmd", 1)  # depth=1, no queuing
        self.create_service(Trigger, "/controller/start",     self._svc_start)
        self.create_service(Trigger, "/controller/stop",      self._svc_stop)
        self.create_service(Trigger, "/controller/calibrate", self._svc_calibrate)

        self.timer = self.create_timer(0.033, self._tick)

        self.get_logger().info(
            "\n=== ControllerNode ready ===\n"
            "  /controller/start      → start following path\n"
            "  /controller/stop       → stop + level board\n"
            "  /controller/calibrate  → send centre, check marble pos\n"
            "\nTuning (ros2 param set /controller_node <param> <value>):\n"
            "  kp_x / kp_y    start at 0.5  (raise if too slow)\n"
            "  kd_x / kd_y    start at 0.08 (raise if oscillating)\n"
            "  max_output     default 150   (servo units from centre 500)\n"
            "  invert_x/y     true/false    (if board tilts wrong way)\n"
        )

    def _load_params(self):
        mx = self.get_parameter("max_output").value
        self.servo_center = self.get_parameter("servo_center").value
        self.lookahead_px = self.get_parameter("lookahead_px").value
        self.arrival_px   = self.get_parameter("arrival_px").value
        self.cmd_time_ms  = self.get_parameter("cmd_time_ms").value
        self.ff_gain      = self.get_parameter("ff_gain").value
        self.invert_x     = self.get_parameter("invert_x").value
        self.invert_y     = self.get_parameter("invert_y").value

        self.pid_x = PID(
            self.get_parameter("kp_x").value,
            self.get_parameter("ki_x").value,
            self.get_parameter("kd_x").value,
            -mx, mx)
        self.pid_y = PID(
            self.get_parameter("kp_y").value,
            self.get_parameter("ki_y").value,
            self.get_parameter("kd_y").value,
            -mx, mx)

    def _on_marble(self, msg):
        was_lost = self.marble_pos is None
        self.marble_pos = None if msg.z < 0 else (msg.x, msg.y)
        # Reset PID when marble reappears to avoid windup from lost period
        if was_lost and self.marble_pos is not None:
            self.pid_x.reset()
            self.pid_y.reset()
            self.get_logger().info("Marble reacquired — PID reset")

    def _on_waypoints(self, msg):
        self.waypoints = list(msg.data)
        if self.waypoints and len(self.waypoints) >= 4:
            self.follower = PathFollower(
                self.waypoints, self.lookahead_px, self.arrival_px)
            self.get_logger().info(
                "Path loaded: " +
                str(len(self.waypoints) // 2) + " waypoints")

    def _send_servo(self, out_x, out_y, time_ms=None):
        """Convert PID output to servo positions and publish."""
        if time_ms is None:
            time_ms = self.cmd_time_ms

        sx = -1 if self.invert_x else 1
        sy = -1 if self.invert_y else 1

        pos1 = int(np.clip(self.servo_center + sx * out_x,
                           0, 1000))
        pos2 = int(np.clip(self.servo_center + sy * out_y,
                           0, 1000))

        cmd = Int32MultiArray()
        cmd.data = [pos1, pos2, time_ms]
        self.pub_cmd.publish(cmd)
        return pos1, pos2

    def _tick(self):
        # Reload params every tick so ros2 param set works live
        self._load_params()

        now = time.time()
        dt  = now - self.t_last
        self.t_last = now

        if not self.active or self.follower is None:
            return  # send nothing when inactive — don't flood topic

        # Marble lost → level board and wait
        if self.marble_pos is None:
            self._stop_and_level()
            self.get_logger().warn(
                "Marble lost — levelling board",
                throttle_duration_sec=1.0)
            return

        target, wp_idx, done = self.follower.update(self.marble_pos)

        if done:
            self.get_logger().info("Goal reached! Stopping controller.")
            self._stop_and_level()
            self.active = False
            return

        # Error in topdown pixels
        err_x = float(target[0] - self.marble_pos[0])
        err_y = float(target[1] - self.marble_pos[1])

        out_x = self.pid_x.update(err_x, dt)
        out_y = self.pid_y.update(err_y, dt)

        # Feed-forward from board tilt estimate
        if self.est_state and self.ff_gain > 0 and self.est_state[5] > 0.5:
            # tilt_x, tilt_y in degrees → scale to servo units
            out_x += self.ff_gain * self.est_state[3] * 10.0
            out_y += self.ff_gain * self.est_state[4] * 10.0

        pos1, pos2 = self._send_servo(out_x, out_y)

        self.get_logger().info(
            "wp=" + str(wp_idx) + "/" + str(len(self.follower.path)-1) +
            "  marble=(" + str(round(self.marble_pos[0])) +
            "," + str(round(self.marble_pos[1])) + ")" +
            "  target=(" + str(round(float(target[0]))) +
            "," + str(round(float(target[1]))) + ")" +
            "  err=(" + str(round(err_x)) + "," + str(round(err_y)) + ")" +
            "  out=(" + str(round(out_x)) + "," + str(round(out_y)) + ")" +
            "  servo=(" + str(pos1) + "," + str(pos2) + ")",
            throttle_duration_sec=0.2)

    def _stop_and_level(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self._send_servo(0, 0, time_ms=600)

    def _svc_start(self, req, res):
        if self.follower is None and self.waypoints:
            self.follower = PathFollower(
                self.waypoints, self.lookahead_px, self.arrival_px)
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
        self.follower.idx  = 0
        self.follower.done = False
        self.active  = True
        self.t_last  = time.time()
        res.success  = True
        res.message  = "Controller started"
        self.get_logger().info("Controller STARTED")
        return res

    def _svc_stop(self, req, res):
        self.active = False
        self._stop_and_level()
        res.success = True
        res.message = "Stopped, board levelled"
        self.get_logger().info("Controller STOPPED")
        return res

    def _svc_calibrate(self, req, res):
        """
        Send centre position and log marble location.
        Use this to verify:
          1. Servos move to centre (board should be flat)
          2. Marble position is correct in topdown space
        """
        self._stop_and_level()
        if self.marble_pos:
            res.message = (
                "Board levelled. Marble at (" +
                str(round(self.marble_pos[0])) + "," +
                str(round(self.marble_pos[1])) + ")" +
                " in topdown space (0-1000)")
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