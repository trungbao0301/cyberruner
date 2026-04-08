"""
controller_node (waypoint move-and-settle PD + Kalman filter)
-------------------------------------------------------------
Servo mapping (Hiwonder 0-1000, centre=500):
  servo 1 -> X-axis tilt  (right = >500, left = <500)
  servo 2 -> Y-axis tilt  (down  = >500, up   = <500)

Control idea:
- Estimate marble position with a Kalman filter.
- Drive the marble from waypoint to waypoint with PD control.
- Use derivative-on-measurement from an EMA-filtered velocity estimate.
- When the marble stays close enough to the target and slow enough for long
  enough, advance to the next waypoint.
- Keep marble and path aligned in flat-board coordinates using the board
  transform from estimator_node.

Parameters:
  kp_x, kp_y                  float
  kd_x, kd_y                  float
  max_output                  int
  servo_center                int
  arrival_px                  float   capture radius to enter SETTLE state
  balance_window_px           float   tighter radius that must be held
  waypoint_balance_speed_px   float   max speed allowed while balancing
  balance_hold_s              float   required hold time inside balance window
  derivative_alpha            float   EMA smoothing for derivative-on-measurement
  cmd_time_ms                 int
  invert_x                    bool
  invert_y                    bool
  kalman_q_pos                float
  kalman_q_vel                float
  kalman_r_meas               float
  hold_kp_scale               float   Kp multiplier while in SETTLE
  hold_kd_scale               float   Kd multiplier while in SETTLE
  auto_start                  bool
  settle_speed_px             float
  settle_frames               int
  settle_timeout_s            float
  recovery_wait_s             float
  tilt_balance_enabled        bool
  tilt_balance_kp             float
  tilt_balance_ki             float
  tilt_balance_deadband       float
  tilt_balance_max_trim       float

Services:
  /controller/start           Trigger
  /controller/stop            Trigger
  /controller/calibrate       Trigger
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger


class PD:
    def __init__(self, kp, kd, lo, hi):
        self.kp = kp
        self.kd = kd
        self.lo = lo
        self.hi = hi

    def reset(self):
        pass

    def set_gains_and_limits(self, kp, kd, lo, hi):
        self.kp = float(kp)
        self.kd = float(kd)
        self.lo = float(lo)
        self.hi = float(hi)

    def update(self, err, vel, kp_scale=1.0, kd_scale=1.0):
        out = (self.kp * kp_scale) * float(err) - (self.kd * kd_scale) * float(vel)
        return float(np.clip(out, self.lo, self.hi))


class MeasurementDerivative2D:
    """Derivative-on-measurement with EMA smoothing.

    alpha=1.0 means no smoothing. Lower alpha increases smoothing.
    """

    def __init__(self, alpha=0.85):
        self.alpha = float(alpha)
        self.reset()

    def reset(self):
        self.prev_pos = None
        self.vx = 0.0
        self.vy = 0.0

    def update(self, x, y, dt):
        x = float(x)
        y = float(y)
        dt = max(float(dt), 1e-3)
        alpha = float(np.clip(self.alpha, 0.0, 1.0))

        if self.prev_pos is None:
            self.prev_pos = (x, y)
            self.vx = 0.0
            self.vy = 0.0
            return self.vx, self.vy, 0.0

        px, py = self.prev_pos
        vx_raw = (x - px) / dt
        vy_raw = (y - py) / dt

        self.vx = alpha * vx_raw + (1.0 - alpha) * self.vx
        self.vy = alpha * vy_raw + (1.0 - alpha) * self.vy
        self.prev_pos = (x, y)

        speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        return self.vx, self.vy, speed


class KalmanFilter2D:
    """
    State:       [x, y, vx, vy]
    Measurement: [x, y]
    Model:       constant velocity
    """

    def __init__(self, q_pos=1.0, q_vel=50.0, r_meas=10.0):
        self.x = np.zeros(4)
        self.P = np.eye(4) * 500.0
        self.q_pos = float(q_pos)
        self.q_vel = float(q_vel)
        self.r_meas = float(r_meas)
        self.initialized = False

        self._F = np.eye(4, dtype=np.float64)
        self._Q = np.zeros((4, 4), dtype=np.float64)
        self._inn = np.zeros(2, dtype=np.float64)
        self._S_inv = np.zeros((2, 2), dtype=np.float64)

    def reset(self):
        self.x[:] = 0.0
        self.P = np.eye(4) * 500.0
        self.initialized = False

    def init(self, x, y):
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 500.0
        self.initialized = True

    def update(self, x_meas, y_meas, dt):
        dt = max(float(dt), 1e-3)

        if not self.initialized:
            self.init(x_meas, y_meas)
            return self.x[0], self.x[1], self.x[2], self.x[3]

        self._F[0, 2] = dt
        self._F[1, 3] = dt
        F = self._F

        dt2 = dt * dt
        dt3 = dt2 * dt * 0.5
        dt4 = dt2 * dt2 * 0.25
        self._Q.fill(0.0)
        self._Q[0, 0] = self._Q[1, 1] = self.q_pos + self.q_vel * dt4
        self._Q[2, 2] = self._Q[3, 3] = self.q_vel * dt2
        self._Q[0, 2] = self._Q[2, 0] = self.q_vel * dt3
        self._Q[1, 3] = self._Q[3, 1] = self.q_vel * dt3
        Q = self._Q

        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        self._inn[0] = float(x_meas) - x_pred[0]
        self._inn[1] = float(y_meas) - x_pred[1]
        S = P_pred[:2, :2] + np.eye(2) * self.r_meas

        det = S[0, 0] * S[1, 1] - S[0, 1] * S[1, 0]
        if abs(det) < 1e-9:
            self.x = x_pred
            self.P = P_pred
            return self.x[0], self.x[1], self.x[2], self.x[3]

        self._S_inv[0, 0] = S[1, 1] / det
        self._S_inv[0, 1] = -S[0, 1] / det
        self._S_inv[1, 0] = -S[1, 0] / det
        self._S_inv[1, 1] = S[0, 0] / det

        K = P_pred[:, :2] @ self._S_inv

        self.x = x_pred + K @ self._inn
        self.P = P_pred - K @ P_pred[:2, :]
        self.P = (self.P + self.P.T) * 0.5

        return self.x[0], self.x[1], self.x[2], self.x[3]


class WaypointFollower:
    def __init__(
        self,
        waypoints_flat,
        arrival_px,
        balance_window_px,
        balance_speed_px,
        hold_time_s,
    ):
        self.raw_path = np.array(waypoints_flat, dtype=np.float32).reshape(-1, 2)
        self.arrival = float(arrival_px)
        self.balance_window_px = float(balance_window_px)
        self.balance_speed_px = float(balance_speed_px)
        self.hold_time_s = float(hold_time_s)
        self.reset()

    def reset(self):
        self.done = False
        self.state = "MOVE"
        self.arrival_blend = 0.0
        self.target_wp_idx = 0 if len(self.raw_path) < 2 else 1

    def update(self, marble_xy, speed_px, dt):
        p = np.array(marble_xy, dtype=np.float32)
        _ = float(speed_px)
        _ = max(float(dt), 0.0)

        if len(self.raw_path) == 0:
            self.done = True
            self.state = "DONE"
            self.arrival_blend = 0.0
            self.target_wp_idx = -1
            return np.array([0.0, 0.0], dtype=np.float32), -1, True

        # Bounds check: ensure target_wp_idx is valid
        if self.target_wp_idx < 0 or self.target_wp_idx >= len(self.raw_path):
            self.target_wp_idx = 0

        target = self.raw_path[self.target_wp_idx].copy()
        dist = float(np.linalg.norm(p - target))
        capture_radius = max(self.arrival, self.balance_window_px)
        blend_span = max(capture_radius - self.balance_window_px, 1e-6)
        self.arrival_blend = float(np.clip(
            (capture_radius - dist) / blend_span,
            0.0,
            1.0,
        ))

        if dist < capture_radius:
            self.state = "ARRIVE"
        else:
            self.state = "MOVE"

        if dist < self.balance_window_px:
            if self.target_wp_idx >= len(self.raw_path) - 1:
                self.done = True
                self.state = "DONE"
                self.arrival_blend = 1.0
            else:
                self.target_wp_idx += 1
                target = self.raw_path[self.target_wp_idx].copy()
                self.state = "MOVE"
                self.arrival_blend = 0.0

        return target, self.target_wp_idx, self.done


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.declare_parameter("kp_x", 0.30)
        self.declare_parameter("kp_y", 0.30)
        self.declare_parameter("kd_x", 0.08)
        self.declare_parameter("kd_y", 0.08)
        self.declare_parameter("max_output", 80)
        self.declare_parameter("servo_center", 500)
        self.declare_parameter("arrival_px", 35.0)
        self.declare_parameter("balance_window_px", 18.0)
        self.declare_parameter("waypoint_balance_speed_px", 15.0)
        self.declare_parameter("balance_hold_s", 0.30)
        self.declare_parameter("derivative_alpha", 0.85)
        self.declare_parameter("hold_kp_scale", 0.70)
        self.declare_parameter("hold_kd_scale", 1.15)
        self.declare_parameter("cmd_time_ms", 20)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("loop_hz", 114.0)

        self.declare_parameter("kalman_q_pos", 1.0)
        self.declare_parameter("kalman_q_vel", 50.0)
        self.declare_parameter("kalman_r_meas", 10.0)

        self.declare_parameter("auto_start", False)
        self.declare_parameter("settle_speed_px", 15.0)
        self.declare_parameter("settle_frames", 10)
        self.declare_parameter("settle_timeout_s", 5.0)
        self.declare_parameter("recovery_wait_s", 3.0)

        self.declare_parameter("tilt_balance_enabled", True)
        self.declare_parameter("tilt_balance_kp", 8.0)
        self.declare_parameter("tilt_balance_ki", 1.0)
        self.declare_parameter("tilt_balance_deadband", 0.2)
        self.declare_parameter("tilt_balance_max_trim", 120.0)

        self.marble_pos = None
        self.waypoints = None
        self.est_state = None
        self.follower = None
        self.active = False
        self._levelling = False

        self.marble_lost_frames = 0
        self.marble_lost_threshold = 30
        self._pending_kalman_reset = False
        self._last_wp_idx = None
        self._was_active = False

        self._settling = False
        self._settle_count = 0
        self._settle_start = 0.0

        self._recovering = False
        self._recovery_arrived = False
        self._recovery_wait_until = 0.0

        self.board_M_inv = None
        self._tilt_trim_x = 0.0
        self._tilt_trim_y = 0.0
        self._tilt_int_x = 0.0
        self._tilt_int_y = 0.0

        self._state_lock = threading.Lock()
        self.t_last = time.time()

        self.pid_x = PD(0.0, 0.0, -80.0, 80.0)
        self.pid_y = PD(0.0, 0.0, -80.0, 80.0)
        self.kalman = KalmanFilter2D()
        self.measurement_derivative = MeasurementDerivative2D()

        self._load_params()
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.sub_marble = self.create_subscription(
            Point, "/marble/position", self._on_marble, 2
        )
        self.sub_wp = self.create_subscription(
            Float32MultiArray, "/path/waypoints", self._on_waypoints, 2
        )
        self.sub_state = self.create_subscription(
            Float32MultiArray, "/estimator/state", self._on_est_state, 10
        )
        self.sub_board_xfm = self.create_subscription(
            Float32MultiArray, "/estimator/board_transform",
            self._on_board_transform, 2
        )

        self.pub_cmd = self.create_publisher(Int32MultiArray, "/hiwonder/cmd", 1)
        self.pub_wp_idx = self.create_publisher(Int32MultiArray, "/controller/wp_idx", 1)

        self.create_service(Trigger, "/controller/start", self._svc_start)
        self.create_service(Trigger, "/controller/stop", self._svc_stop)
        self.create_service(Trigger, "/controller/calibrate", self._svc_calibrate)

        loop_hz = max(float(self.get_parameter("loop_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / loop_hz, self._tick)

        self.get_logger().info(
            "\n=== ControllerNode ready ===\n"
            "  /controller/start      -> start waypoint control\n"
            "  /controller/stop       -> stop + level board\n"
            "  /controller/calibrate  -> send centre, check marble pos\n"
        )

    def _on_params_changed(self, params):
        overrides = {param.name: param.value for param in params}
        self._load_params(overrides)
        return SetParametersResult(successful=True)

    def _load_params(self, overrides=None):
        overrides = overrides or {}

        def pv(name):
            if name in overrides:
                return overrides[name]
            return self.get_parameter(name).value

        mx = float(pv("max_output"))

        self.servo_center = int(pv("servo_center"))
        self.arrival_px = float(pv("arrival_px"))
        self.balance_window_px = float(pv("balance_window_px"))
        self.waypoint_balance_speed_px = float(pv("waypoint_balance_speed_px"))
        self.balance_hold_s = float(pv("balance_hold_s"))
        self.derivative_alpha = float(pv("derivative_alpha"))
        self.hold_kp_scale = float(pv("hold_kp_scale"))
        self.hold_kd_scale = float(pv("hold_kd_scale"))
        self.cmd_time_ms = int(pv("cmd_time_ms"))
        self.invert_x = bool(pv("invert_x"))
        self.invert_y = bool(pv("invert_y"))

        self.measurement_derivative.alpha = self.derivative_alpha

        self.kalman.q_pos = float(pv("kalman_q_pos"))
        self.kalman.q_vel = float(pv("kalman_q_vel"))
        self.kalman.r_meas = float(pv("kalman_r_meas"))

        self.auto_start = bool(pv("auto_start"))
        self.settle_speed_px = float(pv("settle_speed_px"))
        self.settle_frames = int(pv("settle_frames"))
        self.settle_timeout_s = float(pv("settle_timeout_s"))
        self.recovery_wait_s = float(pv("recovery_wait_s"))

        self.tilt_balance_enabled = bool(pv("tilt_balance_enabled"))
        self.tilt_balance_kp = float(pv("tilt_balance_kp"))
        self.tilt_balance_ki = float(pv("tilt_balance_ki"))
        self.tilt_balance_deadband = float(pv("tilt_balance_deadband"))
        self.tilt_balance_max_trim = float(pv("tilt_balance_max_trim"))

        self.pid_x.set_gains_and_limits(float(pv("kp_x")), float(pv("kd_x")), -mx, mx)
        self.pid_y.set_gains_and_limits(float(pv("kp_y")), float(pv("kd_y")), -mx, mx)

        if not self.tilt_balance_enabled:
            self._reset_tilt_balance()

        if self.follower is not None:
            self.follower.arrival = self.arrival_px
            self.follower.balance_window_px = self.balance_window_px
            self.follower.balance_speed_px = self.waypoint_balance_speed_px
            self.follower.hold_time_s = self.balance_hold_s

    def _on_est_state(self, msg):
        self.est_state = list(msg.data)

    def _publish_wp_idx(self, idx):
        if idx == self._last_wp_idx:
            return
        self._last_wp_idx = idx
        msg = Int32MultiArray()
        msg.data = [int(idx)]
        self.pub_wp_idx.publish(msg)

    def _build_follower(self, waypoints_flat):
        return WaypointFollower(
            waypoints_flat,
            self.arrival_px,
            self.balance_window_px,
            self.waypoint_balance_speed_px,
            self.balance_hold_s,
        )

    def _load_follower(self, waypoints_flat, log_loaded=True):
        self.follower = self._build_follower(waypoints_flat)
        if log_loaded:
            self.get_logger().info(
                f"Loaded new path with {len(self.follower.raw_path)} waypoints"
            )

    def _reset_all(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.measurement_derivative.reset()
        self._pending_kalman_reset = True

        if self.follower is not None:
            self.follower.reset()

        self._publish_wp_idx(-1)
        self._send_servo(0, 0, time_ms=60, level_hold=True)
        self.get_logger().info("Marble lost - reset controller state")

    def _tilt_balance_valid(self):
        return (
            self.tilt_balance_enabled
            and self.est_state is not None
            and len(self.est_state) > 5
            and float(self.est_state[5]) > 0.5
        )

    def _reset_tilt_balance(self):
        self._tilt_trim_x = 0.0
        self._tilt_trim_y = 0.0
        self._tilt_int_x = 0.0
        self._tilt_int_y = 0.0

    def _update_tilt_trim(self, dt):
        if not self._tilt_balance_valid():
            return

        tilt_x = float(self.est_state[3])
        tilt_y = float(self.est_state[4])

        if abs(tilt_x) < self.tilt_balance_deadband:
            tilt_x = 0.0
        if abs(tilt_y) < self.tilt_balance_deadband:
            tilt_y = 0.0

        self._tilt_int_x += tilt_x * dt
        self._tilt_int_y += tilt_y * dt

        if self.tilt_balance_ki > 1e-9:
            max_int = self.tilt_balance_max_trim / self.tilt_balance_ki
            self._tilt_int_x = float(np.clip(self._tilt_int_x, -max_int, max_int))
            self._tilt_int_y = float(np.clip(self._tilt_int_y, -max_int, max_int))

        self._tilt_trim_x = float(np.clip(
            -(self.tilt_balance_kp * tilt_x + self.tilt_balance_ki * self._tilt_int_x),
            -self.tilt_balance_max_trim,
            self.tilt_balance_max_trim,
        ))
        self._tilt_trim_y = float(np.clip(
            -(self.tilt_balance_kp * tilt_y + self.tilt_balance_ki * self._tilt_int_y),
            -self.tilt_balance_max_trim,
            self.tilt_balance_max_trim,
        ))

    def _start_active(self):
        self._settling = False
        self._settle_count = 0
        self.pid_x.reset()
        self.pid_y.reset()
        self.measurement_derivative.reset()
        if self.follower is not None:
            self.follower.reset()
        self.active = True
        self.t_last = time.time()
        self.get_logger().info("Marble settled - waypoint control started")

    def _on_marble(self, msg):
        with self._state_lock:
            self._on_marble_locked(msg)

    def _on_marble_locked(self, msg):
        was_none = self.marble_pos is None
        new_pos = None if msg.z < 0 else (float(msg.x), float(msg.y))

        if new_pos is None:
            self.marble_lost_frames += 1
            if self.marble_lost_frames >= self.marble_lost_threshold and not was_none:
                self._was_active = self.active
                self.marble_pos = None
                self.active = False
                self._levelling = True
                self._reset_all()
        else:
            # Hysteresis: only recover if frames drop below half threshold
            recovery_threshold = max(self.marble_lost_threshold // 2, 1)
            if was_none and self.marble_lost_frames < recovery_threshold:
                self._levelling = False
                self.get_logger().info("Marble reacquired")
                if self.follower is not None and (self._was_active or self.auto_start):
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.measurement_derivative.reset()
                    self._pending_kalman_reset = True
                    self._recovering = True
                    self._recovery_arrived = False
                    self._was_active = False
                    self.get_logger().info(
                        "Marble reacquired - guiding to start position"
                    )
            self.marble_lost_frames = 0
            self.marble_pos = new_pos

    def _on_board_transform(self, msg):
        try:
            if len(msg.data) != 9:
                self.board_M_inv = None
                return
            M = np.array(msg.data, dtype=np.float64).reshape(3, 3)
            if (not np.all(np.isfinite(M)) or np.linalg.cond(M) > 1e10):
                self.board_M_inv = None
                return
            self.board_M_inv = np.linalg.inv(M)
            self._pending_kalman_reset = True  # Reset Kalman after board calibration
        except (np.linalg.LinAlgError, ValueError):
            self.board_M_inv = None

    def _on_waypoints(self, msg):
        new_waypoints = list(msg.data)

        if not new_waypoints or len(new_waypoints) < 2:
            return

        if self.waypoints == new_waypoints:
            return

        if self.active:
            self.get_logger().warn("Received new path while active; ignoring update")
            return

        self.waypoints = new_waypoints
        self._load_follower(self.waypoints)

    def _send_servo(self, out_x, out_y, time_ms=None, level_hold=False):
        if time_ms is None:
            time_ms = self.cmd_time_ms

        if level_hold:
            self._update_tilt_trim(max(float(time_ms) / 1000.0, 1e-3))

        sx = -1 if self.invert_x else 1
        sy = -1 if self.invert_y else 1

        out_x_eff = float(out_x) + self._tilt_trim_x
        out_y_eff = float(out_y) + self._tilt_trim_y

        pos1 = int(np.clip(self.servo_center + sx * out_x_eff, 0, 1000))
        pos2 = int(np.clip(self.servo_center + sy * out_y_eff, 0, 1000))

        cmd = Int32MultiArray()
        cmd.data = [pos1, pos2, int(time_ms)]
        self.pub_cmd.publish(cmd)
        return pos1, pos2

    def _tick(self):
        now = time.time()
        dt = max(now - self.t_last, 1e-3)
        self.t_last = now

        if not self.active and not self._settling and not self._recovering:
            self._publish_wp_idx(-1)
            if self._levelling:
                self._send_servo(0, 0, level_hold=True)
            return

        with self._state_lock:
            marble_pos_snap = self.marble_pos
            pending_kalman_reset = self._pending_kalman_reset
            self._pending_kalman_reset = False

        if pending_kalman_reset:
            self.kalman.reset()
            self.measurement_derivative.reset()

        if marble_pos_snap is None:
            if self._settling:
                self._settling = False
                self.get_logger().info("Marble lost during settling - aborting")
            else:
                self.get_logger().warn(
                    "Waiting for marble...", throttle_duration_sec=1.0
                )
            return

        mx_raw, my_raw = marble_pos_snap
        if self.board_M_inv is not None:
            ph = self.board_M_inv @ np.array([mx_raw, my_raw, 1.0])
            mx_raw = float(ph[0] / ph[2])
            my_raw = float(ph[1] / ph[2])

        kx, ky, _, _ = self.kalman.update(mx_raw, my_raw, dt)
        mvx, mvy, measured_speed = self.measurement_derivative.update(kx, ky, dt)

        if self._settling:
            self._send_servo(0, 0, level_hold=True)
            if measured_speed < self.settle_speed_px:
                self._settle_count += 1
            else:
                self._settle_count = 0

            self.get_logger().info(
                f"Settling  speed={round(measured_speed, 1)} px/s"
                f"  count={self._settle_count}/{self.settle_frames}",
                throttle_duration_sec=0.3,
            )

            if self._settle_count >= self.settle_frames:
                self._start_active()
            elif now - self._settle_start > self.settle_timeout_s:
                self._settling = False
                self.get_logger().warn(
                    f"Settling timeout ({self.settle_timeout_s}s)"
                    " - marble did not settle, staying idle"
                )
            return

        if self._recovering:
            start_xy = self.follower.raw_path[0]
            err_x = float(start_xy[0]) - kx
            err_y = float(start_xy[1]) - ky
            dist = math.sqrt(err_x * err_x + err_y * err_y)
            if not self._recovery_arrived:
                if dist < self.arrival_px:
                    self._recovery_arrived = True
                    self._recovery_wait_until = now + self.recovery_wait_s
                    self._send_servo(0, 0, level_hold=True)
                    self.get_logger().info(
                        f"Reached start - waiting {self.recovery_wait_s:.1f}s before run"
                    )
                else:
                    out_x = self.pid_x.update(err_x, mvx)
                    out_y = self.pid_y.update(err_y, mvy)
                    self._send_servo(out_x, out_y)
                    self.get_logger().info(
                        f"Recovering  dist={round(dist)}px",
                        throttle_duration_sec=0.3,
                    )
            else:
                self._send_servo(0, 0, level_hold=True)
                remaining = self._recovery_wait_until - now
                self.get_logger().info(
                    f"Waiting at start  {round(remaining, 1)}s remaining",
                    throttle_duration_sec=0.5,
                )
                if now >= self._recovery_wait_until:
                    self._recovering = False
                    self._start_active()
            return

        target, wp_idx, done = self.follower.update((kx, ky), measured_speed, dt)
        self._publish_wp_idx(wp_idx)

        if done:
            self.get_logger().info("Goal reached! Stopping controller.")
            self._stop_and_level()
            self._publish_wp_idx(-1)
            self.active = False
            return

        err_x = float(target[0]) - kx
        err_y = float(target[1]) - ky
        dist_to_target = math.sqrt(err_x * err_x + err_y * err_y)

        arrival_blend = float(np.clip(self.follower.arrival_blend, 0.0, 1.0))
        kp_scale = 1.0 + arrival_blend * (self.hold_kp_scale - 1.0)
        kd_scale = 1.0 + arrival_blend * (self.hold_kd_scale - 1.0)

        out_x = self.pid_x.update(err_x, mvx, kp_scale, kd_scale)
        out_y = self.pid_y.update(err_y, mvy, kp_scale, kd_scale)

        mx = float(self.pid_x.hi)
        out_x = float(np.clip(out_x, -mx, mx))
        out_y = float(np.clip(out_y, -mx, mx))

        self._send_servo(out_x, out_y)

        self.get_logger().info(
            f"state={self.follower.state}"
            f"  target_wp={self.follower.target_wp_idx}"
            f"  target=({round(float(target[0]))},{round(float(target[1]))})"
            f"  dist={round(dist_to_target, 1)}"
            f"  err=({round(err_x, 1)},{round(err_y, 1)})"
            f"  speed={round(measured_speed, 1)}"
            f"  blend={round(arrival_blend, 2)}"
            f"  out=({round(out_x, 1)},{round(out_y, 1)})",
            throttle_duration_sec=0.2,
        )

    def _stop_and_level(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.kalman.reset()
        self.measurement_derivative.reset()
        self._send_servo(0, 0, time_ms=60, level_hold=True)

    def _svc_start(self, req, res):
        if self.follower is None and self.waypoints:
            self._load_follower(self.waypoints, log_loaded=False)

        if self.follower is None:
            res.success = False
            res.message = "No path loaded - call /path/draw first"
            return res

        if self.marble_pos is None:
            res.success = False
            res.message = "Marble not detected - check /marble/position"
            return res

        self.kalman.reset()
        self.measurement_derivative.reset()
        self.follower.reset()
        self._settling = True
        self._settle_count = 0
        self._settle_start = time.time()
        self.t_last = time.time()

        res.success = True
        res.message = "Controller settling - will activate when marble is still"
        self.get_logger().info("Controller SETTLING")
        return res

    def _svc_stop(self, req, res):
        self.active = False
        self._was_active = False
        self._settling = False
        self._recovering = False
        self._levelling = True
        self._stop_and_level()
        self._publish_wp_idx(-1)

        res.success = True
        res.message = "Stopped, levelling board from measured tilt"
        self.get_logger().info("Controller STOPPED")
        return res

    def _svc_calibrate(self, req, res):
        self.active = False
        self._settling = False
        self._recovering = False
        self._levelling = True
        self._stop_and_level()

        if self.marble_pos is not None:
            res.message = (
                "Board levelling from measured tilt. Marble at ("
                + str(round(self.marble_pos[0])) + ","
                + str(round(self.marble_pos[1]))
                + ") in topdown space (0-1000)"
            )
        else:
            res.message = "Board levelling from measured tilt. Marble NOT detected."

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
