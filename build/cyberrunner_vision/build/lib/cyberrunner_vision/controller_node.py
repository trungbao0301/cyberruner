"""
controller_node  (segment-following PD + Kalman filter)
--------------------------------------------------------
Servo mapping (Hiwonder 0-1000, centre=500):
  servo 1 → X-axis tilt  (right = >500, left = <500)
  servo 2 → Y-axis tilt  (down  = >500, up   = <500)

Control idea:
- Kalman filter estimates marble position + velocity from noisy detections
- Follow path segments with lookahead; PD error in filtered position space
- Kd acts on Kalman velocity directly (no noisy finite-difference needed)
- Latency prediction shifts the error target by predict_latency_s seconds
- Auto-restarts when marble is reacquired after being lost mid-run

PD error is in PIXELS (topdown space, 0-1000px).
Output is directly in servo UNITS (not degrees).

Parameters:
  kp_x, kp_y           float
  kd_x, kd_y           float
  max_output            int
  servo_center          int
  arrival_px            float   waypoint-switch radius
  lookahead_px          float   forward target distance along segment
  line_stick_gain       float   extra cross-track pull back toward current path segment
  cmd_time_ms           int
  ff_gain               float
  invert_x              bool
  invert_y              bool
  waypoint_pause_s      float   pause duration (s) after each waypoint advance
  predict_latency_s     float   latency compensation — advance error target by this many s
  kalman_q_pos          float   process noise — position (lower = trust model more)
  kalman_q_vel          float   process noise — velocity (higher = track fast moves)
  kalman_r_meas         float   measurement noise (higher = trust camera less)
  deg_per_unit          float   0.0   degrees of plate tilt per servo unit away from centre
                                      (0 = disabled, use manual kp/kd)
  omega_n_x             float   3.0   desired natural frequency X axis (rad/s)
  omega_n_y             float   2.0   desired natural frequency Y axis (rad/s)
                                      (use different values to reduce 2D overshoot)
  zeta                  float   1.0   desired damping ratio (1.0 = critically damped)
  friction_rho          float   0.0   Coulomb friction magnitude (servo units, 0=disabled)
  friction_eta          float   10.0  Coulomb friction sharpness (high=sign, low=smooth)
  tilt_balance_enabled  bool    True  keep flat/neutral using measured tilt_x/tilt_y
  tilt_balance_kp       float   8.0   servo-unit trim per degree of tilt error
  tilt_balance_ki       float   1.0   integral trim per degree-second of tilt error
  tilt_balance_deadband float   0.2   ignore tiny tilt noise around level
  tilt_balance_max_trim float   120.0 max neutral trim in servo units per axis

Services:
  /controller/start             Trigger
  /controller/stop              Trigger
  /controller/calibrate         Trigger
"""

import hashlib
import os
import json
import math
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


# ── PD ────────────────────────────────────────────────────────────────────────
class PD:
    def __init__(self, kp, kd, lo, hi):
        self.kp = kp
        self.kd = kd
        self.lo = lo
        self.hi = hi

    def reset(self):
        pass

    def set_gains_and_limits(self, kp, kd, lo, hi):
        self.kp = kp
        self.kd = kd
        self.lo = lo
        self.hi = hi

    def update(self, err, vel, kp_scale=1.0, kd_scale=1.0):
        """err: position error in px. vel: Kalman velocity in px/s (same axis).
        kp_scale / kd_scale: runtime multipliers for corner gain scheduling."""
        out = (self.kp * kp_scale) * err - (self.kd * kd_scale) * vel
        return float(np.clip(out, self.lo, self.hi))


# ── 2D Constant-velocity Kalman filter ───────────────────────────────────────
class KalmanFilter2D:
    """
    State:       [x, y, vx, vy]
    Measurement: [x, y]
    Model:       constant velocity  (x += vx*dt, y += vy*dt)

    Tuning:
      q_pos   — how much the position can drift from the model per step
                (low = smooth position, high = follows detections faster)
      q_vel   — how much velocity can change per step
                (high = tracks acceleration, low = smoother velocity)
      r_meas  — how noisy the camera measurement is
                (high = trust model more, low = trust camera more)
    """
    def __init__(self, q_pos=1.0, q_vel=50.0, r_meas=10.0):
        self.x = np.zeros(4)           # [x, y, vx, vy]
        self.P = np.eye(4) * 500.0     # large initial uncertainty
        self.q_pos  = q_pos
        self.q_vel  = q_vel
        self.r_meas = r_meas
        self.initialized = False
        # Pre-allocated prediction matrices — updated in-place each call
        # to avoid repeated numpy array allocation at 30 Hz.
        self._F = np.eye(4, dtype=np.float64)    # dt slots at [0,2] and [1,3]
        self._Q = np.zeros((4, 4), dtype=np.float64)
        self._inn   = np.zeros(2,      dtype=np.float64)
        self._S_inv = np.zeros((2, 2), dtype=np.float64)

    def reset(self):
        self.x[:] = 0.0
        self.P = np.eye(4) * 500.0
        self.initialized = False

    def init(self, x, y):
        self.x = np.array([x, y, 0.0, 0.0])
        self.P = np.eye(4) * 500.0
        self.initialized = True

    def update(self, x_meas, y_meas, dt):
        dt = max(dt, 1e-3)

        if not self.initialized:
            self.init(x_meas, y_meas)
            return self.x[0], self.x[1], self.x[2], self.x[3]

        # ── Predict ──────────────────────────────────────────────────────────
        # Update dt slots in-place — avoids allocating a new 4×4 array every call
        self._F[0, 2] = dt
        self._F[1, 3] = dt
        F = self._F
        # Update Q in-place with proper constant-velocity cross-terms (dt-dependent).
        # Derived from Q = q_vel * integral(F_c G G' F_c' dt) where G = [dt^2/2, dt]'
        # plus an independent position noise term q_pos on the diagonal.
        dt2 = dt * dt
        dt3 = dt2 * dt * 0.5    # dt^3 / 2
        dt4 = dt2 * dt2 * 0.25  # dt^4 / 4
        self._Q[0, 0] = self._Q[1, 1] = self.q_pos + self.q_vel * dt4
        self._Q[2, 2] = self._Q[3, 3] = self.q_vel * dt2
        self._Q[0, 2] = self._Q[2, 0] = self.q_vel * dt3   # x–vx coupling
        self._Q[1, 3] = self._Q[3, 1] = self.q_vel * dt3   # y–vy coupling
        Q = self._Q

        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        # ── Update ───────────────────────────────────────────────────────────
        # H = [[1,0,0,0],[0,1,0,0]] — exploit structure with slices, avoid matmul
        self._inn[0] = x_meas - x_pred[0]
        self._inn[1] = y_meas - x_pred[1]
        S   = P_pred[:2, :2] + np.eye(2) * self.r_meas         # S = H P H' + R (2×2)

        # Analytical 2×2 inverse — faster than np.linalg.inv for fixed-size matrix
        det = S[0, 0] * S[1, 1] - S[0, 1] * S[1, 0]
        if abs(det) < 1e-9:
            # Singular — skip update, return prediction only
            self.x = x_pred
            self.P = P_pred
            return self.x[0], self.x[1], self.x[2], self.x[3]
        self._S_inv[0, 0] =  S[1, 1] / det
        self._S_inv[0, 1] = -S[0, 1] / det
        self._S_inv[1, 0] = -S[1, 0] / det
        self._S_inv[1, 1] =  S[0, 0] / det

        K = P_pred[:, :2] @ self._S_inv                        # Kalman gain (4×2)

        self.x = x_pred + K @ self._inn
        self.P = P_pred - K @ P_pred[:2, :]                    # = (I - KH) P_pred
        self.P = (self.P + self.P.T) * 0.5                     # re-symmetrise against float drift

        return self.x[0], self.x[1], self.x[2], self.x[3]  # x, y, vx, vy


# ── Segment-following path follower ──────────────────────────────────────────
class PathFollower:
    def __init__(self, waypoints_flat, arrival_px, lookahead_px=25.0):
        self.path = np.array(waypoints_flat, dtype=np.float32).reshape(-1, 2)
        self.arrival = float(arrival_px)
        self.lookahead = float(lookahead_px)
        self.idx = 0              # current segment start index
        self.done = False
        # Angle (degrees) at each waypoint: 0 = straight, 90 = right angle, 180 = U-turn.
        # Precomputed once — used by controller for gain scheduling.
        self.corner_angles = self._compute_corner_angles()

    def _compute_corner_angles(self):
        n = len(self.path)
        angles = np.zeros(n, dtype=np.float32)
        for i in range(1, n - 1):
            ab = self.path[i]   - self.path[i - 1]
            bc = self.path[i + 1] - self.path[i]
            len_ab = float(np.linalg.norm(ab))
            len_bc = float(np.linalg.norm(bc))
            if len_ab < 1e-9 or len_bc < 1e-9:
                continue
            cos_a = float(np.dot(ab, bc) / (len_ab * len_bc))
            angles[i] = math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))
        return angles

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
            return (
                np.array([0.0, 0.0], dtype=np.float32),
                np.array([0.0, 0.0], dtype=np.float32),
                -1,
                True,
                0.0,
            )

        if len(self.path) == 1:
            target = self.path[0]
            if np.linalg.norm(p - target) < self.arrival:
                self.done = True
            return target, target.copy(), 0, self.done, 0.0

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
        line_stick_blend = 1.0

        # Look ahead along the path — if the lookahead overflows the current
        # segment, continue into the next segment(s).  This lets the controller
        # "see around the corner" on short segments and start pre-tilting early,
        # giving more time to execute sharp turns before reaching a hole.
        if seg_len > 1e-9:
            dist_to_end = (1.0 - t) * seg_len
            if self.lookahead > 1e-9:
                # Fade cross-track pull out near corners so line-sticking does
                # not fight the lookahead target as it starts to preview the
                # next segment.
                line_stick_blend = max(
                    0.0, min(1.0, dist_to_end / self.lookahead)
                )
            if self.lookahead <= dist_to_end:
                # Lookahead stays within current segment — normal case
                t_look = t + self.lookahead / seg_len
                target = a + t_look * (b - a)
            else:
                # Overflow: walk remaining lookahead along subsequent segments
                remaining = self.lookahead - dist_to_end
                look_idx  = self.idx + 1
                target    = b.copy()
                while remaining > 1e-9 and look_idx < len(self.path) - 1:
                    na = self.path[look_idx]
                    nb = self.path[look_idx + 1]
                    next_len = float(np.linalg.norm(nb - na))
                    if next_len < 1e-9:
                        look_idx += 1
                        continue
                    if remaining >= next_len:
                        remaining -= next_len
                        target    = nb.copy()
                        look_idx += 1
                    else:
                        target = na + (remaining / next_len) * (nb - na)
                        break
        else:
            target = b.copy()

        final_wp = self.path[-1]
        if np.linalg.norm(p - final_wp) < self.arrival:
            self.done = True
            target = final_wp.copy()
            proj = final_wp.copy()
            line_stick_blend = 0.0

        return target, proj, self.idx, self.done, line_stick_blend


# ── Controller Node ───────────────────────────────────────────────────────────
class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Parameters
        self.declare_parameter("kp_x", 0.3)
        self.declare_parameter("kp_y", 0.3)
        self.declare_parameter("kd_x", 0.08)
        self.declare_parameter("kd_y", 0.08)
        self.declare_parameter("max_output", 80)
        self.declare_parameter("servo_center", 500)
        self.declare_parameter("arrival_px", 35.0)
        self.declare_parameter("lookahead_px", 25.0)
        self.declare_parameter("line_stick_gain", 0.6)
        self.declare_parameter("cmd_time_ms", 20)
        self.declare_parameter("ff_gain", 0.0)
        self.declare_parameter("ff_board_scale", 10.0)  # est_state board-angle → servo unit scale
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("loop_hz", 114.0)
        self.declare_parameter("waypoint_pause_s",    0.3)
        self.declare_parameter("predict_latency_s",  0.05)
        self.declare_parameter("kalman_q_pos",  1.0)
        self.declare_parameter("kalman_q_vel",  50.0)
        self.declare_parameter("kalman_r_meas", 10.0)

        # Physics-model gains (Nokhbeh & Khashabi 2011, eq.25)
        self.declare_parameter("deg_per_unit", 0.0)   # 0 = disabled, use manual kp/kd
        self.declare_parameter("omega_n_x",    3.0)   # natural frequency X (rad/s)
        self.declare_parameter("omega_n_y",    2.0)   # natural frequency Y (rad/s)
        self.declare_parameter("zeta",         1.0)   # damping ratio
        # Coulomb friction feedforward (Nokhbeh & Khashabi 2011, §5.1.2)
        self.declare_parameter("friction_rho", 0.0)   # magnitude (0 = disabled)
        self.declare_parameter("friction_eta", 10.0)  # sharpness
        self.declare_parameter("tilt_balance_enabled", True)
        self.declare_parameter("tilt_balance_kp", 8.0)
        self.declare_parameter("tilt_balance_ki", 1.0)
        self.declare_parameter("tilt_balance_deadband", 0.2)
        self.declare_parameter("tilt_balance_max_trim", 120.0)

        # Corner gain scheduling — ramp Kp and Kd up near sharp turns
        self.declare_parameter("corner_kp_scale",     2.0)   # max Kp multiplier at corner
        self.declare_parameter("corner_kd_scale",     2.5)   # max Kd multiplier at corner
        self.declare_parameter("corner_angle_thresh", 25.0)  # min corner angle (deg) to activate
        self.declare_parameter("corner_preview_px",  100.0)  # ramp-up starts this far before corner

        # Auto-start after reload — settle before running
        self.declare_parameter("auto_start",        False)
        self.declare_parameter("settle_speed_px",   15.0)   # px/s below which marble is settled
        self.declare_parameter("settle_frames",     10)     # consecutive frames required
        self.declare_parameter("settle_timeout_s",  5.0)    # give up if marble hasn't settled
        self.declare_parameter("recovery_wait_s",   3.0)    # seconds to wait at start pos before running

        # ILC — iterative learning control
        self.declare_parameter("ilc_enabled", True)
        self.declare_parameter("ilc_gain",    0.1)    # learning rate L (0.05 – 0.3)
        self.declare_parameter("ilc_path",
                               os.path.expanduser("~/cyberrunner_ilc.json"))

        # State
        self.marble_pos = None   # raw position from marble_node
        self.waypoints = None
        self.est_state = None
        self.follower = None
        self.active = False
        self._levelling = False   # True while holding board flat after marble loss

        self.marble_lost_frames = 0
        self.marble_lost_threshold = 30
        self._pending_kalman_reset = False   # set in sub callback, consumed in timer tick
        self._last_wp_idx = None
        self._was_active   = False   # True if controller was active when marble was lost

        self._paused_at_wp = False
        self._pause_until  = 0.0

        # Settling state — board stays level while marble decelerates after reload
        self._settling      = False
        self._settle_count  = 0       # consecutive frames below speed threshold
        self._settle_start  = 0.0     # time.time() when settling began

        # Recovery state — guide marble back to start after loss, then wait
        self._recovering          = False
        self._recovery_arrived    = False
        self._recovery_wait_until = 0.0

        # ILC state
        self._ilc_ff          = None  # np.array (n_segments, 2) — feedforward corrections
        self._ilc_error_log   = None  # list of lists — path errors per segment per tick
        self._ilc_run_count   = 0     # total completed runs (for logging)
        self._current_path_hash = None

        self.board_M_inv  = None  # inverse of flat→current board homography
        self._tilt_trim_x = 0.0
        self._tilt_trim_y = 0.0
        self._tilt_int_x = 0.0
        self._tilt_int_y = 0.0

        # Lock protecting shared state written from subscriptions and read from
        # the timer tick when a MultiThreadedExecutor is used.
        self._state_lock = threading.Lock()

        self.t_last = time.time()

        # PD objects created once
        self.pid_x = PD(0.0, 0.0, -80.0, 80.0)
        self.pid_y = PD(0.0, 0.0, -80.0, 80.0)

        # Kalman filter for marble position + velocity
        self.kalman = KalmanFilter2D()

        self._load_params()
        self.add_on_set_parameters_callback(self._on_params_changed)

        # ROS
        self.sub_marble = self.create_subscription(
            Point, "/marble/position", self._on_marble, 2
        )
        self.sub_wp = self.create_subscription(
            Float32MultiArray, "/path/waypoints", self._on_waypoints, 2
        )
        self.sub_state = self.create_subscription(
            Float32MultiArray, "/estimator/state", self._on_est_state, 10)
        self.sub_board_xfm = self.create_subscription(
            Float32MultiArray, "/estimator/board_transform",
            self._on_board_transform, 2
        )

        self.pub_cmd = self.create_publisher(Int32MultiArray, "/hiwonder/cmd", 1)
        self.pub_wp_idx = self.create_publisher(Int32MultiArray, "/controller/wp_idx", 1)

        self.create_service(Trigger, "/controller/start",              self._svc_start)
        self.create_service(Trigger, "/controller/stop",               self._svc_stop)
        self.create_service(Trigger, "/controller/calibrate",          self._svc_calibrate)
        self.create_service(Trigger, "/controller/reset_ilc",          self._svc_reset_ilc)

        loop_hz = max(float(self.get_parameter("loop_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / loop_hz, self._tick)

        self.get_logger().info(
            "\n=== ControllerNode ready ===\n"
            "  /controller/start      → start following path\n"
            "  /controller/stop       → stop + level board\n"
            "  /controller/calibrate  → send centre, check marble pos\n"
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
        self.lookahead_px = float(pv("lookahead_px"))
        self.line_stick_gain = float(pv("line_stick_gain"))
        self.cmd_time_ms = int(pv("cmd_time_ms"))
        self.ff_gain        = float(pv("ff_gain"))
        self.ff_board_scale = float(pv("ff_board_scale"))
        self.invert_x = bool(pv("invert_x"))
        self.invert_y = bool(pv("invert_y"))
        self.waypoint_pause_s  = float(pv("waypoint_pause_s"))
        self.predict_latency_s = float(pv("predict_latency_s"))

        self.kalman.q_pos  = float(pv("kalman_q_pos"))
        self.kalman.q_vel  = float(pv("kalman_q_vel"))
        self.kalman.r_meas = float(pv("kalman_r_meas"))

        self.corner_kp_scale     = float(pv("corner_kp_scale"))
        self.corner_kd_scale     = float(pv("corner_kd_scale"))
        self.corner_angle_thresh = float(pv("corner_angle_thresh"))
        self.corner_preview_px   = float(pv("corner_preview_px"))

        self.auto_start       = bool(pv("auto_start"))
        self.settle_speed_px  = float(pv("settle_speed_px"))
        self.settle_frames    = int(pv("settle_frames"))
        self.settle_timeout_s = float(pv("settle_timeout_s"))
        self.recovery_wait_s  = float(pv("recovery_wait_s"))

        self.ilc_enabled = bool(pv("ilc_enabled"))
        self.ilc_gain    = float(pv("ilc_gain"))
        self._ilc_path   = pv("ilc_path")

        self.pid_x.set_gains_and_limits(
            float(pv("kp_x")),
            float(pv("kd_x")),
            -mx,
            mx,
        )
        self.pid_y.set_gains_and_limits(
            float(pv("kp_y")),
            float(pv("kd_y")),
            -mx,
            mx,
        )

        self.deg_per_unit  = float(pv("deg_per_unit"))
        self.omega_n_x     = float(pv("omega_n_x"))
        self.omega_n_y     = float(pv("omega_n_y"))
        self.zeta          = float(pv("zeta"))
        self.friction_rho  = float(pv("friction_rho"))
        self.friction_eta  = float(pv("friction_eta"))
        self.tilt_balance_enabled = bool(pv("tilt_balance_enabled"))
        self.tilt_balance_kp = float(pv("tilt_balance_kp"))
        self.tilt_balance_ki = float(pv("tilt_balance_ki"))
        self.tilt_balance_deadband = float(pv("tilt_balance_deadband"))
        self.tilt_balance_max_trim = float(pv("tilt_balance_max_trim"))
        if self.deg_per_unit > 0.0:
            self._apply_physics_gains(mx)

        if self.follower is not None:
            self.follower.arrival = self.arrival_px
            self.follower.lookahead = self.lookahead_px

    def _apply_physics_gains(self, mx):
        """Derive kp/kd from the ball-plate physics model (Nokhbeh & Khashabi 2011, eq.25).

        Plant:   x_ddot = -13.73 * alpha   [m/s² per rad]  (universal for solid sphere)
        K_plant = 13.73 * deg_per_unit*(π/180) * scale_px_m   [px/s² per servo-unit]
        kp = ωn² / K_plant,  kd = 2ζωn / K_plant

        X and Y use separate ωn (§5.2: equal ωn gives worst 2D overshoot).
        """
        scale_px_mm = (self.est_state[2]
                       if self.est_state and len(self.est_state) > 2
                       else 1.0)
        scale_px_m  = scale_px_mm * 1000.0
        K_plant = 13.73 * (self.deg_per_unit * math.pi / 180.0) * scale_px_m
        if K_plant < 1e-6:
            self.get_logger().warn("Physics gains: K_plant too small — check deg_per_unit")
            return
        kp_x = self.omega_n_x ** 2 / K_plant
        kd_x = 2.0 * self.zeta * self.omega_n_x / K_plant
        kp_y = self.omega_n_y ** 2 / K_plant
        kd_y = 2.0 * self.zeta * self.omega_n_y / K_plant
        self.pid_x.set_gains_and_limits(kp_x, kd_x, -mx, mx)
        self.pid_y.set_gains_and_limits(kp_y, kd_y, -mx, mx)
        self.get_logger().info(
            f"Physics gains: scale={scale_px_mm:.2f}px/mm  K_plant={K_plant:.4f}"
            f"  x: kp={kp_x:.4f} kd={kd_x:.4f} (ωn={self.omega_n_x})"
            f"  y: kp={kp_y:.4f} kd={kd_y:.4f} (ωn={self.omega_n_y})"
            f"  ζ={self.zeta}"
        )

    def _on_est_state(self, msg):
        prev_scale = (self.est_state[2]
                      if self.est_state and len(self.est_state) > 2
                      else None)
        self.est_state = list(msg.data)
        # Recompute physics gains only when scale changes (happens on corner re-click)
        if self.deg_per_unit > 0.0 and prev_scale != self.est_state[2]:
            mx = float(self.get_parameter("max_output").value)
            self._apply_physics_gains(mx)

    def _publish_wp_idx(self, idx):
        if idx == self._last_wp_idx:
            return
        self._last_wp_idx = idx
        msg = Int32MultiArray()
        msg.data = [int(idx)]
        self.pub_wp_idx.publish(msg)

    def _reset_all(self):
        # Called from the subscription callback — defer Kalman reset to the
        # timer thread via flag so that concurrent Kalman reads in _tick are safe
        # when a MultiThreadedExecutor is used.
        self.pid_x.reset()
        self.pid_y.reset()
        self._pending_kalman_reset = True

        if self.follower is not None:
            self.follower.reset()

        self._paused_at_wp = False
        self._publish_wp_idx(-1)
        self._send_servo(0, 0, time_ms=60, level_hold=True)
        self.get_logger().info("Marble lost — reset controller state")

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
        """Transition from SETTLING to ACTIVE. Kalman is already warmed up — do not reset it."""
        self._settling     = False
        self._settle_count = 0
        self.pid_x.reset()
        self.pid_y.reset()
        if self.follower is not None:
            self.follower.reset()
        self._paused_at_wp = False
        self._ilc_reset_log()
        self.active  = True
        self.t_last  = time.time()
        self.get_logger().info("Marble settled — path following started")

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
                self.marble_pos  = None
                self.active      = False
                self._levelling  = True
                self._ilc_end_run(completed=False)
                self._reset_all()
        else:
            if was_none and self.marble_lost_frames >= self.marble_lost_threshold:
                self._levelling = False
                self.get_logger().info("Marble reacquired")
                if self.follower is not None and (self._was_active or self.auto_start):
                    # Guide marble back to start position, wait, then run from beginning
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self._pending_kalman_reset = True
                    self._recovering          = True
                    self._recovery_arrived    = False
                    self._paused_at_wp        = False
                    self._was_active          = False
                    self.get_logger().info(
                        "Marble reacquired — guiding to start position")
            self.marble_lost_frames = 0
            self.marble_pos = new_pos

    def _on_board_transform(self, msg):
        """Receive flat→current board homography from estimator.
        We store its inverse (current→flat) to map marble pos into flat space."""
        M = np.array(msg.data, dtype=np.float64).reshape(3, 3)
        try:
            if np.linalg.cond(M) > 1e10:
                self.board_M_inv = None
                return
            self.board_M_inv = np.linalg.inv(M)
        except np.linalg.LinAlgError:
            self.board_M_inv = None

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
        # Initialise ILC for this path (loads saved data if path unchanged)
        n_seg = len(self.follower.path) - 1
        if n_seg > 0:
            self._current_path_hash = self._path_hash(new_waypoints)
            self._ilc_init(n_seg)
            self._ilc_reset_log()
        self.get_logger().info(
            f"Loaded new path with {len(self.follower.path)} waypoints"
        )

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
            marble_pos_snap        = self.marble_pos
            pending_kalman_reset   = self._pending_kalman_reset
            self._pending_kalman_reset = False

        if pending_kalman_reset:
            self.kalman.reset()

        if marble_pos_snap is None:
            if self._settling:
                self._settling = False
                self.get_logger().info("Marble lost during settling — aborting")
            else:
                self.get_logger().warn(
                    "Waiting for marble...", throttle_duration_sec=1.0)
            return

        # Map marble from current topdown space → flat board space so that
        # path waypoints (drawn when board was flat) stay aligned with the maze.
        mx_raw, my_raw = marble_pos_snap
        if self.board_M_inv is not None:
            ph = self.board_M_inv @ np.array([mx_raw, my_raw, 1.0])
            mx_raw, my_raw = float(ph[0] / ph[2]), float(ph[1] / ph[2])

        # Kalman always runs — in SETTLING this warms up the filter before ACTIVE
        kx, ky, vx, vy = self.kalman.update(mx_raw, my_raw, dt)

        # ── SETTLING ─────────────────────────────────────────────────────────
        if self._settling:
            self._send_servo(0, 0, level_hold=True)   # hold board flat from measured tilt
            speed = math.sqrt(vx * vx + vy * vy)
            if speed < self.settle_speed_px:
                self._settle_count += 1
            else:
                self._settle_count = 0

            self.get_logger().info(
                f"Settling  speed={round(speed, 1)} px/s"
                f"  count={self._settle_count}/{self.settle_frames}",
                throttle_duration_sec=0.3,
            )

            if self._settle_count >= self.settle_frames:
                self._start_active()
            elif now - self._settle_start > self.settle_timeout_s:
                self._settling = False
                self.get_logger().warn(
                    f"Settling timeout ({self.settle_timeout_s}s)"
                    f" — marble did not settle, staying idle")
            return

        # ── RECOVERING ───────────────────────────────────────────────────────
        # Guide marble to path start, hold flat for recovery_wait_s, then run
        if self._recovering:
            start_xy = self.follower.path[0]
            err_x = float(start_xy[0]) - kx
            err_y = float(start_xy[1]) - ky
            dist  = math.sqrt(err_x * err_x + err_y * err_y)
            if not self._recovery_arrived:
                if dist < self.arrival_px:
                    self._recovery_arrived    = True
                    self._recovery_wait_until = now + self.recovery_wait_s
                    self._send_servo(0, 0, level_hold=True)
                    self.get_logger().info(
                        f"Reached start — waiting {self.recovery_wait_s:.1f}s before run")
                else:
                    out_x = self.pid_x.update(err_x, vx)
                    out_y = self.pid_y.update(err_y, vy)
                    self._send_servo(out_x, out_y)
                    self.get_logger().info(
                        f"Recovering  dist={round(dist)}px",
                        throttle_duration_sec=0.3)
            else:
                self._send_servo(0, 0, level_hold=True)
                remaining = self._recovery_wait_until - now
                self.get_logger().info(
                    f"Waiting at start  {round(remaining, 1)}s remaining",
                    throttle_duration_sec=0.5)
                if now >= self._recovery_wait_until:
                    self._recovering = False
                    self._start_active()
            return

        # ── ACTIVE ───────────────────────────────────────────────────────────
        filtered_pos = (kx, ky)

        target, proj, seg_idx, done, line_stick_blend = self.follower.update(
            filtered_pos
        )

        # Detect waypoint advance → pause briefly
        if (self._last_wp_idx is not None
                and self._last_wp_idx >= 0
                and seg_idx > self._last_wp_idx
                and not self._paused_at_wp):
            self._paused_at_wp = True
            self._pause_until  = now + self.waypoint_pause_s
            self.pid_x.reset()
            self.pid_y.reset()
            self._send_servo(0, 0, time_ms=60, level_hold=True)
            self.get_logger().info(
                f"Waypoint {seg_idx} reached — pausing {self.waypoint_pause_s:.1f}s")

        self._publish_wp_idx(seg_idx)

        if done:
            self._ilc_end_run(completed=True)
            self.get_logger().info("Goal reached! Stopping controller.")
            self._stop_and_level()
            self._publish_wp_idx(-1)
            self.active = False
            return

        # Still waiting out waypoint pause
        if self._paused_at_wp:
            if now < self._pause_until:
                return
            self._paused_at_wp = False
            self.get_logger().info("Resuming after waypoint pause")

        # ILC error recording — raw path error before any target modification.
        # This is the true tracking error: how far the marble is from the path.
        if self._ilc_error_log is not None and seg_idx < len(self._ilc_error_log):
            self._ilc_error_log[seg_idx].append((
                float(target[0]) - kx,
                float(target[1]) - ky,
            ))

        tx, ty = float(target[0]), float(target[1])

        # Add extra weight to cross-track error so the marble hugs the current
        # segment more tightly instead of cutting wide around it. The effect
        # fades out near corners so it does not fight the next-segment preview.
        if self.line_stick_gain > 0.0:
            tx += self.line_stick_gain * line_stick_blend * (float(proj[0]) - kx)
            ty += self.line_stick_gain * line_stick_blend * (float(proj[1]) - ky)

        # ILC feedforward — pre-correct systematic error learned from past runs.
        if (self.ilc_enabled
                and self._ilc_ff is not None
                and seg_idx < len(self._ilc_ff)):
            tx += float(self._ilc_ff[seg_idx, 0])
            ty += float(self._ilc_ff[seg_idx, 1])

        # Latency prediction — shift error to where marble will be
        kx_p = kx + vx * self.predict_latency_s
        ky_p = ky + vy * self.predict_latency_s
        err_x = float(tx - kx_p)
        err_y = float(ty - ky_p)

        # ── Corner gain scheduling ────────────────────────────────────────────
        # As the marble approaches a sharp corner, smoothly ramp Kp and Kd up.
        # Kp increase → stronger lateral correction toward the new direction.
        # Kd increase → harder braking of velocity in the old direction.
        # Both scale linearly with blend (proximity) × angle_factor (sharpness).
        kp_scale = kd_scale = 1.0
        next_idx = seg_idx + 1
        if next_idx < len(self.follower.path):
            corner_angle = float(self.follower.corner_angles[next_idx])
            if corner_angle > self.corner_angle_thresh:
                nwp  = self.follower.path[next_idx]
                dist = math.sqrt((kx - float(nwp[0])) ** 2 +
                                 (ky - float(nwp[1])) ** 2)
                if dist < self.corner_preview_px:
                    blend        = 1.0 - dist / self.corner_preview_px
                    angle_factor = min(1.0, corner_angle / 90.0)
                    boost        = blend * angle_factor
                    kp_scale = 1.0 + boost * (self.corner_kp_scale - 1.0)
                    kd_scale = 1.0 + boost * (self.corner_kd_scale - 1.0)

        # PD uses Kalman velocity directly — no noisy finite-difference needed
        out_x = self.pid_x.update(err_x, vx, kp_scale, kd_scale)
        out_y = self.pid_y.update(err_y, vy, kp_scale, kd_scale)

        mx = float(self.pid_x.hi)

        # Feed-forward
        if self.est_state and len(self.est_state) > 5 and self.ff_gain > 0 and self.est_state[5] > 0.5:
            out_x += self.ff_gain * self.est_state[3] * self.ff_board_scale
            out_y += self.ff_gain * self.est_state[4] * self.ff_board_scale

        # Coulomb friction feedforward (Nokhbeh & Khashabi 2011, §5.1.2)
        # Tf = ρ * tanh(η * ẋ) — tilts board extra to overcome static/kinetic friction
        if self.friction_rho > 0.0:
            out_x += self.friction_rho * math.tanh(self.friction_eta * vx)
            out_y += self.friction_rho * math.tanh(self.friction_eta * vy)

        out_x = float(np.clip(out_x, -mx, mx))
        out_y = float(np.clip(out_y, -mx, mx))

        self._send_servo(out_x, out_y)

        self.get_logger().info(
            f"seg={seg_idx}/{len(self.follower.path)-2}"
            f"  target=({round(float(target[0]))},{round(float(target[1]))})"
            f"  cte={round(float(np.linalg.norm(np.array([kx, ky]) - proj)),1)}"
            f"  err=({round(err_x,1)},{round(err_y,1)})"
            f"  vel=({round(vx,1)},{round(vy,1)})"
            f"  out=({round(out_x,1)},{round(out_y,1)})"
            f"  kp_scale={round(kp_scale,2)}",
            throttle_duration_sec=0.2,
        )

    def _find_nearest_segment(self, pos):
        """Return the follower segment index whose projection is closest to pos."""
        p = np.array(pos, dtype=np.float32)
        best_idx, best_dist = 0, float('inf')
        for i in range(len(self.follower.path) - 1):
            proj, _, _ = PathFollower._project_to_segment(
                p, self.follower.path[i], self.follower.path[i + 1])
            d = float(np.linalg.norm(p - proj))
            if d < best_dist:
                best_dist = d
                best_idx  = i
        return best_idx

    def _stop_and_level(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.kalman.reset()
        self._send_servo(0, 0, time_ms=60, level_hold=True)

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

        self.kalman.reset()
        self.follower.reset()
        self._paused_at_wp = False
        self._ilc_reset_log()
        # Use the settling phase to warm up the Kalman velocity estimate before
        # activating PD — same path as auto_start.  _start_active() resets PD.
        self._settling     = True
        self._settle_count = 0
        self._settle_start = time.time()
        self.t_last        = time.time()

        res.success = True
        res.message = "Controller settling — will activate when marble is still"
        self.get_logger().info("Controller SETTLING")
        return res

    def _svc_stop(self, req, res):
        self.active      = False
        self._was_active = False   # manual stop — don't auto-restart
        self._settling   = False
        self._recovering = False
        self._levelling  = True
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

    # ── ILC helpers ───────────────────────────────────────────────────────────
    @staticmethod
    def _path_hash(waypoints_flat):
        data = ",".join(str(round(v)) for v in waypoints_flat)
        return hashlib.md5(data.encode()).hexdigest()

    def _ilc_init(self, n_segments):
        """Load saved ILC data if it matches the current path, else start fresh."""
        if os.path.exists(self._ilc_path):
            try:
                with open(self._ilc_path) as f:
                    data = json.load(f)
                if (data.get("path_hash") == self._current_path_hash
                        and len(data.get("ff", [])) == n_segments):
                    self._ilc_ff        = np.array(data["ff"], dtype=np.float64)
                    self._ilc_run_count = int(data.get("run_count", 0))
                    mags = np.linalg.norm(self._ilc_ff, axis=1)
                    self.get_logger().info(
                        f"ILC loaded  segments={n_segments}"
                        f"  runs={self._ilc_run_count}"
                        f"  max_ff={round(float(np.max(mags)), 1)}px")
                    return
            except Exception:
                pass
        self._ilc_ff        = np.zeros((n_segments, 2), dtype=np.float64)
        self._ilc_run_count = 0
        self.get_logger().info(f"ILC initialised  segments={n_segments}  (fresh)")

    def _ilc_reset_log(self):
        if self._ilc_ff is not None:
            self._ilc_error_log = [[] for _ in range(len(self._ilc_ff))]

    def _ilc_end_run(self, completed):
        """Update ILC feedforward from this run's error log, then save."""
        if self._ilc_ff is None or self._ilc_error_log is None:
            return
        updated = 0
        for i, errors in enumerate(self._ilc_error_log):
            if not errors:
                continue
            mean_ex = float(np.mean([e[0] for e in errors]))
            mean_ey = float(np.mean([e[1] for e in errors]))
            self._ilc_ff[i, 0] = float(np.clip(
                self._ilc_ff[i, 0] + self.ilc_gain * mean_ex, -150.0, 150.0))
            self._ilc_ff[i, 1] = float(np.clip(
                self._ilc_ff[i, 1] + self.ilc_gain * mean_ey, -150.0, 150.0))
            updated += 1
        self._ilc_run_count += 1
        self._ilc_save()
        self._ilc_reset_log()
        mags = np.linalg.norm(self._ilc_ff, axis=1)
        self.get_logger().info(
            f"ILC updated  run={self._ilc_run_count}"
            f"  segments_updated={updated}/{len(self._ilc_ff)}"
            f"  completed={completed}"
            f"  max_ff={round(float(np.max(mags)), 1)}px")

    def _ilc_save(self):
        tmp = self._ilc_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({
                "ff":         self._ilc_ff.tolist(),
                "run_count":  self._ilc_run_count,
                "path_hash":  self._current_path_hash,
            }, f, indent=2)
        os.replace(tmp, self._ilc_path)

    def _svc_reset_ilc(self, req, res):
        if self._ilc_ff is not None:
            n = len(self._ilc_ff)
            self._ilc_ff        = np.zeros((n, 2), dtype=np.float64)
            self._ilc_run_count = 0
            self._ilc_save()
            self._ilc_reset_log()
            res.success = True
            res.message = f"ILC reset ({n} segments)"
        else:
            res.success = False
            res.message = "No path loaded"
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
