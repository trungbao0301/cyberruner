"""
controller_node  (path-projection PD + Kalman filter)
------------------------------------------------------
Servo mapping (Hiwonder 0-1000, centre=500):
  servo 1 → X-axis tilt  (right = >500, left = <500)
  servo 2 → Y-axis tilt  (down  = >500, up   = <500)

Control idea:
- Kalman filter estimates marble position + velocity from noisy detections
- Project onto the hand-drawn reference path and look ahead along arc length
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
  lookahead_px          float   forward target distance along path
  cmd_time_ms           int
  invert_x              bool
  invert_y              bool
  predict_latency_s     float   latency compensation — advance error target by this many s
  kalman_q_pos          float   process noise — position (lower = trust model more)
  kalman_q_vel          float   process noise — velocity (higher = track fast moves)
  kalman_r_meas         float   measurement noise (higher = trust camera less)
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


class ProjectedPathFollower:
    def __init__(self, waypoints_flat, arrival_px, lookahead_px=25.0,
                 sample_spacing_px=8.0):
        self.raw_path = np.array(waypoints_flat, dtype=np.float32).reshape(-1, 2)
        self.arrival = float(arrival_px)
        self.lookahead = float(lookahead_px)
        self.sample_spacing_px = float(sample_spacing_px)

        # Resample the exact hand-drawn path densely so we can do global
        # projection/lookahead without cutting across maze corners.
        self.path, self.path_parent_idx = self._build_sampled_path(self.raw_path)
        self.seg_lengths = self._compute_segment_lengths(self.path)
        self.seg_starts = self._compute_segment_starts(self.seg_lengths)
        self.total_length = float(np.sum(self.seg_lengths))
        self.corner_angles = self._compute_corner_angles(self.raw_path)
        self.num_segments = max(len(self.raw_path) - 1, 0)
        self._raw_wp_s = self._compute_raw_wp_s()

        self.idx = 0
        self.progress_s = 0.0
        self.done = False

    @staticmethod
    def _compute_corner_angles(path):
        n = len(path)
        angles = np.zeros(n, dtype=np.float32)
        for i in range(1, n - 1):
            ab = path[i] - path[i - 1]
            bc = path[i + 1] - path[i]
            len_ab = float(np.linalg.norm(ab))
            len_bc = float(np.linalg.norm(bc))
            if len_ab < 1e-9 or len_bc < 1e-9:
                continue
            cos_a = float(np.dot(ab, bc) / (len_ab * len_bc))
            angles[i] = math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))
        return angles

    @staticmethod
    def _compute_segment_lengths(path):
        if len(path) < 2:
            return np.zeros(0, dtype=np.float64)
        return np.linalg.norm(path[1:] - path[:-1], axis=1).astype(np.float64)

    @staticmethod
    def _compute_segment_starts(seg_lengths):
        starts = np.zeros(len(seg_lengths), dtype=np.float64)
        if len(seg_lengths) > 1:
            starts[1:] = np.cumsum(seg_lengths[:-1])
        return starts

    def _compute_raw_wp_s(self):
        """Arc-length s value for each raw waypoint on the sampled path.
        Raw waypoint wi is the endpoint of segment wi-1, which corresponds to
        the last sampled point whose parent_idx == wi-1."""
        s_vals = np.zeros(len(self.raw_path), dtype=np.float64)
        if len(self.path_parent_idx) == 0:
            return s_vals
        for wi in range(1, len(self.raw_path)):
            matches = np.where(self.path_parent_idx == wi - 1)[0]
            if len(matches) == 0:
                s_vals[wi] = self.total_length
            else:
                s_vals[wi] = float(self.seg_starts[matches[-1]])
        return s_vals

    def _build_sampled_path(self, raw_path):
        if len(raw_path) == 0:
            return np.zeros((0, 2), dtype=np.float32), np.zeros(0, dtype=np.int32)
        if len(raw_path) == 1:
            return raw_path.copy(), np.zeros(0, dtype=np.int32)

        pts = raw_path.astype(np.float64)
        sampled = [pts[0].astype(np.float32)]
        parent_idx = []

        for i in range(len(pts) - 1):
            seg = pts[i + 1] - pts[i]
            seg_len = float(np.linalg.norm(seg))
            if seg_len < 1e-9:
                continue

            steps = max(int(math.ceil(seg_len / self.sample_spacing_px)), 1)
            for step in range(1, steps + 1):
                t = step / steps
                pt = pts[i] + t * seg
                if np.linalg.norm(pt - sampled[-1]) < 1e-6:
                    continue
                sampled.append(pt.astype(np.float32))
                parent_idx.append(i)

        if len(sampled) == 1:
            sampled.append(pts[-1].astype(np.float32))
            parent_idx.append(max(len(pts) - 2, 0))

        return (np.array(sampled, dtype=np.float32),
                np.array(parent_idx, dtype=np.int32))

    def reset(self):
        self.idx = 0
        self.progress_s = 0.0
        self.done = False

    @staticmethod
    def _project_to_segment(p, a, b):
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

    def _project_to_path(self, p):
        best_dist2 = float("inf")
        best_idx = 0
        best_t = 0.0
        best_len = 0.0

        for i in range(len(self.path) - 1):
            proj, t, seg_len = self._project_to_segment(
                p, self.path[i], self.path[i + 1])
            d = p - proj
            dist2 = float(np.dot(d, d))
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_idx = i
                best_t = t
                best_len = seg_len

        s_proj = float(self.seg_starts[best_idx] + best_t * best_len)
        return best_idx, s_proj

    def _point_at_s(self, s):
        if len(self.path) == 0:
            return np.array([0.0, 0.0], dtype=np.float32)
        if len(self.path) == 1 or len(self.seg_lengths) == 0:
            return self.path[0].copy()

        s = float(np.clip(s, 0.0, self.total_length))
        seg_idx = int(np.searchsorted(self.seg_starts, s, side="right") - 1)
        seg_idx = int(np.clip(seg_idx, 0, len(self.seg_lengths) - 1))
        seg_len = float(self.seg_lengths[seg_idx])
        if seg_len < 1e-9:
            return self.path[seg_idx].copy()

        t = (s - self.seg_starts[seg_idx]) / seg_len
        return self.path[seg_idx] + t * (self.path[seg_idx + 1] - self.path[seg_idx])

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

        _, s_proj = self._project_to_path(p)
        max_step = max(self.lookahead * 2.0, 60.0)
        s_proj = min(max(s_proj, self.progress_s), self.progress_s + max_step)
        self.progress_s = max(self.progress_s, s_proj)

        # Force advance past any raw waypoint the marble is close enough to.
        # Handles the case where the marble approaches from the side and the
        # arc-length projection stalls before reaching the waypoint's s value.
        for wi, wp_s in enumerate(self._raw_wp_s):
            if (wp_s > self.progress_s
                    and np.linalg.norm(p - self.raw_path[wi]) < self.arrival):
                self.progress_s = wp_s
                break

        target_s = min(self.progress_s + self.lookahead, self.total_length)
        target = self._point_at_s(target_s)

        sample_seg_idx = int(np.searchsorted(
            self.seg_starts, self.progress_s, side="right") - 1)
        sample_seg_idx = int(np.clip(sample_seg_idx, 0, len(self.path) - 2))

        if len(self.path_parent_idx) > 0:
            raw_idx = int(self.path_parent_idx[
                int(np.clip(sample_seg_idx, 0, len(self.path_parent_idx) - 1))])
        else:
            raw_idx = 0
        self.idx = raw_idx

        final_wp = self.raw_path[-1]
        if (np.linalg.norm(p - final_wp) < self.arrival
                or self.total_length - self.progress_s < self.arrival):
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
        self.declare_parameter("kd_x", 0.08)
        self.declare_parameter("kd_y", 0.08)
        self.declare_parameter("max_output", 80)
        self.declare_parameter("servo_center", 500)
        self.declare_parameter("arrival_px", 35.0)
        self.declare_parameter("lookahead_px", 25.0)
        self.declare_parameter("cmd_time_ms", 20)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("loop_hz", 114.0)
        self.declare_parameter("predict_latency_s",  0.05)
        self.declare_parameter("kalman_q_pos",  1.0)
        self.declare_parameter("kalman_q_vel",  50.0)
        self.declare_parameter("kalman_r_meas", 10.0)

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

        # Model feedforward — physics-based inversion of rolling-ball dynamics.
        # Model: ẍ = (5/7)·g·β,  ÿ = −(5/7)·g·α  (estimator.py KF B-matrix)
        # Inversion: given desired acceleration, compute required tilt angle,
        # then convert to servo units via ff_su_per_rad.
        # ff_scale=0  → disabled (safe default);  ff_scale=1 → full model feedforward.
        # m_per_px is read live from /estimator/state[2] (scale px/mm).
        self.declare_parameter("ff_su_per_rad",  458.4)  # servo-units per radian ≈ 8 su/deg × (180/π)
        self.declare_parameter("ff_T_horizon",   0.10)   # time-horizon for inversion [s]; tune 0.05–0.3
        self.declare_parameter("ff_scale",       0.0)    # blend 0=off → 1=full; start low (~0.3)

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
        self.cmd_time_ms = int(pv("cmd_time_ms"))
        self.invert_x = bool(pv("invert_x"))
        self.invert_y = bool(pv("invert_y"))
        self.predict_latency_s = float(pv("predict_latency_s"))

        self.kalman.q_pos = float(pv("kalman_q_pos"))
        self.kalman.q_vel = float(pv("kalman_q_vel"))
        self.kalman.r_meas = float(pv("kalman_r_meas"))

        self.corner_kp_scale = float(pv("corner_kp_scale"))
        self.corner_kd_scale = float(pv("corner_kd_scale"))
        self.corner_angle_thresh = float(pv("corner_angle_thresh"))
        self.corner_preview_px = float(pv("corner_preview_px"))

        self.auto_start = bool(pv("auto_start"))
        self.settle_speed_px = float(pv("settle_speed_px"))
        self.settle_frames = int(pv("settle_frames"))
        self.settle_timeout_s = float(pv("settle_timeout_s"))
        self.recovery_wait_s = float(pv("recovery_wait_s"))

        self.ilc_enabled = bool(pv("ilc_enabled"))
        self.ilc_gain = float(pv("ilc_gain"))
        self._ilc_path = pv("ilc_path")
        self.ff_su_per_rad = float(pv("ff_su_per_rad"))
        self.ff_T_horizon  = float(pv("ff_T_horizon"))
        self.ff_scale      = float(pv("ff_scale"))

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

        self.tilt_balance_enabled = bool(pv("tilt_balance_enabled"))
        self.tilt_balance_kp = float(pv("tilt_balance_kp"))
        self.tilt_balance_ki = float(pv("tilt_balance_ki"))
        self.tilt_balance_deadband = float(pv("tilt_balance_deadband"))
        self.tilt_balance_max_trim = float(pv("tilt_balance_max_trim"))
        if not self.tilt_balance_enabled:
            self._reset_tilt_balance()

        if self.follower is not None:
            self.follower.arrival = self.arrival_px
            self.follower.lookahead = self.lookahead_px

    def _on_est_state(self, msg):
        self.est_state = list(msg.data)

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
            if (not np.all(np.isfinite(M))
                    or np.linalg.cond(M) > 1e10):
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
        self.follower = ProjectedPathFollower(
            self.waypoints,
            self.arrival_px,
            self.lookahead_px
        )
        # Initialise ILC for this path (loads saved data if path unchanged)
        n_seg = self.follower.num_segments
        if n_seg > 0:
            self._current_path_hash = self._path_hash(new_waypoints)
            self._ilc_init(n_seg)
            self._ilc_reset_log()
        self.get_logger().info(
            f"Loaded new path with {len(self.follower.raw_path)} waypoints"
            f" and {len(self.follower.path)} sampled points"
        )

    def _send_servo(self, out_x, out_y, time_ms=None, level_hold=False):
        if time_ms is None:
            time_ms = self.cmd_time_ms

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
            self._send_servo(0, 0, level_hold=True)
            return

        with self._state_lock:
            marble_pos_snap      = self.marble_pos
            pending_kalman_reset = self._pending_kalman_reset
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
            self._send_servo(0, 0)
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
            self._send_servo(0, 0, level_hold=True)
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
            start_xy = self.follower.raw_path[0]
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

        target, seg_idx, done = self.follower.update(filtered_pos)

        self._publish_wp_idx(seg_idx)

        if done:
            self._ilc_end_run(completed=True)
            self.get_logger().info("Goal reached! Stopping controller.")
            self._stop_and_level()
            self._publish_wp_idx(-1)
            self.active = False
            return

        # ILC error recording — raw path error before any target modification.
        # This is the true tracking error: how far the marble is from the path.
        if self._ilc_error_log is not None and seg_idx < len(self._ilc_error_log):
            self._ilc_error_log[seg_idx].append((
                float(target[0]) - kx,
                float(target[1]) - ky,
            ))

        tx, ty = float(target[0]), float(target[1])

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

        # ── Physics feedforward ───────────────────────────────────────────────
        # Model (estimator.py KF B-matrix): ẍ = (5/7)·g·β,  ÿ = −(5/7)·g·α
        # Inversion: what tilt is needed to bring marble from (kx_p, vx) to tx
        #            in ff_T_horizon seconds?
        #   x(T) = kx_p + vx·T + ½·ax·T²  →  ax = 2·(err_x − vx·T) / T²  [px/s²]
        #   β = ax · m_per_px · 7/(5·g)   [rad]
        #   out_ff = β · ff_su_per_rad     [servo units]
        # m_per_px is read from /estimator/state[2] (scale in px/mm).
        if (self.ff_scale > 0.0
                and self.est_state is not None
                and len(self.est_state) > 2
                and float(self.est_state[2]) > 0.0):
            m_per_px = 1.0 / (float(self.est_state[2]) * 1000.0)  # px/mm → m/px
            T   = max(self.ff_T_horizon, dt)
            _k  = m_per_px * (7.0 / (5.0 * 9.81)) * self.ff_su_per_rad * self.ff_scale
            ff_x = _k * 2.0 * (err_x - vx * T) / (T * T)
            ff_y = _k * 2.0 * (err_y - vy * T) / (T * T)
        else:
            ff_x = ff_y = 0.0

        # ── Corner gain scheduling ────────────────────────────────────────────
        # As the marble approaches a sharp corner, smoothly ramp Kp and Kd up.
        # Kp increase → stronger lateral correction toward the new direction.
        # Kd increase → harder braking of velocity in the old direction.
        # Both scale linearly with blend (proximity) × angle_factor (sharpness).
        kp_scale = kd_scale = 1.0
        next_idx = seg_idx + 1
        if next_idx < len(self.follower.raw_path):
            corner_angle = float(self.follower.corner_angles[next_idx])
            if corner_angle > self.corner_angle_thresh:
                nwp  = self.follower.raw_path[next_idx]
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

        out_x = float(np.clip(out_x + ff_x, -mx, mx))
        out_y = float(np.clip(out_y + ff_y, -mx, mx))

        self._send_servo(out_x, out_y)

        self.get_logger().info(
            f"seg={seg_idx}/{max(self.follower.num_segments - 1, 0)}"
            f"  target=({round(float(target[0]))},{round(float(target[1]))})"
            f"  err=({round(err_x,1)},{round(err_y,1)})"
            f"  vel=({round(vx,1)},{round(vy,1)})"
            f"  ff=({round(ff_x,1)},{round(ff_y,1)})"
            f"  out=({round(out_x,1)},{round(out_y,1)})"
            f"  kp_scale={round(kp_scale,2)}",
            throttle_duration_sec=0.2,
        )

    def _stop_and_level(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.kalman.reset()
        self._send_servo(0, 0, time_ms=60, level_hold=True)

    def _svc_start(self, req, res):
        if self.follower is None and self.waypoints:
            self.follower = ProjectedPathFollower(
                self.waypoints,
                self.arrival_px,
                self.lookahead_px
            )

        if self.follower is None:
            res.success = False
            res.message = "No path loaded — call /path/draw first"
            return res

        if self.board_M_inv is None:
            self.get_logger().warn(
                "board_transform not received — marble position and path waypoints "
                "may be in different spaces. Ensure estimator is calibrated.")

        if self.marble_pos is None:
            res.success = False
            res.message = "Marble not detected — check /marble/position"
            return res

        self.kalman.reset()
        self.follower.reset()
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
