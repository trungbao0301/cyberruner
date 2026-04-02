"""
controller_waypoint_hold_node
-----------------------------
Simple waypoint-to-waypoint controller for a marble on a tilting plate.

Behavior:
- Move to the current waypoint with PD control.
- Estimate velocity from position difference over time.
- Low-pass filter the velocity estimate before using it in D control.
- When distance and speed are both small, enter HOLD state.
- In HOLD state, keep balancing at the same waypoint for waypoint_pause_s.
- After the hold time, advance to the next waypoint.

This node keeps the same topics/services as the existing controller so it can
drop into the current launch flow without changing path planning.
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


class LowPassVelocity2D:
    """Finite-difference velocity estimate with a first-order low-pass filter."""

    def __init__(self, alpha=0.7):
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
        alpha = float(np.clip(self.alpha, 0.0, 0.99))

        if self.prev_pos is None:
            self.prev_pos = (x, y)
            self.vx = 0.0
            self.vy = 0.0
            return self.vx, self.vy, 0.0

        px, py = self.prev_pos
        vx_raw = (x - px) / dt
        vy_raw = (y - py) / dt

        self.vx = alpha * self.vx + (1.0 - alpha) * vx_raw
        self.vy = alpha * self.vy + (1.0 - alpha) * vy_raw
        self.prev_pos = (x, y)

        speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        return self.vx, self.vy, speed


class WaypointHoldTracker:
    """Simple MOVE/HOLD state machine over a list of waypoints."""

    def __init__(self, waypoints_flat, arrival_px, speed_threshold_px, hold_time_s):
        self.path = np.array(waypoints_flat, dtype=np.float32).reshape(-1, 2)
        self.arrival = float(arrival_px)
        self.speed_threshold_px = float(speed_threshold_px)
        self.hold_time_s = float(hold_time_s)
        self.reset()

    def reset(self):
        self.done = False
        self.state = "MOVE"
        self.hold_elapsed_s = 0.0
        self.target_idx = 0 if len(self.path) > 0 else -1

    def update(self, marble_xy, speed_px, dt):
        p = np.array(marble_xy, dtype=np.float32)
        speed_px = float(speed_px)
        dt = max(float(dt), 0.0)

        if len(self.path) == 0:
            self.done = True
            self.target_idx = -1
            return np.array([0.0, 0.0], dtype=np.float32), -1, True, 0.0, self.state

        target = self.path[self.target_idx].copy()
        dist = float(np.linalg.norm(p - target))
        arrived = dist < self.arrival and speed_px < self.speed_threshold_px

        if self.state == "MOVE":
            self.hold_elapsed_s = 0.0
            if arrived:
                self.state = "HOLD"

        if self.state == "HOLD":
            if arrived:
                self.hold_elapsed_s += dt
            else:
                self.hold_elapsed_s = 0.0

            if self.hold_elapsed_s >= self.hold_time_s:
                if self.target_idx >= len(self.path) - 1:
                    self.done = True
                else:
                    self.target_idx += 1
                    self.state = "MOVE"
                    self.hold_elapsed_s = 0.0
                    target = self.path[self.target_idx].copy()
                    dist = float(np.linalg.norm(p - target))

        return target, self.target_idx, self.done, dist, self.state


class WaypointHoldControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self._declare_parameters()
        self._declare_compatibility_parameters()

        self.marble_pos = None
        self.waypoints = None
        self.est_state = None
        self.tracker = None
        self.active = False
        self._levelling = False
        self._last_wp_idx = None

        self.board_M_inv = None
        self._tilt_trim_x = 0.0
        self._tilt_trim_y = 0.0
        self._tilt_int_x = 0.0
        self._tilt_int_y = 0.0

        self._state_lock = threading.Lock()
        self._vel_est = LowPassVelocity2D()
        self.t_last = time.time()

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
        self.create_service(Trigger, "/controller/reset_ilc", self._svc_reset_ilc)

        self.timer = self.create_timer(1.0 / self.loop_hz, self._tick)

        self.get_logger().info(
            "\n=== WaypointHoldControllerNode ready ===\n"
            "  /controller/start      -> start MOVE/HOLD waypoint tracking\n"
            "  /controller/stop       -> stop + level board\n"
            "  /controller/calibrate  -> send centre and report marble position\n"
        )

    def _declare_parameters(self):
        self.declare_parameter("kp_x", 0.32)
        self.declare_parameter("kp_y", 0.32)
        self.declare_parameter("kd_x", 0.10)
        self.declare_parameter("kd_y", 0.10)
        self.declare_parameter("hold_kp_scale", 0.55)
        self.declare_parameter("hold_kd_scale", 1.20)
        self.declare_parameter("vel_lpf_alpha", 0.70)
        self.declare_parameter("deadzone_px", 5.0)
        self.declare_parameter("max_output", 80)
        self.declare_parameter("servo_center", 500)
        self.declare_parameter("arrival_px", 35.0)
        self.declare_parameter("speed_threshold_px", 18.0)
        self.declare_parameter("waypoint_pause_s", 1.0)
        self.declare_parameter("cmd_time_ms", 20)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("loop_hz", 114.0)

        self.declare_parameter("tilt_balance_enabled", True)
        self.declare_parameter("tilt_balance_kp", 8.0)
        self.declare_parameter("tilt_balance_ki", 1.0)
        self.declare_parameter("tilt_balance_deadband", 0.2)
        self.declare_parameter("tilt_balance_max_trim", 120.0)

    def _declare_compatibility_parameters(self):
        # Kept so the existing tuner/config files do not fail noisily when they
        # talk to this node. Most of these are intentionally unused here.
        compat_defaults = [
            ("lookahead_px", 25.0),
            ("balance_each_waypoint", True),
            ("waypoint_balance_speed_px", 18.0),
            ("kalman_q_pos", 1.0),
            ("kalman_q_vel", 50.0),
            ("kalman_r_meas", 10.0),
            ("predict_latency_s", 0.0),
            ("corner_kp_scale", 2.0),
            ("corner_kd_scale", 2.5),
            ("corner_angle_thresh", 25.0),
            ("corner_preview_px", 100.0),
            ("auto_start", False),
            ("settle_speed_px", 15.0),
            ("settle_frames", 10),
            ("settle_timeout_s", 5.0),
            ("ilc_enabled", False),
            ("ilc_gain", 0.1),
            ("deg_per_unit", 0.0),
            ("omega_n_x", 3.0),
            ("omega_n_y", 2.0),
            ("zeta", 1.0),
            ("friction_rho", 0.0),
            ("friction_eta", 10.0),
        ]
        for name, default in compat_defaults:
            self.declare_parameter(name, default)

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

        self.kp_x = float(pv("kp_x"))
        self.kp_y = float(pv("kp_y"))
        self.kd_x = float(pv("kd_x"))
        self.kd_y = float(pv("kd_y"))
        self.hold_kp_scale = float(pv("hold_kp_scale"))
        self.hold_kd_scale = float(pv("hold_kd_scale"))
        self.vel_lpf_alpha = float(pv("vel_lpf_alpha"))
        self.deadzone_px = float(pv("deadzone_px"))
        self.max_output = float(pv("max_output"))
        self.servo_center = int(pv("servo_center"))
        self.arrival_px = float(pv("arrival_px"))

        if "speed_threshold_px" in overrides:
            self.speed_threshold_px = float(overrides["speed_threshold_px"])
        elif "waypoint_balance_speed_px" in overrides:
            self.speed_threshold_px = float(overrides["waypoint_balance_speed_px"])
        else:
            self.speed_threshold_px = float(pv("speed_threshold_px"))

        self.waypoint_pause_s = float(pv("waypoint_pause_s"))
        self.cmd_time_ms = int(pv("cmd_time_ms"))
        self.invert_x = bool(pv("invert_x"))
        self.invert_y = bool(pv("invert_y"))
        self.loop_hz = max(float(pv("loop_hz")), 1.0)

        self.tilt_balance_enabled = bool(pv("tilt_balance_enabled"))
        self.tilt_balance_kp = float(pv("tilt_balance_kp"))
        self.tilt_balance_ki = float(pv("tilt_balance_ki"))
        self.tilt_balance_deadband = float(pv("tilt_balance_deadband"))
        self.tilt_balance_max_trim = float(pv("tilt_balance_max_trim"))

        self._vel_est.alpha = self.vel_lpf_alpha

        if not self.tilt_balance_enabled:
            self._reset_tilt_balance()

        if self.tracker is not None:
            self.tracker.arrival = self.arrival_px
            self.tracker.speed_threshold_px = self.speed_threshold_px
            self.tracker.hold_time_s = self.waypoint_pause_s

    def _build_tracker(self, waypoints_flat):
        self.tracker = WaypointHoldTracker(
            waypoints_flat,
            self.arrival_px,
            self.speed_threshold_px,
            self.waypoint_pause_s,
        )

    def _reset_run_state(self):
        self._vel_est.reset()
        self.t_last = time.time()
        if self.tracker is not None:
            self.tracker.reset()

    def _publish_wp_idx(self, idx):
        if idx == self._last_wp_idx:
            return
        self._last_wp_idx = idx
        msg = Int32MultiArray()
        msg.data = [int(idx)]
        self.pub_wp_idx.publish(msg)

    def _on_est_state(self, msg):
        self.est_state = list(msg.data)

    def _on_marble(self, msg):
        with self._state_lock:
            if msg.z < 0:
                self.marble_pos = None
            else:
                self.marble_pos = (float(msg.x), float(msg.y))

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
        self._build_tracker(self.waypoints)
        self.get_logger().info(
            f"Loaded new path with {len(self.tracker.path)} waypoints for MOVE/HOLD control"
        )

    def _on_board_transform(self, msg):
        M = np.array(msg.data, dtype=np.float64).reshape(3, 3)
        try:
            if (not np.all(np.isfinite(M))
                    or np.linalg.cond(M) > 1e10):
                self.board_M_inv = None
                return
            self.board_M_inv = np.linalg.inv(M)
        except np.linalg.LinAlgError:
            self.board_M_inv = None

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

    def _apply_deadzone(self, err):
        err = float(err)
        if abs(err) < self.deadzone_px:
            return 0.0
        return err

    def _current_gains(self):
        if self.tracker is not None and self.tracker.state == "HOLD":
            return (
                self.kp_x * self.hold_kp_scale,
                self.kp_y * self.hold_kp_scale,
                self.kd_x * self.hold_kd_scale,
                self.kd_y * self.hold_kd_scale,
            )
        return self.kp_x, self.kp_y, self.kd_x, self.kd_y

    def _stop_and_level(self):
        self._reset_run_state()
        self._send_servo(0, 0, time_ms=60, level_hold=True)

    def _tick(self):
        now = time.time()
        dt = max(now - self.t_last, 1e-3)
        self.t_last = now

        if not self.active:
            self._publish_wp_idx(-1)
            if self._levelling:
                self._send_servo(0, 0, level_hold=True)
            return

        with self._state_lock:
            marble_pos_snap = self.marble_pos

        if marble_pos_snap is None:
            self.get_logger().warn("Marble lost - stopping controller")
            self.active = False
            self._levelling = True
            self._stop_and_level()
            self._publish_wp_idx(-1)
            return

        if self.tracker is None:
            self.get_logger().warn("No path loaded - stopping controller")
            self.active = False
            self._levelling = True
            self._stop_and_level()
            self._publish_wp_idx(-1)
            return

        mx_raw, my_raw = marble_pos_snap
        if self.board_M_inv is not None:
            ph = self.board_M_inv @ np.array([mx_raw, my_raw, 1.0])
            mx_raw, my_raw = float(ph[0] / ph[2]), float(ph[1] / ph[2])

        vx, vy, speed = self._vel_est.update(mx_raw, my_raw, dt)

        target, wp_idx, done, dist, state = self.tracker.update(
            (mx_raw, my_raw), speed, dt
        )
        self._publish_wp_idx(wp_idx)

        if done:
            self.get_logger().info("Final waypoint held - stopping controller")
            self.active = False
            self._levelling = True
            self._stop_and_level()
            self._publish_wp_idx(-1)
            return

        err_x = self._apply_deadzone(float(target[0]) - mx_raw)
        err_y = self._apply_deadzone(float(target[1]) - my_raw)
        kp_x, kp_y, kd_x, kd_y = self._current_gains()

        out_x = kp_x * err_x - kd_x * vx
        out_y = kp_y * err_y - kd_y * vy
        out_x = float(np.clip(out_x, -self.max_output, self.max_output))
        out_y = float(np.clip(out_y, -self.max_output, self.max_output))

        self._send_servo(out_x, out_y)

        self.get_logger().info(
            f"state={state}"
            f"  wp={wp_idx + 1}/{len(self.tracker.path)}"
            f"  target=({round(float(target[0]))},{round(float(target[1]))})"
            f"  dist={round(dist, 1)}"
            f"  speed={round(speed, 1)}"
            f"  hold={round(self.tracker.hold_elapsed_s, 2)}/{round(self.waypoint_pause_s, 2)}s"
            f"  out=({round(out_x, 1)},{round(out_y, 1)})",
            throttle_duration_sec=0.2,
        )

    def _svc_start(self, req, res):
        if self.tracker is None and self.waypoints:
            self._build_tracker(self.waypoints)

        if self.tracker is None:
            res.success = False
            res.message = "No path loaded - call /path/draw first"
            return res

        if self.marble_pos is None:
            res.success = False
            res.message = "Marble not detected - check /marble/position"
            return res

        self._levelling = False
        self.active = True
        self._reset_run_state()
        res.success = True
        res.message = "MOVE/HOLD waypoint controller started"
        self.get_logger().info(res.message)
        return res

    def _svc_stop(self, req, res):
        self.active = False
        self._levelling = True
        self._stop_and_level()
        self._publish_wp_idx(-1)

        res.success = True
        res.message = "Stopped, levelling board from measured tilt"
        self.get_logger().info(res.message)
        return res

    def _svc_calibrate(self, req, res):
        self.active = False
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

    def _svc_reset_ilc(self, req, res):
        res.success = True
        res.message = "No ILC in controller_waypoint_hold_node"
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = WaypointHoldControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
