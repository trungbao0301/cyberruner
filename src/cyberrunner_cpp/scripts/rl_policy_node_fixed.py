#!/usr/bin/env python3

"""
RL policy node for the current cyberrunner_cpp stack.

This node does NOT talk to the Hiwonder board directly.
It publishes normalized actions to /ri/action_cmd.
Your C++ ri_node converts those actions to Hiwonder position commands:

    action [-1, 1]  ->  pos = home_pos + action * action_span

Required C++ side:
    ros2 launch cyberrunner_cpp cyberrunner.launch.py launch_ri:=true ri_control_mode:=external
    ros2 service call /ri/start std_srvs/srv/Trigger {}

Modes:
    zero       -> always publish [0, 0]
    random     -> random safe actions for testing only
    heuristic  -> path-following hand policy using /ri/observation
    model      -> load Stable-Baselines3 model and run inference
"""

import math
import random
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RLPolicyNode(Node):
    def __init__(self):
        super().__init__("rl_policy_node")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("mode", "heuristic")  # zero, random, heuristic, model
        self.declare_parameter("publish_hz", 20.0)

        # Which future relative goal pair to follow from /ri/observation.
        # /path/relative_goal publishes dx0,dy0, dx1,dy1, ...
        self.declare_parameter("goal_pair_index", 2)
        self.declare_parameter("goal_norm_px", 120.0)

        # Heuristic gains. This is only a starting point, not final RL training.
        self.declare_parameter("kp", 0.75)
        self.declare_parameter("tilt_kp", 0.02)

        # Safety and smoothness.
        self.declare_parameter("max_action", 0.45)
        self.declare_parameter("action_rate_limit", 0.08)  # max action change per tick
        self.declare_parameter("deadband", 0.03)
        self.declare_parameter("publish_zero_when_inactive", True)

        # Axis correction. Keep false here first; prefer fixing invert_x/y in ri_node.
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        # Optional Stable-Baselines3 model path for real policy inference.
        self.declare_parameter("model_path", "")
        self.declare_parameter("deterministic", True)

        self.mode = str(self.get_parameter("mode").value)
        self.publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.goal_pair_index = max(0, int(self.get_parameter("goal_pair_index").value))
        self.goal_norm_px = max(1.0, float(self.get_parameter("goal_norm_px").value))
        self.kp = float(self.get_parameter("kp").value)
        self.tilt_kp = float(self.get_parameter("tilt_kp").value)
        self.max_action = self._clamp(float(self.get_parameter("max_action").value), 0.02, 1.0)
        self.action_rate_limit = self._clamp(
            float(self.get_parameter("action_rate_limit").value), 0.0, 1.0
        )
        self.deadband = self._clamp(float(self.get_parameter("deadband").value), 0.0, 0.5)
        self.publish_zero_when_inactive = bool(
            self.get_parameter("publish_zero_when_inactive").value
        )
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.deterministic = bool(self.get_parameter("deterministic").value)

        self.latest_obs: Optional[List[float]] = None
        self.prev_action = [0.0, 0.0]
        self.model = None

        if self.mode == "model":
            self._load_model()

        # -------------------------
        # ROS I/O
        # -------------------------
        self.sub_obs = self.create_subscription(
            Float32MultiArray,
            "/ri/observation",
            self.on_observation,
            10,
        )

        self.pub_action = self.create_publisher(
            Float32MultiArray,
            "/ri/action_cmd",
            10,
        )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.tick)

        self.get_logger().info(
            "RLPolicyNode ready: "
            f"mode={self.mode}, max_action={self.max_action}, "
            f"goal_pair_index={self.goal_pair_index}, publish_hz={self.publish_hz}"
        )

    def _load_model(self):
        if not self.model_path:
            self.get_logger().warn("mode=model but model_path is empty. Using zero action.")
            return
        try:
            from stable_baselines3 import PPO, SAC, TD3

            # Try common continuous-control algorithms.
            last_err = None
            for cls in (PPO, SAC, TD3):
                try:
                    self.model = cls.load(self.model_path)
                    self.get_logger().info(
                        f"Loaded RL model with {cls.__name__}: {self.model_path}"
                    )
                    return
                except Exception as exc:  # keep trying other SB3 classes
                    last_err = exc
            self.get_logger().error(f"Could not load model: {last_err}")
        except Exception as exc:
            self.get_logger().error(
                "stable-baselines3 is not available or model loading failed: " + str(exc)
            )

    def on_observation(self, msg: Float32MultiArray):
        self.latest_obs = list(msg.data)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _rate_limit(self, ax: float, ay: float) -> Tuple[float, float]:
        if self.action_rate_limit <= 0.0:
            self.prev_action = [ax, ay]
            return ax, ay

        dx = self._clamp(ax - self.prev_action[0], -self.action_rate_limit, self.action_rate_limit)
        dy = self._clamp(ay - self.prev_action[1], -self.action_rate_limit, self.action_rate_limit)
        ax = self.prev_action[0] + dx
        ay = self.prev_action[1] + dy
        self.prev_action = [ax, ay]
        return ax, ay

    def _sanitize_action(self, ax: float, ay: float) -> Tuple[float, float]:
        if not math.isfinite(ax):
            ax = 0.0
        if not math.isfinite(ay):
            ay = 0.0

        if abs(ax) < self.deadband:
            ax = 0.0
        if abs(ay) < self.deadband:
            ay = 0.0

        if self.invert_x:
            ax *= -1.0
        if self.invert_y:
            ay *= -1.0

        ax = self._clamp(ax, -self.max_action, self.max_action)
        ay = self._clamp(ay, -self.max_action, self.max_action)
        return self._rate_limit(ax, ay)

    def _obs_is_safe_active(self, obs: List[float]) -> bool:
        if len(obs) < 13:
            return False

        marble_valid = obs[4]
        off_path = obs[7]
        done = obs[8]
        active = obs[9]
        waiting = obs[10]

        if marble_valid < 0.5:
            return False
        if active < 0.5:
            return False
        if waiting > 0.5:
            return False
        if off_path > 0.5 or done > 0.5:
            return False
        return True

    def _heuristic_action(self, obs: List[float]) -> Tuple[float, float]:
        # Observation format from ri_node:
        # [0] tilt_x, [1] tilt_y, [2] marble_x, [3] marble_y,
        # [4] marble_valid, [5] progress_idx, [6] progress_delta,
        # [7] off_path, [8] done, [9] active, [10] waiting,
        # [11] episode, [12] episode_step,
        # [13...] relative goal pairs dx0,dy0, dx1,dy1...
        if not self._obs_is_safe_active(obs):
            return 0.0, 0.0

        rel_start = 13
        if len(obs) < rel_start + 2:
            return 0.0, 0.0

        pair_count = (len(obs) - rel_start) // 2
        pair_i = min(self.goal_pair_index, max(0, pair_count - 1))
        idx = rel_start + 2 * pair_i

        dx = obs[idx]
        dy = obs[idx + 1]
        tilt_x = obs[0]
        tilt_y = obs[1]

        gx = self._clamp(dx / self.goal_norm_px, -1.0, 1.0)
        gy = self._clamp(dy / self.goal_norm_px, -1.0, 1.0)

        # For Hiwonder position mode: action is desired tilt position, not velocity.
        ax = self.kp * gx - self.tilt_kp * tilt_x
        ay = self.kp * gy - self.tilt_kp * tilt_y
        return ax, ay

    def _model_action(self, obs: List[float]) -> Tuple[float, float]:
        if self.model is None:
            return 0.0, 0.0
        if not self._obs_is_safe_active(obs):
            return 0.0, 0.0

        # Use full observation by default. Your training environment should match this.
        action, _ = self.model.predict(obs, deterministic=self.deterministic)
        ax = float(action[0])
        ay = float(action[1])
        return ax, ay

    def compute_action(self, obs: List[float]) -> Tuple[float, float]:
        if self.mode == "zero":
            ax, ay = 0.0, 0.0
        elif self.mode == "random":
            if not self._obs_is_safe_active(obs):
                ax, ay = 0.0, 0.0
            else:
                ax = random.uniform(-self.max_action, self.max_action)
                ay = random.uniform(-self.max_action, self.max_action)
        elif self.mode == "model":
            ax, ay = self._model_action(obs)
        else:
            ax, ay = self._heuristic_action(obs)

        return self._sanitize_action(ax, ay)

    def publish_action(self, ax: float, ay: float):
        msg = Float32MultiArray()
        msg.data = [float(ax), float(ay)]
        self.pub_action.publish(msg)

    def tick(self):
        if self.latest_obs is None:
            if self.publish_zero_when_inactive:
                self.publish_action(0.0, 0.0)
            return

        ax, ay = self.compute_action(self.latest_obs)
        self.publish_action(ax, ay)

    def stop(self):
        self.prev_action = [0.0, 0.0]
        self.publish_action(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
