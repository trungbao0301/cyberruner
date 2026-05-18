#!/usr/bin/env python3

import math
import random
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RLPolicyNode(Node):
    """
    Stronger test policy for CyberRunner RI.

    Publishes:
        /ri/action_cmd Float32MultiArray [ax, ay]

    Modes:
        zero      -> [0, 0]
        random    -> random strong pushes
        heuristic -> move toward relative path goal
        sweep     -> test one-axis sweep

    Important:
        This is still not trained RL.
        It is a stronger hardware-test policy.
    """

    def __init__(self):
        super().__init__("rl_policy_node")

        self.declare_parameter("mode", "random")
        self.declare_parameter("publish_hz", 10.0)

        self.declare_parameter("max_action", 1.0)
        self.declare_parameter("min_push", 0.45)
        self.declare_parameter("deadband", 0.05)

        self.declare_parameter("random_hold_s", 0.6)

        self.declare_parameter("goal_pair_index", 2)
        self.declare_parameter("goal_norm_px", 100.0)
        self.declare_parameter("kp", 1.2)
        self.declare_parameter("tilt_kp", 0.0)

        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.mode = str(self.get_parameter("mode").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.max_action = float(self.get_parameter("max_action").value)
        self.min_push = float(self.get_parameter("min_push").value)
        self.deadband = float(self.get_parameter("deadband").value)

        self.random_hold_s = float(self.get_parameter("random_hold_s").value)

        self.goal_pair_index = int(self.get_parameter("goal_pair_index").value)
        self.goal_norm_px = float(self.get_parameter("goal_norm_px").value)
        self.kp = float(self.get_parameter("kp").value)
        self.tilt_kp = float(self.get_parameter("tilt_kp").value)

        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.max_action = max(0.05, min(1.0, self.max_action))
        self.min_push = max(0.0, min(self.max_action, self.min_push))
        self.goal_norm_px = max(1.0, self.goal_norm_px)
        self.publish_hz = max(1.0, self.publish_hz)
        self.random_hold_s = max(0.05, self.random_hold_s)

        self.latest_obs: Optional[List[float]] = None

        self.random_action = [0.0, 0.0]
        self.last_random_time = self.get_clock().now()

        self.sweep_phase = 0

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

        self.timer = self.create_timer(
            1.0 / self.publish_hz,
            self.tick,
        )

        self.get_logger().info(
            f"RLPolicyNode started: mode={self.mode}, "
            f"max_action={self.max_action}, min_push={self.min_push}, "
            f"publish_hz={self.publish_hz}"
        )

    def on_observation(self, msg: Float32MultiArray):
        self.latest_obs = list(msg.data)

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def apply_min_push(self, x):
        """
        If command is nonzero but too small, force it to minimum push.
        This helps overcome static friction.
        """
        if abs(x) < self.deadband:
            return 0.0

        sign = 1.0 if x > 0.0 else -1.0
        mag = max(abs(x), self.min_push)
        mag = min(mag, self.max_action)
        return sign * mag

    def clean_action(self, ax, ay):
        if math.isnan(ax) or math.isinf(ax):
            ax = 0.0
        if math.isnan(ay) or math.isinf(ay):
            ay = 0.0

        ax = self.clamp(ax, -self.max_action, self.max_action)
        ay = self.clamp(ay, -self.max_action, self.max_action)

        ax = self.apply_min_push(ax)
        ay = self.apply_min_push(ay)

        if self.invert_x:
            ax *= -1.0
        if self.invert_y:
            ay *= -1.0

        return ax, ay

    def random_policy(self):
        """
        Random action, but held for a short time.
        Without holding, the servo jitters around center and does not push the marble well.
        """
        now = self.get_clock().now()
        age = (now - self.last_random_time).nanoseconds * 1e-9

        if age >= self.random_hold_s:
            # Choose strong random actions.
            ax = random.choice([-1.0, 1.0]) * random.uniform(self.min_push, self.max_action)
            ay = random.choice([-1.0, 1.0]) * random.uniform(self.min_push, self.max_action)

            # Sometimes push only one axis to make behavior easier to see.
            if random.random() < 0.35:
                ay = 0.0
            elif random.random() < 0.70:
                ax = 0.0

            self.random_action = [ax, ay]
            self.last_random_time = now

        return self.random_action[0], self.random_action[1]

    def sweep_policy(self):
        """
        Deterministic test pattern.
        Useful to prove commands are strong enough.
        """
        t = self.get_clock().now().nanoseconds * 1e-9
        period = 2.0
        phase = int(t / period) % 4

        if phase == 0:
            return self.max_action, 0.0
        if phase == 1:
            return -self.max_action, 0.0
        if phase == 2:
            return 0.0, self.max_action
        return 0.0, -self.max_action

    def heuristic_policy(self, obs: List[float]):
        """
        Uses /ri/observation:
          [0] tilt_x
          [1] tilt_y
          [4] marble_valid
          [7] off_path
          [8] done
          [9] active
          [10] waiting
          [13...] relative goal pairs
        """
        if len(obs) < 15:
            return 0.0, 0.0

        tilt_x = obs[0]
        tilt_y = obs[1]

        marble_valid = obs[4]
        off_path = obs[7]
        done = obs[8]
        active = obs[9]
        waiting = obs[10]

        if marble_valid < 0.5 or active < 0.5 or waiting > 0.5:
            return 0.0, 0.0

        if off_path > 0.5 or done > 0.5:
            return 0.0, 0.0

        rel_start = 13
        pair_i = max(0, self.goal_pair_index)
        idx = rel_start + 2 * pair_i

        if idx + 1 >= len(obs):
            idx = rel_start

        if idx + 1 >= len(obs):
            return 0.0, 0.0

        dx = obs[idx]
        dy = obs[idx + 1]

        gx = self.clamp(dx / self.goal_norm_px, -1.0, 1.0)
        gy = self.clamp(dy / self.goal_norm_px, -1.0, 1.0)

        ax = self.kp * gx - self.tilt_kp * tilt_x
        ay = self.kp * gy - self.tilt_kp * tilt_y

        return ax, ay

    def compute_action(self):
        if self.mode == "zero":
            return 0.0, 0.0

        if self.mode == "random":
            return self.random_policy()

        if self.mode == "sweep":
            return self.sweep_policy()

        if self.mode == "heuristic":
            if self.latest_obs is None:
                return 0.0, 0.0
            return self.heuristic_policy(self.latest_obs)

        self.get_logger().warn(f"Unknown mode={self.mode}; using zero")
        return 0.0, 0.0

    def tick(self):
        ax, ay = self.compute_action()
        ax, ay = self.clean_action(ax, ay)

        msg = Float32MultiArray()
        msg.data = [float(ax), float(ay)]
        self.pub_action.publish(msg)

        self.get_logger().info(
            f"action_cmd ax={ax:.3f}, ay={ay:.3f}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_msg = Float32MultiArray()
    stop_msg.data = [0.0, 0.0]
    node.pub_action.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()