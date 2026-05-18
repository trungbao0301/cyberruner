#!/usr/bin/env python3

import math
import random
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class RLPolicyNode(Node):
    """
    Simple RL policy node for CyberRunner.

    Subscribes:
        /ri/observation   Float32MultiArray

    Publishes:
        /ri/action_cmd    Float32MultiArray [ax, ay]

    Action range:
        ax, ay in [-1, 1]

    Your ri_node.cpp converts this to Hiwonder position commands:
        servo = home_pos + action * action_span
    """

    def __init__(self):
        super().__init__("rl_policy_node")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("mode", "heuristic")   # heuristic, zero, random
        self.declare_parameter("publish_hz", 20.0)

        self.declare_parameter("goal_pair_index", 2)
        self.declare_parameter("goal_norm_px", 120.0)

        self.declare_parameter("kp", 0.8)
        self.declare_parameter("tilt_kp", 0.03)

        self.declare_parameter("max_action", 0.6)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.mode = self.get_parameter("mode").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.goal_pair_index = int(self.get_parameter("goal_pair_index").value)
        self.goal_norm_px = float(self.get_parameter("goal_norm_px").value)

        self.kp = float(self.get_parameter("kp").value)
        self.tilt_kp = float(self.get_parameter("tilt_kp").value)

        self.max_action = float(self.get_parameter("max_action").value)
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.max_action = max(0.05, min(1.0, self.max_action))
        self.goal_norm_px = max(1.0, self.goal_norm_px)

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

        self.latest_obs: Optional[List[float]] = None

        self.timer = self.create_timer(
            1.0 / self.publish_hz,
            self.tick,
        )

        self.get_logger().info(
            f"RLPolicyNode ready. mode={self.mode}, max_action={self.max_action}"
        )

    def on_observation(self, msg: Float32MultiArray):
        self.latest_obs = list(msg.data)

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def compute_action(self, obs: List[float]):
        """
        Expected /ri/observation from your ri_node is approximately:

            [0] tilt_x
            [1] tilt_y
            [2] marble_x
            [3] marble_y
            [4] marble_valid
            [5] progress_idx
            [6] progress_delta
            [7] off_path
            [8] done
            [9] active
            [10] waiting_for_restart
            [11] episode
            [12] episode_step
            [13...] relative goal pairs:
                   dx0, dy0, dx1, dy1, ...

        This node uses a future relative goal point:
            dx, dy = goal direction from ball to path target
        """

        if self.mode == "zero":
            return 0.0, 0.0

        if self.mode == "random":
            ax = random.uniform(-self.max_action, self.max_action)
            ay = random.uniform(-self.max_action, self.max_action)
            return ax, ay

        # Default: heuristic mode
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

        # Relative goal starts at obs[13]
        rel_start = 13
        pair_i = max(0, self.goal_pair_index)
        idx = rel_start + 2 * pair_i

        if idx + 1 >= len(obs):
            idx = rel_start

        if idx + 1 >= len(obs):
            return 0.0, 0.0

        dx = obs[idx]
        dy = obs[idx + 1]

        # Normalize relative goal direction
        gx = self.clamp(dx / self.goal_norm_px, -1.0, 1.0)
        gy = self.clamp(dy / self.goal_norm_px, -1.0, 1.0)

        # Basic position-mode action
        ax = self.kp * gx
        ay = self.kp * gy

        # Small tilt compensation
        # If board is already tilted, reduce command slightly.
        ax -= self.tilt_kp * tilt_x
        ay -= self.tilt_kp * tilt_y

        if self.invert_x:
            ax *= -1.0
        if self.invert_y:
            ay *= -1.0

        ax = self.clamp(ax, -self.max_action, self.max_action)
        ay = self.clamp(ay, -self.max_action, self.max_action)

        if math.isnan(ax) or math.isinf(ax):
            ax = 0.0
        if math.isnan(ay) or math.isinf(ay):
            ay = 0.0

        return ax, ay

    def tick(self):
        if self.latest_obs is None:
            return

        ax, ay = self.compute_action(self.latest_obs)

        msg = Float32MultiArray()
        msg.data = [float(ax), float(ay)]
        self.pub_action.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Send neutral action before shutdown
    stop_msg = Float32MultiArray()
    stop_msg.data = [0.0, 0.0]
    node.pub_action.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
