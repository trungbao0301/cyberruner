#!/usr/bin/env python3
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

import gym
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

from cyberrunner_interfaces.msg import StateEstimateSub
from cyberrunner_dreamer import cyberrunner_layout
from cyberrunner_dreamer.path import LinearPath


class CyberrunnerGymHiwonder(gym.Env):
    """
    Dreamer Gym Env that uses HIWONDER servos instead of Dynamixel.

    Publishes:
      /hiwonder/cmd   std_msgs/Int32MultiArray  [pos1, pos2, time_ms]

    Calls:
      /hiwonder/reset std_srvs/Trigger
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        repeat=1,
        layout=cyberrunner_layout.cyberrunner_hard_layout,
        num_rel_path=5,
        num_wait_steps=30,
        reward_on_fail=0.0,
        reward_on_goal=0.5,
        # -------- HIWONDER mapping params --------
        home_pos=500,          # center position your hiwonder_node uses
        servo_min=0,
        servo_max=1000,
        max_delta=250,         # action=±1 maps to home_pos ± max_delta
        default_time_ms=80,
    ):
        super().__init__()

        if not rclpy.ok():
            rclpy.init()

        self.future = None
        self.repeat = int(repeat)

        # --- Gym spaces ---
        self.observation_space = gym.spaces.Dict(
            image=gym.spaces.Box(0, 255, (64, 64, 1), np.uint8),
            states=gym.spaces.Box(-np.inf, np.inf, (4,), np.float32),
            goal=gym.spaces.Box(-np.inf, np.inf, (num_rel_path * 2,), np.float32),
            progress=gym.spaces.Box(-np.inf, np.inf, (1,), np.float32),
            log_reward=gym.spaces.Box(-np.inf, np.inf, (1,), np.float32),
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)

        self.num_rel_path = int(num_rel_path)
        self.num_wait_steps = int(num_wait_steps)
        self.reward_on_fail = float(reward_on_fail)
        self.reward_on_goal = float(reward_on_goal)

        # Normalization constants (same as your original)
        self.norm_max = np.array([10 * np.pi / 180.0, 10 * np.pi / 180.0, 0.276, 0.231], dtype=np.float32)
        self.goal_norm_max = np.array(
            [0.0002 * 60 * k for k in range(1, self.num_rel_path + 1) for _ in range(2)],
            dtype=np.float32,
        )

        # --- ROS ---
        self.node = Node("cyberrunner_gym_hiwonder")

        self.publisher = self.node.create_publisher(
            Int32MultiArray,
            "/hiwonder/cmd",
            10,
        )

        self.subscription = self.node.create_subscription(
            StateEstimateSub,
            "cyberrunner_state_estimation/estimate_subimg",
            self._msg_to_obs,
            10,
        )

        self.reset_client = self.node.create_client(Trigger, "/hiwonder/reset")
        self.br = CvBridge()

        # --- Path ---
        self.offset = (np.array([0.276, 0.231], dtype=np.float32) / 2.0)

        shared = get_package_share_directory("cyberrunner_dreamer")
        self.p = LinearPath.load(os.path.join(shared, "path_0002_hard.pkl"))

        # reward/termination helpers
        self.cheat = False
        self.cheat_threshold = int(0.057 / self.p.distance)
        self.prev_pos_path = 0
        self.progress = 0
        self.off_path = False

        # episode state
        self.ball_detected = False
        self.new_obs = False
        self.obs = {
            "states": np.zeros((4,), dtype=np.float32),
            "goal": np.zeros((self.num_rel_path * 2,), dtype=np.float32),
            "image": np.zeros((64, 64, 1), dtype=np.uint8),
        }

        self.steps = 0
        self.episodes = 0
        self.accum_reward = 0.0
        self.success = False
        self.last_time = time.time()

        # --- HIWONDER mapping ---
        self.home_pos = int(home_pos)
        self.servo_min = int(servo_min)
        self.servo_max = int(servo_max)
        self.max_delta = int(max_delta)
        self.default_time_ms = int(default_time_ms)

        # Ensure reset service exists (don’t block forever)
        self._wait_for_service(self.reset_client, "/hiwonder/reset", timeout_sec=3.0)

        # go home once at start (best-effort)
        self._reset_board()

    # -------------------- Gym API (NEW) --------------------

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.episodes += 1
        self.steps = 0
        self.accum_reward = 0.0
        self.success = False
        self.ball_detected = False
        self.progress = 0
        self.off_path = False

        # Reset servos to home
        self._reset_board()

        # Send "hold home" once (optional but stabilizes)
        self._send_action(np.zeros((2,), dtype=np.float32))

        # Wait for stable detection (ball detected for N steps)
        count = 0
        obs = self._get_obs()
        while count < self.num_wait_steps:
            obs = self._get_obs()
            count = count + 1 if self.ball_detected else 0

        # set progress reference
        try:
            self.prev_pos_path = int(self.p.closest_point(obs["states"][2:4])[0])
            if self.prev_pos_path < 0:
                self.prev_pos_path = 0
        except Exception:
            self.prev_pos_path = 0

        # normalize outputs for Dreamer
        obs = self._format_obs(obs, reward_value=0.0, done=False)
        return obs, {}

    def step(self, action):
        self.steps += 1

        # Send action to HIWONDER
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)
        self._send_action(action)

        # Get observation
        obs = self._get_obs()

        # Compute reward / done
        reward = self._get_reward(obs)
        done = self._get_done(obs)

        if done and (not self.success):
            reward = self.reward_on_fail
        if self.success:
            reward += self.reward_on_goal

        # Reset board on terminal or timeout
        truncated = bool(self.steps >= 3000)
        terminated = bool(done)

        if terminated or truncated:
            if self.success:
                time.sleep(2.0)
            self._reset_board()

        # Info (Dreamer uses this sometimes)
        info = {"is_terminal": False} if self.success else {}

        # normalize outputs for Dreamer
        obs = self._format_obs(obs, reward_value=reward, done=terminated)

        self.accum_reward += (reward if not terminated else 0.0)

        # (optional) fps debug
        # now = time.time()
        # if (now - self.last_time) > (1.0 / 35.0):
        #     print("Slower than 35fps")
        # self.last_time = now

        return obs, float(reward), terminated, truncated, info

    def render(self):
        return None

    def close(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # -------------------- HIWONDER control --------------------

    def _send_action(self, action: np.ndarray):
        """
        action in [-1, 1] -> servo pos [0..1000] around home_pos
        """
        a1 = float(action[0])
        a2 = float(action[1])

        pos1 = int(round(self.home_pos + a1 * self.max_delta))
        pos2 = int(round(self.home_pos + a2 * self.max_delta))

        pos1 = int(np.clip(pos1, self.servo_min, self.servo_max))
        pos2 = int(np.clip(pos2, self.servo_min, self.servo_max))

        msg = Int32MultiArray()
        msg.data = [pos1, pos2, self.default_time_ms]
        self.publisher.publish(msg)

    def _reset_board(self):
        """
        Calls /hiwonder/reset (Trigger) to go_home() in your hiwonder_node.
        """
        if not self.reset_client.service_is_ready():
            # best-effort
            self._wait_for_service(self.reset_client, "/hiwonder/reset", timeout_sec=1.0)

        if self.reset_client.service_is_ready():
            req = Trigger.Request()
            future = self.reset_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)

    def _wait_for_service(self, client, name, timeout_sec=3.0):
        t0 = time.time()
        while not client.service_is_ready():
            if time.time() - t0 > timeout_sec:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return True

    # -------------------- Observation handling --------------------

    def _get_obs(self):
        # wait until new_obs arrives
        while not self.new_obs:
            for _ in range(self.repeat):
                rclpy.spin_once(self.node, timeout_sec=0.05)
        self.new_obs = False
        return self.obs.copy()

    def _msg_to_obs(self, msg: StateEstimateSub):
        # If ball not detected -> safe zeros (no NaNs)
        if np.isnan(msg.state.x_b):
            self.ball_detected = False
            zeros_img = np.zeros((64, 64, 1), dtype=np.uint8)
            zeros_states = np.zeros((4,), dtype=np.float32)
            zeros_goal = np.zeros((self.num_rel_path * 2,), dtype=np.float32)
            self.obs = {"states": zeros_states, "goal": zeros_goal, "image": zeros_img}
            self.new_obs = True
            return

        self.ball_detected = True

        states = np.array(
            [msg.state.alpha, msg.state.beta, msg.state.x_b, msg.state.y_b],
            dtype=np.float32,
        )
        states[2:] += self.offset

        rel_path = self.p.get_rel_path(states[2:], self.num_rel_path, 60).astype(np.float32).flat

        img = self.br.imgmsg_to_cv2(msg.subimg).mean(axis=-1, keepdims=True)
        img = np.clip(img, 0, 255).astype(np.uint8)

        self.obs = {"states": states, "goal": rel_path, "image": img}
        self.new_obs = True

    def _format_obs(self, obs, reward_value: float, done: bool):
        # Normalize like your original (Dreamer expects this)
        out = dict(obs)

        out["states"] = (out["states"] / self.norm_max).astype(np.float32)
        out["goal"] = (out["goal"] / self.goal_norm_max).astype(np.float32)
        out["progress"] = np.asarray([1 + self.prev_pos_path], dtype=np.float32)
        out["log_reward"] = np.asarray([reward_value if not done else 0.0], dtype=np.float32)

        # Ensure image dtype
        out["image"] = np.asarray(out["image"], dtype=np.uint8)

        return out

    # -------------------- Reward / termination (same logic) --------------------

    def _get_reward(self, obs):
        if not self.ball_detected:
            return 0.0

        curr_pos_path, _ = self.p.closest_point(obs["states"][2:4])
        self.off_path = (curr_pos_path == -1)

        if self.off_path and self.cheat:
            curr_pos_path = self.prev_pos_path

        self.progress = curr_pos_path - self.prev_pos_path
        reward = float(curr_pos_path - self.prev_pos_path) * 0.004 / 16.0
        self.prev_pos_path = curr_pos_path
        return reward

    def _get_done(self, obs):
        # Done if ball not detected
        done = (not self.ball_detected)

        # Done if off path and not cheating
        if (not self.cheat) and self.off_path:
            done = True
            print("[Done]: OFFPATH")

        # Done if reached goal
        if self.p.num_points - self.prev_pos_path <= 1:
            self.success = True
            done = True
            print("[Done]: SUCCESS")
            self._send_action(np.array([0.0, 0.0], dtype=np.float32))

        # Too high progress (kept from your code)
        if (not self.cheat) and (self.progress > self.cheat_threshold):
            done = True
            print("[Done]: Too high progress")

        return bool(done)


# -------------------- Minimal python runner --------------------
if __name__ == "__main__":
    env = CyberrunnerGymHiwonder(
        repeat=1,
        num_rel_path=5,
        home_pos=500,       # match your hiwonder_node param
        max_delta=250,      # tune this
        default_time_ms=80,
    )

    obs, info = env.reset()
    print("Reset OK. obs keys:", obs.keys())

    try:
        while True:
            # random actions for testing
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                print(f"Episode ended. terminated={terminated}, truncated={truncated}, reward={reward}")
                obs, info = env.reset()
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
