import sys
import os
import time

import gym
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from cyberrunner_dreamer import cyberrunner_layout
from cyberrunner_dreamer.path import LinearPath
from cyberrunner_interfaces.msg import StateEstimateSub



import numpy as np

def imgmsg_to_numpy(msg):
    import numpy as np

    enc = (msg.encoding or "").lower()
    h, w = msg.height, msg.width

    # 8-bit, 3 channel
    if enc in ("bgr8", "rgb8", "8uc3"):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)

    # 8-bit, 1 channel
    if enc in ("mono8", "8uc1"):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 1)

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


    data = np.frombuffer(msg.data, dtype=np.uint8)
    row_stride = int(msg.step)
    needed = w * c

    if row_stride == needed:
        img = data.reshape((h, w, c))
    else:
        img = data.reshape((h, row_stride))[:, :needed].reshape((h, w, c))

    if c == 1:
        img = img[..., 0]
    return img

class CyberrunnerGym(gym.Env):
    """
    Hiwonder version:
      - Publishes Int32MultiArray to /hiwonder/cmd: [pos1, pos2, time_ms]
      - Calls Trigger service /hiwonder/reset to go home
      - Converts Dreamer action in [-1,1]^2 to servo position increments each step
    """

    def __init__(
        self,
        repeat=1,
        layout=cyberrunner_layout.cyberrunner_hard_layout,
        num_rel_path=5,
        num_wait_steps=30,
        reward_on_fail=0.0,
        reward_on_goal=0.5,
        # ---- Hiwonder control params ----
        home_pos=500,          # 0..1000
        min_pos=0,
        max_pos=1000,
        step_delta=35,         # counts per env step at |action|=1.0 (tune this)
        cmd_time_ms=40,        # servo move time per step (tune this)
        reset_time_ms=600,     # used by hiwonder_node internally; here we just call reset service
    ):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()

        self.future = None
        self.cheat = False

        self.observation_space = gym.spaces.Dict(
            image=gym.spaces.Box(0, 255, (64, 64, 1), np.uint8),
            states=gym.spaces.Box(-np.inf, np.inf, (4,), np.float32),
            goal=gym.spaces.Box(-np.inf, np.inf, (num_rel_path * 2,), np.float32),
            progress=gym.spaces.Box(-np.inf, np.inf, (1,), np.float32),
            log_reward=gym.spaces.Box(-np.inf, np.inf, (1,), np.float32),
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,))
        self.obs = dict(self.observation_space.sample())

        self.num_rel_path = num_rel_path
        self.norm_max = np.array([10 * np.pi / 180.0, 10 * np.pi / 180.0, 0.276, 0.231])
        self.goal_norm_max = np.array(
            [0.0002 * 60 * k for k in range(1, self.num_rel_path + 1) for _ in range(2)]
        )

        # ROS node
        self.node = Node("cyberrunner_gym")

        # Publish to Hiwonder topic (hiwonder_node subscribes to this)
        self.hiwonder_pub = self.node.create_publisher(Int32MultiArray, "/hiwonder/cmd", 10)

        # Call reset service (hiwonder_node provides this)
        self.reset_client = self.node.create_client(Trigger, "/hiwonder/reset")

        # Subscribe to state estimation (same as before)
        self.subscription = self.node.create_subscription(
            StateEstimateSub,
            "cyberrunner_state_estimation/estimate_subimg",
            self._msg_to_obs,
            1,
        )
        self.repeat = repeat

        # Path setup
        self.offset = np.array([0.276, 0.231]) / 2.0
        shared = get_package_share_directory("cyberrunner_dreamer")
        self.p = LinearPath.load(os.path.join(shared, "path_0002_hard.pkl"))

        self.cheat_threshold = int(0.057 / self.p.distance)
        self.prev_pos_path = 0
        self.num_wait_steps = num_wait_steps
        self.reward_on_fail = reward_on_fail
        self.reward_on_goal = reward_on_goal

        self.ball_detected = False
        self.last_time = 0
        self.progress = 0
        self.accum_reward = 0.0
        self.steps = 0
        self.off_path = False
        self.episodes = 0
        self.new_obs = False
        self.success = False

        # ---- Hiwonder control state ----
        self.home_pos = int(home_pos)
        self.min_pos = int(min_pos)
        self.max_pos = int(max_pos)
        self.step_delta = float(step_delta)
        self.cmd_time_ms = int(cmd_time_ms)
        self.reset_time_ms = int(reset_time_ms)

        # Track current commanded positions (start at home)
        self.pos1 = self.home_pos
        self.pos2 = self.home_pos

        # Try to reset board once on startup (non-blocking wait)
        self._try_reset_board()

    def step(self, action):
        self.steps += 1

        # Send action to hiwonder
        if self.cheat and (self.p.num_points - self.prev_pos_path <= 200):
            action = np.array(action, dtype=np.float32).copy()
            action[1] = 1.0
        self._send_action(action)

        # Get observation
        obs = self._get_obs()

        # Compute reward
        reward = self._get_reward(obs)

        # Done logic
        done = self._get_done(obs)
        if done and (not self.success):
            reward = self.reward_on_fail
        if self.success:
            reward += self.reward_on_goal

        if done or self.steps == 3000:
            if self.success:
                time.sleep(2)
            print("Reset board")
            self._reset_board()

        info = {"is_terminal": False} if self.success else {}

        now = time.time()
        if (now - self.last_time) > (1.0 / 35.0):
            print("Slower than 35fps")
        self.last_time = now

        self.accum_reward += reward if not done else 0

        # Normalize outputs for Dreamer
        obs["states"] = (obs["states"] / self.norm_max).astype(np.float32)
        obs["goal"] = (obs["goal"] / self.goal_norm_max).astype(np.float32)
        obs["progress"] = np.asarray([1 + self.prev_pos_path], dtype=np.float32)
        obs["log_reward"] = np.asarray([reward if not done else 0], dtype=np.float32)
        return obs, reward, done, info

    def reset(self):
        print("Resetting ...")
        self.episodes += 1
        print("Previous reward: {}".format(self.accum_reward))
        print("Previous episode length: {}".format(self.steps / 60.0))
        print("Episodes: {}".format(self.episodes))

        self.accum_reward = 0.0
        self.steps = 0
        self.success = False
        self.ball_detected = False

        # Wait for reset service call to complete (if any)
        if self.future is not None:
            rclpy.spin_until_future_complete(self.node, self.future, timeout_sec=10)

        # Stop motion and go to home pos logically
        self.pos1 = self.home_pos
        self.pos2 = self.home_pos
        self._publish_positions(self.pos1, self.pos2, self.cmd_time_ms)

        # Wait until ball detected stable
        count = 0
        obs = self._get_obs()
        while count < self.num_wait_steps:
            obs = self._get_obs()
            count = count + 1 if self.ball_detected else 0

        self.prev_pos_path = self.p.closest_point(obs["states"][2:4])[0]
        self.progress = 0
        self.last_time = time.time()

        obs["states"] = (obs["states"] / self.norm_max).astype(np.float32)
        obs["goal"] = (obs["goal"] / self.goal_norm_max).astype(np.float32)
        obs["progress"] = np.asarray([1 + self.prev_pos_path], dtype=np.float32)
        obs["log_reward"] = np.asarray([0], dtype=np.float32)
        return obs

    def render(self, mode="human"):
        pass

    # ---------------- Hiwonder control ----------------

    def _publish_positions(self, p1, p2, time_ms):
        msg = Int32MultiArray()
        msg.data = [int(p1), int(p2), int(time_ms)]
        self.hiwonder_pub.publish(msg)

    def _send_action(self, action):
        # action in [-1,1]
        a = np.array(action, dtype=np.float32).copy()
        a = np.clip(a, -1.0, 1.0)

        # Convert to incremental position change
        # NOTE: sign may need flipping depending on your physical axes
        d1 = int(round(a[0] * self.step_delta))
        d2 = int(round(a[1] * self.step_delta))

        self.pos1 = int(np.clip(self.pos1 + d1, self.min_pos, self.max_pos))
        self.pos2 = int(np.clip(self.pos2 + d2, self.min_pos, self.max_pos))

        self._publish_positions(self.pos1, self.pos2, self.cmd_time_ms)

    def _try_reset_board(self):
        # Non-blocking: only call if service is up
        if not self.reset_client.wait_for_service(timeout_sec=0.2):
            self.node.get_logger().warn("/hiwonder/reset not available yet")
            return
        req = Trigger.Request()
        self.future = self.reset_client.call_async(req)

    def _reset_board(self):
        # Call /hiwonder/reset (go_home in hiwonder_node)
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("/hiwonder/reset service not available")
            return
        req = Trigger.Request()
        self.future = self.reset_client.call_async(req)

        # Also reset internal commanded positions
        self.pos1 = self.home_pos
        self.pos2 = self.home_pos

    # ---------------- Observations / reward ----------------

    def _get_obs(self):
        while not self.new_obs:
            for _ in range(self.repeat):
                rclpy.spin_once(self.node, timeout_sec=0.05)
        self.new_obs = False
        return self.obs.copy()

    def _get_reward(self, obs):
        if not self.ball_detected:
            reward = 0.0
        else:
            curr_pos_path, _p = self.p.closest_point(obs["states"][2:4])
            self.off_path = curr_pos_path == -1
            if self.off_path and self.cheat:
                curr_pos_path = self.prev_pos_path
            self.progress = curr_pos_path - self.prev_pos_path
            reward = float(curr_pos_path - self.prev_pos_path) * 0.004 / 16.0
            self.prev_pos_path = curr_pos_path
        return reward

    def _get_done(self, obs):
        done = not self.ball_detected

        if (not self.cheat) and self.off_path:
            done = True
            print("[Done]: OFFPATH")

        if self.p.num_points - self.prev_pos_path <= 1:
            self.success = True
            done = True
            print("[Done]: SUCCESS")
            # stop motion
            self._publish_positions(self.pos1, self.pos2, self.cmd_time_ms)

        if (not self.cheat) and self.progress > self.cheat_threshold:
            done = True
            print("[Done]: Too high progress")

        return done

    def _msg_to_obs(self, msg):
        if np.isnan(msg.state.x_b):
            self.ball_detected = False
        else:
            self.ball_detected = True
            states = np.array([msg.state.alpha, msg.state.beta, msg.state.x_b, msg.state.y_b])
            states[2:] += self.offset
            rel_path = self.p.get_rel_path(states[2:], self.num_rel_path, 60).flat
            img = imgmsg_to_numpy(msg.subimg).mean(axis=-1, keepdims=True)
            self.obs = {"states": states, "goal": rel_path, "image": img}
        self.new_obs = True

