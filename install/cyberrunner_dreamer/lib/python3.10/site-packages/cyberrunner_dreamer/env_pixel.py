import time

import cv2
import gym
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

from cyberrunner_dreamer.path import LinearPath


class CyberrunnerPixelGym(gym.Env):
    """Dreamer environment for the current C++ pixel-space ROS stack."""

    def __init__(
        self,
        repeat=1,
        num_rel_path=5,
        num_wait_steps=20,
        path_distance_px=2.0,
        rel_path_step=20,
        board_width_px=1000.0,
        board_height_px=1000.0,
        home_pos=500,
        min_pos=350,
        max_pos=650,
        step_delta=8,
        cmd_time_ms=40,
        max_episode_steps=3000,
        reward_scale=0.01,
        reward_on_fail=-1.0,
        reward_on_goal=5.0,
        goal_threshold_px=25.0,
    ):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()

        self.repeat = int(repeat)
        self.num_rel_path = int(num_rel_path)
        self.num_wait_steps = int(num_wait_steps)
        self.path_distance_px = float(path_distance_px)
        self.rel_path_step = int(rel_path_step)
        self.board_width_px = float(board_width_px)
        self.board_height_px = float(board_height_px)
        self.max_episode_steps = int(max_episode_steps)
        self.reward_scale = float(reward_scale)
        self.reward_on_fail = float(reward_on_fail)
        self.reward_on_goal = float(reward_on_goal)
        self.goal_threshold_px = float(goal_threshold_px)

        self.home_pos = int(home_pos)
        self.min_pos = int(min_pos)
        self.max_pos = int(max_pos)
        self.step_delta = float(step_delta)
        self.cmd_time_ms = int(cmd_time_ms)
        self.pos1 = self.home_pos
        self.pos2 = self.home_pos

        self.observation_space = gym.spaces.Dict(
            image=gym.spaces.Box(0, 255, (64, 64, 1), np.uint8),
            states=gym.spaces.Box(-np.inf, np.inf, (4,), np.float32),
            goal=gym.spaces.Box(-np.inf, np.inf, (self.num_rel_path * 2,), np.float32),
            progress=gym.spaces.Box(-np.inf, np.inf, (1,), np.float32),
            log_reward=gym.spaces.Box(-np.inf, np.inf, (1,), np.float32),
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)

        self.norm_max = np.array(
            [10.0, 10.0, self.board_width_px, self.board_height_px], dtype=np.float32
        )
        self.goal_norm_max = np.array(
            [self.path_distance_px * self.rel_path_step * k
             for k in range(1, self.num_rel_path + 1)
             for _ in range(2)],
            dtype=np.float32,
        )

        self.node = Node("cyberrunner_pixel_gym")
        self.bridge = CvBridge()
        self.hiwonder_pub = self.node.create_publisher(Int32MultiArray, "/hiwonder/cmd", 10)
        self.reset_client = self.node.create_client(Trigger, "/hiwonder/reset")

        self.sub_marble = self.node.create_subscription(
            Point, "/marble/position", self._on_marble, 2
        )
        self.sub_waypoints = self.node.create_subscription(
            Float32MultiArray, "/path/waypoints", self._on_waypoints, 2
        )
        self.sub_state = self.node.create_subscription(
            Float32MultiArray, "/estimator/state", self._on_estimator_state, 2
        )
        self.sub_image = self.node.create_subscription(
            Image, "/camera/topdown", self._on_image, 2
        )

        self.path = None
        self.prev_pos_path = 0
        self.progress = 0
        self.steps = 0
        self.episodes = 0
        self.accum_reward = 0.0
        self.ball_detected = False
        self.off_path = False
        self.success = False
        self.new_obs = False
        self.future = None

        self.marble_xy = np.zeros(2, dtype=np.float32)
        self.tilt_xy = np.zeros(2, dtype=np.float32)
        self.image = np.zeros((64, 64, 1), dtype=np.uint8)
        self.obs = {
            "image": self.image.copy(),
            "states": np.zeros(4, dtype=np.float32),
            "goal": np.zeros(self.num_rel_path * 2, dtype=np.float32),
            "progress": np.zeros(1, dtype=np.float32),
            "log_reward": np.zeros(1, dtype=np.float32),
        }

        self._try_reset_board()

    def step(self, action):
        self.steps += 1
        self._send_action(action)
        obs = self._get_obs()

        reward = self._get_reward()
        done = self._get_done()

        if done and not self.success:
            reward = self.reward_on_fail
        if self.success:
            reward += self.reward_on_goal

        if done or self.steps >= self.max_episode_steps:
            self._reset_board()

        self.accum_reward += reward if not done else 0.0
        return self._normalize_obs(obs, reward), reward, done, {}

    def reset(self):
        print("Resetting pixel Dreamer env ...")
        self.episodes += 1
        print("Previous reward: {}".format(self.accum_reward))
        print("Previous episode length: {}".format(self.steps))
        print("Episodes: {}".format(self.episodes))

        self.steps = 0
        self.accum_reward = 0.0
        self.progress = 0
        self.success = False
        self.off_path = False
        self.pos1 = self.home_pos
        self.pos2 = self.home_pos
        self._publish_positions(self.pos1, self.pos2, self.cmd_time_ms)

        if self.future is not None:
            rclpy.spin_until_future_complete(self.node, self.future, timeout_sec=5.0)

        count = 0
        obs = self._get_obs()
        while count < self.num_wait_steps:
            obs = self._get_obs()
            count = count + 1 if self.ball_detected and self.path is not None else 0

        if self.path is not None:
            idx, _ = self.path.closest_point(self.marble_xy)
            self.prev_pos_path = max(0, idx)
        else:
            self.prev_pos_path = 0

        return self._normalize_obs(obs, 0.0)

    def render(self, mode="human"):
        return None

    def close(self):
        self.node.destroy_node()

    def _on_marble(self, msg):
        if msg.z < 0.0:
            self.ball_detected = False
        else:
            self.ball_detected = True
            self.marble_xy = np.array([msg.x, msg.y], dtype=np.float32)
        self._update_obs()

    def _on_waypoints(self, msg):
        data = np.asarray(msg.data, dtype=np.float32)
        if data.size < 4:
            self.path = None
            return
        waypoints = data[: data.size - (data.size % 2)].reshape(-1, 2)
        self.path = LinearPath(
            waypoints,
            distance=self.path_distance_px,
            board_width=self.board_width_px,
            board_height=self.board_height_px,
        )
        idx, _ = self.path.closest_point(self.marble_xy)
        self.prev_pos_path = max(0, idx)
        print("Loaded pixel path with {} samples".format(self.path.num_points))

    def _on_estimator_state(self, msg):
        if len(msg.data) > 4:
            self.tilt_xy = np.array([msg.data[3], msg.data[4]], dtype=np.float32)
        self._update_obs()

    def _on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (64, 64), interpolation=cv2.INTER_AREA)
        self.image = small[..., None].astype(np.uint8)
        self._update_obs()

    def _update_obs(self):
        states = np.array(
            [self.tilt_xy[0], self.tilt_xy[1], self.marble_xy[0], self.marble_xy[1]],
            dtype=np.float32,
        )
        if self.path is None or not self.ball_detected:
            goal = np.zeros(self.num_rel_path * 2, dtype=np.float32)
        else:
            goal = self.path.get_rel_path(
                self.marble_xy, self.num_rel_path, self.rel_path_step
            ).astype(np.float32).flat

        self.obs = {
            "image": self.image.copy(),
            "states": states,
            "goal": np.asarray(goal, dtype=np.float32),
            "progress": np.asarray([1 + self.prev_pos_path], dtype=np.float32),
            "log_reward": np.zeros(1, dtype=np.float32),
        }
        self.new_obs = True

    def _get_obs(self):
        while not self.new_obs:
            for _ in range(max(1, self.repeat)):
                rclpy.spin_once(self.node, timeout_sec=0.05)
        self.new_obs = False
        return self.obs.copy()

    def _normalize_obs(self, obs, reward):
        out = obs.copy()
        out["states"] = (out["states"] / self.norm_max).astype(np.float32)
        out["goal"] = (out["goal"] / self.goal_norm_max).astype(np.float32)
        out["progress"] = np.asarray([1 + self.prev_pos_path], dtype=np.float32)
        out["log_reward"] = np.asarray([reward], dtype=np.float32)
        return out

    def _get_reward(self):
        if not self.ball_detected or self.path is None:
            return 0.0

        curr_idx, _ = self.path.closest_point(self.marble_xy)
        self.off_path = curr_idx == -1
        if self.off_path:
            return self.reward_on_fail

        self.progress = curr_idx - self.prev_pos_path
        self.prev_pos_path = curr_idx
        return float(self.progress) * self.reward_scale

    def _get_done(self):
        if not self.ball_detected:
            print("[Done]: marble lost")
            return True
        if self.path is None:
            return False
        if self.off_path:
            print("[Done]: off path")
            return True
        if self.prev_pos_path >= self.path.num_points - 1:
            self.success = True
            print("[Done]: success")
            return True
        if np.linalg.norm(self.path.points[-1] - self.marble_xy) < self.goal_threshold_px:
            self.success = True
            print("[Done]: reached final waypoint")
            return True
        return False

    def _publish_positions(self, p1, p2, time_ms):
        msg = Int32MultiArray()
        msg.data = [int(p1), int(p2), int(time_ms)]
        self.hiwonder_pub.publish(msg)

    def _send_action(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        self.pos1 = int(np.clip(self.pos1 + round(a[0] * self.step_delta),
                                self.min_pos, self.max_pos))
        self.pos2 = int(np.clip(self.pos2 + round(a[1] * self.step_delta),
                                self.min_pos, self.max_pos))
        self._publish_positions(self.pos1, self.pos2, self.cmd_time_ms)

    def _try_reset_board(self):
        if not self.reset_client.wait_for_service(timeout_sec=0.2):
            self.node.get_logger().warn("/hiwonder/reset not available yet")
            return
        self.future = self.reset_client.call_async(Trigger.Request())

    def _reset_board(self):
        self.pos1 = self.home_pos
        self.pos2 = self.home_pos
        self._publish_positions(self.pos1, self.pos2, self.cmd_time_ms)
        if self.reset_client.wait_for_service(timeout_sec=0.5):
            self.future = self.reset_client.call_async(Trigger.Request())
        time.sleep(0.2)
