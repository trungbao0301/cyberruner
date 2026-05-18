from datetime import datetime

import rclpy
from dreamerv3.train import main as train


def main(args=None):
    rclpy.init(args=args)
    date_str = datetime.now().strftime("%Y%m%d-%H%M%S")
    argv = [
        "--configs",
        "cyberrunner",
        "large",
        "--task",
        "gym_cyberrunner_dreamer:cyberrunner-pixel-ros-v0",
        "--logdir",
        "~/cyberrunner_logs/pixel_" + date_str,
        "--replay_size",
        "1e6",
        "--run.script",
        "train",
        "--run.train_ratio",
        "128",
        "--run.save_every",
        "20",
        "--run.log_every",
        "10",
        "--jax.policy_devices",
        "0",
        "--jax.train_devices",
        "0",
    ]
    train(argv)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
