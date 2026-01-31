from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Optional: ignore ~/.local Python packages to avoid numpy/cv2 conflicts
    py_no_user_site = LaunchConfiguration("py_no_user_site")

    return LaunchDescription([
        DeclareLaunchArgument(
            "py_no_user_site",
            default_value="0",
            description="Set to 1 to ignore ~/.local site-packages (helps avoid numpy/cv2 conflicts).",
        ),

        # If enabled, set env var for all nodes in this launch
        SetEnvironmentVariable(
            name="PYTHONNOUSERSITE",
            value=py_no_user_site,
        ),

        # -------------------------
        # Camera publisher
        # -------------------------
        Node(
            package="cyberrunner_camera",
            executable="cam_publisher.py",   # if your package installs it as a script/executable
            name="cam_publisher",
            output="screen",
        ),

        # -------------------------
        # State estimation
        # -------------------------
        Node(
            package="cyberrunner_state_estimation",
            executable="estimator_sub",
            name="state_estimator",
            output="screen",
        ),

        # -------------------------
        # Hiwonder motor driver
        # -------------------------
        Node(
            package="cyberrunner_hiwonder",
            executable="hiwonder_node",
            name="hiwonder_node",
            output="screen",
            parameters=[{
                "servo1_id": 1,
                "servo2_id": 2,
                "home_pos": 500,
            }],
        ),
    ])
