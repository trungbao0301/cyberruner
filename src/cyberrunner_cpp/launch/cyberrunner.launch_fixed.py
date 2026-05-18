"""
cyberrunner.launch.py  —  full C++ pipeline
---------------------------------------------
Starts all cyberrunner_cpp nodes in one launch.

Usage:
  ros2 launch cyberrunner_cpp cyberrunner.launch.py
  ros2 launch cyberrunner_cpp cyberrunner.launch.py device_path:=/dev/video2
  ros2 launch cyberrunner_cpp cyberrunner.launch.py show_gui:=false
  ros2 launch cyberrunner_cpp cyberrunner.launch.py fps:=114 loop_hz:=114
  ros2 launch cyberrunner_cpp cyberrunner.launch.py fps:=114 loop_hz:=114 width:=1280 height:=720

After launch:
  ros2 service call /path/draw        std_srvs/srv/Trigger {}  # open path GUI
  ros2 service call /controller/start std_srvs/srv/Trigger {}  # start controller
  ros2 service call /ri/start         std_srvs/srv/Trigger {}  # start RI controller
  ros2 service call /controller/stop  std_srvs/srv/Trigger {}  # stop + level board
  ros2 service call /hiwonder/reset   std_srvs/srv/Trigger {}  # reset servos to home
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            "device_path", default_value="/dev/video2",
            description="V4L2 device path, e.g. /dev/video0. Overrides camera_index."),
        DeclareLaunchArgument(
            "camera_index", default_value="0",
            description="Camera index (used only if device_path is empty)."),
        DeclareLaunchArgument(
            "width", default_value="1920",
            description="Requested camera capture width in pixels."),
        DeclareLaunchArgument(
            "height", default_value="1200",
            description="Requested camera capture height in pixels."),
        DeclareLaunchArgument(
            "fps", default_value="114",
            description="Requested camera capture and publish rate in Hz."),
        DeclareLaunchArgument(
            "loop_hz", default_value="114",
            description="Loop rate in Hz for controller and GUI timers."),
        DeclareLaunchArgument(
            "path_distance_px", default_value="2.0",
            description="Spacing in pixels for generated dense path samples."),
        DeclareLaunchArgument(
            "path_off_path_threshold_px", default_value="90.0",
            description="Max distance in px from generated path before off_path=1."),
        DeclareLaunchArgument(
            "path_goal_threshold_px", default_value="25.0",
            description="Distance in px to final generated point before done=1."),
        DeclareLaunchArgument(
            "rel_path_count", default_value="5",
            description="Number of future relative path points published."),
        DeclareLaunchArgument(
            "rel_path_step", default_value="20",
            description="Generated-path sample step between future relative points."),
        DeclareLaunchArgument(
            "board_width_mm", default_value="280.0",
            description="Playable board width in mm (left-right, horizontal)."),
        DeclareLaunchArgument(
            "board_height_mm", default_value="230.0",
            description="Playable board height in mm (top-bottom, vertical)."),
        DeclareLaunchArgument(
            "show_gui", default_value="true",
            description="Show OpenCV debug windows. Set false on headless machines."),
        DeclareLaunchArgument(
            "tilt_x_offset_deg", default_value="0.0",
            description="Correction added to estimator tilt_x in degrees."),
        DeclareLaunchArgument(
            "tilt_y_offset_deg", default_value="0.0",
            description="Correction added to estimator tilt_y in degrees."),
        DeclareLaunchArgument(
            "launch_tuner", default_value="false",
            description="Launch the C++ OpenCV tuner window."),
        DeclareLaunchArgument(
            "launch_path_progress", default_value="true",
            description="Launch C++ dense-path progress/relative-goal node."),
        DeclareLaunchArgument(
            "launch_ri", default_value="false",
            description="Launch the C++ reinforcement interface node."),
        DeclareLaunchArgument(
            "ri_control_mode", default_value="heuristic",
            description="RI mode: heuristic or external."),
        DeclareLaunchArgument(
            "ri_servo_mode", default_value="absolute",
            description="RI servo mode: absolute or incremental."),
        DeclareLaunchArgument(
            "ri_loop_hz", default_value="30",
            description="RI action/observation loop rate in Hz."),
        DeclareLaunchArgument(
            "ri_goal_pair_index", default_value="4",
            description="Future relative path point used by heuristic RI."),
        DeclareLaunchArgument(
            "ri_goal_norm_px", default_value="140.0",
            description="Pixels that map to full-scale heuristic action."),
        DeclareLaunchArgument(
            "ri_step_delta", default_value="4.0",
            description="Servo counts per RI tick at full-scale action in incremental mode."),
        DeclareLaunchArgument(
            "ri_action_span", default_value="80.0",
            description="Servo counts from home at full-scale action in absolute mode."),
        DeclareLaunchArgument(
            "ri_min_pos", default_value="400",
            description="Minimum servo command allowed by RI."),
        DeclareLaunchArgument(
            "ri_max_pos", default_value="600",
            description="Maximum servo command allowed by RI."),
        DeclareLaunchArgument(
            "ri_cmd_time_ms", default_value="60",
            description="Servo motion duration for each RI command."),
        DeclareLaunchArgument(
            "ri_auto_restart", default_value="true",
            description="Auto-restart RI after marble is detected again."),
        DeclareLaunchArgument(
            "ri_marble_restart_stable_ticks", default_value="15",
            description="RI ticks with valid marble/path before auto-restart."),
        DeclareLaunchArgument(
            "launch_rl", default_value="false",
            description="Launch Python RL policy node that publishes /ri/action_cmd."),
        DeclareLaunchArgument(
            "rl_mode", default_value="heuristic",
            description="RL policy mode: zero, random, heuristic, or model."),
        DeclareLaunchArgument(
            "rl_publish_hz", default_value="20",
            description="RL policy publish rate in Hz."),
        DeclareLaunchArgument(
            "rl_goal_pair_index", default_value="2",
            description="Future relative-goal pair used by RL heuristic."),
        DeclareLaunchArgument(
            "rl_goal_norm_px", default_value="120.0",
            description="Pixel distance that maps to full-scale heuristic action."),
        DeclareLaunchArgument(
            "rl_max_action", default_value="0.45",
            description="Max normalized action published by RL node."),
        DeclareLaunchArgument(
            "rl_action_rate_limit", default_value="0.08",
            description="Max normalized action change per RL tick."),
        DeclareLaunchArgument(
            "rl_model_path", default_value="",
            description="Optional Stable-Baselines3 model path for rl_mode:=model."),

        # ── camera_node ───────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "device_path":    LaunchConfiguration("device_path"),
                "camera_index":   ParameterValue(
                    LaunchConfiguration("camera_index"), value_type=int),
                "width":          ParameterValue(
                    LaunchConfiguration("width"),  value_type=int),
                "height":         ParameterValue(
                    LaunchConfiguration("height"), value_type=int),
                "fps":            ParameterValue(
                    LaunchConfiguration("fps"),    value_type=int),
                "show_preview":   ParameterValue(
                    LaunchConfiguration("show_gui"), value_type=bool),
            }],
        ),

        # ── estimator_node ────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="estimator_node",
            name="estimator_node",
            output="screen",
            parameters=[{
                "board_width_mm":  ParameterValue(
                    LaunchConfiguration("board_width_mm"),  value_type=float),
                "board_height_mm": ParameterValue(
                    LaunchConfiguration("board_height_mm"), value_type=float),
                "loop_hz":         ParameterValue(
                    LaunchConfiguration("loop_hz"), value_type=float),
                "show_window":     ParameterValue(
                    LaunchConfiguration("show_gui"), value_type=bool),
                "tilt_x_offset_deg": ParameterValue(
                    LaunchConfiguration("tilt_x_offset_deg"), value_type=float),
                "tilt_y_offset_deg": ParameterValue(
                    LaunchConfiguration("tilt_y_offset_deg"), value_type=float),
            }],
        ),

        # ── marble_node ───────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="marble_node",
            name="marble_node",
            output="screen",
            parameters=[{
                "show_debug": ParameterValue(
                    LaunchConfiguration("show_gui"), value_type=bool),
            }],
        ),

        # ── path_node ─────────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="path_node",
            name="path_node",
            output="screen",
            parameters=[{
                "loop_hz": ParameterValue(
                    LaunchConfiguration("loop_hz"), value_type=float),
                "path_distance_px": ParameterValue(
                    LaunchConfiguration("path_distance_px"), value_type=float),
            }],
        ),

        # ── path_progress_node ────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="path_progress_node",
            name="path_progress_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_path_progress")),
            parameters=[{
                "path_distance_px": ParameterValue(
                    LaunchConfiguration("path_distance_px"), value_type=float),
                "waypoints_topic": "/path/generated_waypoints",
                "rel_path_count": ParameterValue(
                    LaunchConfiguration("rel_path_count"), value_type=int),
                "rel_path_step": ParameterValue(
                    LaunchConfiguration("rel_path_step"), value_type=int),
                "off_path_threshold_px": ParameterValue(
                    LaunchConfiguration("path_off_path_threshold_px"), value_type=float),
                "goal_threshold_px": ParameterValue(
                    LaunchConfiguration("path_goal_threshold_px"), value_type=float),
            }],
        ),

        # ── ri_node ───────────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="ri_node",
            name="ri_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_ri")),
            parameters=[{
                "loop_hz": ParameterValue(
                    LaunchConfiguration("ri_loop_hz"), value_type=float),
                "control_mode": LaunchConfiguration("ri_control_mode"),
                "servo_mode": LaunchConfiguration("ri_servo_mode"),
                "goal_pair_index": ParameterValue(
                    LaunchConfiguration("ri_goal_pair_index"), value_type=int),
                "goal_norm_px": ParameterValue(
                    LaunchConfiguration("ri_goal_norm_px"), value_type=float),
                "step_delta": ParameterValue(
                    LaunchConfiguration("ri_step_delta"), value_type=float),
                "action_span": ParameterValue(
                    LaunchConfiguration("ri_action_span"), value_type=float),
                "min_pos": ParameterValue(
                    LaunchConfiguration("ri_min_pos"), value_type=int),
                "max_pos": ParameterValue(
                    LaunchConfiguration("ri_max_pos"), value_type=int),
                "cmd_time_ms": ParameterValue(
                    LaunchConfiguration("ri_cmd_time_ms"), value_type=int),
                "auto_restart_on_marble_detected": ParameterValue(
                    LaunchConfiguration("ri_auto_restart"), value_type=bool),
                "marble_restart_stable_ticks": ParameterValue(
                    LaunchConfiguration("ri_marble_restart_stable_ticks"), value_type=int),
            }],
        ),

        # ── controller_node ───────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{
                "loop_hz": ParameterValue(
                    LaunchConfiguration("loop_hz"), value_type=float),
            }],
        ),

        # ── hiwonder_node ─────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="hiwonder_node",
            name="hiwonder_node",
            output="screen",
            parameters=[{
                "servo1_id": 1,
                "servo2_id": 2,
                "home_pos":  500,
            }],
        ),

        # ── rl_policy_node ───────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="rl_policy_node.py",
            name="rl_policy_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_rl")),
            parameters=[{
                "mode": LaunchConfiguration("rl_mode"),
                "publish_hz": ParameterValue(
                    LaunchConfiguration("rl_publish_hz"), value_type=float),
                "goal_pair_index": ParameterValue(
                    LaunchConfiguration("rl_goal_pair_index"), value_type=int),
                "goal_norm_px": ParameterValue(
                    LaunchConfiguration("rl_goal_norm_px"), value_type=float),
                "max_action": ParameterValue(
                    LaunchConfiguration("rl_max_action"), value_type=float),
                "action_rate_limit": ParameterValue(
                    LaunchConfiguration("rl_action_rate_limit"), value_type=float),
                "model_path": LaunchConfiguration("rl_model_path"),
            }],
        ),

        # ── tuner_node ────────────────────────────────────────────────────────
        Node(
            package="cyberrunner_cpp",
            executable="tuner_node",
            name="tuner_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_tuner")),
        ),
    ])
