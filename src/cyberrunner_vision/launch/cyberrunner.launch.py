"""
cyberrunner.launch.py  —  full pipeline
-----------------------------------------
Starts all cyberrunner_vision nodes + hiwonder_node.

Usage:
  ros2 launch cyberrunner_vision cyberrunner.launch.py
  ros2 launch cyberrunner_vision cyberrunner.launch.py device_path:=/dev/video2
  ros2 launch cyberrunner_vision cyberrunner.launch.py show_gui:=false
  ros2 launch cyberrunner_vision cyberrunner.launch.py fps:=114 loop_hz:=114
  ros2 launch cyberrunner_vision cyberrunner.launch.py fps:=114 loop_hz:=114 width:=1280 height:=720

After launch:
  ros2 service call /path/draw       std_srvs/srv/Trigger {}   # open path GUI
  ros2 service call /controller/start std_srvs/srv/Trigger {}  # start controller
  ros2 service call /controller/stop  std_srvs/srv/Trigger {}  # stop + level board

Optional (separate terminal):
  ros2 run cyberrunner_vision tuner_node         # live parameter sliders
  ros2 run cyberrunner_vision calibration_node   # Kalman tuning tests
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────────
        # Camera: use device_path (e.g. /dev/video0) OR camera_index (int)
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
            description="Requested loop rate in Hz for controller and camera-driven GUI timers."),

        # Board physical dimensions (mm) — inside the 4 blue calibration dots
        DeclareLaunchArgument(
            "board_width_mm", default_value="280.0",
            description="Playable board width in mm (left-right, horizontal)."),
        DeclareLaunchArgument(
            "board_height_mm", default_value="230.0",
            description="Playable board height in mm (top-bottom, vertical)."),

        # GUI: set false on headless machines (no display)
        DeclareLaunchArgument(
            "show_gui", default_value="true",
            description="Show OpenCV/Tkinter debug windows. Set false if headless."),

        # ── camera_node ───────────────────────────────────────────────────────
        # Reads from fisheye camera, rectifies with OCamCalib model, publishes
        # /camera/rectified. Requires GStreamer (gstreamer1.0-plugins-good/bad).
        Node(
            package="cyberrunner_vision",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "device_path":    LaunchConfiguration("device_path"),
                "camera_index":   LaunchConfiguration("camera_index"),
                "width":          LaunchConfiguration("width"),
                "height":         LaunchConfiguration("height"),
                "fps":            LaunchConfiguration("fps"),
                "show_preview":   LaunchConfiguration("show_gui"),
            }],
        ),

        # ── estimator_node ────────────────────────────────────────────────────
        # Consumes /camera/rectified.
        # Click 4 stable corners then 4 moving corners in the GUI window.
        # Publishes /camera/topdown and /estimator/state.
        # Config auto-saved to ~/cyberrunner_config.json after pressing 's'.
        Node(
            package="cyberrunner_vision",
            executable="estimator_node",
            name="estimator_node",
            output="screen",
            parameters=[{
                "board_width_mm":  LaunchConfiguration("board_width_mm"),
                "board_height_mm": LaunchConfiguration("board_height_mm"),
                "loop_hz":         ParameterValue(LaunchConfiguration("loop_hz"), value_type=float),
                "show_window":     LaunchConfiguration("show_gui"),
            }],
        ),

        # ── marble_node ───────────────────────────────────────────────────────
        # Consumes /camera/topdown.
        # Detects green marble via HSV + Hough circles with EMA smoothing.
        # Publishes /marble/position (Point: x,y=topdown px, z=radius/-1=lost).
        Node(
            package="cyberrunner_vision",
            executable="marble_node",
            name="marble_node",
            output="screen",
            parameters=[{
                "show_debug": LaunchConfiguration("show_gui"),
            }],
        ),

        # ── path_node ─────────────────────────────────────────────────────────
        # Consumes /camera/topdown.
        # Trigger /path/draw to open click-to-add-waypoint GUI.
        # Publishes /path/waypoints. Auto-loads ~/cyberrunner_path.json on start.
        Node(
            package="cyberrunner_vision",
            executable="path_node",
            name="path_node",
            output="screen",
            parameters=[{
                "loop_hz": ParameterValue(LaunchConfiguration("loop_hz"), value_type=float),
            }],
        ),

        # ── controller_node ───────────────────────────────────────────────────
        # Consumes /marble/position + /path/waypoints + /estimator/state.
        # PD + Kalman filter, segment-following lookahead.
        # Publishes /hiwonder/cmd.
        # Trigger /controller/start to begin, /controller/stop to level board.
        Node(
            package="cyberrunner_vision",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{
                "loop_hz": ParameterValue(LaunchConfiguration("loop_hz"), value_type=float),
            }],
        ),

        # ── hiwonder_node ─────────────────────────────────────────────────────
        # Consumes /hiwonder/cmd (Int32MultiArray: [pos1, pos2, time_ms]).
        # Drives two servos over USB HID (VID=0x0483, PID=0x5750).
        # Requires: pip install hidapi
        #           sudo apt install libhidapi-hidraw0
        #           udev rule — see hand_tests.txt or README
        Node(
            package="cyberrunner_hiwonder",
            executable="hiwonder_node",
            name="hiwonder_node",
            output="screen",
            parameters=[{
                "servo1_id": 1,
                "servo2_id": 2,
                "home_pos":  500,
            }],
        ),

    ])
