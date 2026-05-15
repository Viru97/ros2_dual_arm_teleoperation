"""
servo.launch.py
===============
Launches both MoveIt Servo nodes (left + right) and automatically brings
both arms to a safe ready pose before Servo accepts teleoperation commands.

Timeline
--------
  t=0 s   : Servo nodes start (in paused/unstarted state)
  t=0 s   : ready_pose_node starts, waits 1.5 s then publishes goal
  t=1.5 s : JointTrajectory goals sent → arms begin moving (~3.5 s motion)
  t=5.5 s : Servo start services called → teleop can begin immediately

The auto_start argument (default true) controls whether Servo is started
automatically.  Set auto_start:=false when using MoveIt plan-and-execute
alongside Servo and you want to manage the start call yourself.
"""

import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_ur5e", package_name="dual_arm_moveit_config")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("dual_arm_moveit_config", "config/servo.yaml")

    # ── Launch arguments ─────────────────────────────────────────────────── #
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",   # Changed from false → true; ready_pose_node
        description=(           # handles the delay so it's safe.
            "Automatically start Servo after the ready pose completes. "
            "Set false only when manually managing Servo start."
        ),
    )

    # ── Per-arm Servo configs ─────────────────────────────────────────────── #
    left_servo_yaml = copy.deepcopy(servo_yaml)
    left_servo_yaml.update({
        "move_group_name": "left_arm",
        "ee_frame_name": "left_tool0",
        "robot_link_command_frame": "left_base_link",
        "command_out_topic": "/left_arm_controller/joint_trajectory",
    })

    right_servo_yaml = copy.deepcopy(servo_yaml)
    right_servo_yaml.update({
        "move_group_name": "right_arm",
        "ee_frame_name": "right_tool0",
        "robot_link_command_frame": "right_base_link",
        "command_out_topic": "/right_arm_controller/joint_trajectory",
    })

    # ── Servo nodes ──────────────────────────────────────────────────────── #
    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_servo_node",
        parameters=[
            {"moveit_servo": left_servo_yaml},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_servo_node",
        parameters=[
            {"moveit_servo": right_servo_yaml},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # ── Ready pose node ───────────────────────────────────────────────────── #
    # Drives both arms to a safe non-singular configuration via direct
    # JointTrajectory commands.  Runs once then exits.
    ready_pose_node = Node(
        package="dual_arm_teleop",
        executable="ready_pose_node",
        name="ready_pose_node",
        output="screen",
    )

    # ── Servo start services ──────────────────────────────────────────────── #
    # Called AFTER the ready pose has had time to complete.
    #   ready_pose_node internal delay: 1.5 s
    #   motion duration:                3.5 s
    #   safety buffer:                  0.5 s
    #   Total timer delay below:        5.5 s
    SERVO_START_DELAY = 5.5

    start_left_servo = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/left_servo_node/start_servo",
            "std_srvs/srv/Trigger", "{}",
        ],
        output="screen",
    )
    start_right_servo = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/right_servo_node/start_servo",
            "std_srvs/srv/Trigger", "{}",
        ],
        output="screen",
    )

    return LaunchDescription([
        auto_start_arg,

        # Start Servo nodes immediately (they sit idle until start_servo is called)
        left_servo_node,
        right_servo_node,

        # Bring arms to ready pose right away
        ready_pose_node,

        # Start Servo only after the ready pose motion has completed
        TimerAction(
            period=SERVO_START_DELAY,
            actions=[start_left_servo, start_right_servo],
            condition=IfCondition(LaunchConfiguration("auto_start")),
        ),
    ])