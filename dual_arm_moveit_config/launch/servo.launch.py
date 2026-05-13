from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
import os
import copy
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dual_ur5e", package_name="dual_arm_moveit_config").to_moveit_configs()

    servo_yaml = load_yaml("dual_arm_moveit_config", "config/servo.yaml")

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

    start_left_servo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/left_servo_node/start_servo", "std_srvs/srv/Trigger", "{}"],
        output="screen",
    )
    start_right_servo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/right_servo_node/start_servo", "std_srvs/srv/Trigger", "{}"],
        output="screen",
    )

    return LaunchDescription([
        left_servo_node,
        right_servo_node,
        TimerAction(period=3.0, actions=[start_left_servo, start_right_servo]),
    ])
