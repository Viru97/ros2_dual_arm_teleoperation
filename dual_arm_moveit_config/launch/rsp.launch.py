from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dual_ur5e", package_name="dual_arm_moveit_config").to_moveit_configs()

    publish_frequency_arg = DeclareLaunchArgument(
        "publish_frequency",
        default_value="100.0",
        description="Maximum robot_state_publisher TF publish frequency in Hz.",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
        ],
    )

    return LaunchDescription([
        publish_frequency_arg,
        robot_state_publisher,
    ])
