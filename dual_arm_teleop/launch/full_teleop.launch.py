import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name, relative_path, launch_arguments=None):
    package_share = get_package_share_directory(package_name)
    launch_path = os.path.join(package_share, relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    arm_arg = DeclareLaunchArgument(
        "arm",
        default_value="both",
        description="Arm to control with hands: left, right, or both.",
    )
    camera_index_arg = DeclareLaunchArgument(
        "camera_index",
        default_value="0",
        description="OpenCV camera index.",
    )
    show_debug_image_arg = DeclareLaunchArgument(
        "show_debug_image",
        default_value="true",
        description="Show OpenCV hand tracking window.",
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        "max_linear_speed",
        default_value="0.35",
        description="Maximum Cartesian Servo speed in m/s.",
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        "max_angular_speed",
        default_value="1.0",
        description="Maximum yaw Servo speed in rad/s.",
    )
    deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.04",
        description="Normalised deadzone around image centre.",
    )
    motion_full_scale_arg = DeclareLaunchArgument(
        "motion_full_scale",
        default_value="0.30",
        description="Normalised palm displacement that maps to max speed.",
    )
    filter_alpha_arg = DeclareLaunchArgument(
        "filter_alpha",
        default_value="0.45",
        description="EMA smoothing factor for palm pose and roll.",
    )
    no_hand_pause_timeout_arg = DeclareLaunchArgument(
        "no_hand_pause_timeout",
        default_value="0.4",
        description="Seconds without a detected hand before pausing Servo.",
    )
    ramp_rate_arg = DeclareLaunchArgument(
        "ramp_rate",
        default_value="0.12",
        description="Max fractional velocity change per control tick.",
    )
    servo_status_log_period_arg = DeclareLaunchArgument(
        "servo_status_log_period",
        default_value="2.0",
        description="Seconds between repeated Servo warning logs.",
    )
    start_gripper_bridge_arg = DeclareLaunchArgument(
        "start_gripper_bridge",
        default_value="true",
        description="Start gripper command bridge for hand pinch control.",
    )
    servo_auto_start_arg = DeclareLaunchArgument(
        "servo_auto_start",
        default_value="false",
        description=(
            "If true, servo.launch.py also sends the ready pose and starts Servo. "
            "Default false lets hand_teleop start Servo when a hand is detected."
        ),
    )

    demo_launch = include_launch(
        "dual_arm_moveit_config",
        "launch/demo.launch.py",
    )

    servo_launch = include_launch(
        "dual_arm_moveit_config",
        "launch/servo.launch.py",
        {"auto_start": LaunchConfiguration("servo_auto_start")},
    )

    hand_teleop_launch = include_launch(
        "dual_arm_teleop",
        "launch/hand_teleop.launch.py",
        {
            "arm": LaunchConfiguration("arm"),
            "camera_index": LaunchConfiguration("camera_index"),
            "show_debug_image": LaunchConfiguration("show_debug_image"),
            "max_linear_speed": LaunchConfiguration("max_linear_speed"),
            "max_angular_speed": LaunchConfiguration("max_angular_speed"),
            "deadzone": LaunchConfiguration("deadzone"),
            "motion_full_scale": LaunchConfiguration("motion_full_scale"),
            "filter_alpha": LaunchConfiguration("filter_alpha"),
            "no_hand_pause_timeout": LaunchConfiguration("no_hand_pause_timeout"),
            "ramp_rate": LaunchConfiguration("ramp_rate"),
            "servo_status_log_period": LaunchConfiguration("servo_status_log_period"),
            "start_gripper_bridge": LaunchConfiguration("start_gripper_bridge"),
        },
    )

    return LaunchDescription([
        arm_arg,
        camera_index_arg,
        show_debug_image_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        deadzone_arg,
        motion_full_scale_arg,
        filter_alpha_arg,
        no_hand_pause_timeout_arg,
        ramp_rate_arg,
        servo_status_log_period_arg,
        start_gripper_bridge_arg,
        servo_auto_start_arg,

        LogInfo(msg="Starting MoveIt demo stack: RViz, move_group, ros2_control, controllers."),
        demo_launch,

        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg="Starting MoveIt Servo nodes."),
                servo_launch,
            ],
        ),

        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg="Starting hand teleop and gripper bridge."),
                hand_teleop_launch,
            ],
        ),
    ])
