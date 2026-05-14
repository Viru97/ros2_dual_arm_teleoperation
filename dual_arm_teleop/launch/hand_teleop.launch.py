from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_arg = DeclareLaunchArgument(
        "arm",
        default_value="left",
        description="Arm to control: left, right, or both.",
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
        default_value="0.15",
        description="Maximum Cartesian Servo speed in m/s at full hand deflection.",
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        "max_angular_speed",
        default_value="0.6",
        description="Maximum yaw Servo speed in rad/s at full hand roll.",
    )
    deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.06",
        description="Normalized camera deadzone around center.",
    )
    # motion_full_scale_arg = DeclareLaunchArgument(
    #     "motion_full_scale",
    #     default_value="0.25",
    #     description="Hand displacement from neutral that maps to full speed.",
    # )
    filter_alpha_arg = DeclareLaunchArgument(
        "filter_alpha",
        default_value="0.3",
        description="EMA smoothing factor. Higher is faster, lower is smoother.",
    )
    no_hand_pause_timeout_arg = DeclareLaunchArgument(
        "no_hand_pause_timeout",
        default_value="0.4",
        description="Seconds without a detected hand before pausing Servo.",
    )

    teleop_node = Node(
        package="dual_arm_teleop",
        executable="teleop_node",
        name="hand_teleop_node",
        output="screen",
        parameters=[
            {
                "arm": LaunchConfiguration("arm"),
                "camera_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int),
                "show_debug_image": ParameterValue(LaunchConfiguration("show_debug_image"), value_type=bool),
                "max_linear_speed": ParameterValue(LaunchConfiguration("max_linear_speed"), value_type=float),
                "max_angular_speed": ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float),
                "deadzone": ParameterValue(LaunchConfiguration("deadzone"), value_type=float),
                # "motion_full_scale": ParameterValue(LaunchConfiguration("motion_full_scale"), value_type=float),
                "filter_alpha": ParameterValue(LaunchConfiguration("filter_alpha"), value_type=float),
                "no_hand_pause_timeout": ParameterValue(
                    LaunchConfiguration("no_hand_pause_timeout"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription([
        arm_arg,
        camera_index_arg,
        show_debug_image_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        deadzone_arg,
        # motion_full_scale_arg,
        filter_alpha_arg,
        no_hand_pause_timeout_arg,
        teleop_node,
    ])
