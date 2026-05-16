from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_arg = DeclareLaunchArgument(
        "arm",
        default_value="both",
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
        default_value="0.35",
        description="Maximum Cartesian Servo speed in m/s at full hand deflection.",
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        "max_angular_speed",
        default_value="1.0",
        description="Maximum yaw Servo speed in rad/s at full hand roll.",
    )
    deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.04",
        description=(
            "Normalised deadzone fraction around image centre. "
            "Hand positions within this radius produce zero velocity. "
            "Output rescales smoothly from zero at the boundary (no velocity jump)."
        ),
    )
    motion_full_scale_arg = DeclareLaunchArgument(
        "motion_full_scale",
        default_value="0.30",
        description=(
            "Normalised palm displacement from centre that maps to max speed. "
            "0.30 means moving the palm 30%% of frame width from centre = full speed. "
            "Decrease for a more sensitive (twitchy) feel; increase for calmer control."
        ),
    )
    filter_alpha_arg = DeclareLaunchArgument(
        "filter_alpha",
        default_value="0.45",
        description=(
            "EMA smoothing factor applied to palm position and roll. "
            "Range 0–1: higher = more responsive but jitterier; "
            "lower = smoother but laggier. 0.45 is a good default."
        ),
    )
    no_hand_pause_timeout_arg = DeclareLaunchArgument(
        "no_hand_pause_timeout",
        default_value="0.4",
        description="Seconds without a detected hand before pausing Servo.",
    )
    ramp_rate_arg = DeclareLaunchArgument(
        "ramp_rate",
        default_value="0.12",
        description=(
            "Max fractional velocity change per control tick (0–1). "
            "Limits acceleration to avoid jerky starts/stops. "
            "0.12 → ramp from 0 to max_speed in ~8 ticks (~80 ms at 100 Hz)."
        ),
    )
    servo_status_log_period_arg = DeclareLaunchArgument(
        "servo_status_log_period",
        default_value="2.0",
        description="Seconds between repeated Servo warning logs with joint angles.",
    )
    start_gripper_bridge_arg = DeclareLaunchArgument(
        "start_gripper_bridge",
        default_value="true",
        description="Start the bridge from /teleop/*_gripper_target to gripper controllers.",
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
                "motion_full_scale": ParameterValue(LaunchConfiguration("motion_full_scale"), value_type=float),
                "filter_alpha": ParameterValue(LaunchConfiguration("filter_alpha"), value_type=float),
                "no_hand_pause_timeout": ParameterValue(
                    LaunchConfiguration("no_hand_pause_timeout"), value_type=float
                ),
                "ramp_rate": ParameterValue(LaunchConfiguration("ramp_rate"), value_type=float),
                "servo_status_log_period": ParameterValue(
                    LaunchConfiguration("servo_status_log_period"), value_type=float
                ),
            }
        ],
    )
    gripper_bridge_node = Node(
        package="dual_arm_teleop",
        executable="gripper_bridge_node",
        name="gripper_bridge_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_gripper_bridge")),
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
        teleop_node,
        gripper_bridge_node,
    ])
