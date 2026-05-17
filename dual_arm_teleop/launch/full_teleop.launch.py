import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        default_value="0.45",
        description="Maximum Cartesian Servo speed in m/s at full hand deflection.",
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        "max_angular_speed",
        default_value="1.2",
        description="Maximum yaw Servo speed in rad/s at full hand roll.",
    )
    deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.04",
        description="Normalised deadzone around image centre.",
    )
    motion_full_scale_arg = DeclareLaunchArgument(
        "motion_full_scale",
        default_value="0.24",
        description="Normalised palm displacement that maps to max speed.",
    )
    filter_alpha_arg = DeclareLaunchArgument(
        "filter_alpha",
        default_value="0.50",
        description="EMA smoothing factor for palm pose and roll.",
    )
    no_hand_pause_timeout_arg = DeclareLaunchArgument(
        "no_hand_pause_timeout",
        default_value="0.4",
        description="Seconds without a detected hand before pausing Servo.",
    )
    ramp_rate_arg = DeclareLaunchArgument(
        "ramp_rate",
        default_value="0.10",
        description="Max fractional velocity change per control tick.",
    )
    servo_status_log_period_arg = DeclareLaunchArgument(
        "servo_status_log_period",
        default_value="2.0",
        description="Seconds between repeated Servo warning logs.",
    )
    operator_lock_enabled_arg = DeclareLaunchArgument(
        "operator_lock_enabled",
        default_value="true",
        description="Reject likely second-person hands or sudden tracking identity swaps.",
    )
    operator_acquire_radius_arg = DeclareLaunchArgument(
        "operator_acquire_radius",
        default_value="0.45",
        description="Normalized radius around image center where a new operator hand may acquire control.",
    )
    operator_max_jump_arg = DeclareLaunchArgument(
        "operator_max_jump",
        default_value="0.30",
        description="Maximum normalized palm jump between frames before a hand is rejected.",
    )
    safety_latch_enabled_arg = DeclareLaunchArgument(
        "safety_latch_enabled",
        default_value="true",
        description="Pause and lock out hand commands after hard Servo stops.",
    )
    split_control_window_arg = DeclareLaunchArgument(
        "split_control_window",
        default_value="true",
        description="Divide the camera window into left/right control panes, one per arm.",
    )
    invert_lateral_axis_arg = DeclareLaunchArgument(
        "invert_lateral_axis",
        default_value="true",
        description="Use front-view intuitive lateral motion: hand right moves robot right.",
    )
    swap_control_panes_arg = DeclareLaunchArgument(
        "swap_control_panes",
        default_value="true",
        description="Swap pane ownership: left robot uses right pane, right robot uses left pane.",
    )
    visualization_publish_frequency_arg = DeclareLaunchArgument(
        "visualization_publish_frequency",
        default_value="100.0",
        description="MoveIt/RViz visualization publish frequency in Hz.",
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
    start_handover_baton_arg = DeclareLaunchArgument(
        "start_handover_baton",
        default_value="false",
        description="Optionally start the RViz marker publisher for the handover demo object.",
    )
    baton_frame_arg = DeclareLaunchArgument(
        "baton_frame",
        default_value="world",
        description="Frame used for the handover baton marker pose.",
    )
    baton_x_arg = DeclareLaunchArgument(
        "baton_x",
        default_value="0.5",
        description="Handover baton marker x position.",
    )
    baton_y_arg = DeclareLaunchArgument(
        "baton_y",
        default_value="0.0",
        description="Handover baton marker y position.",
    )
    baton_z_arg = DeclareLaunchArgument(
        "baton_z",
        default_value="0.7",
        description="Handover baton marker z position.",
    )

    demo_launch = include_launch(
        "dual_arm_moveit_config",
        "launch/demo.launch.py",
        {"publish_frequency": LaunchConfiguration("visualization_publish_frequency")},
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
            "operator_lock_enabled": LaunchConfiguration("operator_lock_enabled"),
            "operator_acquire_radius": LaunchConfiguration("operator_acquire_radius"),
            "operator_max_jump": LaunchConfiguration("operator_max_jump"),
            "safety_latch_enabled": LaunchConfiguration("safety_latch_enabled"),
            "split_control_window": LaunchConfiguration("split_control_window"),
            "invert_lateral_axis": LaunchConfiguration("invert_lateral_axis"),
            "swap_control_panes": LaunchConfiguration("swap_control_panes"),
            "start_gripper_bridge": LaunchConfiguration("start_gripper_bridge"),
        },
    )

    handover_baton_node = Node(
        package="dual_arm_teleop",
        executable="handover_baton_node",
        name="handover_baton_node",
        output="screen",
        parameters=[
            {
                "frame_id": LaunchConfiguration("baton_frame"),
                "x": ParameterValue(LaunchConfiguration("baton_x"), value_type=float),
                "y": ParameterValue(LaunchConfiguration("baton_y"), value_type=float),
                "z": ParameterValue(LaunchConfiguration("baton_z"), value_type=float),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_handover_baton")),
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
        operator_lock_enabled_arg,
        operator_acquire_radius_arg,
        operator_max_jump_arg,
        safety_latch_enabled_arg,
        split_control_window_arg,
        invert_lateral_axis_arg,
        swap_control_panes_arg,
        visualization_publish_frequency_arg,
        start_gripper_bridge_arg,
        servo_auto_start_arg,
        start_handover_baton_arg,
        baton_frame_arg,
        baton_x_arg,
        baton_y_arg,
        baton_z_arg,

        LogInfo(msg="Starting MoveIt demo stack: RViz, move_group, ros2_control, controllers."),
        demo_launch,
        handover_baton_node,

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
