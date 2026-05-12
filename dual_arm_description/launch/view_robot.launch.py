import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('dual_arm_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'dual_arm.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')

    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both', parameters=[params]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])