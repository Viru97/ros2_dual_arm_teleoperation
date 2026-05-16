from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'mediapipe', 'numpy'],
    zip_safe=True,
    maintainer='adm-tin-ap',
    maintainer_email='engr.apurv@gmail.com',
    description='Dual-arm hand teleoperation package',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'teleop_node          = dual_arm_teleop.teleop_node:main',
            'servo_smoke_test     = dual_arm_teleop.servo_smoke_test:main',
            'moveit_joint_goal_test = dual_arm_teleop.moveit_joint_goal_test:main',
            'controller_joint_test  = dual_arm_teleop.controller_joint_test:main',
            'ready_pose_node      = dual_arm_teleop.ready_pose_node:main',
            'gripper_bridge_node  = dual_arm_teleop.gripper_bridge_node:main',
        ],
    },
)