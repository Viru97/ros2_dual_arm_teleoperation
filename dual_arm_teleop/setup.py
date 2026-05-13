from setuptools import find_packages, setup

package_name = 'dual_arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'mediapipe',
        'numpy',
    ],
    zip_safe=True,
    maintainer='adm-tin-ap',
    maintainer_email='engr.apurv@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = dual_arm_teleop.teleop_node:main',
            'servo_smoke_test = dual_arm_teleop.servo_smoke_test:main',
            'moveit_joint_goal_test = dual_arm_teleop.moveit_joint_goal_test:main',
            'controller_joint_test = dual_arm_teleop.controller_joint_test:main',
        ],
    },
)
