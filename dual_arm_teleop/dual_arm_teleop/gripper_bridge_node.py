"""
gripper_bridge_node.py
======================
Bridges the Float64 gripper targets published by teleop_node to the
JointTrajectory topic consumed by each gripper's JointTrajectoryController.

Teleop node output:
  /teleop/left_gripper_target   std_msgs/Float64  [0.0 open … 1.0 closed]
  /teleop/right_gripper_target  std_msgs/Float64  [0.0 open … 1.0 closed]

Controller input:
  /left_gripper_controller/joint_trajectory   trajectory_msgs/JointTrajectory
  /right_gripper_controller/joint_trajectory  trajectory_msgs/JointTrajectory

Mapping: value × FINGER_JOINT_MAX_RAD → finger_joint position command.

Robotiq 2F-85 finger_joint limits:
  0.0 rad = fully open
  0.8 rad = fully closed
VERIFY the joint name with:
  grep -r "joint name" ~/teleop_challenge_ws/src/ros2_robotiq_gripper --include="*.xacro" | grep finger
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Robotiq 2F-85: 0 (open) → 0.8 rad (closed).
FINGER_JOINT_MAX_RAD = 0.8

# How quickly the gripper should move to the commanded position.
# 0.25 s gives snappy open/close without jarring the fake hardware.
MOTION_DURATION_SEC = 0.25

GRIPPER_CONFIG = {
    "left": {
        "joint": "left_finger_joint",
        "sub_topic": "/teleop/left_gripper_target",
        "pub_topic": "/left_gripper_controller/joint_trajectory",
    },
    "right": {
        "joint": "right_finger_joint",
        "sub_topic": "/teleop/right_gripper_target",
        "pub_topic": "/right_gripper_controller/joint_trajectory",
    },
}


class GripperBridgeNode(Node):
    """Converts normalised [0-1] Float64 gripper targets to JointTrajectory."""

    def __init__(self):
        super().__init__("gripper_bridge_node")

        self._publishers = {}
        self._subscriptions = []

        for arm, cfg in GRIPPER_CONFIG.items():
            pub = self.create_publisher(JointTrajectory, cfg["pub_topic"], 10)
            self._publishers[arm] = (pub, cfg["joint"])

            # Capture arm name in closure
            sub = self.create_subscription(
                Float64,
                cfg["sub_topic"],
                lambda msg, a=arm: self._on_gripper_target(a, msg),
                10,
            )
            self._subscriptions.append(sub)

        self.get_logger().info(
            "GripperBridgeNode ready. "
            "Listening on /teleop/{left,right}_gripper_target "
            "→ publishing to /{left,right}_gripper_controller/joint_trajectory"
        )

    def _on_gripper_target(self, arm: str, msg: Float64) -> None:
        """Convert normalised 0-1 Float64 to a JointTrajectory command."""
        # Clamp to [0, 1] in case teleop sends slightly out-of-range values
        normalised = max(0.0, min(1.0, msg.data))
        position = normalised * FINGER_JOINT_MAX_RAD

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()

        pub, joint_name = self._publishers[arm]
        traj.joint_names = [joint_name]

        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.velocities = [0.0]
        pt.time_from_start.nanosec = int(MOTION_DURATION_SEC * 1e9)
        traj.points.append(pt)

        pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
