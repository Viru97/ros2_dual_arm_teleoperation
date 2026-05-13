import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = {
    "left": [
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_elbow_joint",
        "left_wrist_1_joint",
        "left_wrist_2_joint",
        "left_wrist_3_joint",
    ],
    "right": [
        "right_shoulder_pan_joint",
        "right_shoulder_lift_joint",
        "right_elbow_joint",
        "right_wrist_1_joint",
        "right_wrist_2_joint",
        "right_wrist_3_joint",
    ],
}

TARGETS = {
    "left": [0.20, -1.35, 1.65, -1.85, -1.57, 0.20],
    "right": [-0.20, -1.35, 1.65, -1.85, 1.57, -0.20],
}


class ControllerJointTest(Node):
    """Publish one JointTrajectory directly to ros2_control."""

    def __init__(self):
        super().__init__("controller_joint_test")
        self.declare_parameter("arm", "left")
        self.arm = self.get_parameter("arm").value
        if self.arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")

        topic = f"/{self.arm}_arm_controller/joint_trajectory"
        self.publisher = self.create_publisher(JointTrajectory, topic, 10)
        self.start_time = time.monotonic()
        self.done = False
        self.timer = self.create_timer(0.5, self.publish_once)
        self.get_logger().info(f"Will publish one controller trajectory to {topic}.")

    def publish_once(self):
        if time.monotonic() - self.start_time < 1.0:
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINTS[self.arm]

        point = JointTrajectoryPoint()
        point.positions = TARGETS[self.arm]
        point.time_from_start.sec = 3
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"Published direct controller target for {self.arm}_arm.")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = ControllerJointTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
