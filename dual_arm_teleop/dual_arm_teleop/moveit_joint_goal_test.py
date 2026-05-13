import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


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
    "left": [0.30, -1.35, 1.65, -1.85, -1.57, 0.25],
    "right": [-0.30, -1.35, 1.65, -1.85, 1.57, -0.25],
}


class MoveItJointGoalTest(Node):
    """Send one simple MoveIt 2 joint-space plan-and-execute request."""

    def __init__(self):
        super().__init__("moveit_joint_goal_test")
        self.declare_parameter("arm", "left")
        self.declare_parameter("plan_only", False)

        self.arm = self.get_parameter("arm").value
        self.plan_only = bool(self.get_parameter("plan_only").value)
        if self.arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")

        self.action_client = ActionClient(self, MoveGroup, "/move_action")
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.current_positions = {}
        self.start_positions = {}
        self.result_received_time = None
        self.movement_checked = False
        self.sent = False
        self.done = False
        self.timer = self.create_timer(0.5, self.try_send_goal)
        self.result_check_timer = self.create_timer(0.2, self.check_movement_after_result)
        self.get_logger().info(f"Waiting for MoveIt /move_action to command {self.arm}_arm.")

    def joint_state_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def try_send_goal(self):
        if self.sent or not self.action_client.server_is_ready():
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = f"{self.arm}_arm"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        constraints = Constraints()
        constraints.name = f"{self.arm}_joint_goal"
        for joint_name, position in zip(JOINTS[self.arm], TARGETS[self.arm]):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = self.plan_only
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 2
        goal_msg.planning_options.planning_scene_diff.is_diff = True

        self.sent = True
        self.destroy_timer(self.timer)
        self.start_positions = {
            joint_name: self.current_positions.get(joint_name)
            for joint_name in JOINTS[self.arm]
        }
        self.get_logger().info(f"Sending MoveIt goal to {self.arm}_arm: {TARGETS[self.arm]}")
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_goal_response)

    def handle_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the goal.")
            self.done = True
            return

        self.get_logger().info("MoveIt accepted the goal; waiting for result.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_result)

    def handle_result(self, future):
        result = future.result().result
        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("MoveIt plan-and-execute succeeded.")
        else:
            self.get_logger().error(f"MoveIt failed with error code {code}.")
        self.result_received_time = self.get_clock().now()

    def check_movement_after_result(self):
        if self.result_received_time is None or self.movement_checked:
            return

        elapsed = (self.get_clock().now() - self.result_received_time).nanoseconds / 1e9
        if elapsed < 0.5:
            return

        max_delta = 0.0
        changed_joints = []
        for joint_name in JOINTS[self.arm]:
            start_position = self.start_positions.get(joint_name)
            current_position = self.current_positions.get(joint_name)
            if start_position is None or current_position is None:
                continue
            delta = abs(current_position - start_position)
            max_delta = max(max_delta, delta)
            changed_joints.append(f"{joint_name}={delta:.3f}")

        if max_delta > 0.02:
            self.get_logger().info(
                f"/joint_states changed after execution. Max delta: {max_delta:.3f} rad. "
                + ", ".join(changed_joints)
            )
        else:
            self.get_logger().warn(
                f"MoveIt reported success, but /joint_states barely changed. Max delta: {max_delta:.3f} rad."
            )

        self.movement_checked = True
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = MoveItJointGoalTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
